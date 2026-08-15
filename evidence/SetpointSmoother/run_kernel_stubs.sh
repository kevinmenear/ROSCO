#!/bin/bash
# Hand red tests for unit #40's kernel replay.
#
# `vit verify` DECLINED to construct its own red test for this unit -- no
# by-value floating-point parameter and no floating-point result -- and printed
# NON_DISCRIMINATING. That verdict is about the instrument, not the
# translation, and the only way to turn it into a number is to perturb the
# translation by hand and watch the kernel.
#
# Five stubs. Each is the SHIPPED translation with exactly one thing changed,
# and each names a specific question the kernel is being asked to answer.
# Run in the foreground; each `vit verify` is about four minutes.
#
#   bash evidence/SetpointSmoother/run_kernel_stubs.sh
#
# Writes evidence/SetpointSmoother/kernel.stubs.txt and restores the shipped
# translation and vit.yaml afterwards.

set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

CPP=translations/ControllerBlocks/setpointsmoother.cpp
OUT=evidence/SetpointSmoother/kernel.stubs.txt
SHIPPED=/tmp/setpointsmoother.shipped.cpp
YAML=/tmp/vit.yaml.shipped

cp "$CPP" "$SHIPPED"
cp vit.yaml "$YAML"

: > "$OUT"
{
  echo "Kernel hand red tests -- SetpointSmoother (unit #40)"
  echo "kernel/SetpointSmoother, 62 cases, scenario 25, invocations 1-20 / 8000-8020 / 15900-15920"
  echo "The shipped translation passes 62/62 with all 14,508 field rows IDENTICAL."
  echo
} >> "$OUT"

run_stub () {
  local name="$1" question="$2" expect="$3"
  {
    echo "=================================================================="
    echo "STUB      $name"
    echo "ASKS      $question"
    echo "EXPECT    $expect"
    echo "DIFF from the shipped translation:"
    diff -u "$SHIPPED" "$CPP" | sed -n '3,$p' | sed 's/^/    /'
    echo "RESULT:"
  } >> "$OUT"
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && vit verify SetpointSmoother $CPP \
      -f rosco/controller/src/ControllerBlocks.f90 --kernel-dir kernel/SetpointSmoother" \
      2>&1 | grep -E "VERIFICATION|passed|PASSED|FAILED|mismatch|✗|✓|Error|error:" | head -20 | sed 's/^/    /' >> "$OUT"
  echo >> "$OUT"
  cp "$SHIPPED" "$CPP"
}

# --- 1. the unit as a no-op ------------------------------------------------
python3 - "$CPP" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p).read()
body = s[s.index('void SetpointSmoother'):]
head = s[:s.index('void SetpointSmoother')]
open(p, 'w').write(head + '''void SetpointSmoother(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst) {
    (void)LocalVar; (void)CntrPar; (void)objInst;
}
''')
PY
run_stub "no-op" \
  "can this kernel see the unit at all?" \
  "FAIL -- SS_DelOmegaF is written on every one of the 62 cases"

# --- 2. the ELSE arm's constant --------------------------------------------
python3 - "$CPP" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace('        LocalVar->SS_DelOmegaF = 0;',
              '        LocalVar->SS_DelOmegaF = 12345.0;')
open(p, 'w').write(s)
PY
run_stub "else-arm 0 -> 12345.0" \
  "does any captured case take the SS_Mode /= 1 arm?" \
  "PASS -- coverage says line :514 has hits in scenario 23 ALONE, and this kernel is scenario 25"

# --- 3. R_Total dropped ----------------------------------------------------
python3 - "$CPP" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace('        R_Total = LocalVar->PRC_R_Speed * LocalVar->PRC_R_Torque * LocalVar->PRC_R_Pitch;',
              '        R_Total = 1.0;')
open(p, 'w').write(s)
PY
run_stub "R_Total -> 1.0" \
  "was scenario 25 the right choice? PRC_R_Speed is 0.9 there and 1.0 in every other scenario" \
  "FAIL -- and on any other scenario this stub would have passed"

# --- 4. the SS_VSGain term dropped -----------------------------------------
python3 - "$CPP" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace('        DelOmega = ((LocalVar->BlPitchCMeas - LocalVar->PC_MinPit) / 0.524) * CntrPar->SS_VSGain',
              '        DelOmega = 0.0')
open(p, 'w').write(s)
PY
run_stub "the (BlPitchCMeas - PC_MinPit)/0.524 * SS_VSGain term -> 0.0" \
  "BlPitchCMeas and PC_MinPit are EQUAL in 23 of the 62 captured cases -- is the term visible in the other 39?" \
  "FAIL, and the count should be about 39 rather than 62"

# --- 5. the divide and the multiply transposed -----------------------------
python3 - "$CPP" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace('''                   - ((CntrPar->VS_RtPwr * R_Total - LocalVar->VS_LastGenPwr) / CntrPar->VS_RtPwr)
                         * CntrPar->SS_PCGain;''',
              '''                   - (CntrPar->VS_RtPwr * R_Total - LocalVar->VS_LastGenPwr)
                         * CntrPar->SS_PCGain / CntrPar->VS_RtPwr;''')
open(p, 'w').write(s)
PY
run_stub "(num/VS_RtPwr)*SS_PCGain -> num*SS_PCGain/VS_RtPwr" \
  "is the kernel sensitive to the ASSOCIATION, not just the algebra? the two agree exactly in real arithmetic" \
  "FAIL if the rounding differs on any of the 62 -- and this is the defect shape `vit check`'s guidance names"

cp "$SHIPPED" "$CPP"
cp "$YAML" vit.yaml
echo "restored the shipped translation and vit.yaml"
echo "--- $OUT ---"
cat "$OUT"
