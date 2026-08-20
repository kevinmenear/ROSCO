#!/bin/bash
# ONE kernel hand red test per invocation, for unit #60 `VariableSpeedControl`.
#
# Copied from `evidence/YawRateControl/run_one_kernel_stub.sh` (P4) with this
# unit's three names changed and nothing else. The EXIT trap, the one-stub-per-
# command rule and the pre-integration `-f` are all that file's, and each was
# paid for by a failure it records.
#
# `vit verify` DECLINED to construct its own red test here and printed
# NON_DISCRIMINATING -- the same refusal it gave units #40 through #44, for the
# same stated reason: every argument arrives behind a pointer or inside a
# derived type, so there is no by-value floating-point input it can perturb
# without changing what BOTH implementations compute. The kernel's
# discriminating power therefore has to be measured by hand or not at all.
#
# ONE STUB PER COMMAND, deliberately. Unit #40 tried four in one foreground
# command and was killed at the 600-second tool ceiling inside its third,
# orphaning a `vit verify` in the container. Each `vit verify` on this kernel
# takes ~230 s at 60 cases.
#
#   bash scripts/run_if_time_remains.sh 500 \
#       bash evidence/VariableSpeedControl/run_one_kernel_stub.sh <stub.cpp> "<name>" "<asks>" "<expect>"
#
# `-f` IS THE PRE-INTEGRATION FILE, NOT THE LIVE ONE. `vit integrate` has since
# replaced this unit's Fortran body with a wrapper that calls
# `variablespeedcontrol_c`, and the green these stubs are red tests FOR was
# taken against the body. The kernel directory carries its own generated Fortran
# and `-f` supplies only the signature, so a gitignored copy taken from the
# campaign's pinned clean baseline is both sufficient and exactly what the green
# used (RUNBOOK, unit #11):
#
#   git show 54dd134:rosco/controller/src/Controllers.f90 \
#       > .vit/Controllers.clean54dd134.f90
#
# The EXIT trap restores the translation AND vit.yaml from git: `vit verify`
# edits the .cpp path it is handed and rewrites vit.yaml on every run. A killed
# dispatch that does not restore leaves a MUTANT live in the shipped
# translation -- unit #40's third stub did exactly that.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Controllers/variablespeedcontrol.cpp
E=evidence/VariableSpeedControl; OUT=$E/kernel.stubs.txt
REF=.vit/Controllers.clean54dd134.f90
[ -s "$REF" ] || { echo "run_one_kernel_stub: $REF is missing; see this script's header" >&2; exit 2; }
trap 'git checkout -- "$CPP" vit.yaml; docker exec vit-dev bash -lc "pkill -9 -f \"vit verify VariableSpeedControl\"" >/dev/null 2>&1; echo "restored $CPP and vit.yaml"' EXIT
{ echo "=================================================================="
  echo "STUB      $2"; echo "ASKS      $3"; echo "EXPECT    $4"; echo "RESULT:"; } >> "$OUT"
cp "$E/$1" "$CPP"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && vit verify VariableSpeedControl $CPP \
    -f $REF --kernel-dir kernel/VariableSpeedControl" 2>&1 \
  | grep -E "VERIFICATION|NON_DISCRIMINATING|passed|FAILED|mismatch|error" | head -8 | sed 's/^/    /' >> "$OUT"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/kernel/VariableSpeedControl && ./kernel.exe 2>&1 \
    | grep -E 'Total number of verification cases|verification-passed cases'" \
  | sed 's/^ */    kernel.exe: /' >> "$OUT"
echo >> "$OUT"; tail -14 "$OUT"
