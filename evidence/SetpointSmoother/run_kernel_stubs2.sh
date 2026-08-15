#!/bin/bash
# The kernel hand red tests `run_kernel_stubs.sh` did not finish.
#
# THAT SCRIPT WAS KILLED INSIDE ITS STUB 3, between the edit that writes
# `R_Total = 1.0;` into the shipped translation and the `cp` that ends the stub,
# and it left the mutant live in the tree. This one restores from `git checkout`
# on an EXIT trap -- the whole difference -- and restores vit.yaml too, because
# `vit verify` rewrites it (stripping all 449 lines of comment) on every run.
#
# Four runs, ~4 minutes each. Run in the FOREGROUND.
#   bash scripts/run_if_time_remains.sh 1800 bash evidence/SetpointSmoother/run_kernel_stubs2.sh
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/ControllerBlocks/setpointsmoother.cpp
E=evidence/SetpointSmoother
OUT=$E/kernel.stubs2.txt
trap 'git checkout -- "$CPP" vit.yaml; echo "restored $CPP and vit.yaml"' EXIT

{
  echo "Kernel hand red tests, part 2 -- SetpointSmoother (unit #40)"
  echo "kernel/SetpointSmoother, 62 cases, scenario 25, invocations 1-20 / 8000-8020 / 15900-15920"
  echo "The shipped translation passes 62/62 with all 14,508 field rows IDENTICAL."
  echo
  echo "TAKEN AFTER INTEGRATION, which is why run 0 is here: the kernel is a"
  echo "standalone fixture replaying CAPTURED state, so integrating the unit"
  echo "cannot make it compare a mutant against itself the way unit #29 found"
  echo "the differential harness does -- but that is an argument, and run 0 is"
  echo "the measurement. It reproduces the pre-integration no-op exactly."
  echo
} > "$OUT"

run () {
    local stub="$1" name="$2" asks="$3" expect="$4"
    {
      echo "=================================================================="
      echo "STUB      $name"
      echo "ASKS      $asks"
      echo "EXPECT    $expect"
      echo "RESULT:"
    } >> "$OUT"
    cp "$E/$stub" "$CPP"
    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && vit verify SetpointSmoother $CPP \
        -f rosco/controller/src/ControllerBlocks.f90 --kernel-dir kernel/SetpointSmoother" 2>&1 \
      | grep -E "VERIFICATION|NON_DISCRIMINATING|passed" | head -4 | sed 's/^/    /' >> "$OUT"
    echo >> "$OUT"
    git checkout -- "$CPP" vit.yaml
    tail -5 "$OUT"
}

run setpointsmoother.noop-stub.cpp        "0. the no-op, RE-TAKEN post-integration (P10 control)" \
    "does this kernel still discriminate now that the unit is integrated?" \
    "FAIL 62/62, reproducing the pre-integration run in kernel.stubs.txt"
run setpointsmoother.kstub-r-total.cpp    "3. R_Total -> 1.0" \
    "was scenario 25 the right choice? this is the stub the killed dispatch never got a verdict for" \
    "FAIL -- and on a scenario with all three PRC_R_* at 1.0 it would have passed"
run setpointsmoother.kstub-vsgain-term.cpp "4. the (BlPitchCMeas - PC_MinPit)/0.524 factor -> 0.0" \
    "BlPitchCMeas and PC_MinPit may be equal in many captured cases -- is the term visible in the rest?" \
    "FAIL on fewer than 62 if the two are ever equal, on 62 if never"
run setpointsmoother.kstub-reassociated.cpp "5. (num/VS_RtPwr)*SS_PCGain -> num*SS_PCGain/VS_RtPwr" \
    "is the kernel sensitive to the ASSOCIATION rather than the algebra? the two agree exactly in real arithmetic" \
    "FAIL if the rounding differs on any of the 62 -- this is the defect shape vit check's guidance names"

echo "--- $OUT ---"; cat "$OUT"
