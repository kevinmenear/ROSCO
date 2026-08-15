#!/bin/bash
# Pre-integration harness RED TEST for unit #40 -- the unit as a no-op.
#
# THE TRAP IS THE POINT. `run_kernel_stubs.sh`, one dispatch earlier, edited the
# shipped translation in place and restored it only on completion; it was killed
# inside stub 3 and left `R_Total = 1.0;` live in the tree. Every stub runner in
# this unit restores from `git checkout` on an EXIT trap, so an ordinary
# interrupt cannot leave a mutant behind.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
CPP=translations/ControllerBlocks/setpointsmoother.cpp
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT

cp evidence/SetpointSmoother/setpointsmoother.noop-stub.cpp "$CPP"
bash scripts/harness.sh SetpointSmoother ControllerBlocks setpointsmoother \
     rosco/controller/src/ControllerBlocks.f90 --against translation \
     --out harness/SetpointSmoother.redtest.json \
     --red-test "the unit as a no-op (one constant-argument lpfilter_c call kept, result discarded, so the callee bridge still links)"
