#!/bin/bash
# ONE kernel stub per invocation. `run_kernel_stubs2.sh` tried four in one
# foreground command and was killed at the 600-second tool ceiling inside its
# third -- the trap restored the tree, but a `vit verify` ORPHANED in the
# container and had to be pkill'd. One run per command is what fits.
#
#   bash evidence/SetpointSmoother/run_one_kernel_stub.sh <stub.cpp> "<name>" "<asks>" "<expect>"
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/ControllerBlocks/setpointsmoother.cpp
E=evidence/SetpointSmoother; OUT=$E/kernel.stubs2.txt
trap 'git checkout -- "$CPP" vit.yaml; docker exec vit-dev bash -lc "pkill -9 -f \"vit verify SetpointSmoother\"" >/dev/null 2>&1; echo "restored $CPP and vit.yaml"' EXIT
{ echo "=================================================================="
  echo "STUB      $2"; echo "ASKS      $3"; echo "EXPECT    $4"; echo "RESULT:"; } >> "$OUT"
cp "$E/$1" "$CPP"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && vit verify SetpointSmoother $CPP \
    -f rosco/controller/src/ControllerBlocks.f90 --kernel-dir kernel/SetpointSmoother" 2>&1 \
  | grep -E "VERIFICATION|NON_DISCRIMINATING|passed" | head -4 | sed 's/^/    /' >> "$OUT"
echo >> "$OUT"; tail -6 "$OUT"
