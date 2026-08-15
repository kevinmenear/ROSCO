#!/bin/bash
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/ControllerBlocks/setpointsmoother.cpp
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT
cp evidence/SetpointSmoother/setpointsmoother.restart-false-probe.cpp "$CPP"
bash scripts/harness.sh SetpointSmoother ControllerBlocks setpointsmoother \
     rosco/controller/src/ControllerBlocks.f90 --against translation \
     --out evidence/SetpointSmoother/harness.restart-false-probe.json \
     --red-test "P10 probe: the reset argument forced to 0, counting the cases in which restart is TRUE and reaches LPFilter"
