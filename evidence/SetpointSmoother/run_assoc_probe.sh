#!/bin/bash
# DOES ANY INSTRUMENT IN THIS CAMPAIGN SEE THE ASSOCIATION? -- unit #40.
#
# Kernel stub 5 rewrites `(num/VS_RtPwr)*SS_PCGain` as `num*SS_PCGain/VS_RtPwr`
# -- equal in real arithmetic, a different rounding in IEEE -- and the kernel
# PASSES it 62/62. The mutation sweep does not answer the question either:
# `assoc_reorder` is in the operator set OFFERED and produced no mutant here.
# That leaves the differential harness, and it has to be asked on a tree where
# the reference is REAL FORTRAN.
#
# WHY A `git checkout` OF ONE PATH AND NOT reset_to_clean.sh. The tree is
# integrated, and in PRE mode after integration the reference side runs
#     Fortran bridge -> the wrapper -> setpointsmoother_c -> the harness's copy
# which is the STUB -- unit #29's finding, both sides running the mutant. Only
# this unit's Fortran body has to go back, and the reset/restore pair reverts
# all forty and blocks commits for the duration. Unit #31's shape.
#
# RUN 1 IS THE CONTROL AND IT IS NOT OPTIONAL: a window that reports a stub
# green is indistinguishable from a window that reports everything green.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/ControllerBlocks/setpointsmoother.cpp
F=rosco/controller/src/ControllerBlocks.f90
PRE="${1:?usage: run_assoc_probe.sh <pre-integration-commit>}"
E=evidence/SetpointSmoother
build () { docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" >/dev/null; }
# `git checkout <commit> -- <path>` WRITES THE INDEX AS WELL AS THE WORKING TREE,
# so the restore must name HEAD explicitly. The first version of this trap said
# `git checkout -- "$F"`, which restores from the INDEX -- and the index was
# holding the PRE-INTEGRATION body this probe had just staged. It reported
# "restored", and the wrapper was gone from the tree: `grep -c setpointsmoother_c
# ControllerBlocks.f90` returned 0, staged, one rebuild away from a libdiscon.so
# with no C++ in it. `git status --short` showed `M ` in the first column, which
# is the only thing that said so.
trap 'git checkout HEAD -- "$CPP" "$F" vit.yaml; build; echo "restored $F and $CPP from HEAD, rebuilt"' EXIT

git checkout "$PRE" -- "$F"
grep -q "R_Total = LocalVar%PRC_R_Speed" "$F" || { echo "the Fortran body is not back"; exit 3; }
build

echo "### CONTROL -- the SHIPPED translation, in this same window"
bash scripts/harness.sh SetpointSmoother ControllerBlocks setpointsmoother "$F" \
     --against translation --out "$E/harness.assoc-window-control.json" 2>&1 | grep -E "^HARNESS"

echo "### PROBE -- the re-association kernel stub 5 passes 62/62 on"
cp "$E/setpointsmoother.kstub-reassociated.cpp" "$CPP"
bash scripts/harness.sh SetpointSmoother ControllerBlocks setpointsmoother "$F" \
     --against translation --out "$E/harness.assoc-reordered.json" \
     --red-test "(num/VS_RtPwr)*SS_PCGain rewritten as num*SS_PCGain/VS_RtPwr -- algebraically equal, differently rounded; the kernel passes it 62/62 and no assoc_reorder mutant exists for this unit" 2>&1 | grep -E "^HARNESS|RED TEST|case "
