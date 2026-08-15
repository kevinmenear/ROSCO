#!/bin/bash
# POST-INTEGRATION RED TEST for unit #40 -- unit #23's defect planted deliberately.
#
# After integration the Fortran body IS the translation, so this harness measures
# the WRAPPER and nothing else. The wrapper's load-bearing statement is the one
# `--reverse-copy` generates: `LocalVar%SS_DelOmegaF` is a SCALAR of a view-type
# INOUT argument, so the C++ writes it into `LocalVar_view` and only the copy-back
# puts it in `LocalVar`. Delete that line and every case must fail.
#
# REBUILD BETWEEN THE EDIT AND THE RUN. `harness.sh --post-integration` links the
# CMake build tree's ControllerBlocks.f90.o; an unrebuilt edit measures the old
# wrapper and reports green.
#
# Restores the source AND rebuilds on an EXIT trap, so an interrupt cannot leave
# a crippled wrapper compiled into libdiscon.so.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/ControllerBlocks.f90
build () {
    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" >/dev/null
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
}
trap 'git checkout -- "$F"; build; echo "restored and rebuilt $F"' EXIT

python3 - "$F" <<'PY'
import sys
p = sys.argv[1]; s = open(p).read()
old = """        CALL setpointsmoother_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
"""
new = """        CALL setpointsmoother_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(objInst))
        ! RED TEST: the --reverse-copy line deleted.
"""
assert s.count(old) == 1, f"wrapper text matched {s.count(old)} times"
open(p, "w").write(s.replace(old, new))
print("wrapper crippled")
PY
build
bash scripts/harness.sh SetpointSmoother ControllerBlocks setpointsmoother \
     rosco/controller/src/ControllerBlocks.f90 --post-integration \
     --out harness/SetpointSmoother.postintegration.redtest.json \
     --red-test "wrapper: the --reverse-copy line CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar) deleted, rebuilt between the edit and the run"
