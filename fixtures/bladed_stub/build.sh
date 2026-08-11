#!/bin/bash
# Build the Bladed-style stub controller that is ExtController's ORACLE fixture.
#
#   docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && bash fixtures/bladed_stub/build.sh"
#
# The output is DELIBERATELY NOT COMMITTED (.gitignore) -- it is a build product,
# and a committed .so would be an unhashable binary in a tree whose whole point is
# that every artifact says where it came from. `discon_stub.c` is the artifact;
# this script is how it becomes loadable. Anything that needs the library builds
# it first.
#
# -O2 and no -ffp-contract=off: this library is not under translation and is not
# compared against anything. It is the SAME object on both sides of every
# differential comparison, so its own code generation cannot make the two sides
# disagree. E1.2's flag requirement is about the code under test.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$HERE/libdiscon_stub.so"

gcc -shared -fPIC -O2 -o "$OUT" "$HERE/discon_stub.c"

# ASSERT the symbol the loader will ask for actually exists, by the name
# LoadDynamicLibProc passes to dlsym -- `DLL_ProcName` is "DISCON" in all 14
# Examples/*.IN. A library that builds and does not export DISCON fails exactly
# where a missing library does (null ProcAddr, SIGSEGV two statements later),
# and the failure would read as a defect in the unit rather than in the fixture.
if ! nm -D --defined-only "$OUT" | grep -qw DISCON; then
    echo "build.sh: $OUT does not export DISCON" >&2
    exit 1
fi

echo "built $OUT"
nm -D --defined-only "$OUT" | grep -w DISCON
