#!/bin/bash
# E1.2 -- assert the translation-phase flag reached the compiler, and worked.
#
#   bash scripts/assert_fp_contract.sh      # exit 0 iff both assertions hold
#
# Two assertions, because either alone can pass while the requirement fails:
#
#   1. `-ffp-contract=off` is on the Fortran build line CMake actually emits.
#      A flag declared in vit.yaml and absent from CMakeLists is what this
#      campaign had until 2026-08-10: the requirement stated, the build not
#      meeting it, and nothing announcing the gap.
#   2. The linked libdiscon.so contains ZERO fused multiply-add instructions.
#      This is the assertion that cannot be satisfied by declaring anything.
#      Before the fix, `__functions_MOD_colemantransform` alone carried 4
#      `vfmadd`, and 105 of 200 random inputs disagreed with an equivalent C++
#      translation by ~1 ULP. Afterwards: 0 in the whole library, 0 of 200.
#
# Assertion 2 is deliberately whole-library rather than per-file. A per-file
# check passes the moment the file under translation is clean and says nothing
# about the next unit's file.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORKDIR="/workspace/$(basename "$ROOT")"
BUILD="$WORKDIR/rosco/controller/build"

flag=$(docker exec "$CONTAINER" bash -lc \
    "grep -c -- '-ffp-contract=off' $BUILD/CMakeFiles/discon.dir/flags.make || true")
fma=$(docker exec "$CONTAINER" bash -lc \
    "objdump -d $BUILD/libdiscon.so 2>/dev/null | grep -cE 'vfmadd|fmadd' || true")

echo "E1.2: -ffp-contract=off on $flag Fortran build line(s); $fma FMA instruction(s) in libdiscon.so"

rc=0
if [ "$flag" = "0" ]; then
    echo "  FAILED: the flag is not on the build line" >&2; rc=1
fi
if [ "$fma" != "0" ]; then
    echo "  FAILED: the build still contracts multiply-add; it is not bit-comparable with C++" >&2; rc=1
fi
exit $rc
