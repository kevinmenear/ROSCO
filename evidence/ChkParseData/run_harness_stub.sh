#!/bin/bash
# Run ONE stub through the differential harness and keep its artifact.
#
#   bash evidence/ChkParseData/run_harness_stub.sh <stub.cpp> <out.json> "<what was cut>"
#
# The shipped translation is restored on the way out, and the stub is
# HASH-VERIFIED from inside the container before anything is built (unit #23:
# `cp` onto a bind-mounted file is read half-written often enough to need a
# guard, not a warning). The harness Makefile's first rule copies
# `translations/ROSCO_Helpers/chkparsedata.cpp` to `chkparsedata.hpp`, so
# swapping that file is the whole of the lever.
#
# THE TREE MUST BE CLEAN (`scripts/reset_to_clean.sh`). The pre-integration
# harness links this campaign's Fortran objects, and after integration
# `ROSCO_Helpers.f90.o` IS the wrapper -- there is no independent reference
# left to compare against.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
STUB="$1"; OUT="$2"; WHAT="$3"
LIVE=translations/ROSCO_Helpers/chkparsedata.cpp
KEEP=$(mktemp)
cp "$LIVE" "$KEEP"
restore() { cp "$KEEP" "$LIVE"; rm -f "$KEEP"; }
trap restore EXIT

cp "$STUB" "$LIVE"
want=$(md5 -q "$STUB")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$LIVE | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$STUB" "$LIVE"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH on $LIVE"; exit 1; }
echo "stub md5 (verified inside the container): $got  <- $STUB"

bash scripts/harness.sh ChkParseData ROSCO_Helpers chkparsedata \
     rosco/controller/src/ROSCO_Helpers.f90 \
     --against translation --out "$OUT" --red-test "$WHAT" 2>&1 | tail -4
