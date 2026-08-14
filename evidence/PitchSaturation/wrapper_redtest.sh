#!/bin/bash
# Perturb the INTEGRATED WRAPPER, rebuild, re-run the post-integration harness,
# then revert and rebuild. On the INTEGRATED tree, from the repository root.
#
#   bash evidence/PitchSaturation/wrapper_redtest.sh <sed-expr> <out.json> "<what>"
#
# THE REBUILD BETWEEN EDIT AND RUN IS THE WHOLE POINT. `harness.sh
# --post-integration` links the objects in the build tree; without a rebuild it
# would measure the wrapper as it stood BEFORE the edit and report a green that
# reads exactly like a red test that could not fail.
#
# The sed is anchored on the PitchSaturation wrapper's own line number range so it
# cannot reach the PIController or PIDController wrappers above it -- unit #13's
# finding, and Controllers.f90 is now the case that shows why it matters: three
# sibling wrappers in one file carry the same tokens, and `grep -n 'FUNCTION
# PitchSaturation'` is itself ambiguous against `FUNCTION PIController` only
# because of the second I. The `head -1` below takes the WRAPPER, which
# `vit integrate` writes in place of the original body.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
EXPR="$1"; OUT="$2"; WHAT="$3"
F=rosco/controller/src/ControllerBlocks.f90
LO=$(grep -n 'FUNCTION PitchSaturation' "$F" | head -1 | cut -d: -f1)
HI=$(grep -n 'END FUNCTION PitchSaturation' "$F" | head -1 | cut -d: -f1)

cp "$F" /tmp/ControllerBlocks.f90.redtest-backup
sed -i.bak "${LO},${HI}${EXPR}" "$F"; rm -f "$F.bak"
if diff -q "$F" /tmp/ControllerBlocks.f90.redtest-backup > /dev/null; then
    echo "wrapper_redtest: the sed matched NOTHING -- refusing to report on an unperturbed wrapper" >&2
    exit 2
fi
docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"

bash scripts/harness.sh PitchSaturation ControllerBlocks pitchsaturation "$F" \
     --post-integration --out "$OUT" --red-test "$WHAT" > /tmp/wrapper_redtest.log 2>&1 || true
tail -3 /tmp/wrapper_redtest.log

cp /tmp/ControllerBlocks.f90.redtest-backup "$F"
docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
git diff --quiet -- "$F" && echo "wrapper_redtest: $F reverted and rebuilt" \
    || { echo "wrapper_redtest: $F IS STILL MODIFIED" >&2; exit 3; }
