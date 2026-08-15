#!/bin/bash
# Perturb the INTEGRATED WRAPPER, rebuild, re-run the post-integration harness,
# then revert and rebuild. On the INTEGRATED tree, from the repository root.
#
#   bash evidence/ResController/wrapper_redtest.sh <sed-expr> <out.json> "<what>"
#
# THE REBUILD BETWEEN EDIT AND RUN IS THE WHOLE POINT. `harness.sh
# --post-integration` links the objects in the build tree; without a rebuild it
# would measure the wrapper as it stood BEFORE the edit and report a green that
# reads exactly like a red test that could not fail.
#
# The sed is anchored on the ResController wrapper's own line number range so it
# cannot reach the PIController, PIDController or PIIController wrappers above
# it -- unit #13's finding, and Controllers.f90 is the file that shows why it
# matters: FOUR sibling wrappers in one file carry the same tokens (`minValue`,
# `maxValue`, `MERGE(1_C_INT, 0_C_INT, reset)`), and an unanchored sed would
# perturb all four while the artifact named one. This unit is a FUNCTION rather
# than a SUBROUTINE, so the anchors are FUNCTION/END FUNCTION.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
EXPR="$1"; OUT="$2"; WHAT="$3"
F=rosco/controller/src/Controllers.f90
LO=$(grep -n 'FUNCTION ResController' "$F" | head -1 | cut -d: -f1)
HI=$(grep -n 'END FUNCTION ResController' "$F" | head -1 | cut -d: -f1)

cp "$F" /tmp/Controllers.f90.redtest-backup
sed -i.bak "${LO},${HI}${EXPR}" "$F"; rm -f "$F.bak"
if diff -q "$F" /tmp/Controllers.f90.redtest-backup > /dev/null; then
    echo "wrapper_redtest: the sed matched NOTHING -- refusing to report on an unperturbed wrapper" >&2
    exit 2
fi
docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"

bash scripts/harness.sh ResController Controllers rescontroller "$F" \
     --post-integration --out "$OUT" --red-test "$WHAT" > /tmp/wrapper_redtest.log 2>&1 || true
tail -3 /tmp/wrapper_redtest.log

cp /tmp/Controllers.f90.redtest-backup "$F"
docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
git diff --quiet -- "$F" && echo "wrapper_redtest: $F reverted and rebuilt" \
    || { echo "wrapper_redtest: $F IS STILL MODIFIED" >&2; exit 3; }
