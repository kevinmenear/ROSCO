#!/bin/bash
# Perturb the INTEGRATED WRAPPER, rebuild, re-run the post-integration harness,
# then revert and rebuild. On the INTEGRATED tree, from the repository root.
#
#   bash evidence/PIDController/wrapper_redtest.sh <sed-expr> <out.json> "<what>"
#
# THE REBUILD BETWEEN EDIT AND RUN IS THE WHOLE POINT. `harness.sh
# --post-integration` links the objects in the build tree; without a rebuild it
# would measure the wrapper as it stood BEFORE the edit and report a green that
# reads exactly like a red test that could not fail.
#
# The sed is anchored on the PIDController wrapper's own line number range so it
# cannot reach PIController's identically-spelled MERGE eleven lines above --
# unit #13's finding, and this file is the case that shows why it matters: the
# two wrappers carry the same tokens.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
EXPR="$1"; OUT="$2"; WHAT="$3"
F=rosco/controller/src/Controllers.f90
LO=$(grep -n 'FUNCTION PIDController' "$F" | head -1 | cut -d: -f1)
HI=$(grep -n 'END FUNCTION PIDController' "$F" | head -1 | cut -d: -f1)

cp "$F" /tmp/Controllers.f90.redtest-backup
sed -i.bak "${LO},${HI}${EXPR}" "$F"; rm -f "$F.bak"
if diff -q "$F" /tmp/Controllers.f90.redtest-backup > /dev/null; then
    echo "wrapper_redtest: the sed matched NOTHING -- refusing to report on an unperturbed wrapper" >&2
    exit 2
fi
docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"

bash scripts/harness.sh PIDController Controllers pidcontroller "$F" \
     --post-integration --out "$OUT" --red-test "$WHAT" > /tmp/wrapper_redtest.log 2>&1 || true
tail -3 /tmp/wrapper_redtest.log

cp /tmp/Controllers.f90.redtest-backup "$F"
docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
git diff --quiet -- "$F" && echo "wrapper_redtest: $F reverted and rebuilt" \
    || { echo "wrapper_redtest: $F IS STILL MODIFIED" >&2; exit 3; }
