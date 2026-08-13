#!/bin/bash
# Plant a defect in the SHIPPED WRAPPER, prove the post-integration harness sees
# it, then revert, rebuild and prove the green comes back.
#
#   bash evidence/wrap_360/run_wrapper_redtest.sh
#
# WHY THE WRAPPER AND NOT THE C++. The pre-integration harness already covers the
# translation; this layer exists for the one thing no other layer sees -- the
# Fortran bridge `vit integrate` generated. Unit #23 is the case: a wrapper
# missing `--reverse-copy` dropped every write to a scalar field, and the kernel
# (62/62) and the gate (5,252,000 / 0) both passed it.
#
# wrap_360's wrapper has no reverse-copy line to delete: its only dummy is
# INTENT(IN). What it does have is the ARGUMENT HANDOFF, so that is what is
# perturbed -- the sign of `x` on its way into `wrap_360_c`.
#
# The SIGN is chosen deliberately over, say, a scale factor. `wrap_360` is not
# symmetric about 0 the way `wrap_180` is about its own centre: negating x maps
# the live high arm onto the dead low arm, so the perturbation is one this
# unit's own corpus has a reason to catch and the two bit-exact layers, which
# never see the low arm, would not.
#
# REBUILD BETWEEN THE EDIT AND THE RUN, both ways. A post-integration harness
# links against the built library, so an unrebuilt edit measures the previous
# one; RUNBOOK unit #18/#24.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

F=rosco/controller/src/Functions.f90
FROM='wrap_360_result = REAL(wrap_360_c(x), 8)'
TO='wrap_360_result = REAL(wrap_360_c(-x), 8)'

# Anchored to a line that names the unit, and asserted UNIQUE before cutting:
# unit #26 perturbed by matching a generated line and hit three units at once.
n=$(grep -c -F "$FROM" "$F")
echo "anchor occurrences in $F: $n"
[ "$n" = "1" ] || { echo "REFUSING: the anchor is not unique"; exit 1; }

KEEP=$(mktemp); cp "$F" "$KEEP"
rebuild() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j8 > /dev/null \
         && cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
}
restore() {
    cp "$KEEP" "$F"; rm -f "$KEEP"
    rebuild
    echo "reverted and rebuilt"
}
trap restore EXIT

python3 - "$F" "$FROM" "$TO" <<'PY'
import sys
p, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(p).read()
assert s.count(a) == 1
open(p, 'w').write(s.replace(a, b))
PY
grep -n -F "$TO" "$F"
rebuild
echo "--- perturbed build in place; running the post-integration harness"
bash scripts/harness.sh wrap_360 Functions wrap_360 "$F" --post-integration \
     --out harness/wrap_360.postintegration.redtest.json \
     --red-test "the wrapper hands -x to wrap_360_c instead of x" 2>&1 | tail -6

restore; trap - EXIT
echo "--- reverted build in place; re-running the green"
bash scripts/harness.sh wrap_360 Functions wrap_360 "$F" --post-integration \
     --out evidence/wrap_360/harness.postintegration.revert-verified.json 2>&1 | tail -3
