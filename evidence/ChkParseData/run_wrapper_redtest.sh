#!/bin/bash
# Plant a defect in the SHIPPED WRAPPER, prove the post-integration harness sees
# it, then revert, rebuild and prove the green comes back.
#
#   bash evidence/ChkParseData/run_wrapper_redtest.sh
#
# WHY THE WRAPPER AND NOT THE C++. The pre-integration harness already covers
# the translation; this layer exists for the one thing no other layer sees --
# the Fortran bridge `vit integrate` generated. Unit #23 is the case: a wrapper
# missing `--reverse-copy` dropped every write to a scalar field, and the kernel
# and the gate both passed it.
#
# WHAT IS PERTURBED, and why this one. `ChkParseData` is the first unit in this
# campaign whose SHIPPING wrapper stages a CHARACTER ARRAY, and the whole of
# that staging is one index expression:
#
#     Words_c((vit_j_Words - 1) * (LEN(Words)) + vit_i_Words) = Words(vit_j)(vit_i:vit_i)
#
# Replacing `vit_j_Words - 1` with `2 - vit_j_Words` TRANSPOSES the two words --
# the C++ then reads Words(2) where the reference reads Words(1). Nothing about
# the types, the lengths or the ABI changes, so nothing but a value comparison
# can see it. That is the defect a column-major stride is actually exposed to.
#
# ANCHORED TO THE UNIT, NOT TO THE STRING -- unit #26, and this file is the
# reason that rule exists in its sharpest form. The line above occurs THREE
# times in ROSCO_Helpers.f90 (once here, twice in GetWords' wrapper), because a
# generator wrote all three. A `str.replace` would perturb three units, measure
# none of them, and write a red artifact indistinguishable from the right one.
# So the edit is confined to this subroutine's own line range and the other two
# occurrences are ASSERTED to survive.
#
# REBUILD BETWEEN THE EDIT AND THE RUN, both ways. A post-integration harness
# links against the built library, so an unrebuilt edit measures the previous
# one; RUNBOOK units #18/#24.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

F=rosco/controller/src/ROSCO_Helpers.f90
KEEP=$(mktemp); cp "$F" "$KEEP"

# THE EDIT MUST BE VISIBLE INSIDE THE CONTAINER BEFORE ANYTHING IS BUILT, and
# this is not a precaution -- it is a repair. On 2026-08-14 this script's own
# revert-verification read `1552 checked, 9 failed` with the source PROVABLY
# reverted (`grep '2 - vit_j_Words'` empty) and the object a second OLDER than
# the source it was supposed to be built from:
#
#   rosco/controller/src/ROSCO_Helpers.f90        23:34:47.715
#   build/.../ROSCO_Helpers.f90.o                 23:34:46.732
#
# `make` stat'd the file while the bind mount still showed the pre-revert
# content, called it up to date, and the mount caught up afterwards. So the
# green that certifies the red test measured the PERTURBED build and reported
# the red test's own failure count -- a red test whose revert is silently a
# no-op cannot certify anything.
#
# It surfaced only once the cycle got fast: the whole perturb-build-revert-build
# now runs in about three seconds, which is inside the window the race needs.
# The countermeasure is the one `run_harness_stub.sh` already uses for the same
# hazard one file over (unit #23) -- hash the file FROM INSIDE THE CONTAINER and
# do not build until it agrees.
verify_seen() {           # $1 = the content that must be visible in-container
    local want got
    want=$(md5 -q "$1")
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$F | cut -d' ' -f1")
        # TOUCHED FROM INSIDE THE CONTAINER once the content agrees, and the
        # touch is the half that makes the rebuild DETERMINISTIC rather than
        # merely likely. The hash says the content arrived; it says nothing
        # about the mtime `make` compared against, and the failure this guard
        # was written for was an object stamped a second BEFORE the source it
        # was supposed to be built from. A source that is unambiguously newer
        # than its object cannot be skipped.
        [ "$want" = "$got" ] && {
            docker exec vit-dev bash -lc "touch /workspace/ROSCO-r2/$F"
            echo "  container sees $got (and the source is now newer than its object)"
            return 0; }
        sleep 1
    done
    echo "run_wrapper_redtest: the container still sees $got, not $want" >&2
    return 1
}
rebuild() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j8 > /dev/null \
         && cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
}
restore() {
    cp "$KEEP" "$F"
    verify_seen "$KEEP"
    rm -f "$KEEP"
    rebuild
    echo "reverted and rebuilt"
}
trap restore EXIT

python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().splitlines(keepends=True)

FROM = "(vit_j_Words - 1) * (LEN(Words))"
TO   = "(2 - vit_j_Words) * (LEN(Words))"

total = sum(l.count(FROM) for l in lines)
print(f"occurrences of the generated stride in the whole file: {total}")
assert total >= 2, "the hazard this script exists for is gone; re-read the file"

# The unit's own line range, found by its wrapper's SUBROUTINE/END pair.
start = next(i for i, l in enumerate(lines)
             if re.match(r"\s*SUBROUTINE ChkParseData\(", l))
end = next(i for i, l in enumerate(lines[start:], start)
           if re.match(r"\s*END SUBROUTINE ChkParseData\s*$", l))
inside = sum(lines[i].count(FROM) for i in range(start, end + 1))
print(f"  inside SUBROUTINE ChkParseData (lines {start+1}..{end+1}): {inside}")
assert inside == 1, "expected exactly one stride expression in this wrapper"

for i in range(start, end + 1):
    if FROM in lines[i]:
        lines[i] = lines[i].replace(FROM, TO)
        print(f"  perturbed line {i+1}: {lines[i].strip()}")
open(p, "w").write("".join(lines))

after = sum(l.count(FROM) for l in open(p).read().splitlines())
print(f"  untouched occurrences elsewhere: {after}  (expected {total - 1})")
assert after == total - 1
PY

# The same check on the way IN, and this direction is the more dangerous one: a
# perturbation the container has not seen yet builds the ORIGINAL and reports a
# red test that stayed green -- which reads as "the layer is blind" rather than
# as "the edit did not arrive".
verify_seen "$F"
rebuild
echo "--- perturbed build in place; running the post-integration harness"
bash scripts/harness.sh ChkParseData ROSCO_Helpers chkparsedata "$F" --post-integration \
     --out harness/ChkParseData.postintegration.redtest.json \
     --red-test "the wrapper stages Words(2) where the reference reads Words(1): (vit_j_Words - 1) -> (2 - vit_j_Words)" 2>&1 | tail -6

restore; trap - EXIT
echo "--- reverted build in place; re-running the green"
bash scripts/harness.sh ChkParseData ROSCO_Helpers chkparsedata "$F" --post-integration \
     --out evidence/ChkParseData/harness.postintegration.revert-verified.json 2>&1 | tail -3
