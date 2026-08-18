#!/bin/bash
# Plant a defect in the SHIPPED WRAPPER, prove the post-integration harness sees
# it, then revert, rebuild and prove the green comes back.
#
#   bash evidence/ParseDbAry_Opt/run_wrapper_redtest.sh
#
# Copied from `evidence/ChkParseData/run_wrapper_redtest.sh` (P4) -- the
# container-visibility guard, the touch, the rebuild-both-ways and the
# anchored-to-the-unit edit are all that file's, and all of them were paid for
# by a failure it records. Only the unit's names and the perturbation differ.
#
# WHAT IS PERTURBED. `CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)`
# -- the reverse copy `--reverse-copy` put there. This unit's only writes to a
# derived-type dummy are the SCALAR `ErrVar%aviFAIL` and the deferred-length
# `ErrVar%ErrMsg`; without the reverse copy the first is dropped on the floor.
# Unit #23 is the case this layer exists for.
#
# THE PASS SET IS PREDICTED BEFORE THE RUN, which is what makes the number
# evidence rather than a number. From
# `evidence/ParseDbAry_Opt/harness_partition.txt`, the cases that reach an arm
# writing `aviFAIL` are `not-allowed` (4006 + 805), `read-failed` (61 + 42) and
# `already-alloc` (5697) = 10,611; the `other` cells (3003 + 60) = 3063 write no
# scalar at all and must still pass.
#
# THIRD DISPATCH, on the 16,512-case corpus, re-derived from the re-taken
# partition rather than scaled from the number above: `not-allowed`
# (5207 + 872), `read-failed` (59 + 311) and `already-alloc` (6767) = 13,216
# must fail; `other` (3036) and `entered-failed` (184 + 76) = 3,296 write no
# scalar and must pass. `ErrStat` and `size_avcMSG` are 0 both supplied and
# returned in all 16,512, so `aviFAIL` alone decides the set.
#
#     PREDICTED 13216 failing, 3296 passing.
#
# ANCHORED TO THE UNIT, NOT TO THE STRING. That CALL is generated into every
# wrapper in this file that takes an `ErrorVariables`, so a `str.replace` would
# perturb a dozen units, measure none of them, and write a red artifact
# indistinguishable from the right one (unit #26).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

F=rosco/controller/src/ROSCO_Helpers.f90
KEEP=$(mktemp); cp "$F" "$KEEP"

verify_seen() {           # $1 = the content that must be visible in-container
    local want got
    want=$(md5 -q "$1")
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$F | cut -d' ' -f1")
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

FROM = "CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)"
TO   = "! RED TEST: the reverse copy deleted"

total = sum(l.count(FROM) for l in lines)
print(f"occurrences of the reverse copy in the whole file: {total}")
assert total >= 2, "the hazard this script exists for is gone; re-read the file"

start = next(i for i, l in enumerate(lines)
             if re.match(r"\s*SUBROUTINE ParseDbAry_Opt\(", l))
end = next(i for i, l in enumerate(lines[start:], start)
           if re.match(r"\s*END SUBROUTINE ParseDbAry_Opt\s*$", l))
inside = sum(lines[i].count(FROM) for i in range(start, end + 1))
print(f"  inside SUBROUTINE ParseDbAry_Opt (lines {start+1}..{end+1}): {inside}")
assert inside == 1, "expected exactly one reverse copy in this wrapper"

for i in range(start, end + 1):
    if FROM in lines[i]:
        lines[i] = lines[i].replace(FROM, TO)
        print(f"  perturbed line {i+1}: {lines[i].strip()}")
open(p, "w").write("".join(lines))

after = sum(l.count(FROM) for l in open(p).read().splitlines())
print(f"  untouched occurrences elsewhere: {after}  (expected {total - 1})")
assert after == total - 1
PY

verify_seen "$F"
rebuild
echo "--- perturbed build in place; running the post-integration harness"
bash scripts/harness.sh ParseDbAry_Opt ROSCO_Helpers parsedbary_opt "$F" --post-integration \
     --out harness/ParseDbAry_Opt.postintegration.redtest.json \
     --red-test "this unit's own vit_copy_scalars_to_errorvariables deleted from its own wrapper, so every write to the scalar ErrVar%aviFAIL is dropped" 2>&1 | tail -4

restore; trap - EXIT
echo "--- reverted; re-taking the green"
bash scripts/harness.sh ParseDbAry_Opt ROSCO_Helpers parsedbary_opt "$F" --post-integration \
     --out harness/ParseDbAry_Opt.postintegration.revert-verified.json 2>&1 | tail -3
