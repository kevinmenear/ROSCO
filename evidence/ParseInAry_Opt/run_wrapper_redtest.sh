#!/bin/bash
# Plant a defect in the SHIPPED WRAPPER, prove the post-integration harness sees
# it, then revert, rebuild and prove the green comes back.
#
#   bash evidence/ParseInAry_Opt/run_wrapper_redtest.sh
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
# THE FAILING SET IS PREDICTED BEFORE THE RUN, which is what makes the number
# evidence rather than a number. From
# `evidence/ParseInAry_Opt/harness_partition.txt`, MEASURED ON THIS UNIT'S OWN
# CORPUS, the prediction is simply that table's `aviFAIL-changed` COLUMN summed:
# a case fails iff the unit changed a scalar the copy-back is what carries.
#
#   fourth dispatch, 19,536 cases   1,322 + 104 + 233 + 7,700 + 951 + 811
#                                   = 11,121 must FAIL
#                                   8,415 write no scalar and must PASS
#                                   MEASURED 11,121, exact
#
# The counts are NOT hardcoded here on purpose -- they are read out of the
# partition each time the corpus moves, and the third dispatch's (9,937 of
# 17,520) and the second's (15,504) are in this file's git history.
#
# SECOND DISPATCH: THE NUMBER MOVED AND THE PREDICTION WAS RE-DERIVED, NOT
# EDITED TO MATCH. The first dispatch predicted and measured 10,611 of 13,674 on
# the corpus it had; the two base-draw corrections in `harness/ranges.toml`
# changed which arm the base draw takes, so both the corpus and the partition
# moved. The number above comes from the re-measured table, before this run.
#
# THIRD DISPATCH, 17,520 CASES AT LOOP 59aa876, AND THE PREDICTION IS RE-DERIVED
# A THIRD TIME. `run_partition_probe.sh` now measures the table instead of a
# sentence describing how to, and it reports the `aviFAIL-changed` column
# directly -- which IS this prediction, because the copy-back carries the
# SCALARS of the view struct back and `ErrStat` and `size_avcMSG` come back 0 on
# all 17,520 cases:
#
#     not-allowed        1224 +  881
#     read-failed         155 +  479
#     already-alloc              7094
#     error, msg refused          104
#                        ------------
#     PREDICTED                  9,937 must FAIL
#     the rest                   7,583 write no scalar and must PASS
#                        ------------
#                               17,520
#
# Predicted before this run, from `evidence/ParseInAry_Opt/harness_partition.txt`.
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
             if re.match(r"\s*SUBROUTINE ParseInAry_Opt\(", l))
end = next(i for i, l in enumerate(lines[start:], start)
           if re.match(r"\s*END SUBROUTINE ParseInAry_Opt\s*$", l))
inside = sum(lines[i].count(FROM) for i in range(start, end + 1))
print(f"  inside SUBROUTINE ParseInAry_Opt (lines {start+1}..{end+1}): {inside}")
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
bash scripts/harness.sh ParseInAry_Opt ROSCO_Helpers parseinary_opt "$F" --post-integration \
     --out harness/ParseInAry_Opt.postintegration.redtest.json \
     --red-test "this unit's own vit_copy_scalars_to_errorvariables deleted from its own wrapper, so every write to the scalar ErrVar%aviFAIL is dropped" 2>&1 | tail -4

restore; trap - EXIT
echo "--- reverted; re-taking the green"
bash scripts/harness.sh ParseInAry_Opt ROSCO_Helpers parseinary_opt "$F" --post-integration \
     --out harness/ParseInAry_Opt.postintegration.revert-verified.json 2>&1 | tail -3
