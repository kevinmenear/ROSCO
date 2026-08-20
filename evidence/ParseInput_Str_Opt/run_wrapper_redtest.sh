#!/bin/bash
# Plant a defect in the SHIPPED WRAPPER, prove the post-integration harness sees
# it, then revert, rebuild and prove the green comes back.
#
#   bash evidence/ParseInput_Str_Opt/run_wrapper_redtest.sh
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
# THE FAILING SET IS PREDICTED BEFORE THE RUN, and it is read out of
# `evidence/ParseInput_Str_Opt/harness_partition.txt` rather than typed here:
# a case fails iff the reference changed a scalar the copy-back is what
# carries. That is the partition's `scalar-changed` column summed.
#
#   not-allowed                 2123
#   not-allowed, msg refused      91
#                        ------------
#   PREDICTED                   2,214 must FAIL
#   the rest                   11,902 write no scalar and must PASS
#                        ------------
#                              14,116
#
# The arms that write no scalar are `default-warned` (10,595 -- it writes
# `Variable` and a stdout record, and neither crosses the view struct) and
# `read-ok` (1,117). `pre-failed` (190) writes nothing at all.
#
# ONE DIFFERENCE FROM THE SIBLING, and it is the same one that runs through this
# unit: both `not-allowed` rows are the SAME ARM here. On #57 the msg-refused
# cell held two arms and needed an identity to split; this unit's READ-error arm
# is dead, so the cell is unambiguous and the sum is a point prediction.
#
# The count is NOT hardcoded on purpose -- it is read out of the partition each
# time the corpus moves, exactly as the sibling does.
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
             if re.match(r"\s*SUBROUTINE ParseInput_Str_Opt\(", l))
end = next(i for i, l in enumerate(lines[start:], start)
           if re.match(r"\s*END SUBROUTINE ParseInput_Str_Opt\s*$", l))
inside = sum(lines[i].count(FROM) for i in range(start, end + 1))
print(f"  inside SUBROUTINE ParseInput_Str_Opt (lines {start+1}..{end+1}): {inside}")
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
bash scripts/harness.sh ParseInput_Str_Opt ROSCO_Helpers parseinput_str_opt "$F" --post-integration \
     --out harness/ParseInput_Str_Opt.postintegration.redtest.json \
     --red-test "this unit's own vit_copy_scalars_to_errorvariables deleted from its own wrapper, so every write to the scalar ErrVar%aviFAIL is dropped" 2>&1 | tail -4

restore; trap - EXIT
echo "--- reverted; re-taking the green"
bash scripts/harness.sh ParseInput_Str_Opt ROSCO_Helpers parseinput_str_opt "$F" --post-integration \
     --out harness/ParseInput_Str_Opt.postintegration.revert-verified.json 2>&1 | tail -3
