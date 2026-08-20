#!/bin/bash
# Plant a defect in the SHIPPED WRAPPER, prove the post-integration harness sees
# it, then revert, rebuild and prove the green comes back.
#
#   bash evidence/RefSpeedExclusion/run_wrapper_redtest.sh
#
# Copied from `evidence/ParseInput_Str_Opt/run_wrapper_redtest.sh` (P4), whose
# own ancestry is evidence/ChkParseData -- the container-visibility guard, the
# touch, the rebuild-both-ways and the anchored-to-the-unit edit are all that
# file's, and all of them were paid for by a failure it records. Only the unit's
# names and the perturbation differ.
#
# WHAT IS PERTURBED. `CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)`
# -- the reverse copy `--reverse-copy` put there. `objInst%instRL` is the ONLY
# output of this unit that does not travel through that call: it crosses as a
# plain `objectinstances_t*` by `C_LOC`. FA_Hist, TRA_LastRefSpd, VS_RefSpd_TRA,
# VS_RefSpd_RL, VS_RefSpd and the nested `rlP%LastSignal` all do.
#
# THE FAILING SET IS PREDICTED AT 13,777 OF 13,777, AND IT IS A MEASUREMENT
# RATHER THAN AN EXPECTATION. `evidence/RefSpeedExclusion/harness.view-noop-stub.json`
# is a PRE-integration run of a stub whose body is deleted except for the
# `instRL` increment -- so its failing set is exactly the set of cases in which
# this unit changes something inside `LocalVar`, which is exactly the set this
# red test must move. It failed 13,777 of 13,777. Two configurations, one number,
# and the prediction was taken before this script ran.
#
# ANCHORED TO THE UNIT, NOT TO THE STRING. That CALL is generated into every
# wrapper in this file that takes a `LocalVariables`, so a `str.replace` would
# perturb a dozen units, measure none of them, and write a red artifact
# indistinguishable from the right one (unit #26).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

F=rosco/controller/src/ControllerBlocks.f90
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

FROM = "CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)"
TO   = "! RED TEST: the reverse copy deleted"

total = sum(l.count(FROM) for l in lines)
print(f"occurrences of the reverse copy in the whole file: {total}")
assert total >= 2, "the hazard this script exists for is gone; re-read the file"

start = next(i for i, l in enumerate(lines)
             if re.match(r"\s*SUBROUTINE RefSpeedExclusion\(", l))
end = next(i for i, l in enumerate(lines[start:], start)
           if re.match(r"\s*END SUBROUTINE RefSpeedExclusion\s*$", l))
inside = sum(lines[i].count(FROM) for i in range(start, end + 1))
print(f"  inside SUBROUTINE RefSpeedExclusion (lines {start+1}..{end+1}): {inside}")
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
bash scripts/harness.sh RefSpeedExclusion ControllerBlocks refspeedexclusion "$F" --post-integration \
     --out harness/RefSpeedExclusion.postintegration.redtest.json \
     --red-test "this unit's own vit_copy_scalars_to_localvariables deleted from its own wrapper, so every write to FA_Hist, TRA_LastRefSpd, VS_RefSpd_TRA, VS_RefSpd_RL, VS_RefSpd and rlP%LastSignal is dropped; objInst%instRL still advances because it crosses by C_LOC" 2>&1 | tail -4

restore; trap - EXIT
echo "--- reverted; re-taking the green"
bash scripts/harness.sh RefSpeedExclusion ControllerBlocks refspeedexclusion "$F" --post-integration \
     --out harness/RefSpeedExclusion.postintegration.revert-verified.json 2>&1 | tail -3
