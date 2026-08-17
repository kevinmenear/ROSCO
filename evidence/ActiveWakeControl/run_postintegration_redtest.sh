#!/bin/bash
# The POST-INTEGRATION harness red test for unit #47 `ActiveWakeControl`.
#
#   bash scripts/run_if_time_remains.sh 600 \
#       bash evidence/ActiveWakeControl/run_postintegration_redtest.sh
#
# WHAT THIS RUN CAN MEASURE. After integration the Fortran `ActiveWakeControl`
# IS the translation -- its body is a wrapper calling `activewakecontrol_c` --
# so no comparison against the integrated build can check the ARITHMETIC. What
# is left, and is covered nowhere else, is the MARSHALLING: this unit's four
# dummies cross as two populated view structs and two flat structs, and the
# writes come back only because `vit integrate --reverse-copy` emitted two
# `vit_copy_scalars_to_*` calls.
#
# SO THE PERTURBATION IS ONE OF THOSE CALLS, AND IT IS SCOPED TO THIS UNIT'S OWN
# WRAPPER. `vit_copy_scalars_to_localvariables(` appears in Controllers.f90 in
# more than one generated wrapper -- unit #45's rule is that a perturbation
# which matches a generated line matches it in every unit that has one, and then
# the number it moves is not about the unit that claims it. The sed below is
# anchored to the ActiveWakeControl wrapper's own line range, found by matching
# its `activewakecontrol_c(` call, so nothing outside it can be touched. The
# number of edits is asserted to be exactly 1.
#
# The revert REBUILDS and the green is RE-TAKEN. Unit #43 measured a wrapper red
# test whose revert rebuilt nothing: `git diff` was clean and the installed
# library still carried the perturbation.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/Controllers.f90
E=evidence/ActiveWakeControl

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" > /tmp/awc_pi_build.log 2>&1
    local rc=$?
    [ $rc -eq 0 ] || { echo "BUILD FAILED"; tail -20 /tmp/awc_pi_build.log; }
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
    return $rc
}

trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT

# The wrapper's line range: from its SUBROUTINE line to its END SUBROUTINE line.
python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split("\n")
lo = next(i for i, l in enumerate(lines) if re.match(r"\s*SUBROUTINE ActiveWakeControl\(", l))
hi = next(i for i, l in enumerate(lines) if i > lo and "END SUBROUTINE ActiveWakeControl" in l)
n = 0
for i in range(lo, hi):
    if "vit_copy_scalars_to_localvariables(" in lines[i] and lines[i].strip().startswith("CALL"):
        lines[i] = ("        ! RED TEST: this unit's LocalVariables copy-back deleted -- "
                    "the writes to PitCom, AWC_complexangle, TiltMean and YawMean never "
                    "leave the view struct.")
        n += 1
assert n == 1, f"expected exactly 1 edit inside the ActiveWakeControl wrapper, made {n}"
open(p, "w").write("\n".join(lines))
print(f"perturbed {p}: {n} line(s), inside lines {lo+1}..{hi+1}")
PY
[ $? -eq 0 ] || exit 1

build || exit 1

bash scripts/harness.sh ActiveWakeControl Controllers activewakecontrol "$F" \
    --post-integration --out "$E/harness.postintegration.redtest.json" \
    --red-test "this unit's vit_copy_scalars_to_localvariables call deleted from its own wrapper" 2>&1 | tail -6
