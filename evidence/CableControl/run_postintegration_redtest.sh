#!/bin/bash
# The POST-INTEGRATION harness red test for unit #49 `CableControl`.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/CableControl/run_postintegration_redtest.sh
#
# WHAT THIS RUN CAN MEASURE. After integration the Fortran `CableControl` IS the
# translation -- its body is a wrapper calling `cablecontrol_c` -- so no
# comparison against the integrated build can check the ARITHMETIC. What is left,
# and is covered nowhere else, is the MARSHALLING: four of this unit's five
# dummies cross as populated view structs, and the three arrays this unit writes
# come back only because `vit integrate --reverse-copy` emitted
# `vit_copy_scalars_to_localvariables`.
#
# SO THE PERTURBATION IS THAT CALL, AND IT IS SCOPED TO THIS UNIT'S OWN WRAPPER.
# `vit_copy_scalars_to_localvariables(` appears in Controllers.f90 in more than
# one generated wrapper -- StructuralControl's, ActiveWakeControl's and
# PIController's among them -- and unit #45's rule is that a perturbation
# matching a generated line matches it in every unit that has one, after which
# the number it moves is not about the unit that claims it. The edit below is
# anchored to the CableControl wrapper's own line range, found from its
# `SUBROUTINE CableControl(` and `END SUBROUTINE CableControl` lines, and the
# number of edits is asserted to be exactly 1.
#
# WHY `localvariables` AND NOT `errorvariables`. This unit's whole answer is
# three fixed-size REAL(DbKi)(12) fields of `LocalVariables` --
# `CC_DesiredL`, `CC_ActuatedL`, `CC_ActuatedDL` -- and they are carried back by
# `vit_copy_scalars_to_localvariables` alone (vit_localvariables_view.f90:676-678).
# `avrSWAP` is passed straight through as `REAL(4)(*)` and `objInst` by `C_LOC`,
# so neither depends on a copy-back at all; deleting the ErrorVariables one would
# measure a path this unit only reaches on 223 of 3354 cases.
#
# WHAT IT SHOULD MOVE, PREDICTED BEFORE THE RUN. Every case in which the unit
# CHANGES one of the three arrays from the value it arrived with. The
# filter/integrator loop writes `CC_ActuatedDL` and `CC_ActuatedL` on every case
# with `CC_Group_N >= 1`, whatever `CC_Mode` holds, so the number should be close
# to -- and no larger than -- the no-op red test's 3207 of 3354.
#
# The revert REBUILDS and the green is RE-TAKEN. Unit #43 measured a wrapper red
# test whose revert rebuilt nothing: `git diff` was clean and the installed
# library still carried the perturbation.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/Controllers.f90
E=evidence/CableControl

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" > /tmp/cc_pi_build.log 2>&1
    local rc=$?
    [ $rc -eq 0 ] || { echo "BUILD FAILED"; tail -20 /tmp/cc_pi_build.log; }
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
    return $rc
}

trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT

python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split("\n")
lo = next(i for i, l in enumerate(lines)
          if re.match(r"\s*SUBROUTINE CableControl\(", l))
hi = next(i for i, l in enumerate(lines)
          if i > lo and "END SUBROUTINE CableControl" in l)
n = 0
for i in range(lo, hi):
    if ("vit_copy_scalars_to_localvariables(" in lines[i]
            and lines[i].strip().startswith("CALL")):
        lines[i] = ("        ! RED TEST: this unit's LocalVariables copy-back deleted -- "
                    "CC_DesiredL, CC_ActuatedL and CC_ActuatedDL never leave the view struct.")
        n += 1
assert n == 1, f"expected exactly 1 edit inside the CableControl wrapper, made {n}"
open(p, "w").write("\n".join(lines))
print(f"perturbed {p}: {n} line(s), inside lines {lo+1}..{hi+1}")
PY
[ $? -eq 0 ] || exit 1

build || exit 1

bash scripts/harness.sh CableControl Controllers cablecontrol "$F" \
    --post-integration --out "$E/harness.postintegration.redtest.json" \
    --red-test "this unit's vit_copy_scalars_to_localvariables call deleted from its own wrapper" 2>&1 | tail -6
