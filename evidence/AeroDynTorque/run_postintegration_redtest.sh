#!/bin/bash
# The POST-INTEGRATION harness red test for unit #48 `AeroDynTorque`.
#
#   bash scripts/run_if_time_remains.sh 600 \
#       bash evidence/AeroDynTorque/run_postintegration_redtest.sh
#
# WHAT THIS RUN CAN MEASURE. After integration the Fortran `AeroDynTorque` IS
# the translation -- its body is a wrapper calling `aerodyntorque_c` -- so no
# comparison against the integrated build can check the ARITHMETIC. What is
# left, and is covered nowhere else, is the MARSHALLING: four of this unit's six
# dummies cross as populated view structs, and the one write that comes back
# does so only because `vit integrate --reverse-copy` emitted
# `vit_copy_scalars_to_errorvariables`.
#
# SO THE PERTURBATION IS THAT CALL, AND IT IS SCOPED TO THIS UNIT'S OWN WRAPPER.
# `vit_copy_scalars_to_errorvariables(` appears in Functions.f90 in more than
# one generated wrapper -- interp1d's and interp2d's among them -- and unit
# #45's rule is that a perturbation matching a generated line matches it in
# every unit that has one, after which the number it moves is not about the unit
# that claims it. The edit below is anchored to the AeroDynTorque wrapper's own
# line range, found from its `FUNCTION AeroDynTorque(` and `END FUNCTION
# AeroDynTorque` lines, and the number of edits is asserted to be exactly 1.
#
# WHAT IT SHOULD MOVE, PREDICTED BEFORE THE RUN. The copy-back carries
# `size_avcMSG`, `aviFAIL` and `ErrStat`. `aviFAIL` is the one this unit's
# callee writes: the C++ interp2d sets it to -1 on a malformed table, and
# `harness/AeroDynTorque.json`'s knob puts it at -1, 0, 1 and 2 on entry. So the
# cases that must move are those in which the C++ side's `aviFAIL` on exit
# differs from the value on entry. ErrMsg is NOT carried by the scalar copy-back
# -- it is a pointer into the caller's own buffer -- so a message written
# through the view still arrives, and the count below is about `aviFAIL` alone.
#
# The revert REBUILDS and the green is RE-TAKEN. Unit #43 measured a wrapper red
# test whose revert rebuilt nothing: `git diff` was clean and the installed
# library still carried the perturbation.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/Functions.f90
E=evidence/AeroDynTorque

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" > /tmp/adt_pi_build.log 2>&1
    local rc=$?
    [ $rc -eq 0 ] || { echo "BUILD FAILED"; tail -20 /tmp/adt_pi_build.log; }
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
    return $rc
}

trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT

python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split("\n")
lo = next(i for i, l in enumerate(lines)
          if re.match(r"\s*FUNCTION AeroDynTorque\(", l))
hi = next(i for i, l in enumerate(lines)
          if i > lo and "END FUNCTION AeroDynTorque" in l)
n = 0
for i in range(lo, hi):
    if ("vit_copy_scalars_to_errorvariables(" in lines[i]
            and lines[i].strip().startswith("CALL")):
        lines[i] = ("        ! RED TEST: this unit's ErrorVariables copy-back deleted -- "
                    "aviFAIL, ErrStat and size_avcMSG never leave the view struct.")
        n += 1
assert n == 1, f"expected exactly 1 edit inside the AeroDynTorque wrapper, made {n}"
open(p, "w").write("\n".join(lines))
print(f"perturbed {p}: {n} line(s), inside lines {lo+1}..{hi+1}")
PY
[ $? -eq 0 ] || exit 1

build || exit 1

bash scripts/harness.sh AeroDynTorque Functions aerodyntorque "$F" \
    --post-integration --out "$E/harness.postintegration.redtest.json" \
    --red-test "this unit's vit_copy_scalars_to_errorvariables call deleted from its own wrapper" 2>&1 | tail -6
