#!/bin/bash
# The POST-INTEGRATION harness red test for unit #50 `FlapControl`.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/FlapControl/run_postintegration_redtest.sh
#
# WHAT THIS RUN CAN MEASURE. After integration the Fortran `FlapControl` IS the
# translation -- its body is a wrapper calling `flapcontrol_c` -- so no
# comparison against the integrated build can check the ARITHMETIC. What is
# left, and is covered nowhere else, is the MARSHALLING: two of this unit's four
# dummies cross as populated view structs, and the two arrays it writes come
# back only because `vit integrate --reverse-copy` emitted
# `vit_copy_scalars_to_localvariables`.
#
# SO THE PERTURBATION IS THAT CALL, AND IT IS SCOPED TO THIS UNIT'S OWN WRAPPER.
# `vit_copy_scalars_to_localvariables(` appears in Controllers.f90 in more than
# one generated wrapper -- CableControl's, StructuralControl's,
# ActiveWakeControl's and PIController's among them -- and unit #45's rule is
# that a perturbation matching a generated line matches it in every unit that
# has one, after which the number it moves is not about the unit that claims it.
# The edit below is anchored to the FlapControl wrapper's own line range, found
# from its `SUBROUTINE FlapControl(` and `END SUBROUTINE FlapControl` lines, and
# the number of edits is asserted to be exactly 1.
#
# WHY `localvariables` AND NOT `controlparameters`. This unit's whole answer is
# `LocalVar%Flp_Angle` -- three REAL(DbKi) elements of the LocalVariables view,
# which every arm writes and which the three `avrSWAP` slots then copy -- plus
# `LocalVar%RootMyb_Last` on the initialisation arm. It writes NOTHING in
# `CntrPar`, so deleting the ControlParameters copy-back would measure a path
# this unit never takes. `avrSWAP` is passed straight through as `REAL(4)(*)`
# and `objInst` by `C_LOC`, so neither depends on a copy-back at all.
#
# WHAT IT SHOULD MOVE, PREDICTED BEFORE THE RUN. Every case in which the unit
# CHANGES `Flp_Angle` or `RootMyb_Last` from the value it arrived with. That is
# a SUBSET of the no-op red test's 7292 of 9721: the no-op also fails wherever
# only `avrSWAP` moves, and `avrSWAP` is written directly rather than through
# the view. The `Flp_Mode == 1` arm assigns `Flp_Angle` to itself, so it changes
# nothing and should be in neither.
#
# COMMIT THE WRAPPER FIRST. Unit #49 measured what happens otherwise: the trap
# is `git checkout -- Controllers.f90`, and if HEAD does not yet carry this
# unit's wrapper the revert deletes the WRAPPER rather than the perturbation --
# after which the run still builds, still passes, and is a green about a tree
# nobody meant to measure. Asserted below rather than remembered.
#
# The revert REBUILDS and the green is RE-TAKEN. Unit #43 measured a wrapper red
# test whose revert rebuilt nothing: `git diff` was clean and the installed
# library still carried the perturbation.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/Controllers.f90
E=evidence/FlapControl

git show "HEAD:$F" | grep -q 'flapcontrol_c' || {
    echo "run_postintegration_redtest: HEAD's $F does not call flapcontrol_c."
    echo "  The EXIT trap below is \`git checkout -- $F\`, which would DELETE"
    echo "  this unit's wrapper instead of reverting the perturbation, and the"
    echo "  green that followed would be about a tree nobody meant to measure."
    echo "  Unit #49 did exactly that. Commit the integration first."; exit 1; }

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" > /tmp/fc_pi_build.log 2>&1
    local rc=$?
    [ $rc -eq 0 ] || { echo "BUILD FAILED"; tail -20 /tmp/fc_pi_build.log; }
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
    return $rc
}

trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT

python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split("\n")
lo = next(i for i, l in enumerate(lines)
          if re.match(r"\s*SUBROUTINE FlapControl\(", l))
hi = next(i for i, l in enumerate(lines)
          if i > lo and "END SUBROUTINE FlapControl" in l)
n = 0
for i in range(lo, hi):
    if ("vit_copy_scalars_to_localvariables(" in lines[i]
            and lines[i].strip().startswith("CALL")):
        lines[i] = ("        ! RED TEST: this unit's LocalVariables copy-back deleted -- "
                    "Flp_Angle and RootMyb_Last never leave the view struct.")
        n += 1
assert n == 1, f"expected exactly 1 edit inside the FlapControl wrapper, made {n}"
open(p, "w").write("\n".join(lines))
print(f"perturbed {p}: {n} line(s), inside lines {lo+1}..{hi+1}")
PY
[ $? -eq 0 ] || exit 1

build || exit 1

bash scripts/harness.sh FlapControl Controllers flapcontrol "$F" \
    --post-integration --out "$E/harness.postintegration.redtest.json" \
    --red-test "this unit's vit_copy_scalars_to_localvariables call deleted from its own wrapper" 2>&1 | tail -6
