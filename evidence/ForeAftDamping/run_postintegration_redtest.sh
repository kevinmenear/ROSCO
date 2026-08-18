#!/bin/bash
# The POST-INTEGRATION harness red test for unit #52 `ForeAftDamping`.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/ForeAftDamping/run_postintegration_redtest.sh
#
# Copied from evidence/FloatingFeedback/run_postintegration_redtest.sh (unit
# #51) with the unit, the anchors and the prediction changed. Its rules hold.
#
# WHAT THIS RUN CAN MEASURE. After integration the Fortran `ForeAftDamping` IS
# the translation -- its body is a wrapper calling `foreaftdamping_c` -- so no
# comparison against the integrated build can check the ARITHMETIC. What is
# left, and is covered nowhere else, is the MARSHALLING: two of this unit's
# three dummies cross as populated view structs, and BOTH of its LocalVariables
# writes come back only because `vit integrate --reverse-copy` emitted
# `vit_copy_scalars_to_localvariables`.
#
# SO THE PERTURBATION IS THAT CALL, AND IT IS SCOPED TO THIS UNIT'S OWN WRAPPER.
# `vit_copy_scalars_to_localvariables(` appears in Controllers.f90 in more than
# one generated wrapper -- CableControl's, FlapControl's, FloatingFeedback's,
# StructuralControl's and ActiveWakeControl's among them -- and unit #45's rule
# is that a perturbation matching a generated line matches it in every unit that
# has one, after which the number it moves is not about the unit that claims it.
# The edit below is anchored to the ForeAftDamping wrapper's own line range,
# found from its `SUBROUTINE ForeAftDamping(CntrPar` and
# `END SUBROUTINE ForeAftDamping` lines, and the number of edits is asserted to
# be exactly 1.
#
# WHY `localvariables` AND NOT `controlparameters`. This unit's wrapper carries
# both copy-backs, and only one of them can go red. `LocalVar%FA_AccHPFI` and
# `LocalVar%FA_PitCom` are this unit's WHOLE answer -- everything it computes
# lives in those two fields. `CntrPar` is INTENT(INOUT) in ROSCO's declaration
# but this unit assigns nothing in it, so deleting the ControlParameters
# copy-back would move NOTHING and would be a red test that stays green. (That
# is worth knowing rather than guessing about, and it is the difference between
# this unit and unit #51, whose CntrPar is INTENT(IN) and whose wrapper has no
# ControlParameters copy-back at all.)
#
# WHAT IT SHOULD MOVE, PREDICTED BEFORE THE RUN: THE WHOLE CORPUS. Deleting the
# copy-back leaves `FA_AccHPFI` and all three `FA_PitCom` slots at the values
# they arrived with. `FA_AccHPFI` is the PIController result, written on EVERY
# case with no guard, so a case can only pass if that result happens to equal
# the drawn incoming `FA_AccHPFI` bit for bit. There is no arm-shaped pass set
# here and no `NumBl == 0` exemption either: NumBl gates the FA_PitCom loop, not
# the scalar. The expectation is therefore `failed == checked == 7567`, the same
# count as `harness/ForeAftDamping.postintegration.json` (unit #26's rule), and
# any pass at all is a finding about the corpus rather than about the wrapper.
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
E=evidence/ForeAftDamping

git show "HEAD:$F" | grep -q 'foreaftdamping_c' || {
    echo "run_postintegration_redtest: HEAD's $F does not call foreaftdamping_c."
    echo "  The EXIT trap below is \`git checkout -- $F\`, which would DELETE"
    echo "  this unit's wrapper instead of reverting the perturbation, and the"
    echo "  green that followed would be about a tree nobody meant to measure."
    echo "  Unit #49 did exactly that. Commit the integration first."; exit 1; }

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" > /tmp/fad_pi_build.log 2>&1
    local rc=$?
    [ $rc -eq 0 ] || { echo "BUILD FAILED"; tail -20 /tmp/fad_pi_build.log; }
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
    return $rc
}

trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT

python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split("\n")
lo = next(i for i, l in enumerate(lines)
          if re.match(r"\s*SUBROUTINE ForeAftDamping\(CntrPar", l))
hi = next(i for i, l in enumerate(lines)
          if i > lo and "END SUBROUTINE ForeAftDamping" in l)
n = 0
for i in range(lo, hi):
    if ("vit_copy_scalars_to_localvariables(" in lines[i]
            and lines[i].strip().startswith("CALL")):
        lines[i] = ("        ! RED TEST: this unit's LocalVariables copy-back deleted -- "
                    "FA_AccHPFI and FA_PitCom never leave the view struct.")
        n += 1
assert n == 1, f"expected exactly 1 edit inside the ForeAftDamping wrapper, made {n}"
open(p, "w").write("\n".join(lines))
print(f"perturbed {p}: {n} line(s), inside lines {lo+1}..{hi+1}")
PY
[ $? -eq 0 ] || exit 1

build || exit 1

bash scripts/harness.sh ForeAftDamping Controllers foreaftdamping "$F" \
    --post-integration --out "$E/harness.postintegration.redtest.json" \
    --red-test "this unit's vit_copy_scalars_to_localvariables call deleted from its own wrapper" 2>&1 | tail -6
