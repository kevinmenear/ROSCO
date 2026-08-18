#!/bin/bash
# The POST-INTEGRATION harness red test for unit #51 `FloatingFeedback`.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/FloatingFeedback/run_postintegration_redtest.sh
#
# Copied from evidence/FlapControl/run_postintegration_redtest.sh (unit #50)
# with the unit, the anchors and the prediction changed. Its rules hold.
#
# WHAT THIS RUN CAN MEASURE. After integration the Fortran `FloatingFeedback`
# IS the translation -- its body is a wrapper calling `floatingfeedback_c` -- so
# no comparison against the integrated build can check the ARITHMETIC. What is
# left, and is covered nowhere else, is the MARSHALLING: three of this unit's
# four dummies cross as populated view structs, and the one scalar it writes
# comes back only because `vit integrate --reverse-copy` emitted
# `vit_copy_scalars_to_localvariables`.
#
# SO THE PERTURBATION IS THAT CALL, AND IT IS SCOPED TO THIS UNIT'S OWN WRAPPER.
# `vit_copy_scalars_to_localvariables(` appears in Controllers.f90 in more than
# one generated wrapper -- CableControl's, FlapControl's, StructuralControl's
# and ActiveWakeControl's among them -- and unit #45's rule is that a
# perturbation matching a generated line matches it in every unit that has one,
# after which the number it moves is not about the unit that claims it. The edit
# below is anchored to the FloatingFeedback wrapper's own line range, found from
# its `FUNCTION FloatingFeedback(LocalVar` and `END FUNCTION FloatingFeedback`
# lines, and the number of edits is asserted to be exactly 1.
#
# WHY `localvariables` AND NOT `errorvariables`. This unit's wrapper carries
# BOTH copy-backs, and either would go red. `LocalVar%Kp_Float` is the one that
# is THIS UNIT'S OWN answer: `interp1d` writes it, both arms read it back, and
# it is the only field of `LocalVariables` this procedure assigns. The
# `ErrorVariables` copy-back carries `aviFAIL` and `ErrMsg`, which are written
# by the CALLEE and not by this unit -- perturbing it would measure interp1d's
# marshalling under this unit's name.
#
# WHAT IT SHOULD MOVE, PREDICTED BEFORE THE RUN. Every case in which the unit
# CHANGES `Kp_Float` from the value it arrived with -- that is, every case where
# `interp1d(Fl_U, Fl_Kp, WE_Vw_F)` differs from the drawn incoming
# `LocalVar%Kp_Float`. `Kp_Float` is one of the corpus's varied reals and the
# interpolant is a different draw, so that should be nearly the whole corpus;
# it is NOT expected to be all of it, because `vit_result` is compared too and
# a case whose `Kp_Float` happens to agree will match on both. There is no
# arm-shaped pass set here: unlike unit #50, no guard skips the write.
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
E=evidence/FloatingFeedback

git show "HEAD:$F" | grep -q 'floatingfeedback_c' || {
    echo "run_postintegration_redtest: HEAD's $F does not call floatingfeedback_c."
    echo "  The EXIT trap below is \`git checkout -- $F\`, which would DELETE"
    echo "  this unit's wrapper instead of reverting the perturbation, and the"
    echo "  green that followed would be about a tree nobody meant to measure."
    echo "  Unit #49 did exactly that. Commit the integration first."; exit 1; }

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" > /tmp/ff_pi_build.log 2>&1
    local rc=$?
    [ $rc -eq 0 ] || { echo "BUILD FAILED"; tail -20 /tmp/ff_pi_build.log; }
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
    return $rc
}

trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT

python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split("\n")
lo = next(i for i, l in enumerate(lines)
          if re.match(r"\s*FUNCTION FloatingFeedback\(LocalVar", l))
hi = next(i for i, l in enumerate(lines)
          if i > lo and "END FUNCTION FloatingFeedback" in l)
n = 0
for i in range(lo, hi):
    if ("vit_copy_scalars_to_localvariables(" in lines[i]
            and lines[i].strip().startswith("CALL")):
        lines[i] = ("        ! RED TEST: this unit's LocalVariables copy-back deleted -- "
                    "Kp_Float never leaves the view struct.")
        n += 1
assert n == 1, f"expected exactly 1 edit inside the FloatingFeedback wrapper, made {n}"
open(p, "w").write("\n".join(lines))
print(f"perturbed {p}: {n} line(s), inside lines {lo+1}..{hi+1}")
PY
[ $? -eq 0 ] || exit 1

build || exit 1

bash scripts/harness.sh FloatingFeedback Controllers floatingfeedback "$F" \
    --post-integration --out "$E/harness.postintegration.redtest.json" \
    --red-test "this unit's vit_copy_scalars_to_localvariables call deleted from its own wrapper" 2>&1 | tail -6
