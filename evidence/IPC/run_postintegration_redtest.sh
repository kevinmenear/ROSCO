#!/bin/bash
# The POST-INTEGRATION harness red test for unit #53 `IPC`.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/IPC/run_postintegration_redtest.sh
#
# Copied from evidence/ForeAftDamping/run_postintegration_redtest.sh (unit #52)
# with the unit, the anchors and the prediction changed. Its rules hold.
#
# WHAT THIS RUN CAN MEASURE. After integration the Fortran `IPC` IS the
# translation -- its body is a wrapper calling `ipc_c` -- so no comparison
# against the integrated build can check the ARITHMETIC. What is left, and is
# covered nowhere else, is the MARSHALLING: three of this unit's five dummies
# cross as populated view structs, and every one of its fourteen
# `LocalVariables` writes comes back only because `vit integrate
# --reverse-copy` emitted `vit_copy_scalars_to_localvariables`.
#
# SO THE PERTURBATION IS THAT CALL, AND IT IS SCOPED TO THIS UNIT'S OWN WRAPPER.
# `vit_copy_scalars_to_localvariables(` appears in Controllers.f90 in more than
# one generated wrapper -- CableControl's, FlapControl's, FloatingFeedback's,
# ForeAftDamping's, StructuralControl's and ActiveWakeControl's among them --
# and unit #45's rule is that a perturbation matching a generated line matches
# it in every unit that has one, after which the number it moves is not about
# the unit that claims it. The edit below is anchored to the IPC wrapper's own
# line range, found from its `SUBROUTINE IPC(CntrPar` and `END SUBROUTINE IPC`
# lines, and the number of edits is asserted to be exactly 1.
#
# WHY `localvariables` AND NOT `controlparameters` OR `errorvariables`. This
# unit's wrapper carries all three and they are not equivalent choices:
#
#   controlparameters  INERT. ROSCO declares CntrPar INTENT(INOUT) here and this
#                      unit assigns nothing in it, so deleting this call moves
#                      NOTHING -- a red test that stays green.
#   errorvariables     the CALLEES' writes plus this unit's `'IPC:'` prefix, and
#                      the prefix is behind `aviFAIL < 0`. It would go red, but
#                      on an arm rather than on this unit's answer.
#   localvariables     EVERYTHING THIS UNIT COMPUTES: the four Coleman axes,
#                      axisYawF_1P, the four IPC integrals, IPC_IntSat, the two
#                      ramped gain pairs and IPC_PitComF(3).
#
# WHAT IT SHOULD MOVE, PREDICTED BEFORE THE RUN: THE WHOLE CORPUS. The two
# `ColemanTransform` calls are the first statements of the body and have no
# guard above them, so `axisTilt_1P`, `axisYaw_1P`, `axisTilt_2P` and
# `axisYaw_2P` are written on EVERY case whatever any input holds -- the same
# argument the no-op stub made, and it went red on 63888 of 63888. A case can
# only pass here if all four Coleman outputs happen to equal the drawn incoming
# values bit for bit, AND the same holds for every other field. The expectation
# is `failed == checked == 63888`, the same count as
# `harness/IPC.postintegration.json` (unit #26's rule).
#
# Unit #52 predicted the whole corpus for the same shape and was wrong by 42,
# because its pass set was "the unit changed NOTHING in the type" rather than
# "the unit wrote nothing". If a pass set appears here it is characterised in
# evidence/IPC/, not explained away.
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
E=evidence/IPC

git show "HEAD:$F" | grep -q 'ipc_c' || {
    echo "run_postintegration_redtest: HEAD's $F does not call ipc_c."
    echo "  The EXIT trap below is \`git checkout -- $F\`, which would DELETE"
    echo "  this unit's wrapper instead of reverting the perturbation, and the"
    echo "  green that followed would be about a tree nobody meant to measure."
    echo "  Unit #49 did exactly that. Commit the integration first."; exit 1; }

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j4" > /tmp/ipc_pi_build.log 2>&1
    local rc=$?
    [ $rc -eq 0 ] || { echo "BUILD FAILED"; tail -20 /tmp/ipc_pi_build.log; }
    cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
    return $rc
}

trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT

python3 - "$F" <<'PY'
import re, sys
p = sys.argv[1]
lines = open(p).read().split("\n")
lo = next(i for i, l in enumerate(lines)
          if re.match(r"\s*SUBROUTINE IPC\(CntrPar", l))
hi = next(i for i, l in enumerate(lines)
          if i > lo and "END SUBROUTINE IPC" in l)
n = 0
for i in range(lo, hi):
    if ("vit_copy_scalars_to_localvariables(" in lines[i]
            and lines[i].strip().startswith("CALL")):
        lines[i] = ("        ! RED TEST: this unit's LocalVariables copy-back deleted -- "
                    "every LocalVariables write stays inside the view struct.")
        n += 1
assert n == 1, f"expected exactly 1 edit inside the IPC wrapper, made {n}"
open(p, "w").write("\n".join(lines))
print(f"perturbed {p}: {n} line(s), inside lines {lo+1}..{hi+1}")
PY
[ $? -eq 0 ] || exit 1

build || exit 1

bash scripts/harness.sh IPC Controllers ipc "$F" \
    --post-integration --out "$E/harness.postintegration.redtest.json" \
    --red-test "this unit's vit_copy_scalars_to_localvariables call deleted from its own wrapper" 2>&1 | tail -6
