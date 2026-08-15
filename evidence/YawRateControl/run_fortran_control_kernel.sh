#!/bin/bash
# Rebuild this unit's KGen kernel with the ORIGINAL FORTRAN BODY in place of the
# C++ bridge, run it against the SAME 104 captured cases, and diff its verbose
# output against the C++ kernel's.
#
#   bash evidence/YawRateControl/run_fortran_control_kernel.sh
#
# WHY THIS EXISTS. `vit verify` printed `VERIFICATION PASSED: 104/104 passed`
# while the kernel it had just run printed `FAILED verification` and
# `Number of verification-passed cases : 102`. Two field rows in
# `kernel.verify_fields.csv` are OUT_TOL, both of them `debugvar%yawratecom`,
# both at an edge of this unit's state machine.
#
# Two readings of those two rows are possible and they call for opposite
# actions:
#
#   (a) the translation is wrong at exactly the two invocations where the state
#       machine changes state -- which is where a wrong translation WOULD show
#       first, and would be a defect to fix;
#   (b) the kernel cannot replay this unit at all, because the driver calls the
#       subroutine three times per case (evalstage, warmupstage, mainstage) and
#       verifies the THIRD, while `INTEGER, SAVE :: YawState` is a subroutine
#       local that no state file carries and nothing restores between them.
#
# A POSITIVE CONTROL DISTINGUISHES THEM AND NOTHING ELSE DOES (P10, X4). If the
# reference's OWN body fails the same two cases with the same numbers, the rows
# are the instrument's; if it passes them, they are the translation's.
#
# The control swaps ONE file. `Controllers.f90.kgen` is KGen's own output before
# VIT substituted the C++ bridge into it, so it holds the original
# `YawRateControl` body, the four SAVE declarations included. Everything else --
# the driver, the state files, the Makefile, the flags, the other modules -- is
# the kernel VIT built, unchanged, so the only difference between the two runs
# is which language computes the body.
#
# `kernel_driver.f90.kgen` is deliberately NOT restored: KGen emits `USE discon`
# for a callsite that is a SUBROUTINE, which does not compile, and VIT patches
# that out. Restoring it would replace the question with a build failure.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORKDIR="/workspace/$(basename "$ROOT")"
K="$WORKDIR/kernel/YawRateControl"
C="$WORKDIR/kernel/YawRateControl_fortran_control"

docker exec "$CONTAINER" bash -lc "
set -e
rm -rf '$C'
cp -r '$K' '$C'
cd '$C'
rm -f *.o *.mod kernel.exe
cp Controllers.f90.kgen Controllers.f90     # <- the whole of the change
make build 2>&1 | tail -1
./kernel.exe > run.txt 2>&1 || true
cd '$K'
./kernel.exe > cpp_run.txt 2>&1 || true

echo '--- the C++ kernel VIT built ---'
grep -n 'Verification FAILED' cpp_run.txt || echo '(none)'
sed -n '/kernel execution summary/,/FAILED verification/p' cpp_run.txt | grep -E 'Total number|verification-passed|FAILED'

echo
echo '--- the same kernel with the ORIGINAL FORTRAN BODY ---'
grep -n 'Verification FAILED' '$C/run.txt' || echo '(none)'
sed -n '/kernel execution summary/,/FAILED verification/p' '$C/run.txt' | grep -E 'Total number|verification-passed|FAILED'

echo
echo '--- the two mismatching field rows, reference body ---'
grep 'yawratecom' '$C/run.txt' | grep -v IDENTICAL

echo
echo '--- full verbose output, C++ against reference body ---'
echo '    (timing lines dropped: they are wall clock, not a result)'
grep -v 'usec\|call time' cpp_run.txt > /tmp/a.txt
grep -v 'usec\|call time' '$C/run.txt' > /tmp/b.txt
echo \"    lines compared : \$(wc -l < /tmp/a.txt)\"
echo \"    lines differing: \$(diff /tmp/a.txt /tmp/b.txt | wc -l)\"
"
