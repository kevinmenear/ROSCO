#!/bin/bash
# Red test for VIT's `vit check` procedure scoping (vit/checks.py::procedure_slice).
#
# The defect: every Fortran-reading check was handed the WHOLE FILE, so a
# sibling procedure could supply the Fortran half of a finding. FindLine
# contains no SCAN/INDEX/VERIFY at all and was reported as missing GetPath's
# backslash, 900 lines away in the same file.
#
# The fix is only worth having if the check still FIRES when the delimiter set
# genuinely differs inside the procedure being checked. Probe 3 is that test,
# and its first version was a silent no-op -- the sed matched nothing and the
# green it produced was the absence of a perturbation, not the absence of a
# defect. It now refuses rather than passing when the edit does not land.
#
# Run inside the container:  bash evidence/FindLine/vit_check_scope_redtest.sh
set -u
cd /workspace/ROSCO-r2
F=rosco/controller/src/ROSCO_Helpers.f90

echo "=== 1. FindLine: the false positive must be GONE"
vit check translations/ROSCO_Helpers/findline.cpp -f $F --function FindLine
echo "exit $?"

echo
echo "=== 2. GetPath, shipped: the same check RUNS over its own procedure and passes"
vit check translations/ROSCO_Helpers/getpath.cpp -f $F --function GetPath
echo "exit $?"

echo
echo "=== 3. GetPath, backslash literal replaced by '@': the check MUST fire"
sed "s/index_back(GivenFil, len_GivenFil, '\\\\\\\\')/index_back(GivenFil, len_GivenFil, '@')/" \
    translations/ROSCO_Helpers/getpath.cpp > /tmp/getpath_broken.cpp
grep -n "index_back(GivenFil, len_GivenFil, '@')" /tmp/getpath_broken.cpp \
    || { echo "PERTURBATION DID NOT LAND -- this probe measures nothing"; exit 9; }
vit check /tmp/getpath_broken.cpp -f $F --function GetPath
echo "exit $?  (1 is the pass condition here)"

echo
echo "=== 4. a name absent from the file: falls back to the whole file and SAYS SO"
vit check translations/ROSCO_Helpers/findline.cpp -f $F --function NoSuchProcedure
echo "exit $?"
