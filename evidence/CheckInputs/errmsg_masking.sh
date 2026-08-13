#!/bin/bash
# Why 165 of CheckInputs' 173 behavioural mutants survive 16,769 cases.
#
#   bash evidence/CheckInputs/errmsg_masking.sh
#
# Four runs of the SAME harness on the SAME corpus, three of them deliberately
# perturbed. Prints the failed count of each; exit 0 when the four agree with
# what mutation_clean_tree.md records, 1 otherwise, so this can be read as a
# check rather than as output.
#
# MUST RUN ON THE CLEAN TREE. On the integrated tree the harness's Fortran side
# calls the wrapper, which calls checkinputs_c, which links the harness's own
# copy of the perturbed translation -- both sides perturbed, every difference
# cancelled, all four runs report 0 and the file reads like a refutation.
#
# So this script opens the reset window ITSELF and closes it on EXIT. That trap
# is right here and wrong in `reset_to_clean.sh`: this is one measurement whose
# lifetime is the script's, where that script deliberately leaves the tree clean
# for the session that follows it. Two units of this campaign ended inside a
# window a session had opened by hand.

set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
CPP=translations/ReadSetParameters/checkinputs.cpp
TESTDIR=/workspace/ROSCO-r2/translations/ReadSetParameters/checkinputs_test
CONTAINER="${VIT_CONTAINER:-vit-dev}"

dirty=$(git status --porcelain | grep -v '^?? ' | wc -l | tr -d ' ')
if [ "$dirty" != "0" ]; then
    echo "errmsg_masking: the tree has uncommitted changes. This script resets to"
    echo "  clean source and restores from HEAD; commit first." >&2
    exit 2
fi

cp "$CPP" /tmp/errmsg_masking_orig.cpp
cleanup() {
    cp /tmp/errmsg_masking_orig.cpp "$CPP"
    bash scripts/restore_integrated.sh >/dev/null 2>&1
    echo "  [tree restored, window closed]"
}
trap cleanup EXIT

bash scripts/reset_to_clean.sh >/dev/null 2>&1
bash scripts/harness.sh CheckInputs ReadSetParameters checkinputs \
     rosco/controller/src/ReadSetParameters.f90 --no-generate >/dev/null 2>&1

run() {
    docker exec "$CONTAINER" bash -lc \
        "cd $TESTDIR && make test >/dev/null 2>&1 && ./test 2>/dev/null" \
        | sed -n 's/.*"failed": \([0-9]*\).*/\1/p'
}
patch_helper() {   # insert one line at the top of assign_errmsg
    python3 - "$1" <<'PY'
import pathlib, sys
p = pathlib.Path('translations/ReadSetParameters/checkinputs.cpp')
t = p.read_text()
head = "void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {\n"
assert t.count(head) == 1, "assign_errmsg is not the single message sink any more"
p.write_text(t.replace(head, head + "    " + sys.argv[1] + "\n"))
PY
}

green=$(run)
patch_helper 'if (ErrVar->aviFAIL != 0) return;  // PROBE A'
probe_a=$(run)
cp /tmp/errmsg_masking_orig.cpp "$CPP"
patch_helper 'return;  // PROBE B'
probe_b=$(run)
cp /tmp/errmsg_masking_orig.cpp "$CPP"
regreen=$(run)

echo "unperturbed                          failed=$green"
echo "PROBE A  first-writer-wins           failed=$probe_a"
echo "PROBE B  no ErrMsg ever written       failed=$probe_b"
echo "unperturbed, again                   failed=$regreen"
echo
echo "A at 16769 => the FIRST failing check differs from the LAST in every case,"
echo "  so every case raises at least TWO errors and the one discriminating"
echo "  output is whichever check happens to be last."
echo "B at 16769 => every case raises at least one, so aviFAIL is -1 in all of"
echo "  them and cannot distinguish anything."

[ "$green" = "0" ] && [ "$regreen" = "0" ] \
    && [ "$probe_a" = "16769" ] && [ "$probe_b" = "16769" ] || {
    echo "MISMATCH against mutation_clean_tree.md" >&2; exit 1; }
echo "exit 0 == the masking still holds"
