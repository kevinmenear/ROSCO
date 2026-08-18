#!/bin/bash
# Reproduce evidence/ParseInAry_Opt/r13_base_draw_probe.txt.
#
#   bash evidence/ParseInAry_Opt/run_base_draw_probe.sh
#
# WHAT IT ASKS. Two questions about the CORPUS, both of which a mutation
# survivor raised and neither of which any artifact already answered:
#
#   1. Is there a case in which the reference assigns an `ErrVar%ErrMsg` whose
#      LENGTH equals the staging capacity the case supplied? That is the only
#      input that can tell `s.size() > cap` from `s.size() >= cap`, which is
#      survivor 66860b6f.
#   2. What does R13's 256-case staging-capacity block have at its base draw?
#      R13 exists to produce the case question 1 asks for.
#
# HOW, and it is unit #47's probe unchanged: ONE fprintf added to the generated
# test, a rebuild WITHOUT regenerating the corpus, and the analysis done offline
# from the printed stream. Everything printed is either a quantity the CASE
# supplied or a value the REFERENCE returned -- nothing on the translation's
# side -- so the table is a property of (corpus x reference) and is identical
# under every mutant.
#
# THE TREE MAY BE INTEGRATED for this one, and that is stated rather than
# assumed. The probe never compares the two sides; it reads the reference's
# returned `aviFAIL` and `n_ErrMsg`, and on an integrated tree that reference is
# the wrapper around the shipped translation. That is admissible HERE because
# both runs report 13,674 / 0 -- the pre-integration harness against the clean
# Fortran and the post-integration harness against the wrapper -- so the message
# lengths are the same bytes either way. A probe that asked about ARITHMETIC
# would not be admissible on this tree and this one does not.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
DIR=translations/ROSCO_Helpers/parseinary_opt_test
T="$DIR/parseinary_opt_test.cpp"

[ -f "$T" ] || { echo "no generated test at $T -- run scripts/harness.sh first" >&2; exit 1; }
cp "$T" "$T.probe.keep"
restore() { cp "$T.probe.keep" "$T"; rm -f "$T.probe.keep"; }
trap restore EXIT

python3 - "$T" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); t = p.read_text()
anchor = "        int bad = 0;\n"
probe = '''        fprintf(stderr, "PROBE %d cap=%d head=%d pre_n=%d post_n=%d entry_avi=%d "
                "post_avi=%d alloc=%d arylen=%d n_ary=%d hasAD=%d AD=%d hasUn=%d "
                "Un=%d lenFL=%d nFL=%d lenPN=%d\\n",
                c, (int)ErrVar_b.n_ErrMsg_cap, (int)ErrVar_ErrMsg_headroom,
                (int)(ErrVar_b.n_ErrMsg_cap - ErrVar_ErrMsg_headroom),
                (int)ErrVar_b.n_ErrMsg,
                (int)vit_supplied_ErrVar_aviFAIL, (int)ErrVar_b.aviFAIL,
                (int)vit_supplied_alloc_Ary, AryLen_a, (int)n_Ary_a,
                has_AllowDefault_a, AllowDefault_a, has_UnEc_a, UnEc_a,
                (int)len_FileLines_a, (int)n_FileLines_a, (int)len_ParamName_a);
'''
assert t.count(anchor) == 1, "the anchor is not unique -- the generated test moved"
p.write_text(t.replace(anchor, probe + anchor))
print("probe inserted at 1 site")
PY

# `pre_n` is recovered as `cap - headroom` rather than read from `ErrVar_a`:
# both calls have already run at the print, so `ErrVar_a.n_ErrMsg` is the
# TRANSLATION's post value and would be a mutant-dependent quantity in a table
# that must not have one.
bash scripts/harness.sh ParseInAry_Opt ROSCO_Helpers parseinary_opt \
     rosco/controller/src/ROSCO_Helpers.f90 \
     --post-integration --out "$DIR/probe_post.json" 2>&1 | tail -2

docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/$DIR && ./test parseinary_opt_cases.bin \
     > /dev/null 2> probe_stderr.txt; wc -l < probe_stderr.txt"

echo "raw stream: $DIR/probe_stderr.txt"
echo "the table in r13_base_draw_probe.txt is that stream aggregated; the"
echo "aggregation is plain counting and is reproduced in DECISIONS.md."
