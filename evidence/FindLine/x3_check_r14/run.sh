#!/bin/bash
# R14's X3 check: for every unit R14 can fire on, is the corpus WITH the rule a
# byte-prefix extension of the corpus WITHOUT it?
#
#   bash scripts/reset_to_clean.sh          # narrow_widths comes from the
#   bash evidence/FindLine/x3_check_r14/run.sh   # CLEAN Fortran, not a wrapper
#   bash scripts/restore_integrated.sh
#
# THE TREE MUST BE CLEAN, and the reason is not hygiene. R12's widths are read
# out of the reference's own fixed-width locals; on an integrated tree the
# procedure is a WRAPPER and has none, so R12 reports "the reference truncates
# nothing" and R14's narrowing-width block never runs. The comparison would then
# be between two corpora neither of which is the one any unit was scored on.
#
# `FindLine`'s corpus is regenerated at the end, because the last thing this
# script would otherwise leave in `findline_test/` is the R14-OFF case file, and
# `harness.sh --post-integration` reuses whatever is there.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
X=evidence/FindLine/x3_check_r14
F=rosco/controller/src/ROSCO_Helpers.f90

for pair in "ChkParseData chkparsedata" "GetWords getwords" "FindLine findline"; do
    set -- $pair; U=$1; S=$2
    bash scripts/harness.sh "$U" ROSCO_Helpers "$S" "$F" \
         --against translation --out "$X/$U.harness.json" 2>&1 | grep -E '^HARNESS'
    cp "translations/ROSCO_Helpers/${S}_test/${S}_cases.bin" "$X/$U.r14on.bin"
    bash scripts/harness.sh "$U" ROSCO_Helpers "$S" "$F" \
         --against translation --disable R14_planted_word \
         --out "$X/$U.harness.r14off.json" 2>&1 | grep -E '^HARNESS'
    cp "translations/ROSCO_Helpers/${S}_test/${S}_cases.bin" "$X/$U.r14off.bin"
done

python3 - "$X" <<'EOF'
import pathlib, sys
X = pathlib.Path(sys.argv[1])
bad = 0
for u in ("ChkParseData", "GetWords", "FindLine"):
    on = (X / f"{u}.r14on.bin").read_bytes()
    off = (X / f"{u}.r14off.bin").read_bytes()
    ok = on[:len(off)] == off
    bad += not ok
    print(f"{u:14} r14off {len(off):>9} B   r14on {len(on):>9} B   "
          f"prefix identical: {ok}")
# A prefix that MOVED is the whole finding this script exists to catch, so it
# is an exit status and not a line of output somebody has to read.
raise SystemExit(2 if bad else 0)
EOF

# Leave FindLine's own corpus as the R14-ON one -- see the header.
bash scripts/harness.sh FindLine ROSCO_Helpers findline "$F" \
     --against translation 2>&1 | grep -E '^HARNESS'
rm -f "$X"/*.bin
