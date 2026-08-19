#!/bin/bash
# WHICH cases does the no-READ stub fail on? The prediction missed by twelve and
# this is the twelve, classified rather than argued about.
#
#   bash evidence/ParseInAry_Opt/run_read_residual_probe.sh    # tree must be CLEAN
#
# WHY. `harness_partition.txt` predicted the READ-deleted stub at 634 -- the two
# `read-failed` cells, 155 + 479 -- and the run measured 646. On the second
# dispatch's corpus the same prediction was EXACT (146 = 59 + 87), so the twelve
# are something that corpus did not have. The obvious reading is that the new
# R14 numeric leads produce records whose READ SUCCEEDS, so deleting the READ is
# visible in `Ary` and not only in `aviFAIL` -- but "the obvious reading" is what
# this campaign measures instead of asserting (P10: a residual re-derived from
# the same table and then matched is the check; a residual explained in prose is
# not).
#
# HOW. One `fprintf` beside the harness's own verdict, printing the case index
# and the REFERENCE's returned aviFAIL and ErrMsg for every FAILING case, with
# the no-READ stub in place. The arm classifier is the one
# `run_partition_probe.sh` uses, with its substrings copied from
# ROSCO_Helpers.f90. The stub and the test source are both restored on the way
# out.
#
# The mismatch list in the artifact cannot answer this: the generated test caps
# `diffs` at 16 entries, so 646 failing cases leave 16 recorded.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
D=translations/ROSCO_Helpers/parseinary_opt_test
T=$D/parseinary_opt_test.cpp
LIVE=translations/ROSCO_Helpers/parseinary_opt.cpp
STUB=evidence/ParseInAry_Opt/parseinary_opt.no-read-stub.cpp

KEEPT=$(mktemp); cp "$T" "$KEEPT"
KEEPC=$(mktemp); cp "$LIVE" "$KEEPC"
restore() { cp "$KEEPT" "$T"; cp "$KEEPC" "$LIVE"; rm -f "$KEEPT" "$KEEPC"; }
trap restore EXIT

cp "$STUB" "$LIVE"
want=$(md5 -q "$STUB")
got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$LIVE | cut -d' ' -f1")
[ "$want" = "$got" ] || { echo "HASH MISMATCH on $LIVE"; exit 1; }

python3 - "$T" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
anchor = "        if (bad) failed++;\n"
assert s.count(anchor) == 1, f"anchor appears {s.count(anchor)} times"
probe = ('        if (bad) fprintf(stderr, "FAILCASE %d %d %.200s\\n", c,\n'
         '                         (int)ErrVar_b.aviFAIL, ErrVar_ErrMsg_b.data());\n')
p.write_text(s.replace(anchor, probe + anchor))
print("probe inserted")
PY

docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/$D && rm -f parseinary_opt.hpp && \
     cp /workspace/ROSCO-r2/$LIVE parseinary_opt.hpp && \
     rm -f parseinary_opt_test.o test && make test > /dev/null 2>&1 && \
     ./test parseinary_opt_cases.bin > /dev/null" \
    2> /tmp/parseinary_read_residual_raw.txt || true

python3 - /tmp/parseinary_read_residual_raw.txt <<'PY'
import collections, pathlib, sys
rows = []
for line in pathlib.Path(sys.argv[1]).read_text(errors="replace").splitlines():
    if not line.startswith("FAILCASE "):
        continue
    f = line.split(" ", 3)
    rows.append((int(f[1]), int(f[2]), f[3] if len(f) > 3 else ""))
print(f"{len(rows)} FAILING case(s) under the no-READ stub")

def arm(msg):
    m = msg.strip()
    if "Missing or default values are not allowed" in m:         return "not-allowed"
    if "A fatal error occurred when parsing data" in m:          return "read-failed"
    if "array was already allocated" in m:                       return "already-alloc"
    if "Fatal error allocating memory for the Words array" in m: return "words-alloc-failed"
    if "Error allocating memory for" in m:                       return "alloc-failed"
    return "no-error"

tab = collections.Counter(arm(r[2]) for r in rows)
print("\n  the REFERENCE's arm on each failing case")
for k in sorted(tab):
    print(f"    {k:20s} {tab[k]:6d}")
resid = [r for r in rows if arm(r[2]) == "no-error"]
print(f"\n  PREDICTED (the two read-failed cells)      634")
print(f"  measured on a read-failed arm             {tab.get('read-failed', 0)}")
print(f"  RESIDUAL -- the reference's READ SUCCEEDED {len(resid)}")
print(f"  their returned aviFAIL values              "
      f"{sorted({r[1] for r in resid})}")
print(f"  first ten case indices                     "
      f"{[r[0] for r in resid][:10]}")
PY
