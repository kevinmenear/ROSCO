#!/bin/bash
# Partition the differential harness's cases by (alloc_Ary on entry, the arm the
# REFERENCE took), and count what a deleted scalar copy-back would move.
#
#   bash evidence/ParseDbAry_Opt/run_partition_probe.sh
#
# WHY IT IS A SCRIPT NOW. The first dispatch produced `harness_partition.txt`
# with "one fprintf in the generated test and a --no-generate rebuild", and left
# no way to repeat it. The corpus then moved from 13,674 cases to 16,512 and the
# table -- which the wrapper red test's PREDICTED pass set is read out of --
# could not be re-taken. A probe whose method is a sentence is a probe that
# expires with the corpus it was run on.
#
# WHAT IT ADDS, and it is one statement: after both calls, print the case index,
# the `alloc_Ary` the case SUPPLIED, the `aviFAIL` it supplied, the three scalars
# the reference RETURNED, and the reference's `ErrMsg`. Every field is read off
# the `_b` (reference) side except the two supplied ones, which the generated
# test already snapshots before the calls for `no_oracle_when`.
#
# The test .cpp is restored on the way out and the harness is left rebuilt, so
# this leaves the tree as it found it.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
D=translations/ROSCO_Helpers/parsedbary_opt_test
T=$D/parsedbary_opt_test.cpp
KEEP=$(mktemp); cp "$T" "$KEEP"
restore() { cp "$KEEP" "$T"; rm -f "$KEEP"; }
trap restore EXIT

python3 - "$T" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
anchor = "        int bad = 0;\n"
assert s.count(anchor) == 1, f"anchor appears {s.count(anchor)} times"
probe = (
    '        fprintf(stderr, "PART %d %d %d %d %d %d %.200s\\n", c,\n'
    '                (int)vit_supplied_alloc_Ary, (int)vit_supplied_ErrVar_aviFAIL,\n'
    '                (int)ErrVar_b.aviFAIL, (int)ErrVar_b.ErrStat,\n'
    '                (int)ErrVar_b.size_avcMSG, ErrVar_ErrMsg_b.data());\n')
p.write_text(s.replace(anchor, probe + anchor))
print("probe inserted")
PY

docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/$D && rm -f parsedbary_opt_test.o test && \
     make test > /dev/null 2>&1 && ./test parsedbary_opt_cases.bin > /dev/null" \
    2> /tmp/parsedbary_partition_raw.txt || true

python3 - /tmp/parsedbary_partition_raw.txt <<'PY'
import collections, pathlib, sys, re
rows = []
for line in pathlib.Path(sys.argv[1]).read_text(errors="replace").splitlines():
    if not line.startswith("PART "):
        continue
    f = line.split(" ", 7)
    c, alloc, sup, avi, est, sz = (int(x) for x in f[1:7])
    msg = f[7] if len(f) > 7 else ""
    rows.append((c, alloc, sup, avi, est, sz, msg))
print(f"{len(rows)} case(s) with a probe record")

# THE ARM IS READ OFF THE REFERENCE'S OWN MESSAGE, not off a guess about which
# branch ran, and the substrings are COPIED from `ROSCO_Helpers.f90` at the
# clean baseline rather than recalled. Tested in the order the source WRITES
# them, last writer first: the already-allocated ALLOCATE arm sets `ErrMsg` and
# does not return, so the `.NOT. AllowDefault_` arm and the READ arm overwrite
# it, and the final message is the arm the call ended on.
#
# The first version of this classifier guessed the strings ("Line does not
# contain", "Error parsing") and matched NONE of them, so 8,302 cases came back
# `other` and the two error cells read as empty. Kept in the history rather than
# quietly corrected: a partition whose buckets are spelled wrong reports the
# region it cannot see as absent.
def arm(msg, sup, avi):
    m = msg.strip()
    if sup < 0:                                                  return "entered-failed"
    if "Missing or default values are not allowed" in m:         return "not-allowed"
    if "A fatal error occurred when parsing data" in m:          return "read-failed"
    if "array was already allocated" in m:                       return "already-alloc"
    if "Fatal error allocating memory for the Words array" in m: return "words-alloc-failed"
    if "Error allocating memory for" in m:                       return "alloc-failed"
    return "other"

tab = collections.Counter((r[1], arm(r[6], r[2], r[3])) for r in rows)
chg = collections.Counter((r[1], arm(r[6], r[2], r[3]))
                          for r in rows if r[3] != r[2])
print("\n  alloc  arm                 cases   aviFAIL-changed")
for k in sorted(tab):
    print(f"    {k[0]:3d}  {k[1]:18s} {tab[k]:7d}   {chg.get(k, 0):7d}")

# WHAT A DELETED `vit_copy_scalars_to_errorvariables` WOULD MOVE. The call
# carries the SCALARS of the view struct back into the Fortran derived type, so
# without it the reference side returns the values the case SUPPLIED. A case
# fails iff the unit changed one of them -- and `aviFAIL` is the only one the
# probe can compare against its supplied value directly, so `ErrStat` and
# `size_avcMSG` are reported beside it rather than folded in.
moved = sum(1 for r in rows if r[3] != r[2])
print(f"\n  cases where the reference CHANGED aviFAIL: {moved} of {len(rows)}")
print(f"  distinct returned ErrStat values: {sorted({r[4] for r in rows})}")
print(f"  distinct returned size_avcMSG:    {sorted({r[5] for r in rows})[:8]}")
PY
