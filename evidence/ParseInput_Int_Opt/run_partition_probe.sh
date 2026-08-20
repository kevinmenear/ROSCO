#!/bin/bash
# Partition the differential harness's cases by the arm the REFERENCE took, and
# count what each red test below must move.
#
#   bash evidence/ParseInput_Int_Opt/run_partition_probe.sh
#
# COPIED IN SHAPE from `evidence/ParseInAry_Opt/run_partition_probe.sh` (P4) --
# the anchor, the `--no-generate`-free rebuild, the restore-on-exit and the
# reason a probe must be a script rather than a sentence are all that file's.
# WHAT IS NOT COPIED IS THE CLASSIFIER: this unit's arms are not the siblings'.
# It has no ALLOCATE, so there is no `alloc_Ary` axis and no already-allocated
# cell; and its READ is guarded by `ErrVar%aviFAIL >= 0 .AND. FoundLine`, where
# both siblings guard on `FoundLine` alone.
#
# THE TREE MUST BE CLEAN (`scripts/reset_to_clean.sh`). This probe reads the
# reference's ANSWERS, not just the case's questions, so it needs a build in
# which `parseinput_int_opt_f90` reaches real Fortran all the way down.
#
# The five arms, read off `ROSCO_Helpers.f90` at 54dd134 (lines 144-186):
#
#   pre-failed      ErrVar%aviFAIL < 0 on entry: the whole body is skipped
#   not-allowed     .NOT. FoundLine .AND. .NOT. AllowDefault_: aviFAIL = -1,
#                   a message, RETURN -- no record
#   default-warned  .NOT. FoundLine .AND. AllowDefault_: Variable = 0 and the
#                   PRINT -- a record, and no scalar written
#   read-failed     FoundLine and the list-directed READ set IOSTAT /= 0:
#                   aviFAIL = -1 and a message
#   read-ok         FoundLine and the READ succeeded: nothing but Variable
#
# The classifier tests the reference's OWN returned message and its OWN returned
# aviFAIL, in the order the source writes them. The substrings are copied from
# the clean source rather than recalled.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
D=translations/ROSCO_Helpers/parseinput_int_opt_test
T=$D/parseinput_int_opt_test.cpp
OUT=evidence/ParseInput_Int_Opt/harness_partition.txt
KEEP=$(mktemp); cp "$T" "$KEEP"
restore() { cp "$KEEP" "$T"; rm -f "$KEEP"; }
trap restore EXIT

python3 - "$T" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()

snap = "        vit_record.begin();\n        ParseInput_Int_Opt("
assert s.count(snap) == 1, f"snapshot anchor appears {s.count(snap)} times"
s = s.replace(snap,
              "        const int vit_sup_avi = ErrVar_a.aviFAIL;\n"
              "        const int vit_sup_var = Variable_a;\n" + snap)

anchor = "        int bad = 0;\n"
assert s.count(anchor) == 1, f"anchor appears {s.count(anchor)} times"
probe = (
    '        fprintf(stderr, "PART %d %d %d %d %d %d %d %d %.200s\\n", c,\n'
    '                vit_sup_avi, (int)ErrVar_b.aviFAIL, (int)ErrVar_b.ErrStat,\n'
    '                (int)ErrVar_b.size_avcMSG, (int)(vit_rec_b.empty() ? 0 : 1),\n'
    '                (int)(Variable_b == vit_sup_var ? 0 : 1),\n'
    '                (int)(vit_sup_var == 0 ? 0 : 1),\n'
    '                ErrVar_ErrMsg_b.data());\n')
p.write_text(s.replace(anchor, probe + anchor))
print("probe inserted")
PY

docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/$D && rm -f parseinput_int_opt_test.o test && \
     make test > /dev/null 2>&1 && ./test parseinput_int_opt_cases.bin > /dev/null" \
    2> /tmp/parseinput_int_partition_raw.txt || true

python3 - /tmp/parseinput_int_partition_raw.txt "$OUT" <<'PY'
import collections, pathlib, sys
rows = []
for line in pathlib.Path(sys.argv[1]).read_text(errors="replace").splitlines():
    if not line.startswith("PART "):
        continue
    f = line.split(" ", 9)
    c, sup, avi, est, sz, rec, varmoved, varin = (int(x) for x in f[1:9])
    msg = f[9] if len(f) > 9 else ""
    rows.append((c, sup, avi, est, sz, rec, varmoved, varin, msg))

NOTALLOWED = "ParseInput_Int_Opt:Missing or default values are not allowed for "
READFAIL   = " >> A fatal error occurred when parsing data from \""

def arm(sup, avi, rec, msg):
    if sup < 0:
        return "pre-failed"
    m = msg.strip()
    if m.startswith(NOTALLOWED):
        return "not-allowed"
    if READFAIL in msg:
        return "read-failed"
    # The two arms that write aviFAIL but whose message the staging buffer
    # REFUSED (R13's short-capacity cases) are aviFAIL = -1 with the message
    # the case ARRIVED with, and NOTHING here separates them: `not-allowed`
    # RETURNs before the PRINT and `read-failed` runs only when FoundLine, so
    # neither writes a record. They are reported as one cell rather than split
    # on a guess -- the prediction this table exists for is the `scalar-changed`
    # column, which does not depend on the split.
    if avi < 0:
        return "error, msg refused"
    if rec:
        return "default-warned"
    return "read-ok"

cnt = collections.Counter()
avi_changed = collections.Counter()
rec_written = collections.Counter()
var_moved = collections.Counter()
var_in_nonzero = collections.Counter()
for c, sup, avi, est, sz, rec, vm, vin, msg in rows:
    a = arm(sup, avi, rec, msg)
    cnt[a] += 1
    if avi != sup or est != 0 or sz != 0:
        avi_changed[a] += 1
    rec_written[a] += rec
    var_moved[a] += vm
    var_in_nonzero[a] += vin

out = []
out.append(f"harness partition -- {len(rows)} case(s), "
           f"corpus translations/ROSCO_Helpers/parseinput_int_opt_test/"
           f"parseinput_int_opt_cases.bin")
out.append("")
out.append(f"{'arm':28s} {'cases':>7s} {'scalar-changed':>15s} {'record':>8s} "
           f"{'Variable moved':>15s} {'Variable IN /= 0':>17s}")
for a in sorted(cnt, key=lambda k: -cnt[k]):
    out.append(f"{a:28s} {cnt[a]:7d} {avi_changed[a]:15d} {rec_written[a]:8d} "
               f"{var_moved[a]:15d} {var_in_nonzero[a]:17d}")
out.append(f"{'TOTAL':28s} {sum(cnt.values()):7d} {sum(avi_changed.values()):15d} "
           f"{sum(rec_written.values()):8d} {sum(var_moved.values()):15d} "
           f"{sum(var_in_nonzero.values()):17d}")
out.append("")
out.append("PREDICTIONS READ OFF THIS TABLE, each stated before its run:")
out.append(f"  post-integration wrapper red test (the reverse copy deleted):")
out.append(f"      the `scalar-changed` column summed = {sum(avi_changed.values())} must FAIL")
out.append(f"      the rest, {len(rows) - sum(avi_changed.values())}, write no scalar and must PASS")
out.append(f"  the PRINT deleted:  the `record` column summed = "
           f"{sum(rec_written.values())} must FAIL")
out.append(f"  the READ deleted:   read-failed cases move a scalar and a message;"
           f" read-ok cases move Variable")
out.append("")
out.append("AND THE COLUMN THAT IS NOT A PREDICTION BUT A CORPUS MEASUREMENT:")
out.append("  `Variable IN /= 0` counts the cases that enter with a NONZERO")
out.append("  Variable. The three ways gfortran's list-directed READ leaves an")
out.append("  item UNTRANSFERRED -- an END condition, a null value, a `/`")
out.append("  terminator -- all leave `Variable` at the value it arrived with,")
out.append("  so a mutant whose only effect is to store 0 where the reference")
out.append("  stores nothing is invisible on a case that arrived at 0. Unit #56")
out.append("  measured all 293 of its cases reaching the READ entering at 0.0")
out.append("  exactly, because `_base` draws at lo + (hi - lo) * 0.5 = 0.0 on")
out.append("  the +/-1e3 default (translation-loop harness/generate.py::_base).")
out.append("  This unit's item is an INTEGER and R6's integer ladder is a")
out.append("  different draw, so the number is MEASURED here rather than")
out.append("  inherited -- the whole point of the column.")
pathlib.Path(sys.argv[2]).write_text("\n".join(out) + "\n")
print("\n".join(out))
PY
