#!/bin/bash
# Partition the differential harness's cases by the arm the REFERENCE took, and
# count what each red test below must move.
#
#   bash evidence/ParseInput_Str_Opt/run_partition_probe.sh
#
# COPIED IN SHAPE from `evidence/ParseInput_Int_Opt/run_partition_probe.sh`
# (P4) -- the anchor, the restore-on-exit and the reason a probe must be a
# script rather than a sentence are that file's. WHAT IS NOT COPIED IS THE
# CLASSIFIER, and this unit's differs from the sibling's in one structural way:
#
#   `read-failed` IS DEAD. The READ is `'(A)'`, not list-directed, and an `A`
#   edit descriptor whose width equals the item length over a padded internal
#   record has no failure mode -- 144 of 144 pairs at IOSTAT 0 in
#   `record_form_probe.txt`. So the ONLY arm that assigns `ErrVar%ErrMsg` is
#   `not-allowed`, and R13's short-capacity cell -- the cases where the staging
#   buffer REFUSES the message, which unit #57 could not split without the
#   A + B = 901 identity -- is unambiguously that one arm here. The cell needs
#   no identity to resolve, which is the whole difference the dead arm makes to
#   this table.
#
# THE TREE MUST BE CLEAN (`scripts/reset_to_clean.sh`). This probe reads the
# reference's ANSWERS, not just the case's questions, so it needs a build in
# which `parseinput_str_opt_f90` reaches real Fortran all the way down.
#
# The arms, read off `ROSCO_Helpers.f90` at 54dd134 (lines 278-354):
#
#   pre-failed      ErrVar%aviFAIL < 0 on entry: the whole body is skipped
#   not-allowed     .NOT. FoundLine .AND. .NOT. AllowDefault_: aviFAIL = -1,
#                   a message, RETURN -- no record
#   default-warned  .NOT. FoundLine .AND. AllowDefault_: Variable = 'unused'
#                   and the PRINT -- a record, and no scalar written
#   read-ok         FoundLine: the `'(A)'` READ, which always succeeds
#   read-failed     FoundLine and IOSTAT /= 0 -- DEAD IN THE PROGRAM. It is
#                   still a row in the classifier, and its count is the
#                   measurement rather than the absence of one.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
D=translations/ROSCO_Helpers/parseinput_str_opt_test
T=$D/parseinput_str_opt_test.cpp
OUT=evidence/ParseInput_Str_Opt/harness_partition.txt
KEEP=$(mktemp); cp "$T" "$KEEP"
restore() { cp "$KEEP" "$T"; rm -f "$KEEP"; }
trap restore EXIT

python3 - "$T" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()

snap = "        vit_record.begin();\n        ParseInput_Str_Opt("
assert s.count(snap) == 1, f"snapshot anchor appears {s.count(snap)} times"
s = s.replace(snap,
              "        const int vit_sup_avi = ErrVar_a.aviFAIL;\n"
              "        const std::string vit_sup_var(Variable_a.data(), Variable_a.size());\n"
              + snap)

anchor = "        int bad = 0;\n"
assert s.count(anchor) == 1, f"anchor appears {s.count(anchor)} times"
probe = (
    '        const std::string vit_out_var(Variable_b.data(), Variable_b.size());\n'
    '        fprintf(stderr, "PART %d %d %d %d %d %d %d %d %.200s\\n", c,\n'
    '                vit_sup_avi, (int)ErrVar_b.aviFAIL, (int)ErrVar_b.ErrStat,\n'
    '                (int)ErrVar_b.size_avcMSG, (int)(vit_rec_b.empty() ? 0 : 1),\n'
    '                (int)(vit_out_var == vit_sup_var ? 0 : 1),\n'
    '                (int)len_Variable_a,\n'
    '                ErrVar_ErrMsg_b.data());\n')
p.write_text(s.replace(anchor, probe + anchor))
print("probe inserted")
PY

docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/$D && rm -f parseinput_str_opt_test.o test && \
     make test > /dev/null 2>&1 && ./test parseinput_str_opt_cases.bin > /dev/null" \
    2> /tmp/parseinput_str_partition_raw.txt || true

python3 - /tmp/parseinput_str_partition_raw.txt "$OUT" <<'PY'
import collections, pathlib, sys
rows = []
for line in pathlib.Path(sys.argv[1]).read_text(errors="replace").splitlines():
    if not line.startswith("PART "):
        continue
    f = line.split(" ", 9)
    c, sup, avi, est, sz, rec, varmoved, lenvar = (int(x) for x in f[1:9])
    msg = f[9] if len(f) > 9 else ""
    rows.append((c, sup, avi, est, sz, rec, varmoved, lenvar, msg))

NOTALLOWED = "ParseInput_Str_Opt:Missing or default values are not allowed for "
READFAIL   = " >> A fatal error occurred when parsing data from \""

def arm(sup, avi, rec, msg):
    if sup < 0:
        return "pre-failed"
    m = msg.strip()
    if m.startswith(NOTALLOWED):
        return "not-allowed"
    if READFAIL in msg:
        return "read-failed"
    # R13's short-capacity cases: aviFAIL = -1 and the message the case ARRIVED
    # with, because the staging buffer refused the new one. On the siblings two
    # arms could land here; on THIS unit only `not-allowed` ever assigns a
    # message, so the cell is that arm and nothing else. Named separately all
    # the same, because the classifier reads the message and this is the row
    # where it cannot.
    if avi < 0:
        return "not-allowed, msg refused"
    if rec:
        return "default-warned"
    return "read-ok"

cnt = collections.Counter()
avi_changed = collections.Counter()
rec_written = collections.Counter()
var_moved = collections.Counter()
len_short = collections.Counter()   # LEN(Variable) < 6: the default TRUNCATES
for c, sup, avi, est, sz, rec, vm, lenvar, msg in rows:
    a = arm(sup, avi, rec, msg)
    cnt[a] += 1
    if avi != sup or est != 0 or sz != 0:
        avi_changed[a] += 1
    rec_written[a] += rec
    var_moved[a] += vm
    if lenvar < 6:
        len_short[a] += 1

out = []
out.append(f"harness partition -- {len(rows)} case(s), "
           f"corpus translations/ROSCO_Helpers/parseinput_str_opt_test/"
           f"parseinput_str_opt_cases.bin")
out.append("")
out.append(f"{'arm':28s} {'cases':>7s} {'scalar-changed':>15s} {'record':>8s} "
           f"{'Variable moved':>15s} {'LEN(Variable) < 6':>18s}")
for a in sorted(cnt, key=lambda k: -cnt[k]):
    out.append(f"{a:28s} {cnt[a]:7d} {avi_changed[a]:15d} {rec_written[a]:8d} "
               f"{var_moved[a]:15d} {len_short[a]:18d}")
out.append(f"{'TOTAL':28s} {sum(cnt.values()):7d} {sum(avi_changed.values()):15d} "
           f"{sum(rec_written.values()):8d} {sum(var_moved.values()):15d} "
           f"{sum(len_short.values()):18d}")
out.append("")
out.append("PREDICTIONS READ OFF THIS TABLE, each stated before its run:")
out.append(f"  post-integration wrapper red test (the reverse copy deleted):")
out.append(f"      the `scalar-changed` column summed = {sum(avi_changed.values())} must FAIL")
out.append(f"      the rest, {len(rows) - sum(avi_changed.values())}, write no scalar and must PASS")
out.append(f"  the PRINT deleted:  the `record` column summed = "
           f"{sum(rec_written.values())} must FAIL")
out.append(f"  the `.NOT. AllowDefault_` arm stubbed out: the two `not-allowed`")
out.append(f"      rows summed = "
           f"{cnt['not-allowed'] + cnt['not-allowed, msg refused']} must FAIL")
out.append(f"  the READ deleted:   the `read-ok` row's `Variable moved` count = "
           f"{var_moved['read-ok']} must FAIL")
out.append("")
out.append("AND THE TWO COLUMNS THAT ARE MEASUREMENTS RATHER THAN PREDICTIONS:")
out.append("  `read-failed` is DEAD IN THE PROGRAM -- an `A` edit descriptor")
out.append("  whose width equals the item length, over an internal record that")
out.append("  is blank-padded when short, has no failure mode. Its count here")
out.append("  is a second instrument agreeing with record_form_probe.txt's")
out.append("  144 of 144 IOSTAT zeros, on the corpus rather than on a form list.")
out.append("")
out.append("  `LEN(Variable) < 6` counts the cases where `Variable = 'unused'`")
out.append("  TRUNCATES, which is the ONLY input this unit's PRINT record has:")
out.append("  every other case writes `...default value of unused`. A corpus")
out.append("  with a zero in this column would make the record a constant and")
out.append("  every mutant of the value item equivalent.")
pathlib.Path(sys.argv[2]).write_text("\n".join(out) + "\n")
print("\n".join(out))
PY
