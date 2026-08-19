#!/bin/bash
# For each SURVIVING mutant outside the PRINT region, search a record space for
# one record on which it differs from the shipped translation.
#
#   bash evidence/ParseDbAry_Opt/run_survivor_record_search.sh
#
# THE QUESTION IT SETTLES. A survivor is equivalent, or unreached by the corpus,
# or a blind spot, and the sweep's answer for all three is the same word. This
# asks the two PROGRAMS directly, over a record space that is printed rather than
# implied. A differing record names the corpus lever and ends the argument; no
# differing record is evidence toward equivalence, bounded by the space.
#
# IT IS NOT A DECLARATION AND NOTHING IS FOLDED INTO THE SCORE. It compares the
# translation against itself-mutated, so it says nothing about whether either
# agrees with the Fortran reference -- that is the differential harness's job.
#
# PRINT-region mutants are skipped BY POSITION, not by name: the replay
# (`run_print_conformance.sh`) is their instrument and this one drives
# `list_read_reals`, which the PRINT record is not reached from.
#
# It never writes to the tree under test: every mutant is applied to a COPY in a
# scratch directory and the shipped .cpp is only ever read.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
OUT=evidence/ParseDbAry_Opt/survivor_record_search.txt

python3 - "$ROOT" "$OUT" <<'PY'
import json, os, pathlib, subprocess, sys

root, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, '/Users/kmenear/Artifacts/vit_translation/translation-loop')
from harness.cppmutate import mutants

CPP = os.path.join(root, 'translations/ROSCO_Helpers/parsedbary_opt.cpp')
src = pathlib.Path(CPP).read_text()
ms = {m.mid: m for m in mutants('parsedbary_opt', src)}
surv = [s['id'] for s in
        json.load(open(os.path.join(root, 'mutation/ParseDbAry_Opt.json')))['survivors']]

# The PRINT region, taken from the source rather than from a name list -- the
# same range `run_print_conformance.sh` computes.
lines = src.splitlines()
lo = next(i for i, l in enumerate(lines, 1) if l.startswith('std::string field('))
hi = next(i for i, l in enumerate(lines, 1) if 'void print_default_warning(' in l)
hi = next(i for i, l in enumerate(lines[hi:], hi + 1) if l.rstrip() == '}')

os.makedirs(os.path.join(root, '.record_search'), exist_ok=True)

def run(container_path, tag):
    cmd = (f"cd /workspace/ROSCO-r2/.record_search && "
           f"g++ -O0 -ffp-contract=off -std=c++17 "
           f"-I/workspace/ROSCO-r2/translations/ROSCO_Helpers/parsedbary_opt_test "
           f"-DVIT_TRANSLATION='\"{container_path}\"' "
           f"/workspace/ROSCO-r2/evidence/ParseDbAry_Opt/survivor_record_search.cpp "
           f"-o s_{tag} -lgfortran 2>&1 && ./s_{tag}")
    r = subprocess.run(['docker', 'exec', 'vit-dev', 'bash', '-lc', cmd],
                       capture_output=True, text=True)
    return r.returncode, r.stdout, r.stderr

rc, base, err = run('/workspace/ROSCO-r2/translations/ROSCO_Helpers/parsedbary_opt.cpp',
                    'base')
report = []
if rc != 0:
    report.append("BASELINE BUILD OR RUN FAILED -- refusing to search:")
    report.append(base[-3000:]); report.append(err[-2000:])
    pathlib.Path(out).write_text("\n".join(report) + "\n")
    print("\n".join(report[-8:])); sys.exit(1)
baselines = base.splitlines()
report.append("ParseDbAry_Opt -- a record that distinguishes each surviving mutant")
report.append("from the SHIPPED translation, searched rather than argued.")
report.append("")
report.append(f"RECORD SPACE: {len(baselines)} records. Every string of length 1..3 over")
report.append("  \"0123456789+-.eEdDqQnNaAiIfFtTyYxX ,;/*()\" at the HEAD of a 2048-byte")
report.append("  blank-padded record and again at its TAIL, so the token's last character")
report.append("  is the record's last byte; plus 13 crafted full-width records. Three")
report.append("  elements are read, each pre-set to the -987.654 sentinel, so an element")
report.append("  the transfer never reached is visible as itself.")
report.append("")
report.append("WHAT A ROW MEANS. `differs` names the FIRST record on which the mutant's")
report.append("(iostat, element bits) triple differs from the shipped translation's --")
report.append("that record is the corpus lever and the mutant is NOT equivalent. `none`")
report.append("means no record IN THIS SPACE distinguishes it, which is evidence toward")
report.append("equivalence bounded by the space and is not a declaration.")
report.append("")

rows = []
for mid in sorted(surv):
    m = ms[mid]
    if lo <= m.line <= hi:
        continue
    mut = src[:m.pos] + m.after + src[m.pos + len(m.before):]
    p = os.path.join(root, '.record_search', f'm_{mid}.cpp')
    pathlib.Path(p).write_text(mut)
    rc, got, err = run(f'/workspace/ROSCO-r2/.record_search/m_{mid}.cpp', mid)
    if rc != 0:
        rows.append((mid, m, 'DID NOT BUILD OR RUN', 0, ''))
        continue
    got_lines = got.splitlines()
    diffs = [i for i in range(max(len(baselines), len(got_lines)))
             if (baselines[i] if i < len(baselines) else None)
             != (got_lines[i] if i < len(got_lines) else None)]
    if diffs:
        i = diffs[0]
        rows.append((mid, m, 'differs', len(diffs),
                     f"{baselines[i]}   vs   {got_lines[i] if i < len(got_lines) else '<missing>'}"))
    else:
        rows.append((mid, m, 'none', 0, ''))

report.append(f"{sum(1 for r in rows if r[2] == 'differs')} of {len(rows)} "
              f"non-PRINT survivors are distinguished by some record in this space.")
report.append("")
for mid, m, verdict, n, detail in rows:
    report.append(f"{mid}  {m.operator:14s} L{m.line:4d}  {m.before!r} -> {m.after!r}")
    if verdict == 'differs':
        report.append(f"    DIFFERS on {n} record(s). First:")
        report.append(f"      {detail}")
    else:
        report.append(f"    {verdict.upper()} -- no record in this space distinguishes it")
pathlib.Path(out).write_text("\n".join(report) + "\n")
print("\n".join(report))
PY
