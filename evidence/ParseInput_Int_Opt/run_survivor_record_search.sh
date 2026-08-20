#!/bin/bash
# For each SURVIVING mutant outside the PRINT region, search a record space for
# one record on which it differs from the shipped translation.
#
#   bash evidence/ParseInput_Int_Opt/run_survivor_record_search.sh
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
# PRINT-region mutants are skipped BY POSITION, not by name: this instrument
# drives `list_read_ints`, which the PRINT record is not reached from, so a
# `none` for one of them would say only that it was never executed. This unit
# has no PRINT replay yet -- the sibling's `run_print_conformance.sh` is the
# shape, and the five PRINT survivors are named as the gap in
# `evidence/ParseInput_Int_Opt/mutation_survivors.txt` rather than measured here.
#
# THE BUILD CARRIES -fsanitize=address,undefined, which the sibling's does not.
# Ten of the twenty-six survivors differ from the shipped translation ONLY at
# `rec[2048]`, one past the end of the record's `std::vector<char>`, and on this
# allocator that byte is usually a blank -- so the two programs compute the same
# answer and a value comparison sees nothing. The BASELINE running clean under
# the same flags is the control on that: a shipped translation that reported
# would make every mutant "differ" by inheriting it.
#
# It never writes to the tree under test: every mutant is applied to a COPY in a
# scratch directory and the shipped .cpp is only ever read.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
OUT=evidence/ParseInput_Int_Opt/survivor_record_search.txt

python3 - "$ROOT" "$OUT" <<'PY'
import json, os, pathlib, subprocess, sys

root, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, '/Users/kmenear/Artifacts/vit_translation/translation-loop')
from harness.cppmutate import mutants

CPP = os.path.join(root, 'translations/ROSCO_Helpers/parseinput_int_opt.cpp')
src = pathlib.Path(CPP).read_text()
ms = {m.mid: m for m in mutants('parseinput_int_opt', src)}
surv = [s['id'] for s in
        json.load(open(os.path.join(root, 'evidence/ParseInput_Int_Opt/sweep_probe0.json')))['survivors']]

# The PRINT region, taken from the source rather than from a name list -- the
# same range `run_print_conformance.sh` computes.
lines = src.splitlines()
lo = next(i for i, l in enumerate(lines, 1) if l.startswith('std::string field('))
hi = next(i for i, l in enumerate(lines, 1) if 'void print_default_warning(' in l)
hi = next(i for i, l in enumerate(lines[hi:], hi + 1) if l.rstrip() == '}')

os.makedirs(os.path.join(root, '.record_search'), exist_ok=True)

# THE BUILD AND THE RUN ARE SEPARATE STEPS, AND THAT IS THE REPAIR (C12).
#
# The inherited version ran `g++ ... && ./s_<tag>` in one shell and treated a
# non-zero status as "DID NOT BUILD OR RUN". With no sanitiser in the build
# those really are the same case. With one they are opposites: a mutant that
# ABORTS under AddressSanitizer has been distinguished by a record, in the
# strongest way this instrument can distinguish anything, and eleven of this
# unit's survivors came back reported as "no record distinguishes it" for
# exactly that reason. The wrong artifact is kept at
# `evidence/ParseInput_Int_Opt/survivor_record_search.MISLABELLED.txt`.
def build(container_path, tag):
    cmd = (f"cd /workspace/ROSCO-r2/.record_search && "
           f"g++ -O0 -ffp-contract=off -std=c++17 -fsanitize=address,undefined "
           f"-I/workspace/ROSCO-r2/rosco/controller/src "
           f"-DVIT_TRANSLATION='\"{container_path}\"' "
           f"/workspace/ROSCO-r2/evidence/ParseInput_Int_Opt/survivor_record_search.cpp "
           f"-o s_{tag} -lgfortran")
    r = subprocess.run(['docker', 'exec', 'vit-dev', 'bash', '-lc', cmd],
                       capture_output=True, text=True)
    return r.returncode, (r.stdout + r.stderr)

def execute(tag):
    cmd = f"cd /workspace/ROSCO-r2/.record_search && ./s_{tag}"
    r = subprocess.run(['docker', 'exec', 'vit-dev', 'bash', '-lc', cmd],
                       capture_output=True, text=True)
    return r.returncode, r.stdout, r.stderr

def run(container_path, tag):
    rc, log = build(container_path, tag)
    if rc != 0:
        return 'nocompile', '', log
    rc, out, err = execute(tag)
    if rc != 0:
        return 'aborted', out, err
    return 'ok', out, err

status, base, err = run('/workspace/ROSCO-r2/translations/ROSCO_Helpers/parseinput_int_opt.cpp',
                        'base')
report = []
if status != 'ok':
    report.append("BASELINE BUILD OR RUN FAILED -- refusing to search:")
    report.append(base[-3000:]); report.append(err[-2000:])
    pathlib.Path(out).write_text("\n".join(report) + "\n")
    print("\n".join(report[-8:])); sys.exit(1)
baselines = base.splitlines()
report.append("ParseInput_Int_Opt -- a record that distinguishes each surviving mutant")
report.append("from the SHIPPED translation, searched rather than argued.")
report.append("Built with -fsanitize=address,undefined; the BASELINE ran clean under")
report.append("those flags. The sanitiser is a GUARD here rather than the instrument:")
report.append("byte 201 of Words is Words(2)(1:1), so a read one past the record is")
report.append("inside the allocation and the difference it makes is a VALUE.")
report.append("")
report.append(f"RECORD SPACE: {len(baselines)} records. Every string of length 1..3 over")
report.append("  \"0123456789+-.eEdDqQnNaAiIfFtTyYxX ,;/*()\" at the HEAD of a 200-byte")
report.append("  blank-padded record and again at its TAIL, so the token's last character")
report.append("  is the record's last byte; plus 32 crafted full-width records. ONE")
report.append("  element is read -- the reference transfers one -- pre-set to the")
report.append("  -987.654 sentinel and printed as bits, so a stored 0.0 is")
report.append("  distinguishable from an untouched slot. The record lives in the first")
report.append("  200 bytes of a 400-byte object whose second half holds a parameter")
report.append("  name, which is what Words(1) and Words(2) are.")
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
    status, got, err = run(f'/workspace/ROSCO-r2/.record_search/m_{mid}.cpp', mid)
    if status == 'nocompile':
        rows.append((mid, m, 'nocompile', 0, err.strip().splitlines()[-1] if err.strip() else ''))
        continue
    got_lines = got.splitlines()
    diffs = [i for i in range(max(len(baselines), len(got_lines)))
             if (baselines[i] if i < len(baselines) else None)
             != (got_lines[i] if i < len(got_lines) else None)]
    if status == 'aborted':
        # THE RECORD IT DIED ON is the first one the baseline printed and this
        # did not: the sanitiser aborts the process, so output stops there.
        i = len(got_lines)
        rec = baselines[i] if i < len(baselines) else '<past the end of the space>'
        san = next((l for l in err.splitlines() if 'ERROR:' in l), err.strip()[:160])
        rows.append((mid, m, 'aborted', len(diffs),
                     f"{rec}   ->   {san.strip()}"))
        continue
    if diffs:
        i = diffs[0]
        rows.append((mid, m, 'differs', len(diffs),
                     f"{baselines[i]}   vs   {got_lines[i] if i < len(got_lines) else '<missing>'}"))
    else:
        rows.append((mid, m, 'none', 0, ''))

report.append(f"{sum(1 for r in rows if r[2] in ('differs', 'aborted'))} of {len(rows)} "
              f"non-PRINT survivors are distinguished by some record in this space "
              f"({sum(1 for r in rows if r[2] == 'differs')} by a VALUE, "
              f"{sum(1 for r in rows if r[2] == 'aborted')} by an ADDRESS -- the "
              f"sanitiser aborting on a read the shipped translation does not make).")
report.append("")
for mid, m, verdict, n, detail in rows:
    report.append(f"{mid}  {m.operator:14s} L{m.line:4d}  {m.before!r} -> {m.after!r}")
    if verdict == 'differs':
        report.append(f"    DIFFERS on {n} record(s). First:")
        report.append(f"      {detail}")
    elif verdict == 'aborted':
        report.append(f"    SANITISER ABORT -- distinguished by an ADDRESS, not by a value.")
        report.append(f"    The record it died on, and what it did:")
        report.append(f"      {detail}")
    elif verdict == 'nocompile':
        report.append(f"    DID NOT COMPILE -- out of both sides of the score already")
        report.append(f"      {detail}")
    else:
        report.append(f"    NONE -- no record in this space distinguishes it")
pathlib.Path(out).write_text("\n".join(report) + "\n")
print("\n".join(report))
PY
