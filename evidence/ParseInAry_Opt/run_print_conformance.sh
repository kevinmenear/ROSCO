#!/bin/bash
# Compare the reference's OWN default-warning record with the SHIPPED
# translation's, case for case -- and then ask which of the surviving PRINT
# mutants that comparison would kill.
#
#   bash evidence/ParseInAry_Opt/run_print_conformance.sh
#
# COPIED from `evidence/ParseDbAry_Opt/run_print_conformance.sh` (P4) with the
# unit's names changed and nothing else. The region detection, the
# mutant-from-a-copy loop, the kill rule and the negative control are all that
# file's.
#
# THIS IS A SECOND INSTRUMENT AND ITS RESULT IS FOLDED INTO NOTHING. A kill by a
# different instrument is not a kill by the sweep, and
# mutation/ParseInAry_Opt.json is not touched by this script. What it buys is
# that "three survivors nothing compares" becomes a measurement. Same rule this
# unit recorded for `parser_conformance`.
#
# It never writes to the tree under test: the mutants are applied to COPIES in a
# scratch directory and the shipped .cpp is only ever read.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
HERE=evidence/ParseInAry_Opt
TESTDIR=translations/ROSCO_Helpers/parseinary_opt_test
WORK=/workspace/ROSCO-r2
OUT=$HERE/print_replay.txt

# 1. The reference side, once.
docker exec vit-dev bash -lc "
set -e
mkdir -p $WORK/.print_conf && cd $WORK/.print_conf
gfortran -O2 -ffree-line-length-none -fdefault-real-8 -fdefault-double-8 \
         -ffp-contract=off -o print_conformance $WORK/$HERE/print_conformance.f90
./print_conformance
wc -l < print_conformance.ref
"

# 2. The translation side, and every mutant of the PRINT region, from copies.
python3 - "$ROOT" "$OUT" <<'PY'
import json, os, subprocess, sys, pathlib, re
root, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, '/Users/kmenear/Artifacts/vit_translation/translation-loop')
from harness.cppmutate import mutants

CPP = os.path.join(root, 'translations/ROSCO_Helpers/parseinary_opt.cpp')
src = pathlib.Path(CPP).read_text()
ms = {m.mid: m for m in mutants('parseinary_opt', src)}
surv = [s['id'] for s in
        json.load(open(os.path.join(root, 'mutation/ParseInAry_Opt.json')))['survivors']]

# THE REGION IS TAKEN FROM THE MUTANT'S OWN POSITION, not from a name list: the
# PRINT region is `field`, `nonfinite_text`, `list_directed_real` and
# `print_default_warning`, which is the line range from `field`'s definition to
# the end of `print_default_warning`.
lines = src.splitlines()
lo = next(i for i, l in enumerate(lines, 1) if l.startswith('std::string field('))
hi = next(i for i, l in enumerate(lines, 1) if 'void print_default_warning(' in l)
hi = next(i for i, l in enumerate(lines[hi:], hi + 1) if l.rstrip() == '}')
print(f"PRINT region: lines {lo}..{hi}")

def run(container_path, tag):
    """Build the replay against `container_path` and return its stdout."""
    cmd = (f"cd /workspace/ROSCO-r2/.print_conf && "
           f"g++ -O0 -ffp-contract=off "
           f"-I/workspace/ROSCO-r2/translations/ROSCO_Helpers/parseinary_opt_test "
           f"-DVIT_TRANSLATION='\"{container_path}\"' "
           f"/workspace/ROSCO-r2/evidence/ParseInAry_Opt/print_conformance.cpp "
           f"-o replay_{tag} -lgfortran 2>&1 && ./replay_{tag} print_conformance.bin")
    r = subprocess.run(['docker', 'exec', 'vit-dev', 'bash', '-lc', cmd],
                       capture_output=True, text=True)
    return r.returncode, r.stdout, r.stderr

def records(text):
    return text.splitlines()

ref = subprocess.run(['docker', 'exec', 'vit-dev', 'bash', '-lc',
                      'cat /workspace/ROSCO-r2/.print_conf/print_conformance.ref'],
                     capture_output=True, text=True).stdout
refrecs = records(ref)
report = []
report.append(f"{len(refrecs)} reference record(s) from print_conformance.f90")

rc, got, err = run('/workspace/ROSCO-r2/translations/ROSCO_Helpers/parseinary_opt.cpp',
                   'clean')
if rc != 0:
    report.append("BASELINE BUILD OR RUN FAILED -- refusing to score:")
    report.append(got[-3000:]); report.append(err[-2000:])
    pathlib.Path(out).write_text("\n".join(report) + "\n")
    print("\n".join(report[-8:])); sys.exit(1)
gotrecs = records(got)
diff = [i for i in range(max(len(refrecs), len(gotrecs)))
        if (refrecs[i] if i < len(refrecs) else None)
        != (gotrecs[i] if i < len(gotrecs) else None)]
report.append(f"CONFORMANCE -- the UNMUTATED translation against the reference:")
report.append(f"BASELINE: {len(gotrecs)} replayed record(s), {len(diff)} mismatched")
for i in diff[:12]:
    report.append(f"  case {i}")
    report.append(f"    ref |{refrecs[i] if i < len(refrecs) else '<missing>'}|")
    report.append(f"    got |{gotrecs[i] if i < len(gotrecs) else '<missing>'}|")

killed, alive, region = [], [], []
for mid in sorted(surv):
    m = ms[mid]
    if not (lo <= m.line <= hi):
        continue
    region.append(mid)
    mut = src[:m.pos] + m.after + src[m.pos + len(m.before):]
    p = os.path.join(root, '.print_conf', f'mutant_{mid}.cpp')
    os.makedirs(os.path.dirname(p), exist_ok=True)
    pathlib.Path(p).write_text(mut)
    rc, mgot, merr = run(f'/workspace/ROSCO-r2/.print_conf/mutant_{mid}.cpp', mid)
    if rc != 0:
        killed.append((mid, 'did not build or did not run'))
        continue
    mrecs = records(mgot)
    # A KILL IS "THE MUTANT'S OWN OUTPUT DIFFERS FROM THE UNMUTATED
    # TRANSLATION'S", not "differs from the reference". The baseline itself
    # deviates from the reference on one record (the n = 0 case, below), and
    # scoring against the reference makes every mutant inherit that deviation --
    # which is exactly what happened the first time this ran, and it turned the
    # negative control green-looking at 23 of 23.
    n = sum(1 for i in range(max(len(mrecs), len(gotrecs)))
            if (gotrecs[i] if i < len(gotrecs) else None)
            != (mrecs[i] if i < len(mrecs) else None))
    (killed if n else alive).append((mid, f"{n} of {len(gotrecs)} record(s) differ"))

report.append("")
report.append(f"PRINT-region survivors replayed: {len(region)}")
report.append(f"  KILLED by the record comparison: {len(killed)}")
for mid, why in killed:
    report.append(f"    {mid}  {ms[mid].operator:14s} L{ms[mid].line}  "
                  f"{ms[mid].before!r} -> {ms[mid].after!r}   {why}")
report.append(f"  STILL ALIVE: {len(alive)}")
for mid, why in alive:
    report.append(f"    {mid}  {ms[mid].operator:14s} L{ms[mid].line}  "
                  f"{ms[mid].before!r} -> {ms[mid].after!r}   {why}")

# NEGATIVE CONTROL, and it is built into the instrument's shape rather than
# chosen: the replay enters the translation ONLY through print_default_warning,
# so every survivor OUTSIDE the PRINT region must come back unreached.
outside = [mid for mid in sorted(surv) if not (lo <= ms[mid].line <= hi)]
ctrl_killed = []
for mid in outside:
    m = ms[mid]
    mut = src[:m.pos] + m.after + src[m.pos + len(m.before):]
    p = os.path.join(root, '.print_conf', f'ctrl_{mid}.cpp')
    pathlib.Path(p).write_text(mut)
    rc, cgot, cerr = run(f'/workspace/ROSCO-r2/.print_conf/ctrl_{mid}.cpp', f'c{mid}')
    if rc != 0:
        ctrl_killed.append((mid, 'build/run failed'))
        continue
    crecs = records(cgot)
    n = sum(1 for i in range(max(len(crecs), len(gotrecs)))
            if (gotrecs[i] if i < len(gotrecs) else None)
            != (crecs[i] if i < len(crecs) else None))
    if n:
        ctrl_killed.append((mid, f"{n} record(s) differ"))
report.append("")
report.append(f"NEGATIVE CONTROL: {len(outside)} survivor(s) OUTSIDE the PRINT region, "
              f"{len(ctrl_killed)} of them moved a record")
for mid, why in ctrl_killed:
    report.append(f"    {mid}  {ms[mid].operator:14s} L{ms[mid].line}  {why}")
pathlib.Path(out).write_text("\n".join(report) + "\n")
print("\n".join(report))
PY
