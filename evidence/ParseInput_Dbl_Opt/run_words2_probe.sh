#!/bin/bash
# Re-run the record search for the survivors it reported NONE, with a DIFFERENT
# `Words(2)`.
#
#   bash evidence/ParseInput_Dbl_Opt/run_words2_probe.sh
#
# WHY. `survivor_record_search.txt` reported six survivors as `NONE -- no record
# in this space distinguishes it`, and five of the six are
# `p < len` -> `p <= len`. At `p == len` those read `rec[200]`, which is
# `Words(2)(1:1)` -- the first character of the parameter name `FindLine`
# matched -- and the search puts `PC_KP` there. 'P' is not a digit, not a '.',
# not a sign and not an e/d/q, so all five guards take the same branch and the
# search is blind to them BY ITS OWN CHOICE OF THAT NAME.
#
# `Words(2)` is not a constant of the program: it is whatever `VarName` was, and
# in this unit's corpus `VarName` is an arbitrary CHARACTER(*). A Fortran
# parameter name cannot begin with a digit; a harness `VarName` can. So this
# probe re-runs exactly those six ids with `VIT_WORDS2 = "7E+9"` -- a digit, an
# exponent letter, a sign, and a digit, which is one character for each of the
# five guards -- and reports which of them the record space then distinguishes.
#
# WHAT IT SETTLES. A survivor that differs under SOME admissible `Words(2)` is a
# CORPUS GAP and must not be declared equivalent. One that differs under none is
# still only bounded-by-the-space evidence. Nothing here is folded into the
# score and nothing here is a declaration.
#
# THE DEFAULT IS UNCHANGED, so `survivor_record_search.txt` is still the run
# this file's own `#ifndef VIT_WORDS2` produces with no -D.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
OUT=evidence/ParseInput_Dbl_Opt/words2_probe.txt

python3 - "$ROOT" "$OUT" <<'PY'
import json, os, pathlib, subprocess, sys
root, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, '/Users/kmenear/Artifacts/vit_translation/translation-loop')
from harness.cppmutate import mutants

CPP = os.path.join(root, 'translations/ROSCO_Helpers/parseinput_dbl_opt.cpp')
src = pathlib.Path(CPP).read_text()
ms = {m.mid: m for m in mutants('parseinput_dbl_opt', src)}

# THE SET IS READ OUT OF THE FIRST SEARCH'S ARTIFACT, not typed here: a probe
# whose subject list is a literal goes stale the moment the sweep moves.
txt = pathlib.Path(os.path.join(
    root, 'evidence/ParseInput_Dbl_Opt/survivor_record_search.txt')).read_text()
none_ids, cur = [], None
for line in txt.splitlines():
    if len(line) >= 8 and line[:8] in ms and line[8:9] == ' ':
        cur = line[:8]
    elif cur and line.strip().startswith('NONE'):
        none_ids.append(cur); cur = None
none_ids = sorted(set(none_ids))
if not none_ids:
    print("no NONE rows parsed out of survivor_record_search.txt", file=sys.stderr)
    sys.exit(2)

WORDS2 = '7E+9'
os.makedirs(os.path.join(root, '.words2_probe'), exist_ok=True)

def run(container_path, tag):
    cmd = (f"cd /workspace/ROSCO-r2/.words2_probe && "
           f"g++ -O0 -ffp-contract=off -std=c++17 -fsanitize=address,undefined "
           f"-I/workspace/ROSCO-r2/translations/ROSCO_Helpers/parseinput_dbl_opt_test "
           f"-DVIT_TRANSLATION='\"{container_path}\"' "
           f"-DVIT_WORDS2='\"{WORDS2}\"' "
           f"/workspace/ROSCO-r2/evidence/ParseInput_Dbl_Opt/survivor_record_search.cpp "
           f"-o w_{tag} && ./w_{tag}")
    r = subprocess.run(['docker', 'exec', 'vit-dev', 'bash', '-lc', cmd],
                       capture_output=True, text=True)
    return r.returncode, r.stdout, r.stderr

rc, base, err = run('/workspace/ROSCO-r2/translations/ROSCO_Helpers/parseinput_dbl_opt.cpp', 'base')
report = []
if rc != 0:
    report += ["BASELINE BUILD OR RUN FAILED -- refusing to probe:",
               base[-2000:], err[-2000:]]
    pathlib.Path(out).write_text("\n".join(report) + "\n")
    print("\n".join(report[-8:])); sys.exit(1)
baselines = base.splitlines()
report.append("ParseInput_Dbl_Opt -- the six `NONE` survivors re-run with a")
report.append(f"DIFFERENT Words(2): VIT_WORDS2 = {WORDS2!r} instead of the default 'PC_KP'.")
report.append("")
report.append("Words(2) is the parameter name FindLine matched, and it is the byte a")
report.append("`p <= len` guard reads at p == len. A Fortran parameter name cannot begin")
report.append("with a digit; a harness VarName can, and this unit's corpus varies it")
report.append("freely. Nothing here is folded into the score and nothing is a declaration.")
report.append("")
report.append(f"RECORD SPACE: {len(baselines)} records, the same space as")
report.append("survivor_record_search.txt.")
report.append("")
for mid in none_ids:
    m = ms[mid]
    mut = src[:m.pos] + m.after + src[m.pos + len(m.before):]
    pp = os.path.join(root, '.words2_probe', f'm_{mid}.cpp')
    pathlib.Path(pp).write_text(mut)
    rc, got, err2 = run(f'/workspace/ROSCO-r2/.words2_probe/m_{mid}.cpp', mid)
    line = f"{mid}  {m.operator:14s} L{m.line:4d}  {m.before!r} -> {m.after!r}"
    if rc != 0:
        report.append(line); report.append("    BUILD OR RUN FAILED / SANITISER ABORT")
        report.append("      " + (err2.strip().splitlines() or [''])[-1][:160])
        continue
    g = got.splitlines()
    diffs = [i for i in range(max(len(baselines), len(g)))
             if (baselines[i] if i < len(baselines) else None)
             != (g[i] if i < len(g) else None)]
    report.append(line)
    if diffs:
        i = diffs[0]
        report.append(f"    DIFFERS on {len(diffs)} record(s). First:")
        report.append(f"      {baselines[i]}   vs   {g[i] if i < len(g) else '<missing>'}")
    else:
        report.append("    STILL NONE under this Words(2) either")
pathlib.Path(out).write_text("\n".join(report) + "\n")
print("\n".join(report))
PY
