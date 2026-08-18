#!/bin/bash
# Put EVERY surviving mutant through the OTHER instrument.
#
#   bash evidence/ParseInAry_Opt/run_survivor_replay.sh
#
# The RUNBOOK's rule (unit #53) is: before calling a mutation survivor a corpus
# gap, ask whether another instrument can reach it -- one run replaces the
# argument. Unit #54 asked it once per REGION, with two gate runs. This unit has
# a cheaper second instrument for its largest region, so it asks for EVERY
# survivor: `parser_conformance.cpp` replays 113 records taken from gfortran's
# own runtime through `list_read_ints`, and a build-and-replay is about a
# second.
#
# THE NEGATIVE CONTROL IS BUILT IN AND IS THE HALF THAT MAKES IT EVIDENCE. The
# replay enters the translation ONLY through the parser, so it cannot see the
# PRINT record, `assign_errmsg`, `ary_at` or the unit's own body -- those
# survivors MUST come back unreached. A run in which everything died would mean
# the replay was measuring something other than what it claims.
#
# Each mutant's source is `harness/cppmutate.py`'s OWN `.source`, not a text
# substitution: the same bytes vit_mutate.py compiled.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
python3 - <<'PY'
import json, sys, pathlib, subprocess, tempfile, collections
sys.path.insert(0, str(pathlib.Path.home()/'Artifacts/vit_translation/translation-loop'))
from harness.cppmutate import mutants
W = '/workspace/ROSCO-r2'
src = pathlib.Path('translations/ROSCO_Helpers/parseinary_opt.cpp').read_text()
ms = {m.mid: m for m in mutants('parseinary_opt', src)}
d = json.load(open('mutation/ParseInAry_Opt.json'))
surv = d['survivors']
assert all(s['id'] in ms for s in surv)
lines = src.split('\n')

def fn_span(sig):
    i = next(k for k,l in enumerate(lines) if l.startswith(sig))
    depth=0; started=False
    for k in range(i, len(lines)):
        depth += lines[k].count('{') - lines[k].count('}')
        if '{' in lines[k]: started=True
        if started and depth==0: return (i+1,k+1)
    raise AssertionError(sig)

READ = [fn_span(s) for s in ('bool is_blank(','bool is_eol(','bool is_digit(',
        'bool is_value_terminator(','bool parse_int(','int list_read_ints(')]
in_read = lambda ln: any(a<=ln<=b for a,b in READ)

def run_replay(container_dir):
    cmd = ("set -e; GCC_INC=$(dirname $(gfortran -print-file-name=include/ISO_Fortran_binding.h)); "
           f"g++ -std=c++17 -O2 -ffp-contract=off -I\"$GCC_INC\" -I{W}/rosco/controller/src "
           f"-I{container_dir} {W}/evidence/ParseInAry_Opt/parser_conformance.cpp "
           f"-o /tmp/sr_replay -lgfortran; /tmp/sr_replay {W}/evidence/ParseInAry_Opt/parser_conformance.txt")
    r = subprocess.run(['docker','exec','vit-dev','bash','-lc',cmd],
                       capture_output=True, text=True, errors='replace')
    for ln in r.stdout.splitlines():
        if ln.startswith('parser_conformance:'):
            return int(ln.split()[-2]), r.returncode
    return None, r.returncode

# positive control: the shipped translation must replay clean
subprocess.run(['docker','exec','vit-dev','bash','-lc',f'rm -rf /tmp/sr && mkdir -p /tmp/sr'],
               capture_output=True)
base, _ = run_replay(f'{W}/translations/ROSCO_Helpers')
print(f"control: the shipped translation replays {base} mismatched of 113 records")
assert base == 0, "the shipped translation does not replay clean -- refusing to score"

rows = []
tmp = pathlib.Path(tempfile.mkdtemp())
for i, s in enumerate(sorted(surv, key=lambda s: ms[s['id']].line), 1):
    m = ms[s['id']]
    f = tmp/'parseinary_opt.cpp'
    f.write_text(m.source)
    subprocess.run(['docker','exec','vit-dev','bash','-lc','rm -rf /tmp/sr && mkdir -p /tmp/sr'],
                   capture_output=True)
    subprocess.run(['docker','cp',str(f),'vit-dev:/tmp/sr/parseinary_opt.cpp'], capture_output=True)
    n, rc = run_replay('/tmp/sr')
    verdict = 'nocompile' if n is None else ('KILLED' if n > 0 else 'unreached')
    rows.append((s['id'], m.line, in_read(m.line), verdict, n, s['operator'], s['before'], s['after']))
    print(f"  [{i:3d}/{len(surv)}] {s['id']} line {m.line:4d} {'READ ' if in_read(m.line) else 'other'} "
          f"{verdict:<10s} {'' if n is None else str(n)+'/113'}  {s['operator']}: {s['before']!r} -> {s['after']!r}")

c = collections.Counter((r[2], r[3]) for r in rows)
print()
print("  region        verdict      count")
for k in sorted(c, key=lambda k: (not k[0], k[1])):
    print(f"  {'READ ' if k[0] else 'other'}         {k[1]:<12s} {c[k]}")
killed = [r for r in rows if r[3]=='KILLED']
print()
print(f"the parser replay KILLS {len(killed)} of {len(rows)} survivors, "
      f"{sum(1 for r in killed if r[2])} of them in the READ region and "
      f"{sum(1 for r in killed if not r[2])} outside it")
pathlib.Path('evidence/ParseInAry_Opt/survivor_replay.json').write_text(json.dumps(
    [{'id':r[0],'line':r[1],'in_read_region':r[2],'replay':r[3],'mismatched_records':r[4],
      'operator':r[5],'before':r[6],'after':r[7]} for r in rows], indent=1) + "\n")
PY
