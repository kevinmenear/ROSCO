#!/bin/bash
# The X3 question for the whole-array-copy hop added to `order_arrays_from`
# (scripts/vit_harness.py, rosco-r2 unit #26).
#
# A corpus addition can only ADD cases, so it cannot invalidate a committed
# artifact's claim about what IT checked -- but it CAN change the case count and
# the score of a unit that is re-run, so which units it touches is a number
# rather than an argument.
#
# Read against the CLEAN baseline source (54dd134), NOT the working tree. The
# tree is integrated: 21 of the 25 scored units have a wrapper for a body, and
# a wrapper contains no subscript at all -- so a sweep over the working tree
# would report `old=[] new=[]` for every one of them and look like a proof
# when it is an artefact of where it read.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
rm -rf /tmp/unwrap_x3_clean && mkdir -p /tmp/unwrap_x3_clean
git archive 54dd134 rosco/controller/src | tar -x -C /tmp/unwrap_x3_clean
cp -R /tmp/unwrap_x3_clean "$ROOT/.x3_clean_src"
# The PREVIOUS `order_arrays_from`, from the loop repo's own history, staged
# where the container can read it. Nothing is left behind in either repo.
git -C "$(cd "$ROOT/../translation-loop" && pwd)" show HEAD:scripts/vit_harness.py \
    > "$ROOT/.x3_clean_src/vit_harness_old.py"
trap 'rm -rf "$ROOT/.x3_clean_src"' EXIT

docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && python3 - <<'PY'
import sys, pathlib, importlib.util
sys.path.insert(0,'/workspace/translation-loop'); sys.path.insert(0,'/workspace/translation-loop/scripts')
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s)
    s.loader.exec_module(m); return m
new = load('/workspace/translation-loop/scripts/vit_harness.py','vh_new')
old = load('.x3_clean_src/vit_harness_old.py','vh_old')
from vit.fortran_parser import parse_signature_from_source

SRC = pathlib.Path('.x3_clean_src/rosco/controller/src')
files = sorted(list(SRC.glob('*.f90')) + list(SRC.glob('*.F90')))
units = sorted({p.stem.split('.')[0] for p in pathlib.Path('harness').glob('*.json')})
print(f'source read from 54dd134 (clean, pre-integration): {SRC}')
print()
changed = []
for u in units:
    done = False
    for f in files:
        text = f.read_text(errors='ignore')
        try: sig = parse_signature_from_source(text, u)
        except Exception: sig = None
        if sig is None: continue
        try: body = new.unit_body(f, u)
        except Exception: break
        params = {a.name for a in sig.args if getattr(a,'is_array',False)}
        if not params:
            print(f'  {u:24s} no array argument'); done=True; break
        o = sorted(old.order_arrays_from(body, set(params)))
        n = sorted(new.order_arrays_from(body, set(params)))
        mark = '' if o == n else '   <-- CHANGED'
        if o != n: changed.append(u)
        print(f'  {u:24s} old={str(o):22s} new={str(n):22s}{mark}')
        done=True; break
    if not done:
        print(f'  {u:24s} NOT FOUND in the clean tree')
print()
print(f'units with a committed harness artifact whose order-sensitive set MOVES: {len(changed)} {changed}')
PY"
