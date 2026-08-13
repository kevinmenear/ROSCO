#!/bin/bash
# The X3 question for the `SIZE(<arr>)` form added to `_result_extents`
# (harness/vitbridge.py, rosco-r2 unit #26).
#
# Unit #24 established that a change to a shared instrument is TWO questions and
# only one of them is expensive: does it move what is already scored, and does it
# create new work. Here the first is answerable by enumeration, because the new
# branch is reached ONLY where the old code returned a refusal -- so the only
# units it can touch are those with an array-valued FUNCTION RESULT.
set -euo pipefail
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && python3 - <<'PY'
import sys, pathlib, yaml
sys.path.insert(0,'/workspace/translation-loop')
from vit.fortran_parser import parse_signature_from_source
files=sorted(list(pathlib.Path('rosco/controller/src').glob('*.f90'))
             +list(pathlib.Path('rosco/controller/src').glob('*.F90')))
units=sorted({p.stem.split('.')[0] for p in pathlib.Path('harness').glob('*.json')})
print('units with a committed harness artifact:', len(units))
hits=[]
for u in units:
    for f in files:
        try: sig=parse_signature_from_source(f.read_text(errors='ignore'), u)
        except Exception: continue
        if sig is None: continue
        if getattr(sig,'result_is_array',False):
            hits.append((u,f.name,getattr(sig,'result_dims',None)))
        break
print('of those, with an ARRAY-VALUED FUNCTION RESULT -- the only shape the new')
print('branch can reach:', len(hits))
for u,f,d in hits:
    print(f'  {u:14s} {f:18s} result_dims={d!r}')
print()
print('A dimension that is a bare NAME (\`n\`) or an INTEGER LITERAL was already')
print('accepted and takes the same path it always did. The new branch fires only')
print('on \`SIZE(<arr>)\`, which the old code REFUSED -- so it can turn a refusal')
print('into a mapping and cannot turn a mapping into anything else.')
PY"
