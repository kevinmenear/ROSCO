#!/usr/bin/env python3
"""Re-point this unit's `--equivalences` declarations at the SITES they were
written about, after an edit to the translation renumbered the mutant ids.

    python3 evidence/ParseInput_Dbl_Opt/remap_declarations.py <old-rev> [--write]

WHY. `cppmutate` identifies a mutant by an occurrence index over identical
edits, so removing one `const_tweak` site renumbers every later `const_tweak`
mutant. RUNBOOK, unit #55: enumerate `(id -> site)`, not ids -- a declaration
that silently re-points at a live site RAISES the score, which is the dangerous
direction. Measured on THIS repair: 181 -> 182 mutants, 4 lost and 5 gained by
id, and **25 ids re-pointed at a different site**, four of them declared.

THE KEY IS THE SITE, NOT THE LINE NUMBER. A repair moves line numbers, so the
key is `(operator, before, after, the stripped source line, and which
occurrence of that exact tuple it is in file order)`. Two `p < len` guards on
textually identical lines are distinguished by the last field and by nothing
else, which is why it is there.

A declaration whose site NO LONGER EXISTS is dropped and named. That is the
right outcome and not a loss: `vit_mutate.py` cannot check a claim about a
mutant it is not offered.
"""
import json
import pathlib
import subprocess
import sys

sys.path.insert(0, '/Users/kmenear/Artifacts/vit_translation/translation-loop')
from harness.cppmutate import mutants  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
CPP = 'translations/ROSCO_Helpers/parseinput_dbl_opt.cpp'
EQ = ROOT / 'mutation/ParseInput_Dbl_Opt.equivalences.json'


def keyed(text: str) -> dict[str, tuple]:
    lines = text.splitlines()
    seen: dict[tuple, int] = {}
    out: dict[str, tuple] = {}
    for m in mutants('parseinput_dbl_opt', text):
        base = (m.operator, m.before, m.after, lines[m.line - 1].strip())
        n = seen.get(base, 0)
        seen[base] = n + 1
        out[m.mid] = base + (n,)
    return out


def main() -> int:
    rev = sys.argv[1]
    write = '--write' in sys.argv
    old = subprocess.run(['git', 'show', f'{rev}:{CPP}'], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout
    new = (ROOT / CPP).read_text()
    A, B = keyed(old), keyed(new)
    back = {v: k for k, v in B.items()}

    declared = json.loads(EQ.read_text())
    remapped, dropped, unmoved = {}, [], 0
    for mid in declared:
        site = A.get(mid)
        if site is None:
            dropped.append((mid, 'not in the OLD enumeration at all'))
            continue
        new_id = back.get(site)
        if new_id is None:
            dropped.append((mid, f'the site no longer exists: {site[:4]}'))
            continue
        remapped[mid] = new_id
        unmoved += (new_id == mid)

    print(f'{len(declared)} declared equivalence(s): '
          f'{len(remapped)} re-pointed by site ({unmoved} kept their id), '
          f'{len(dropped)} dropped')
    for mid, new_id in sorted(remapped.items()):
        if new_id != mid:
            print(f'  {mid} -> {new_id}   {A[mid][:3]}  {A[mid][3][:60]}')
    for mid, why in dropped:
        print(f'  DROPPED {mid}: {why}')

    if write:
        EQ.write_text(json.dumps(sorted(remapped.values()), indent=2) + '\n')
        print(f'wrote {EQ.relative_to(ROOT)}')
        mapping = ROOT / 'evidence/ParseInput_Dbl_Opt/equivalence_remap.json'
        mapping.write_text(json.dumps(
            {'old_rev': rev,
             'remapped': remapped,
             'dropped': {m: w for m, w in dropped}}, indent=2) + '\n')
        print(f'wrote {mapping.relative_to(ROOT)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
