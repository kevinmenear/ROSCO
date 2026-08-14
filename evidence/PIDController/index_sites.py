#!/usr/bin/env python3
"""One variant per `[i]` occurrence in the shipped translation, each with that
ONE occurrence changed to `[i + 1]`.

`mutation/PIDController.json` names two surviving `index_offset` mutants by
(operator, before, after) only -- `'[i]' -> '[i + 1]'`, twelve times over -- so
the artifact cannot say WHICH sites they are. Unit #32's rule: when several
sites match, BUILD EACH CANDIDATE AND RUN IT rather than reasoning about which
one it is. Twelve runs at about six seconds each turn a guess into a table.
"""
import pathlib, sys

real = pathlib.Path('translations/Controllers/pidcontroller.cpp').read_text()
out = pathlib.Path('evidence/PIDController/index_sites')
out.mkdir(parents=True, exist_ok=True)

lines = real.splitlines(keepends=True)
sites = []          # (variant_index, line_no, col, the line's text)
for ln, text in enumerate(lines, 1):
    if text.lstrip().startswith('//'):
        continue
    start = 0
    while True:
        c = text.find('[i]', start)
        if c < 0:
            break
        sites.append((ln, c, text.strip()))
        start = c + 1

for k, (ln, c, text) in enumerate(sites, 1):
    body = list(lines)
    t = body[ln - 1]
    body[ln - 1] = t[:c] + '[i + 1]' + t[c + 3:]
    (out / f'site{k:02d}.cpp').write_text(''.join(body))

with open(out / 'SITES.txt', 'w') as f:
    for k, (ln, c, text) in enumerate(sites, 1):
        f.write(f'site{k:02d}  line {ln:>3} col {c:>3}  {text}\n')
print(f'{len(sites)} site(s)')
