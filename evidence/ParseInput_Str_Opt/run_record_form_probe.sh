#!/bin/bash
# Price every record form this unit's corpus can contain against the REFERENCE,
# before the generator is touched.
#
#   bash evidence/ParseInput_Str_Opt/run_record_form_probe.sh
#
# The runbook's rule, from unit #54 and sharpened by unit #56: a corpus change
# that could make the reference do something new is a measurement to take, not a
# risk to accept. A form the two sides disagree on turns the differential
# harness RED, and a red primary layer takes the mutation layer with it.
#
# THIS UNIT HAS A SECOND REASON, which the number-reading siblings did not: the
# translation's claim that `IF (ErrStatLcl /= 0)` is DEAD IN THE PROGRAM is what
# licenses every `unreachable` declaration in that block. A single non-zero
# IOSTAT in this table refutes it, and the repair would be a corpus round rather
# than a declaration.
#
# BUILD AND RUN ARE SEPARATE STEPS and each status is classified on its own --
# unit #55's C12 finding about a copied runner that collapsed "did not build"
# into "found nothing".
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
OUT=evidence/ParseInput_Str_Opt/record_form_probe.txt
W=/workspace/ROSCO-r2
D=$W/evidence/ParseInput_Str_Opt

docker exec vit-dev bash -lc "
    set -e
    cd /tmp
    gfortran -O0 -ffp-contract=off $D/record_form_probe.f90 -o rfp_str_ref
    g++ -O0 -ffp-contract=off -std=c++17 \
        -I$W/rosco/controller/src \
        -DVIT_TRANSLATION='\"$W/translations/ROSCO_Helpers/parseinput_str_opt.cpp\"' \
        $D/record_form_probe.cpp -o rfp_str_got -lgfortran
" || { echo "record_form_probe: BUILD FAILED -- nothing measured"; exit 1; }

docker exec vit-dev bash -lc "cd /tmp && ./rfp_str_ref" > /tmp/rfp_str_ref.txt
docker exec vit-dev bash -lc "cd /tmp && ./rfp_str_got" > /tmp/rfp_str_got.txt

python3 - "$ROOT/$OUT" <<'PY'
import pathlib, sys, re
out = sys.argv[1]

def rows(p):
    d = {}
    for line in pathlib.Path(p).read_text().splitlines():
        m = re.match(r'R (\S+) (\d+) iostat= (-?\d+) lentrim= (\d+) h= (\d+) p="(.*)"$', line)
        if m:
            d[(m.group(1), int(m.group(2)))] = (int(m.group(3)), int(m.group(4)),
                                                int(m.group(5)), m.group(6))
    return d

ref, got = rows('/tmp/rfp_str_ref.txt'), rows('/tmp/rfp_str_got.txt')
bad = [k for k in ref if ref[k] != got.get(k)]
nonzero = sorted({k for k in ref if ref[k][0] != 0})

lines = [
    "ParseInput_Str_Opt -- `READ (Words(1),'(A)',IOSTAT=e) Variable`, taken by",
    "gfortran's own runtime and by the SHIPPED translation's `read_a_edit`, out",
    "of the same 400-byte `CHARACTER(200) :: Words(2)` object.",
    "",
    "Sixteen record forms crossed with nine item lengths. The item is pre-set to",
    "'~' in every byte on both sides, so an UNTOUCHED item is distinguishable",
    "from one the READ filled with blanks -- which is the only way a failing",
    "READ could be told from a successful one that stored nothing.",
    "",
    "`h=` is a rolling 31-multiplier hash over ALL len bytes, so the comparison",
    "is over the whole item and not over the 24-byte prefix it prints.",
    "",
    "Forms tagged `fn:` are admissible to the FUNCTION and not to the UNIT --",
    "`GetWords` splits on blank, comma, semicolon, '!', quote and tab, so no",
    "input to this unit can put one in `Words(1)`. They are measured anyway and",
    "tagged, because unit #56 twice read a function-only record as a corpus",
    "lever.",
    "",
    f"{len(ref) - len(bad)} of {len(ref)} (form, length) pairs agree on "
    f"(IOSTAT, LEN_TRIM, hash, prefix).",
    "",
    f"IOSTAT non-zero on {len(nonzero)} of {len(ref)} pairs in the REFERENCE.",
    "  Zero here is the measurement the translation's dead-arm claim rests on:",
    "  `IF (ErrStatLcl /= 0)` and its whole message block cannot be entered by",
    "  any input, which is a property of the PROGRAM and not of the corpus.",
    "",
]
for k in sorted(ref):
    r, g = ref[k], got.get(k, ('--', '--', '--', '--'))
    same = 'SAME' if r == g else 'DIFFERS  <-- DO NOT PLANT'
    lines.append(
        f"  {k[0]:<10s} L={k[1]:<5d} ref iostat={r[0]:<3} lentrim={r[1]:<5} h={r[2]:<11}"
        f"   got iostat={g[0]:<3} lentrim={g[1]:<5} h={g[2]:<11}  {same}")
    if r != g:
        lines.append(f'        ref p="{r[3]}"')
        lines.append(f'        got p="{g[3]}"')
pathlib.Path(out).write_text("\n".join(lines) + "\n")
print("\n".join(lines[:24]))
print(f"... full table in {out}")
sys.exit(1 if bad else 0)
PY
