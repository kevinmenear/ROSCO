#!/bin/bash
# Price every record form this unit's corpus can contain against the REFERENCE,
# before the generator is touched.
#
#   bash evidence/ParseInput_Int_Opt/run_record_form_probe.sh
#
# The runbook's rule, from unit #54 and sharpened by unit #56: a corpus change
# that could make the reference do something new is a measurement to take, not a
# risk to accept. A form the two sides disagree on turns the differential
# harness RED, and a red primary layer takes the mutation layer with it. On unit
# #56 both of that unit's defects appeared as a DIFFERS row here.
#
# BUILD AND RUN ARE SEPARATE STEPS and each status is classified on its own --
# unit #55's C12 finding about a copied runner that collapsed "did not build"
# into "found nothing".
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
OUT=evidence/ParseInput_Int_Opt/record_form_probe.txt
W=/workspace/ROSCO-r2
D=$W/evidence/ParseInput_Int_Opt

docker exec vit-dev bash -lc "
    set -e
    cd /tmp
    gfortran -O0 -ffp-contract=off $D/record_form_probe.f90 -o rfp_int_ref
    g++ -O0 -ffp-contract=off -std=c++17 \
        -I$W/rosco/controller/src \
        -DVIT_TRANSLATION='\"$W/translations/ROSCO_Helpers/parseinput_int_opt.cpp\"' \
        $D/record_form_probe.cpp -o rfp_int_got -lgfortran
" || { echo "record_form_probe: BUILD FAILED -- nothing measured"; exit 1; }

docker exec vit-dev bash -lc "cd /tmp && ./rfp_int_ref" > /tmp/rfp_int_ref.txt
docker exec vit-dev bash -lc "cd /tmp && ./rfp_int_got" > /tmp/rfp_int_got.txt

python3 - "$ROOT/$OUT" <<'PY'
import pathlib, sys, re
out = sys.argv[1]
def rows(p):
    d = {}
    for line in pathlib.Path(p).read_text().splitlines():
        m = re.match(r'\s*\w+\s+(\S+)\s+iostat=\s*(-?\d+)\s+value=\s*(-?\d+)', line)
        if m:
            d[m.group(1)] = (int(m.group(2)), int(m.group(3)))
    return d
ref, got = rows('/tmp/rfp_int_ref.txt'), rows('/tmp/rfp_int_got.txt')
lines = [
    "ParseInput_Int_Opt -- every record form this unit's corpus can contain,",
    "read by gfortran's own list-directed READ and by the SHIPPED translation's",
    "`list_read_ints`, out of the same 400-byte `CHARACTER(200) :: Words(2)`.",
    "",
    "The item is pre-set to -987654 on both sides, so an UNTOUCHED slot is",
    "distinguishable from a stored value -- which is the whole difference",
    "between the two failure kinds the INTEGER reader has.",
    "",
    "Every form is a WORD `GetWords` could produce: no blank, comma, semicolon,",
    "quote, '!' or tab, left-justified into a blank-padded 200-byte element.",
    "Forms admissible to the FUNCTION and not to the UNIT are excluded on",
    "purpose -- unit #56 recorded two survivors misread for exactly that reason.",
    "",
]
bad = [k for k in ref if ref[k] != got.get(k)]
lines.append(f"{len(ref) - len(bad)} of {len(ref)} form(s) agree on (IOSTAT, value).")
lines.append("")
for k in ref:
    r, g = ref[k], got.get(k, ('--', '--'))
    lines.append(f"  {k:10s} ref iostat={r[0]:<6} value={r[1]:<12}   "
                 f"got iostat={g[0]:<6} value={g[1]:<12}   "
                 f"{'SAME' if r == g else 'DIFFERS  <-- DO NOT PLANT'}")
pathlib.Path(out).write_text("\n".join(lines) + "\n")
print("\n".join(lines))
sys.exit(1 if bad else 0)
PY
