#!/bin/bash
# Price this unit's PRINT record against gfortran's own, before the corpus is
# generated.
#
#   bash evidence/ParseInput_Str_Opt/run_print_record_probe.sh
#
# The record is COMPARED on every case that reaches the arm (`vit_record` in
# harness/ranges.toml), so the harness is the oracle in the end -- but a
# derivation that is wrong shows up there as a red primary layer, which takes
# the mutation layer with it. Pricing it first costs seconds.
#
# BUILD AND RUN ARE SEPARATE STEPS, each classified on its own.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
OUT=evidence/ParseInput_Str_Opt/print_record_probe.txt
W=/workspace/ROSCO-r2
D=$W/evidence/ParseInput_Str_Opt

docker exec vit-dev bash -lc "
    set -e
    cd /tmp
    gfortran -O0 -ffp-contract=off $D/print_record_probe.f90 -o prp_str_ref
    g++ -O0 -ffp-contract=off -std=c++17 \
        -I$W/rosco/controller/src \
        -DVIT_TRANSLATION='\"$W/translations/ROSCO_Helpers/parseinput_str_opt.cpp\"' \
        $D/print_record_probe.cpp -o prp_str_got -lgfortran
" || { echo "print_record_probe: BUILD FAILED -- nothing measured"; exit 1; }

docker exec vit-dev bash -lc "cd /tmp && ./prp_str_ref" > /tmp/prp_str_ref.txt
docker exec vit-dev bash -lc "cd /tmp && ./prp_str_got" > /tmp/prp_str_got.txt

python3 - "$ROOT/$OUT" <<'PY'
import pathlib, sys
out = sys.argv[1]
ref = pathlib.Path('/tmp/prp_str_ref.txt').read_bytes().split(b'\n')
got = pathlib.Path('/tmp/prp_str_got.txt').read_bytes().split(b'\n')
if ref and ref[-1] == b'': ref.pop()
if got and got[-1] == b'': got.pop()

def show(b):
    return ''.join(chr(c) if 32 <= c < 127 else f'\\x{c:02X}' for c in b)

n = max(len(ref), len(got))
bad = []
lines = [
    "ParseInput_Str_Opt -- the PRINT record, gfortran's own against the SHIPPED",
    "translation's `print_default_warning`.",
    "",
    "`PRINT *, \"ROSCO Warning: Did not find \"//TRIM(VarName)//\" in input file.",
    "  Using default value of \", TRIM(Variable)` -- TWO ADJACENT CHARACTER",
    "ITEMS, a layout no unit in this family has had. The derivation under test:",
    "one leading blank starts the record (written once, for the first item), and",
    "a CHARACTER item that FOLLOWS a CHARACTER item takes NO separator, because",
    "libgfortran's list_formatted_write_scalar writes one only when",
    "`type != BT_CHARACTER || !char_flag || delim_status != DELIM_NONE` and",
    "DELIM_NONE is the default for list-directed output.",
    "",
    "`Variable = 'unused'` is the statement immediately above, so the second",
    "item is the six-byte literal TRUNCATED to LEN(Variable). The lengths",
    "straddle six and include zero.",
    "",
    "Trailing blanks are significant here and are shown escaped; a record whose",
    "only difference is a trailing blank is exactly what a separator rule gets",
    "wrong.",
    "",
]
for i in range(n):
    r = ref[i] if i < len(ref) else b'<missing>'
    g = got[i] if i < len(got) else b'<missing>'
    ok = (r == g)
    if not ok:
        bad.append(i)
    lines.append(f"  case {i+1}  {'SAME' if ok else 'DIFFERS'}  len ref={len(r)} got={len(g)}")
    lines.append(f'      ref |{show(r)}|')
    if not ok:
        lines.append(f'      got |{show(g)}|')
lines.insert(20, f"{n - len(bad)} of {n} records byte-identical.")
lines.insert(21, "")
pathlib.Path(out).write_text("\n".join(lines) + "\n")
print("\n".join(lines))
sys.exit(1 if bad else 0)
PY
