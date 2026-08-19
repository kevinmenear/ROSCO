#!/bin/bash
# Run both halves of the IEEE-word probe and write the two columns side by side.
#
#   bash evidence/ParseDbAry_Opt/run_ieee_word_probe.sh
#
# It answers ONE question, asked before the corpus was changed rather than after
# it went red: does gfortran's list-directed READ of `inf`/`nan` leave the same
# 64 bits in a REAL(8) as the translation's `strtod` path? The harness compares
# `Ary` with `memcmp`, so a NaN payload that differs by one bit is a FAILED case
# on every planted record.
#
# It never writes to the tree under test: both halves are built in a scratch
# directory and the shipped .cpp is only ever read.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
HERE=evidence/ParseDbAry_Opt
WORK=/workspace/ROSCO-r2
OUT=$HERE/ieee_word_probe.txt

docker exec vit-dev bash -lc "
set -e
mkdir -p $WORK/.ieee_probe && cd $WORK/.ieee_probe
gfortran -O2 -ffree-line-length-none -fdefault-real-8 -fdefault-double-8 \
         -ffp-contract=off -o ieee_ref $WORK/$HERE/ieee_word_probe.f90
g++ -O0 -ffp-contract=off \
    -I$WORK/translations/ROSCO_Helpers/parsedbary_opt_test \
    -DVIT_TRANSLATION='\"$WORK/translations/ROSCO_Helpers/parsedbary_opt.cpp\"' \
    $WORK/$HERE/ieee_word_probe.cpp -o ieee_got -lgfortran
./ieee_ref > ref.txt
./ieee_got > got.txt
paste -d'\n' ref.txt got.txt
" > "$OUT.raw"

python3 - "$OUT.raw" "$OUT" <<'PY'
import sys, re
raw, out = sys.argv[1], sys.argv[2]
lines = [l.rstrip() for l in open(raw) if l.strip()]
ref = {}
got = {}
for l in lines:
    m = re.match(r'(REF|GOT)\s+(\S+)\s+iostat=(-?\d+)\s+bits=([0-9A-F]+)', l)
    if not m:
        continue
    (ref if m.group(1) == 'REF' else got)[m.group(2)] = (int(m.group(3)), m.group(4))
rows = []
agree = 0
for w in ref:
    r, g = ref[w], got.get(w)
    ok = (g is not None and r[1] == g[1])
    # IOSTAT is NOT part of the verdict here: the reference reads ONE value out
    # of a record that holds one, so it always ends with an END condition (-1)
    # while the translation models the same exhaustion as its own `return -1`.
    # The BITS are the thing the harness compares element by element.
    agree += ok
    rows.append(f"  {w:<12} ref iostat={r[0]:<6} bits={r[1]}   "
                f"got iostat={g[0] if g else '?':<6} bits={g[1] if g else '?'}   "
                f"{'SAME' if ok else 'DIFFER'}")
hdr = [
    "IEEE-word probe -- gfortran's list-directed READ against the shipped",
    "translation's parse_real/list_read_reals, ten words, 64 bits each.",
    "",
    "THE QUESTION: the differential harness compares `Ary` with memcmp, so a",
    "corpus record spelling `nan` is only safe to plant if both sides produce",
    "the SAME bit pattern. Asked before the corpus was changed.",
    "",
    f"{agree} of {len(ref)} word(s) agree bit for bit.",
    "",
]
open(out, 'w').write("\n".join(hdr + rows) + "\n")
print("\n".join(hdr + rows))
PY
