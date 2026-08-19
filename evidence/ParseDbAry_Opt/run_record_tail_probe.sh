#!/bin/bash
# Run both halves of the record-tail probe and write the two columns side by
# side.
#
#   bash evidence/ParseDbAry_Opt/run_record_tail_probe.sh
#
# It asks whether the reference and the translation agree on the four records
# that END INSIDE A VALUE -- the records the corpus change plants so that the
# six `p == len` boundary mutants become distinguishable. Asked BEFORE the
# corpus was changed: a 2048-digit field is exactly the kind of input a runtime
# is entitled to reject, and a planted record the two sides disagree about is a
# red harness that says nothing about the mutants it was planted for.
#
# It never writes to the tree under test.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
HERE=evidence/ParseDbAry_Opt
WORK=/workspace/ROSCO-r2
OUT=$HERE/record_tail_probe.txt

docker exec vit-dev bash -lc "
set -e
mkdir -p $WORK/.tail_probe && cd $WORK/.tail_probe
gfortran -O2 -ffree-line-length-none -fdefault-real-8 -fdefault-double-8 \
         -ffp-contract=off -o tail_ref $WORK/$HERE/record_tail_probe.f90
g++ -O0 -ffp-contract=off -std=c++17 \
    -I$WORK/translations/ROSCO_Helpers/parsedbary_opt_test \
    -DVIT_TRANSLATION='\"$WORK/translations/ROSCO_Helpers/parsedbary_opt.cpp\"' \
    $WORK/$HERE/record_tail_probe.cpp -o tail_got -lgfortran
./tail_ref > ref.txt
./tail_got > got.txt
cat ref.txt got.txt
" > "$OUT.raw"

python3 - "$OUT.raw" "$OUT" <<'PY'
import sys, re
raw, out = sys.argv[1], sys.argv[2]
ref, got = {}, {}
for l in open(raw):
    m = re.match(r'(REF|GOT)\s+(\S+)\s+iostat=(-?\d+)\s+b1=([0-9A-F]+)\s+b2=([0-9A-F]+)',
                 l.strip())
    if m:
        (ref if m.group(1) == 'REF' else got)[m.group(2)] = (
            int(m.group(3)), m.group(4), m.group(5))
rows, agree = [], 0
for w in ref:
    r, g = ref[w], got.get(w)
    # THE WHOLE TRIPLE IS THE VERDICT here, unlike the IEEE-word probe: these
    # records decide both which elements were transferred and whether the READ
    # failed, and the harness compares `Ary` AND `ErrVar%aviFAIL`, which the
    # reference sets from exactly this IOSTAT.
    ok = g is not None and r == g
    agree += ok
    rows.append(f"  {w:<12} ref iostat={r[0]:<4} b1={r[1]} b2={r[2]}\n"
                f"  {'':<12} got iostat={(g[0] if g else '?'):<4} "
                f"b1={g[1] if g else '?'} b2={g[2] if g else '?'}   "
                f"{'SAME' if ok else 'DIFFER'}")
hdr = [
    "Record-tail probe -- the four records that end INSIDE a value, gfortran's",
    "list-directed READ against the shipped translation's list_read_reals.",
    "Two elements read; the sentinel is -987.654 (bits C08ED4A3D70A3D71), so an",
    "element the transfer never reached is visible as itself.",
    "",
    f"{agree} of {len(ref)} record(s) agree on IOSTAT and on both elements.",
    "",
]
open(out, 'w').write("\n".join(hdr + rows) + "\n")
print("\n".join(hdr + rows))
PY
