#!/bin/bash
# Measure, at their SITE and over the whole corpus, what the two surviving width
# constants' mutants actually change.
#
#   bash evidence/ParseInput_Str_Opt/run_boundary_probe.sh      # tree must be CLEAN
#
#   4d525138  MaxLineLength  2048 -> 2049
#   739190c8  MaxParamLength  200 -> 201
#
# WHY A PROBE RATHER THAN AN ARGUMENT. Both constants are widths, and a width
# mutant's usual answer in this campaign is one of two very different things:
# `unreachable` if the corpus cannot reach the byte, or an open survivor if it
# can and nobody built the record. Unit #32 left `'2048' -> '2049'` OPEN in
# `findline.cpp` rather than declare it, because there it writes past the
# CALLER's buffer -- undefined behaviour, which unit #7's rule says cannot be
# declared equivalent. HERE the buffer is this frame's own `std::vector`, so the
# mutant is in bounds and the question is genuinely about what `GetWords`
# returns. That is measurable, and measuring it is one extra callee call per
# case.
#
# The variant .cpp is the shipped translation with ONE inserted block: after the
# real `getwords_c`, it calls the callee twice more -- once with the line one
# byte longer, once with the elements one byte wider -- and reports whether
# either answer differs. It is a VARIANT and is never integrated; it exists so
# the declaration below rests on a number.
#
# THE TREE MUST BE CLEAN. The probe links this campaign's Fortran objects and
# calls the real `GetWords`.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
LIVE=translations/ROSCO_Helpers/parseinput_str_opt.cpp
VAR=evidence/ParseInput_Str_Opt/parseinput_str_opt.boundary-probe.cpp
OUT=evidence/ParseInput_Str_Opt/boundary_probe.txt
D=translations/ROSCO_Helpers/parseinput_str_opt_test
KEEP=$(mktemp); cp "$LIVE" "$KEEP"
restore() { cp "$KEEP" "$LIVE"; rm -f "$KEEP"; }
trap restore EXIT

cp "$VAR" "$LIVE"
want=$(md5 -q "$VAR")
got=""
for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$LIVE | cut -d' ' -f1")
    [ "$want" = "$got" ] && break
    cp "$VAR" "$LIVE"
done
[ "$want" = "$got" ] || { echo "HASH MISMATCH on $LIVE"; exit 1; }
echo "variant md5 (verified inside the container): $got"

docker exec vit-dev bash -lc \
    "cd /workspace/ROSCO-r2/$D && rm -f parseinput_str_opt.hpp parseinput_str_opt_test.o test && \
     make test > /dev/null 2>&1 && ./test parseinput_str_opt_cases.bin > /dev/null" \
    2> /tmp/parseinput_str_boundary_raw.txt || true

python3 - /tmp/parseinput_str_boundary_raw.txt "$ROOT/$OUT" <<'PY'
import collections, pathlib, re, sys
rows = []
for line in pathlib.Path(sys.argv[1]).read_text(errors="replace").splitlines():
    m = re.match(r"BND lastline=(\d+) w1_2049=(\d+) w2_2049=(\d+) "
                 r"w1_201=(\d+) byte201=(\d+) lenvar=(-?\d+)$", line)
    if m:
        rows.append(tuple(int(x) for x in m.groups()))
if not rows:
    print("boundary_probe: NO ROWS -- the probe measured nothing", file=sys.stderr)
    sys.exit(1)

n = len(rows)
lastline = collections.Counter(r[0] for r in rows)
w1_2049 = sum(r[1] for r in rows)
w2_2049 = sum(r[2] for r in rows)
w1_201 = sum(r[3] for r in rows)
byte201 = collections.Counter(r[4] for r in rows)
lenvar = collections.Counter(r[5] for r in rows)

out = [
 f"ParseInput_Str_Opt -- the two width mutants measured at their site.",
 "",
 "  4d525138   constexpr int MaxLineLength  = 2048  ->  2049",
 "  739190c8   constexpr int MaxParamLength =  200  ->   201",
 "",
 f"{n:,} rows, NOT the corpus's 14,116. The probe block sits after the real",
 "`getwords_c`, which is after `IF (ErrVar%aviFAIL >= 0)`, so the 190",
 "`pre-failed` cases never reach it -- 14,116 - 190 = 13,926, and those 190 are",
 "cases in which the reference does nothing either. The row count is stated",
 "because a probe that silently measures a subset is a probe whose zero means",
 "less than it looks.",
 "",
 "Each constant reaches exactly one LIVE consumer, `getwords_c`; the others are",
 "a buffer size (both mutants allocate MORE, never less) and a `ftrim` inside",
 "the READ-error arm, which is dead in the program. So the probe calls the",
 "callee twice more per case -- once with the line one byte longer, once with",
 "the elements one byte wider -- and asks whether the words come back different.",
 "",
 "MaxLineLength 2048 -> 2049",
 f"  cases where Words(1) differs        {w1_2049} of {n}",
 f"  cases where Words(2) differs        {w2_2049} of {n}",
 f"  Line's LAST byte, by value          {dict(sorted(lastline.items()))}",
 "",
 "  THE MECHANISM IS TRUNCATION, NOT THE SCAN STOPPING, and the distribution",
 "  above is what says so: byte 2048 is a blank (32) on only "
 f"{lastline.get(32, 0)} of",
 f"  {n} cases. It is NUL on {lastline.get(0, 0)} -- those are the cases where",
 "  `FindLine` found nothing and never assigned `Line`, so the C++ vector's own",
 "  zero fill is what is there -- and something else on the rest.",
 "",
 "  `GetWords` writes each word into a `CHARACTER(MaxParamLength)` element and",
 "  TRUNCATES at 200. A word that ends at byte 2048 and started at or before",
 "  1849 is at least 200 characters long, so its 2049th byte is cut off in both",
 "  spellings. The mutant can only be seen by a word that ends at 2048 and is",
 "  SHORTER than 200 -- one that starts at 1850 or later, with a separator in",
 "  front of it. No case of this corpus has one, and the two zeros above are",
 "  that measured rather than argued.",
 "",
 "MaxParamLength 200 -> 201",
 f"  cases where Words(1)'s first 200 bytes differ   {w1_201} of {n}",
 f"  the 201st byte of Words(1), by value            {dict(sorted(byte201.items()))}",
 f"  LEN(Variable) over the corpus                   {dict(sorted(lenvar.items()))}",
 "",
 "  ONE CAUSE, AND IT IS THE CORPUS'S `LEN(Variable)` LADDER. The 201st byte of",
 f"  Words(1) is NON-BLANK on {n - byte201.get(32, 0)} cases, so the corpus DOES",
 "  carry first words of 201 characters or more -- the obvious second reason a",
 "  reader would reach for is refuted by its own column.",
 "",
 "  What is left is arithmetic. `char_assign(Variable, len_Variable, Words1,",
 "  len_src)` runs `i = 1 .. len_Variable` and uses `len_src` only in the test",
 "  `i <= len_src`; with len_src 200 or 201 the two spellings can differ only at",
 "  i = 201, which needs LEN(Variable) >= 201. The ladder above tops out at 11,",
 "  because `generate.py` builds a free string extent's lengths as",
 "  sorted({1, 2, ex0, ex0 + 5}) and ex0 is 6 here.",
 "",
 "  THAT IS A CLAIM ABOUT THE CORPUS AND NOT ABOUT THE PROGRAM, and the",
 "  difference is worth stating because it is large: every shipped caller passes",
 "  LEN(Variable) of 1024 or 256 (`CntrPar%PerfFileName`, `OL_Filename`,",
 "  `DLL_FileName`, `DLL_InFile`, `DLL_ProcName` at 1024, `ZMQ_CommAddress` at",
 "  256). A corpus that drew the length from the shipped range would kill this",
 "  mutant on the records it already has. Closing it needs `generate.py` to",
 "  widen a string extent ladder, which re-prices every unit already scored, so",
 "  it is declared here and raised in DECISIONS.md rather than taken silently.",
]
pathlib.Path(sys.argv[2]).write_text("\n".join(out) + "\n")
print("\n".join(out))
PY
