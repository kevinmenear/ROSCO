#!/usr/bin/env python3
"""P4 for unit #57: re-extract every copied block from its SOURCE file and
compare bytes, rather than asserting that a copy was made.

This unit is `ParseInput_Dbl_Opt` (#56) with one declaration changed, and the
two halves of it come from two different siblings:

  * the CALLER half -- `ftrim`, `int2lstr_trimmed`, and the subroutine body's
    control flow -- from `parseinput_dbl_opt.cpp`;
  * the ITEM half -- the whole list-directed INTEGER reader and the
    list-directed INTEGER output field -- from `parseinary_opt.cpp`, which is
    the unit that MEASURED both against gfortran for an INTEGER(4) item.

The split is the runbook's own discriminator (item TYPE against record
GRAMMAR): unit #55 broke by copying unit #54's separator set, which is item
type; unit #56 held by reusing grammar rules. So every item-typed rule here
comes from the INTEGER sibling and none from the REAL one, and this script is
what makes that a checkable claim instead of a sentence.

A block is named by its first and last line, matched EXACTLY, so a block that
has drifted in either file is a KeyError rather than a silent partial match.

    python3 evidence/ParseInput_Int_Opt/check_p4_blocks.py

Exit 0 if every block is byte-identical, 1 otherwise. It never writes to a
source file.
"""
from __future__ import annotations

import hashlib
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
MINE = ROOT / "translations/ROSCO_Helpers/parseinput_int_opt.cpp"
DBL = ROOT / "translations/ROSCO_Helpers/parseinput_dbl_opt.cpp"
INARY = ROOT / "translations/ROSCO_Helpers/parseinary_opt.cpp"

# (label, source file, first line, last line, which occurrence of the last line)
#
# THE ONE PLACE THE COPY IS NOT BYTE-IDENTICAL IS NAMED HERE RATHER THAN LEFT
# INSIDE A BLOCK. `list_read_ints`'s doc comment says "what makes `Ary` an
# output on the failure path too" in unit #55 and "`Variable`" here, because
# this unit's item is a scalar dummy and not an array. That comment sits
# BETWEEN blocks 3 and 4 and is deliberately outside both, so the check stays
# a byte comparison instead of becoming a fuzzy one.
BLOCKS = [
    ("ftrim", DBL,
     "std::string_view ftrim(const char* s, int len) {", "}", 1),
    ("int2lstr_trimmed", DBL,
     "std::string int2lstr_trimmed(int Num) {", "}", 1),
    ("Sep .. parse_int", INARY,
     "enum class Sep { Blank, Comma, Eol };", "}", 2),
    ("list_read_ints", INARY,
     "int list_read_ints(const char* rec, int len, int32_t* v, int n) {", "}", 1),
    ("field", INARY,
     "std::string field(const std::string& text, int w) {", "}", 1),
    ("list_directed_int", INARY,
     "std::string list_directed_int(int32_t v) {", "}", 1),
]


def extract(path: Path, first: str, last: str, occ: int = 1) -> str:
    """The block from the line equal to `first` through the `occ`-th line equal
    to `last` at or after it. Both matched on the stripped-of-newline line, so
    indentation is part of the comparison."""
    lines = path.read_text().split("\n")
    try:
        i = lines.index(first)
    except ValueError as exc:
        raise KeyError(f"{path.name}: first line not found: {first!r}") from exc
    seen = 0
    for j in range(i, len(lines)):
        if lines[j] == last:
            seen += 1
            if seen == occ:
                return "\n".join(lines[i:j + 1]) + "\n"
    raise KeyError(f"{path.name}: last line #{occ} not found after {first!r}: {last!r}")


def sha(text: str) -> str:
    return hashlib.sha256(text.encode()).hexdigest()


def main() -> int:
    bad = 0
    print("P4 block check -- unit #57 ParseInput_Int_Opt")
    print(f"  target  {MINE.relative_to(ROOT)}")
    print()
    print(f"  {'block':<26} {'bytes':>7}  {'sha256[:16]':<18} source")
    for label, src, first, last, occ in BLOCKS:
        try:
            a = extract(src, first, last, occ)
            b = extract(MINE, first, last, occ)
        except KeyError as exc:
            print(f"  {label:<26} {'--':>7}  MISSING            {exc}")
            bad += 1
            continue
        ok = a == b
        bad += 0 if ok else 1
        print(f"  {label:<26} {len(a):>7}  {sha(a)[:16]:<18} "
              f"{src.name}  {'IDENTICAL' if ok else '*** DIFFERS ***'}")
    print()
    total = sum(len(extract(src, f, l, o)) for _, src, f, l, o in BLOCKS)
    print(f"  {len(BLOCKS)} blocks, {total} bytes copied and hash-verified"
          if bad == 0 else f"  {bad} of {len(BLOCKS)} blocks DIFFER")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
