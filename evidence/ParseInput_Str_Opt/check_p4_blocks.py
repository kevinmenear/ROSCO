#!/usr/bin/env python3
"""Re-extract every block this translation COPIED and compare it byte for byte.

    python3 evidence/ParseInput_Str_Opt/check_p4_blocks.py

P4 says copy and hash-verify, never re-implement from prose. A copy ASSERTED in
a comment is a copy nobody can check, and it goes stale silently the first time
either side is edited. This script re-reads each block out of its SOURCE FILE
and out of this unit's translation and compares the bytes.

COPIED IN SHAPE from `evidence/ParseInput_Int_Opt/check_p4_blocks.py` (P4), with
this unit's blocks and their two sources. The difference from the sibling is
which sources they come from, and that is the interesting half:

  * the CALLER half -- `ftrim` and `int2lstr_trimmed` -- comes from
    `parseinput_int_opt.cpp` (#57), the unit whose clean Fortran body differs
    from this one's only in the declaration of `Variable`, the default value,
    the FORM of the READ, and whitespace;

  * `char_assign` comes from `findline.cpp` (#32) and NOT from #57, which has
    no such function at all. It is the block whose form unit #32 MEASURED: the
    obvious `std::min` + `memcpy` + `memset` shape leaves three mutants alive
    that no value comparison can kill, and the one-loop form deletes their
    sites. Taking the measured form is the whole reason this block is a copy.

`assign_errmsg` is deliberately NOT checked as a byte copy: it carries this
unit's own name in two diagnostics, so it is a copy with a stated edit rather
than a copy. Saying so here is cheaper than a checker that would have to know
which differences are allowed.
"""
from __future__ import annotations

import hashlib
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
MINE = ROOT / "translations/ROSCO_Helpers/parseinput_str_opt.cpp"

# (label, source file, the block's first line, the block's last line)
BLOCKS = [
    ("ftrim", "translations/ROSCO_Helpers/parseinput_int_opt.cpp",
     "std::string_view ftrim(const char* s, int len) {", "}"),
    ("int2lstr_trimmed", "translations/ROSCO_Helpers/parseinput_int_opt.cpp",
     "std::string int2lstr_trimmed(int Num) {", "}"),
    ("char_assign", "translations/ROSCO_Helpers/findline.cpp",
     "void char_assign(char* dst, int len_dst, const char* src, int len_src) {", "}"),
]


def extract(path: pathlib.Path, first: str, last: str) -> str | None:
    text = path.read_text()
    i = text.find(first)
    if i < 0:
        return None
    j = text.find("\n" + last + "\n", i)
    if j < 0:
        return None
    return text[i:j + len(last) + 2]


def main() -> int:
    out = [
        "ParseInput_Str_Opt -- the copied blocks, re-extracted from their SOURCE",
        "files and compared byte for byte against the shipped translation.",
        "",
        f"{'block':22s} {'bytes':>7s}  {'sha256[:16]':16s}  source",
    ]
    bad = 0
    total = 0
    for label, src, first, last in BLOCKS:
        a = extract(MINE, first, last)
        b = extract(ROOT / src, first, last)
        if a is None:
            out.append(f"{label:22s}  NOT FOUND in the translation")
            bad += 1
            continue
        if b is None:
            out.append(f"{label:22s}  NOT FOUND in {src}")
            bad += 1
            continue
        h = hashlib.sha256(a.encode()).hexdigest()[:16]
        total += len(a)
        if a == b:
            out.append(f"{label:22s} {len(a):7d}  {h}  {src}")
        else:
            out.append(f"{label:22s} {len(a):7d}  {h}  {src}   DIFFERS")
            bad += 1
    out.append(f"{'':22s} {'-' * 7}")
    out.append(f"{'':22s} {total:7d} bytes, "
               + ("all IDENTICAL" if not bad else f"{bad} block(s) DIFFER"))
    out += [
        "",
        "NOT CHECKED, and named rather than hidden:",
        "  assign_errmsg      copied from parseinput_int_opt.cpp with this unit's",
        "                     name in two fprintf diagnostics -- a copy with a",
        "                     stated edit, not a byte copy.",
        "  the subroutine     copied in SHAPE from parseinput_int_opt.cpp: same",
        "  body               arms, same callee calls, same order. The item half",
        "                     (the READ and the PRINT) is this unit's own, and is",
        "                     priced against gfortran in record_form_probe.txt and",
        "                     print_record_probe.txt rather than copied from",
        "                     anywhere.",
        "",
        "WHY char_assign COMES FROM findline.cpp AND NOT FROM THE SIBLING.",
        "#57 has no char_assign; its item is an INTEGER. The first draft of this",
        "unit wrote the obvious form --",
        "",
        "    const int n = std::min(len_dst, len_src);",
        "    if (n > 0)        std::memcpy(dst, src, n);",
        "    if (len_dst > n)  std::memset(dst + n, ' ', len_dst - n);",
        "",
        "-- and its first sweep left exactly the three mutants unit #32's comment",
        "already names at GetWords' version of the same expression: the swapped",
        "`min` arguments, `n > 0` -> `n >= 0`, and `len_dst > n` -> `>=`. All three",
        "are behaviour-preserving. Unit #7's rule is to delete the SITE rather than",
        "declare them, and unit #32 had already written the site that does it.",
    ]
    text = "\n".join(out) + "\n"
    (ROOT / "evidence/ParseInput_Str_Opt/p4_blocks.txt").write_text(text)
    print(text, end="")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
