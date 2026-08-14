#!/usr/bin/env python3
"""Is `ChkParseData` reached by any of the 27 scenarios? Read it, don't argue it.

    python3 evidence/ChkParseData/coverage_deadness.py     # exit 0 == the unit is dead

WHAT THIS ASSERTS, and why each half is here.

`coverage/line_coverage.json` records only NON-ZERO counts -- `scripts/coverage.py`
writes `if n:` -- so an absent key means "zero hits" OR "not instrumented", and
those are different answers. The P10 control is therefore not optional, and for
this unit it does not have to be borrowed from another file: `ParseDbAry`, the
procedure containing call site 4, has hits on the lines either side of the call.

    489  SUBROUTINE ParseDbAry                50   <- the procedure IS entered
    567  IF (CheckName_) THEN                 50   <- the guard IS reached
    568      CALL ChkParseData (...)           0   <- and is FALSE every time
    572  ... after the block                  50   <- and the body continues

So the zero at 568 is a measured zero sitting between two measured non-zeros in
the same basic-block chain, which is what makes it a fact about the GUARD rather
than about instrumentation.

The other four sites are dead for a different reason and the script says which:
their enclosing procedures are never entered at all. `ParseInput` and `ParseAry`
are generic interfaces carrying both a unit-number form and a `FileLines` form;
ROSCO reads its input file into an array, so only the `_Opt` variants resolve.

Line numbers are the CLEAN baseline's (54dd134), which is what the coverage run
measured; the working tree carries an integration wrapper and its lines differ.
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
BASELINE = "54dd134"
SRC = "ROSCO_Helpers.f90"

# (line, what it is) -- at the clean baseline.
CALL_SITES = [
    (88,  "CALL ChkParseData, in ParseInput_Int"),
    (396, "CALL ChkParseData, in ParseInput_Dbl"),
    (460, "CALL ChkParseData, in ParseInput_Str"),
    (568, "CALL ChkParseData, in ParseDbAry"),
    (691, "CALL ChkParseData, in ParseInAry"),
]
ENCLOSING = [
    (51,   "SUBROUTINE ParseInput_Int"),
    (359,  "SUBROUTINE ParseInput_Dbl"),
    (423,  "SUBROUTINE ParseInput_Str"),
    (489,  "SUBROUTINE ParseDbAry"),
    (612,  "SUBROUTINE ParseInAry"),
]
UNIT_LINE = (1023, "SUBROUTINE ChkParseData")
# The P10 control: the guard beside site 4, and the statements either side of it.
CONTROL = [
    (552, "inside ParseDbAry, before the guard"),
    (567, "IF (CheckName_) THEN        <- the guard"),
    (572, "inside ParseDbAry, after the block"),
]


def total(hits: dict, line: int) -> int:
    return sum(hits.get(str(line), {}).values())


def source_line(text: str, n: int) -> str:
    lines = text.splitlines()
    return lines[n - 1].strip() if 0 < n <= len(lines) else "<out of range>"


def main() -> int:
    cov = json.loads((ROOT / "coverage" / "line_coverage.json").read_text())
    hits = cov["hits"][SRC]
    n_scen = len(cov["scenarios"])
    src = subprocess.run(["git", "-C", str(ROOT), "show", f"{BASELINE}:rosco/controller/src/{SRC}"],
                         capture_output=True, text=True, check=True).stdout

    ok = True
    print(f"coverage/line_coverage.json, {n_scen} scenario(s), {SRC} @ {BASELINE}\n")

    print("THE UNIT AND ITS FIVE CALL SITES -- every one must be ZERO")
    for line, what in [UNIT_LINE] + CALL_SITES:
        n = total(hits, line)
        flag = "" if n == 0 else "   <- NOT DEAD"
        if n:
            ok = False
        print(f"  {line:5d}  {n:6d}  {what}{flag}")
        got = source_line(src, line)
        want = "ChkParseData"
        if want.lower() not in got.lower():
            ok = False
            print(f"         LINE NUMBER IS STALE: {got!r} does not name {want}")

    print("\nTHE FIVE ENCLOSING PROCEDURES -- four never entered, ONE is")
    entered = []
    for line, what in ENCLOSING:
        n = total(hits, line)
        print(f"  {line:5d}  {n:6d}  {what}")
        if n:
            entered.append((line, what, n))
    if len(entered) != 1 or entered[0][0] != 489:
        ok = False
        print("  EXPECTED exactly ParseDbAry to be entered; the shape has changed")

    print("\nTHE P10 CONTROL -- an absent key is a ZERO and not un-instrumented,")
    print("because the guard beside the dead call is itself hit, in the same file")
    for line, what in CONTROL:
        n = total(hits, line)
        print(f"  {line:5d}  {n:6d}  {what}")
        if n == 0:
            ok = False
            print("         CONTROL IS ITSELF ZERO -- this check proves nothing")

    guard, call = total(hits, 567), total(hits, 568)
    after = total(hits, 572)
    print(f"\n  guard reached {guard} time(s); call taken {call} time(s); "
          f"body continued {after} time(s)")
    if not (guard == after and guard > 0 and call == 0):
        ok = False
        print("  EXPECTED guard == after > 0 and call == 0")
    else:
        print("  -> the guard is REACHED and FALSE every time. Site 4 is dead because")
        print("     ReadSetParameters.f90:822 and :824 pass CheckName = .FALSE.,")
        print("     not because nothing calls ParseDbAry.")

    print("\n" + ("VERDICT: the unit is DEAD in all 27 scenarios, and the control holds."
                  if ok else "VERDICT: FAILED -- see the lines above."))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
