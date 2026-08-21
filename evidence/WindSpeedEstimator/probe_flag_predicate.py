#!/usr/bin/env python3
"""Family J: is `1` at an LPFilter flag argument a VALUE or a TRUTH?

Two `const_tweak` survivors change a literal `1` that this unit hands to
`lpfilter_c` into a `2`:

    :348   LocalVar->restart ? 1 : 0        ->  ? 2 : 0     (the LOGICAL `reset`)
    :349   ..., 1, LocalVar->WE_Vw)         ->  ..., 2, ... (`has_InitialValue`)

Both are behaviour-preserving IFF every implementation that consumes them tests
"non-zero" rather than "equal to 1". That is a claim about the CALLEE, not
about this unit, and it has to hold in BOTH configurations the campaign runs:

  * the mutation sweep and the differential harness link the generated KERNEL
    CALLEE BRIDGE, which calls the Fortran `LPFilter`;
  * the integrated build links the translated `lpfilter_c` in
    `rosco/controller/src/lpfilter.cpp`.

So both are read here, and neither is taken on trust.

THE CONTROL, which is the point of the file (P10): a detector that only ever
answers "no `== 1` found" cannot fail. The same detector is run over a synthetic
source that DOES compare the flag against 1, and it must find it.

    python3 evidence/WindSpeedEstimator/probe_flag_predicate.py

Exit 0 if all four checks hold, 1 otherwise.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent

# `reset` and `has_InitialValue` are the two dummies under test. A predicate
# that decides on the VALUE rather than on non-zero-ness is what would break
# the equivalence: `== 1`, `.EQ. 1`, `== 1_C_INT`, or a select/case on it.
VALUE_TEST = re.compile(
    r"\b(reset|has_InitialValue)\b\s*(==|\.EQ\.|/=\s*1\b(?!\s*\))|\.NE\.\s*1\b)\s*1?",
    re.IGNORECASE)
# The truth tests that make the equivalence hold.
TRUTH_TEST = re.compile(
    r"\b(reset|has_InitialValue)\b\s*(/=|!=)\s*0\b|if\s*\(\s*has_InitialValue\s*\)",
    re.IGNORECASE)


def scan(text: str) -> tuple[list[str], list[str]]:
    value, truth = [], []
    for line in text.splitlines():
        if line.lstrip().startswith(("!", "//", "*")):
            continue                       # comments are not the program
        if VALUE_TEST.search(line):
            value.append(line.strip())
        if TRUTH_TEST.search(line):
            truth.append(line.strip())
    return value, truth


def main() -> int:
    targets = [
        ("kernel callee bridge (Fortran, what the sweep links)",
         ROOT / "translations/ControllerBlocks/windspeedestimator_test"
              / "windspeedestimator_callees.f90"),
        ("integrated translation (C++, what the gate links)",
         ROOT / "rosco/controller/src/lpfilter.cpp"),
    ]
    ok = True
    for label, path in targets:
        if not path.is_file():
            print(f"MISSING  {label}: {path}")
            ok = False
            continue
        value, truth = scan(path.read_text())
        print(f"{label}")
        print(f"    value tests on the flags (must be 0): {len(value)}")
        for l in value:
            print(f"        {l}")
        print(f"    non-zero tests on the flags (must be > 0): {len(truth)}")
        for l in truth:
            print(f"        {l}")
        if value or not truth:
            ok = False

    # THE CONTROL. The same detector over a source that decides on the value.
    synthetic = (
        "        IF (has_InitialValue == 1) THEN\n"
        "        if (reset == 1) { }\n"
    )
    cvalue, _ = scan(synthetic)
    print(f"CONTROL  synthetic source comparing the flags against 1: "
          f"{len(cvalue)} value test(s) found (must be 2)")
    for l in cvalue:
        print(f"        {l}")
    if len(cvalue) != 2:
        ok = False

    print("ALL FOUR CHECKS HOLD -- `1` at these two arguments is a TRUTH"
          if ok else "REFUTED -- read the lines above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
