#!/usr/bin/env python3
"""Re-derive `mutation/ParseInAry_Opt.unreachable.json` from the MEASURED line
coverage, and refuse if the measurement is not of this corpus.

    python3 evidence/ParseInAry_Opt/make_unreachable.py

COPIED from `evidence/ParseDbAry_Opt/make_unreachable.py` (P4) with the unit's
two names changed, the NOCOMPILE id re-measured for this translation, and the
prose corrected to this unit's numbers. Nothing about the derivation is new.

WHY IT IS A SCRIPT. On the sibling, 12 of 30 declarations became false the
moment the corpus widened. A declared-unreachable mutant the corpus then KILLS
fails P12 outright, which is the correct behaviour and the expensive way to find
out. The declaration set is a FUNCTION of the coverage file, so it is computed
from it rather than carried forward by hand -- and this unit's set is derived on
its FIRST use, against the 17,520-case corpus, rather than inherited from the
15,504-case one its 87 survivors were found on.

WHAT IT CLAIMS, per mutant: the line the mutant edits is reported `#####` by
gcov over the whole corpus, at -O0 so no line is zero because the optimiser
folded it. That is a claim about the CORPUS, and the sweep checks it: a mutant
in `unreachable_but_killed` is this file being wrong.

TWO CONTROLS, both read after the sweep rather than asserted here:

  * the entry line's gcov count must equal the case count (in
    `line_coverage.txt`, written by `run_line_coverage_probe.sh`);
  * NON-survivors on never-executed lines must be nocompile rather than kills --
    a mutant the corpus killed on a line it never ran would mean the coverage
    run measured a different program.

The mutant population comes from `harness.cppmutate`, the same function
`vit_mutate.py` calls, so this cannot disagree with the sweep about which
mutants exist or where they sit (P4).
"""
from __future__ import annotations

import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
LOOP = pathlib.Path("/Users/kmenear/Artifacts/vit_translation/translation-loop")
sys.path.insert(0, str(LOOP))

COV = ROOT / "evidence/ParseInAry_Opt/line_coverage.txt"
CPP = ROOT / "translations/ROSCO_Helpers/parseinary_opt.cpp"
HARNESS = ROOT / "harness/ParseInAry_Opt.json"
OUT = ROOT / "mutation/ParseInAry_Opt.unreachable.json"


def main() -> int:
    from harness.cppmutate import mutants

    checked = json.loads(HARNESS.read_text())["checked"]
    text = COV.read_text()

    # THE COVERAGE FILE MUST NAME THIS CORPUS. Without this the script happily
    # re-derives declarations from a stale measurement, which is the exact
    # failure it exists to stop.
    if f"{checked:,}-case" not in text:
        print(f"REFUSING: {COV} does not name a {checked:,}-case corpus -- "
              f"re-run evidence/ParseInAry_Opt/run_line_coverage_probe.sh",
              file=sys.stderr)
        return 2
    ctrl = re.search(r"L\d+\s+count=(\d+)\s+void ParseInAry_Opt", text)
    if not ctrl or int(ctrl.group(1)) != checked:
        print(f"REFUSING: the coverage file's entry-line control is "
              f"{ctrl.group(1) if ctrl else 'absent'}, not {checked}",
              file=sys.stderr)
        return 2

    zero = {int(m.group(1)) for m in re.finditer(r"^  L\s*(\d+)\s", text, re.M)}
    if not zero:
        print("REFUSING: no never-executed line was parsed out of the coverage "
              "file; a declaration set of zero is not a measurement", file=sys.stderr)
        return 2

    # AN ALREADY-DECLARED EQUIVALENCE WINS, and `vit_mutate.py` refuses a mutant
    # carrying both claims rather than picking for you: one says the PROGRAMS
    # agree and the other says the CORPUS never asks. The equivalence is the
    # stronger claim and it survives a corpus that later reaches the line, so a
    # mutant holding one is skipped here instead of being re-declared. Measured:
    # On this unit: `56160d94` (L299) and `6b5c6a0f` (L330), `5010` -> `5011` on
    # lines this corpus does not run, both already argued in
    # `mutation/ParseInAry_Opt.equivalences.md` as one nonzero IOSTAT for
    # another under a single `!= 0` reader.
    equiv = set(json.loads((ROOT / "mutation/ParseInAry_Opt.equivalences.json"
                            ).read_text()))

    # AND A MUTANT THAT DOES NOT COMPILE IS NOT DECLARED EITHER, because it is
    # already excluded from both sides of the ratio and declaring it makes the
    # merge's arithmetic double-count it:
    #
    #   _mutation_merge: REFUSING -- 128 killed + 22 survived + 17 equivalent
    #                    + 19 unreachable != 185 behavioural
    #
    # This unit's one nocompile is `804cac95`, `return -1` -> `1 - return` at
    # line 276 (`evidence/ParseInAry_Opt/mutation_census.txt:56`). That line IS
    # executed, so the two claims do not actually meet here -- the guard is kept
    # anyway, because it is the coverage file that decides which lines are in
    # the set and a later corpus can retire that line. Named rather than
    # filtered by a pattern: a rule that guessed which edits compile would be a
    # second implementation of the compiler.
    NOCOMPILE = {"804cac95"}

    src = CPP.read_text()
    out: dict[str, dict[str, str]] = {}
    skipped_equiv, skipped_nocompile = [], []
    for m in mutants("parseinary_opt", src):
        if m.line not in zero:
            continue
        if m.mid in equiv:
            skipped_equiv.append(m.mid)
            continue
        if m.mid in NOCOMPILE:
            skipped_nocompile.append(m.mid)
            continue
        out[m.mid] = {
            "reason": (
                f"parseinary_opt.cpp:{m.line} is NEVER EXECUTED by any of the "
                f"{checked:,} cases -- gcov reports ##### for that line over the "
                f"whole corpus, at -O0. The mutant edits {m.before!r} to "
                f"{m.after!r} on a line no case runs, so no case can "
                f"distinguish it."),
            "evidence": "evidence/ParseInAry_Opt/line_coverage.txt",
        }
    OUT.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(f"{len(out)} declaration(s) over {len(zero)} never-executed line(s), "
          f"corpus {checked:,} -> {OUT.relative_to(ROOT)}")
    if skipped_nocompile:
        print(f"  {len(skipped_nocompile)} on a never-executed line that does "
              f"not COMPILE, left undeclared: {', '.join(sorted(skipped_nocompile))}")
    if skipped_equiv:
        print(f"  {len(skipped_equiv)} on a never-executed line already declared "
              f"EQUIVALENT, left there: {', '.join(sorted(skipped_equiv))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
