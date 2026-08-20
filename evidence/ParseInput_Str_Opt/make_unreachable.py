#!/usr/bin/env python3
"""Re-derive `mutation/ParseInput_Str_Opt.unreachable.json` from the MEASURED line
coverage, and refuse if the measurement is not of this corpus.

    python3 evidence/ParseInput_Str_Opt/make_unreachable.py

COPIED from `evidence/ParseInAry_Opt/make_unreachable.py` (P4) -- itself a copy
of `evidence/ParseDbAry_Opt`'s -- with the unit's two names changed, the
NOCOMPILE set re-measured for this translation, and ONE addition: a
CONTINUATION line, which gcov marks `-` rather than `#####` while the statement
it belongs to is `#####`. Nothing else about the derivation is new.

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

COV = ROOT / "evidence/ParseInput_Str_Opt/line_coverage.txt"
CPP = ROOT / "translations/ROSCO_Helpers/parseinput_str_opt.cpp"
HARNESS = ROOT / "harness/ParseInput_Str_Opt.json"
OUT = ROOT / "mutation/ParseInput_Str_Opt.unreachable.json"


def main() -> int:
    from harness.cppmutate import mutants

    checked = json.loads(HARNESS.read_text())["checked"]
    text = COV.read_text()

    # THE COVERAGE FILE MUST NAME THIS CORPUS. Without this the script happily
    # re-derives declarations from a stale measurement, which is the exact
    # failure it exists to stop.
    if f"{checked:,}-case" not in text:
        print(f"REFUSING: {COV} does not name a {checked:,}-case corpus -- "
              f"re-run evidence/ParseInput_Str_Opt/run_line_coverage_probe.sh",
              file=sys.stderr)
        return 2
    ctrl = re.search(r"L\d+\s+count=(\d+)\s+void ParseInput_Str_Opt", text)
    if not ctrl or int(ctrl.group(1)) != checked:
        print(f"REFUSING: the coverage file's entry-line control is "
              f"{ctrl.group(1) if ctrl else 'absent'}, not {checked}",
              file=sys.stderr)
        return 2

    # ONLY THE LINES UNDER THE "LINES NEVER EXECUTED" HEADING, AND THAT IS A
    # REPAIR RATHER THAN A REFINEMENT (C12).
    #
    # The inherited version matched `^  L\s*(\d+)\s` over the WHOLE file, and
    # the file's own CONTROL line has that shape:
    #
    #     CONTROL: the entry line's count must equal the case count.
    #       L445  count=12059  void ParseInput_Str_Opt(char* FileLines, ...
    #
    # so the unit's ENTRY LINE -- the one line that provably runs on every case
    # -- was in the never-executed set. It cost nothing on this unit and on the
    # sibling because `cppmutate` offers no mutant on either entry line, so the
    # set was never asked about it. That is luck, not design: a signature with a
    # literal in it (a default argument, an array bound) would have had the
    # campaign declare a mutant on the entry line UNREACHABLE, with the coverage
    # file's own control as the evidence for it.
    #
    # The heading is where the list starts, so the split is on the heading.
    body = text.split("LINES NEVER EXECUTED:", 1)
    if len(body) != 2:
        print("REFUSING: the coverage file has no 'LINES NEVER EXECUTED:' "
              "heading -- its format changed and this parser is guessing",
              file=sys.stderr)
        return 2
    zero = {int(m.group(1)) for m in re.finditer(r"^  L\s*(\d+)\s", body[1], re.M)}
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
    # `mutation/ParseInput_Str_Opt.equivalences.md` as one nonzero IOSTAT for
    # another under a single `!= 0` reader.
    equiv = set(json.loads((ROOT / "mutation/ParseInput_Str_Opt.equivalences.json"
                            ).read_text()))

    # AND A MUTANT THAT DOES NOT COMPILE IS NOT DECLARED EITHER, because it is
    # already excluded from both sides of the ratio and declaring it makes the
    # merge's arithmetic double-count it:
    #
    #   _mutation_merge: REFUSING -- 128 killed + 22 survived + 17 equivalent
    #                    + 19 unreachable != 185 behavioural
    #
    # This unit's nocompile set is READ from `nocompile_ids.json`, which
    # `evidence/ParseInput_Str_Opt/make_nocompile_ids.py` writes by asking the
    # SWEEP's own artifact which mutants did not compile -- so the two cannot
    # disagree, and the control is that the count equals the sweep's
    # `nocompile` field. Named rather than filtered by a pattern: a rule that
    # guessed which edits compile would be a second implementation of the
    # compiler.
    NOCOMPILE = set(json.loads(
        (ROOT / "evidence/ParseInput_Str_Opt/nocompile_ids.json").read_text()))

    # FOUR MUTANTS THAT SIT OUTSIDE THE `#####` SET AND ARE STILL UNREACHABLE,
    # AND THEY ARE KEYED BY SITE RATHER THAN BY MUTANT ID.
    #
    # The inherited version of this script carried a hand exception keyed by
    # mutant id, and the RUNBOOK records what that cost: a repair moved the
    # occurrence index, `4a9b3707` re-pointed onto a line the corpus runs 1,120
    # times, and the sweep refuted a declaration nobody had re-derived. A mutant
    # id is an OCCURRENCE INDEX. So the key here is
    # `(operator, before, after, source line)` -- the tuple the RUNBOOK
    # prescribes -- and a key that no longer matches simply produces no
    # declaration, which fails LOUDLY as an open survivor rather than quietly as
    # a false one.
    #
    # WHY THESE FOUR NEED AN EXCEPTION AT ALL. The coverage rule below declares a
    # mutant when gcov reports `#####` for ITS OWN line. All four of these edit a
    # line gcov marks `-` (a `constexpr` definition, or a stack declaration
    # inside a function that is never entered), so the rule cannot see them --
    # and yet the sites are as unreachable as the `#####` block itself.
    #
    #   `:98`, `:104`  MEASURED AT THE SITE, not argued:
    #                  evidence/ParseInput_Str_Opt/boundary_probe.txt runs
    #                  `GetWords` a second and a third time per case, with the
    #                  line one byte longer and with the elements one byte wider,
    #                  and reports 0 of 13,926 cases in which either answer
    #                  differs. Each constant has exactly one live consumer.
    #
    #   `:108`, `:168` BOTH LIVE INSIDE `int2lstr_trimmed`, whose body lines
    #                  (`:167`, `:169`, `:170`) are `#####` over the whole
    #                  corpus, because its only call site is in the READ-error
    #                  arm at `:396-414` -- also `#####`. That arm is dead in the
    #                  PROGRAM, not merely unexercised: an `A` edit descriptor
    #                  with no field width takes `w = LEN(item)` and a short
    #                  internal record is blank-padded, so IOSTAT is 0 on every
    #                  input (evidence/ParseInput_Str_Opt/record_form_probe.txt,
    #                  144 of 144).
    SITE_UNREACHABLE: dict[tuple, str] = {
        ("const_tweak", "2048", "2049", 98): (
            "parseinput_str_opt.cpp:98 defines MaxLineLength, whose only LIVE "
            "consumer is `getwords_c(Line.data(), MaxLineLength, ...)` -- the "
            "other two uses are a std::vector size (the mutant allocates MORE, "
            "never less) and a `ftrim` inside the READ-error arm, which no case "
            "runs. MEASURED at the site over the corpus: calling GetWords again "
            "with the line one byte longer changes Words(1) on 0 of 13,926 "
            "cases and Words(2) on 0 of 13,926. The mechanism is truncation -- "
            "GetWords writes each word into a 200-byte element, so a word "
            "running to byte 2048 is already cut off; only a word ENDING at "
            "2048 and shorter than 200 could tell the widths apart, and no case "
            "of this corpus has one."),
        ("const_tweak", "200", "201", 104): (
            "parseinput_str_opt.cpp:104 defines MaxParamLength, which reaches "
            "`char_assign(Variable, len_Variable, Words1, len_src)` as `len_src` "
            "and is used there only in the test `i <= len_src` over "
            "`i = 1 .. len_Variable`. 200 and 201 can differ only at i = 201, "
            "which needs LEN(Variable) >= 201; the corpus's LEN(Variable) "
            "ladder is {1, 2, 6, 7, 11}. MEASURED at the site: Words(1)'s first "
            "200 bytes differ on 0 of 13,926 cases. The 201st byte of Words(1) "
            "IS non-blank on 28 cases, so the corpus does carry long enough "
            "words -- the length ladder is the whole cause, and it is a claim "
            "about the CORPUS: every shipped caller passes 1024 or 256."),
        ("const_tweak", "11", "12", 108): (
            "parseinput_str_opt.cpp:108 defines Int2LStrLen, whose only "
            "consumers are `char buf[Int2LStrLen]` and `ftrim(buf, "
            "Int2LStrLen)` inside `int2lstr_trimmed` -- a function whose body "
            "lines :167, :169 and :170 are ##### over all 14,116 cases, because "
            "its only call site is in the READ-error arm at :396-414, also "
            "#####. No case runs the function, so no case can distinguish an "
            "11-byte buffer from a 12-byte one."),
        ("index_offset", "[Int2LStrLen]", "[Int2LStrLen + 1]", 168): (
            "parseinput_str_opt.cpp:168 is `char buf[Int2LStrLen]` inside "
            "`int2lstr_trimmed`, whose body lines :167, :169 and :170 are ##### "
            "over all 14,116 cases. The function is never entered, so the "
            "twelfth byte is written by nothing and read by nothing on every "
            "case."),
    }
    CONTINUATION: dict[str, str] = {}

    src = CPP.read_text()
    out: dict[str, dict[str, str]] = {}
    skipped_equiv, skipped_nocompile = [], []
    site_hits = 0
    for m in mutants("parseinput_str_opt", src):
        key = (m.operator, m.before, m.after, m.line)
        if key in SITE_UNREACHABLE and m.mid not in equiv:
            out[m.mid] = {
                "reason": SITE_UNREACHABLE[key],
                "evidence": ("evidence/ParseInput_Str_Opt/boundary_probe.txt"
                             if m.line in (98, 104)
                             else "evidence/ParseInput_Str_Opt/line_coverage.txt"),
            }
            site_hits += 1
            continue
        if m.mid in CONTINUATION and m.mid not in equiv:
            out[m.mid] = {"reason": CONTINUATION[m.mid],
                          "evidence": "evidence/ParseInput_Str_Opt/line_coverage.txt"}
            continue
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
                f"parseinput_str_opt.cpp:{m.line} is NEVER EXECUTED by any of the "
                f"{checked:,} cases -- gcov reports ##### for that line over the "
                f"whole corpus, at -O0. The mutant edits {m.before!r} to "
                f"{m.after!r} on a line no case runs, so no case can "
                f"distinguish it."),
            "evidence": "evidence/ParseInput_Str_Opt/line_coverage.txt",
        }
    OUT.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(f"{len(out)} declaration(s) over {len(zero)} never-executed line(s), "
          f"corpus {checked:,} -> {OUT.relative_to(ROOT)}")
    # A SITE KEY THAT NO LONGER MATCHES IS A SILENT LOSS, so it is counted and
    # refused rather than left to show up as a survivor nobody expected.
    if site_hits != len(SITE_UNREACHABLE):
        print(f"REFUSING: {len(SITE_UNREACHABLE)} site-keyed declaration(s) are "
              f"written and only {site_hits} matched a mutant. A key is keyed by "
              f"(operator, before, after, line); one of them no longer names a "
              f"site in the translation.", file=sys.stderr)
        return 2
    if skipped_nocompile:
        print(f"  {len(skipped_nocompile)} on a never-executed line that does "
              f"not COMPILE, left undeclared: {', '.join(sorted(skipped_nocompile))}")
    if skipped_equiv:
        print(f"  {len(skipped_equiv)} on a never-executed line already declared "
              f"EQUIVALENT, left there: {', '.join(sorted(skipped_equiv))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
