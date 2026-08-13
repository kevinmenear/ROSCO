#!/usr/bin/env python3
"""Does each unit's red test measure the same corpus as the green it certifies?

WHY THIS EXISTS. Unit #26's own evidence README tabulated three stub red tests
as "363 / 4 / 361 of 403". All three were taken at a 377-case corpus and their
committed JSON says so; when the corpus was widened by the constant-step block,
only the GREEN was re-taken at 403 and the three red numbers were carried down.

That is not a transcription slip, it is a class. A red test says "this green can
go red". Widen the corpus and the green is a different measurement, while the
red test on disk still looks like its pair -- the two artifacts sit in the same
directory with no field relating them, and every one of them already records the
one number that would settle it.

So: compare `checked` in harness/<U>.json against `checked` in
harness/<U>.redtest.json, and the same for the post-integration pair. A unit
whose counts differ has a red test taken against a corpus its green has outgrown
(or, in at least one case here, the reverse).

READS ONLY. It changes nothing and decides nothing -- what to do about a skewed
pair is a question about what closes a unit, which is X3 and the Driver's call.
A unit with no pre-integration red test at all is reported separately, because
"not comparable" and "comparable and equal" are different answers and collapsing
them is the failure this campaign keeps finding (P10: name the set).

    python3 evidence/unwrap/redtest_corpus_skew.py

Exit 0 always: this is a census, not a gate.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent


def checked(path: Path):
    try:
        return json.loads(path.read_text()).get("checked")
    except (OSError, ValueError):
        return None


def census(units, green_tmpl: str, red_tmpl: str, label: str) -> int:
    rows, missing = [], []
    for n in units:
        g, r = ROOT / green_tmpl.format(n=n), ROOT / red_tmpl.format(n=n)
        if not g.exists():
            missing.append((n, "no green"))
            continue
        if not r.exists():
            missing.append((n, "no red test"))
            continue
        gc, rc = checked(g), checked(r)
        rows.append((n, gc, rc, gc == rc))

    print(f"\n=== {label} ===\n")
    print(f"{'unit':26s} {'green':>7s} {'redtest':>8s}  verdict")
    for n, gc, rc, ok in rows:
        print(f"{n:26s} {gc:>7} {rc:>8}  {'ok' if ok else 'SKEW'}")

    skewed = [(n, gc, rc) for n, gc, rc, ok in rows if not ok]
    print(f"\npairs compared : {len(rows)}")
    print(f"SKEWED         : {len(skewed)}")
    for n, gc, rc in skewed:
        d = rc - gc
        print(f"   {n:24s} green {gc}, red test {rc}  ({d:+d} cases in the red test)")
    print(f"not comparable : {len(missing)}")
    for n, why in missing:
        print(f"   {n:24s} {why}")
    return len(skewed)


def main() -> int:
    plan = json.loads((ROOT / "plan.json").read_text())
    units = [u["name"] for u in plan["units"] if u.get("disposition")]
    print(f"units with a disposition: {len(units)}")

    census(units, "harness/{n}.json", "harness/{n}.redtest.json",
           "pre-integration: the green against the CLEAN Fortran, and its red test")
    census(units,
           "harness/{n}.postintegration.json", "harness/{n}.postintegration.redtest.json",
           "post-integration: the wrapper's green, and its red test")
    return 0


if __name__ == "__main__":
    sys.exit(main())
