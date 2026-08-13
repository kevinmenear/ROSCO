#!/usr/bin/env python3
"""Can a unit's evidence be checked for commensurability? Three questions.

WHY THIS EXISTS. `mutation/CheckInputs.json` was scored against one corpus and
`harness/CheckInputs.postintegration.json` against another, and the unit's
done-condition passed both. `_mutation_merge.py:189` refuses parts from
different `loop_rev`s INSIDE a merge; nothing asked the same question ACROSS a
unit's artifacts. The audit that found it also found two failures more common
than the split, which is why this asserts three things and not one:

  (1) BASE SHA AGREEMENT -- every result artifact of a unit names the same
      generator commit. A unit whose corpus changed between two artifacts has
      two measurements of two different things, and nothing says so.
  (2) loop_rev PRESENT -- 17 units carry artifacts with no `loop_rev` field at
      all. That is worse than a disagreement: not a claim that fails a check,
      but a claim with no recorded input to check it against.
  (3) NO `-dirty` REV -- 12 units carry an artifact stamped at a dirty tree. A
      SHA is recorded, so it reads as identified, while naming code that is not
      what ran. It is a green that looks precise, which is the shape this
      campaign exists to remove.

WHAT AN ARTIFACT IS, and why it is not a filename list. Membership is decided by
CONTENT -- any JSON under mutation/, harness/ or gate/ belonging to this unit
and carrying at least one result key. A hardcoded family list would silently
stop covering artifacts nobody thought to add, and the difference is not
academic: on this campaign the five core families report 4 splits and every
result artifact reports 9. The extra five are redtests and operator-filtered
runs taken at a different generator revision from the artifact they qualify --
a redtest that proves a DIFFERENT instrument can go red proves nothing about
this one. `--core` restricts to the done-condition's own globs, and both counts
are always printed, because a check that quietly bounds its own coverage reads
as "all clear" when it means "as far as I looked".

ADVISORY ON CLOSED UNITS. A unit already integrated or blocked is REPORTED with
a named count and does not set the exit status in audit mode. The campaign has
27 dispositions that were taken in good faith against artifacts whose hygiene
nobody was checking; reopening them on that basis would cost more than it buys.
`--strict` makes every finding count, for the run where that is the question.

A unit with NO result artifacts is NOT_EVALUABLE and is never a pass (P6).

    python3 scripts/revcheck.py                 # audit every unit, advisory
    python3 scripts/revcheck.py --unit CheckInputs   # gate one unit, exit 2 on any finding
    python3 scripts/revcheck.py --strict        # audit, and let closed units fail it

Exit 0 clean, 2 on a finding that counts, 3 on NOT_EVALUABLE. It is a reader:
it never writes to the tree.
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DIRS = ("mutation", "harness", "gate")
RESULT_KEYS = {"checked", "killed", "mutants", "values", "mismatched",
               "scenarios", "failed", "score"}
CORE = ("mutation/{n}.json", "harness/{n}.json", "harness/{n}.postintegration.json",
        "gate/{n}.json", "gate/{n}.redtest.json")
CLOSED = ("integrated", "blocked")


def base(rev: str) -> str:
    """`2e2295f-nogit` and `2e2295f-dirty` are the same COMMIT.

    Comparing the suffixed strings reports a split that is not one -- which is
    exactly the false positive this function exists to stop, and it was found by
    a hand comparison making that mistake."""
    return rev.split("-", 1)[0]


def artifacts(names: list[str], core_only: bool) -> dict[str, dict[str, str | None]]:
    """{unit: {path: loop_rev-or-None}} over RESULT artifacts only."""
    out: dict[str, dict[str, str | None]] = collections.defaultdict(dict)
    if core_only:
        for n in names:
            for fam in CORE:
                p = fam.format(n=n)
                if (ROOT / p).is_file():
                    try:
                        out[n][p] = json.loads((ROOT / p).read_text()).get("loop_rev")
                    except Exception:
                        out[n][p] = None
        return out
    longest = sorted(names, key=len, reverse=True)   # `interp1d` before `interp1`
    for d in DIRS:
        for p in sorted(glob.glob(str(ROOT / d / "*.json"))):
            b = os.path.basename(p)
            for n in longest:
                if b == f"{n}.json" or b.startswith(f"{n}."):
                    try:
                        doc = json.loads(Path(p).read_text())
                    except Exception:
                        break
                    if isinstance(doc, dict) and (RESULT_KEYS & set(doc)):
                        out[n][os.path.relpath(p, ROOT)] = doc.get("loop_rev")
                    break
    return out


SPLIT, ABSENT, DIRTY = "BASE-SHA SPLIT", "NO loop_rev", "DIRTY TREE"
NOREAD, RETRO = "NO RECORDED READING", "RETROACTIVE READING"


def _git(*a: str) -> str:
    import subprocess
    r = subprocess.run(["git", "-C", str(ROOT), *a], capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else ""


def readings(disp: dict[str, str | None]) -> dict[str, tuple[str, str]] | None:
    """Did a CLOSED unit ever have the done-condition read WHILE IT WAS CURRENT?

    A closed unit with no `evidence/<u>/done_check.txt` has no recorded reading
    of the condition it had to satisfy. That is NOT_EVALUABLE, and the tempting
    fix -- capture one now -- is the defect this whole file exists to name: every
    predicate done_check evaluates reads the CURRENT tree, so a capture made
    today for a unit closed on 08-11 describes today while reading, to anyone
    later, as the reading taken at the close. A loud honest absence would become
    a quiet dishonest presence, and this check would go green having learned
    nothing.

    So the question is not "does a file exist" -- which generating a file would
    answer -- but "was the reading taken while this unit was still the unit being
    worked". Measured WITHOUT a time threshold: no OTHER unit may have gained a
    disposition between this unit's closing commit and its capture's commit. On
    this campaign that separates 11 contemporaneous captures (zero units in
    between) from 3 retroactive ones (19, 6 and 1) with nothing near the boundary
    -- and generating the 14 missing files today would move them from NO
    RECORDED READING to RETROACTIVE READING, not to clean.

    Returns None when git history cannot be read, which is reported and never
    silently passed."""
    log = _git("log", "--format=%H", "--reverse", "--", "plan.json")
    order = _git("log", "--format=%H", "--reverse")
    if not log.strip() or not order.strip():
        return None
    pos = {s: i for i, s in enumerate(order.split())}
    closed_at: dict[str, str] = {}
    seq: list[tuple[str, str]] = []
    for sha in log.split():
        raw = _git("show", f"{sha}:plan.json")
        if not raw:
            continue
        try:
            units = json.loads(raw)["units"]
        except Exception:
            continue
        for u in units:
            if u.get("disposition") and u["name"] not in closed_at:
                closed_at[u["name"]] = sha
                seq.append((sha, u["name"]))
    out: dict[str, tuple[str, str]] = {}
    for n, close in closed_at.items():
        if not disp.get(n):
            continue          # re-opened since; not a closed unit now
        ev = f"evidence/{n}/done_check.txt"
        cap = _git("log", "-1", "--format=%H", "--", ev).strip()
        if not cap:
            out[n] = (NOREAD, f"closed at {close[:7]} with no {ev}")
            continue
        a, b = pos.get(close, -1), pos.get(cap, -1)
        moved = [m for s, m in seq if m != n and a < pos.get(s, -1) <= b]
        if moved:
            out[n] = (RETRO, f"{ev} committed at {cap[:7]}, after the campaign "
                             f"closed {len(moved)} other unit(s): {', '.join(moved[:4])}"
                             + (" ..." if len(moved) > 4 else ""))
    return out


def findings(arts: dict[str, str | None]) -> list[tuple[str, str]]:
    """(code, detail) for each of the three assertions this unit fails.

    Artifacts are named by PATH, never basename: `mutation/unwrap.json` and
    `harness/unwrap.json` share one, and a report that collapses them shows the
    same file three times in a split it cannot then be used to diagnose. The
    code is returned separately so the summary counts on a fixed string rather
    than on a prefix of the detail, which counted zero of 17."""
    f = []
    absent = sorted(p for p, r in arts.items() if not r)
    present = {p: r for p, r in arts.items() if r}
    by = collections.defaultdict(list)
    for p, r in present.items():
        by[base(r)].append(p)
    if len(by) > 1:
        f.append((SPLIT, "  vs  ".join(f"{b} ({', '.join(sorted(v))})"
                                       for b, v in sorted(by.items()))))
    if absent:
        f.append((ABSENT, f"{len(absent)} artifact(s): {', '.join(absent)}"))
    dirty = sorted(p for p, r in present.items() if r.endswith("-dirty"))
    if dirty:
        f.append((DIRTY, ", ".join(f"{p}={present[p]}" for p in dirty)))
    return f


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--unit", default=None, help="gate ONE unit; any finding exits 2")
    ap.add_argument("--core", action="store_true",
                    help="only the done-condition's own globs, not every result artifact")
    ap.add_argument("--strict", action="store_true",
                    help="let findings on already-closed units set the exit status")
    a = ap.parse_args()

    plan = json.loads((ROOT / "plan.json").read_text())["units"]
    names = [u["name"] for u in plan]
    disp = {u["name"]: u.get("disposition") for u in plan}
    found = artifacts(names, a.core)
    reads = readings(disp)

    if a.unit:
        if a.unit not in names:
            print(f"no unit {a.unit} in plan.json", file=sys.stderr)
            return 2
        arts = found.get(a.unit, {})
        print(f"revcheck {a.unit}: examined 1 unit, scanned {len(arts)} result artifact(s)"
              f"{' [--core]' if a.core else ''}")
        if not arts:
            print("  NOT_EVALUABLE -- no result artifact carries a revision to check.")
            return 3
        fs = findings(arts)
        if reads is None:
            print("  READING CHECK UNAVAILABLE -- git history unreadable; not a pass")
        elif a.unit in reads:
            fs = fs + [reads[a.unit]]
        for p, r in sorted(arts.items()):
            print(f"    {r or '(no loop_rev)':18} {p}")
        for code, detail in fs:
            print(f"  FINDING: {code}: {detail}")
        print("  clean" if not fs else f"  {len(fs)} finding(s)")
        return 2 if fs else 0

    # ---- audit mode -------------------------------------------------------
    scanned = sum(len(v) for v in found.values())
    print(f"revcheck: examined {len(names)} planned unit(s), {len(found)} with artifacts, "
          f"scanned {scanned} result artifact(s){' [--core]' if a.core else ''}")
    if not a.core:
        core_found = artifacts(names, True)
        c = sum(1 for v in core_found.values()
                if any(k == SPLIT for k, _ in findings(v)))
        w = sum(1 for v in found.values()
                if any(k == SPLIT for k, _ in findings(v)))
        print(f"  coverage: {w} split(s) over every result artifact; {c} over the "
              f"five core families alone (--core)")

    counts = collections.Counter()
    blocking, advisory, ne = [], [], []
    for n in names:
        arts = found.get(n, {})
        if not arts:
            if disp.get(n):
                ne.append(n)
                if reads and n in reads:
                    advisory.append((n, disp.get(n), [reads[n]]))
                    counts[reads[n][0]] += 1
            continue
        fs = findings(arts)
        if reads and n in reads:
            fs = fs + [reads[n]]
        if not fs:
            continue
        for code, _ in fs:
            counts[code] += 1
        (advisory if disp.get(n) in CLOSED else blocking).append((n, disp.get(n), fs))

    for title, rows in (("OPEN units -- these count", blocking),
                        ("CLOSED units -- ADVISORY, not reopened here", advisory)):
        print(f"\n{title}: {len(rows)}")
        for n, d, fs in rows:
            print(f"  {n}  [{d or 'pending'}]")
            for code, detail in fs:
                print(f"      {code}: {detail}")
    if ne:
        print(f"\nNOT_EVALUABLE -- closed with no result artifact: {len(ne)}  {', '.join(ne)}")

    print("\nby assertion:")
    if reads is None:
        print("\n  READING CHECK UNAVAILABLE -- git history unreadable; not a pass")
    for k in (SPLIT, ABSENT, DIRTY, NOREAD, RETRO):
        print(f"  {k:24} {counts.get(k, 0)} unit(s)")

    if ne:
        return 3
    if blocking or (a.strict and advisory):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
