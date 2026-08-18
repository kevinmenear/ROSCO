#!/usr/bin/env python3
"""Which translations have an operator that `cppmutate`'s cap TRUNCATED?

WHY THIS EXISTS. `harness.cppmutate.mutants` takes `limit_per_operator=40` and
its own docstring says of it: "**It is a cap, and a silent cap reads as full
coverage** -- callers must report when it bites." `scripts/dbgmutate.py` does
report it, in `capped_operators`. `scripts/vit_mutate.py` did not, for the whole
of rosco-r2 up to loop 4751161 -- so every mutation artifact this campaign has
written through that path states a population that is the mutator's OFFER and
reads as the translation's.

Found on unit #53. IPC's artifact says 118 mutants; the .cpp has 157 sites, 79
of them `const_tweak` against a cap of 40. The score was right about what it
scored and silent about what it was never offered.

This asks every translation the same question, by enumerating twice -- once at
the cap the mutators use, once at a bound nothing reaches -- and reporting the
difference. It reads the .cpp files and the committed artifacts; it never
builds, never mutates, and never writes into mutation/.

    python3 scripts/mutation_cap_audit.py                 # table to stdout
    python3 scripts/mutation_cap_audit.py --json out.json

THE COUNT IS NOT THE SCORE, and this cannot compute what the score WOULD have
been: the unenumerated mutants were never compiled or run, so whether they would
have died is unknown and not 'none' (P6). What it can say is how much of each
unit's population was outside the sweep, which is the number a reader needs to
know how far a score reaches.

Exit 0 always -- it is a reader. A non-empty table is the finding, not an error.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONTAINER = os.environ.get("VIT_CONTAINER", "vit-dev")
WORKDIR = f"/workspace/{ROOT.name}"
LOOP = "/workspace/translation-loop"
CAP = 40          # cppmutate's default, and what both mutators pass
UNBOUNDED = 10 ** 9


def populations() -> dict[str, dict[str, int]]:
    """{stem: {operator: uncapped count}} for every committed translation.

    Enumerated INSIDE the container, from the same `harness.cppmutate` the
    mutators import, so the answer is the instrument's and not a second
    implementation of its rules (P4).
    """
    prog = (
        "import sys, glob, json, collections; sys.path.insert(0, %r);"
        "from harness.cppmutate import mutants;"
        "out = {};"
        "[out.__setitem__(f.split('/')[-1][:-4], dict(collections.Counter("
        "m.operator for m in mutants(f.split('/')[-1][:-4], open(f).read(),"
        " limit_per_operator=%d))))"
        " for f in sorted(glob.glob(%r))];"
        "print(json.dumps(out))"
        % (LOOP, UNBOUNDED, f"{WORKDIR}/translations/*/*.cpp")
    )
    r = subprocess.run(["docker", "exec", CONTAINER, "python3", "-c", prog],
                       capture_output=True, text=True)
    if r.returncode != 0 or not r.stdout.strip():
        print(r.stderr.strip()[-800:], file=sys.stderr)
        # An empty population is not "nothing was capped" -- it is "the
        # question was not asked", and the two must not render the same (P6).
        raise SystemExit("mutation_cap_audit: could not reach harness.cppmutate; "
                         "no answer, which is not the same as no finding")
    return json.loads(r.stdout)


def scored() -> dict[str, int]:
    """{Unit: mutants recorded in its committed mutation/<Unit>.json}."""
    out: dict[str, int] = {}
    for p in sorted(glob.glob(str(ROOT / "mutation" / "*.json"))):
        name = Path(p).stem
        if "." in name:            # a part, an ablation, a probe -- not the unit
            continue
        try:
            d = json.loads(Path(p).read_text())
        except Exception:
            continue
        if isinstance(d, dict) and "mutants" in d:
            out[name] = int(d.get("total") or d.get("mutants") or 0)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    pops = populations()
    have = scored()
    lower = {k.lower(): k for k in have}

    rows = []
    for stem, ops in sorted(pops.items()):
        capped = {o: n for o, n in ops.items() if n >= CAP}
        if not capped:
            continue
        uncapped_total = sum(ops.values())
        offered = sum(min(n, CAP) for n in ops.values())
        unit = lower.get(stem)
        rows.append({
            "stem": stem,
            "unit": unit,
            "capped_operators": dict(sorted(capped.items())),
            "population_uncapped": uncapped_total,
            "population_offered": offered,
            "never_enumerated": uncapped_total - offered,
            "artifact_total": have.get(unit) if unit else None,
        })

    print(f"cppmutate cap = {CAP} mutants per operator. "
          f"{len(rows)} of {len(pops)} translation(s) have an operator at or "
          f"over it.\n")
    print(f"{'translation':26s} {'offered':>8s} {'actual':>8s} {'unseen':>7s}  "
          f"{'artifact':>8s}  capped operators")
    for r in rows:
        art = "-" if r["artifact_total"] is None else str(r["artifact_total"])
        ops = ", ".join(f"{o}:{n}" for o, n in r["capped_operators"].items())
        print(f"{r['stem']:26s} {r['population_offered']:8d} "
              f"{r['population_uncapped']:8d} {r['never_enumerated']:7d}  "
              f"{art:>8s}  {ops}")
    print(f"\n{sum(r['never_enumerated'] for r in rows)} mutant(s) across these "
          f"translations were never enumerated by any sweep.")
    print("`offered` is what the mutator returns at the cap; `actual` is what "
          "the .cpp has; `unseen` is the difference.")
    print("`artifact` is the committed mutation/<Unit>.json total, blank where "
          "the unit has none. Where it is below `offered` the unit was also")
    print("scored under a --limit or is missing operators; where it equals "
          "`offered` the sweep was complete AS OFFERED and short of `actual`.")

    if a.json:
        Path(a.json).write_text(json.dumps(
            {"cap": CAP, "translations_examined": len(pops),
             "translations_capped": len(rows), "rows": rows}, indent=1) + "\n")
        print(f"\nwrote {a.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
