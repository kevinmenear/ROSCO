#!/usr/bin/env python3
"""Print WHERE each surviving mutant is, in the translation's own source.

A survivor list of `{"id": "4930efc0", "operator": "compare_op", "before": "==",
"after": "!="}` names an edit and not a site: `==` occurs 30 times in this
translation. Triaging a survivor -- equivalent, unreachable by the corpus, or a
gap -- starts with the line it sits on, and re-deriving that by hand is how a
survivor gets attributed to the wrong statement.

The mapping is recomputed from `harness.cppmutate`, the same generator that
produced the ids, over the translation AS COMMITTED. It is therefore a check as
well as a lookup: an id in the artifact that this cannot place means the
artifact and the source have diverged, and it says so rather than skipping it.

    python3 evidence/CheckInputs/survivor_sites.py CheckInputs
    python3 evidence/CheckInputs/survivor_sites.py CheckInputs --rev HEAD

`--rev` reads the translation out of git rather than the working tree, which
matters while a sweep is running: `vit_mutate.py` mutates the .cpp IN PLACE, so
the working tree holds a mutant until it finishes.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOOP = Path("/Users/kmenear/Artifacts/vit_translation/translation-loop")
if not LOOP.is_dir():
    LOOP = Path("/workspace/translation-loop")
sys.path.insert(0, str(LOOP))

from harness.cppmutate import mutants  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("unit")
    ap.add_argument("--artifact", default=None)
    ap.add_argument("--rev", default=None, help="read the .cpp from this git rev")
    a = ap.parse_args()

    art = Path(a.artifact) if a.artifact else ROOT / "mutation" / f"{a.unit}.json"
    doc = json.loads(art.read_text())
    plan = json.loads((ROOT / "plan.json").read_text())
    spec = next(u for u in plan["units"] if u["name"] == a.unit)
    rel = spec["translation"]

    if a.rev:
        src = subprocess.run(["git", "-C", str(ROOT), "show", f"{a.rev}:{rel}"],
                             capture_output=True, text=True, check=True).stdout
    else:
        src = (ROOT / rel).read_text()
    lines = src.splitlines()

    sites = {}
    for m in mutants(a.unit.lower(), src):
        ln = src.count("\n", 0, m.pos) + 1
        sites[m.mid] = (ln, lines[ln - 1].strip())

    survivors = doc.get("survivors") or []
    print(f"{art.name}: {len(survivors)} survivor(s) of {doc.get('mutants')} "
          f"mutant(s); score {doc.get('score')}")
    print(f"source: {rel} @ {a.rev or 'working tree'}\n")
    unplaced = 0
    for s in survivors:
        site = sites.get(s["id"])
        if site is None:
            unplaced += 1
            print(f"  {s['id']}  {s['operator']:14s} {s['before']!r} -> {s['after']!r}"
                  f"   NOT IN THIS SOURCE -- artifact and source have diverged")
            continue
        ln, text = site
        print(f"  {s['id']}  {s['operator']:14s} {s['before']!r} -> {s['after']!r}"
              f"   line {ln}: {text[:96]}")
    if unplaced:
        print(f"\n{unplaced} survivor(s) could not be placed. The artifact was "
              f"produced from a different revision of {rel}.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
