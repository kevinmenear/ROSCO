#!/usr/bin/env python3
"""Evaluate the loop's own done-condition for one unit, here, on demand.

WHY THIS EXISTS. `plan.json` carried `disposition: integrated` for
ColemanTransform while the loop's done-condition returned INCOMPLETE at 12 of
13 -- because the loop crashed on its timeout before evaluating it, and nothing
in the unit session could evaluate it either. A session that cannot run the
condition it must satisfy is guessing, and the guess was wrong.

This runs `loop.done.DoneVerifier` against this repository with EXACTLY the
configuration `scripts/run_campaign.py` builds (the four globs and the four
state files, unmodified). It is a reader: it never writes to the tree.

    python3 scripts/done_check.py ColemanTransform
    python3 scripts/done_check.py ColemanTransform --baseline <sha>

Exit 0 only when the verdict is COMPLETE. Anything else exits 1, so it can gate
a commit rather than be read past.

NOTE ON THE BASELINE: the verifier walks commits in (baseline, head]. It
defaults to the commit before the unit's first functional commit, found by
searching backwards for the first commit that touches the unit's translation.
Pass --baseline explicitly when that guess is wrong; the guess is printed.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOOP = Path("/Users/kmenear/Artifacts/vit_translation/translation-loop")
if not LOOP.is_dir():  # inside the container the mount root differs
    LOOP = Path("/workspace/translation-loop")
sys.path.insert(0, str(LOOP))

from loop.done import DoneConfig, DoneVerifier, UnitSpec  # noqa: E402
from loop.gitrepo import GitRepo  # noqa: E402


def git(*args: str) -> str:
    return subprocess.run(["git", "-C", str(ROOT), *args],
                          capture_output=True, text=True, check=True).stdout.strip()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("unit")
    ap.add_argument("--baseline", default=None)
    ap.add_argument("--head", default="HEAD")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    doc = json.loads((ROOT / "plan.json").read_text())
    spec = next((u for u in doc["units"] if u["name"] == a.unit), None)
    if spec is None:
        print(f"no unit {a.unit} in plan.json", file=sys.stderr)
        return 2

    unit = UnitSpec(
        name=spec["name"],
        source_file=spec.get("source_file"),
        translation=spec.get("translation"),
        disposition=spec.get("disposition"),
        contract=spec.get("contract"),
        evidence=tuple(spec.get("evidence") or ()),
        harness_required=spec.get("harness_required"),
    )

    head = git("rev-parse", a.head)
    baseline = a.baseline
    if baseline is None:
        # The commit BEFORE the first one that touched this unit's translation.
        touching = git("log", "--format=%H", "--reverse", "--", unit.translation or "").split()
        if not touching:
            print("cannot locate a functional commit for this unit; pass --baseline",
                  file=sys.stderr)
            return 2
        baseline = git("rev-parse", f"{touching[0]}~1")
        print(f"baseline (inferred): {baseline[:7]}  "
              f"-- the commit before {touching[0][:7]} touched {unit.translation}")

    cfg = DoneConfig(
        # STATUS.md deleted 2026-08-20: a state file that had become a log.
        # P7 (a state commit exists) and P8 (the unit is named in one) both
        # resolve through DECISIONS.md and plan.json, which every close
        # commits -- so neither predicate weakens.
        state_files=("DECISIONS.md", "plan.json"),
        gate_result_glob="gate/{name}.json",
        redtest_result_glob="gate/{name}.redtest.json",
        harness_result_glob="harness/{name}.postintegration.json",
        mutation_result_glob="mutation/{name}.json",
        require_clean_tree=True,
    )
    result = DoneVerifier(GitRepo(ROOT), cfg).verify(unit, baseline, head)

    if a.json:
        print(json.dumps({
            "unit": result.name,
            "verdict": str(result.verdict),
            "predicates": [{"id": p.id, "name": p.name, "status": str(p.status),
                            "detail": p.reason} for p in result.predicates],
        }, indent=1))
    else:
        passed = sum(1 for p in result.predicates if str(p.status).endswith("PASS"))
        print(f"\n{result.name}: {result.verdict}  ({passed}/{len(result.predicates)} PASS)\n")
        for p in result.predicates:
            mark = {"PASS": "  ok  ", "FAIL": " FAIL ", "NOT_EVALUABLE": " n/e  "}
            s = str(p.status).rsplit(".", 1)[-1]
            print(f"[{mark.get(s, s):^6}] {p.id:4s} {p.name:34s} {p.reason}")
        print()

    return 0 if str(result.verdict).rsplit(".", 1)[-1] == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
