#!/usr/bin/env python3
"""Union sliced/operator-filtered dbgmutate runs into one artifact, or refuse.

`scripts/_mutation_merge.py` does this for `vit_mutate.py` artifacts and is NOT
reused here, for one reason that is not cosmetic: it refuses any part whose
`compared_against` is not the literal `fortran_reference_on_a_clean_tree`. That
string is a claim about a CONFIGURATION -- unit #29's finding -- and this
instrument's configuration is a different one that happens to be sound for the
same reason: the reference is a committed archive of bytes written by a build
containing no C++ `Debug` at all. Passing that check by renaming the field would
be asserting a configuration this run does not have.

Everything else is that script's discipline, kept:

  * no part without an `operators_filter` -- an unfiltered run is a whole sweep
  * no two parts covering the same (operator, slice)
  * the operator population is asked of `harness.cppmutate` DIRECTLY, never
    derived from the parts, so "did the union cover the sweep" can fail
  * per operator, the parts' mutant totals must sum to that operator's own
    population, so a part that scored a subset of its own operator cannot pass
  * survivor ids must not repeat
  * the parts must agree about the oracle, the reference archive and the
    scenarios, and must have been produced by the same instruments

The grades are not authored here. Every `killed`, `nocompile` and survivor came
out of `dbgmutate.py`; this sums fields and recomputes the two derived numbers.

    python3 scripts/dbgmutate_merge.py --unit Debug \\
        --cpp rosco/controller/src/debug.cpp \\
        --part mutation/Debug.compare_op.0.json ... --out mutation/Debug.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOOP = os.environ.get("LOOP_ROOT", str(ROOT.parent / "translation-loop"))
sys.path.insert(0, LOOP)
from harness.cppmutate import mutants  # noqa: E402

NOCOMPILE_LIMIT = 0.25


def die(msg: str) -> int:
    print(f"dbgmutate_merge: REFUSING -- {msg}", file=sys.stderr)
    return 2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--unit", required=True)
    ap.add_argument("--cpp", required=True)
    ap.add_argument("--part", action="append", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--why", required=True)
    ap.add_argument("--equivalences", default=None,
                    help="JSON {id: reason} of mutants declared EQUIVALENT. "
                         "Applied here rather than inside dbgmutate.py, for two "
                         "reasons. An equivalence is a claim about the PROGRAM "
                         "and does not change what the sweep must run, so "
                         "binding it to the run means re-running the sweep to "
                         "change a claim. And here the claim can be REFUTED: a "
                         "mutant declared equivalent that the sweep KILLED is a "
                         "wrong declaration, and this refuses rather than "
                         "quietly dropping it from the denominator.")
    a = ap.parse_args()

    parts = []
    for p in a.part:
        f = ROOT / p
        if not f.is_file():
            return die(f"no such part: {p}")
        parts.append((p, json.loads(f.read_text())))

    for name, d in parts:
        if d.get("unit") != a.unit:
            return die(f"{name} is for unit {d.get('unit')!r}")
        if not d.get("operators_filter"):
            return die(f"{name} has no operators_filter -- an unfiltered run is "
                       f"not a part of a union, it is a whole sweep")
        if d.get("operators_filter_unmatched"):
            return die(f"{name} names operator(s) that reached no site: "
                       f"{d['operators_filter_unmatched']}")

    # (operator, slice) must not repeat.
    seen: dict[tuple, str] = {}
    for name, d in parts:
        for op in d["operators_filter"]:
            key = (op, d.get("slice"))
            if key in seen:
                return die(f"{key} is in both {seen[key]} and {name}; those "
                           f"mutants would be counted twice")
            seen[key] = name

    population = Counter(m.operator for m in
                         mutants(a.unit.lower(), (ROOT / a.cpp).read_text()))
    if not population:
        return die("harness.cppmutate produced no mutants for this file")
    missing = set(population) - {op for op, _ in seen}
    if missing:
        return die(f"operator(s) {sorted(missing)} found a site in this unit and "
                   f"are in NO part -- this union does not cover the sweep")
    extra = {op for op, _ in seen} - set(population)
    if extra:
        return die(f"part(s) filter on {sorted(extra)}, which found no site here")

    # Per operator, the parts must sum to the population -- this is what makes a
    # SLICED part safe: a missing slice shows up as a short sum.
    got = Counter()
    for _, d in parts:
        for r in d["results"]:
            got[r["operator"]] += 1
    for op, want in population.items():
        if got[op] != want:
            return die(f"operator {op!r}: the parts scored {got[op]} mutant(s) "
                       f"but the mutator produces {want} -- a slice is missing")

    for field in ("oracle", "reference_archive", "compared_against"):
        vals = {json.dumps(d.get(field)) for _, d in parts}
        if len(vals) != 1:
            return die(f"the parts disagree about {field}: {vals}")
    scen = {tuple(d["scenarios"]) for _, d in parts}
    if len(scen) != 1:
        return die(f"the parts ran different scenarios: {scen}")
    revs = {(d.get("loop_rev"), d.get("vit_rev")) for _, d in parts}
    if len(revs) != 1:
        return die(f"the parts were produced by different instruments: {revs}")

    survivors, sid = [], {}
    for name, d in parts:
        for s in d["survivors"]:
            if s["id"] in sid:
                return die(f"survivor {s['id']} appears in both {sid[s['id']]} "
                           f"and {name}")
            sid[s["id"]] = name
            survivors.append(s)

    total = sum(d["total"] for _, d in parts)
    beh = sum(d["mutants"] for _, d in parts)
    nocompile = sum(d["nocompile"] for _, d in parts)
    killed = sum(d["killed"] for _, d in parts)
    survived = sum(d["survived"] for _, d in parts)
    eq = sum(d["equivalent_declared"] for _, d in parts)

    declared: dict[str, str] = {}
    if a.equivalences:
        f = ROOT / a.equivalences
        if not f.is_file():
            return die(f"no such equivalences file: {a.equivalences}")
        declared = json.loads(f.read_text())
        if eq:
            return die(f"the parts already declare {eq} equivalence(s); "
                       f"--equivalences would double-count them")
        by_id = {r["mid"]: r for _, d in parts for r in d["results"]}
        unknown = sorted(set(declared) - set(by_id))
        if unknown:
            return die(f"declared equivalent but not in this sweep: {unknown}")
        refuted = sorted(i for i in declared if by_id[i]["killed"])
        if refuted:
            return die(f"declared EQUIVALENT and yet KILLED by this corpus: "
                       f"{refuted} -- the declaration is wrong, not the sweep")
        nocomp = sorted(i for i in declared if by_id[i]["outcome"] == "nocompile")
        if nocomp:
            return die(f"declared equivalent but did not compile: {nocomp} -- a "
                       f"mutant that is not in the behavioural set cannot be "
                       f"removed from its denominator")
        eq = len(declared)
    if beh + nocompile != total:
        return die(f"{beh} behavioural + {nocompile} nocompile != {total}")
    if killed + survived != beh:
        return die(f"{killed} killed + {survived} survived != {beh}")
    if len(survivors) != survived:
        return die(f"{len(survivors)} survivor records but survived={survived}")

    denom = beh - eq
    ratio = nocompile / total if total else 0.0
    first = parts[0][1]
    doc = {
        "unit": a.unit,
        "merged_from": [n for n, _ in parts],
        "why_split": a.why,
        "total": total,
        "mutants": beh,
        "killed": killed,
        "survived": survived,
        "equivalent_declared": eq,
        "score": killed / denom if denom else 0.0,
        "nocompile": nocompile,
        "nocompile_ratio": ratio,
        "nocompile_refused": bool(total and ratio > NOCOMPILE_LIMIT),
        "operators": sorted(population),
        "operator_population": dict(population),
        "limit_per_operator": first["limit_per_operator"],
        "capped_operators": first["capped_operators"],
        "scenarios": first["scenarios"],
        "reference_archive": first["reference_archive"],
        "compared_against": first["compared_against"],
        "oracle": first["oracle"],
        "survivors": survivors,
        # Named, with the reason, IN the artifact: an equivalence that lives
        # only in prose is a number nobody can check.
        "equivalences": declared,
        "equivalences_file": a.equivalences,
        "loop_rev": first.get("loop_rev"),
        "vit_rev": first.get("vit_rev"),
        "campaign_rev": first.get("campaign_rev"),
    }
    out = ROOT / a.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, indent=2, sort_keys=True))
    print(f"{killed}/{denom} = {doc['score']:.4f}  "
          f"({beh} behavioural, {nocompile} nocompile of {total})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
