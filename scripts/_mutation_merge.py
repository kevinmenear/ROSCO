#!/usr/bin/env python3
"""Union operator-filtered mutation runs into one artifact, or refuse.

WHY THIS EXISTS, and why it is not the tool re-implemented. `vit_mutate.py`
scores every mutant of a unit in ONE process, which for `CheckInputs` is 192
mutants at ~9.5s each -- 32 minutes. A dispatch's foreground command may block
for 600 seconds. A 32-minute sweep therefore cannot be one command, and the
alternative -- backgrounding it -- is the failure this campaign has already paid
for twice: the sweep outlives the session, orphans in the container, and writes
into the tree minutes after the driver has moved on.

So the sweep is run as five `--operator`-filtered invocations, each in the
foreground, and this joins them. THE GRADES ARE NOT AUTHORED HERE. Every
`killed`, `nocompile` and `survivor` in the output came out of `vit_mutate.py`;
this script sums fields and recomputes the two derived numbers with the tool's
own arithmetic (score over BEHAVIOURAL mutants, nocompile excluded from both
sides; `nocompile_ratio` over ALL mutants). Nothing else is computed.

WHAT IT REFUSES, because a union is a coverage claim and a coverage claim that
cannot fail is not one (P10):

  * a part with no `operators_filter`  -- an unfiltered run is not a part
  * parts whose filters overlap WITHOUT `scored_ids` to tell them apart -- a
    mutant scored twice, counted twice. An operator too large for one
    foreground call may be split across parts by `vit_mutate.py --offset`, and
    then the partition is checked by ID: no mutant in two parts, every mutant
    of every named operator in one. See the block at `ids_ok` for why the id
    check is stronger than the count arithmetic it replaces rather than a
    relaxation of it.
  * parts that do not COVER the unit's operator set. The population is asked of
    `harness.cppmutate` directly -- the same function `vit_mutate.py` calls --
    because a part's own `operators` field is computed AFTER the filter and so
    lists only that part's operators. Deriving it from the parts would make the
    coverage check a tautology. Being a per-operator COUNT, it also settles the
    stronger question: each part's mutant total must equal the number of mutants
    its operators produce, so a part that scored a subset of its own operator's
    sites cannot pass either. A missing operator is the shape that would make
    this file read as a full sweep while measuring part of one.
  * an unmatched operator in any part  -- named a filter that reached nothing
  * parts that disagree about `compared_against`, or that carry a part scored on
    an integrated tree. The whole point of the re-take is the reference side.
  * survivor ids repeated across parts

Exit 0 on a written artifact, 2 on any refusal. It writes `merged_from`, so a
reader cannot mistake this for a single sweep -- which is the same argument
`operators_filter` makes about the parts.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONTAINER = "vit-dev"
WORKDIR = f"/workspace/{ROOT.name}"
LOOP = "/workspace/translation-loop"
NOCOMPILE_LIMIT = 0.25   # vit_mutate.py's own limit, not a second opinion


def population_from_cppmutate(unit: str, cpp: str) -> dict[str, list[str]]:
    """{operator: [mutant id, ...]} straight out of the mutator vit_mutate.py uses.

    Read from the instrument rather than from the parts, so that "did the union
    cover the sweep" is a question the union can fail. Returns {} when the
    mutator cannot be reached, which the caller renders as a refusal and never
    as an empty population that every union trivially covers (P6).

    IT RETURNS IDS AND NOT ONLY COUNTS, and the difference is what lets one
    operator be scored by several runs. A count answers "are all 40 there"; ids
    answer "are they the SAME 40", which is the only question worth asking of a
    union of slices -- two parts can agree about indices 0..19 and 20..39 of two
    different enumerations and sum to 40 having scored one mutant twice and
    another never. Counts are derived from the ids below, so every check the
    count-based version made is still made, over a strictly stronger input.
    """
    prog = (
        "import sys, json, collections; sys.path.insert(0, %r);"
        "from harness.cppmutate import mutants;"
        "src = open(%r).read();"
        "d = collections.defaultdict(list);"
        "[d[m.operator].append(m.mid) for m in mutants(%r, src)];"
        "print(json.dumps(d))"
        % (LOOP, f"{WORKDIR}/{cpp}", unit.lower())
    )
    r = subprocess.run(
        ["docker", "exec", CONTAINER, "python3", "-c", prog],
        capture_output=True, text=True)
    if r.returncode != 0 or not r.stdout.strip():
        print(r.stderr.strip()[-500:], file=sys.stderr)
        return {}
    return json.loads(r.stdout)


def die(msg: str) -> int:
    print(f"_mutation_merge: REFUSING -- {msg}", file=sys.stderr)
    return 2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--unit", required=True)
    ap.add_argument("--part", action="append", required=True,
                    help="an operator-filtered mutation artifact (repeatable)")
    ap.add_argument("--cpp", required=True,
                    help="the translation, relative to root -- the population "
                         "is asked of the mutator over THIS file")
    ap.add_argument("--out", required=True)
    ap.add_argument("--why", required=True,
                    help="why the sweep was split; lands in the artifact")
    a = ap.parse_args()

    parts = []
    for p in a.part:
        f = ROOT / p
        if not f.is_file():
            return die(f"no such part: {p}")
        parts.append((p, json.loads(f.read_text())))

    # --- every part must be a filtered run of THIS unit -----------------------
    for name, d in parts:
        if d.get("unit") != a.unit:
            return die(f"{name} is for unit {d.get('unit')!r}, not {a.unit!r}")
        if "operators_filter" not in d:
            return die(f"{name} has no operators_filter -- an unfiltered run is "
                       f"not a part of a union, it is a whole sweep")
        if d.get("operators_filter_unmatched"):
            return die(f"{name} names operator(s) that reached no site: "
                       f"{d['operators_filter_unmatched']} -- an empty "
                       f"measurement wearing the name of one")

    # --- the union must be disjoint and must exhaust the operator set ---------
    # AN OPERATOR MAY SPAN SEVERAL PARTS, AND ONLY IF THEIR IDS PARTITION IT.
    #
    # Until unit #53 the rule was one operator, one part: a repeat was refused
    # outright because its mutants would be counted twice. That rule was right
    # about the hazard and wrong about the remedy, and IPC is where the
    # difference cost a measurement -- 40 const_tweak mutants at 26.5s on a
    # 63,888-case corpus is 1,040s against a 600s foreground ceiling, so the
    # operator could not be scored by ONE part and was not allowed to be scored
    # by two. The unit closed with no `mutation/IPC.json` at all.
    #
    # What the old rule really enforced was a PARTITION, using "at most one
    # part" as a sufficient condition for disjointness because nothing in the
    # artifact could express any other. `scored_ids` can, so the partition is
    # now checked directly: no id in two parts, and every id of every operator
    # in some part. That is strictly stronger than the count arithmetic it
    # replaces -- it catches a part that scored 40 of the RIGHT SIZE and the
    # wrong membership, which counts cannot see.
    #
    # THE OLD RULE STILL APPLIES TO PARTS THAT CANNOT BE CHECKED THAT WAY. An
    # artifact written before `scored_ids` existed carries no ids, so for those
    # the sharing refusal and the count arithmetic stand unchanged. A union may
    # not mix the two: a part with ids and a part without, sharing an operator,
    # is precisely the case neither rule covers.
    ids_ok = all("scored_ids" in d for _, d in parts)
    seen: dict[str, list[str]] = {}
    for name, d in parts:
        for op in d["operators_filter"]:
            if op in seen and not ids_ok:
                return die(f"operator {op!r} is in both {seen[op][0]} and "
                           f"{name}, and at least one of them carries no "
                           f"`scored_ids` -- its mutants would be counted twice "
                           f"and nothing here could tell")
            seen.setdefault(op, []).append(name)

    population = population_from_cppmutate(a.unit, a.cpp)
    if not population:
        return die(f"could not ask harness.cppmutate for {a.unit}'s mutant "
                   f"population; an unchecked union is not a sweep")
    counts = {op: len(v) for op, v in population.items()}
    pop = set(population)
    missing = pop - set(seen)
    if missing:
        return die(f"operator(s) {sorted(missing)} found a site in this unit and "
                   f"are in NO part -- this union does not cover the sweep")
    extra = set(seen) - pop
    if extra:
        return die(f"part(s) filter on {sorted(extra)}, which found no site in "
                   f"this unit; the parts and the population disagree")

    if ids_ok:
        # Every id scored, and the part that scored it. Overlap is a repeat.
        by_id: dict[str, str] = {}
        for name, d in parts:
            if len(d["scored_ids"]) != d["total"]:
                return die(f"{name} lists {len(d['scored_ids'])} scored id(s) "
                           f"but reports total={d['total']}; the artifact does "
                           f"not agree with itself")
            for mid in d["scored_ids"]:
                if mid in by_id:
                    return die(f"mutant {mid} was scored by both {by_id[mid]} "
                               f"and {name} -- the slices overlap and it would "
                               f"be counted twice")
                by_id[mid] = name
        want_ids = {mid for op in pop for mid in population[op]}
        unscored = want_ids - set(by_id)
        if unscored:
            owner = {mid: op for op in pop for mid in population[op]}
            byop: dict[str, int] = {}
            for mid in unscored:
                byop[owner[mid]] = byop.get(owner[mid], 0) + 1
            return die(f"{len(unscored)} mutant(s) of this unit are in NO part "
                       f"({', '.join(f'{o}:{n}' for o, n in sorted(byop.items()))})"
                       f" -- the slices do not cover the operators they name")
        alien = set(by_id) - want_ids
        if alien:
            return die(f"{len(alien)} scored id(s) are not in this unit's "
                       f"population at all (e.g. {sorted(alien)[:3]}) -- the "
                       f"parts and the mutator disagree about what the mutants "
                       f"ARE, not merely how many")
    else:
        for name, d in parts:
            want = sum(counts[op] for op in d["operators_filter"])
            if d["total"] != want:
                return die(f"{name} scored {d['total']} mutant(s) but its "
                           f"operators {d['operators_filter']} produce {want} "
                           f"-- the part is not a whole slice of the sweep")

    # --- the reference side, which is the whole reason for the re-take --------
    against = {d.get("compared_against") for _, d in parts}
    if len(against) != 1:
        return die(f"the parts were compared against different things: {against}")
    compared_against = next(iter(against))
    if compared_against != "fortran_reference_on_a_clean_tree":
        return die(f"compared_against is {compared_against!r} -- this is not a "
                   f"measurement of the translation")

    # --- survivors: ids must not repeat --------------------------------------
    survivors, sid = [], {}
    for name, d in parts:
        for s in d.get("survivors", []):
            if s["id"] in sid:
                return die(f"survivor {s['id']} appears in both {sid[s['id']]} "
                           f"and {name}")
            sid[s["id"]] = name
            survivors.append(s)

    # --- the sums, and the tool's own arithmetic over them --------------------
    total = sum(d["total"] for _, d in parts)
    behavioural = sum(d["mutants"] for _, d in parts)
    nocompile = sum(d["nocompile"] for _, d in parts)
    killed = sum(d["killed"] for _, d in parts)
    survived = sum(d["survived"] for _, d in parts)
    eq = sum(d["equivalent_declared"] for _, d in parts)
    hung_ids = [i for _, d in parts for i in d.get("killed_by_timeout_ids", [])]

    if behavioural + nocompile != total:
        return die(f"{behavioural} behavioural + {nocompile} nocompile != "
                   f"{total} total")
    # `mutants` COUNTS THE DECLARED EQUIVALENCES and `killed`/`survived` do not,
    # which this check did not know. `vit_mutate.py` writes mutants=40,
    # killed=28, survived=0, equivalent_declared=12 for one filtered run, and
    # `denom = behavioural - eq` five lines down already reads it that way -- so
    # the tool's own two statements of the identity disagreed, and the one that
    # ran first refused every merge with an equivalence in it:
    #
    #     _mutation_merge: REFUSING -- 114 killed + 0 survived != 126 behavioural
    #
    # C12: recorded with its failing artifact before the fix. rosco-r2 unit #38
    # is the campaign's first SPLIT sweep that declares any equivalence, so the
    # gap could not have surfaced earlier; PitchSaturation (#36) declares six and
    # ran in one part, and every earlier split part declared none.
    if killed + survived + eq != behavioural:
        return die(f"{killed} killed + {survived} survived + {eq} declared "
                   f"equivalent != {behavioural} behavioural")
    if len(survivors) != survived:
        return die(f"{len(survivors)} survivor record(s) but survived={survived}")

    denom = behavioural - eq
    score = killed / denom if denom else 0.0
    ratio = nocompile / total if total else 0.0

    revs = {(d.get("loop_rev"), d.get("vit_rev")) for _, d in parts}
    if len(revs) != 1:
        return die(f"the parts were produced by different instruments: {revs}")
    loop_rev, vit_rev = next(iter(revs))

    doc = {
        "unit": a.unit,
        "operators": sorted(pop),
        "operator_population": counts,
        "operators_offered": parts[0][1]["operators_offered"],
        "compared_against": compared_against,
        "mutants": behavioural,
        "equivalent_declared": eq,
        "nocompile": nocompile,
        "killed_by_timeout": len(hung_ids),
        "killed_by_timeout_ids": hung_ids,
        "nocompile_ratio": round(ratio, 4),
        "nocompile_refused": ratio > NOCOMPILE_LIMIT,
        "total": total,
        "killed": killed,
        "survived": survived,
        "equivalent": eq,
        "declared_but_killed": [i for _, d in parts
                                for i in d.get("declared_but_killed", [])],
        "score": round(score, 4),
        "loop_rev": loop_rev,
        "vit_rev": vit_rev,
        "merged_from": [
            {"part": name,
             "operators_filter": d["operators_filter"],
             **({"mutant_slice": d["mutant_slice"]} if "mutant_slice" in d else {}),
             "total": d["total"], "mutants": d["mutants"],
             "nocompile": d["nocompile"], "killed": d["killed"],
             "survived": d["survived"], "score": d["score"]}
            for name, d in parts],
        "merged_why": a.why,
        "merged_by": "scripts/_mutation_merge.py",
        "survivors": survivors,
    }
    (ROOT / a.out).write_text(json.dumps(doc, indent=1) + "\n")
    print(f"{a.out}: {killed} killed of {behavioural} behavioural "
          f"({nocompile} nocompile, {total} mutants)   score {score:.4f}")
    print(f"  union of {len(parts)} filtered run(s), covering "
          f"{len(pop)} operator(s): {', '.join(sorted(pop))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
