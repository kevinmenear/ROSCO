#!/usr/bin/env python3
"""Union `rcpfsmutate.py` parts into one artifact, or refuse.

WHY THIS EXISTS, and why it is not the mutator re-run. A dispatch's foreground
command may block for 600 seconds; this unit's 213-mutant population costs about
750 s of value sweep and, for the survivors, another 700 s under the sanitisers.
So the sweep is three `--slice` parts plus two `--only` sanitised re-takes, each
in the foreground, and this joins them. The alternative -- backgrounding one --
is the failure this campaign has paid for twice.

THE GRADES ARE NOT AUTHORED HERE. Every `killed`, `nocompile` and channel list
comes out of `rcpfsmutate.py`; this file unions results by mutant id and
recomputes the two derived numbers with `rcpfsmutate.summarise`'s own
arithmetic. Copied from `scripts/_mutation_merge.py`'s contract (P4) rather than
its code, because the parts have a different schema.

WHAT IT REFUSES, because a union is a coverage claim and a coverage claim that
cannot fail is not one (P10):

  * parts that do not COVER the enumeration. The population is asked of
    `harness.cppmutate` directly -- the same function the mutator calls -- so
    deriving it from the parts cannot make the check a tautology.
  * a mutant scored twice by two VALUE parts (a slice overlap).
  * parts taken at different `loop_rev`s, or against different corpora. Two
    corpora with the same case count can be different files, so the CASE COUNT
    and the baseline COPYBK total must agree across parts.
  * a sanitised part that names an id no value part scored, or that reports a
    mutant SURVIVING which a value part killed -- the sanitised run is a
    re-take of the survivors and may only ADD kills.
  * a declaration whose id is not in the enumeration.

    python3 scripts/rcpfsmutate_merge.py --part /tmp/f_part0.json ... \\
        --sanitized /tmp/fsan_a.json --sanitized /tmp/fsan_b.json \\
        --equivalences mutation/X.equivalences.json \\
        --unreachable mutation/X.unreachable.json \\
        --out mutation/X.json

Exit 0 on a written artifact, 2 on any refusal.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import rcpfsmutate as R  # noqa: E402
from harness.cppmutate import mutants, operator_counts  # noqa: E402


def die(msg: str) -> int:
    print(f"rcpfsmutate_merge: REFUSING -- {msg}", file=sys.stderr)
    return 2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--part", action="append", required=True)
    ap.add_argument("--sanitized", action="append", default=[])
    ap.add_argument("--equivalences", default=None)
    ap.add_argument("--unreachable", default=None)
    ap.add_argument("--unit", default="ReadControlParameterFileSub")
    ap.add_argument("--cpp", default="translations/ReadSetParameters/"
                                     "readcontrolparameterfilesub.cpp")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    parts = [json.loads(Path(p).read_text()) for p in a.part]
    sans = [json.loads(Path(p).read_text()) for p in a.sanitized]
    if not parts:
        return die("no value parts")

    # Commensurability, across every part and every sanitised re-take.
    keys = {(d.get("loop_rev"), d.get("vit_rev"), d.get("compared_against"),
             d.get("corpus_cases"), d.get("baseline", {}).get("copyback"))
            for d in parts + sans}
    if len(keys) != 1:
        return die(f"parts disagree about the instrument or the corpus: {keys}")
    if any(d.get("sanitize") for d in parts):
        return die("a value part was taken with --sanitize")
    if not all(d.get("sanitize") for d in sans):
        return die("a --sanitized part was not taken with --sanitize")

    res: dict[str, dict] = {}
    for d in parts:
        for r in d["results"]:
            if r["mid"] in res:
                return die(f"mutant {r['mid']} scored by two value parts")
            res[r["mid"]] = dict(r, killed_by="value" if r["killed"] else None)

    for d in sans:
        for r in d["results"]:
            if r["mid"] not in res:
                return die(f"sanitised part names {r['mid']}, which no value "
                           f"part scored")
            cur = res[r["mid"]]
            if cur["killed"] and not r["killed"]:
                # Not a contradiction to route around: the value channels are
                # identical in both builds, so this would mean the oracle is
                # not deterministic.
                return die(f"mutant {r['mid']} killed by value and SURVIVING "
                           f"under the sanitisers -- the oracle disagrees with "
                           f"itself")
            if r["killed"] and not cur["killed"]:
                cur.update(killed=True, killed_by="sanitizer",
                           channels=r["channels"], san=r.get("san"))
            cur["sanitized"] = True

    original = (ROOT / a.cpp).read_text()
    enum = mutants(a.unit.lower(), original)
    capped = operator_counts(enum)
    uncapped = operator_counts(mutants(a.unit.lower(), original,
                                       limit_per_operator=10 ** 9))
    want = {m.mid for m in enum}
    if want - set(res):
        return die(f"{len(want - set(res))} enumerated mutant(s) in no part: "
                   f"{sorted(want - set(res))[:6]}")
    if set(res) - want:
        return die(f"part(s) score {len(set(res) - want)} mutant(s) this "
                   f"enumeration does not contain")

    equivalent = set(json.loads(Path(a.equivalences).read_text())
                     if a.equivalences else [])
    unreachable = {}
    if a.unreachable:
        for k, v in json.loads(Path(a.unreachable).read_text()).items():
            if not str(v.get("reason", "")).strip() or not str(v.get("evidence", "")).strip():
                return die(f"unreachable {k}: reason and evidence are required")
            if not (ROOT / str(v["evidence"]).split(":")[0]).exists():
                return die(f"unreachable {k}: evidence path {v['evidence']} "
                           f"does not exist")
            unreachable[k] = {"reason": v["reason"], "evidence": v["evidence"]}
    bad = (equivalent | set(unreachable)) - want
    if bad:
        return die(f"declaration(s) for id(s) not in the enumeration: {sorted(bad)}")
    both = equivalent & set(unreachable)
    if both:
        return die(f"id(s) declared both equivalent and unreachable: {sorted(both)}")

    class Args:
        unit, operator, slice = a.unit, [], None
        sanitize, only = False, None
    art = R.summarise(Args(), list(res.values()), capped, uncapped,
                      dict(parts[0]["baseline"], cases=parts[0]["corpus_cases"]),
                      equivalent, unreachable)
    art["merged_from"] = {"value_parts": a.part, "sanitized_parts": a.sanitized}
    art["sanitize"] = False
    art["sanitized_retake"] = sorted(m for m, r in res.items() if r.get("sanitized"))
    art["channels_compared"] = parts[0]["channels_compared"] + ["sanitizer (survivors only)"]
    # BEHAVIOURAL kills only. A nocompile carries `killed: True` so the
    # mutator's own arithmetic can exclude it from both sides of the ratio;
    # counting it here as "killed by value" would report 159 kills next to a
    # `killed` field of 155.
    beh = [r for r in res.values() if r["outcome"] != "nocompile"]
    art["killed_by_value"] = sum(1 for r in beh if r.get("killed_by") == "value")
    art["killed_by_sanitizer"] = sum(1 for r in beh
                                     if r.get("killed_by") == "sanitizer")
    out = ROOT / a.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(art, indent=2, sort_keys=True))
    scored = art["mutants"] - art["equivalent_declared"] - art["unreachable_declared"]
    print(f"{art['killed']} of {scored} scored killed, score {art['score']:.3f} "
          f"({art['killed_by_value']} by value, {art['killed_by_sanitizer']} by "
          f"sanitiser); {art['equivalent_declared']} equivalent, "
          f"{art['unreachable_declared']} unreachable; nocompile {art['nocompile']}")
    if art["survived"] - art["equivalent_declared"] - art["unreachable_declared"]:
        print("UNDECLARED SURVIVORS REMAIN", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
