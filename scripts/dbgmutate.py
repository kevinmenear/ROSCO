#!/usr/bin/env python3
"""Mutation score for a unit whose only output is a FILE.

P13 makes a mutation score mandatory for a `respecify` unit, and `vit_mutate.py`
cannot produce one here. It scores a mutant by rebuilding the generated
differential harness and comparing the unit's MAPPED OUTPUTS -- and `Debug`
assigns nothing in its own signature. Every mutant would survive, and the 0.000
would be a fact about the instrument.

So the oracle is the same one that verified the unit: the bytes of the
`.RO.dbg` file the 27 scenarios write (`scripts/dbgcheck.py`). Each mutant is
compiled into `libdiscon.so`, the scenarios in `--scenarios` are run, and the
archive is compared against `pre` -- the reference taken with `Debug` still
FORTRAN.

THE REFERENCE SIDE IS THE FORTRAN, WHICH IS THE CONFIGURATION QUESTION unit #29
got wrong. `vit_mutate` on an integrated tree routes both sides of its
comparison through the harness's own copy of the mutant and reports a red that
measured nothing. That cannot happen here: `pre` is a committed archive of bytes
produced by a build that contained no C++ `Debug` at all, so the reference side
is fixed before the sweep starts and cannot be perturbed by it.

Killed means the archive differed, the build failed, or the run did not
terminate inside the watchdog. Each is reported as its own outcome, because a
mutant killed by a watchdog was killed by a clock rather than by a compared
byte.

    python3 scripts/dbgmutate.py --cpp rosco/controller/src/debug.cpp \
        --unit Debug --scenarios 3,7,27 --operator compare_op \
        --out mutation/Debug.compare_op.json

Always through scripts/mutate_guarded.sh: this edits the SHIPPED translation in
place and restores it on completion, so a kill without the marker leaves a
mutant compiled into the library the gate then measures.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTAINER = os.environ.get("VIT_CONTAINER", "vit-dev")
WORKDIR = f"/workspace/{ROOT.name}"
LOOP = os.environ.get("LOOP_ROOT", str(ROOT.parent / "translation-loop"))
sys.path.insert(0, LOOP)

from harness.cppmutate import mutants, operator_counts  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts"))
import dbgcheck  # noqa: E402

# vit_mutate.py's own limit, kept rather than re-chosen: above this share of
# unbuildable mutants the run is measuring the build and not the oracle.
NOCOMPILE_LIMIT = 0.25


def dexec(script: str, timeout: float | None = None) -> int:
    try:
        return subprocess.run(["docker", "exec", CONTAINER, "bash", "-lc", script],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              timeout=timeout).returncode
    except subprocess.TimeoutExpired:
        return 124


def build(cpp_rel: str) -> bool:
    """Rebuild and install. The md5 is the bind-mount rule (RUNBOOK, units #23
    and #30): prove the edit ARRIVED, and touch it so the object cannot be
    stamped older than the source `make` compares it against."""
    dexec(f"md5sum {WORKDIR}/{cpp_rel} && touch {WORKDIR}/{cpp_rel}")
    return dexec(f"cd {WORKDIR}/rosco/controller/build && cmake --build . -j4 && "
                 f"cp libdiscon.so {WORKDIR}/rosco/lib/libdiscon.so") == 0


def run_and_compare(scenarios: list[int], ref: Path, timeout: float) -> tuple[str, dict]:
    """('ok'|'hang'|'norun', report). 'ok' means every scenario ran; the report
    then says whether the bytes matched."""
    for p in dbgcheck.DBGDIR.glob("*.RO.dbg*"):
        p.unlink()
    for s in scenarios:
        rc = dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py --scenario {s}", timeout)
        if rc == 124:
            return "hang", {}
        if rc != 0:
            return "norun", {"scenario": s, "rc": rc}

    rep = {"records_compared": 0, "records_mismatched": 0, "files_compared": 0,
           "missing": [], "extra": []}
    produced = {p.name: p for p in dbgcheck.DBGDIR.glob("*.RO.dbg*")}
    for rp in sorted(ref.glob("*.RO.dbg*")):
        cp = produced.pop(rp.name, None)
        if cp is None:
            rep["missing"].append(rp.name)
            continue
        ra = rp.read_bytes().split(b"\n")
        rb = cp.read_bytes().split(b"\n")
        rep["files_compared"] += 1
        for i in range(max(len(ra), len(rb))):
            la = ra[i] if i < len(ra) else None
            lb = rb[i] if i < len(rb) else None
            if la is None or lb is None:
                rep["records_mismatched"] += 1
                continue
            rep["records_compared"] += 1
            if i == 0:
                if not dbgcheck.compare_first_record(la, lb, {}):
                    rep["records_mismatched"] += 1
                continue
            if la != lb:
                rep["records_mismatched"] += 1
    rep["extra"] = sorted(produced)
    return "ok", rep


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unit", required=True)
    ap.add_argument("--cpp", required=True, help="path relative to the campaign root")
    ap.add_argument("--scenarios", default="3,7,27")
    ap.add_argument("--reference", default="pre", help="dbgcheck archive label")
    ap.add_argument("--operator", action="append", default=[])
    ap.add_argument("--equivalences", default=None)
    ap.add_argument("--case-timeout", type=float, default=120.0)
    ap.add_argument("--slice", default=None,
                    help="i/n -- score only slice i of n, in the mutator's own "
                         "deterministic order. A dispatch's foreground command "
                         "may block for 600 seconds and a mutant here costs a "
                         "rebuild plus a simulation, so a large operator does "
                         "not fit in one command. The slice lands in the "
                         "artifact and the merge refuses a set that does not "
                         "cover every slice of every operator.")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    cpp = ROOT / args.cpp
    scenarios = [int(x) for x in args.scenarios.split(",")]
    ref = dbgcheck.ARCHIVE / args.reference
    if not ref.is_dir():
        print(f"error: no reference archive at {ref}", file=sys.stderr)
        return 2

    # The reference archive holds all 27 scenarios' files; this run compares the
    # ones its scenarios produce. Restrict by BUILDING the subset directory
    # view rather than by trusting names to line up.
    original = cpp.read_text()
    ms = mutants(args.unit.lower(), original)
    all_counts = operator_counts(ms)
    if args.operator:
        want = set(args.operator)
        kept = [m for m in ms if m.operator in want]
        print(f"NOTE: operator filter {sorted(want)} -- scoring {len(kept)} of "
              f"{len(ms)}. The other {len(ms) - len(kept)} are NOT run here and "
              f"this artifact says nothing about them.")
        ms = kept
    if args.slice:
        i, n = (int(x) for x in args.slice.split("/"))
        before = len(ms)
        ms = [m for k, m in enumerate(ms) if k % n == i]
        print(f"NOTE: slice {args.slice} -- scoring {len(ms)} of {before}. The "
              f"other {before - len(ms)} are NOT run here.")
    equivalent = set(json.loads(Path(args.equivalences).read_text())
                     if args.equivalences else [])

    # A subset reference: only the files the chosen scenarios write.
    import tempfile, shutil
    subset = Path(tempfile.mkdtemp(prefix="dbgmut_ref_"))
    try:
        # Establish the baseline FIRST, and take the reference file list from
        # what the unmutated build actually writes. A reference file the sweep
        # never produces would count every mutant killed.
        if not build(args.cpp):
            print("baseline does not build -- refusing", file=sys.stderr)
            return 2
        for p in dbgcheck.DBGDIR.glob("*.RO.dbg*"):
            p.unlink()
        for s in scenarios:
            if dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py --scenario {s}") != 0:
                print(f"baseline scenario {s} failed -- refusing", file=sys.stderr)
                return 2
        names = sorted(p.name for p in dbgcheck.DBGDIR.glob("*.RO.dbg*"))
        for n in names:
            if not (ref / n).is_file():
                print(f"reference archive has no {n} -- refusing", file=sys.stderr)
                return 2
            shutil.copy2(ref / n, subset / n)
        outcome, base = run_and_compare(scenarios, subset, args.case_timeout)
        if outcome != "ok" or base["records_mismatched"] != 0:
            print(f"BASELINE IS NOT GREEN ({outcome}, {base}) -- refusing", file=sys.stderr)
            return 2
        print(f"baseline green: {base['files_compared']} file(s), "
              f"{base['records_compared']} records, 0 mismatched")

        results = []
        t0 = time.time()
        for i, m in enumerate(ms, 1):
            cpp.write_text(m.source)
            if not build(args.cpp):
                outcome, rep, killed = "nocompile", {}, True
            else:
                outcome, rep = run_and_compare(scenarios, subset, args.case_timeout)
                if outcome == "hang":
                    killed = True
                elif outcome == "norun":
                    killed = True          # the mutant crashed the controller
                else:
                    killed = (rep["records_mismatched"] > 0
                              or bool(rep["missing"]) or bool(rep["extra"]))
            results.append({"mid": m.mid, "operator": m.operator, "line": m.line,
                            "before": m.before, "after": m.after,
                            "outcome": outcome, "killed": killed,
                            "records_mismatched": rep.get("records_mismatched", 0)})
            print(f"  [{i}/{len(ms)}] {m.operator} {m.mid} "
                  f"{'KILLED' if killed else 'SURVIVED'} ({outcome}, "
                  f"{rep.get('records_mismatched', 0)} records) "
                  f"{time.time() - t0:.0f}s", flush=True)
    finally:
        cpp.write_text(original)
        build(args.cpp)
        shutil.rmtree(subset, ignore_errors=True)

    art = summarise(args, results, all_counts, base, equivalent, scenarios)
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(art, indent=2, sort_keys=True))
    print(f"\n{art['killed']} of {art['mutants'] - art['equivalent_declared']} "
          f"scored killed, score {art['score']}")
    print(f"nocompile {art['nocompile']}/{art['total']}")
    return 2 if art["nocompile_refused"] else 0


# `mutants` is the BEHAVIOURAL count and `killed` is a count over it, because
# that is what loop/done.py's P12 reads and what _mutation_merge.py's arithmetic
# assumes. A no-compile is excluded from both sides rather than counted as a
# kill: it establishes that the mutator broke the build, not that the oracle
# discriminates.
def summarise(args, results, all_counts, base, equivalent, scenarios) -> dict:
    total = len(results)
    nocompile = sum(1 for r in results if r["outcome"] == "nocompile")
    beh = [r for r in results if r["outcome"] != "nocompile"]
    killed = sum(1 for r in beh if r["killed"])
    survivors = [{"id": r["mid"], "operator": r["operator"], "line": r["line"],
                  "before": r["before"], "after": r["after"]}
                 for r in beh if not r["killed"]]
    eq = len(equivalent & {r["mid"] for r in results})
    denom = len(beh) - eq
    ratio = (nocompile / total) if total else 0.0
    return {
        "unit": args.unit,
        "total": total,
        "mutants": len(beh),
        "killed": killed,
        "survived": len(survivors),
        "equivalent_declared": eq,
        "score": (killed / denom) if denom else 0.0,
        "nocompile": nocompile,
        "nocompile_ratio": ratio,
        "nocompile_refused": bool(total and ratio > NOCOMPILE_LIMIT),
        "operators": sorted({r["operator"] for r in results}),
        "operators_filter": sorted(args.operator) or None,
        "slice": args.slice,
        "operators_filter_unmatched": sorted(
            set(args.operator) - {r["operator"] for r in results}),
        "operator_population": all_counts,
        # A SILENT CAP READS AS FULL COVERAGE. cppmutate stops at 40 mutants per
        # operator; the operators that hit the cap are named here so a reader
        # can see that those operators were SAMPLED and not exhausted.
        "limit_per_operator": 40,
        "capped_operators": sorted(k for k, v in all_counts.items() if v >= 40),
        "scenarios": scenarios,
        "reference_archive": args.reference,
        "compared_against": "fortran_reference_dbg_archive",
        "oracle": ("byte identity of *.RO.dbg against the committed `pre` "
                   "archive, produced by a build whose Debug was Fortran"),
        "baseline": base,
        "survivors": survivors,
        "results": results,
        **dbgcheck.instrument_revs(),
    }


if __name__ == "__main__":
    sys.exit(main())
