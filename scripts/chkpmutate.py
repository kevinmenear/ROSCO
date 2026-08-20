#!/usr/bin/env python3
"""Mutation score for `WriteRestartFile`, whose only output is a FILE.

P13 makes a mutation score mandatory for a `respecify` unit and `vit_mutate.py`
cannot produce one here, for a reason one step past `Debug`'s. `vit_mutate.py`
scores a mutant by rebuilding the generated differential harness and comparing
the unit's MAPPED OUTPUTS; this unit assigns nothing in `CntrPar` or `objInst`
and writes `ErrVar` only on its two I/O-failure arms, so almost every mutant
would survive. Worse, a differential harness would have BOTH sides opening the
same path, and the second write would erase the first.

So the oracle is the one that verified the unit: the bytes of the `*.RO.chkp`
files scenarios 36-40 write (`scripts/chkpcheck.py`). Each mutant is compiled
into `libdiscon.so`, the scenarios are run, and the archive is compared against
`pre` -- the reference taken with `WriteRestartFile` still FORTRAN.

THE REFERENCE SIDE IS THE FORTRAN, WHICH IS THE CONFIGURATION QUESTION unit #29
got wrong. `vit_mutate` on an integrated tree routes both sides of its
comparison through the harness's own copy of the mutant and reports a red that
measured nothing. That cannot happen here: `pre` is a committed manifest over
bytes produced by a build that contained no C++ `WriteRestartFile` at all, so
the reference side is fixed before the sweep starts and nothing the sweep does
can perturb it.

Killed means the archive differed, the build failed, or the run did not
terminate inside the watchdog. Each is reported as its own outcome, because a
mutant killed by a watchdog was killed by a clock rather than by a compared
byte.

    python3 scripts/chkpmutate.py --cpp rosco/controller/src/writerestartfile.cpp \\
        --unit WriteRestartFile --operator const_tweak --slice 0/2 \\
        --out mutation/WriteRestartFile.const_tweak.0.json

ALWAYS through `scripts/mutate_guarded.sh`: this edits the SHIPPED translation
in place and restores it on completion, so a kill without the marker leaves a
mutant compiled into the library every later measurement then reads.

COPIED, NOT REWRITTEN (P4). `scripts/dbgmutate.py` at campaign rev cf13ed01 is
the source for `dexec`, `build`, the baseline-first protocol, `main`'s
filter/slice handling and `summarise`. What differs:

  * the comparison is over whole-file BYTES, not `\\n`-separated records, and
    it reports `bytes_mismatched` as well as `files_mismatched`.
  * there is no stdout stream: this unit writes nothing to unit 6. `Debug`'s
    fourth stream carried nine of its mutants, so the absence is stated rather
    than left to be noticed.
  * `<stem>.cpp` under `rosco/controller/src/` is what is mutated, and the
    translation under `translations/ROSCO_IO/` is left alone. They are byte
    identical apart from VIT's generated header and `extern "C"` wrapper; the
    build reads the first.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTAINER = os.environ.get("VIT_CONTAINER", "vit-dev")
WORKDIR = f"/workspace/{ROOT.name}"
LOOP = os.environ.get("LOOP_ROOT", str(ROOT.parent / "translation-loop"))
sys.path.insert(0, LOOP)

from harness.cppmutate import mutants, operator_counts  # noqa: E402

sys.path.insert(0, str(ROOT / "scripts"))
import chkpcheck  # noqa: E402

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
    for p in chkpcheck.CHKPDIR.glob(chkpcheck.PATTERN):
        p.unlink()
    for s in scenarios:
        rc = dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py --scenario {s}", timeout)
        if rc == 124:
            return "hang", {}
        if rc != 0:
            return "norun", {"scenario": s, "rc": rc}

    rep = {"files_compared": 0, "files_mismatched": 0,
           "bytes_compared": 0, "bytes_mismatched": 0,
           "missing": [], "extra": []}
    produced = {p.name: p for p in chkpcheck.CHKPDIR.glob(chkpcheck.PATTERN)}
    for rp in sorted(ref.glob(chkpcheck.PATTERN)):
        cp = produced.pop(rp.name, None)
        if cp is None:
            rep["missing"].append(rp.name)
            continue
        da, db = rp.read_bytes(), cp.read_bytes()
        rep["files_compared"] += 1
        rep["bytes_compared"] += len(da)
        nd, _ = chkpcheck.compare_bytes(da, db)
        if nd:
            rep["files_mismatched"] += 1
            rep["bytes_mismatched"] += nd
    rep["extra"] = sorted(produced)
    return "ok", rep


def summarise(args, results, all_counts, base, equivalent, scenarios) -> dict:
    """`mutants` is the BEHAVIOURAL count and `killed` is a count over it,
    because that is what loop/done.py's P12 reads. A no-compile is excluded from
    both sides rather than counted as a kill: it establishes that the mutator
    broke the build, not that the oracle discriminates."""
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
        "compared_against": "fortran_reference_chkp_archive",
        "oracle": ("byte identity of every *.RO.chkp the checkpoint scenarios "
                   "write, against a committed archive produced by a build "
                   "whose WriteRestartFile was Fortran"),
        "streams_compared": ["RO.chkp"],
        "baseline": base,
        "survivors": survivors,
        "results": results,
        **chkpcheck.instrument_revs(),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unit", required=True)
    ap.add_argument("--cpp", required=True, help="path relative to the campaign root")
    ap.add_argument("--scenarios", default="36,37,38,39,40")
    ap.add_argument("--reference", default="pre", help="chkpcheck archive label")
    ap.add_argument("--operator", action="append", default=[])
    ap.add_argument("--equivalences", default=None)
    ap.add_argument("--case-timeout", type=float, default=120.0)
    ap.add_argument("--slice", default=None,
                    help="i/n -- score only slice i of n, in the mutator's own "
                         "deterministic order. The Bash tool's foreground "
                         "ceiling is 600 s and a mutant here costs a rebuild "
                         "plus five simulations, so a large operator does not "
                         "fit in one command. The slice lands in the artifact "
                         "and the merge refuses a set that does not cover every "
                         "slice of every operator.")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    cpp = ROOT / args.cpp
    scenarios = [int(x) for x in args.scenarios.split(",")]
    ref = chkpcheck.ARCHIVE / args.reference
    if not ref.is_dir():
        print(f"error: no reference archive at {ref}", file=sys.stderr)
        return 2

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

    subset = Path(tempfile.mkdtemp(prefix="chkpmut_ref_"))
    try:
        # Establish the baseline FIRST, and take the reference file list from
        # what the unmutated build actually writes. A reference file the sweep
        # never produces would count every mutant killed.
        if not build(args.cpp):
            print("baseline does not build -- refusing", file=sys.stderr)
            return 2
        for p in chkpcheck.CHKPDIR.glob(chkpcheck.PATTERN):
            p.unlink()
        for s in scenarios:
            if dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py --scenario {s}") != 0:
                print(f"baseline scenario {s} failed -- refusing", file=sys.stderr)
                return 2
        names = sorted(p.name for p in chkpcheck.CHKPDIR.glob(chkpcheck.PATTERN))
        if not names:
            print("baseline wrote no checkpoint -- refusing", file=sys.stderr)
            return 2
        for n in names:
            if not (ref / n).is_file():
                print(f"reference archive has no {n} -- refusing", file=sys.stderr)
                return 2
            shutil.copy2(ref / n, subset / n)
        outcome, base = run_and_compare(scenarios, subset, args.case_timeout)
        if (outcome != "ok" or base["bytes_mismatched"] != 0
                or base["missing"] or base["extra"]):
            print(f"BASELINE IS NOT GREEN ({outcome}, {base}) -- refusing", file=sys.stderr)
            return 2
        print(f"baseline green: {base['files_compared']} file(s), "
              f"{base['bytes_compared']} bytes, 0 mismatched")

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
                    killed = (rep["bytes_mismatched"] > 0
                              or bool(rep["missing"]) or bool(rep["extra"]))
            results.append({"mid": m.mid, "operator": m.operator, "line": m.line,
                            "before": m.before, "after": m.after,
                            "outcome": outcome, "killed": killed,
                            "files_mismatched": rep.get("files_mismatched", 0),
                            "bytes_mismatched": rep.get("bytes_mismatched", 0),
                            "files_missing": rep.get("missing", []),
                            "files_extra": rep.get("extra", [])})
            print(f"  [{i}/{len(ms)}] {m.operator} {m.mid} "
                  f"{'KILLED' if killed else 'SURVIVED'} ({outcome}, "
                  f"{rep.get('files_mismatched', 0)} files, "
                  f"{rep.get('bytes_mismatched', 0)} bytes) "
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


if __name__ == "__main__":
    sys.exit(main())
