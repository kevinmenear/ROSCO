#!/usr/bin/env python3
"""Mutation score for `ReadControlParameterFileSub`, whose instrument is a PROBE.

P13 makes a mutation score mandatory for a `respecify` unit and `vit_mutate.py`
cannot produce one here for the reason the unit's own record already states: it
drives `<stem>_cases.bin` and `./test`, both produced by the GENERATED
differential harness, and that harness REFUSED this unit
(`evidence/ReadControlParameterFileSub/harness.plan-dump.*`). No case file, no
`./test`, no score -- and a `blocked` disposition with no number is exactly the
thing the loop's P12 exists to prevent.

So the oracle is the one that verified the unit: the standalone differential
probe (`evidence/ReadControlParameterFileSub/gen_probe.py` +
`run_probe.sh`). One Fortran program calls the FORTRAN reference and the C++
translation on the same real `DISCON.IN` and compares all 213 components of
`TYPE(ControlParameters)` plus `ErrVar%aviFAIL` and `ErrVar%ErrMsg`, over the
37-file corpus. Each mutant is compiled into that program in place of the
translation, and the run is compared against the unmutated baseline.

THE REFERENCE SIDE IS THE FORTRAN, WHICH IS THE CONFIGURATION QUESTION UNIT #29
GOT WRONG. `vit_mutate` on an integrated tree routes both sides of its
comparison through the harness's own copy of the mutant and reports a red that
measured nothing. That cannot happen here: the reference side is
`readcontrolparameterfilesub_` inside the already-built `libdiscon.so`, which
this sweep never rebuilds -- only the probe's own object is recompiled. If this
unit is ever integrated, that stops being true, and the sweep must then be run
against a de-integrated library. `--refuse-if-integrated` checks the one thing
that would silently invert the comparison: the presence of the C++ wrapper in
the reference file.

THREE CHANNELS, NOT ONE, and the third was added because the first two cannot
see an error path:

  FIELD    a differing component of TYPE(ControlParameters), or aviFAIL, or
           ErrMsg. The probe's own `STOP 1`.
  COPYBK   the count of components the reference left UNALLOCATED and the view
           brought back allocated-empty. 586 on the baseline; it is not a
           defect (P6 in the copy-back, see gen_probe.py) but it is a
           deterministic function of what the translation allocated, so a
           mutant that moves it has been observed.
  STDOUT   the probe program's whole output, including the `ROSCO:` lines the
           unit itself prints and the `ERR ... ref ErrMsg` reports. `Debug`'s
           and `WriteRestartFile`'s sweeps both found error-path mutants that
           the value comparison alone could not see.
  ECHO     the SET of `*.RO.echo` files the run leaves in `Examples/` and in
           the corpus directory -- name, size and a digest with the wall-clock
           header masked. 149 bytes on the baseline for each of the reference's
           and the translation's, byte-identical to each other.

Killed means any channel differed, the mutant did not compile (reported
separately and excluded from both sides of the ratio), or the run did not
terminate inside the watchdog.

    python3 scripts/rcpfsmutate.py --unit ReadControlParameterFileSub \\
        --cpp translations/ReadSetParameters/readcontrolparameterfilesub.cpp \\
        --operator const_tweak --out mutation/ReadControlParameterFileSub.const_tweak.json

ALWAYS through `scripts/mutate_guarded.sh`. This driver never writes the
tracked translation -- the mutant goes into the container over stdin and the
build reads that copy -- but the marker costs nothing and the campaign's rule is
about the class, not about one implementation's carefulness.

COPIED, NOT REWRITTEN (P4). `scripts/chkpmutate.py` at campaign rev aaf0353c is
the source for `dexec`, the baseline-first protocol, `main`'s filter/slice
handling, `summarise` and the artifact's key set. What differs:

  * the oracle is the probe, so a mutant costs ~2 s (one `g++ -c`, one
    `gfortran` link, 37 parses) instead of a library rebuild plus five
    simulations. The whole 213-mutant population fits in one foreground call.
  * the translation is never written to disk. `chkpmutate` writes the mutant
    into `rosco/controller/src/<stem>.cpp` and restores it in a `finally`;
    three hard kills in this campaign have left a mutant behind that way.
  * `operator_counts` is taken over the UNCAPPED enumeration as well, so the
    artifact says what the 40-per-operator cap excluded rather than reporting a
    sample as a population.
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
from _harness_stamp import _rev  # noqa: E402

# vit_mutate.py's own limit, kept rather than re-chosen: above this share of
# unbuildable mutants the run is measuring the build and not the oracle.
NOCOMPILE_LIMIT = 0.25

MUTDIR = "/tmp/rcpfsmut"
PROBE_F90 = "evidence/ReadControlParameterFileSub/vit_rcpfs_probe.f90"
CORPUS = "../evidence/ReadControlParameterFileSub/corpus"


def instrument_revs() -> dict:
    """The three revisions revcheck reads. Same shape as chkpcheck's."""
    def _first(*cands):
        for c in cands:
            if c and (Path(c) / ".git").exists():
                return Path(c)
        return None
    loop = _first(os.environ.get("LOOP_ROOT"), "/workspace/translation-loop",
                  ROOT.parent / "translation-loop")
    vit_root = _first("/workspace/vit", ROOT.parent / "vit")
    return {"loop_rev": _rev(loop) if loop else "unknown",
            "vit_rev": _rev(vit_root) if vit_root else "unknown",
            "campaign_rev": _rev(ROOT)}


def dexec(script: str, timeout: float | None = None,
          stdin: str | None = None) -> tuple[int, str]:
    """Run one script in the container; return (rc, stdout+stderr)."""
    try:
        r = subprocess.run(["docker", "exec", *(["-i"] if stdin is not None else []),
                            CONTAINER, "bash", "-lc", script],
                           input=stdin, capture_output=True, text=True,
                           errors="replace", timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return 124, "<timeout>"


# ---------------------------------------------------------------------------
# The probe, split into the part that is built ONCE and the part that is built
# per mutant. `run_probe.sh` does both every time; at 213 mutants the shim and
# the driver's own compile are worth hoisting, and nothing about them depends
# on the mutant.

def setup(san: str) -> str:
    return f"""
set -e
cd {WORKDIR}
mkdir -p {MUTDIR}
python3 scripts/_integration_shim.py ReadControlParameterFileSub \\
    -f rosco/controller/src/ReadSetParameters.f90 > {MUTDIR}/shim.cpp
g++ -std=c++17 -O2 -fPIC -ffp-contract=off {san} -I rosco/controller/src \\
    -c {MUTDIR}/shim.cpp -o {MUTDIR}/shim.o
gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off \\
    -I rosco/controller/build/ftnmods -c {PROBE_F90} -o {MUTDIR}/probe.o \\
    -J {MUTDIR}
"""


def compile_cmd(san: str) -> str:
    return f"""
set -e
cd {WORKDIR}
g++ -std=c++17 -O2 -fPIC -ffp-contract=off {san} -I rosco/controller/src \\
    -include rosco/controller/src/vit_translated.h \\
    -c {MUTDIR}/unit.cpp -o {MUTDIR}/rcpfs.o
gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off {san} \\
    -o {MUTDIR}/probe {MUTDIR}/probe.o {MUTDIR}/rcpfs.o {MUTDIR}/shim.o \\
    -L rosco/controller/build -ldiscon -lstdc++
"""


# `-fsanitize=address,undefined` reaches the class no value comparison can: a
# mutant whose only difference is at an address the program does not own.
# `detect_leaks=0` because libdiscon's Fortran side allocates and does not free,
# which is not this unit's business; `halt_on_error=0` for UBSan so a run reports
# every site rather than the first.
SAN_FLAGS = "-fsanitize=address,undefined -fno-omit-frame-pointer -g"
SAN_ENV = ("ASAN_OPTIONS=detect_leaks=0:abort_on_error=0 "
           "UBSAN_OPTIONS=print_stacktrace=0 ")

# The echo files are a by-product of the Echo=1 case and are removed by the run
# that made them, exactly as run_probe.sh does: left behind they dirty the tree
# and `capture_done_check.sh` refuses on a dirty tree.
#
# TWO THINGS THE FIRST VERSION OF THIS BLOCK GOT WRONG, both preserved under C12
# in evidence/ReadControlParameterFileSub/mutation.the-echo-channel-was-a-clock.json:
#
#  1. THE DIGEST WAS A CLOCK. Line 2 of a .RO.echo is
#     ` Generated on 20-Aug-2026 at 18:10:17 using ROSCO-2.10.1`, written
#     through this unit's CurDate/CurTime permanent bridges, so two unmutated
#     runs one second apart digest differently and 96 of 207 mutants were
#     "killed" by the wall clock alone. The digits of that one line are masked
#     to `#` -- masked rather than dropped, so a mutant that changed the header
#     FORMAT (or swapped date for time) still moves the digest. `dbgcheck.py`
#     line 66 carries the same header's regex for the same reason.
#  2. IT LOOKED IN ONE DIRECTORY. Mutants that perturb the Echo flag or the
#     RootName write .RO.echo files into `Examples/` and under names with
#     trailing blanks -- 23 of them survived the first sweep as untracked files
#     in the tree. The SET of echo files is now a channel: a mutant that writes
#     a file the reference does not is killed by that, and nothing is left
#     behind for `capture_done_check.sh` to refuse on.
RUN = f"""
cd {WORKDIR}/Examples
find . {CORPUS} -maxdepth 1 -name '*.RO.echo*' -delete
env $VIT_SAN_ENV LD_LIBRARY_PATH={WORKDIR}/rosco/controller/build {MUTDIR}/probe \\
    DISCON*.IN {CORPUS}/*.IN 2>&1
echo "PROBE_EXIT=$?"
find . {CORPUS} -maxdepth 1 -name '*.RO.echo*' -print0 | sort -z | while IFS= read -r -d '' e; do
    echo "ECHO [$e] $(wc -c < "$e") $(sed -E '/^ Generated on /s/[0-9]/#/g' "$e" | md5sum)"
done
find . {CORPUS} -maxdepth 1 -name '*.RO.echo*' -delete
"""


def signature(out: str) -> dict:
    """What the run observed, in the four channels the artifact reports.

    `text` is the whole filtered output and is the strictest of them; FIELD,
    COPYBK and the echo lines are pulled out separately so a survivor list says
    WHICH channel moved rather than only that something did."""
    keep, field, copyback, echo = [], None, None, []
    exit_code = None
    san = set()
    for line in out.splitlines():
        # A SANITISER REPORT IS A CHANNEL OF ITS OWN, normalised to its KIND:
        # the addresses and the pc list differ run to run and would make the
        # text channel a second clock.
        if "runtime error:" in line:
            san.add(line.split("runtime error:", 1)[1].strip()[:60])
            continue
        if "AddressSanitizer" in line or "UndefinedBehaviorSanitizer" in line:
            san.add(line.split("ERROR:", 1)[-1].strip()[:60] if "ERROR:" in line
                    else line.strip()[:60])
            continue
    for line in out.splitlines():
        if "runtime error:" in line or "Sanitizer" in line:
            continue
        if line.startswith("PROBE_EXIT="):
            exit_code = int(line.split("=", 1)[1])
            continue
        if line.startswith("ECHO ["):
            echo.append(line.strip())
            continue
        # `ROSCO Warning` is filtered by run_probe.sh and kept filtered here so
        # the two instruments compare the same stream.
        if "ROSCO Warning" in line:
            continue
        keep.append(line.rstrip())
        if line.startswith("cases "):
            parts = line.split()
            field = int(parts[parts.index("(translation)") + 1])
            copyback = int(parts[-1])
    return {"text": "\n".join(keep), "field": field, "copyback": copyback,
            "echo": sorted(echo), "exit": exit_code, "san": sorted(san)}


def summarise(args, results, capped, uncapped, base, equivalent,
              unreachable) -> dict:
    """`mutants` is the BEHAVIOURAL count and `killed` is a count over it,
    because that is what loop/done.py's P12 reads. A no-compile is excluded from
    both sides rather than counted as a kill: it establishes that the mutator
    broke the build, not that the oracle discriminates."""
    total = len(results)
    nocompile = sum(1 for r in results if r["outcome"] == "nocompile")
    beh = [r for r in results if r["outcome"] != "nocompile"]
    killed = sum(1 for r in beh if r["killed"])
    ids = {r["mid"] for r in beh}
    survivors = [{"id": r["mid"], "operator": r["operator"], "line": r["line"],
                  "before": r["before"], "after": r["after"]}
                 for r in beh if not r["killed"]]
    eq = len(set(equivalent) & ids)
    unr = len(set(unreachable) & ids)
    # A declaration the run itself refuted. P12 reads both lists.
    eq_killed = sorted(m for m in set(equivalent) & ids
                       if next(r for r in beh if r["mid"] == m)["killed"])
    unr_killed = sorted(m for m in set(unreachable) & ids
                        if next(r for r in beh if r["mid"] == m)["killed"])
    denom = len(beh) - eq - unr
    ratio = (nocompile / total) if total else 0.0
    return {
        "unit": args.unit,
        "total": total,
        "mutants": len(beh),
        "killed": killed,
        "survived": len(survivors),
        "equivalent_declared": eq,
        "unreachable_declared": unr,
        "unreachable": [dict(id=m, **unreachable[m]) for m in sorted(unreachable)
                        if m in ids],
        "equivalent_but_killed": eq_killed,
        "unreachable_but_killed": unr_killed,
        "score": (killed / denom) if denom else 0.0,
        "nocompile": nocompile,
        "nocompile_ratio": ratio,
        "nocompile_refused": bool(total and ratio > NOCOMPILE_LIMIT),
        "operators": sorted({r["operator"] for r in results}),
        "operators_filter": sorted(args.operator) or None,
        "slice": args.slice,
        "operators_filter_unmatched": sorted(
            set(args.operator) - {r["operator"] for r in results}),
        "operator_population": capped,
        # A SILENT CAP READS AS FULL COVERAGE. cppmutate stops at 40 mutants per
        # operator, and the capped and uncapped populations are
        # indistinguishable after the call -- so the uncapped enumeration is
        # taken here (it costs a pass over the source and no build) and both
        # are reported. What it CANNOT say is how the excluded ones would have
        # scored: they were never built, so their survival is UNKNOWN.
        "limit_per_operator": 40,
        "operator_population_uncapped": uncapped,
        "capped_operators": sorted(k for k, v in capped.items() if v >= 40),
        "mutants_not_enumerated": sum(uncapped.values()) - sum(capped.values()),
        "corpus_cases": base.get("cases"),
        "compared_against": "fortran_reference_in_process",
        "oracle": (
            "the standalone differential probe: one Fortran program calls the "
            "FORTRAN reference and the C++ translation on the same DISCON.IN "
            "and compares 213 components of TYPE(ControlParameters) plus "
            "ErrVar%aviFAIL and ErrVar%ErrMsg, over 37 files. Four channels: "
            "FIELD, COPYBK, the program's whole stdout, and the bytes of the "
            "Echo=1 arm's .RO.echo."),
        "channels_compared": (["field", "copyback", "stdout", "echo"]
                              + (["sanitizer"] if args.sanitize else [])),
        "sanitize": bool(args.sanitize),
        "only": args.only,
        "baseline": {k: v for k, v in base.items() if k != "text"},
        "survivors": survivors,
        "results": results,
        **instrument_revs(),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unit", default="ReadControlParameterFileSub")
    ap.add_argument("--cpp", default="translations/ReadSetParameters/"
                                     "readcontrolparameterfilesub.cpp",
                    help="path relative to the campaign root")
    ap.add_argument("--operator", action="append", default=[])
    ap.add_argument("--equivalences", default=None)
    ap.add_argument("--unreachable", default=None)
    ap.add_argument("--case-timeout", type=float, default=120.0)
    ap.add_argument("--slice", default=None, help="i/n, in the mutator's own order")
    ap.add_argument("--limit-per-operator", type=int, default=40)
    ap.add_argument("--only", default=None,
                    help="comma-separated mutant ids -- re-take exactly these. "
                         "The population fields still describe the WHOLE "
                         "enumeration, so an artifact from a --only run cannot "
                         "be read as a full sweep.")
    ap.add_argument("--sanitize", action="store_true",
                    help="build every mutant with -fsanitize=address,undefined. "
                         "Reaches the class no value comparison can: a mutant "
                         "whose only difference is at an address the program "
                         "does not own. REFUSES if the unmutated build already "
                         "reports -- that is a finding about the translation or "
                         "the harness, and routing around it would score every "
                         "mutant against the original's own defect.")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    cpp = ROOT / args.cpp
    original = cpp.read_text()

    # The one configuration question that would silently invert the comparison.
    ref = (ROOT / "rosco/controller/src/ReadSetParameters.f90").read_text()
    if "readcontrolparameterfilesub_c" in ref:
        print("the reference file already calls the C++ wrapper -- both sides "
              "of this comparison would be the mutant. Refusing.", file=sys.stderr)
        return 2

    ms = mutants(args.unit.lower(), original,
                 limit_per_operator=args.limit_per_operator)
    capped = operator_counts(ms)
    uncapped = operator_counts(mutants(args.unit.lower(), original,
                                       limit_per_operator=10 ** 9))
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
    unreachable = {}
    if args.unreachable:
        raw = json.loads(Path(args.unreachable).read_text())
        for k, v in raw.items():
            if not str(v.get("reason", "")).strip() or not str(v.get("evidence", "")).strip():
                print(f"unreachable {k}: reason and evidence are both required",
                      file=sys.stderr)
                return 2
            p = ROOT / str(v["evidence"]).split(":")[0]
            if not p.exists():
                print(f"unreachable {k}: evidence path {v['evidence']} does not exist",
                      file=sys.stderr)
                return 2
            unreachable[int(k)] = {"reason": v["reason"], "evidence": v["evidence"]}

    if args.only:
        want_ids = {x.strip() for x in args.only.split(",") if x.strip()}
        before = len(ms)
        ms = [m for m in ms if m.mid in want_ids]
        missing = want_ids - {m.mid for m in ms}
        if missing:
            print(f"--only names {len(missing)} id(s) this enumeration does not "
                  f"contain: {sorted(missing)}", file=sys.stderr)
            return 2
        print(f"NOTE: --only -- scoring {len(ms)} of {before}. The other "
              f"{before - len(ms)} are NOT run here.")

    san = SAN_FLAGS if args.sanitize else ""
    SETUP, COMPILE = setup(san), compile_cmd(san)
    global RUN
    RUN = RUN.replace("$VIT_SAN_ENV", SAN_ENV if args.sanitize else "")

    rc, out = dexec(SETUP, timeout=300)
    if rc != 0:
        print(f"probe setup failed:\n{out}", file=sys.stderr)
        return 2

    # BASELINE FIRST. A baseline that is not green means every mutant would be
    # scored against a reference that already differs, and the number would be
    # about the instrument.
    dexec(f"cat > {MUTDIR}/unit.cpp", stdin=original)
    rc, out = dexec(COMPILE, timeout=300)
    if rc != 0:
        print(f"baseline does not compile:\n{out}", file=sys.stderr)
        return 2
    rc, out = dexec(RUN, timeout=args.case_timeout)
    base = signature(out)
    base["cases"] = sum(1 for l in base["text"].splitlines()
                        if l.startswith("  ok ") or l.startswith("  FAIL "))
    # A SANITISER BASELINE THAT REPORTS ON THE UNMUTATED TRANSLATION IS A
    # FINDING, NOT AN OBSTACLE: every mutant would then die of the original's
    # own defect and the score would be about that.
    if args.sanitize and base["san"]:
        print("THE UNMUTATED TRANSLATION REPORTS UNDER THE SANITISERS -- "
              "refusing to score.\n  " + "\n  ".join(base["san"]), file=sys.stderr)
        return 2
    if base["field"] != 0 or base["exit"] != 0 or base["cases"] == 0 or not base["echo"]:
        print(f"BASELINE IS NOT GREEN (field={base['field']} exit={base['exit']} "
              f"cases={base['cases']} echo={base['echo']}) -- refusing", file=sys.stderr)
        return 2
    print(f"baseline green: {base['cases']} case(s), FIELD 0, COPYBK "
          f"{base['copyback']}, echo {base['echo']}", flush=True)

    results = []
    t0 = time.time()
    for i, m in enumerate(ms, 1):
        rc, _ = dexec(f"cat > {MUTDIR}/unit.cpp", stdin=m.source)
        rc, out = dexec(COMPILE, timeout=300)
        if rc != 0:
            outcome, killed, sig, why = "nocompile", True, {}, ["nocompile"]
        else:
            rc, out = dexec(RUN, timeout=args.case_timeout)
            sig = signature(out)
            if rc == 124:
                outcome, killed, why = "hang", True, ["hang"]
            else:
                outcome = "ok"
                why = [c for c in ("field", "copyback", "echo", "exit", "san")
                       if sig.get(c) != base[c]]
                if sig["text"] != base["text"] and "stdout" not in why:
                    why.append("stdout")
                killed = bool(why)
        results.append({"mid": m.mid, "operator": m.operator, "line": m.line,
                        "before": m.before, "after": m.after,
                        "outcome": outcome, "killed": killed, "channels": why,
                        "field": sig.get("field"), "copyback": sig.get("copyback"),
                        "exit": sig.get("exit"), "san": sig.get("san")})
        print(f"  [{i}/{len(ms)}] {m.operator} {m.mid} "
              f"{'KILLED' if killed else 'SURVIVED'} ({outcome}, "
              f"field={sig.get('field')} copyback={sig.get('copyback')} "
              f"{'+'.join(why) or '-'}) {time.time() - t0:.0f}s", flush=True)

    art = summarise(args, results, capped, uncapped, base, equivalent, unreachable)
    out_path = ROOT / args.out
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(art, indent=2, sort_keys=True))
    scored = art["mutants"] - art["equivalent_declared"] - art["unreachable_declared"]
    print(f"\n{art['killed']} of {scored} scored killed, score {art['score']:.3f}")
    print(f"nocompile {art['nocompile']}/{art['total']}")
    return 2 if art["nocompile_refused"] else 0


if __name__ == "__main__":
    sys.exit(main())
