#!/usr/bin/env python3
"""The gate, with a machine-readable verdict. E3.1, E3.2, E4.3.

`regress.sh` runs the 27 scenarios and prints a verdict, but it counts
CHANNELS and it writes nothing a check can read. Both matter:

  * A channel count is `27`-ish where the value count is ~13 million. P9 only
    asserts `compared > 0`, so a channel-counted artifact would pass while
    saying four orders of magnitude less than it appears to. Worse, the red
    test in RUNBOOK.md was recorded in VALUES (198,892 of 624,000) -- so a
    channel-counted gate and a value-counted red test describe two different
    instruments, and E3.1's artifact would not be evidence about the thing
    E3.2 observed failing. This script counts VALUES, and records the channel
    counts alongside rather than instead.

  * "the compared count exists only in stdout and in prose, so no artifact can
    be checked" is `loop/done.py` P9's own NOT_EVALUABLE reason. A gate whose
    output dies with the terminal cannot be audited afterwards.

Comparison is on the exact bit pattern, per element, via a uint8 view. Not
`==`: two NaNs with the same payload are the same value and `==` calls them
different, and `np.array_equal` collapses a whole channel to one bool.

    python3 scripts/gate.py <unit>
    python3 scripts/gate.py __gate__ --perturb-file rosco/controller/src/Filters.f90 \
        --perturb-from '<literal>' --perturb-to '<literal>'

Exits non-zero on mismatch, on a scenario that failed to run, on a missing
baseline, AND on comparing nothing. That last one is not in E3.1 as written;
it follows from SPEC §7 -- a gate that compares zero values and exits 0 is the
failure this project exists to remove. Recorded in DECISIONS.md as a local
decision, not as the criterion.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONTAINER = os.environ.get("VIT_CONTAINER", "vit-dev")
WORKDIR = f"/workspace/{ROOT.name}"

# regress.sh's order, kept rather than re-derived: 3/4/5 lead because they are
# the ones that historically broke first. Each runs in its own process because
# the DLL holds Fortran SAVE state that dlclose() does not reliably release.
SCENARIO_ORDER = [3, 4, 5, 1, 2] + list(range(6, 28))


def dexec(script: str, quiet: bool = True) -> int:
    """Run a bash line inside the container. Returns the exit code."""
    kw = {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL} if quiet else {}
    return subprocess.run(["docker", "exec", CONTAINER, "bash", "-lc", script], **kw).returncode


def telemetry(function: str, event: str, **kw) -> None:
    """Bracket the step so it is visible to the telemetry joiner.

    Without this the gate's minutes vanish from per-function cost and every
    total reads low. `__gate__` is in checks/brackets.py's STANDARD set as
    phase 3; a unit name needs no bracket declaration.
    """
    extra = " ".join(f"--{k.replace('_', '-')} {v}" for k, v in kw.items())
    dexec(f"cd {WORKDIR} && VIT_TELEMETRY_LOG={WORKDIR}/.vit/cycle_log.jsonl "
          f"PYTHONPATH=/workspace/vit python3 -m vit.telemetry {event} "
          f"--function {function} {extra}")


def build_and_install() -> bool:
    """Rebuild the controller and install the DLL the scenarios load."""
    rc = dexec(
        f"cd {WORKDIR}/rosco/controller && mkdir -p build && cd build && "
        f"cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . -j4 && "
        f"cp {WORKDIR}/rosco/controller/build/libdiscon.so {WORKDIR}/rosco/lib/libdiscon.so")
    return rc == 0


def run_scenarios(scenarios: list[int], out_container: str) -> list[int]:
    """Run each scenario in its own process. Returns those that failed."""
    failed = []
    for s in scenarios:
        rc = dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py "
                   f"--scenario {s} --output-dir {out_container}")
        if rc != 0:
            failed.append(s)
    return failed


def elementwise_bits(a: np.ndarray) -> np.ndarray:
    """One row of raw bytes per element, so equality is bit-identity.

    `==` is wrong here: NaN != NaN, so a channel that legitimately reproduces
    a NaN would report as mismatched forever.
    """
    flat = np.ascontiguousarray(a).ravel()
    return flat.view(np.uint8).reshape(flat.size, flat.dtype.itemsize)


def compare(baseline: Path, current: Path, scenarios: list[int]) -> dict:
    """Value-level comparison of every channel of every scenario."""
    out = {
        "compared": 0, "mismatched": 0,
        "channels_compared": 0, "channels_mismatched": 0,
        "missing_scenarios": [], "missing_channels": [],
        "shape_mismatches": [], "mismatched_channels": [],
    }
    for s in scenarios:
        b_path, c_path = baseline / f"scenario_{s}.npz", current / f"scenario_{s}.npz"
        if not (b_path.is_file() and c_path.is_file()):
            out["missing_scenarios"].append(s)
            continue
        B, C = np.load(b_path), np.load(c_path)
        for k in sorted(B.files):
            b = B[k]
            if k not in C.files:
                # Count it as compared-and-wrong. A channel the run stopped
                # emitting is a regression, not an absence of evidence.
                out["missing_channels"].append(f"scenario_{s}:{k}")
                out["compared"] += int(b.size)
                out["mismatched"] += int(b.size)
                out["channels_compared"] += 1
                out["channels_mismatched"] += 1
                continue
            c = C[k]
            out["channels_compared"] += 1
            if b.shape != c.shape:
                out["shape_mismatches"].append(
                    f"scenario_{s}:{k} baseline{b.shape} current{c.shape}")
                out["compared"] += int(b.size)
                out["mismatched"] += int(b.size)
                out["channels_mismatched"] += 1
                continue
            bad = int(np.any(elementwise_bits(b) != elementwise_bits(c), axis=1).sum())
            out["compared"] += int(b.size)
            out["mismatched"] += bad
            if bad:
                out["channels_mismatched"] += 1
                out["mismatched_channels"].append(f"scenario_{s}:{k} {bad}/{b.size}")
    return out


def snapshot_inputs() -> dict[Path, bytes]:
    """Capture the scenario input files before a run.

    `vit_sim.py`'s `write_discon()` REGENERATES `Examples/DISCON*.IN` from the
    turbine model and then patches per-scenario values into it, in place. It
    never restores them, so a gate run leaves the tree carrying whatever the
    last scenario wanted -- measured here as IPC_ControlMode 1->2 and
    F_NumNotchFilts 0->1.

    That matters twice over. `loop/done.py` runs with `require_clean_tree=True`,
    so a unit session that ran the gate could not close; and a session that
    resolved the dirt with `git add -A` would commit a SILENTLY RECONFIGURED
    gate for every unit that followed. An instrument that edits its own subject
    is not measuring it.
    """
    return {q: q.read_bytes() for q in sorted((ROOT / "Examples").glob("*.IN"))}


def restore_inputs(snap: dict[Path, bytes]) -> list[str]:
    """Put them back. Returns the ones that had actually been changed."""
    changed = []
    for q, original in snap.items():
        if q.read_bytes() != original:
            q.write_bytes(original)
            changed.append(str(q.relative_to(ROOT)))
    return changed


def residual_dirt() -> list[str]:
    """Anything still modified under Examples/ after restoring.

    Self-check rather than assumption: if the gate learns to touch a file this
    snapshot does not cover, this reports it instead of leaving it to be
    discovered as someone else's commit.
    """
    r = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "Examples"],
                       capture_output=True, text=True)
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def gate_once(scenarios: list[int], work: Path) -> dict:
    """Run the scenarios and compare. No build, no perturbation."""
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    snap = snapshot_inputs()
    try:
        failed = run_scenarios(scenarios, f"{WORKDIR}/{work.name}")
    finally:
        restored = restore_inputs(snap)
    res = compare(ROOT / "baseline_arrays", work, scenarios)
    res["scenarios"] = scenarios
    res["scenarios_failed"] = failed
    res["inputs_restored"] = restored
    res["residual_dirt"] = residual_dirt()
    return res


def verdict_of(res: dict) -> tuple[str, int]:
    """PASS only if something was compared and all of it matched."""
    broken = (res["scenarios_failed"] or res["missing_scenarios"]
              or res["mismatched"] or res["compared"] <= 0)
    return ("FAIL" if broken else "PASS"), (1 if broken else 0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="gate.py")
    ap.add_argument("unit", help="unit name, or __gate__ for phase-3 setup evidence")
    ap.add_argument("--scenarios", default=None,
                    help="comma-separated subset; default all 27")
    ap.add_argument("--perturb-file", default=None,
                    help="path relative to the campaign root; makes this a RED TEST")
    ap.add_argument("--perturb-from", default=None)
    ap.add_argument("--perturb-to", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)

    scenarios = ([int(x) for x in a.scenarios.replace(",", " ").split()]
                 if a.scenarios else SCENARIO_ORDER)
    perturbing = bool(a.perturb_file)
    if perturbing and (a.perturb_from is None or a.perturb_to is None):
        print("--perturb-file requires --perturb-from and --perturb-to", file=sys.stderr)
        return 2

    work = ROOT / (".gate_redtest" if perturbing else ".gate_current")
    default_name = f"{a.unit}.redtest.json" if perturbing else f"{a.unit}.json"
    dest = Path(a.out) if a.out else ROOT / "gate" / default_name
    dest.parent.mkdir(parents=True, exist_ok=True)

    started = time.time()
    telemetry(a.unit, "start", command=("gate_redtest" if perturbing else "gate"))

    payload: dict = {"unit": a.unit, "unit_of_count": "values",
                     "generated_by": "scripts/gate.py",
                     "baseline": "baseline_arrays", "perturbed": perturbing}
    try:
        if not perturbing:
            payload.update(gate_once(scenarios, work))
        else:
            target = ROOT / a.perturb_file
            if not target.is_file():
                print(f"no such file: {target}", file=sys.stderr)
                return 2
            original = target.read_text()
            n = original.count(a.perturb_from)
            if n == 0:
                print(f"--perturb-from not found in {a.perturb_file}", file=sys.stderr)
                return 2
            payload["perturbation"] = {"file": a.perturb_file, "from": a.perturb_from,
                                       "to": a.perturb_to, "replacements": n}
            try:
                target.write_text(original.replace(a.perturb_from, a.perturb_to))
                if not build_and_install():
                    print("PERTURBED BUILD FAILED -- no red test was performed", file=sys.stderr)
                    payload["build_failed"] = True
                    return 2
                payload.update(gate_once(scenarios, work))
            finally:
                # Unconditional. A perturbation left in the tree is a corrupted
                # campaign, and the failure mode is silent.
                target.write_text(original)
                restored = build_and_install()
            payload["reverted"] = True
            payload["revert_build_ok"] = restored
            # RUNBOOK.md's lesson: a red is only attributable to the
            # perturbation if green comes back when it is removed.
            after = gate_once(scenarios, ROOT / ".gate_revert")
            payload["revert_compared"] = after["compared"]
            payload["revert_mismatched"] = after["mismatched"]
            payload["revert_verified"] = (after["mismatched"] == 0 and after["compared"] > 0)
            # ADDED at unit #15, by ADDITION: a perturbation can be visible to
            # the simulation without moving a single compared VALUE, by stopping
            # the scenario from producing output at all. PathIsRelative forced
            # to answer .TRUE. prefixes PriPath onto an already-absolute
            # PerfFileName, and 24 of the 27 scenarios die in the Fortran
            # runtime rather than finishing wrong
            # (evidence/PathIsRelative/gate.always-true-scenario1-runtime-error.log).
            # `went_red` counts value mismatches and so reads False, and the
            # message under it -- "the line is never executed, or the gate
            # cannot observe it" -- is then FALSE of this run.
            #
            # `went_red` and the exit code are deliberately NOT changed: every
            # artifact this campaign has already committed means what it says,
            # and re-defining the verdict mid-run is exactly what X3 forbids.
            # What is added is the measurement a reader needs, named so it
            # cannot be missed, plus an accurate message below.
            payload["revert_scenarios_failed"] = after["scenarios_failed"]
            payload["perturbation_broke_scenarios"] = sorted(
                set(payload["scenarios_failed"]) - set(after["scenarios_failed"]))

        v, rc = verdict_of(payload)
        payload["verdict"] = v

        if perturbing:
            # A red test INVERTS the verdict: not going red is the failure.
            went_red = payload["mismatched"] > 0 and payload["compared"] > 0
            payload["went_red"] = went_red
            rc = 0 if (went_red and payload["revert_verified"]) else 1
            # AND THE ARTIFACT MUST SAY SO. Until unit #1 `verdict` was left at
            # the COMPARISON's verdict on a perturbing run, so a red test that
            # correctly went red wrote `verdict: FAIL` and one that failed to
            # go red wrote `verdict: PASS`. Both are backwards to any reader who
            # does not already know which kind of run produced the file, and
            # AddToList produced the dangerous half: a gate that cannot see the
            # unit at all, filed under `PASS`.
            #
            # The comparison's own verdict is kept, under its own name, because
            # it is a real measurement -- it is `revert_verified`'s sibling and
            # says whether the PERTURBED build still matched baseline.
            #
            # The new spelling is deliberately not `PASS`/`FAIL`: artifacts
            # written before this change carry the old vocabulary, and a reader
            # can tell which convention a file uses from the value itself
            # instead of from its date. `went_red` means the same thing in both
            # and predates this.
            payload["comparison_verdict"] = payload["verdict"]
            payload["verdict"] = "RED_TEST_PASS" if rc == 0 else "RED_TEST_FAIL"
            print(f"RED TEST {'PASS' if rc == 0 else 'FAIL'}: perturbation moved "
                  f"{payload['mismatched']} of {payload['compared']} compared value(s); "
                  f"after revert {payload['revert_mismatched']} of "
                  f"{payload['revert_compared']}")
            broke = payload.get("perturbation_broke_scenarios") or []
            if not went_red and broke:
                print(f"  NOT 'the gate cannot see it': the perturbation STOPPED "
                      f"{len(broke)} of {len(scenarios)} scenario(s) from running at all "
                      f"({broke}), and those scenarios ran on the revert. A scenario that "
                      f"dies produces no values to mismatch, so `went_red` -- which counts "
                      f"VALUES -- reads False. Read `perturbation_broke_scenarios`.",
                      file=sys.stderr)
            elif not went_red:
                print("  the gate did not see the perturbation -- either the line is "
                      "never executed by these scenarios, or the gate cannot observe it. "
                      "See RUNBOOK.md: this is exactly attempt 1.", file=sys.stderr)
        else:
            print(f"GATE {v}: compared {payload['compared']} value(s) across "
                  f"{payload['channels_compared']} channel(s) / {len(scenarios)} scenario(s); "
                  f"mismatched {payload['mismatched']}")
            for m in payload["mismatched_channels"][:20]:
                print(f"  MISMATCH {m}")
            if payload["compared"] <= 0:
                print("  compared NOTHING -- a green here would be vacuous", file=sys.stderr)

        dest.write_text(json.dumps(payload, indent=1, sort_keys=True) + "\n")
        # `relative_to` RAISES on an --out outside the campaign root, after the
        # artifact is already written and after the verdict is already decided.
        # The traceback then replaces the exit code, so a red gate exits 1 for
        # the wrong reason and a green one exits 1 too. Observed with
        # --out /tmp/...json on 2026-08-10.
        try:
            print(f"wrote {dest.relative_to(ROOT)}")
        except ValueError:
            print(f"wrote {dest}")
        return rc
    finally:
        telemetry(a.unit, "end", command=("gate_redtest" if perturbing else "gate"),
                  exit_code=0, duration_s=int(time.time() - started))


if __name__ == "__main__":
    raise SystemExit(main())
