#!/usr/bin/env python3
"""Per-scenario line coverage of the ROSCO controller. C2's input.

C2 says "select the call site and test case from coverage data". This campaign
had none: E2.3 is still `manual` in phases.toml, and the only thing available
was reasoning about which scenario name sounds like it reaches a call site.
Reasoning about reachability is what the RUNBOOK's first red-test attempt did,
and it produced a green from a line that never ran.

So this measures it. A separate `--coverage -O0` build (its own directory, so
the Release build the gate measures is never touched), each scenario run in its
own process with the counters cleared first, `gcov` after each. The result is a
per-scenario hit count per line, which answers both questions C2 asks: which
call site, and which scenario.

    python3 scripts/coverage.py --out coverage/line_coverage.json
    python3 scripts/coverage.py --scenarios 6,2 --files Functions.f90

NOT a substitute for E2.3. This build carries `--coverage -O0`, not the
campaign's Release flags, so it says which LINES ran, not what the gated build
computed. E2.3 asks for coverage from a clean build committed as phase
evidence; that is Phase 2 work and remains open.

The scenarios rewrite `Examples/*.IN` in place (see gate.py's snapshot_inputs);
this restores them the same way, and reports anything it could not.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTAINER = os.environ.get("VIT_CONTAINER", "vit-dev")
WORKDIR = f"/workspace/{ROOT.name}"
BUILD = "rosco/controller/build_cov"
SCENARIOS = [3, 4, 5, 1, 2] + list(range(6, 28))

# `<hits>:<line>:<source>`; `-` is a non-instrumented line, `#####` never ran.
_GCOV = re.compile(r"^\s*(-|#####|\$\$\$\$\$|\d+\*?):\s*(\d+):(.*)$")


def dexec(script: str, quiet: bool = True) -> int:
    kw = {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL} if quiet else {}
    return subprocess.run(["docker", "exec", CONTAINER, "bash", "-lc", script], **kw).returncode


def build() -> None:
    """A coverage build in its own directory, installed where the sims load it."""
    rc = dexec(
        f"cd {WORKDIR}/rosco/controller && rm -rf {WORKDIR}/{BUILD} && "
        f"mkdir -p {WORKDIR}/{BUILD} && cd {WORKDIR}/{BUILD} && "
        f"cmake .. -DCMAKE_BUILD_TYPE=Debug "
        f"-DCMAKE_Fortran_FLAGS='--coverage -O0' && cmake --build . -j4 && "
        f"cp libdiscon.so {WORKDIR}/rosco/lib/libdiscon.so", quiet=False)
    if rc != 0:
        sys.exit("coverage build failed")


def hits_for(files: list[str]) -> dict[str, dict[int, int]]:
    """gcov the current counters; {file: {line: hits}} for instrumented lines."""
    # The .gcno is named `Functions.f90.gcno`, and `gcov -o <dir> <src>.f90`
    # strips the extension and looks for `Functions.gcno` -- it then prints
    # "cannot open notes file / No executable lines" and exits 0. Naming the
    # notes file directly is what actually reads the counters. The first run of
    # this script reported 0/0 lines for all 27 scenarios because of it, which
    # is only visible at all because the denominator is printed next to the
    # numerator.
    objdir = f"{WORKDIR}/{BUILD}/CMakeFiles/discon.dir/src"
    dexec(f"cd {WORKDIR}/{BUILD} && rm -f *.gcov && "
          f"gcov {' '.join(f'{objdir}/{f}.gcno' for f in files)}")
    out: dict[str, dict[int, int]] = {}
    for f in files:
        p = ROOT / BUILD / f"{f}.gcov"
        if not p.is_file():
            out[f] = {}
            continue
        lines: dict[int, int] = {}
        for ln in p.read_text(errors="replace").splitlines():
            if (m := _GCOV.match(ln)) and m.group(1) not in ("-",):
                n = 0 if m.group(1).startswith(("#", "$")) else int(m.group(1).rstrip("*"))
                lines[int(m.group(2))] = n
        out[f] = lines
    return out


def snapshot_inputs() -> dict[Path, bytes]:
    return {q: q.read_bytes() for q in sorted((ROOT / "Examples").glob("*.IN"))}


def restore_inputs(snap: dict[Path, bytes]) -> list[str]:
    changed = []
    for q, original in snap.items():
        if q.read_bytes() != original:
            q.write_bytes(original)
            changed.append(str(q.relative_to(ROOT)))
    return changed


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--files", default="Functions.f90,Controllers.f90",
                    help="comma-separated source basenames under rosco/controller/src")
    ap.add_argument("--scenarios", default=None, help="comma-separated; default all 27")
    ap.add_argument("--out", default="coverage/line_coverage.json")
    ap.add_argument("--no-build", action="store_true")
    args = ap.parse_args()

    files = [f.strip() for f in args.files.split(",") if f.strip()]
    scen = ([int(s) for s in args.scenarios.split(",")] if args.scenarios else SCENARIOS)

    if not args.no_build:
        build()

    snap = snapshot_inputs()
    per: dict[str, dict[str, dict[str, int]]] = {f: {} for f in files}
    failed: list[int] = []
    try:
        for s in scen:
            dexec(f"cd {WORKDIR}/{BUILD} && find . -name '*.gcda' -delete")
            rc = dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py --scenario {s} "
                       f"--output-dir /tmp/cov_{s}")
            if rc != 0:
                failed.append(s)
                print(f"  scenario {s}: FAILED to run", flush=True)
                continue
            got = hits_for(files)
            for f in files:
                for line, n in got[f].items():
                    if n:
                        per[f].setdefault(str(line), {})[str(s)] = n
            print(f"  scenario {s}: "
                  + ", ".join(f"{f} {sum(1 for v in got[f].values() if v)}/{len(got[f])} lines"
                              for f in files), flush=True)
    finally:
        restored = restore_inputs(snap)

    payload = {
        "build": "--coverage -O0 (NOT the gated Release build)",
        "scenarios": scen, "scenarios_failed": failed,
        "inputs_restored": restored,
        "hits": per,
    }
    dest = ROOT / args.out
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(payload, indent=1) + "\n")
    print(f"\nwrote {dest}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
