#!/usr/bin/env python3
"""The instrument for unit `Debug`: byte-identity of the files it writes.

WHY THIS EXISTS, AND WHY THE GATE CANNOT DO IT.

`scripts/gate.py` compares `baseline_arrays/scenario_N.npz`, and those arrays
come from `Examples/vit_sim.py`'s own record of `avrSWAP` -- the CONTROLLER's
outputs. `Debug` writes none of them. It writes `<RootName>.RO.dbg`, which
nothing in the gate path reads, and it assigns no argument of its own signature
(`ErrVar` only on GetNewUnit's exhaustion arm, `avrSWAP` never). So the gate is
blind to this unit by construction, exactly as plan.json says, and a
differential harness over its mapped signature would compare an empty set.

What CAN compare it is the file. This script runs the 27 scenarios, archives
every `*.RO.dbg*` they write, and compares two archives byte for byte.

THE TWO ARCHIVES DIFFER IN ONE THING. Take the first with `Debug` still
Fortran, integrate, rebuild, take the second. Every other unit is in the same
state in both builds, so a difference is this unit's.

THE ONE LINE THAT CANNOT BE COMPARED IS NAMED RATHER THAN SKIPPED. Record 1 is
    ' Generated on 10-Aug-2026 at 10:44:58 using ROSCO-2.10.1'
whose date and time come from `DATE_AND_TIME`. It is compared STRUCTURALLY --
same length, same fixed text, digits where digits belong -- and the two
variable fields are excluded by NAME, not by dropping the line. A line dropped
from a comparison is a line the comparison cannot report on.

    python3 scripts/dbgcheck.py capture --label pre
    python3 scripts/dbgcheck.py compare --a pre --b post --out evidence/Debug/dbg.json

`capture` runs the scenarios; `--no-run` archives whatever is already there.
Exits non-zero on any mismatch, on a missing file, and on comparing nothing.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTAINER = os.environ.get("VIT_CONTAINER", "vit-dev")
WORKDIR = f"/workspace/{ROOT.name}"

# gate.py's order, kept so the two instruments drive the controller the same way.
SCENARIO_ORDER = [3, 4, 5, 1, 2] + list(range(6, 28))

ARCHIVE = ROOT / "evidence" / "Debug" / "dbg"

# Where the files actually land. `vit_sim.py` runs with `Examples` as its
# working directory and the RootName it passes is relative, so `.RO.dbg` is
# written BESIDE the simulation script -- not at the campaign root, where an
# older run left a set of stale copies that would have been archived as this
# run's output had this been guessed instead of looked up.
DBGDIR = ROOT / "Examples"

# Record 1 of every .RO.dbg / .RO.dbg2. The two bracketed groups are the wall
# clock and are the ONLY bytes this instrument declines to compare.
GENERATED_ON = re.compile(
    rb"^ Generated on (\d{2}-[A-Z][a-z]{2}-\d{4}) at (\d{2}:\d{2}:\d{2})"
    rb" using ROSCO-\d+\.\d+\.\d+$")


def dexec(script: str, quiet: bool = True, out_path: Path | None = None) -> int:
    """Run in the container. With `out_path`, the run's STDOUT is written there.

    THE UNIT WRITES TO FOUR STREAMS AND THIS ARCHIVED THREE. `WRITE(*,100)` --
    the ten-second status line -- goes to unit 6, and nothing in this campaign
    read it: the gate compares avrSWAP channels and this script compared
    `*.RO.dbg*`. Seven of the unit's 57 mutation survivors sit on that line.

    It is comparable, and the first comparison found a real difference (C12,
    evidence/Debug/stdout.DEFECT-unflushed-status-record.*): libgfortran emits a
    preconnected unit's record whole, while a fully-buffered `stdout` split one
    record mid-field and delivered eighteen more after the driver's own output.
    The stream is deterministic -- three runs byte-identical -- so it is
    compared as bytes and not as a filtered projection.
    """
    if out_path is not None:
        with open(out_path, "wb") as fh:
            return subprocess.run(["docker", "exec", CONTAINER, "bash", "-lc", script],
                                  stdout=fh, stderr=subprocess.DEVNULL).returncode
    kw = {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL} if quiet else {}
    return subprocess.run(["docker", "exec", CONTAINER, "bash", "-lc", script], **kw).returncode


def _rev(root: Path) -> str:
    """The revision of an instrument checkout, or `unknown`. gate.py's rule."""
    try:
        r = subprocess.run(["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
                           capture_output=True, text=True, timeout=10)
        rev = r.stdout.strip()
        if rev:
            d = subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                               capture_output=True, text=True, timeout=10)
            return rev + ("-dirty" if d.stdout.strip() else "")
    except Exception:
        pass
    try:
        head = (root / ".git" / "HEAD").read_text().strip()
        if head.startswith("ref: "):
            ref = (root / ".git" / head[5:]).read_text().strip()
            return f"{ref[:7]}-nogit"
    except Exception:
        pass
    return "unknown"


def instrument_revs() -> dict:
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


def capture(label: str, run: bool, scenarios: list[int]) -> int:
    dest = ARCHIVE / label
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)

    failed = []
    if run:
        # Remove any leftovers first, so an archive can never contain a file
        # this run did not write. A stale .dbg compared against a fresh one is
        # the "green that measured nothing" shape with the sign flipped.
        for p in DBGDIR.glob("*.RO.dbg*"):
            p.unlink()
        for s in scenarios:
            t0 = time.time()
            rc = dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py --scenario {s}",
                       out_path=dest / f"stdout.{s}.txt")
            print(f"  scenario {s}: rc={rc} {time.time() - t0:.0f}s", flush=True)
            if rc != 0:
                failed.append(s)

    files = sorted(DBGDIR.glob("*.RO.dbg*"))
    for p in files:
        shutil.copy2(p, dest / p.name)

    outs = sorted(dest.glob("stdout.*.txt"))
    meta = {
        "label": label,
        "ran_scenarios": scenarios if run else [],
        "scenarios_failed": failed,
        "files": {p.name: {"bytes": p.stat().st_size,
                           "md5": hashlib.md5(p.read_bytes()).hexdigest()}
                  for p in files},
        # ADDED, not substituted: an archive taken before the stdout stream was
        # archived has no `stdout` key at all, and compare() then reports zero
        # stdout records compared rather than silently passing.
        "stdout": {p.name: {"bytes": p.stat().st_size,
                            "md5": hashlib.md5(p.read_bytes()).hexdigest()}
                   for p in outs},
        **instrument_revs(),
    }
    (dest / "_manifest.json").write_text(json.dumps(meta, indent=2, sort_keys=True))
    print(f"captured {len(files)} file(s) into {dest.relative_to(ROOT)}")
    if failed:
        print(f"SCENARIOS FAILED: {failed}", file=sys.stderr)
        return 1
    if not files:
        print("captured NOTHING -- refusing", file=sys.stderr)
        return 1
    return 0


def compare_first_record(a: bytes, b: bytes, rep: dict) -> bool:
    """Record 1: BYTES first, the wall-clock exemption only if they differ.

    The first version asserted the `Generated on ...` pattern on both sides and
    failed anything else. That held for `.RO.dbg` and `.RO.dbg2` and broke the
    first time a third file appeared: `.RO.dbg3` opens with
    `WRITE(UnDb3,'(/////)')`, so its record 1 is EMPTY -- and two empty records
    were reported as a mismatch (C12, the artifact is committed as
    evidence/Debug/dbg28.INSTRUMENT-DEFECT-header-assumed-on-dbg3.json).

    Ordering the two tests this way is not just the repair, it is stronger than
    what it replaces: record 1 is now COMPARED, byte for byte, wherever it can
    be, and the exemption is reached only by a record that actually differs and
    that both sides render in the one shape whose two fields are a wall clock.
    """
    if a == b:
        return True
    ma, mb = GENERATED_ON.match(a), GENERATED_ON.match(b)
    if not ma or not mb:
        rep["header_malformed"] = {"a": a.decode("latin-1"), "b": b.decode("latin-1")}
        return False
    # Everything outside the two groups must be byte-equal, which for this
    # pattern means the surrounding literal text and the record length.
    if len(a) != len(b):
        rep["header_length"] = {"a": len(a), "b": len(b)}
        return False
    return True


def compare(a_label: str, b_label: str, out: Path | None) -> int:
    A, B = ARCHIVE / a_label, ARCHIVE / b_label
    names_a = {p.name for p in A.glob("*.RO.dbg*")}
    names_b = {p.name for p in B.glob("*.RO.dbg*")}

    rep = {
        "a": a_label, "b": b_label,
        "files_compared": 0, "files_mismatched": 0,
        "records_compared": 0, "records_mismatched": 0,
        "bytes_compared": 0,
        "only_in_a": sorted(names_a - names_b),
        "only_in_b": sorted(names_b - names_a),
        "mismatches": [],
        "header_records_compared": 0,
        **instrument_revs(),
    }

    for name in sorted(names_a & names_b):
        ra = (A / name).read_bytes().split(b"\n")
        rb = (B / name).read_bytes().split(b"\n")
        rep["files_compared"] += 1
        bad = 0
        n = max(len(ra), len(rb))
        for i in range(n):
            la = ra[i] if i < len(ra) else None
            lb = rb[i] if i < len(rb) else None
            if la is None or lb is None:
                bad += 1
                if len(rep["mismatches"]) < 40:
                    rep["mismatches"].append(
                        {"file": name, "record": i + 1, "reason": "record count differs",
                         "a_records": len(ra), "b_records": len(rb)})
                continue
            rep["records_compared"] += 1
            rep["bytes_compared"] += len(la)
            if i == 0:
                rep["header_records_compared"] += 1
                if not compare_first_record(la, lb, rep):
                    bad += 1
                    if len(rep["mismatches"]) < 40:
                        rep["mismatches"].append(
                            {"file": name, "record": 1, "reason": "header",
                             "a": la.decode("latin-1"), "b": lb.decode("latin-1")})
                continue
            if la != lb:
                bad += 1
                if len(rep["mismatches"]) < 40:
                    k = next((j for j in range(min(len(la), len(lb))) if la[j] != lb[j]),
                             min(len(la), len(lb)))
                    rep["mismatches"].append(
                        {"file": name, "record": i + 1, "first_diff_byte": k,
                         "a": la[max(0, k - 20):k + 20].decode("latin-1"),
                         "b": lb[max(0, k - 20):k + 20].decode("latin-1")})
        rep["records_mismatched"] += bad
        if bad:
            rep["files_mismatched"] += 1

    # The fourth stream. Compared as whole bytes, per scenario, and only for
    # the scenarios BOTH archives recorded -- an archive taken before this
    # existed contributes 0 and says so in `stdout_scenarios_compared`.
    outs_a = {p.name for p in A.glob("stdout.*.txt")}
    outs_b = {p.name for p in B.glob("stdout.*.txt")}
    rep["stdout_only_in_a"] = sorted(outs_a - outs_b)
    rep["stdout_only_in_b"] = sorted(outs_b - outs_a)
    rep["stdout_scenarios_compared"] = 0
    rep["stdout_bytes_compared"] = 0
    rep["stdout_records_compared"] = 0
    rep["stdout_records_mismatched"] = 0
    for name in sorted(outs_a & outs_b):
        ra = (A / name).read_bytes().split(b"\n")
        rb = (B / name).read_bytes().split(b"\n")
        rep["stdout_scenarios_compared"] += 1
        for i in range(max(len(ra), len(rb))):
            la = ra[i] if i < len(ra) else None
            lb = rb[i] if i < len(rb) else None
            if la is None or lb is None:
                rep["stdout_records_mismatched"] += 1
                continue
            rep["stdout_records_compared"] += 1
            rep["stdout_bytes_compared"] += len(la)
            if la != lb:
                rep["stdout_records_mismatched"] += 1
                if len(rep["mismatches"]) < 40:
                    rep["mismatches"].append(
                        {"file": name, "record": i + 1, "stream": "stdout",
                         "a": la[:120].decode("latin-1"), "b": lb[:120].decode("latin-1")})

    rep["verdict"] = ("IDENTICAL" if (rep["records_mismatched"] == 0
                                      and not rep["only_in_a"] and not rep["only_in_b"]
                                      and rep["records_compared"] > 0
                                      and rep["stdout_records_mismatched"] == 0
                                      and not rep["stdout_only_in_a"]
                                      and not rep["stdout_only_in_b"])
                      else "MISMATCH")
    text = json.dumps(rep, indent=2, sort_keys=True)
    if out:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text)
    print(text if len(text) < 4000 else json.dumps(
        {k: v for k, v in rep.items() if k != "mismatches"}, indent=2, sort_keys=True))
    print(f"\n{rep['verdict']}: {rep['files_compared']} file(s), "
          f"{rep['records_compared']} records, {rep['bytes_compared']} bytes, "
          f"{rep['records_mismatched']} mismatched")
    print(f"  stdout: {rep['stdout_scenarios_compared']} scenario(s), "
          f"{rep['stdout_records_compared']} records, "
          f"{rep['stdout_bytes_compared']} bytes, "
          f"{rep['stdout_records_mismatched']} mismatched")
    if rep["records_compared"] == 0:
        print("compared NOTHING -- refusing", file=sys.stderr)
        return 2
    return 0 if rep["verdict"] == "IDENTICAL" else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("capture")
    c.add_argument("--label", required=True)
    c.add_argument("--no-run", action="store_true")
    c.add_argument("--scenarios", default="")

    d = sub.add_parser("compare")
    d.add_argument("--a", required=True)
    d.add_argument("--b", required=True)
    d.add_argument("--out", default=None)

    args = ap.parse_args()
    if args.cmd == "capture":
        scen = ([int(x) for x in args.scenarios.split(",")] if args.scenarios
                else SCENARIO_ORDER)
        return capture(args.label, not args.no_run, scen)
    return compare(args.a, args.b, Path(args.out) if args.out else None)


if __name__ == "__main__":
    sys.exit(main())
