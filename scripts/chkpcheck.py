#!/usr/bin/env python3
"""The instrument for unit `WriteRestartFile`: byte-identity of the checkpoint
files it writes.

WHY THIS EXISTS, AND WHY NEITHER OTHER LAYER CAN DO IT.

`WriteRestartFile` is called from exactly one place, `DISCON.F90:101`, behind
`LocalVar%iStatus == -8`. `coverage/line_coverage.json` records NO hit on that
line and none on any line of the routine: across all 27 gate scenarios the unit
is never called. So this is a stronger statement than unit #31's `Debug`, where
the gate ran the unit 408,000 times and read a different output stream --
here the gate does not reach the unit at all, and a gate run over it compares
the same 5,252,000 values whether the translation is right, wrong or absent.

A generated differential harness cannot decide it either. The unit's mapped
signature is `(LocalVar IN, CntrPar INOUT, ErrVar INOUT, objInst INOUT,
RootName IN, size_avcOUTNAME IN)`; it assigns nothing in `CntrPar` or
`objInst`, and `ErrVar` only on the two I/O-failure arms. Worse, both sides of
such a harness would open the SAME path and the second write would overwrite
the first, so the comparison would be of one program against itself.

What CAN compare it is the file. `Examples/vit_sim.py` scenarios 36, 37 and 38
drive `iStatus = -8` on a schedule, so each writes 39 `*.RO.chkp` files; this
script archives them and compares two archives byte for byte.

THE TWO ARCHIVES DIFFER IN ONE THING. Take the first with `WriteRestartFile`
still Fortran, integrate, rebuild, take the second. Every other unit is in the
same state in both builds, so a difference is this unit's. `dbgcheck.py`'s
argument, one unit over.

NOTHING IN A CHECKPOINT IS EXEMPT FROM COMPARISON. `Debug` had to excuse a
`Generated on <date> at <time>` header; an unformatted STREAM write emits no
header, no record marker and no wall clock, so every one of the 460,813 bytes
is compared. That is the whole file: the layout was derived from the reference's
own 307 `WRITE` statements and its predicted length matches the produced file
exactly (evidence/WriteRestartFile/layout.txt).

    python3 scripts/chkpcheck.py capture --label pre
    python3 scripts/chkpcheck.py compare --a pre --b post \
        --out evidence/WriteRestartFile/chkp.json

`capture` runs the scenarios; `--no-run` archives whatever is already there.
Exits non-zero on any mismatch, on a missing file, and on comparing nothing.

COPIED, NOT REWRITTEN (P4). `dbgcheck.py` at campaign rev f5dbe5c9 is the
source for `dexec`, `_rev`, `instrument_revs`, the capture/compare command
shape and the refuse-on-empty rule. What differs is stated rather than left to
be diffed:

  * the stream is BINARY and fixed-length, so the comparison is over BYTES and
    OFFSETS, not over `\\n`-separated records. A checkpoint holds IEEE doubles;
    a 0x0A inside one is data.
  * there is no header exemption, for the reason above.
  * the driver's stdout is NOT a stream of this unit -- `WriteRestartFile`
    writes nothing to unit 6 -- so it is not compared, and saying so here is
    the point: `Debug`'s fourth stream was invisible until somebody looked.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTAINER = os.environ.get("VIT_CONTAINER", "vit-dev")
WORKDIR = f"/workspace/{ROOT.name}"

# The checkpoint scenarios. NOT `scenario_order`: 36-38 are registered outside
# it, so no gate run and no committed baseline is touched by their existence.
SCENARIO_ORDER = [36, 37, 38, 39, 40, 41]

ARCHIVE = ROOT / "evidence" / "WriteRestartFile" / "chkp"

# Where the files land. `vit_sim.py` runs with `Examples` as its working
# directory and the RootName it passes is relative, so `<n>.RO.chkp` is written
# beside the simulation script. Same rule dbgcheck.py records for `.RO.dbg`.
CHKPDIR = ROOT / "Examples"
PATTERN = "*.RO.chkp"

# How many leading bytes of a differing region to record. A checkpoint is
# 460,813 bytes and a mismatching mutant can differ in most of them; an
# artifact that inlined all of it would be unreadable.
MAX_DIFFS = 40


def dexec(script: str, quiet: bool = True, out_path: Path | None = None) -> int:
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
        # this run did not write.
        for p in CHKPDIR.glob(PATTERN):
            p.unlink()
        for s in scenarios:
            t0 = time.time()
            rc = dexec(f"cd {WORKDIR}/Examples && python3 vit_sim.py --scenario {s}",
                       out_path=dest / f"stdout.{s}.txt")
            print(f"  scenario {s}: rc={rc} {time.time() - t0:.0f}s", flush=True)
            if rc != 0:
                failed.append(s)

    files = sorted(CHKPDIR.glob(PATTERN))
    for p in files:
        shutil.copy2(p, dest / p.name)

    meta = {
        "label": label,
        "ran_scenarios": scenarios if run else [],
        "scenarios_failed": failed,
        "files": {p.name: {"bytes": p.stat().st_size,
                           "md5": hashlib.md5(p.read_bytes()).hexdigest()}
                  for p in files},
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


def compare_bytes(a: bytes, b: bytes) -> tuple[int, int]:
    """(number of differing bytes, offset of the first). (0, -1) if identical.

    Length is compared by the caller; this walks the common prefix and counts
    the tail as differing.
    """
    if a == b:
        return 0, -1
    n = min(len(a), len(b))
    first = -1
    diff = abs(len(a) - len(b))
    for i in range(n):
        if a[i] != b[i]:
            diff += 1
            if first < 0:
                first = i
    if first < 0 and diff:
        first = n
    return diff, first


def compare(a_label: str, b_label: str, out: Path | None) -> int:
    A, B = ARCHIVE / a_label, ARCHIVE / b_label
    names_a = {p.name for p in A.glob(PATTERN)}
    names_b = {p.name for p in B.glob(PATTERN)}

    rep = {
        "a": a_label, "b": b_label,
        "files_compared": 0, "files_mismatched": 0,
        "bytes_compared": 0, "bytes_mismatched": 0,
        "only_in_a": sorted(names_a - names_b),
        "only_in_b": sorted(names_b - names_a),
        "mismatches": [],
        **instrument_revs(),
    }

    for name in sorted(names_a & names_b):
        da = (A / name).read_bytes()
        db = (B / name).read_bytes()
        rep["files_compared"] += 1
        rep["bytes_compared"] += len(da)
        nd, first = compare_bytes(da, db)
        if nd:
            rep["files_mismatched"] += 1
            rep["bytes_mismatched"] += nd
            if len(rep["mismatches"]) < MAX_DIFFS:
                rep["mismatches"].append(
                    {"file": name, "bytes_differing": nd,
                     "first_diff_offset": first,
                     "a_len": len(da), "b_len": len(db),
                     "a": da[first:first + 16].hex(),
                     "b": db[first:first + 16].hex()})

    # THE SECOND STREAM, AND IT IS THE UNIT'S OTHER OUTPUT.
    #
    # `WriteRestartFile` writes `ErrVar%aviFAIL` and `ErrVar%ErrMsg` on both of
    # its failure arms, and nothing here read them: DISCON's own `print` of the
    # message is guarded by `aviFAIL < 0` and this unit sets 1. DISCON copies it
    # into `avcMSG` regardless, `_chkp_drive` prints that buffer on every
    # checkpoint step, and this compares the driver's stdout as bytes.
    #
    # Added after the first sweep, which is the honest order: eleven of the
    # first twenty-two mutants survived and every one of them was in an error
    # path with no oracle. `Debug`'s fourth stream, one unit later.
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
                if len(rep["mismatches"]) < MAX_DIFFS:
                    rep["mismatches"].append(
                        {"file": name, "record": i + 1, "stream": "stdout",
                         "a": la[:160].decode("latin-1"),
                         "b": lb[:160].decode("latin-1")})

    rep["verdict"] = ("IDENTICAL" if (rep["bytes_mismatched"] == 0
                                      and not rep["only_in_a"] and not rep["only_in_b"]
                                      and rep["files_compared"] > 0
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
          f"{rep['bytes_compared']} bytes, {rep['bytes_mismatched']} mismatched "
          f"in {rep['files_mismatched']} file(s)")
    print(f"  stdout: {rep['stdout_scenarios_compared']} scenario(s), "
          f"{rep['stdout_records_compared']} records, "
          f"{rep['stdout_bytes_compared']} bytes, "
          f"{rep['stdout_records_mismatched']} mismatched")
    if rep["files_compared"] == 0:
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
