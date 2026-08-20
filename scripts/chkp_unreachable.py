#!/usr/bin/env python3
"""Derive, rather than assert, the `unreachable` set for WriteRestartFile's
index_offset survivors -- and re-derive it every time it is used.

RUNBOOK: "RE-DERIVE AN `unreachable` SET FROM ITS COVERAGE FILE; CARRYING ONE
FORWARD FAILS P12 OUTRIGHT." The coverage file here is the reference archive.

THE CLAIM AND THE PROOF. An `index_offset` mutant on this unit turns
`wr_dbl(Un, LocalVar->X[i])` into `wr_dbl(Un, LocalVar->X[i + 1])`. The oracle
compares the bytes written, so the mutant is observable exactly when `X(i+1)`
and `X(i+2)` differ -- in Fortran's numbering, when the item at the mutated
offset differs from the item eight bytes later. This reads the 195 REFERENCE
checkpoints and reports, per pair, in how many of them they differ. Zero of 195
is the proof; anything else is a mutant the corpus can kill and the declaration
would be wrong.

The pairs are read out of the mutation artifacts' own survivor records, so the
set cannot drift from the sweep, and out of
`evidence/WriteRestartFile/layout.txt`, whose offsets are themselves derived
from the reference Fortran and checked against a real checkpoint.

    python3 scripts/chkp_unreachable.py \\
        --part mutation/WriteRestartFile.index_offset.0.json \\
        --part mutation/WriteRestartFile.index_offset.1.json \\
        --archive evidence/WriteRestartFile/chkp/pre \\
        --out mutation/WriteRestartFile.unreachable.json \\
        --report evidence/WriteRestartFile/unreachable.txt

Exits non-zero if any declared pair DOES differ somewhere in the archive.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAYOUT = ROOT / "evidence/WriteRestartFile/layout.txt"

# `wr_dbl(Un, LocalVar->FA_PitCom[0]);` -> ('FA_PitCom', 0)
SITE = re.compile(r"wr_(?:dbl|cpx)\(Un, (?:LocalVar|objInst)->([A-Za-z_0-9]+)\[(\d+)\]\)")


def load_layout() -> dict:
    rows = {}
    for ln in LAYOUT.read_text().split("\n"):
        m = re.match(r"\s+(\d+)\s+(\d+)\s+(\S+)\s+(\S+)\s*$", ln)
        if m:
            rows[m.group(4)] = (int(m.group(1)), int(m.group(2)), m.group(3))
    if not rows:
        raise SystemExit("chkp_unreachable: layout.txt has no item table -- "
                         "regenerate it with scripts/chkplayout.py --census")
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--part", action="append", required=True)
    ap.add_argument("--archive", required=True)
    ap.add_argument("--cpp", default="rosco/controller/src/writerestartfile.cpp")
    ap.add_argument("--out", required=True)
    ap.add_argument("--report", required=True)
    a = ap.parse_args()

    src = (ROOT / a.cpp).read_text().split("\n")
    layout = load_layout()
    blobs = [p.read_bytes() for p in sorted(Path(a.archive).glob("*.RO.chkp"))]
    if not blobs:
        print("no reference checkpoints -- refusing", file=sys.stderr)
        return 2

    declared, lines, bad = {}, [], []
    lines.append(f"index_offset survivors declared UNREACHABLE, derived from "
                 f"{len(blobs)} reference checkpoints in {a.archive}")
    lines.append("")
    lines.append("  mutant   item                          off      off+8   differ")
    for part in a.part:
        d = json.loads((ROOT / part).read_text())
        for s in d["survivors"]:
            if s["operator"] != "index_offset":
                continue
            m = SITE.search(src[s["line"] - 1])
            if not m:
                lines.append(f"  {s['id']}  NOT A DATA SITE -- {src[s['line']-1].strip()}")
                continue
            field, idx = m.group(1), int(m.group(2))
            item = f"LocalVar%{field}({idx + 1})"
            nxt = f"LocalVar%{field}({idx + 2})"
            if item not in layout:
                lines.append(f"  {s['id']}  {item} NOT IN LAYOUT -- skipped")
                continue
            off, size, state = layout[item]
            noff = layout[nxt][0] if nxt in layout else off + size
            differ = sum(1 for b in blobs if b[off:off + size] != b[noff:noff + size])
            lines.append(f"  {s['id']} {item:<28} {off:>7} {noff:>7}   "
                         f"{differ} of {len(blobs)}   [{state}]")
            if differ:
                bad.append((s["id"], item, differ))
                continue
            declared[s["id"]] = {
                "reason": (f"the mutant writes {nxt} where the reference writes "
                           f"{item}; the two are byte-identical in all "
                           f"{len(blobs)} reference checkpoints ({state}), so no "
                           f"case in this corpus can distinguish them"),
                "evidence": a.report,
            }

    lines.append("")
    if bad:
        lines.append("REFUSED -- these pairs DO differ somewhere in the archive:")
        for mid, item, n in bad:
            lines.append(f"  {mid} {item}: {n} of {len(blobs)}")
    else:
        lines.append(f"All {len(declared)} pair(s) are byte-identical in every "
                     f"reference checkpoint. Each is a statement about the "
                     f"CORPUS: the same edit one field over, on rootMOOP, moves "
                     f"195 of 195 files (evidence/WriteRestartFile/"
                     f"chkp_redtest.txt, the `moop`/`zmqpit` pair).")

    Path(ROOT / a.report).parent.mkdir(parents=True, exist_ok=True)
    (ROOT / a.report).write_text("\n".join(lines) + "\n")
    (ROOT / a.out).write_text(json.dumps(declared, indent=2, sort_keys=True))
    print("\n".join(lines))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
