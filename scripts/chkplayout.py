#!/usr/bin/env python3
"""Derive the checkpoint file's byte layout from the REFERENCE, and census it.

TWO JOBS, AND THE SECOND IS WHY THE FIRST IS TRUSTWORTHY.

1. LAYOUT. Read the 307 `WRITE( Un, IOSTAT=ErrStat)` statements out of
   `WriteRestartFile`, resolve each item against its declaration in
   `ROSCO_Types.f90`, and lay them out end to end. An unformatted STREAM write
   emits the datum and nothing else -- no record marker, no padding -- so the
   layout is a plain concatenation. `--check <file>` asserts the predicted
   total against a checkpoint the reference actually wrote; if the two disagree
   the layout is wrong and every number derived from it is worthless.

   This is P7: the oracle is the original source. The field list is READ from
   the Fortran rather than transcribed, so it cannot drift from it.

2. CENSUS. For a directory of reference checkpoints, report per field whether
   its bytes ever VARY across the corpus, and whether they are ever non-zero.
   A field that is the same in all 117 files cannot discriminate a mutant that
   writes its neighbour instead -- so this is the table that prices an
   `unreachable` declaration before the sweep rather than after it.

     python3 scripts/chkplayout.py --check Examples/vit_chkp36100.RO.chkp
     python3 scripts/chkplayout.py --census evidence/WriteRestartFile/chkp/pre \\
         --out evidence/WriteRestartFile/layout.txt
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IO_F90 = ROOT / "rosco/controller/src/ROSCO_IO.f90"
TYPES_F90 = ROOT / "rosco/controller/src/ROSCO_Types.f90"

# gfortran on this build: INTEGER(IntKi) and INTEGER(C_INT) are 4 bytes, the
# default LOGICAL is 4, REAL(DbKi) is 8, COMPLEX(DbKi) is two of those, and a
# CHARACTER element is 1. Asserted by --check, not assumed.
SIZES = {"REAL(DbKi)": 8, "INTEGER(IntKi)": 4, "INTEGER(C_INT)": 4,
         "LOGICAL": 4, "COMPLEX(DbKi)": 16, "CHARACTER": 1}
SUBTYPE = {"WE": "WE", "FP": "FilterParameters", "piP": "piParams",
           "resP": "resParams", "rlP": "rlParams"}
DECL_RE = re.compile(
    r"\s*(REAL\(DbKi\)|INTEGER\(IntKi\)|INTEGER\(C_INT\)|LOGICAL|COMPLEX\(DbKi\)"
    r"|CHARACTER)(.*?)::\s*([A-Za-z_0-9]+)")


def write_items() -> list[str]:
    text = IO_F90.read_text().split("\n")
    start = next(i for i, l in enumerate(text)
                 if l.startswith("SUBROUTINE WriteRestartFile"))
    end = next(i for i, l in enumerate(text)
               if l.startswith("END SUBROUTINE WriteRestartFile"))
    items = []
    for line in text[start:end]:
        m = re.match(r"\s*WRITE\(\s*Un,\s*IOSTAT=ErrStat\)\s*(\S+)\s*$", line)
        if m:
            items.append(m.group(1))
    return items


def declaration(typename: str, field: str) -> tuple[str, str]:
    types = TYPES_F90.read_text()
    blk = re.search(r"TYPE, PUBLIC :: %s\b(.*?)END TYPE %s" % (typename, typename),
                    types, re.S).group(1)
    for line in blk.split("\n"):
        m = DECL_RE.match(line)
        if m and m.group(3).lower() == field.lower():
            return m.group(1), m.group(2).strip()
    raise KeyError(f"{typename}%{field}")


def layout(acc_infile_len: int) -> list[dict]:
    rows, off = [], 0
    for item in write_items():
        parts = item.split("%")
        last, idx = parts[-1], None
        m = re.match(r"([A-Za-z_0-9]+)\((\d+)\)$", last)
        if m:
            last, idx = m.group(1), int(m.group(2))
        tn = ("ObjectInstances" if parts[0] == "objInst"
              else SUBTYPE[parts[1]] if len(parts) == 3 else "LocalVariables")
        kind, dim = declaration(tn, last)
        n = 1
        if idx is None and dim:
            mm = re.search(r"DIMENSION\(([^)]*)\)", dim)
            if mm:
                dims = [d.strip() for d in mm.group(1).split(",")]
                if dims == [":"]:
                    n = acc_infile_len          # the one ALLOCATABLE item
                else:
                    n = 1
                    for x in dims:
                        n *= int(x)
        size = SIZES[kind] * n
        rows.append({"item": item, "kind": kind, "elems": n,
                     "offset": off, "size": size})
        off += size
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", default=None, help="a reference .RO.chkp")
    ap.add_argument("--census", default=None, help="a directory of .RO.chkp")
    ap.add_argument("--acc-infile-len", type=int, default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    ref = None
    if args.check:
        ref = Path(args.check).read_bytes()
    files = []
    if args.census:
        files = sorted(Path(args.census).glob("*.RO.chkp"))
        if not files:
            print("census: no *.RO.chkp -- refusing", file=sys.stderr)
            return 2
        if ref is None:
            ref = files[0].read_bytes()

    if ref is None:
        print("nothing to do: pass --check or --census", file=sys.stderr)
        return 2

    # ACC_INFILE's extent is ACC_INFILE_SIZE, and that integer is IN the file,
    # immediately before it. Read it rather than guessing: the length is the
    # parameter file's path and differs per scenario.
    probe = layout(0)
    at = next(r["offset"] for r in probe if r["item"].endswith("ACC_INFILE_SIZE"))
    n_acc = args.acc_infile_len or int.from_bytes(ref[at:at + 4], "little", signed=True)
    rows = layout(n_acc)
    total = rows[-1]["offset"] + rows[-1]["size"]

    lines = [f"WriteRestartFile checkpoint layout, derived from "
             f"{IO_F90.relative_to(ROOT)}",
             f"  items (WRITE statements): {len(rows)}",
             f"  ACC_INFILE extent read from the file: {n_acc}",
             f"  predicted total bytes: {total}"]
    ok = True
    if args.check:
        lines.append(f"  {Path(args.check).name}: {len(ref)} bytes")
        ok = (len(ref) == total)
        lines.append(f"  LAYOUT CHECK: {'PASS' if ok else 'FAIL'}")

    if files:
        blobs = [f.read_bytes() for f in files]
        lines.append("")
        lines.append(f"CENSUS over {len(blobs)} reference checkpoint(s) in "
                     f"{Path(args.census).name}")
        # A file whose length differs cannot be laid out with this table; the
        # ACC_INFILE extent differs per scenario, so group by length.
        lengths = sorted({len(b) for b in blobs})
        lines.append(f"  distinct file lengths: {lengths}")
        usable = [b for b in blobs if len(b) == total]
        lines.append(f"  laid out with this table: {len(usable)} of {len(blobs)}"
                     f"  (the rest have a different ACC_INFILE extent)")
        varying = constant_zero = constant_nonzero = 0
        detail = []
        for r in rows:
            a, b = r["offset"], r["offset"] + r["size"]
            vals = {blob[a:b] for blob in usable}
            zero = all(v == bytes(r["size"]) for v in vals)
            if len(vals) > 1:
                varying += 1
                state = "VARIES"
            elif zero:
                constant_zero += 1
                state = "const-zero"
            else:
                constant_nonzero += 1
                state = "const"
            detail.append(f"  {r['offset']:>7} {r['size']:>6} {state:<10} {r['item']}")
        lines.append(f"  VARIES across the corpus : {varying}")
        lines.append(f"  constant and NON-zero    : {constant_nonzero}")
        lines.append(f"  constant and ZERO        : {constant_zero}"
                     f"   <- no mutant that only moves one of these can die here")
        lines.append("")
        lines.append("  offset   size state      item")
        lines += detail

    text = "\n".join(lines) + "\n"
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text)
    print(text)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
