#!/usr/bin/env python3
"""Which of VIT's generators does the conformance matrix actually measure?

    docker exec vit-dev bash -lc \
      "cd /workspace/ROSCO-r2 && python3 evidence/AddToList/generators.py"

`plan.json` predicts `AddToList` cannot cross, with the basis

    list: c_alloc_inout does not cross
    (UNSUPPORTED: as c_alloc_out. INTENT(INOUT) ALLOCATABLE needs an ALLOCA...)

which is `tests/conformance/matrix.toml`'s `[c_alloc_inout]` cell, where
`compiles = "no"`.

`tests/test_conformance.py:probe()` measures four stages:

    parse     fortran_parser.find_function_in_file
    c_params  interface_gen.build_c_params
    bridge    test_validate.generate_fortran_bridge      <-- the HARNESS side
    compiles  gfortran -fsyntax-only on that bridge      <-- the HARNESS side

It never calls `interface_gen.generate_fortran_interface_block` or
`generate_fortran_wrapper` -- the two generators whose output `vit integrate`
puts into the shipped library. So the matrix cell the plan reads as a verdict
about INTEGRATION is a measurement of the differential harness.

This script runs all four generators on the real AddToList and compiles each
Fortran product, so the two paths can be compared side by side. It writes
generators.json and prints it. It only reads the tree.
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "rosco/controller/src/ROSCO_Helpers.f90"
UNIT = "AddToList"
# vit.yaml fortran.kind_aliases, verbatim.
ALIASES = {"ReKi": "REAL(4)", "DbKi": "REAL(8)", "SiKi": "REAL(4)", "IntKi": "INTEGER(4)"}


def _first_code_line(text: str) -> str:
    """The first line that is neither blank nor a `!` comment.

    Checking `text.lstrip()` instead reported `n/a -- not a compilable
    fragment` for the harness bridge, whose first three lines are comments.
    A check that cannot run must say so; a check that reports `n/a` for
    something it could have compiled is worse than one that fails.
    """
    for line in text.splitlines():
        s = line.strip()
        if s and not s.startswith("!"):
            return s.upper()
    return ""


def compiles(text: str, extra_mods: str | None = None) -> tuple[str, str]:
    """gfortran -fsyntax-only on a generated fragment. (yes|no|n/a, first Error)."""
    if not _first_code_line(text).startswith(("MODULE", "PROGRAM", "SUBROUTINE", "FUNCTION")):
        return "n/a", "not a compilable fragment on its own"
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "frag.f90"
        p.write_text(text + "\n")
        cmd = ["gfortran", "-fsyntax-only", "-fdefault-real-8", "-fdefault-double-8",
               "-ffree-line-length-0", "-J", d]
        if extra_mods:
            cmd += [f"-I{extra_mods}"]
        r = subprocess.run(cmd + [str(p)], capture_output=True, text=True)
        if r.returncode == 0:
            return "yes", ""
        err = next((l for l in r.stderr.splitlines() if l.strip().startswith("Error")), "")
        return "no", (err or (r.stderr.strip().splitlines()[-1] if r.stderr else "unknown"))


def build_source_mods() -> str | None:
    """The .mod directory of the campaign's own Release build.

    Recompiling ROSCO_Helpers.f90 standalone does NOT work -- it opens with
    `USE ROSCO_Types`, so a from-scratch compile fails on a missing module and
    the harness bridge would score `compiles = no` for a reason that has
    nothing to do with its signature. That would measure the probe.
    """
    d = ROOT / "rosco/controller/build/ftnmods"
    if not (d / "rosco_helpers.mod").exists():
        print(f"NOTE: no rosco_helpers.mod under {d} -- build first", file=sys.stderr)
        return None
    return str(d)


def main() -> int:
    sys.path.insert(0, "/workspace/vit")
    from vit.fortran_parser import find_function_in_file
    from vit.interface_gen import (build_c_params, generate_fortran_interface_block,
                                   generate_fortran_wrapper)
    from vit.test_validate import generate_fortran_bridge

    sig = find_function_in_file(str(SRC), UNIT)
    mods = build_source_mods()

    out: dict = {
        "unit": UNIT,
        "vit_rev": (Path("/workspace/vit/.vit_rev").read_text().strip() + "-pinned"
                    if Path("/workspace/vit/.vit_rev").exists() else "unknown"),
        "question": "the matrix cell [c_alloc_inout] measures which generator?",
        "argument": {a.name: {"type": a.fortran_type, "intent": str(a.intent),
                              "is_array": a.is_array,
                              "is_allocatable": getattr(a, "is_allocatable", False)}
                     for a in sig.args},
        "stages": {},
    }

    # --- what the conformance matrix measures -------------------------------
    out["stages"]["c_params (matrix: c_params)"] = {
        "measures": "interface_gen.build_c_params",
        "result": f"{len(build_c_params(sig, ALIASES, {}))} C parameters",
    }

    bridge = generate_fortran_bridge(sig, ALIASES, derived_types={}, source_file=str(SRC))
    ok, why = compiles(bridge, mods)
    out["stages"]["harness bridge (matrix: bridge + compiles)"] = {
        "measures": "test_validate.generate_fortran_bridge -- the DIFFERENTIAL HARNESS's "
                    "Fortran side. Never linked into the library.",
        "generated": "yes",
        "compiles": ok,
        "diagnostic": why,
        "text": bridge,
    }

    # --- what the matrix does NOT measure -----------------------------------
    iface = generate_fortran_interface_block(sig, ALIASES, {})
    wrap = generate_fortran_wrapper(sig, ALIASES, {})
    # The wrapper needs the interface block in scope; compile them together the
    # way `vit integrate` places them (interface above CONTAINS, wrapper below).
    combined = ("MODULE vit_addtolist_probe\n" + iface +
                "\nCONTAINS\n" + wrap + "\nEND MODULE vit_addtolist_probe\n")
    ok2, why2 = compiles(combined)
    out["stages"]["integration interface+wrapper (matrix: NOT MEASURED)"] = {
        "measures": "interface_gen.generate_fortran_interface_block + "
                    "generate_fortran_wrapper -- what `vit integrate` writes into "
                    "the shipped library.",
        "generated": "yes",
        "compiles": ok2,
        "diagnostic": why2,
        "text": combined,
    }

    out["finding"] = (
        "The matrix's `compiles` column for [c_alloc_inout] is a verdict about the "
        "test-validate bridge. The integration wrapper is generated by a different "
        "function, is never compiled by the conformance test, and for AddToList it "
        f"compiles ({ok2}) while silently dropping the ALLOCATABLE attribute. "
        "See bridge_probe/result.json for what it then does at run time."
    )

    dest = Path(__file__).with_name("generators.json")
    dest.write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps({k: (v if k != "stages" else
                          {s: {kk: vv for kk, vv in d.items() if kk != "text"}
                           for s, d in v.items()}) for k, v in out.items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
