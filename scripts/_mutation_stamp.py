#!/usr/bin/env python3
"""Stamp a mutation artifact with WHAT THE MUTANT WAS COMPARED AGAINST.

WHY THIS EXISTS. `mutation/CheckInputs.json` reported 4 of 173 killed, score
0.0231, on a corpus that passes 16,769 of 16,769 against real Fortran. The
number is not a fact about the translation: the run was taken on an INTEGRATED
tree, where

    checkinputs_f90  (the harness's Fortran side)
      -> CheckInputs          the wrapper `vit integrate` wrote
      -> checkinputs_c        -> vit_integration_shim.o
      -> CheckInputs(...)     the harness's OWN compiled copy -- the MUTANT

so every mutant was compared with itself and every behavioural difference
cancelled. The same mutants scored on a CLEAN tree give a number at the other
end of the range, and NOTHING IN THE ARTIFACT DISTINGUISHES THE TWO RUNS.
`harness/<U>.postintegration.json` carries a `measures:` field saying exactly
this for the differential harness; the mutation artifact had no equivalent.

WHAT IT READS, and why it is a measurement rather than a label. The question
"was the reference side the original Fortran" is answered by the object the
harness actually links: after `vit integrate` the Fortran body is a wrapper, so
`<Source>.f90.o` carries an UNDEFINED `<unit>_c` and the call leaves Fortran.
Clean, it carries the real body and no such symbol. That is read out of the
build tree with `nm`, not asserted.

    python3 scripts/_mutation_stamp.py mutation/CheckInputs.json \\
        --unit CheckInputs -f rosco/controller/src/ReadSetParameters.f90 \\
        --module ReadSetParameters --stem checkinputs

MUST BE RUN BEFORE `restore_integrated.sh`. It describes the build tree AS IT
STANDS; once the wrappers are back, the same script would report the integrated
answer about a run that was not taken there. It records the time it looked and
the git HEAD it looked at, so a stamp that drifted from its run is visible.

Exit 0 on a stamp of either verdict -- reporting `translation` is a correct
result about an invalid run, not a failure of this script. Exit 2 when it cannot
tell, which is the third answer and must not be rendered as either of the other
two (P6).
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONTAINER = "vit-dev"
WORKDIR = f"/workspace/{ROOT.name}"
OBJDIR = "rosco/controller/build/CMakeFiles/discon.dir/src"


def nm(path: str, *flags: str) -> str:
    r = subprocess.run(
        ["docker", "exec", CONTAINER, "bash", "-lc", f"nm {' '.join(flags)} {path}"],
        capture_output=True, text=True)
    return r.stdout


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("artifact")
    ap.add_argument("--unit", required=True)
    ap.add_argument("-f", "--file", required=True, help="the unit's Fortran source")
    ap.add_argument("--module", required=True)
    ap.add_argument("--stem", required=True)
    a = ap.parse_args()

    art = ROOT / a.artifact
    doc = json.loads(art.read_text())

    obj = f"{WORKDIR}/{OBJDIR}/{Path(a.file).name}.o"
    syms = nm(obj)
    if not syms.strip():
        print(f"_mutation_stamp: nm produced nothing for {obj}", file=sys.stderr)
        return 2

    unit_c = f"{a.unit.lower()}_c"
    # The POSITIVE CONTROL is in the same read: the object must mention the unit
    # at all. If it mentions neither the Fortran symbol nor the C entry point,
    # this is the wrong object and both verdicts below would be fabricated.
    fortran_sym = f"_MOD_{a.unit.lower()}"
    has_fortran = any(fortran_sym in ln for ln in syms.splitlines())
    calls_c = any(ln.split()[-1] == unit_c and " U " in ln
                  for ln in syms.splitlines() if ln.split())
    if not has_fortran and not calls_c:
        print(f"_mutation_stamp: {obj} defines no {fortran_sym} and references no "
              f"{unit_c}; this is not the object that carries the unit",
              file=sys.stderr)
        return 2

    mk = ROOT / "translations" / a.module / f"{a.stem}_test" / "Makefile"
    mk_text = mk.read_text() if mk.is_file() else ""
    shim_linked = "vit_integration_shim.o" in mk_text
    own_object_linked = f"/{a.stem}.cpp.o" in mk_text

    if calls_c:
        reference = "translation"
        measures = (
            f"NOTHING. The Fortran side of the comparison is the integration "
            f"WRAPPER: {Path(a.file).name}.o carries an undefined {unit_c}, so the "
            f"reference call leaves Fortran and re-enters the harness's own copy "
            f"of the mutant"
            + (" through vit_integration_shim.o" if shim_linked else "")
            + ". Both sides run the mutant and every behavioural difference "
              "cancels. This score is not a fact about the translation.")
    else:
        reference = "fortran"
        measures = (
            f"The translation against the ORIGINAL Fortran. {Path(a.file).name}.o "
            f"defines {fortran_sym} and references no {unit_c}, so the reference "
            f"side runs the Fortran body and nothing routes back into the mutant."
            + ("" if not shim_linked else
               f" vit_integration_shim.o is linked but unreferenced here: on a "
               f"clean tree nothing calls {unit_c}.")
            + ("" if not own_object_linked else
               f" WARNING: {a.stem}.cpp.o is still in LIBS beside the harness's "
               f"own copy of the translation."))

    doc["measures"] = measures
    doc["reference_side"] = reference
    doc["reference_side_basis"] = {
        "object": f"{OBJDIR}/{Path(a.file).name}.o",
        "defines_fortran_symbol": has_fortran,
        "undefined_reference_to_unit_c": calls_c,
        "shim_in_libs": shim_linked,
        "own_object_in_libs": own_object_linked,
    }
    doc["stamped_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                          capture_output=True, text=True)
    doc["stamped_at_head"] = head.stdout.strip() or None
    art.write_text(json.dumps(doc, indent=1) + "\n")
    print(f"{a.artifact}: reference_side={reference}")
    print(f"  {measures}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
