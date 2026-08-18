#!/usr/bin/env python3
"""COUNT what this unit's corpus does at the `IPC_SatMode` / `std::fmin` site.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        python3 evidence/IPC/probe_fmin_site.py"

WHY THIS EXISTS. Six mutants of `translations/Controllers/ipc.cpp` survived a
63,888-case green, and two of them -- `drop_call b36f5d50` and `452847c1` --
were then KILLED by the gate on 159,758 of 5,252,000 values. A mutant one
instrument kills is not equivalent, so the differential harness has a blind
spot, and the runbook's rule for that (unit #50, "An equivalence argument is
executed by COUNTING the cases that reach the site") is to count rather than to
argue.

    LocalVar%IPC_IntSat = min(CntrPar%IPC_IntSat, LocalVar%BlPitchCMeas - <a PC_MinPit>)

`drop_call` rewrites that to `CntrPar%IPC_IntSat`, so it differs from the
reference on exactly the cases where the SECOND argument WINS. This probe
answers, per arm, over the whole corpus:

  * how many cases reach the arm at all              (the positive control:
                                                      a site the corpus never
                                                      reaches makes every mutant
                                                      of it unkillable for a
                                                      duller reason)
  * how many have second < first                     (kills `drop_call`)
  * how many have second == first but DIFFERENT BITS (kills `swap_call_args`:
                                                      glibc's fmin returns its
                                                      FIRST argument when the
                                                      two compare equal, so
                                                      fmin(+0.0, -0.0) is +0.0
                                                      and the swap is -0.0)
  * how many have arm-2's and arm-3's second argument DIFFERENT, and of those
    how many would change the stored answer          (kills the two
                                                      `const_tweak` mutants of
                                                      the arm selector, which is
                                                      transcription decision 3)

IT GENERATES BUT DOES NOT EMIT. `harness.generate.generate` is the whole of the
corpus decision; `emit` is what writes 656 MB and builds. This monkeypatches the
name `vit_harness` imported `generate` under, records the statistics off
`gen.cases`, and exits before `emit` is reached -- so the probe costs one
generation and no build.

IT MUST RUN ON A CLEAN TREE. `literals_from`, `predicate_knobs_from` and
`unit_body` all read the Fortran source, and on an integrated tree that source
is the wrapper. Run it between `scripts/reset_to_clean.sh --no-build` and
`scripts/restore_integrated.sh`.
"""
from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path

ROOT = Path("/workspace/ROSCO-r2")
if not ROOT.is_dir():
    ROOT = Path(__file__).resolve().parents[2]
LOOP = Path("/workspace/translation-loop")
if not LOOP.is_dir():
    LOOP = ROOT.parent / "translation-loop"

sys.path.insert(0, str(LOOP / "scripts"))
sys.path.insert(0, str(LOOP))

import vit_harness  # noqa: E402

OUT = ROOT / "evidence" / "IPC" / "probe_fmin_site.json"

SATMODE = "CntrPar_IPC_SatMode"
FIRST = "CntrPar_IPC_IntSat"
BLPITCH = "LocalVar_BlPitchCMeas"
ARM2 = "CntrPar_PC_MinPit"
ARM3 = "LocalVar_PC_MinPit"


def bits(x: float) -> int:
    return struct.unpack("<Q", struct.pack("<d", x))[0]


def _num(v):
    """One scalar out of a case value, or None when the case does not name it."""
    if v is None or isinstance(v, (list, tuple, dict, str, bytes)):
        return None
    return float(v)


def _stats(cases):
    arms = {2: 0, 3: 0, "else": 0, "missing": 0}
    per_arm = {}
    for arm, minpit in ((2, ARM2), (3, ARM3)):
        per_arm[arm] = {"reached": 0, "second_wins": 0, "equal_same_bits": 0,
                        "equal_diff_bits": 0, "first_wins": 0, "nan": 0,
                        "witness_second_wins": None, "witness_equal_diff": None}
    # The arm-SELECTOR question: does sending an arm-2 case down arm 3 (or the
    # reverse) change the value stored? That is the const_tweak pair.
    selector = {"both_reachable": 0, "minpit_differs": 0, "answer_differs": 0,
                "witness": None}

    for c in cases:
        sm = c.get(SATMODE)
        if sm is None:
            arms["missing"] += 1
            continue
        sm = int(sm)
        arms[sm if sm in (2, 3) else "else"] += 1

        a = _num(c.get(FIRST))
        b = _num(c.get(BLPITCH))
        m2 = _num(c.get(ARM2))
        m3 = _num(c.get(ARM3))
        if a is None or b is None:
            continue

        answers = {}
        for arm, m in ((2, m2), (3, m3)):
            if m is None:
                continue
            second = b - m
            answers[arm] = second
            if sm != arm:
                continue
            s = per_arm[arm]
            s["reached"] += 1
            if math.isnan(a) or math.isnan(second):
                s["nan"] += 1
            elif second < a:
                s["second_wins"] += 1
                if s["witness_second_wins"] is None:
                    s["witness_second_wins"] = {
                        "IPC_IntSat": a, "BlPitchCMeas": b, "PC_MinPit": m,
                        "second": second}
            elif second == a:
                if bits(second) == bits(a):
                    s["equal_same_bits"] += 1
                else:
                    s["equal_diff_bits"] += 1
                    if s["witness_equal_diff"] is None:
                        s["witness_equal_diff"] = {
                            "IPC_IntSat": a, "BlPitchCMeas": b, "PC_MinPit": m,
                            "second": second,
                            "bits_first": hex(bits(a)),
                            "bits_second": hex(bits(second))}
            else:
                s["first_wins"] += 1

        if 2 in answers and 3 in answers and sm in (2, 3):
            selector["both_reachable"] += 1
            if bits(answers[2]) != bits(answers[3]):
                selector["minpit_differs"] += 1
                r2 = min(a, answers[2]) if not (math.isnan(a) or math.isnan(answers[2])) else float("nan")
                r3 = min(a, answers[3]) if not (math.isnan(a) or math.isnan(answers[3])) else float("nan")
                # fmin returns the FIRST argument when the two compare equal.
                f2 = a if not (answers[2] < a) else answers[2]
                f3 = a if not (answers[3] < a) else answers[3]
                if bits(f2) != bits(f3):
                    selector["answer_differs"] += 1
                    if selector["witness"] is None:
                        selector["witness"] = {
                            "IPC_SatMode": sm, "IPC_IntSat": a,
                            "BlPitchCMeas": b, "CntrPar_PC_MinPit": m2,
                            "LocalVar_PC_MinPit": m3,
                            "arm2_answer": f2, "arm3_answer": f3,
                            "unused_r2r3": [r2, r3]}
    return arms, per_arm, selector


_real = vit_harness.generate


def _wrapped(signature, *a, **kw):
    gen = _real(signature, *a, **kw)
    arms, per_arm, selector = _stats(gen.cases)
    doc = {
        "unit": "IPC",
        "cases": len(gen.cases),
        "loop_rev": vit_harness.loop_rev(),
        "gen_rev": vit_harness.gen_rev(ROOT),
        "site": "translations/Controllers/ipc.cpp:380-388 -- the IPC_SatMode "
                "split and its two std::fmin saturations",
        "satmode_distribution": {str(k): v for k, v in arms.items()},
        "per_arm": {str(k): v for k, v in per_arm.items()},
        "arm_selector": selector,
        "inputs_read": {"satmode": SATMODE, "first": FIRST,
                        "blpitch": BLPITCH, "arm2_minpit": ARM2,
                        "arm3_minpit": ARM3},
        "params_present": sorted(
            n for n in (SATMODE, FIRST, BLPITCH, ARM2, ARM3)
            if any(n in c for c in gen.cases[:1] or [{}])),
    }
    OUT.write_text(json.dumps(doc, indent=1) + "\n")
    print(json.dumps(doc, indent=1))
    print(f"\nwrote {OUT}", file=sys.stderr)
    raise SystemExit(0)


vit_harness.generate = _wrapped

sys.argv = ["vit_harness.py", "IPC",
            "--root", str(ROOT),
            "--file", "rosco/controller/src/Controllers.f90",
            "--cpp", "translations/Controllers/ipc.cpp",
            "--module", "Controllers"]
raise SystemExit(vit_harness.main())
