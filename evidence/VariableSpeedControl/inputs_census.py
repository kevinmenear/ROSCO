#!/usr/bin/env python3
"""COUNT this unit's corpus without emitting it.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        python3 evidence/VariableSpeedControl/inputs_census.py"

THIS PROBE DOES NOT COMPLETE ON THIS UNIT, AND IT IS COMMITTED SAYING SO. It
keeps the FULL cases so it can read their values, so it dies exactly where the
harness died -- SIGKILL at 22 seconds, `MemoryError` at 28,296 cases under
`ulimit -v`. `corpus_wall.txt` has the numbers and the two probes beside this
one are the ones that produced them: `probe_corpus_count.py` collapses the array
values after the fill (so the COUNT is exact and the bytes are not) and
`probe_corpus_size.py` measures the bytes. This file is kept because it is the
census unit #59's finding calls for -- "run it for any unit whose statements
DIVIDE" -- and it is the FIRST thing to run when a corpus for this unit exists.
No `inputs_census.json` is committed, because none was ever written.

WHY THIS EXISTS, AND IT IS NOT THE REASON UNIT #59's CENSUS EXISTED. The first
generating run of this unit's harness was SIGKILLed after 68 seconds -- exit
137, the shape `harness/emit.py`'s memory ceiling has (STATUS.md, unit #53: it
died writing 1.0 GB between 84,754 and 95,310 cases). So before any pin can be
argued for on the grounds of what the corpus COMPUTES, the corpus has to be
small enough to exist, and the first number needed is how many cases the
generator chose.

`harness.generate.generate` is the whole of the corpus decision; `emit` is what
writes the case file and builds. This monkeypatches the name `vit_harness`
imported `generate` under, reads `gen.cases`, and exits before `emit` -- so the
probe costs one generation and no build. Copied from
`evidence/IPC/probe_fmin_site.py` (P4); the statistics are this unit's.

WHAT IT COUNTS. Unit #59's finding is that a green harness on a corpus that
computes NaN looks exactly like a green harness, and it named the rule: run this
for any unit whose statements DIVIDE. This one divides twice on its second
statement --

    VS_ConstPwr_GenTq = (VS_RtPwr/(VS_GenEff/100.0))/GenSpeedF * PRC_R_Torque

-- by `VS_GenEff` and by `GenSpeedF`, and `VS_ConstPwr_GenTq` then feeds the
`min` that chooses `VS_MaxTq`, which is the upper saturation of the TSR PI
controller. A corpus in which `GenSpeedF` is 0 in most cases would put an
infinity or a NaN into the saturation bound of the unit's main arm.

It also partitions the corpus by the reference's own arms, so a pin can be
argued against a measured arm population rather than against an expectation.

IT MUST RUN ON A CLEAN TREE. `literals_from`, `predicate_knobs_from` and
`unit_body` all read the Fortran source, and on an integrated tree that source
is the wrapper. Run it between `scripts/reset_to_clean.sh` and
`scripts/restore_integrated.sh`.
"""
from __future__ import annotations

import collections
import json
import math
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

OUT = ROOT / "evidence" / "VariableSpeedControl" / "inputs_census.json"

# The ten predicate knobs R7 found, plus the reals the unit divides by and the
# reals that decide the saturation pairs.
KNOBS = [
    "CntrPar_VS_ControlMode", "CntrPar_VS_FBP", "CntrPar_VS_ConstPower",
    "CntrPar_SD_Method", "CntrPar_OL_Mode", "CntrPar_Ind_GenTq",
    "LocalVar_VS_State", "LocalVar_SD_Trigger", "LocalVar_iStatus",
    "ErrVar_aviFAIL",
]
DIVISORS = ["LocalVar_GenSpeedF", "CntrPar_VS_GenEff"]


def _num(v):
    if v is None or isinstance(v, (list, tuple, dict, str, bytes)):
        return None
    return float(v)


def _classify(v):
    if v is None:
        return "absent"
    if math.isnan(v):
        return "nan"
    if v == 0.0:
        return "zero"
    if math.isinf(v):
        return "inf"
    return "finite_nonzero"


def _arms(cases):
    """Partition by the reference's own control flow, as written."""
    a = collections.Counter()
    for c in cases:
        cm = c.get("CntrPar_VS_ControlMode")
        fbp = c.get("CntrPar_VS_FBP")
        cp = c.get("CntrPar_VS_ConstPower")
        st = c.get("LocalVar_VS_State")
        sdt = c.get("LocalVar_SD_Trigger")
        sdm = c.get("CntrPar_SD_Method")
        olm = c.get("CntrPar_OL_Mode")
        ind = c.get("CntrPar_Ind_GenTq")
        ist = c.get("LocalVar_iStatus")

        # VS_MaxTq
        if fbp is not None and int(fbp) == 0:
            a["maxtq/variable_pitch/" + ("constpwr_min" if cp is not None and int(cp) == 1
                                         else "rttq")] += 1
        else:
            a["maxtq/constant_pitch"] += 1

        # the three control laws
        if cm is not None and int(cm) in (2, 3, 4):
            a["law/tsr_pi"] += 1
            if fbp is not None and int(fbp) == 1:
                a["law/tsr_pi/overspeed_min"] += 1
        elif cm is not None and int(cm) == 1:
            a["law/komega"] += 1
            s = int(st) if st is not None else None
            a["law/komega/state_%s" % s] += 1
            if s == 6 and fbp is not None:
                f = int(fbp)
                if f == 1:
                    a["law/komega/state_6/overspeed"] += 1
                elif f in (2, 3):
                    a["law/komega/state_6/ref_tracking"] += 1
                else:
                    a["law/komega/state_6/fallthrough"] += 1
        else:
            a["law/zero"] += 1

        # shutdown
        if sdt is not None and int(sdt) == 0:
            a["sd/passthrough"] += 1
        else:
            if sdm is not None and int(sdm) in (1, 2):
                a["sd/ramp"] += 1
            else:
                a["sd/carry_gentq_sd"] += 1

        # open loop
        if olm is not None and int(olm) > 0 and ind is not None and int(ind) > 0:
            a["ol/entered"] += 1
            if olm is not None and int(olm) == 2:
                a["ol/azimuth"] += 1
                if ist is not None and int(ist) == 0:
                    a["ol/azimuth/istatus0"] += 1

        # the error tail
        av = c.get("ErrVar_aviFAIL")
        if av is not None and int(av) < 0:
            a["errmsg/prefixed"] += 1
    return dict(sorted(a.items()))


_real = vit_harness.generate


def _wrapped(signature, *args, **kw):
    gen = _real(signature, *args, **kw)
    cases = gen.cases
    doc = {
        "unit": "VariableSpeedControl",
        "cases": len(cases),
        "loop_rev": vit_harness.loop_rev(),
        "gen_rev": vit_harness.gen_rev(),
        "knob_distribution": {
            k: {str(kk): vv for kk, vv in
                sorted(collections.Counter(
                    (int(c[k]) if c.get(k) is not None else None) for c in cases
                ).items(), key=lambda kv: (kv[0] is None, kv[0]))}
            for k in KNOBS
        },
        "divisors": {
            d: {str(kk): vv for kk, vv in
                collections.Counter(_classify(_num(c.get(d))) for c in cases).items()}
            for d in DIVISORS
        },
        "arms": _arms(cases),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(doc, indent=1) + "\n")
    print(json.dumps(doc, indent=1))
    print(f"\nwrote {OUT}", file=sys.stderr)
    raise SystemExit(0)


vit_harness.generate = _wrapped

sys.argv = ["vit_harness.py", "VariableSpeedControl",
            "--root", str(ROOT),
            "--file", "rosco/controller/src/Controllers.f90",
            "--cpp", "translations/Controllers/variablespeedcontrol.cpp",
            "--module", "Controllers"]
raise SystemExit(vit_harness.main())
