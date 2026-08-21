#!/usr/bin/env python3
"""How many cases does this unit's corpus WANT, and which rule asks for them.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        python3 evidence/WindSpeedEstimator/probe_corpus_count.py"

WHY IT EXISTS. `harness.generate.generate` was SIGKILLed on this unit -- exit
137 -- so the case COUNT it was building towards is not recoverable from any
artifact. Nothing that OOMs reports its own size.

HOW IT AVOIDS THE OOM. The memory is in the case DICTS, not in the count: every
case carries a value for all 518 C parameters, and the array-valued ones are
Python lists filled per case (`LocalVar%FP`'s history buffers alone are 1,024 doubles each). This
monkeypatches `harness.generate._case_impl` so that every array value collapses
to a single float AFTER the real fill has run -- so the rng stream, the rule
sequence and therefore the case COUNT are exactly what a real run would produce,
while the retained memory per case is ~200 bytes instead of ~188 KB (the figure unit #60 measured; this unit's cases are its own size).

WHAT IT DOES NOT MEASURE. The VALUES. A case whose arrays have been collapsed
is not a case anything can be scored against, and this probe never emits one.
Its output is two numbers -- the count and the per-rule attribution -- and
`probe_corpus_size.py` beside it measures the bytes.

IT MUST RUN ON A CLEAN TREE, like every generation: `literals_from`,
`predicate_knobs_from` and `unit_body` read the Fortran source, and on an
integrated tree that source is the wrapper.
"""
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
import harness.generate as G  # noqa: E402

_impl = G._case_impl


def tiny(sig, extents, rng, overrides=None, held=None, spread="full",
         corpus=(), shape="mixed"):
    c = _impl(sig, extents, rng, overrides, held, spread, corpus, shape)
    return {k: (0.0 if isinstance(v, list) else v) for k, v in c.items()}


G._case_impl = tiny

_real = vit_harness.generate


def wrapped(sig, *a, **kw):
    gen = _real(sig, *a, **kw)
    print(f"DONE cases={len(gen.cases)}", flush=True)
    for r in gen.coverage:
        print(str(r), flush=True)
    raise SystemExit(0)


vit_harness.generate = wrapped

sys.argv = ["vit_harness.py", "WindSpeedEstimator",
            "--root", str(ROOT),
            "--file", "rosco/controller/src/ControllerBlocks.f90",
            "--cpp", "translations/ControllerBlocks/windspeedestimator.cpp",
            "--module", "ControllerBlocks"]
raise SystemExit(vit_harness.main())
