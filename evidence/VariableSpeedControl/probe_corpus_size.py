#!/usr/bin/env python3
"""How many BYTES does one case of this unit's corpus cost, measured.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        ulimit -v 5500000 && \
        python3 evidence/VariableSpeedControl/probe_corpus_size.py"

`ulimit -v` IS PART OF THE PROBE, not a convenience. Without it the kernel's OOM
killer sends SIGKILL, which produces no traceback, no count and no partial
output -- exactly the state that made the first failure uninformative. With an
address-space cap Python raises `MemoryError` instead, so the run reports how
far it got and where.

It counts `_case_impl` calls and prints `ru_maxrss` every 2,000 of them, so the
per-case cost is a SLOPE over many points rather than a single ratio at the end.
Both are recorded in `corpus_wall.txt`.

IT MUST RUN ON A CLEAN TREE, like every generation.
"""
import resource
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

n = [0]
_impl = G._case_impl


def counting(*a, **kw):
    c = _impl(*a, **kw)
    n[0] += 1
    if n[0] % 2000 == 0:
        rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        print(f"cases {n[0]:8d}  maxrss {rss:8.0f} MB", flush=True)
    return c


G._case_impl = counting

_real = vit_harness.generate


def wrapped(sig, *a, **kw):
    try:
        gen = _real(sig, *a, **kw)
    except MemoryError:
        print(f"MemoryError after {n[0]} _case_impl calls", flush=True)
        raise SystemExit(9)
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    print(f"DONE cases={len(gen.cases)} calls={n[0]} maxrss={rss:.0f} MB", flush=True)
    raise SystemExit(0)


vit_harness.generate = wrapped

sys.argv = ["vit_harness.py", "VariableSpeedControl",
            "--root", str(ROOT),
            "--file", "rosco/controller/src/Controllers.f90",
            "--cpp", "translations/Controllers/variablespeedcontrol.cpp",
            "--module", "Controllers"]
raise SystemExit(vit_harness.main())
