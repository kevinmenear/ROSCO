#!/usr/bin/env python3
"""How many cases the differential harness plans for CheckInputs, and why the
generator was SIGKILLed building them.

Runs the loop's own planner and generator with ONE substitution: `_case` --
which materialises a full 467-parameter state -- is replaced by a one-word
placeholder. Nothing else is stubbed, so the case COUNT and the rule coverage
lines are the generator's own; only the bytes per case are not.

    python3 evidence/CheckInputs/harness_case_arithmetic.py
"""
import resource
import sys
from pathlib import Path

# 5 GiB: turn the container OOM kill (SIGKILL, no traceback) into a MemoryError
# with a stack, so the rule that allocates can be NAMED rather than guessed.
resource.setrlimit(resource.RLIMIT_AS, (5 << 30, 5 << 30))

ROOT = Path("/workspace/ROSCO-r2")
sys.path.insert(0, "/workspace/translation-loop")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, "/workspace/translation-loop/scripts")

import yaml
import harness.generate as G
from harness import statevary
from harness.vitbridge import map_signature
from harness.generate import generate
from vit_harness import (unit_body, load_ranges, split_no_oracle, split_same_as,
                         literals_from, char_literals_from, order_arrays_from,
                         predicate_knobs_from, relational_pairs_from,
                         reduction_pairs_from, divisibility_pairs_from,
                         named_constants_from, order_step_magnitudes,
                         file_params_from, io_list_bound_from,
                         io_unit_params_from, _ORDER_STEP_CAP)
from vit.derived_type_parser import apply_type_strategies, build_derived_type_registry
from vit.fortran_parser import parse_signature_from_source

UNIT = "CheckInputs"
SRC = ROOT / ".vit/ReadSetParameters.clean.f90"
CPP = ROOT / "translations/ReadSetParameters/checkinputs.cpp"

cfg = yaml.safe_load((ROOT / "vit.yaml").read_text())
aliases = (cfg.get("fortran") or {}).get("kind_aliases") or {}
strategies = {k: (v.get("strategy") if isinstance(v, dict) else v)
              for k, v in (cfg.get("types") or {}).items()}
strategies = {k: v for k, v in strategies.items() if v}

sig = parse_signature_from_source(SRC.read_text(), UNIT)
registry = build_derived_type_registry(sig, str(ROOT / "rosco/controller/src"))
apply_type_strategies(registry, strategies)
assumed = (cfg.get("kgen") or {}).get("assumed_size_arrays") or {}
mapped = map_signature(sig, aliases, registry, cpp_source=CPP.read_text(),
                       assumed_size=assumed, fortran_source=unit_body(SRC, UNIT))

stated, _ = split_no_oracle(load_ranges(ROOT, UNIT, None))
stated, tied = split_same_as(stated)
signature = statevary.constrain(mapped.signature, stated)
st = statevary.plan(signature, SRC.read_text(), UNIT, [a.name for a in sig.args])

body = unit_body(SRC, UNIT)
names = {p.name for p in signature.inputs}
consts = named_constants_from(SRC)
knobs = [k for k in predicate_knobs_from(body, names, consts) if k[0] not in st.held]
rel = [r for r in relational_pairs_from(body, names)
       if r[0] not in st.held and r[2] not in st.held]
red = [r for r in reduction_pairs_from(body, names)
       if r[0] not in st.held and r[1] not in st.held]
div = [d for d in divisibility_pairs_from(body, names)
       if d[0] not in st.held and d[2] not in st.held]
ordered = order_arrays_from(body, {p.name for p in signature.inputs if p.dims})
steps = order_step_magnitudes(body, consts)
fparams = [f for f in file_params_from(body, names) if f not in st.held]
bound = io_list_bound_from(body, names)
units = [u for u in io_unit_params_from(body, names) if u not in st.held]

print(f"varied parameters      {len(signature.inputs) - len(st.held)}")
print(f"HELD                   {len(st.held)}")
print(f"predicate knobs        {len(knobs)}")
kv = 1
for _n, _i, v in knobs:
    kv *= max(1, len(v))
print(f"full knob cross product{kv:>40}   (bound is {G._KNOB_CASE_LIMIT})")

# The one substitution. `_case` is a CLOSURE inside generate() -- patching the
# module global of that name does nothing, which cost one run to learn. What it
# delegates to, `_case_impl`, IS a module global, and it is what materialises
# the 467-parameter state that holds the memory.
N = [0]


def _stub(*a, **k):
    N[0] += 1
    return "<case>"


G._case_impl = _stub

# Sweep the bound: `python3 ... 4096 1024 256` prints the case count at each.
if len(sys.argv) > 1:
    for _lim in (int(a) for a in sys.argv[1:]):
        G._KNOB_PAIR_LIMIT, N[0] = _lim, 0
        try:
            generate(signature, seed=0, n_random=24, literals=literals_from(SRC),
                     plan=st, char_literals=char_literals_from(SRC, UNIT),
                     order_arrays=sorted(ordered),
                     order_steps=tuple(steps[:_ORDER_STEP_CAP]), knobs=knobs,
                     rel_pairs=rel, div_pairs=div, red_pairs=red,
                     file_inputs=[(f, bound) for f in fparams],
                     io_units=tuple(units), tied_extents=dict(tied),
                     disabled=frozenset())
        except Exception:
            pass
        print(f"_KNOB_PAIR_LIMIT {_lim:>6}  ->  {N[0]:>8} case(s)")
    raise SystemExit(0)

try:
    generate(signature, seed=0, n_random=24, literals=literals_from(SRC),
             plan=st, char_literals=char_literals_from(SRC, UNIT),
             order_arrays=sorted(ordered), order_steps=tuple(steps[:_ORDER_STEP_CAP]),
             knobs=knobs, rel_pairs=rel, div_pairs=div, red_pairs=red,
             file_inputs=[(f, bound) for f in fparams], io_units=tuple(units),
             tied_extents=dict(tied), disabled=frozenset())
except Exception as e:
    # The post-generation rules index into a case; the stub is a string. Every
    # case-BUILDING rule has already run by then, which is what is being counted.
    print(f"(stopped in a post-generation rule: {type(e).__name__})")
print(f"\nCASES PLANNED          {N[0]:>40}")
