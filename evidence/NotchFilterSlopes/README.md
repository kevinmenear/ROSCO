# NotchFilterSlopes — unit #14

`REAL(DbKi) FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, Moving, InitialValue)`
in `rosco/controller/src/Filters.f90`. An inverted notch with descending slopes:
saturate the corner frequency at zero, reset four state slots on the first call,
recompute five coefficients whenever the frequency is allowed to MOVE, then four
products accumulated in source order, then shift the two input and two output
histories and advance the instance counter.

**Five layers ran and all five are alive** — the third unit in this campaign of
which that is true, after `LPFilter` and `NotchFilter`.

| layer | result | red test |
|---|---|---|
| kernel replay | 62/62 cases, 14,508 field rows, ALL `IDENTICAL` | zero stub 0/62, moves 661 rows; VIT's own `InputSignal * 1.00001` DISCRIMINATING |
| differential harness (clean Fortran) | 4,508 checked, 0 failed, 0 inadmissible | the unit as a no-op fails **4,508 of 4,508** |
| mutation score | **84 of 84** behavioural killed, **1.000**, **0 declared equivalent** | 3 excluded as `killed (no compile)` |
| post-integration harness (the wrapper) | 4,508 checked, 0 failed | `CornerFreq`/`Damp` swapped at the bridge call: **2,192 of 4,508** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 128,918 of 5,252,000**, 0 after revert |

## The gate can see this unit, and the number is its own

`gate/NotchFilterSlopes.redtest.json` — scaling the returned value by 1.000001
moves **128,918 of 5,252,000** across 15 channels, and the revert returns 0
(`replacements: 1`, `revert_verified: true`). A red test that goes red needs no
same-build control: it has just demonstrated the chain a control would be asked
to demonstrate (unit #9's entry exists for the opposite case).

Unit #12's check applied — compare the moved count against every committed
redtest artifact before writing "the gate sees this unit":

```
ColemanTransform 124,353   ColemanTransformInverse 389,644   GetWords 1,857,893
LPFilter 1,592,059   NonDecreasing 1,857,893   NotchFilter 551,278
__gate__ 1,604,573         NotchFilterSlopes 128,918  <- matches none of them
```

So this is a **sixth gate-visible unit** and not another sighting of unit #12's
shared refuse-to-start. `LocalVar%RootMOOPF` is the filtered blade-root moment
that individual pitch control and Coleman-transform flap control are both built
on; the moved channels are blade pitch, generator speed/torque/power and flap
angle.

## What each layer could NOT see

* **Three of the six scenarios that run the unit run it on zeros.**
  `scenario_arguments.txt` is the measurement: `Examples/vit_sim.py` injects a
  synthetic per-blade `rootMOOP` in scenarios 8, 26 and 27 only, and those are
  exactly the three the gate red test moves. Scenarios 6, 16 and 18 execute the
  call site **108,000 times on a zero input**, where the filter's output is zero
  and scaling it is the identity. Hit count is not visibility.
* **The kernel cannot constrain five of the twelve C parameters, nor the
  saturation branch**, and the stub that says so scores 14,508 of 14,508
  `IDENTICAL` (`kernel.constant-args-stub-PASSES.verify_fields.csv`). This unit
  has exactly ONE call site in the whole controller — clean `Filters.f90:405` —
  and it passes `Damp` as the literal `0.7_DbKi`, `Moving` as the literal
  `.TRUE.`, and no `InitialValue` at all. `CornerFreq` is `LocalVar%RotSpeedF`,
  a filtered rotor speed, so `CornerFreq < 0` is false in every captured case
  and the whole ELSE-less half of the guard is unreachable from the simulation.
  A translation with all five substituted by literals and the branch deleted is
  indistinguishable from the real one at this layer.
* **What the kernel CAN see, and `NotchFilter` could not.** That unit's four
  frequency arguments were read only inside its initialisation branch, and 1 of
  62 cases entered it. Here the call site passes `Moving = .TRUE.`, so the
  coefficient block runs on EVERY call: `DT` and `CornerFreq` are read by all 62
  cases, and `rotspeedf` has **23 distinct non-zero reference values**.
* **The harness holds 74 C parameters inert**, all of them the other filters'
  coefficient arrays inside `TYPE(FilterParameters)`. A defect confined to one of
  those is invisible to it — and this unit cannot touch them.

## `DT**2.0` is a REAL exponent, and whether that is `x*x` had to be measured

`pow_two_probe.f90`, re-run here on this unit's own operands rather than cited
from unit #13. gfortran emits a libm `pow` for a real exponent of 3.0 or more
and the two answers differ by 1 ULP, so `x**2.0 == x*x` is not something to
assume. At this campaign's own flags (`-O3 -fdefault-real-8 -fdefault-double-8
-ffp-contract=off`) the two have **identical bit patterns on 200,010 values**,
including the overflow-to-Inf and underflow-to-zero endpoints. So `DT * DT` and
`CornerFreq_ * CornerFreq_` are exact here and carry no libm call.

`**` still binds tighter than `*`: `Damp*DT**2.0*CornerFreq_**2.0` is
`Damp * (DT*DT) * (CornerFreq_*CornerFreq_)`, and it was exactly that
parenthesisation that left four mutants alive at unit #13.

## The one survivor: a corpus that could not contain a negative zero

The mutation score came in at **0.988** with a single survivor:

```
38bbc289   compare_op   'if (CornerFreq < 0)'  ->  'if (CornerFreq <= 0)'
```

It was never equivalent, and this directory proves it rather than arguing it —
the campaign's rule since unit #8. `negative_zero_survivor_probe.cpp`:

* The two predicates disagree on exactly one input, `CornerFreq == 0`. At **+0.0**
  both branches produce identical bits — **0 of 36** draws differ. At **−0.0**
  they do not: the reference takes the ELSE and carries the sign into
  `2.0*DT*CornerFreq_`, the mutant substitutes `+0.0`. **36 of 36** differ, and
  `FP%nfs_b2` comes out `8000000000000000` against `0000000000000000`.
* **In every one of those 36 the RETURN VALUE agrees**, because IEEE addition
  collapses `(+0.0) + (−0.0)` to `+0.0`. A differential harness comparing only
  the unit's result could not have killed this mutant at any input; it is
  reachable only because `R4_compare_all_outputs` compares the coefficient
  out-parameters too.

The gap was in `harness/generate.py`, and it is a new shape: **the corpus could
not hold a −0.0 because the dedup that keeps its ladders honest absorbs one.**
`_real_magnitude_ladder` and `_ladder` both end in `list(dict.fromkeys(out))`,
and `0.0 == -0.0`, so a `-0.0` written into either list disappears into the
`0.0` already there. Every earlier rung this generator gained could be added
where it belonged; this one could not, which is why it went in as a block
appended last (loop repo `5d83048`).

| corpus | cases | killed | score | survivors |
|---|---|---|---|---|
| as inherited (loop `0c6c9bb`) | 4468 | 83/84 | 0.988 | 1 |
| + negative-zero block (loop `5d83048`) | **4508** | **84/84** | **1.000** | **0** |

The mutant dies on **2 of 4508** cases, and 2 is the figure that says the block
is what did it: the block adds 40 cases (5 defaulted scalar reals × 8 flag
variants) and only the two that put `CornerFreq` itself at −0.0 *and* enter the
coefficient branch can move an output.
`negative_zero_block_kills_the_saturation_mutant.txt` is that measurement.

**The translation was already right.** Writing the saturation as the Fortran's
branch rather than as `std::fmax(CornerFreq, 0.0)` is what made the two
distinguishable at all — `fmax(-0.0, 0.0)` returns `+0.0`, which is the mutant's
answer. Transcribing the shape instead of the algebra is a rule this campaign
has now paid for twice.

## Files

| file | what it is |
|---|---|
| `notchfilterslopes.final.cpp` | the translation as committed |
| `notchfilterslopes.zero-stub.cpp` / `kernel.zero-stub-FAILS.verify_fields.csv` | reads nothing, returns 0.0 — 0/62, 661 rows `OUT_TOL` across exactly the 11 fields the unit writes |
| `notchfilterslopes.constant-args-stub.cpp` / `kernel.constant-args-stub-PASSES.verify_fields.csv` | `Damp`, `Moving`, `InitialValue` as literals and the saturation branch deleted — 14,508 of 14,508 IDENTICAL |
| `notchfilterslopes.noop-stub.cpp` / `harness.noop-redtest.full.log` | the harness red test, and the truncated-to-8 mismatch list it prints |
| `kernel.verify_fields.csv` | 14,508 rows, all IDENTICAL |
| `kernel-window.statefiles.lst` | the 62 captured indices, exact against the configured window |
| `kernel-generated-Filters.post-verify.f90` | the generated kernel source (POST-`vit verify`, per unit #7) |
| `scenario_arguments.txt` | every reader of `RootMOOPF`, the guard on the one call site, and which scenarios drive it non-zero |
| `pow_two_probe.*` | `x**2.0` vs `x*x` at the campaign's flags, on this unit's operands |
| `negative_zero_survivor_probe.*` | the probe that turned the survivor into a corpus fix |
| `negative_zero_block_kills_the_saturation_mutant.txt` | 0.988 → 1.000, and the 2 cases that do it |
