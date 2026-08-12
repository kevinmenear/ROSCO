# NotchFilter — unit #13

`REAL(DbKi) FUNCTION NotchFilter(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst, InitialValue)`
in `rosco/controller/src/Filters.f90`. A second-order discrete notch filter: on
the first call or a reset, set four state slots and five coefficients from the
bilinear-transform gain `K = 2/DT`; then five products accumulated in source
order; then shift the two input and two output histories and advance the
instance counter.

**Five layers ran and all five are alive** — the second unit in this campaign of
which that is true, after `LPFilter`.

| layer | result | red test |
|---|---|---|
| kernel replay | 62/62 cases, 14,508 field rows, ALL `IDENTICAL` | zero stub 0/62, moves 375 rows; VIT's own `InputSignal * 1.00001` DISCRIMINATING |
| differential harness (clean Fortran) | 2,652 checked, 0 failed, 0 inadmissible | the unit as a no-op fails **2,652 of 2,652** |
| mutation score | **126 of 126** behavioural killed, **1.000**, **0 declared equivalent** | no survivor; 1 excluded as `killed (no compile)` |
| post-integration harness (the wrapper) | 2,652 checked, 0 failed | `betaNum`/`betaDen` swapped at the bridge call: 980 of 2,652 |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 551,278 of 5,252,000**, 0 after revert |

## The gate can see this unit, and the number is its own

`gate/NotchFilter.redtest.json` — scaling the returned value by 1.000001 moves
**551,278 of 5,252,000**, and the revert returns 0. A red test that goes red
needs no same-build control: it has just demonstrated the chain a control would
be asked to demonstrate (unit #9's entry exists for the opposite case).

Unit #12's check applied — compare the moved count against every committed
redtest artifact before writing "the gate sees this unit":

```
ColemanTransform 124,353   ColemanTransformInverse 389,644   GetWords 1,857,893
LPFilter 1,592,059   NonDecreasing 1,857,893   __gate__ 1,604,573
NotchFilter 551,278   <- matches none of them
```

So this is a fifth gate-visible unit and not a sixth sighting of unit #12's
shared refuse-to-start. `LocalVar%GenSpeedF` is the filtered generator speed the
torque and pitch controllers are both built on, and it leaves the DLL as
`LocalVarOutData(43)`. The count is a third of `LPFilter`'s because only **6 of
the 27 scenarios configure a notch filter at all**.

## What each layer could NOT see

* **The kernel cannot constrain four of the eleven arguments** — `DT`, `omega`,
  `betaNum`, `betaDen` — and the stub that says so scores 14,508 of 14,508
  `IDENTICAL`. Every scenario that runs the unit patches the *same* notch
  configuration (`F_NotchFreqs 1.0000`, `F_NotchBetaNum 0.0000`,
  `F_NotchBetaDen 0.2500`); the 14 unpatched `Examples/*.IN` all carry
  `F_GenSpdNotch_N=0` and never call it. `scenario_arguments.txt` is that grep.
  And it is unit #11's *sharper* version of the finding: all four are read only
  inside the initialisation branch, and exactly **1 of the 62 captured cases has
  `istatus == 0`**, so 61 of them do not read any of the four at all.
  `betaNum = 0` additionally collapses `nf_b2` and `nf_b0` to the same
  expression in every simulated case.
* **The kernel window is at one call site of four, and it is the only live one.**
  The tower-top notch pair (clean `Filters.f90:380/386`) and the `Flp_Mode==2`
  blade site (413) have zero hits in all 27 scenarios. None of the four passes
  `InitialValue`, so the `PRESENT` branch is reached by the harness alone.
* **The harness holds 74 C parameters inert**, all of them the other filters'
  coefficient arrays inside `TYPE(FilterParameters)`. A defect confined to one
  of those is invisible to it — and this unit cannot touch them.

## `K**2.0` is a REAL exponent, and whether that is `K*K` had to be measured

`pow_two_probe.f90`. gfortran emits a libm `pow` for a real exponent of 3.0 or
more and the two answers differ by 1 ULP, so `x**2.0 == x*x` is not something to
assume. At this campaign's own flags (`-O3 -fdefault-real-8 -fdefault-double-8
-ffp-contract=off`) the two have **identical bit patterns on 200,010 values**,
including the overflow-to-Inf and underflow-to-zero endpoints. So `K * K` is
exact here and carries no libm call. Anything above exponent 2.0 would need
`std::pow`.

## The four survivors: a corpus blind spot, closed by measurement not declaration

The mutation score came in at **0.968** with four survivors, all one shape:

```
95f5788b / eddd5776   '2.0 * (omega * omega)' -> '(2.0 * omega) * omega'
3227266f / 6154466d   '2.0 * (K * K)'         -> '(2.0 * K) * K'
```

Two of each because `nf_b1` and `nf_a1` are the same expression written twice,
which is how the Fortran writes it.

**They were never equivalent, and this directory proves it rather than arguing
it** — the campaign's rule since unit #8.

1. `assoc_reorder_survivor_probe.cpp` — in isolation the two groupings differ on
   **130,696 of 20,000,000** full-exponent-range draws and **0 of 20,000,000**
   inside ±1e3. Multiplying by two is exact and rounding is invariant under
   scaling by a power of two — *in the normal range*. Once `x*x` is subnormal the
   grid is absolute, `2*round(y)` is an even multiple of 2⁻¹⁰⁷⁴ and `round(2y)`
   may be an odd one. First witness `x = 1.3462737399449473e-155`.
2. `assoc_reorder_equivalence_probe.cpp` — the same question through the whole
   coefficient, over the **reachable** inputs (`DT`, not `K`: the reference
   computes `K = 2.0/DT`, so `|K|` below `2/DBL_MAX` is not reachable from any
   finite timestep). Witness at `DT = 2.4406824148637749e+204`,
   `omega = 1.3397359972566569e-154`, `betaDen = 0`: the coefficient is exactly
   `2` and the mutant gives `2.0000000000000004`.
3. `hot_mantissa_probe.cpp` — *which* magnitudes. At exponent −535, **126 of 256
   mantissas** differ, and end to end with `DT = 1e300` the same 126 kill the
   omega-mutant. Exponents −512, −520, −530, −540 and −550 give **0 of 256**, and
   the round rungs `1e-155`, `1e-161` and `sqrt(DBL_MIN)` do not differ while
   `1e-156`, `1e-158` and `1e-160` do.

So the gap was in `harness/generate.py`: it had **never drawn a real outside
±1e3** — `_ladder` and `_literal_values` are both driven off the declared range
and `_bounds` defaults to ±1e3, which is a sliver of a REAL(8)'s 1e±308. That is
unit #10's scalar-integer finding, one type over.

Closed by two additions, both appended last and both silent when they do not
fire, so no earlier unit's draws move. `joint_magnitude_block_kills_the_assoc_mutants.txt`
is the number that says what each bought:

| corpus | cases | killed | score | survivors |
|---|---|---|---|---|
| as inherited | 1380 | 122/126 | 0.968 | 4 |
| + one-at-a-time real magnitude ladder | 2172 | 122/126 | 0.968 | 4 |
| + the same rungs with every OTHER defaulted scalar real pinned to 0.0 / 1e300 | 2652 | **126/126** | **1.000** | **0** |

**The middle row is the transferable finding.** Reaching the magnitude was not
enough. A rung puts one parameter at 1e-158 while the other terms of the same
expression stay at ±1e3, and a 2⁻¹⁰⁷⁴ difference is annihilated in the
subtraction. Only when the others are pinned out of the way — `0.0`, or `1e300`
so that `K = 2.0/DT` underflows — does the regrouping reach an output. A ladder
that varies one parameter at a time is blind to any defect the other parameters
can dominate.

## An instrument finding recorded before it was worked around

The **first** post-integration red test reported `checked 2652 failed 0`, which
reads exactly like a harness that cannot fail. It was not: the perturbation was
never applied. `notchfilter_c(InputSignal, DT, omega, betaNum, betaDen,` occurs
**twice** in the integrated `Filters.f90` — once in the `BIND(C)` interface block
and once at the call — so a single-occurrence replacement asserted out and left
the source untouched, and the rebuild dutifully rebuilt nothing. Perturbing the
call site alone (`Filters.f90:330`) fails 980 of 2652. Unit #7's rule ("a
post-integration red test can stay green because the perturbation never reached
the binary") has a second cause, and it is upstream of the rebuild rather than
downstream.

## Files

| file | what it is |
|---|---|
| `notchfilter.final.cpp` | the translation as committed |
| `notchfilter.zero-stub.cpp` / `kernel.zero-stub-FAILS.verify_fields.csv` | reads nothing, returns 0.0 — 0/62, 375 rows `OUT_TOL` |
| `notchfilter.hardcoded-coeffs-stub.cpp` / `kernel.hardcoded-coeffs-stub-PASSES.verify_fields.csv` | `DT`/`omega`/`betaNum`/`betaDen` as literals — 14,508 of 14,508 IDENTICAL |
| `notchfilter.noop-stub.cpp` / `harness.noop-redtest.full.log` | the harness red test, and the truncated-to-8 mismatch list it prints |
| `kernel.verify_fields.csv` | 14,508 rows, all IDENTICAL |
| `kernel-window.statefiles.lst` | the 62 captured indices, exact against the configured window |
| `kernel-generated-Filters.post-verify.f90` | the generated kernel source (POST-`vit verify`, per unit #7) |
| `scenario_arguments.txt` | what the 27 scenarios actually hand the unit, and every reader of `GenSpeedF` |
| `pow_two_probe.*` | `x**2.0` vs `x*x` at the campaign's flags |
| `assoc_reorder_*`, `hot_mantissa_probe.*` | the four probes that turned a survivor into a corpus fix |
| `joint_magnitude_block_kills_the_assoc_mutants.txt` | 0.968 → 0.968 → 1.000, and what changed between them |
