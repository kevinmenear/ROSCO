# LPFilter — unit #11

`REAL(DbKi) FUNCTION LPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue)`
in `rosco/controller/src/Filters.f90`. A first-order discrete low-pass filter:
set six coefficients on the first call or on a reset, then one reciprocal and
three products, then save the input and the output for the next step and advance
the instance counter.

**Five layers ran and all five are alive.** This is the first unit in this
campaign of which that is true — units #5 to #10 each closed with the gate
proved blind to them.

| layer | result | red test |
|---|---|---|
| kernel replay | 62/62 cases, 14,508 field rows, ALL `IDENTICAL` | zero stub moves 172 rows; VIT's own `InputSignal * 1.00001` DISCRIMINATING |
| differential harness (clean Fortran) | 996 checked, 0 failed | the unit as a no-op fails **996 of 996**, naming every output |
| mutation score | **38 of 38** behavioural killed, **1.000**, 0 declared equivalent | no survivor at any point; 2 excluded as `killed (no compile)` |
| post-integration harness (the wrapper) | 996 checked, 0 failed | marshalled result `* 1.000001` fails 656 of 996 |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 1,592,059 of 5,252,000 moved**, 0 after revert |

## The gate can see this unit, and that is new

`gate/LPFilter.redtest.json` — scaling the returned value by 1.000001 moves
**1,592,059 of 5,252,000** compared values, and the revert returns 0. No control
run was needed: a red test that goes RED needs no proof that the chain from
build to install to 27 simulations to bit comparison is alive, because it just
demonstrated it. (Unit #9's control entry exists for the opposite case.)

Four of this campaign's eleven units are now gate-visible — ColemanTransform
(124,353), ColemanTransformInverse (389,644), GetWords (1,857,893), LPFilter
(1,592,059); seven are not.

That was predictable from the call graph and it was still worth measuring: the
unit has **21 call sites, 18 of them live, 3,527,912 calls across 23 of the 27
scenarios**, and `LocalVar%GenSpeedF` — the output of the call site the kernel
was taken at — is the speed error the torque and pitch controllers are built on.

## What each layer could NOT see

* **The kernel cannot constrain `CornerFreq`.** `CntrPar%F_LPFCornerFreq` is
  `1.57080` in all 14 `Examples/*.IN` and `vit_sim.py` never patches it, so a
  translation that ignores the argument and writes the literal is
  indistinguishable from the real one — unit #3's shape, unit #9's at this same
  type. `kernel.hardcoded-cornerfreq-stub.verify_fields.csv` is that
  measurement. Only the differential harness varies it.
* **The kernel window is at one call site of eighteen.** `Filters.f90:347`,
  `LocalVar%GenSpeedF = LPFilter(LocalVar%GenSpeed, ...)`, scenario 1, the
  default `run_cmd`. The window is exactly the configured
  `0:0:1-20,12000-12020,23900-23920` = 62; `kernel-window.statefiles.lst` is the
  index check unit #4's entry asks for, and it is exact — no strays.
* **`has_InitialValue` is outside the kernel but NOT dead.** Unlike HPFilter's,
  exactly one live call site passes the OPTIONAL argument —
  `ControllerBlocks.f90:377`, 407,976 calls in 23 scenarios — so the `PRESENT`
  branch is exercised by the simulation, just not at the site the kernel was
  taken at. The harness varies it and `negate_cond` on it dies 884 of 996.
* **The harness holds 80 of 100 C parameters inert**, all of them the other
  filters' coefficient arrays inside `TYPE(FilterParameters)`. A defect confined
  to one of those is invisible to it — and to everything else, since this unit
  cannot touch them.

## The kernel window is NOT vacuous, and that was checked rather than assumed

`genspeedf` has **62 distinct reference values across 62 cases**. Unit #9's
`HPFilter` first extraction ran on a window where a zero-writing stub scored
14,508/14,508 IDENTICAL; the check that would have caught it is this one, and it
is in the field log rather than in the verdict.

The zero stub here fails: **172 of 14,508 rows `OUT_TOL`**, and the fields it
moves are exactly this unit's outputs — `genspeedf`, `lpf1_a1`, `lpf1_a0`,
`lpf1_b1`, `lpf1_b0`, `lpf1_inputsignallast`, `lpf1_outputsignallast`. The other
14,336 rows are the enclosing procedure's untouched state.

## Two instrument findings, both recorded before they were fixed

### 1. The harness read the INDEX ROLE off the translation, so the red test could not run

`map_signature` promotes a scalar to `role="index"` by finding `FP->lpf1_a1[i]`
in the **C++**. The no-op red test has no such subscript, so `inst` stayed an
ordinary scalar integer, the generator drew it from unit #10's decade ladder up
to `INT_MAX`, and the **reference** — which subscripts a `DIMENSION(1024)` array
with it — segfaulted. `harness.sh` printed *"harness produced no JSON"* and wrote
no artifact: `instrument_defect/harness-noop-redtest.no-json.log`, and the
1157-case count in it against the green run's 996 is the visible symptom.

The input domain of a differential harness must not be a function of the code
under test. Fixed by ADDITION in the loop repo — `infer_indexes_fortran` reads
`arg%field(expr)` out of the unit's own Fortran body, and runs *after* the C++
pass so it can only fill a gap the translation left. Every already-measured
unit's corpus is unchanged; this unit's red run came back at **996 cases, the
green run's own figure**, and failed all 996.

### 2. `vit_mutate.py` mutates the translation IN PLACE, and a killed run leaves the mutant

A mutation run was stopped mid-flight to correct a comment. It restores the file
when it finishes; killed, it does not. What it left was
`*inst = *inst + 2;` in `translations/Filters/lpfilter.cpp` — the file the next
`vit integrate` reads. `git status` showed the same untracked path it had shown
all along.

It was caught by re-running the differential harness for an unrelated reason and
getting `checked 996  failed 996` where the identical command had passed minutes
earlier: `instrument_defect/harness-green.mutant-left-by-killed-mutation-run.log`.
The rule now in the RUNBOOK is to diff the translation against a saved copy after
every mutation run, and to treat any artifact taken during one as measuring a
mutant.

## A post-integration red test that was EQUIVALENT, and why that is a property of this unit

`harness.postintegration.redtest.args-swapped-EQUIVALENT.json`. Swapping two
INTENT(IN) arguments in the wrapper is the campaign's standard post-integration
perturbation — it fails 85 of 829 for HPFilter. Here it fails **0 of 996**, and
that is correct: `DT` and `CornerFreq` are read only inside the initialisation
branch and only as the product `CornerFreq*DT`, which is symmetric. Swapping
them changes nothing that any input can observe.

Kept rather than deleted. A red test that stays green is either a broken
instrument or an equivalent perturbation, and the two are told apart by reading
the unit — not by re-running. The real red test scaled the marshalled result and
failed 656 of 996; the other 340 cases have a reference result of exactly 0.0,
which scaling cannot move.

## Files

| file | what it is |
|---|---|
| `lpfilter.final.cpp` | the translation as committed |
| `lpfilter.zero-stub.cpp` | reads nothing, writes nothing, returns 0.0 |
| `lpfilter.hardcoded-cornerfreq-stub.cpp` | the real translation with `CornerFreq` replaced by `1.57080` |
| `kernel.verify_fields.csv` | 14,508 rows, all IDENTICAL |
| `kernel.zero-stub-FAILS.verify_fields.csv` | 172 rows `OUT_TOL` — the comparison is alive |
| `kernel.hardcoded-cornerfreq-stub.verify_fields.csv` | what the kernel can say about a constant argument |
| `kernel-window.statefiles.lst` | the 62 captured indices, checked against the configured window |
| `kernel-generated-Filters.post-verify.f90` | the generated kernel source (POST-`vit verify`, per unit #7) |
| `instrument_defect/*` | the two instrument findings above, in the state they were found |
| `harness.postintegration.redtest.args-swapped-EQUIVALENT.json` | the red test that stayed green, and should have |
