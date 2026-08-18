# Unit #51 — `FloatingFeedback`

`rosco/controller/src/Controllers.f90:608-637` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested, and the mutation
score is **1.000** against a threshold of 1.000.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/FloatingFeedback.json`) | **6734 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, both callee bridges kept so each side runs one `interp1d` and one `PIController` — this unit's primary evidence | the unit as a no-op: **6734 of 6734**, the whole corpus, the same count, and the count the stub predicted in its own header |
| mutation (`mutation/FloatingFeedback.json`) | **24 of 24 scoreable, 1.0000**, 2 declared, 0 no-compile, 5 operators of 12 offered, **no survivor, `declared_but_killed` empty** | the score *is* the red test, 24 times |
| post-integration (`harness/FloatingFeedback.postintegration.json`) | 6734 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **6734 of 6734**; reverted, rebuilt, green re-taken at 0, and the revert checked both ways |
| gate, 27 scenarios (`gate/FloatingFeedback.json`) | 5,252,000 values / 351 channels, 0 mismatched | **THREE, and they add up**: the unit's answer offset 0.01 rad moves **404,454** across all four scenarios that call it; the mode-1 arm alone **223,222** (scenarios 3, 7); the mode-2 arm alone **181,232** (scenarios 19, 27). 223,222 + 181,232 = 404,454 exactly |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for units #45 through #50, and it is
the right one here for a reason of this unit's own: its `PIController` callee
carries per-instance state indexed by an `objInst` counter it post-increments,
and unit #44 measured that a KGen kernel cannot replay SAVE-like state.

## What this unit is

Seven statements. One `interp1d` schedules the floating gain on the filtered
wind-speed estimate; two `PIController` calls integrate the two nacelle
acceleration signals into velocities; a two-arm `IF`/`ELSEIF` multiplies one of
those velocities by the scheduled gain and returns it. The caller adds the
result to `LocalVar%PC_PitComT`.

Three things are worth knowing before reading the translation:

1. **Both `PIController` calls run unconditionally, above the mode test, and the
   order is load-bearing.** Only one result is ever used, so the obvious shape is
   to compute each inside its own arm — and that is a different program.
   `PIController` is INTENT(INOUT) in both `piP` and `inst`: it writes
   `piP%ITerm(inst)` and `piP%ITermLast(inst)` and post-increments `inst`. The
   reference therefore advances `objInst%instPI` by exactly **two** per call
   whatever the mode, putting the fore-aft signal in instance *n* and the
   nacelle-IMU signal in *n+1*. Moving either call into an arm would swap the
   two integrator states and leave the counter one short for every later PI user
   in the same DISCON call.
2. **`0.0_DbKi - x` and not `-x`**, on both arms. They agree on every finite
   non-zero *x* and differ at a zero, and this is not exotic here: `PIController`
   returns its `I0` on a reset and this call site passes `I0 = 0.0`, so both
   velocities are exactly `+0.0` on the whole `restart` half of the corpus.
   **1544 of 6734 cases return negative zero** — measured, in
   `fallthrough_census.txt`. Had the expression been simplified to `-FA_vel` the
   same cases would return `+0.0`, a different bit pattern on a four-figure
   count.
3. **The field is spelled `NACIMU_FA_AccF` in `LocalVariables`** (all-capitals,
   `ROSCO_Types.f90:427`) while `DebugVariables` carries a separate
   `NacIMU_FA_AccF` in mixed case (`:508`). Fortran is case-insensitive; the view
   struct is generated from the declaration, so C++ must use the capitals.

## The finding: two arms of a three-way split, and the third has no answer

```fortran
Controllers.f90:631-635
    if (CntrPar%Fl_Mode == 1) THEN
        FloatingFeedback = (0.0_DbKi - FA_vel) * LocalVar%Kp_Float
    ELSEIF (CntrPar%Fl_Mode == 2) THEN
        FloatingFeedback = (0.0_DbKi - NacIMU_FA_vel) * LocalVar%Kp_Float
    END IF                                            <- no ELSE
```

The result variable is written on those two arms and nowhere else. On any other
`Fl_Mode` the function returns an undefined slot — the third instance of this
shape in the campaign, after unit #39 (`ResController`'s `reset` path) and unit
#50 (`FlapControl`'s never-written `RootMyb_VelErr`).

**THE SHIPPED PROGRAM CANNOT PRESENT ONE, AND IT IS AN INVARIANT OF TWO
STATEMENTS RATHER THAN LUCK.**

```
Controllers.f90:95     IF (CntrPar%Fl_Mode > 0) THEN
                           LocalVar%Fl_PitCom = FloatingFeedback(...)
checkinputs.cpp:297    if ((Fl_Mode < 0) || (Fl_Mode > 2)) {
                           aviFAIL = -1;  "Fl_Mode must be 0, 1, or 2."
```

(the second is unit #29's translation of the same test in the clean
`ReadSetParameters.f90`; `aviFAIL = -1` aborts before the first controller call).
The call site admits `Fl_Mode >= 1`, the input reader admits `Fl_Mode <= 2`, and
the two arms cover `{1, 2}` exactly. All four gate scenarios take one arm on
every one of their 79,996 calls and none falls through.

**WHAT WAS DONE, AND THE TWO THINGS THAT WERE NOT.**

* The domain is stated: `CntrPar_Fl_Mode = { values = [1, 2] }`, with the source
  of both bounds and the three costs written out in `harness/ranges.toml`.
* **`no_oracle = "vit_result"`** is unit #39's spelling for the same problem and
  it was rejected here on a structural argument rather than a measurement:
  `vit_result` **is** this unit's answer. The two arms are the only arithmetic it
  has, and everything else it writes — `Kp_Float`, `piP`, `instPI` — is written
  by a callee that is the *same implementation on both sides* of this comparison.
  Excusing the return would have left the primary layer comparing nothing this
  translation computes. Unit #39 could afford it because its `reset` path is
  reachable and its other four outputs are its own.
* **The C++ result is left UNINITIALISED**, which is the opposite of what unit
  #39 did (`rescontroller.cpp:48` defines its result at `0.0`). The two units
  differ on a fact, not on taste: `ResController`'s unassigned path is *reachable
  and taken on the first call of every simulation*, so returning an indeterminate
  double there would be undefined behaviour executed in production. This unit's
  is reachable by no ROSCO configuration. Defining it at `0.0` would be this
  translation answering a question the reference does not answer (P7), on inputs
  the program cannot present.

## The knob measurement the green was resting on

The entry's own cost note carried a prediction, from unit #49's `ErrVar_aviFAIL`
histogram `{-1: 2767, 0: 2446, 1: 2422, 2: 5}`: a stated `values` list does not
bound R7's predicate-knob cross product. The harness prints the same warning
shape for this unit —

```
PREDICATE KNOB: CntrPar_Fl_Mode at [0.0, 1.0, 2.0, 3.0]
R6 ... 4 combination(s) of the 1 quantit(y/ies) the reference's own predicates
       test, the full cross product
```

— and 0 and 3 are the collating neighbours of the two literals this unit tests,
both of which fall through. **If the corpus held them, the green would be
resting on two undefined slots that happened to agree** — unit #36's
out-of-bounds pass in a different costume.

Measured (`fallthrough_census.txt`), over the scored corpus with the case file
byte-identical either side:

```
PROBE lines                                6734    (= the whole corpus)
Fl_Mode == 1                               3518
Fl_Mode == 2                               3216
FALLTHROUGH                                   0
cases where the two sides' return differ      0
```

The stated list **did** bound the knob here. That is the narrow statement and the
only one the probe supports: it is not evidence about unit #49's `aviFAIL = 2`
and not a claim about the generator. The reason to write it down is that the
opposite result would have invalidated the primary layer, and nothing but this
probe distinguishes the two.

## Two survivors, one site shape, and the argument was executed

| site | mutants | why unkillable |
|---|---|---|
| `floatingfeedback.cpp:160` and `:163`, the `1` of `LocalVar->restart ? 1 : 0` | 2 (`const_tweak '1' -> '2'`) | the callee bridge converts with `(reset /= 0)` and the integrated `picontroller.cpp:30` with `if (reset != 0)`, so `? 2 : 0` and `? 1 : 0` are one argument on both trees |

**(c)-class** in unit #48's vocabulary — behaviour-preserving everywhere, not
merely unreachable. The same declaration unit #50 made for the same token at
`flapcontrol.cpp:217`.

**EXECUTED, NOT ARGUED** (`equivalence_probe.txt`), with both controls:

```
restart == 1  -- the `? 1` branch is TAKEN     3209 of 6734   <- positive control
restart == 0  -- the `: 0` branch is taken     3525
'0' -> '1' on the SAME conditional, line 160   KILLED at 3461 <- counter-control
'0' -> '1' on the SAME conditional, line 163   KILLED at 3461
```

3209 cases select the very token the mutation changes and it still survives; the
other arm of the same conditional dies at 3461 in both places. Same operator,
same statement, same corpus, opposite verdicts — which is the whole difference
between "the corpus cannot reach this" and "no corpus can distinguish this".

## Nothing here moved zero

This is the first unit in five with no annihilated-by-an-exact-zero gate finding
— #47's `AWC_amp` and #48's `WE_Gamma` were configured gains, #49's was a state
the run length never let the unit write, and #50's was a 1-DOF simulation's
`rootMOOPF`. The reason here is structural rather than lucky: this unit's answer
is *added* to `LocalVar%PC_PitComT`, a live pitch command in every scenario that
reaches the call, and all three perturbations are additive at that sum.

## What each file is

| file | what it records |
|---|---|
| `floatingfeedback.hstub-noop.cpp`, `run_harness_redtest.sh` | the no-op red test and its runner (the stub keeps two guarded, unreachable callee calls so the bridges are still emitted — unit #45's rule; the runner asserts the translation is in HEAD before arming a trap that would delete it) |
| `harness.noop.json` | that red test: **6734 of 6734**, the whole corpus, as the stub predicted |
| `mutation.first_take.json` | the sweep BEFORE any equivalence was declared, `equivalent_declared: 0`, 24 of 26 at 0.9231 — what survived, on the record before it was excused |
| `fallthrough_census.txt` | the empty fall-through set, the 1544 negative zeros, and the knob prediction it corrects |
| `equivalence_probe.txt` | the executed argument behind both declarations, with the positive and the counter control |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the wrapper red test, **6734 of 6734** |
| `gate.redtests.txt` | the three gate perturbations, their per-scenario channel lists, and the sum that says the arms partition |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../harness/ranges.toml` | three pins: the `Fl_U`/`Fl_Kp` extent tie, `Fl_Mode`'s stated domain, and `instPI` |
| `../../mutation/FloatingFeedback.equivalences.json` | the two declarations and their proof |

## The findings worth carrying

1. **A stated `values` list bounding R7's predicate knob is checkable in one
   probe, and it is worth checking when the excluded region is one the reference
   has no answer on.** The green would otherwise rest on two undefined slots
   agreeing.
2. **`no_oracle` on the RETURN VALUE is not available to a unit whose return
   value is its whole answer** — even when the identical shape one unit earlier
   took exactly that route. Unit #39's `vit_result` entry and this unit's
   `values` pin solve the same problem and are not interchangeable.
3. **Two arm-scoped gate perturbations whose counts SUM to a shared one's is an
   arithmetic control on the arm attribution** — cheaper than arguing mutual
   exclusion and it fails loudly if the arms overlap.
4. **`--reverse-copy` is decided by reading the emitted wrapper, not by
   remembering.** The first `vit integrate --apply` here emitted no copy-back at
   all, which would have left `LocalVar%Kp_Float` dying in the view struct.
