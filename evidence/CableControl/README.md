# Unit #49 — `CableControl`

`rosco/controller/src/Controllers.f90:884-964` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested, and one honest
failing number: the mutation score is **0.9647** against a threshold of 1.000.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/CableControl.json`) | **3354 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, all three callee bridges KEPT so both sides run one `interp1d`, one `SecLPFilter_Vel` and one `PIController` — this unit's primary evidence | the unit as a no-op: **3207 of 3354**, the same corpus count, byte-identical case file either side |
| mutation (`mutation/CableControl.json`) | **82 of 85 scoreable, 0.9647**, 4 declared, 0 no-compile, 7 operators, **3 survivors standing** | the score *is* the red test, 82 times |
| post-integration (`harness/CableControl.postintegration.json`) | 3354 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **3157 of 3354**; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/CableControl.json`) | 5,252,000 values / 351 channels, 0 mismatched | **THREE**: the first step constant moves **14,572** (scenarios 7 and 27); PI in its 12th digit moves **2**; the whole `CC_Mode == 2` arm made unreachable moves **0**. All three revert-verified |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for units #45, #46, #47 and #48.

## What this unit is

`StructuralControl`'s twin — the same five arguments, the same three arms, the
same `Time > 500` step, the same `interp1d` row-of-a-2-D-array call, the same
`avrSWAP` write loop. Three things differ and each one is a place a
transcription can go wrong:

1. The write loop writes **two** slots per group, at `CC_GroupIndex(I)` and at
   `CC_GroupIndex(I) + 1`. `harness/ranges.toml` pins the index to 1..**2999**
   and not to 1..3000 for exactly that reason: unit #43's bound would put the
   second write one past the 3000-element buffer the harness allocates.
2. Between the arms and the write loop there is a second loop calling two
   **stateful** integrated callees, `SecLPFilter_Vel` and `PIController`, each
   indexed by an `objInst` counter it post-increments.
3. `RoutineName` is the string `'StructuralControl'`. That is an upstream
   copy-paste in ROSCO and it is transcribed as written (P7): correcting it
   would make this translation disagree with the program it replaces on every
   input that reaches the prefix, and the gate never reaches that statement.

## The finding: a NaN that hid two coefficients, and the state that removed it

The first sweep scored **77 of 89 with twelve survivors**, two of them
`PIController`'s `kp` and `ki`. The kill counts suggested a saturation story —
`-1000.0 -> -1001.0` was killed on 1703 cases, so the integrating arm was plainly
being reached and clamped. That reading is wrong. `SecLPFilter_Vel` computes its
six filter coefficients **only** inside `IF ((iStatus == 0) .OR. reset)`, and
`LocalVar%FP` is a nested type the generator zero-initialises and does not vary —
the harness says so itself, `UNOBSERVABLE LocalVar.FP: nested type,
zero-initialised (not varied)`. So every case with `iStatus /= 0` and `restart`
false divides by `a2 = 0.0`, returns `Inf * 0.0`, and a NaN propagates all the way
to `avrSWAP`. `kp*error` is NaN whether kp is 0.0 or 1.0.

And where the corpus *did* initialise the filter it did so through `restart`,
which is the **same value** this unit hands `PIController` as its `reset` — and
that arm assigns `I0` and reads neither gain. The one configuration in which
either gain can reach an answer is `iStatus == 0` **with** `restart` false, and no
rung of any ladder crosses those two names.

Three baseline states carry it, plus two index ramps at the open-loop arm.
**Five of the twelve survivors died.**

```
              cases   mutants   score
first take     3300   77 of 89  0.865  (12 survivors, 0 declared)
second take    3354   82 of 85  0.9647 ( 3 survivors, 4 declared)
```

## The three that stand, and why they are (b) rather than (c)

All three are in the two copied ErrMsg helpers; **not one is in the arithmetic**.
`errmsg_census.txt` reads the scored corpus's own `(aviFAIL, n_ErrMsg,
n_ErrMsg_cap)` triples and gives, per mutant, the input it needs and the count of
cases holding it — 0, 0 and 0 — plus the argument that a *green* case could hold
it, which is what separates these from unit #48's two blind spots.

The cause is one fact and it is the **base draw**, not the ladders:

```
case 0:   aviFAIL = 300     CC_Mode = -300
```

both midpoints of the ±1e3 default, which is unit #47's "a midpoint is not a
mode" met a second time. R13's whole 256-case capacity block and R6's
character-length rungs are taken "with every other input at its base draw", so
all of them sit where `IF (ErrVar%aviFAIL < 0)` is false and neither helper runs.

**Third unit at this wall** — #43 recorded it, #48 met it from the other side, and
this one measures the reason and names the two levers that exist. Raised in
`DECISIONS.md`.

## A defect this unit's own procedure produced, recorded before it was fixed (C12)

`harness.postintegration.WRONG-TREE.json` printed
`POST-INTEGRATION PASS: checked 3354 failed 0` and was **not** taken against the
integrated build. The red-test runner's trap is `git checkout -- Controllers.f90`,
and at the moment it fired HEAD did not yet carry this unit's wrapper — so the
revert deleted the **wrapper** rather than the perturbation. Nothing failed and
nothing said so. The artifact is kept with that written into its own `notes`
field. The rule it violates already exists one artifact over: the *harness*
red-test runner's header says "COMMIT THE TRANSLATION FIRST". Nobody had written
the same sentence about the wrapper.

## The gate sees two of the three scenarios that call this unit, and the third is zero

Scenario 3 calls it 15,999 times and moved under neither real red test.
`gate.scenario3-probe.json` forces the value written to `avrSWAP` 1.0e5 away and
moves **15,999 of scenario 3's 16,000** samples: the site is fully observable.
What annihilates the perturbations is that scenario 3 runs to t = 400 while every
writer of `CC_DesiredL` is behind `IF (Time > 500)`, so the filter's input is
exactly 0.0 for the whole run and `1/a2 * (b2*0 + ... )` is 0.0 for every value of
the six coefficients. **Third instance in three units** of a five-figure hit count
that no perturbation reaches because a quantity is exactly 0.0 — and the first in
which the zero is not a configured gain but a state the scenario's run length
never lets the unit write.

## The gate's resolution on this unit is single precision

Everything this unit sends to a gate channel goes through `avrSWAP`, which is
`REAL(ReKi) :: avrSWAP(*)` — REAL(4). PI moved in its twelfth significant digit
is a relative 3e-12, five orders below binary32's 1.2e-7 spacing, and it moves
**2 values in 5,252,000**. It is red and revert-verified and it is the wrong
instrument to certify this unit with, which is why the step constant — one part in
1,451 — is the one committed as `gate/CableControl.redtest.json`. Units #47 and
#48 reached REAL(8) channels and never had to know this.

## What each file is

| file | what it records |
|---|---|
| `cablecontrol.hstub-noop.cpp`, `run_harness_redtest.sh` | the no-op red test and its runner (the stub keeps three guarded, unreachable callee calls so the bridges are still emitted — unit #45's rule) |
| `harness.noop.json` | that red test: 3207 of 3354 |
| `run_mutation_part.sh` | one guarded, operator-filtered part |
| `mutation.first_take.*.json` | the first sweep, 3300 cases, 12 survivors, nothing declared |
| `mutation.census.txt` | all twelve first-take survivors with their dispositions, and the NaN finding in full |
| `errmsg_census.txt`, `cablecontrol.errmsg-census-probe.cpp` | the probe and the measurement that dispose the three standing survivors |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the wrapper red test, 3157 of 3354 |
| `harness.postintegration.WRONG-TREE.json` | the C12 artifact: a green about a tree nobody meant to measure |
| `gate.redtests.txt` | the three gate perturbations, their per-scenario channel lists, and the scenario-3 finding |
| `gate.scenario3-probe.json` | the probe that separates annihilation from blindness |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../harness/baseline.CableControl.json` | the three R11 states and why each value in them is what it is |
| `../../mutation/CableControl.equivalences.json` | the four declarations and their proofs |

## The findings worth carrying

1. **A zero-initialised nested type can put a NaN in front of every coefficient a
   callee has**, and the kill counts will still look like an ordinary corpus gap.
   The thing that decides it is the callee's own control flow, not the counts.
2. **A pass-through flag that is both the caller's `reset` and the callee's
   `reset` collapses two arms into one.** `restart` initialises the filter *and*
   sends `PIController` down the arm that reads no gain, so the only live
   configuration needs the *other* initialiser, `iStatus == 0`.
3. **A midpoint is not a mode — second unit.** `aviFAIL = 300` and
   `CC_Mode = -300` are why three mutants stand.
4. **A red test's revert can delete the thing under test.** `git checkout --` on
   an integrated source file needs the integration to be in HEAD, and no runner
   in this campaign said so.
5. **Know the WIDTH of the channel a gate red test has to move.** A REAL(4)
   narrowing costs about nine orders of resolution, and a perturbation sized for
   a REAL(8) channel reads as "the gate can barely see this unit".
6. **A write loop with two statements needs a subscript bound one lower**, and
   the ramp that reaches it needs a step of two.
