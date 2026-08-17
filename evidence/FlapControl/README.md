# Unit #50 — `FlapControl`

`rosco/controller/src/Controllers.f90:639-710` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested, and the mutation
score is **1.000** against a threshold of 1.000.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/FlapControl.json`) | **9721 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, all five callee bridges kept so both sides run one `ColemanTransform`, one `ColemanTransformInverse`, one `PIController`, one `PIIController` and one `saturate` — this unit's primary evidence | the unit as a no-op: **7292 of 9721**, the same corpus count, byte-identical case file either side |
| mutation (`mutation/FlapControl.json`) | **88 of 88 scoreable, 1.0000**, 9 declared, 5 no-compile, 6 operators, **no survivor, `declared_but_killed` empty** | the score *is* the red test, 88 times |
| post-integration (`harness/FlapControl.postintegration.json`) | 9721 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **4862 of 9721**; reverted, rebuilt, green re-taken at 0, and the revert checked both ways (`git diff` clean AND 3 `flapcontrol_c` references still present) |
| gate, 27 scenarios (`gate/FlapControl.json`) | 5,252,000 values / 351 channels, 0 mismatched | **TWO that land and one that does not**: the unit's own `avrSWAP` channel moves **75,995** across all five scenarios that call it; `NP_1` moves **79,992** in the two mode-3 scenarios; `R2D` moves **0**, and a fourth run explains why |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for units #45 through #49.

## What this unit is

Four arms in ONE `ELSEIF` chain, and the chain is headed by `IF (LocalVar%iStatus == 0)`
rather than by a mode test — so the initialisation arm **pre-empts** all three
mode arms. Writing the chain mode-first would be the obvious shape and it is the
wrong one. After the chain, three fixed `avrSWAP` slots (120, 121, 122) are
written on every arm, and the whole body is wrapped in `IF (CntrPar%Flp_Mode > 0)`
with an `ELSE RETURN`.

Three things are worth knowing before reading the translation:

1. **`0 - x` and `-x` both appear, three tokens apart, on one statement**
   (`PIIController(-LocalVar%rootMOOPF(K), 0 - LocalVar%Flp_Angle(K), ...)`).
   They agree on every value except a zero, where the first flips the sign bit
   and the second clears it. This campaign's harness compares bit patterns and
   the gate found a `-0.0` this way in scenario 16. Both are transcribed as
   written.
2. **Two different `Flp_Angle`s.** `CntrPar%Flp_Angle` is a REAL(DbKi) scalar
   parameter; `LocalVar%Flp_Angle` is the REAL(DbKi)(3) per-blade state. The view
   structs keep both, with different C types, so a confusion here would not
   compile — which is worth knowing, because it means no test is carrying that
   risk.
3. **The reference reads a local it never writes.** See below; it is this unit's
   whole story.

## The finding: an undefined read, and the eighth judgement kind

```fortran
Controllers.f90:653   REAL(DbKi) :: RootMyb_VelErr(3)     ! declared, never assigned
Controllers.f90:670   LocalVar%Flp_Angle(K) = PIIController(RootMyb_VelErr(K), ...)
```

`grep -rn RootMyb_VelErr` over ROSCO finds the declaration and that one use.
Nothing assigns it. In Fortran the value is whatever the stack slot holds; no
translation can reproduce it and none should try (P7), so it is transcribed as
an equally uninitialised local.

**THE FIRST HARNESS TAKE WAS RED AT 12 OF 9721**, every mismatch on
`LocalVar.piP` and nothing else. `evidence/FlapControl/undefined_read_census.txt`
reads those twelve cases' inputs with one `fprintf` and `--no-generate`, and
twelve of twelve are the same configuration:

```
Flp_Mode == 2  AND  iStatus == 0  AND  restart == 0  AND  NumBl >= 1
```

which is exactly the region in which that line executes on the branch that reads
its `error` argument. `PIIController` reads `error` only when `.NOT. reset`.

**THE SHIPPED PROGRAM CANNOT PRODUCE THAT PAIR.** `ReadAvrSWAP` runs at the top
of every DISCON call, before every controller, and ends with
`LocalVar%restart = (LocalVar%iStatus == 0)`. Nothing else in ROSCO assigns
`restart`. So `iStatus == 0` implies `restart`, the init arm always takes the
reset branch, and the undefined value is never read — which is why all five gate
scenarios run that line without depending on it.

**WHAT WAS DONE, AND THE THREE THINGS THAT WERE NOT.**

* **`no_oracle = "LocalVar_piP"`** would have been green on this corpus and is
  luck rather than a fix. All twelve failing cases hold `Flp_Kp = 0` (R6's
  isolating pin), so `kp*error` is 0 and the saturated sum hides the difference
  the integrator state still records. At any non-zero `Flp_Kp` the undefined
  value reaches `Flp_Angle` and `avrSWAP` too. It would also have cost the `piP`
  comparison on the 9709 cases where the reference does have an answer.
* **A pin on one of the four inputs** removes an arm with it: `iStatus /= 0`
  deletes the whole initialisation arm, `Flp_Mode /= 2` the PII arm, `NumBl = 0`
  both loops, and `restart` held true sends the mode-2 and mode-3 arms down the
  reset branch that reads neither gain.
* **Leaving the harness red** is not an option that exists: `vit_mutate.py`
  refuses to score against a red baseline, so a red primary layer takes the
  mutation layer with it.

So the relation itself is stated, through a new judgement kind:

```toml
LocalVar_restart = { implied_by = "LocalVar_iStatus", relation = "== 0",
                     reason = "..." }
```

`translation-loop@d947d92`. It is the **eighth** judgement kind and the first
that is about a PAIR of inputs rather than one input's values or one rule's
ladder — every earlier entry narrows a parameter independently, because that is
all `_case_impl` could express. It is applied after every stage, it is counted,
it appears in the coverage table as `STATED_implication` (`rewrote 4782 of 9721`),
and an implication that rewrites no case is an error.

**ADDITIVE, PROVED BY HASH RATHER THAN BY DIFF** (P5, unit #48's control):
`CableControl` states nothing, and its corpus is byte-identical either side of
the generator change — `b7a51f87…`, 7640 cases, harness re-passing 7640/0. 178
of 179 generator tests pass; the one failure is the container's missing `git`
binary and predates the change.

**THE COST, STATED.** `restart` stops being a free R2 flag: its two values now
track `iStatus`, so the pair `(iStatus /= 0, restart true)` leaves the corpus
along with `(iStatus == 0, restart false)`. Neither is producible by the program.
What IS lost is the ability to catch a translation that read `iStatus` where the
reference reads `restart`, or the reverse — this unit reads both, in different
places, and that confusion is now invisible here. It is not invisible to the
gate, which runs the program in which the implication holds; but the gate would
be bit-identical either way, so the honest statement is that **nothing in this
campaign can see it**.

## Nine survivors, two sites, one fact — and the argument was executed

| site | mutants | why unkillable |
|---|---|---|
| `flapcontrol.cpp:127` `double RootMyb_VelErr[3];` | 2 (`[3]`→`[3+1]`, `3`→`4`) | enlarging the declaration of an array that is never written and whose only read feeds a dead argument |
| `flapcontrol.cpp:213/214` the init-arm call's `error`, `error2` and `ki2` | 6 | `PIIController` reads them on the `.NOT. reset` branch only, and `reset` is true on every case that reaches this call |
| `flapcontrol.cpp:217` `restart ? 1 : 0` → `? 2 : 0` | 1 | the bridge converts with `(reset /= 0)`, so 2 and 1 are the same LOGICAL — behaviour-preserving everywhere, not merely unreachable |

**EXECUTED, NOT ARGUED** (unit #48's rule — its first model of the same kind of
proof was false and running it is what caught that).
`evidence/FlapControl/equivalence_probe.txt`:

```
cases=9721   iStatus==0:114   restart true:114   false:9607
Flp_Mode==2 & iStatus==0: 55
reaching :670  with reset: 37   WITHOUT reset: 0
```

37 is the positive control that the site is live; 0 is the whole equivalence
argument. And the counter-check that the operator is not blind: the **same**
`0.0 - Flp_Angle` → `0.0 + Flp_Angle` mutation at the mode-2 arm's call
(`flapcontrol.cpp:262`) was **killed** by this corpus. It is the site that is
dead, not the operator.

These are **(c)-class** in unit #48's vocabulary — blind spots no input can
close — and they are (c) for a reason that is a fact about the PROGRAM rather
than about the harness: the arguments are dead in ROSCO too.

## Four of the five scenarios that call this unit drive it on an exact zero

```
baseline_arrays/scenario_N.npz, flp_angle_1/2/3
  scenario  3   16,000 samples        0 non-zero    1 distinct
  scenario  4    4,000 samples        0 non-zero    1 distinct   <- the mode-2 arm
  scenario  7   24,000 samples        0 non-zero    1 distinct
  scenario 16   16,000 samples        0 non-zero    1 distinct   <- a mode-3 arm
  scenario 26   16,000 samples   15,999 non-zero  ~15,990 distinct
```

`R2D` scaled to 58.0 — 1.2%, four orders above the REAL(4) channel's resolution
— moves **0 of 5,252,000**, because scenario 4 sets `Flp_Angle = 0.0` and runs a
1-DOF simulation whose `rootMOOPF` is exactly zero, so `saturate(0.0, …) * R2D`
is 0.0 for every R2D. The gains are NOT the cause: `Flp_Kp = -0.001` and
`Flp_Ki = -0.0005` in that scenario's own patch.

**Fourth instance in four units** of a five-figure hit count that no perturbation
reaches because of an exact zero — #47's `AWC_amp` and #48's `WE_Gamma` were
configured gains, #49's was a state the run length never let the unit write, and
this one is the driving SIGNAL of a 1-DOF sim. `gate/FlapControl.mode2-probe.json`
is unit #48's separating probe: the same statement perturbed **additively**,
which an exact zero cannot annihilate, moving 11,997 values — all three flap
channels of scenario 4. So the arm is observable and the R2D zero is
annihilation, not blindness.

## What each file is

| file | what it records |
|---|---|
| `flapcontrol.hstub-noop.cpp`, `run_harness_redtest.sh` | the no-op red test and its runner (the stub keeps five guarded, unreachable callee calls so the bridges are still emitted — unit #45's rule; the runner asserts the translation is in HEAD before arming a trap that would delete it) |
| `harness.noop.json` | that red test: **7292 of 9721**. The 2429 that pass are the `Flp_Mode <= 0` set, predicted in the stub's own header |
| `run_mutation_part.sh` | one guarded, operator-filtered part |
| `mutation.first_take.*.json` | the four parts BEFORE any equivalence was declared, `equivalent_declared: 0` — what survived, on the record before it was excused |
| `undefined_read_census.txt` | the twelve red cases of the first harness take and their common configuration |
| `equivalence_probe.txt` | the executed argument behind eight of the nine declarations, with both controls |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the wrapper red test, **4862 of 9721** |
| `gate.redtests.txt` | the four gate perturbations, their per-scenario channel lists, and the zero-signal census |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../harness/ranges.toml` | four pins: `NumBl`, `instPI`, `Flp_Mode`'s stated domain, and the implication |
| `../../mutation/FlapControl.equivalences.json` | the nine declarations and their proofs |

## The findings worth carrying

1. **A reference can read a local it never writes, and the region where that
   matters can be a RELATION between two inputs rather than a range on one.**
   `harness/ranges.toml` had no spelling for that until this unit; it has one now.
2. **`no_oracle` on the output a defect happens to reach is not a fix for an
   undefined READ.** Which output it reaches depends on the corpus; the read
   does not.
3. **A base draw stated BEFORE the survivors rather than after them.**
   `CntrPar_Flp_Mode = { values = [2, 0, 1, 3] }` was written on units #47's and
   #49's finding, not on this unit's own measurement — `2` first because that is
   the deepest arm and R6's ladders are taken at the base draw. This is the first
   unit in the campaign to pay that cost in advance.
4. **A scale factor cannot be red-tested through an exact zero, and an additive
   perturbation at the same statement settles it in one run.**
