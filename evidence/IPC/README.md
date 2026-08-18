# Unit #53 — `IPC`

`rosco/controller/src/Controllers.f90:487-584` (clean, at `54dd134`).
Disposition **deferred**, on P12 and on P12 alone. Four layers, all four
red-tested, and the mutation score is **0.9231 — 96 killed of 104 behavioural,
14 declared equivalent, 8 open**.

**This is the THIRD dispatch, and it exists because six survivors were named and
two of them were not equivalences.** The first produced every layer except a
mutation score (`const_tweak` would not fit a foreground call). The second
produced the score: **0.7797, 92 of 118**. This one asked what the six named
survivors actually were, and the answer split them in half:

| the driver named | what it turned out to be | what was done |
|---|---|---|
| `index_offset` ×4 — `'[3]' -> '[3 + 1]'` at the four local declarations | **(a) genuinely equivalent** | declared, with the argument in `mutation.equivalences.md`, and the four `const_tweak '3' -> '4'` twins at the same four lines declared with it |
| `drop_call` ×2 at the `IPC_SatMode`/`fmin` site | **(b) the corpus could not reach it** — and the gate had already proved it by killing one on 159,758 values | the INPUTS were fixed, not the record: `harness/ranges.toml` now makes the arm selector a flag. Both are KILLED, at 1,494 and 1,453 cases |

The corpus fix took **seven** of the nine mutants at that site with it. The
score moved `0.7797 (92/118) -> 0.8136 (96/118) -> 0.9231 (96/104)`, the first
step from kills and the second from declarations.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/IPC.json`) | **84,754 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, all six callee bridges kept so each side runs one ColemanTransform, one ColemanTransformInverse, one LPFilter, one PIController, one sigma and one wrap_360 — this unit's primary evidence | the unit as a no-op: **84,754 of 84,754**, the whole corpus, the empty pass set the stub predicted in its own header |
| mutation (`mutation/IPC.json`, thirteen parts) | **96 of 104, 0.9231**, 0 no-compile, 14 declared equivalent, 8 open, at four sites | the score *is* the red test, 96 times; `declared_but_killed` is empty, so no declaration excused a mutant the corpus would have killed |
| post-integration (`harness/IPC.postintegration.json`) | 84,754 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **84,754 of 84,754**, an EMPTY pass set; reverted, rebuilt, green re-taken at 0 (`harness/IPC.postintegration.revert-verified.json`) |
| gate, 27 scenarios (`gate/IPC.json`) | 5,252,000 values / 351 channels, 0 mismatched | **THREE**: additive on the stored answer moves **334,388** across all five scenarios that call the unit; multiplicative on the same statement moves **201,604** across two; and the then-surviving `drop_call` mutant moved **159,758** |

**THE GATE IS NOT RE-TAKEN, THE CLAIM IS CHECKED TWO WAYS, AND IT WAS THEN RUN
ANYWAY AS A ROUND-TRIP CONTROL.** A corpus change
re-takes every layer that reads the corpus and exactly those. The gate runs 27
simulations against the shipped library, never reads the case file, and neither
the translation nor the wrapper moved this dispatch — `translations/Controllers/ipc.cpp`
is byte-for-byte what the first dispatch verified, `811d6842`. What WAS re-taken:
the clean-tree green, its no-op red test, all thirteen mutation parts and their
merge, the post-integration green, its red test, and the green after that red
test's revert.

The claim that the gate could not have moved is not an argument here: `git diff`
between the commit that wrote `gate/IPC.json` and HEAD over
`translations/Controllers/ipc.cpp`, `rosco/controller/src/Controllers.f90` and
`rosco/controller/CMakeLists.txt` is EMPTY. And because this dispatch opened
three reset/restore windows and rebuilt the library in each, the gate was then
run once more as a control on the round trip:

```
GATE PASS: compared 5252000 value(s) across 351 channel(s) / 27 scenario(s); mismatched 0
wrote gate/IPC.json          144 s
git status gate/             (nothing)
```

**Byte for byte.** So the tree this dispatch leaves behind builds the library the
gate artifact describes, and `reset_to_clean.sh` / `restore_integrated.sh`
round-tripped three times without moving a compared value.

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for units #45 through #52, and it is
right here for this unit's own reason: three of its six callees carry
per-instance state indexed by an `objInst` counter they post-increment, and unit
#44 measured that a KGen kernel cannot replay SAVE-like state.

## The corpus change, which is this dispatch's whole functional result

**COUNT THE CASES THAT REACH THE SITE BEFORE ARGUING ABOUT THE SITE.**
`probe_fmin_site.py` generates the corpus, stops before `emit`, and reports what
happens at the `IPC_SatMode` split. It costs 30 seconds and no build, which is
why the design was iterated on it four times instead of on a 70-minute sweep.

```
                        arm 2 reached   2nd wins   arm 3 reached   2nd wins   else
before                       4608           0          4608           0      54672
values = [2, 3]             42525        2171         42229        2197          0   <- deletes an ARM
values = [2, 3, 0]  (kept)  28446        1494         28164        1453      28144
```

Both arms were reached BEFORE — that is the positive control, and it is what
makes this a corpus gap rather than a dead site. The saturation simply never bit
in 9,216 chances, because `IPC_SatMode` is an INTEGER the reference compares
against literals, so R7's predicate knob was the only stage that ever put 2 or 3
in it — and in the knob stage every REAL sits at its base draw. The site was
reached 9,216 times at ONE triple of real values.

`values` makes it one of R2's flags, which `flag_variants` crosses into the real
ladder, R6's rungs, the magnitude ladder, the negative-zero stage and R13's
capacity block. **The predicted kills landed to the case:**

```
drop_call     b36f5d50  1494      452847c1  1453        <- the two the driver named
arith_op      1da84c7a  2010      e63a2311  2512
swap_operands 069907e7   979
const_tweak   6058839b  1804 (== 2 -> == 3)   95e20ce8  1453 (== 3 -> == 4)
```

`answer_differs = 621` predicted the last pair; those two are transcription
decision 3 below, and the sweep now says in numbers that the harness checks it.

### Two corrections the run forced, both measured

**1. A stated `values` list DOES bound R7's knob.** The first draft wrote
`values = [2, 3]` on unit #49's note that it does not, reasoning that the
fall-through would still arrive from the knob cross-product. One probe run said
`else: 0`: `vit_harness.narrow_knobs` intersects every knob with the declared
values, so the two-value list DELETED one of this unit's eight arms — and the
harness would have gone green over it. **Every arm a unit has must appear in the
list, or the list deletes it.**

**2. `harness/emit.py` has a memory ceiling between 84,754 and 95,310 cases.**
R2's stratification is the SUM of the flags' arities. Three flags at 4 + 2 + 3
is 95,310 cases and 1.0 GB, and emit was SIGKILLed (137) writing it; nothing
else in the VM was holding memory (the two sibling containers were at 372 KB and
412 KB). So one list had to give up a value, and `LocalVar_NumBl` gave up 2 —
0 (no trips), 1 (one trip) and 3 (what ROSCO ships, and the extent of every
array the loop subscripts) remain, and 2 is the only one of the four bracketed
on both sides by values that stay. `narrow_knobs` means it is gone from the
corpus entirely, not merely from the flag cross.

### And it was not free: two kills were lost, and the reason is worse than the loss

`index_offset 7512a165` and `f7f90ca1` — `LocalVar%IPC_KI(1)` at the two 1P
`PIController` calls — were killed on the 63,888-case corpus and survive on this
one. The first explanation written down was that this dispatch's change
shadowed them; **the probe refutes it.** Of the 17,486 cases that RUN the 1P
block:

```
IPC_IntSat < 0   17474     the saturation pair is INVERTED: -IntSat > +IntSat,
IPC_IntSat = 0       0     so saturate(x, lo, hi) = fmin(fmax(x, lo), hi) = hi
IPC_IntSat > 0      12     for EVERY x, and the gains are unobservable
```

and that was already true before, because `CntrPar%IPC_IntSat`'s own base draw
is negative and 96.5% of cases take `IPC_IntSat` straight from it either way.
**Twelve cases in 84,754 can see a gain at that site at all.** The three
siblings on `IPC_KP(1)` are killed by ONE case each. So which of the four
mutants there gets its one case is a redraw lottery, and the corpus change
merely re-rolled it.

That is the unit's largest remaining blind spot and it is bigger than the two
mutants that exposed it.

**THE FIX IS NAMED, AND — BECAUSE NAMING IT COSTS NOTHING AND THE NEXT DISPATCH
WOULD HAVE TO GUESS — IT IS ALSO MEASURED.** `probe_fmin_site.py` was run once
more with `CntrPar_IPC_IntSat = { lo = 0, hi = 1e3 }` added, and the entry was
then REVERTED without being applied:

```
                        cases    block_runs   IntSat<0   ==0    IntSat>0
committed (no pin)      84754        17486      17474      0          12
with the pin            83754        17478         13     14       17451
```

**It works, and it is CHEAPER than the corpus that is committed** — 83,754
against 84,754 cases, because the pin also takes `CntrPar_IPC_IntSat` out of the
magnitude ladder and the negative-zero stage, which is a thousand cases. The
`fmin` site improves as well rather than regressing: arm 2's second argument
wins 3,024 times against 1,494 and arm 3's 2,564 against 1,453, and all three
`IPC_SatMode` arms survive (28,108 / 27,828 / 27,818). `equal_diff_bits` stays
**0**, so it does not reach the two `swap_call_args`.

**IT IS NOT APPLIED HERE, and the reason is unit #26's rule rather than the
clock.** `harness/IPC.json` and `mutation/IPC.json` were taken on the
84,754-case corpus. Committing a `ranges.toml` that produces a different one,
without re-taking both, would leave the file describing a corpus no artifact was
ever measured on — the exact failure the corpus hash exists to catch. Applying
it means the whole cycle again: green, no-op red, thirteen parts, merge,
post-integration green/red/green. Priced at **~95 minutes** against the **78**
this dispatch had left when the measurement came back. Taking the refusal, and
leaving the next dispatch a number instead of a hypothesis.

## What this unit is

Individual pitch control: two Coleman transforms of the blade root moments into
tilt/yaw axes at 1P and 2P, an optional yaw-by-IPC contribution, a soft cut-in
ramp on the gains, a three-way saturation split, one to four PI integrators, two
inverse Coleman transforms and a per-blade summation with an optional filter.
Five arguments, six distinct callees, **eight arms**, and fourteen
`LocalVariables` outputs.

Three transcription decisions are worth knowing before reading the C++:

1. **The yaw-error `LPFilter` at `:516` is handed `objInst%instSecLPF`, the
   SECOND-order counter**, although `LPFilter` is the first-order filter and
   every other call site in ROSCO passes it `instLPF`. Transcribed as written
   (P7): correcting it would move every filter instance in the process by one.
2. **`LocalVar%axisYawF_2P` is read at `:548` and written by nothing in ROSCO.**
   Grepped across `rosco/controller/src/*.f90`: it appears at `:548` and in
   `ROSCO_IO`'s state serialisation, nowhere else. `axisYawF_1P` has exactly one
   writer — the ELSE arm at `:519`, whose condition is the same one that guards
   its only reader, so that field is safe. The 2P one is an undefined read in
   the REFERENCE and the translation reproduces it by reading the same field.
3. **The two `IPC_SatMode` arms differ in ONE field**: `CntrPar%PC_MinPit` on
   arm 2 and `LocalVar%PC_MinPit` on arm 3. Two identically shaped statements
   reading two different fields with the same name in different types — the one
   transcription error here that staring at the expression does not find,
   because both spellings compile and both are plausible. **The mutation sweep
   has since measured that the corpus cannot tell those two fields apart**
   (§"The eight open survivors"), so this decision rests on reading, not on the harness.

## The mutation layer, and the two tool defects it took to get it

**`vit_mutate.py --offset` (loop `4751161`).** `--limit` truncates the
operator-filtered list, so the tail of an oversized operator was unreachable.
`--offset` skips before `--limit` truncates, so the pair names any window; the
window lands in the artifact as `mutant_slice` and the ids scored as
`scored_ids`. `const_tweak` here is three slices: `[0,14)`, `[14,27)`,
`[27,40)`.

**`_mutation_merge.py` checks the slices are a PARTITION, by id.** The old rule
was one operator, one part — which enforced disjointness by construction because
nothing in the artifact could express any other. It is now: no id in two parts,
every id of every named operator in some part, no id the mutator does not
produce. That is strictly stronger than the count arithmetic it replaces, and it
was seen to fail before it was believed (`mutation.merge_controls.txt`, X4):

```
the middle slice dropped   REFUSED  13 mutant(s) ... in NO part (const_tweak:13)
one slice supplied twice   REFUSED  mutant fd776540 was scored by both ...
a whole operator dropped   REFUSED  operator(s) ['negate_cond'] ... in NO part
```

**AND THE NUMBER IS OVER A SAMPLE OF ONE OPERATOR, WHICH THE ARTIFACT NOW SAYS.**
`cppmutate.mutants` caps every operator at 40. IPC's `const_tweak` has **79**
sites. So **118 is the population the mutator OFFERED and 157 is the population
the translation HAS**, and 39 const_tweak mutants were never compiled or run —
their survival is UNKNOWN and not 'none' (P6). `capped_operators:
["const_tweak"]` is now in every part and in the merged artifact.

This was not an IPC problem. `cppmutate`'s own docstring says "callers must
report when it bites"; `scripts/dbgmutate.py` did, `scripts/vit_mutate.py` never
had. **15 of 55 translations have an operator at or over the cap and 1,856
mutants campaign-wide were never enumerated** — `mutation.cap_audit.txt`, from
`scripts/mutation_cap_audit.py`. CheckInputs reports 192 mutants against 848
sites. Recorded as C12, with the wrong artifacts still in the tree, and raised
in DECISIONS.md as a campaign decision rather than this unit's.

## The eight open survivors, at four sites — and none of them declared

`equivalent_declared` is **14** and every one of the fourteen has its argument
written out in `mutation.equivalences.md`, where it can be disputed. What
follows is the other list: the mutants that are still alive and are NOT claimed
to be equivalent.

| site | mutants | what the corpus cannot do |
|---|---|---|
| `ipc.cpp:381, 384` — the two `std::fmin` saturations | **2** (`swap_call_args`) | **(c) — a blind spot no rule covers.** `fmin(a, b)` and `fmin(b, a)` differ only where the two compare EQUAL with different bit patterns, because glibc's `fmin` returns its FIRST argument on a tie: `fmin(+0.0, -0.0)` is `+0.0` and the swap is `-0.0`. `equal_diff_bits` is **0** before and after the corpus change. Reaching it needs `CntrPar%IPC_IntSat` and `BlPitchCMeas - PC_MinPit` to be zeros of OPPOSITE SIGN in the SAME case, and the negative-zero stage sets ONE real at a time. **No entry in `ranges.toml` can state a coincidence between two parameters' values** — `implied_by` states a relation on integers, not a bit pattern. This is a new category and it is escalated in DECISIONS.md. |
| `ipc.cpp:434, 438` — `LocalVar%IPC_KI(1)` at the two 1P `PIController` calls | **3** (`index_offset` ×2, `const_tweak` ×1) | **(b)**, and the size of the gap is measured above: **12 of the 17,486 cases that run this block have a live saturation window.** The other 17,474 arrive with `IPC_IntSat < 0`, which inverts the pair and pins `saturate()` to `maxValue` for every input. Two of the three were killed on the previous corpus; the siblings on `IPC_KP(1)` are killed by ONE case each, so the site is a lottery at this width. The fix is named above and is not taken. |
| `ipc.cpp:132-133` — `errmsg_trim` | **3** (`const_tweak '0'->'1'` ×2, `'1'->'2'`) | **(b), one gap seen three times.** `n > 1` needs an ErrMsg of length exactly 1 with a non-blank first byte; `: 1` needs a non-positive length with a non-blank byte 0; `+ 2` needs a TRAILING BLANK — and all three need `aviFAIL < 0` in the SAME case, because that is the only path that reaches `errmsg_trim` at all. R6's character stage runs the string at lengths [1, 2, 7, 12] and in the padded shape, and holds `aviFAIL` at its base draw; R7's knob puts -1 in `aviFAIL` and holds the string at ITS base. **The two ladders never cross** — which is the same shape as the `IPC_SatMode` gap this dispatch closed, one parameter over. The same fix applies: `ErrVar_aviFAIL = { values = [-1, 0, 1] }`, unit #49's own entry. **It is priced and refused**: a fourth flag at arity 3 takes the sum of arities from 8 to 11 and the corpus from 84,754 to about 116,000, and `emit` was already killed at 95,310. |

**The four rows sum to 2 + 3 + 3 = 8, which is `survived` in `mutation/IPC.json`**,
and the ids were checked by set difference against the artifact rather than by
adding the column up.

**`swap_call_args` WOULD have been the easy declaration** — a commutative `fmin`
— and it is left open because `fmin` is not commutative at a signed zero and the
campaign compares bit patterns. Unit #21 measured that `fmin`/`fmax` agree with
gfortran's intrinsic on all 12,167 triples *in the argument order written*;
nothing measured the swapped order, and an equivalence nobody measured inside a
site the corpus cannot see is how a real defect gets excused.

## Two range-pin findings, and the first is a trap

### `{ lo = 2, hi = 2 }` on an ALLOCATABLE extent makes the array UNOBSERVABLE

The first `[IPC]` block pinned the four `CntrPar` ALLOCATABLE extents to exactly
2 — the value ROSCO's own reader allocates. The run said:

```
UNOBSERVABLE CntrPar.IPC_Vramp: the bridge and the C struct disagree about its
  shape (no member not in the struct); supplied to Fortran as a zeroed buffer
  and NOT compared
UNOBSERVABLE CntrPar.IPC_KP: ...
UNOBSERVABLE CntrPar.IPC_KI: ...
UNOBSERVABLE CntrPar.IPC_aziOffset: ...
```

`Param.fixed_extent` is **defined** as `lo == hi` (`harness/signature.py:83`) and
means "an extent the TYPE fixes, not one the caller chooses", so `emit.py` laid
all four out as fixed C struct members while the real view struct carries a
pointer and a count. Four of this unit's inputs were zeroed and dropped from the
comparison, silently, by a pin written to protect them.

`{ lo = 2, hi = 3 }` is the same narrowing without the collision: extent 3 is
admissible because the unit reads elements 1 and 2 and nothing else, and the
plan's own notes say the indices were never exercised by R5 anyway
(`CntrPar_IPC_Vramp[0]: subscript not traceable to a parameter`, six times).

**The general shape:** a bounds pin whose two ends are equal is not the same
kind of statement as one whose ends differ. It is worth checking any pin of the
form `{ lo = N, hi = N }` on an ALLOCATABLE against the run's UNOBSERVABLE list.

### The R13 staging window was predicted from two lengths and measured to the case

`evidence/IPC/harness.staging_composition.{json,txt}` keeps the red artifact and
decodes every diff. This unit has **five staged assignments on one path**, the
campaign's longest: four `sigma` calls each prefixing `'sigma:'` and then its own
`'IPC:'`. With the R13 base draw `m = 7`:

```
L_final = 7 + 4*6 + 4 = 35        the reference's own answer
L_first = 7 + 4      = 11         'IPC:' is FOUR bytes and 'sigma:' is SIX,
                                  so the LAST gate is the cheapest and 11
                                  rather than 13 is the bottom of the window
   -> failing window [11, 35), 24 capacities, k = 4..27
```

Measured: `failed 24`, cases 63660..63683. The decoded `got` column climbs the
chain one gate at a time (`IPC:abyz{9:` at 11, `sigma:abyz{9:` at 13,
`IPC:sigma:abyz{9:` at 17, …). Third unit at this shape after `AeroDynTorque`
(two gates, `[16, 29]`) and `PitchSaturation` (ablated R13 instead). The entry is
strictly more input than the ablation it replaces: `--disable
R13_staging_capacity` drops all 256 rungs to avoid 24.

*(That artifact is the one file here still stamped at `d947d92`: it is a
red-test decoding of a window whose boundaries the byte-identical corpus fixes,
so re-taking it would reproduce it by construction. It is named rather than
quietly left.)*

## The corpus is this campaign's largest, and that is what shaped the sweep

**84,754 cases, 870 MB**, against `ForeAftDamping`'s 7,567 and `CheckInputs`'
16,769 — and 656 MB / 63,888 cases before this dispatch widened it. Measured
cost, thirteen parts:

```
118 mutants, ~34.5 s each, 4,070 s total against a 600 s foreground ceiling
const_tweak  40 in 4 slices    index_offset  18 in 2      arith_op  17 in 2
swap_operands 15 in 2          compare_op    13           negate_cond  9
calls          6
```

`mutation/IPC.clean.{index,arith,swap}.json` were DELETED with this dispatch:
they are parts of the 63,888-case sweep and would otherwise sit beside the new
ones under confusable names. `mutation.index.numbl_2value.json` is kept and is
still the nearest measurement of what dropping `NumBl` values costs — the whole
`index_offset` population scored 14 of 18 with the SAME FOUR SURVIVOR IDS on the
two-value list, which is the evidence the `[3, 0, 1]` trade rests on.

**AND ONE THING THAT DID NOT SHOW UP IN `git status` AND SHOULD HAVE.**
`mutation/IPC.clean.compare.json` and `.negate.json` came back byte-identical to
their predecessors — 12/1 and 9/0 both times — because **a mutation part
artifact does not record the corpus it was scored against.** It carries
`loop_rev`, `vit_rev`, `gen_rev`, the operator filter and the slice, and nothing
that identifies the case file. Two parts from two different corpora are
indistinguishable, and `revcheck` cannot see the difference either. Both parts
WERE re-run. Raised in DECISIONS.md.

## The gate is green and it constrains three of five scenarios not at all

Full detail in `gate.redtests.txt`. In one line: the additive/multiplicative pair
on the same statement says `LocalVar%IPC_PitComF` is **exactly 0.0** in scenarios
2, 6 and 18, and the baseline arrays say it independently — all three
`bld_pitch` channels are identical in exactly those three scenarios and differ in
the other two. Scenario 18 is the one that could not be read off
`Examples/DISCON*.IN`: it runs `IPC_ControlMode == 2` with non-zero gains and
still computes zero, because its 1-DOF sim has zero blade root moments. **The 2P
arm is inside the gate's window and the gate cannot tell what it computed** —
unit #46's finding one level up.

Four arms have **zero hits in all 27 scenarios** and the differential harness is
the only instrument that reaches them: `IPC_SatMode == 3`, the `IPC_SatMode`
fall-through, the `IPC_CornerFreqAct > 0` per-blade filter, and the `'IPC:'`
error prefix. All 22 `Examples/DISCON*.IN` set `IPC_SatMode = 2` and
`IPC_CornerFreqAct = 0.0`.

## What each file is

| file | what it records |
|---|---|
| `ipc.hstub-noop.cpp`, `run_harness_redtest.sh` | the no-op red test and its runner (the stub keeps all six `<callee>_c` calls behind an unsatisfiable guard so the bridges are still emitted — unit #45's rule; the runner asserts the translation is in HEAD before arming a trap that would delete it) |
| `harness.staging_composition.{json,txt}` | the 24-case R13 red, and its decoding: the predicted window, the ladder index, and the five gates visible one at a time in the `got` column |
| `mutation.index.numbl_2value.json` | the `index_offset` population scored on the 42,694-case corpus the two-value NumBl list produces — 14 of 18, the same four survivor ids, which is what prices that lever at zero |
| `mutation.merge_controls.txt` | the union's own refusals, run against this unit's real parts: a dropped slice, a duplicated slice, a dropped operator. X4 — the check was seen to fail before the artifact it wrote was believed |
| `mutation.merge_refusal.txt` | the FIRST dispatch's refusal, kept: `_mutation_merge.py` rejecting six parts because `const_tweak` was in none of them. It is why there was no score, and it is the artifact the fix is measured against |
| `mutation.cap_audit.{txt,json}` | 15 of 55 translations have an operator over `cppmutate`'s cap of 40; 1,856 mutants campaign-wide were never enumerated. From `scripts/mutation_cap_audit.py` |
| `run_mutation_part.sh` | the guarded sweep runner, now taking `--offset`/`--limit` |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the copy-back red test, **84,754 of 84,754**, and the argument for perturbing the `localvariables` copy-back rather than either of the other two the wrapper carries |
| `gate.redtests.txt` | the three gate perturbations, the 22-file DISCON survey, the scenario-by-scenario reasons for the three exact zeros, and the drop_call cross-instrument result |
| `gate.redtest.{additive,multiplicative,fmin_dropcall}.json` | those three runs, all three re-taken at the current instrument |
| `probe_fmin_site.{py,json}` | the 30-second generate-but-do-not-emit probe that designed the corpus change and then refuted this dispatch's own first explanation of the two lost kills. It is the cheapest instrument in this unit's evidence and it decided every judgement above |
| `mutation.equivalences.md`, `../../mutation/IPC.equivalences.json` | the fourteen declarations: the id list the tool takes, and the reasons it does not carry. Two of the five groups were settled by READING the generated bridge |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../harness/ranges.toml` | nine pins: four ALLOCATABLE extents at `{lo=2,hi=3}` (and why not `hi=2`), `LocalVar_NumBl` for the fifth time and now at three values rather than four, **`CntrPar_IPC_SatMode` made a flag** (this dispatch), three instance counters for the first unit that needs all three at once, and the `staging_capacity_excludes` window |

## Why the disposition is `deferred`

`min_mutation_score` is **1.0** and this unit scores **0.9231**. P12 fails on
that number and on nothing else; the other thirteen predicates pass.

Reaching 1.0 from here means killing eight mutants, and each of the three groups
has a named reason it is not killed:

* the two `swap_call_args` need an input NO RULE IN THE GENERATOR CAN PRODUCE —
  two independently drawn parameters made equal with opposite zero signs. That
  is a category, not a range, and it is escalated.
* the three `IPC_KI(1)` mutants need `CntrPar_IPC_IntSat` narrowed to a positive
  interval, which deletes half of an admissible domain and that parameter's
  magnitude ladder, and costs a 70-minute re-sweep.
* the three `errmsg_trim` mutants need `ErrVar_aviFAIL` made a flag, which
  `emit` cannot hold: 84,754 cases is already inside a ceiling measured at
  between 84,754 and 95,310.

**None of the eight is declared equivalent**, and the distinction from the
fourteen that are is the whole content of this dispatch: `drop_call` looked
exactly like the `swap_call_args` beside it and was not equivalent at all, which
the gate had proved and the corpus then reproduced at 1,494 cases.

## The findings worth carrying

1. **A `{ lo = N, hi = N }` pin on an ALLOCATABLE extent silently DELETES that
   array from the comparison.** `fixed_extent` is defined as `lo == hi`. The run
   says so in its UNOBSERVABLE list and nowhere else; the harness still passes.
2. **Count the staged CHARACTER assignments, and note that the SHORTEST prefix
   sets the bottom of the window.** Unit #48 counted two; this unit has five, and
   the window starts at `m + len('IPC:')` rather than at `m + len('sigma:')`
   because the last gate is the cheapest.
3. **A mutant that survives one instrument and is killed by another is a corpus
   gap, and running it through the other instrument costs one gate run.** Nine
   of this unit's survivors were about to be reasoned about; one 291 s gate red
   test settled all nine — and the dispatch after it turned seven of the nine
   into kills by fixing the INPUTS, which is what "corpus gap" is supposed to
   mean and what "declare it equivalent" would have prevented forever.
4. **An operator population can exceed one foreground call, and now it splits.**
   40 mutants × 26.5 s against a 600 s ceiling. `--offset` plus a union checked
   as a partition by id. The first dispatch's refusal is kept beside the fix.
5. **A silent cap reads as full coverage, and one of the two mutators had been
   saying so all along.** 1,856 mutants campaign-wide, 15 translations. The fix
   was a copy of `dbgmutate.py`'s two fields, not a new idea.
6. **A re-take across instruments is priced by one hash.** If the regenerated
   corpus is byte-identical, every count in the old evidence survives the move
   and the re-take is 71 s. If it is not, that is the finding. Do not argue it
   from which rules are N/A — take the hash.
7. **A generate-but-do-not-emit probe costs 30 seconds and decides everything a
   70-minute sweep would otherwise decide by accident.** `emit` writes 870 MB and
   builds; `generate` is where the whole corpus decision is made. Monkeypatching
   the name `vit_harness` imported `generate` under, counting, and exiting is
   four lines. This unit's corpus design was iterated FOUR times on it, one of
   those iterations caught a deleted ARM, and the same probe then refuted the
   author's own written explanation of a regression.
8. **A stated `values` list bounds R7's predicate knob, so it can DELETE an arm.**
   Measured: `IPC_SatMode = { values = [2, 3] }` gave `else: 0` on 84,754 cases
   and the harness stayed green. Every arm a unit has must appear in the list.
9. **The corpus has a memory ceiling and it is reached before the case count
   looks alarming.** `emit` SIGKILLed at 95,310 cases / 1.0 GB in a 7.7 GiB VM
   holding nothing else. R2's flag stratification is the SUM of the arities, so
   the ceiling is spent one flag value at a time and the last one bought is the
   one that has to be argued for.
10. **A mutation part artifact does not name its corpus.** Two of this unit's
   thirteen parts came back byte-identical to parts scored on a corpus 25%
   smaller, so `git status` reported them as not re-run. Nothing in the artifact,
   and nothing in `revcheck`, can tell those two situations apart.
