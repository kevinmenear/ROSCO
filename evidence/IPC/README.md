# Unit #53 — `IPC`

`rosco/controller/src/Controllers.f90:487-584` (clean, at `54dd134`).
Disposition **deferred**, on P12 and on P12 alone. Four layers, all four
red-tested, and the mutation score is **0.846 over 78 of 118 mutants** — the
remaining 40 are one operator that will not fit a foreground call, which is
arithmetic rather than judgement and is set out below.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/IPC.json`) | **63,888 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, all six callee bridges kept so each side runs one ColemanTransform, one ColemanTransformInverse, one LPFilter, one PIController, one sigma and one wrap_360 — this unit's primary evidence | the unit as a no-op: **63,888 of 63,888**, the whole corpus, the empty pass set the stub predicted in its own header |
| mutation (`mutation/IPC.clean.*.json`, six parts) | **66 of 78 killed, 0.846**, 0 no-compile, 12 survivors, all at TWO sites | the score *is* the red test, 66 times; and the survivors are probed by the gate, below |
| post-integration (`harness/IPC.postintegration.json`) | 63,888 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **63,888 of 63,888**, an EMPTY pass set; reverted, rebuilt, green re-taken at 0, revert checked both ways |
| gate, 27 scenarios (`gate/IPC.json`) | 5,252,000 values / 351 channels, 0 mismatched | **THREE**: additive on the stored answer moves **334,388** across all five scenarios that call the unit; multiplicative on the same statement moves **201,604** across two; and the surviving `drop_call` mutant moves **159,758** |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for units #45 through #52, and it is
right here for this unit's own reason: three of its six callees carry
per-instance state indexed by an `objInst` counter they post-increment, and unit
#44 measured that a KGen kernel cannot replay SAVE-like state.

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
   because both spellings compile and both are plausible.

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

## The corpus is this campaign's largest, and that is the whole of P12's problem

**63,888 cases, 656 MB**, against `ForeAftDamping`'s 7,567 and `CheckInputs`'
16,769. Measured cost per mutant, from five parts that agree to within 2%:

```
index_offset   18 in 476 s     arith_op   17 in 453 s     swap_operands 15 in 400 s
compare_op     13 in 358 s     negate_cond 9 in 264 s     -> 25.5 s per mutant
```

`const_tweak` produces **40** mutants, so that operator is a **1040 s foreground
call against a 600 s ceiling**. It cannot be split:

* `vit_mutate.py --limit` truncates the operator-filtered list and has **no
  offset**, so mutants 21..40 are not addressable.
* `scripts/_mutation_merge.py` correctly refuses a part that scored a subset of
  its own operator's sites — *"each part's mutant total must equal the number of
  mutants its operators produce"* — and refuses a union missing an operator. Run
  rather than asserted: `mutation.merge_refusal.txt` is the refusal.

So there is **no `mutation/IPC.json`**, P12 cannot be satisfied, and the unit is
`deferred` on that one predicate.

**A narrowing was tried as a way out and is kept because it priced itself at
zero.** `LocalVar_NumBl = { values = [3, 0] }` gives 42,694 cases and 19.4 s per
mutant — not enough (const_tweak would still be 796 s), and `values = [3]` would
not have been either, because 40 compiles alone are ~180 s.
`mutation.index.numbl_2value.json` is the `index_offset` population scored on
that corpus: **14 of 18, the same four survivors by id**, identical to
`mutation/IPC.clean.index.json` on the 63,888-case corpus. So the two-value list
costs nothing measurable here and buys nothing either; the four-value list is
kept because it is strictly more input. That is unit #52's lever priced in the
other direction, and it is the second data point on it.

## The twelve survivors, classified — and eight of them are not equivalences

| site | mutants | classification |
|---|---|---|
| `ipc.cpp:170-173`, the four fixed-size local DECLARATIONS `double PitComIPC[3];` etc. | 4 (`index_offset '[3]' -> '[3 + 1]'`) | **(c)** — declaring a four-element array where three are used. The extra slot is never written and never read, and none of the four locals is compared: they are procedure locals, not fields. Behaviour-preserving everywhere. |
| `ipc.cpp:380-386`, the two `std::fmin` saturation statements | 8 (`drop_call` x2, `swap_call_args` x2, `arith_op` x2, `swap_operands` x1, `compare_op` x1) | **(a) — a CORPUS GAP, established by execution.** Not declared equivalent. |

**The second row is the unit's headline and it was not argued.** The surviving
`drop_call` mutant `b36f5d50` was run through the gate character for character
and **killed on 159,758 of 5,252,000 values**, scenarios 8 and 27
(`gate.redtest.fmin_dropcall.json`). A mutant killed by one instrument is not
equivalent, whatever the other instrument reports.

What the corpus can and cannot do at that site, from its own numbers:

```
swap_operands 'BlPitchCMeas - LocalVar->PC_MinPit'  (SatMode == 3)   KILLED
swap_operands 'BlPitchCMeas - CntrPar->PC_MinPit'   (SatMode == 2)   SURVIVED
```

Both arms **are** reached — one of the two swaps dies there. What the corpus does
not contain is a case in which `fmin`'s SECOND argument wins on the
`SatMode == 2` arm, so every edit that changes only that argument's value is
invisible. The gate supplies exactly that case. `swap_call_args` on a commutative
`fmin` would be a (c) equivalence on its own merits; it is left undeclared here
because it sits inside a site the corpus cannot see, and declaring an equivalence
inside a blind spot is how a real defect gets excused.

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
| `mutation.merge_refusal.txt` | `_mutation_merge.py` REFUSING the six parts because `const_tweak` is in none of them — run, not asserted, and the reason there is no `mutation/IPC.json` |
| `run_mutation_part.sh` | the guarded sweep runner, with the per-operator counts and the 600 s arithmetic in its header |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the copy-back red test, **63,888 of 63,888**, and the argument for perturbing the `localvariables` copy-back rather than either of the other two the wrapper carries |
| `gate.redtests.txt` | the three gate perturbations, the 22-file DISCON survey, the scenario-by-scenario reasons for the three exact zeros, and the drop_call cross-instrument result |
| `gate.redtest.{additive,multiplicative,fmin_dropcall}.json` | those three runs |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../harness/ranges.toml` | eight pins: four ALLOCATABLE extents at `{lo=2,hi=3}` (and why not `hi=2`), `LocalVar_NumBl` for the fifth time, three instance counters for the first unit that needs all three at once, and the `staging_capacity_excludes` window |

## The findings worth carrying

1. **A `{ lo = N, hi = N }` pin on an ALLOCATABLE extent silently DELETES that
   array from the comparison.** `fixed_extent` is defined as `lo == hi`. The run
   says so in its UNOBSERVABLE list and nowhere else; the harness still passes.
2. **Count the staged CHARACTER assignments, and note that the SHORTEST prefix
   sets the bottom of the window.** Unit #48 counted two; this unit has five, and
   the window starts at `m + len('IPC:')` rather than at `m + len('sigma:')`
   because the last gate is the cheapest.
3. **A mutant that survives one instrument and is killed by another is a corpus
   gap, and running it through the other instrument costs one gate run.** Eight
   of this unit's twelve survivors were about to be reasoned about; one 289 s red
   test settled all eight.
4. **An operator population can exceed one foreground call, and nothing in the
   toolchain can split it.** 40 mutants x 25.5 s against a 600 s ceiling, with no
   offset in `--limit`. Raised in DECISIONS.md.
