# Unit #53 — `IPC`

`rosco/controller/src/Controllers.f90:487-584` (clean, at `54dd134`).
Disposition **deferred**, on P12 and on P12 alone. Four layers, all four
red-tested, and the mutation score is **0.7797 — 92 killed of 118**, the first
complete sweep this unit has had.

**This is the second dispatch.** The first produced everything below except the
mutation score: `const_tweak` is 40 mutants at 26.5s on this unit's 63,888-case
corpus — 1,040s against the 600s a foreground command may block for — and
`vit_mutate.py --limit` could only TRUNCATE, so mutants 21..40 were not
addressable by any sequence of runs. The unit closed with no `mutation/IPC.json`
at all. That is fixed rather than worked around; §"What the second dispatch
changed" has it.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/IPC.json`) | **63,888 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, all six callee bridges kept so each side runs one ColemanTransform, one ColemanTransformInverse, one LPFilter, one PIController, one sigma and one wrap_360 — this unit's primary evidence | the unit as a no-op: **63,888 of 63,888**, the whole corpus, the empty pass set the stub predicted in its own header |
| mutation (`mutation/IPC.json`, nine parts) | **92 of 118 killed, 0.7797**, 0 no-compile, 26 survivors, at five sites | the score *is* the red test, 92 times; the union's own refusals are in `mutation.merge_controls.txt`; and the survivors are probed by the gate, below |
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
   because both spellings compile and both are plausible. **The mutation sweep
   has since measured that the corpus cannot tell those two fields apart**
   (§"The 26 survivors"), so this decision rests on reading, not on the harness.

## What the second dispatch changed

Every artifact was re-taken, because completing the mutation layer required it.

**WHY A RE-TAKE AND NOT ONE MORE PART.** The first dispatch's twelve artifacts
were stamped `(loop d947d92, vit a16a7ab)`. Both instruments have since moved —
the loop by five commits, VIT by two callee-bridge fixes. A new part carries the
new stamp, and `_mutation_merge.py` refuses parts from two instruments while
`revcheck.py` reports the same disagreement across the unit. So the choice was a
full re-take or no mutation number, and there was no third option that did not
involve rolling a shared repository backwards to make a stamp read a chosen
value.

**AND THE RE-TAKE WAS CHEAP BECAUSE OF ONE HASH.** `vit_harness.py` regenerates
the corpus; the concern is unit #26's rule, that a score and the green it rests
on must name the same corpus. Measured rather than argued:

```
before  e5909178b374dd940125a9e9717063eb80c0b01b16393e08b1edf4baddcd3ced
after   e5909178b374dd940125a9e9717063eb80c0b01b16393e08b1edf4baddcd3ced
        655,681,672 bytes, 63,888 cases, at d947d92 and at 4751161
```

**Byte-identical.** So every count in this file — the 24-case R13 staging window
at 63660..63683, the no-op red test's 63,888 of 63,888, the per-mutant cost — is
a statement about one corpus across both instruments. `gen_rev` DID change
(`gen-ffac3ef69dd5` → `gen-cf5dcfa09cce`), because it hashes the generator's
code and not its output; the corpus hash is the measurement and `gen_rev` is
not. The reason the output did not move is readable in this unit's own
artifact — `N/A R14_planted_word: no CHARACTER ARRAY input` — and
`no_oracle_when` emits nothing when no range states one. Both are arguments. The
hash is the check, and it could have gone the other way.

Every number re-taken came back **identical**: 63,888/0; 63,888 of 63,888 twice;
5,252,000/0; 334,388; 201,604; 159,758. The four non-const_tweak operators that
had been scored before returned the same survivors BY ID. So the instrument move
is visible in the stamps and nowhere in the results — which is what a re-take
that changes only the instrument should look like.

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

## The 26 survivors, at five sites

| site | mutants | what the corpus cannot do |
|---|---|---|
| `ipc.cpp:170-173` — the four fixed-size local DECLARATIONS `double PitComIPC[3];` etc. | **8** (`index_offset '[3]'->'[3 + 1]'` ×4, `const_tweak '3'->'4'` ×4) | **(c)** — declaring a four-element array where three are used. The extra slot is never written and never read, and none of the four locals is compared: they are procedure locals, not fields. Behaviour-preserving everywhere, on an argument that does not mention the corpus. |
| `ipc.cpp:380-385` — the `IPC_SatMode` split and its two `std::fmin` saturations | **9** (`drop_call` ×2, `swap_call_args` ×2, `arith_op` ×2, `swap_operands` ×1, `const_tweak` ×2) | **(a) — a CORPUS GAP, established by execution.** Not declared equivalent. See below. |
| `ipc.cpp:249, 276, 436` — `LocalVar->restart ? 1 : 0` at the three callee calls | **3** (`const_tweak '1'->'2'`) | **(c)**, and argued in the translation's own comment before these mutants existed: the generated wrapper writes `MERGE(1_C_INT, 0_C_INT, reset)` and the callee converts back with `(reset != 0)`, so the particular non-zero value is not observable on either tree. |
| `ipc.cpp:132-133` — `errmsg_trim`, the blank-trimming helper | **3** (`compare_op '>'->'>='`, `const_tweak '0'->'1'` ×2, `'1'->'2'`) | **(c)/(a) mixed.** `n >= 0` differs from `n > 0` only for a NEGATIVE length, which a length field cannot hold. `find_last_not_of(' ') + 2` appends one character that is a blank by construction, and Fortran CHARACTER assignment blank-pads, so a trailing blank is not observable through this boundary. The two `0 -> 1` need an ErrMsg of length exactly 0 or 1 carrying a non-blank first byte; R13 sweeps the staging CAPACITY, not that. |
| `ipc.cpp:250, 305` — `0, 0.0)` (LPFilter's optional pair) and `Y_MErrF = 0.0` | **2** (`const_tweak '0.0'->'1.0'`) | **(c)** — the first is the VALUE half of an optional-argument pair whose presence flag is the `0` beside it, so the value is never read; the second is the dead assignment the translation notes at the declaration, written by the reference and by this and read on no path. |
| `ipc.cpp:382` — one `swap_operands` and both `arith_op` at the saturation | (counted in the 9 above) | |

**The 380-385 row is the unit's headline and it was not argued.** The surviving
`drop_call` mutant `b36f5d50` was run through the gate character for character
and **killed on 159,758 of 5,252,000 values**, scenarios 8 and 27
(`gate.redtest.fmin_dropcall.json`, and `gate/IPC.redtest.json` is the same run).
A mutant killed by one instrument is not equivalent, whatever the other
instrument reports.

What the corpus can and cannot do at that site, from its own numbers:

```
swap_operands 'BlPitchCMeas - LocalVar->PC_MinPit'  (SatMode == 3)   KILLED
swap_operands 'BlPitchCMeas - CntrPar->PC_MinPit'   (SatMode == 2)   SURVIVED
```

Both arms **are** reached — one of the two swaps dies there. What the corpus does
not contain is a case in which `fmin`'s SECOND argument wins on the
`SatMode == 2` arm, so every edit that changes only that argument's value is
invisible. The gate supplies exactly that case. The two `const_tweak` survivors
newly visible here (`IPC_SatMode == 2` → `== 3`, `== 3` → `== 4`) are the SAME
blind spot seen from the arm selector: sending an arm-2 case down arm 3 swaps
`CntrPar%PC_MinPit` for `LocalVar%PC_MinPit`, which is precisely the difference
the corpus cannot see. That is transcription decision 3 above, and the sweep now
says in numbers that the harness does not check it.

`swap_call_args` on a commutative `fmin` would be a (c) equivalence on its own
merits; it is left undeclared because it sits inside a site the corpus cannot
see, and declaring an equivalence inside a blind spot is how a real defect gets
excused. **`equivalent_declared` is 0 for this unit.** The classifications above
are a reader's map, not a deduction from the score.

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

**63,888 cases, 656 MB**, against `ForeAftDamping`'s 7,567 and `CheckInputs`'
16,769. Measured cost per mutant, from nine parts:

```
const_tweak  40 in 1178 s (3 slices)   index_offset  18 in 511 s
arith_op     17 in  487 s              swap_operands 15 in 413 s
compare_op   13 in  369 s              negate_cond    9 in 268 s
calls         6 in  204 s              ->  26.5 s per mutant, 3,430 s total
```

**A narrowing was tried as a way out and is kept because it priced itself at
zero.** `LocalVar_NumBl = { values = [3, 0] }` gives 42,694 cases and 19.4 s per
mutant. `mutation.index.numbl_2value.json` is the `index_offset` population
scored on that corpus: **14 of 18, the same four survivors by id**, identical to
the 63,888-case result. So the two-value list costs nothing measurable here and
buys nothing either; the four-value list is kept because it is strictly more
input. That is unit #52's lever priced in the other direction, and it is the
second data point on it.

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

## Why the disposition is `deferred`

`min_mutation_score` is **1.0** and this unit scores **0.7797**. P12 fails on
that number. It is the only failing predicate; the other thirteen pass.

The distinction against the first dispatch is the whole point of the relaunch:
P12 failed then because `mutation/IPC.json` did not exist, and fails now because
it does and says 0.7797. Nothing about the translation changed between the two —
`translations/Controllers/ipc.cpp` is byte-for-byte what the first dispatch
verified, and the harness, post-integration and gate layers all reproduced their
numbers exactly.

Reaching 1.0 would require declaring 26 equivalences, and 9 of them sit at a site
the gate has PROVEN non-equivalent by killing one of them on 159,758 values. The
honest reading is that this unit's corpus has one blind spot, at
`IPC_SatMode`/`fmin`, and that the gate covers it.

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
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the copy-back red test, **63,888 of 63,888**, and the argument for perturbing the `localvariables` copy-back rather than either of the other two the wrapper carries |
| `gate.redtests.txt` | the three gate perturbations, the 22-file DISCON survey, the scenario-by-scenario reasons for the three exact zeros, and the drop_call cross-instrument result |
| `gate.redtest.{additive,multiplicative,fmin_dropcall}.json` | those three runs, all three re-taken at the current instrument |
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
   gap, and running it through the other instrument costs one gate run.** Nine
   of this unit's twenty-six survivors were about to be reasoned about; one 291 s
   red test settled all nine.
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
