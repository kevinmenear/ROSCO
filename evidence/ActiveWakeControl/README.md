# Unit #47 — `ActiveWakeControl`

`rosco/controller/src/Controllers.f90:714-881` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested.

**Second dispatch (this one) was opened on one unmet condition: P12 at 0.7857
with 45 mutation survivors.** It closes at **0.9200 with 16**, still below the
1.000 threshold and still failing on purpose, with every survivor classified
against a measurement and every named fix escalated. The translation was not
touched; what changed is the corpus.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ActiveWakeControl.json`) | **21236 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, all five callee bridges kept | the unit as a no-op: **17645 of 21236** — where the first dispatch's no-op moved 86 of 3231 |
| mutation (`mutation/ActiveWakeControl.json`) | **184 of 200 scoreable, 0.9200**, **10 declared**, 14 no-compile, 8 operators, **16 survivors standing** | the score *is* the red test, 184 times |
| post-integration (`harness/ActiveWakeControl.postintegration.json`) | 21236 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted: **17521 of 21236**; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ActiveWakeControl.json`) | 5,252,000 values / 351 channels, 0 mismatched | TWO, because the five arms have disjoint scenario sets: PI moves **97,118** (scenarios 5, 11, 15, 21 = modes 4, 1, 2, 3), D2R moves **50,605** (15, 22 = modes 2, 5). Both revert-verified |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". A KGen
kernel is aimed at one call site in one scenario and this unit's five arms live in
five different scenarios; it also carries four implicitly-`SAVE` locals, which unit
#44 measured a kernel cannot replay.

**The gate is the first dispatch's, unchanged and still current.** The translation
is byte-identical to what it gated, and the first dispatch re-ran the gate after a
full `reset_to_clean` / `restore_integrated` round trip and got an artifact
byte-identical in every field including `vit_rev`
(`gate/ActiveWakeControl.postroundtrip.json`). This dispatch's clock went to the
corpus repair and the 224-mutant re-sweep instead of a third identical gate run.

## What the second dispatch changed, and what it measured

Two entries in `harness/ranges.toml`, both stated with the measurement forcing
them and neither touching the translation:

- **`CntrPar_AWC_Mode = { values = [3, 4, 5, 1, 2, 0] }`** — written as the
  enumeration it is rather than as a range. It had no stated range, so `_base`
  put it at **−600** in 3032 of 3231 cases, on a body that is one `IF`/`ELSEIF`
  chain with no `ELSE`. As `values` it becomes an R2 flag, so every ladder rung
  and every R6 magnitude is re-run under every mode. The order is part of the
  judgement: `values[0]` is the base the first cases carry, and modes 3/4 are the
  only arms that read the `SAVE` locals without necessarily writing them.
- **`CntrPar_AWC_phaseoffset = { lo = 0, hi = 360 }`** — one turn, which narrows
  nothing: it is read only as `aziOffset` inside `cos`/`sin` with an INTEGER
  harmonic. It was 0.0 in **all 91** live cases, and at 0.0 `x*D2R`, `x` and
  `x/D2R` are the same bits.

| | first dispatch | second |
|---|---|---|
| cases | 3231 | **21236** |
| cases reaching an arm | 91 (2.8%) | **17697 (83%)** |
| no-op red test | 86 of 3231 | **17645 of 21236** |
| post-integration red test | 70 of 3231 | **17521 of 21236** |
| distinct `AWC_phaseoffset` over live cases | 1 | **130** |
| distinct `(PC_MinPit, PC_MaxPit)` over live | 1 | **537**, 311 wider than 1e6 |
| live cases with `NumBl == 3` | 0 | **11**, five of them in Mode 1 |
| cases with `AWC_harmonic(1) == 0` | 0 | 0 |
| `AWC_complexangle` supplied non-zero | 0 | 0 |
| mutation score | 0.7857, 45 survivors | **0.9200, 16 survivors** |

Of the 45: **20 killed, 10 declared equivalent, 15 still standing**, plus **one
new survivor** — `ced1c24e`, which the old corpus killed. That regression is
reported rather than absorbed; its mechanism is exact and is in the census.

## What each file is

| file | what it records |
|---|---|
| `activewakecontrol.final.cpp` | the translation as scored — unchanged by this dispatch |
| `mutation.census.txt` | **all 16 survivors in five groups**, each with its measurement and its classification; and the 45 → 20/10/15/+1 map |
| `mutation.extent_equivalence.txt` | the complete subscript enumeration the eight extent declarations rest on, and the assembly diff that does NOT settle them |
| `harness.arm_census.txt` | the 21236-case corpus measured: modes, `NumBl`, `AWC_harmonic(1)`, `phaseoffset`, the PI window, the complex field, and CASE ORDER |
| `harness.live_case_inputs.txt` | **superseded**, kept: it is the measurement that forced the two new `ranges.toml` entries |
| `harness.restart_probe.txt` | the instrumented relink that named the first take's 22 failures as `AWC_Mode == 4` |
| `harness.first_take.restart_indeterminate.json` | that first take, kept: 6436 checked, 22 failed |
| `run_harness_redtest.sh`, `activewakecontrol.noop-harness-stub.cpp` | the no-op red test and its runner |
| `harness.noop.json` | 17645 of 21236 |
| `gate.redtests.txt` | the two gate red tests and which arm each reached |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the wrapper red test, 17521 of 21236 |
| `reset_restore_roundtrip.txt` | the first dispatch's round-trip evidence |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../mutation/ActiveWakeControl.equivalences.{json,md}` | the ten declarations and their reasons |

## The findings worth carrying

1. **`_base` puts a mode selector at the midpoint of its range, and a midpoint is
   not a mode.** `values` and `lo`/`hi` are different mechanisms: `lo`/`hi`
   deepens ONE arm, `values` makes R2 apply and deepens all of them. The cost is
   time, not coverage.
2. **A stratum that already varies a quantity becomes reachable *in a branch* the
   moment the branch selector is a flag.** The first dispatch costed `NumBl == 3`
   as a generator change with an X3 price; it did not need one.
3. **A `SAVE` local's initialiser is an ORDERING property.** No rule in this
   generator orders cases, `values[0]` is the only lever `ranges.toml` has, and it
   is not enough. Two survivors here; `ExtController`'s `ranges.toml` entry
   records the same wall from the other side.
4. **An assembly diff is a confirmation instrument, not a decision instrument.**
   Proposed by the first census, taken here, and retracted: enlarging a stack
   array reshuffles the register allocator, so all eight diffs are non-empty and
   silent.
5. **An out-of-bounds store is not an equivalent program.** `e9d4580d` is
   corrected out of the first census's equivalence group and stays an honest
   survivor; its instrument is a sanitiser build, not an input.
6. **A `COMPLEX(DbKi)` view field is not compared by the generated harness** — 0
   of 481 `VITCMP` lines. Five survivors, up from four, now that `97b678b5` is
   correctly attributed to it.
7. **A predicate on a constant-subscript element of an ALLOCATABLE array is not a
   knob.** Narrowed this dispatch to one regex: `predicate_knobs_from`'s `_NAME`
   has no subscript group, while its two siblings in the same file do.
8. **A nested derived type inside a view struct is zero-filled and never varied.**
   The harness has always said so; this is the first survivor to land on it. With
   `resP` at zero, `ResController`'s Tustin biquad collapses to two terms and the
   second is annihilated.
