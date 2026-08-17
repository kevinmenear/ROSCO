# Unit #47 — `ActiveWakeControl`

`rosco/controller/src/Controllers.f90:714-881` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ActiveWakeControl.json`) | **3231 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, all five callee bridges kept | the unit as a no-op: **86 of 3231** — and 86 is not 3231 because only **91** cases reach an arm at all |
| mutation (`mutation/ActiveWakeControl.json`) | **165 of 210 behavioural, 0.7857**, **0 declared**, 14 no-compile, 8 operators, **45 survivors standing** | the score *is* the red test, 165 times |
| post-integration (`harness/ActiveWakeControl.postintegration.json`) | 3231 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted: **70 of 3231**; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ActiveWakeControl.json`) | 5,252,000 values / 351 channels, 0 mismatched | TWO, because the five arms have disjoint scenario sets: PI moves **97,118** (scenarios 5, 11, 15, 21 = modes 4, 1, 2, 3), D2R moves **50,605** (15, 22 = modes 2, 5). Both revert-verified |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". A KGen
kernel is aimed at one call site in one scenario and this unit's five arms live in
five different scenarios; it also carries four implicitly-`SAVE` locals, which unit
#44 measured a kernel cannot replay.

## What each file is

| file | what it records |
|---|---|
| `activewakecontrol.final.cpp` | the translation as scored |
| `harness.restart_probe.txt` | the instrumented relink that named the 22 first-take failures as `AWC_Mode == 4` |
| `harness.first_take.restart_indeterminate.json` | that first take, kept: 6436 checked, 22 failed |
| `run_harness_redtest.sh`, `activewakecontrol.noop-harness-stub.cpp` | the no-op red test and its runner |
| `harness.noop.json` | 86 of 3231 |
| `harness.arm_census.txt` | AWC_Mode over all 3231 cases; and **0** cases with `AWC_harmonic(1) == 0` |
| `harness.live_case_inputs.txt` | NumBl, Time, phaseoffset, PC_MinPit/MaxPit, gains and the complex field over the 91 live cases |
| `gate.redtests.txt` | the two gate red tests and which arm each reached |
| `run_postintegration_redtest.sh`, `harness.postintegration.redtest.json` | the wrapper red test, 70 of 3231 |
| `mutation.census.txt` | all 45 survivors in ten measured groups, each with the input that would kill it |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |

## The three findings worth carrying

1. **An indeterminate answer stored in a `SAVE` local outlives the case that
   produced it.** `ResController` assigns its result only in its ELSE arm
   (unit #39). This unit stores that value into the implicitly-`SAVE`
   `AWC_TiltYaw`, so three of the first take's sixteen mismatches were at
   `restart == 0`, one case after one at `restart == 1`.
2. **A `COMPLEX(DbKi)` view field is not compared by the generated harness.**
   `LocalVar%AWC_complexangle` crosses in both directions and has **zero**
   `VITCMP` lines. Five mutation survivors live in the half of it that reaches
   no other output.
3. **A predicate on a constant-subscript element of an ALLOCATABLE array is not
   a knob, and no `ranges.toml` pin can repair it.** `AWC_harmonic(1) == 0`
   guards two writes that **no instrument in this campaign reaches** — 0 hits in
   27 scenarios, 0 cases of 3231.
