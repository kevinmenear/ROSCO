# RefSpeedExclusion — unit #59, evidence index

`ControllerBlocks.f90:763-818` in the clean source (54dd134). Tower-resonance
avoidance for the torque controller's speed reference: hold the reference
outside a user-named band, then rate-limit it. Called from one site,
`ControllerBlocks.f90:157`, under `IF (CntrPar%TRA_Mode > 0)`.

Seven compared outputs: `LocalVar%FA_Hist`, `%TRA_LastRefSpd`, `%VS_RefSpd_TRA`,
`%VS_RefSpd_RL`, `%VS_RefSpd` (read on the first line and written on the last),
the nested `%rlP` compared bytewise, and `objInst%instRL`. One callee,
`ratelimit` (unit #43), CALLED and not inlined.

## The layers

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/RefSpeedExclusion.json`) | **13,777 checked, 0 failed, 0 inadmissible, GREEN AT THE FIRST RUN** on both the first corpus (3,967) and the repaired one. R4 compares the return value plus 195 out-parameters | **five stubs; four EXACT against point predictions written before the runs** — 13,777 / 4,752 / 22 / 1,088 — and the fifth (`no-restart-init`, 814 against 845) short by a MEASURED 31 coincident zero snaps |
| corpus repair (`inputs_census.txt`, `harness/ranges.toml`) | `WE_GearboxRatio == 0.0` in **3,027 of 3,967** cases of the first corpus — three quarters of it dividing by zero — and `TRA_ExclBand == -600` in 2,863, which empties the band. Three `values` pins move the in-band arm from **55 of 3,967 to 4,752 of 13,777** | the control is the MUTATION SET, not the case count: five survivors alive before, all five dead after, with nothing declared |
| mutation, sanitised (`mutation/RefSpeedExclusion.json`) | **35 mutants, 1 nocompile, 32 killed, 3 equivalent, 0 open. 32/32 = 1.0000.** `--workers 8` | the score IS the red test (E4.6). TWO undeclared sweeps committed either side of the corpus repair: 28/36 = 0.7778 and 33/36 = 0.9167 |
| mutation, VALUE ORACLE (`mutation/RefSpeedExclusion.value-oracle.json`) | **36 mutants, 0 nocompile, 33 killed, 3 equivalent. 33/33 = 1.0000** — the same (empty) survivor SET | the control is the SURVIVOR SET, not the count. `killed_by_sanitizer` is 0: the sanitiser bought nothing here, and the clean build did not report |
| arm partition (`arm_partition.txt`) | all five arms non-empty on both corpora; `845` (restart AND in-band) computed from the INPUTS agrees to the case with two `negate_cond` kill counts computed from the OUTPUTS | the probe is one `fprintf` under `--no-generate`, so the corpus is byte-identical to the scored one, and it is stripped afterwards |
| gate, 27 scenarios (`gate/RefSpeedExclusion.json`) | 5,252,000 values / 351 channels, 0 mismatched | **three, and one of them is a NEGATIVE CONTROL**: the return path moved **30,051**, the in-band arm **29,895**, both scenario 9 only and both revert-verified at 0; a 1.0e30 write on the zero-coverage snap line moved **0**, predicted exactly |
| post-integration (`harness/RefSpeedExclusion.postintegration.json`) | 13,777 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **13,777 of 13,777, PREDICTED 13,777** from `harness.view-noop-stub.json` before the run; reverted, rebuilt, green re-taken at 0 |

## Files

| file | what it is |
|---|---|
| `inputs_census.txt` | the four band inputs over the FIRST corpus, and the divide-by-zero finding that forced the pins |
| `arm_partition.txt` | both corpora partitioned by the reference's own arms, plus the gate's own coverage of the same arms |
| `redtest_predictions.txt` | the stub predictions for both corpora, written before the runs, with both refutations measured |
| `refspeedexclusion.noop-stub.cpp` and four siblings | the stubs, each one statement or one arm removed |
| `refspeedexclusion.view-noop-stub.cpp` | the body deleted but `instRL` still advanced — a PREDICTOR for the post-integration wrapper red test, not a red test of its own |
| `harness.*-stub.json` | the second-corpus runs |
| `harness.*-stub.corpus3967.json` | the first-corpus runs, kept because they are what the pins were bought with |
| `mutation_survivors.txt` | one answer per survivor, and what 1.0000 does not cover |
| `gate.redtest_predictions.txt` | the three gate perturbations, predicted from the coverage file before the runs |
| `run_harness_stub.sh`, `run_wrapper_redtest.sh` | copied from unit #58 (P4), three names changed |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |

## Two things worth carrying

**A GREEN HARNESS ON A CORPUS THAT COMPUTES NaN IS INDISTINGUISHABLE FROM A
GREEN HARNESS.** The first corpus passed 3,967 of 3,967 at the first run and
three quarters of it was `0.0/0.0`. Nothing in the harness's own report says so
— `UNCONSTRAINED: 9 varied parameter(s) on the +/-1e3 default` is the only hint,
and it names the parameters without saying what `_base` gives them. The mutation
score is what exposed it, and the census is what named it. One `fprintf` over
the inputs, fifteen seconds, and it should be run for any unit whose FIRST
statement divides.

**AN EVIDENCE FILE CAN BE SILENTLY NOT COMMITTED BY A `.gitignore` GLOB.**
`refspeedexclusion.localvar-noop-stub.cpp` matched `.gitignore:64`'s `*local*`
and `git add -A evidence/…` added nothing while reporting success; only
`git commit` saying "nothing to commit" showed it. Renamed to `view-noop`. The
predicate that would have caught it downstream is the done-condition's P5
(unresolved evidence), one layer later than the commit that thought it had
recorded the file.
