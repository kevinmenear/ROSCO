# Unit #65 — `WindSpeedEstimator`

`rosco/controller/src/ControllerBlocks.f90:267-488` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested, and one honest
failing number: the mutation score is **0.7727** against a threshold of 1.000.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/WindSpeedEstimator.json`) | **63,020 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran — this unit's primary evidence | the unit as a no-op: **63,020 of 63,020**, same corpus count, predicted before the run |
| gate, 27 scenarios (`gate/WindSpeedEstimator.json`) | 5,252,000 values / 351 channels, 0 mismatched | three perturbations, three predictions, three confirmations — see below |
| post-integration (`harness/WindSpeedEstimator.postintegration.json`) | 63,020 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **63,020 of 63,020** |
| mutation (`mutation/WindSpeedEstimator.json`) | **204 of 264 scoreable, 0.7727**, 13 declared equivalent, 1 no-compile, 12 operators, **60 survivors standing** | the score *is* the red test, 204 times |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness". The
direct-call harness is the layer taken, as for units #45–#48 and #62–#63.

## The finding: the gate caught a one-character defect that no other layer could

The differential harness was **63,020 of 63,020 GREEN** on a translation that
was wrong, over a corpus that reaches the defective arm 1,173 times.

```
Fortran  ... * CntrPar%WE_BladeRadius**2.0 * 1/LocalVar%WE%om_r * 3.0 * ...
was      ... * (R * R) * (1.0 / om_r) * 3.0 * ...
is       ... * (R * R) * 1 / om_r * 3.0 * ...
```

`*` and `/` associate left to right, so the reference multiplies by 1 and then
DIVIDES; the first version formed the reciprocal and multiplied by it. Two
sites, `F(1,2)` and `F(1,3)`. **The gate saw it at 2 values of 5,252,000** and
the full diagnosis — a control, two refuted hypotheses with runnable probes, and
the per-invocation trace that named invocation 544 — is in `gate.first_take.txt`.

Why nothing else could: a 2-ULP difference in one covariance element does not
survive into any of the 213 compared out-parameters **within a single call**. It
needs the EKF's own feedback across 544 timesteps, and the gate is the only
instrument in this campaign that has them.

## The corpus reached ONE arm, and the first mutation part is what asked

```
corpus 1   13,868 cases   mode1=0  mode2=0  ekf=0      <- and 13,868 of 13,868 GREEN
corpus 6   63,020 cases   mode1=14,070  mode2=12,489  ekf=1,173
```

Every one of the first corpus's 13,868 passing cases took `WE_Vw = HorWindV_F`.
Both estimators — most of the procedure — were never entered, and nothing in
`checked 13868 failed 0` says so. The first mutation part scored **0 of 11**,
all eleven mutants on the five `std::fmax` calls inside the arms no case
reached. Six `ranges.toml` entries and `--persist-nested` moved it; the full
per-corpus, per-arm table and **two refuted predictions** are in
`corpus_arm_census.txt`.

## The gate red tests

```
1 WE_Gamma gain, the I&I arm          0 of 5,252,000   PREDICTED ZERO
2 Ti = 0.18 -> 0.180001       1,084,204 of 5,252,000   107 channels, 21 scenarios
3 WE_Op = 1 -> 0              1,486,321 of 5,252,000   120 channels, 23 scenarios
```

Test 1 was predicted to move nothing and did: `Examples/DISCON.IN:123` sets
`WE_Gamma = 0.0`, so the whole `WE_Mode == 1` arm is annihilated. **The cost is
that those four statements have no gate oracle at all.** Tests 2 and 3 are a
pair whose ORDER is the control: a coefficient inside the EKF moves less than
switching the estimator off.

## Three tool defects, one fixed in the tool

| defect | disposition |
|---|---|
| VIT's callee bridge emitted `identity_result(1:SIZE(alpha))` — a symbol that does not exist — for any array-valued callee with no array dummy | **FIXED** in the VIT repo at `6a71314` (X2), with `ASSOCIATE` + `RESHAPE`; the control is that `VariableSpeedControl`'s seven bridges regenerate byte-identical |
| `vit_view_in_performancedata` was an unconditional `ERROR STOP`, so the first corpus that actually called `AeroDynTorque` died in the view populator | **REGENERATED**; the additivity check is unit #62's per-routine diff — the two routines every integrated wrapper calls are IDENTICAL |
| `vit_types.h` emits a 2-D member's Fortran dims verbatim (`double xh[3][1]`) while documenting a transposed access (`P[j-1][i-1]`) | **RAISED** in `.loop-run/findings.jsonl`; benign here because the minor extent is 1, wrong-address in general |

## What each file is

| file | what it records |
|---|---|
| `gate.first_take.txt` | the red gate, the control, the two refuted hypotheses, the per-invocation trace, and the defect |
| `gate.first_take.json` | the red run itself — the RE-RUN, because `gate.py`'s default `--out` let the green overwrite the first |
| `gate.redtests.txt` | three predictions written before three runs, and their per-scenario channel lists |
| `corpus_arm_census.txt` | six corpora × twelve arms, the two refuted predictions, and what is still not reached |
| `windspeedestimator.arm-census.cpp` | the shipped translation with twelve counters, which produced that table |
| `probe_corpus_count.py` | unit #60's count probe, copied: 194,187 cases at 492 varied parameters, which is why `--transitive-read-set` |
| `probe_case_trace.py` | two `fprintf`s that turn `harness produced no JSON` into a case index and a side |
| `probe_pow.{f90,cpp}` | `x**3.0` vs `std::pow` — identical, and the probe can fail |
| `probe_matmul.{f90,cpp}` | 123 values across all six MATMUL expressions — identical |
| `probe_wrapper_trace.py` | the per-invocation nine-field bit-pattern trace that named invocation 544 |
| `probe_equivalences.{cpp,txt}` | the thirteen equivalences EXECUTED, with a control that can fail — and its own refuted first model kept at the counter that refuted it |
| `run_harness_redtest.sh`, `windspeedestimator.noop-harness-stub.cpp` | the no-op red test and its runner |
| `run_mutation_part.sh` | one guarded, operator-filtered part |
| `run_postintegration_redtest.sh` | the wrapper copy-back red test |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../mutation/WindSpeedEstimator.equivalences.{json,md}` | the thirteen declarations and their proofs |

## The findings worth carrying

1. **A harness green over a corpus that reaches one arm is a statement about
   that arm, and the harness cannot say so.** Only the mutation survivors did.
2. **`values` makes a parameter a FLAG and `lo`/`hi` does not.** Two predictions
   were refuted by writing a domain as a range; the same domain as a `values`
   list moved the arm from 0 cases to 1,173.
3. **The gate is the only instrument here with hundreds of timesteps**, and a
   defect whose signal is 2 ULP inside a feedback loop is invisible to every
   single-call comparison no matter how wide its corpus.
4. **A tool that says `harness produced no JSON` is usually saying SIGSEGV**,
   and two `fprintf`s to stderr cost one run and name the case.
5. **Execute an equivalence before declaring it, and give the probe a control
   that can fail.** The first control here returned 0 disagreements — which
   refuted the model it was defending rather than confirming it.
