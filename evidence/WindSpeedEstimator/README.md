# Unit #65 — `WindSpeedEstimator`

`rosco/controller/src/ControllerBlocks.f90:267-488` (clean, at `54dd134`).
Disposition **integrated**. Four layers, all four red-tested, and one honest
failing number: the mutation score is **0.9234** against a threshold of 1.000.

**Second dispatch (P12 re-take).** Corpus 6 was replaced by corpus 7 -- two
`ranges.toml` entries, same 63,020 cases -- and every layer that depends on the
corpus was re-taken on it. 60 survivors became 19; 204 of 264 became 229 of
248; `arith_op` went from 34 of 40 to **38 of 38, score 1.000**, which is the
whole of the set the driver named. What is left is in
`mutation.survivor_census.corpus7.txt`, in three classes with three
dispositions, and twelve of the nineteen are blocked on an instrument defect
that is localised in `mutation.sanitize_refusal.nan_sign.txt` and escalated.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/WindSpeedEstimator.json`) | **63,020 checked, 0 failed, 0 inadmissible** against the CLEAN Fortran, on corpus 7 (md5 `44290973357db22784b138278ae781e7`) — this unit's primary evidence | the unit as a no-op: **63,020 of 63,020**, same corpus count, predicted before the run |
| gate, 27 scenarios (`gate/WindSpeedEstimator.json`) | 5,252,000 values / 351 channels, 0 mismatched | three perturbations, three predictions, three confirmations — see below |
| post-integration (`harness/WindSpeedEstimator.postintegration.json`) | 63,020 checked, 0 failed | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **63,020 of 63,020** |
| mutation (`mutation/WindSpeedEstimator.json`) | **229 of 248 scoreable, 0.9234**, 29 declared equivalent, 1 no-compile, 12 operators, **19 survivors standing** | the score *is* the red test, 229 times |

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


## The second finding: the survivors were the corpus multiplying by zero

`drop_call std::fmax(0.0, Cp_op) -> 0.0` **survived**. That mutant can only
survive if the clamp already returns `0.0` on every case, so the translation was
re-run with a sign census:

```
corpus 6   ekf=1173  cp_raw_pos=0  cp_zero=1173  F12_nonzero=10
           cp_raw = [-935.478, -387.138]
corpus 7   ekf=1173  cp_raw_pos=1173  cp_zero=0  F12_nonzero=1169
           cp_raw = [0.0355, 0.3237]
```

`Cp_op` multiplies the whole of `F(1,2)` and `F(1,3)`, so with `Cp_op == 0`
every mutation of every factor in the Jacobian's wind-speed column was
multiplied by zero. Twenty-odd survivors, one cause, and **the first corpus
repair was refuted by its own run** — a Cp band straddling zero puts the whole
corpus just *below* zero, because `lambda` is out of the TSR axis so interp2d
clamps to a corner and `_fill_array` ramps from `lo`. Both takes, with their
PHYSICAL / INSTRUMENTAL / WHAT LEAVES statements, are in `harness/ranges.toml`;
the per-corpus table is in `corpus_arm_census.txt`.

## The third finding: the harness adjudicates a NaN's sign bit

The sanitised sweep — the only instrument that reaches twelve of the nineteen
remaining survivors — is blocked by a baseline that fails at one case of
63,020 with an **empty sanitiser stderr**. Widening the generated hex dump to
256 bytes for one run named it: `LocalVar%WE` case 42053 holds fifteen quiet
NaNs, and **fourteen of them differ between the reference and a perturbed build
in exactly one bit, the sign**. Both sides are NaN. IEEE-754 does not say which.
`memcmp` on a nested type is therefore deciding a bit the source program does
not determine, and the shipped translation is green only because its codegen
happens to agree with gfortran's.

Full nineteen-double record in `mutation.sanitize_refusal.nan_sign.txt`.
Raised as an `invalidating_finding`, with the 32-byte truncation that hid it
for a whole dispatch as a `default_change_proposed`.

## The fourth finding: every EKF update in this corpus is the FIRST update

```
upd=1173  vt_zero=1173  k_zero=1173  p_offdiag_zero=1173  p11_eq_p22=1173
```

Read at the top of the update arm before anything writes: on all 1,173 cases
`LocalVar%WE` is exactly what the RESTART arm leaves — `v_t = 0`, `K = 0`,
`P = diag(0.01, 0.01, 1.0)`, the last compared as bit patterns. Four of the
nineteen survivors follow from that one fact, two of them by an exact
algebraic identity rather than a coincidence: moving `F(1,2)` to `F(2,1)` is
invisible on any `P` with `P(1,1) == P(2,2)`.

## The gate was RE-RUN at the final head of this dispatch

The corpus changed, so the harness and mutation layers were re-taken. The gate
does not read the corpus — it runs 27 simulations against committed baselines —
so re-running it was not required. It was re-run anyway, on the integrated build
left by the fourth `reset_to_clean` / `restore_integrated` round trip of this
dispatch, because "the tree I am leaving is the tree the evidence describes" is
a claim and not a deduction:

```
GATE PASS: compared 5252000 value(s) across 351 channel(s) / 27 scenario(s);
           mismatched 0
```

`gate/WindSpeedEstimator.json` came back **byte-identical** to the committed
one, so nothing in this dispatch — two `ranges.toml` entries, six mutation
parts, four reset/restore cycles and five instrumented builds — moved the
product.

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
| `probe_equivalences.{cpp,txt}` | the first thirteen equivalences EXECUTED, with a control that can fail — and its own refuted first model kept at the counter that refuted it |
| `probe_equivalences2.{cpp,txt}` | families F–H (the two one-sided `ABS` tests, `v_m ± v_t`, `n > 0` vs `n >= 0`), three controls, all three failed |
| `probe_flag_predicate.{py,txt}` | family I: both LPFilter implementations read `1` as a TRUTH, and the detector is shown to find a `== 1` when one exists |
| `windspeedestimator.subscript-census.cpp` | the shipped translation with a bounds-recording array wrapper and the `Cp_op` / state counters; it produced every number in the two findings below |
| `corpus7_census.txt` | that probe's output on corpus 7, one line |
| `mutation.survivor_census.corpus7.txt` | the nineteen standing survivors, three classes, one measured reason each |
| `mutation.sanitize_refusal.nan_sign.txt` | the sanitised baseline's failing case localised to fourteen NaN sign bits |
| `run_harness_redtest.sh`, `windspeedestimator.noop-harness-stub.cpp` | the no-op red test and its runner |
| `run_mutation_part.sh` | one guarded, operator-filtered part |
| `run_postintegration_redtest.sh` | the wrapper copy-back red test |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |
| `../../mutation/WindSpeedEstimator.equivalences.{json,md}` | the thirteen declarations and their proofs |

## The findings worth carrying

0. **A mutation survivor is the only instrument that asks whether the arm the
   corpus DOES reach is being multiplied by zero.** `fmax(0.0, Cp_op) -> 0.0`
   surviving is a statement about `Cp_op`, and no green count anywhere says it.
   Two `ranges.toml` entries later, `arith_op` is 38 of 38.
0b. **A comparison that uses `memcmp` on a nested type adjudicates NaN sign
   bits**, which no source program determines — so a green can be codegen luck
   and a sanitised baseline can refuse for a reason that is not the
   translation's. Widen the diagnostic before believing "cannot be localised".
0c. **Ask the corpus what STATE it enters an arm from, not only how often.**
   Here the answer was "always the initial one, 1,173 of 1,173", and it
   explains four survivors exactly, one of them by an identity in P.
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
