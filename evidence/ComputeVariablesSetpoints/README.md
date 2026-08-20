# ComputeVariablesSetpoints — unit #62, evidence index

`ControllerBlocks.f90:87-193` in the clean source (54dd134). The setpoint
block: pick a pitch reference speed, pick a torque reference speed by one of
four control laws, optionally look it up again in Region 3, filter it, exclude
a tower-resonance band, saturate it, apply setpoint smoothing, clamp it to the
minimum rotor speed and form four errors from it. Called from one site,
`DISCON.F90:114`, unguarded — 407,976 times across 23 of the 27 scenarios.

Four callees, all four already translated and all four CALLED rather than
inlined: `LPFilter` (#12, ×2, both post-incrementing `objInst%instLPF`),
`interp1d` (#23, ×0..3), `RefSpeedExclusion` (#59, ×0..1), `saturate` (#24,
×0..1).

## DISPOSITION: `deferred`, on P12 and on one number

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ComputeVariablesSetpoints.json`) | **25,398 checked, 0 failed, 0 inadmissible**. R4 compares the return value plus 480 out-parameters | it went red first: `std::max` at three sites moved **3,300 of 25,398** in exactly two shapes, and that is this unit's one translation defect |
| mutation (`mutation/ComputeVariablesSetpoints.json`) | **58 killed of 79 scoreable, 2 equivalent, 0 nocompile, 0 unreachable — 0.7342.** 21 OPEN | `declared_but_killed` and `unreachable_but_killed` both EMPTY |
| post-integration (`harness/ComputeVariablesSetpoints.postintegration.json`) | **25,398 checked, 0 failed** (E4.5) | this unit's own `vit_copy_scalars_to_localvariables` deleted from its own wrapper: **25,398 of 25,398, PREDICTED 25,398 before the run**; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ComputeVariablesSetpoints.json`) | 5,252,000 values / 351 channels, 0 mismatched | **four, one of them a NEGATIVE CONTROL and one of them a REFUTED PREDICTION** — 728,340 / 0 / 0 / 1,977,826 |

**The number that defers it is 0.7342 against a threshold of 1.0.** Every one
of the 21 open survivors has an answer in `mutation_survivors.txt`; eleven of
them have a MEASURED cause, and none of them is a defect claim.

## THREE INFRASTRUCTURE DEFECTS HAD TO BE REPAIRED BEFORE ANY LAYER COULD RUN

None is in this translation, and the first two are fixed in the tool (X2)
rather than worked around.

1. **The generated callee bridge dereferenced a NULL module stash for its
   `INTENT(IN)` view argument** — `RefSpeedExclusion` is the campaign's first
   translated callee with one. `vit test-validate` declared a `C_PTR` for
   `CntrPar` in the bridge's dummy list and never converted it, reaching for
   `vit_original_controlparameters` instead, which only an integration wrapper
   sets. The differential harness calls the translation directly. Fixed in
   vit `426eef9`; the as-generated artifact is committed beside the diagnosis
   (C12) in `callee_bridge_null_stash.txt`.
2. **`vit_view_in_controlparameters` was an unconditional `ERROR STOP`** over
   three rank-2 ALLOCATABLE fields. The rank-1 bound was written because "a
   higher-rank field's shape is not recoverable" from one extent; the view
   struct spells `n_<f>_rows` and `n_<f>_cols` uniformly and the forward
   populator already reads them from `SIZE(src%<f>, 1..2)`. Also vit
   `426eef9`, and ADDITIVE: `vit_populate_controlparameters` and
   `vit_copy_scalars_to_controlparameters` regenerate BYTE-IDENTICAL, so no
   integrated wrapper moves.
3. **The REFERENCE wrote past its own arrays.** `objInst%instLPF` and
   `%instRL` are 1-based subscripts into `DIMENSION(1024)` members of
   `FilterParameters` and `rlParams`, and the ±1e3 default put them outside.
   The symptom was a neighbouring struct changing mid-call:
   `n_WE_CP` 3 → 2,021,761,243, then a 16 GB `ALLOCATE`. Three probes located
   it to the case and the statement; `harness/ranges.toml` carries the bound.

## THE ONE TRANSLATION DEFECT, AND THE HARNESS IS WHAT FOUND IT

gfortran's two-argument `MAX` returns the **second** operand at a NaN and at a
tie — it lowers to `maxsd`, whose own definition does that — while
`std::max(a, b)` is `(a < b) ? b : a` and returns the **first**. 3,300 of
25,398 cases, in exactly two shapes:

    case   3  LocalVar.VS_RefSpd   ref -1000.0   got nan     MAX(NaN, VS_MinOmSpd)
    case 132  LocalVar.VS_RefSpd   ref     0.0   got -0.0    MAX(-0.0, +0.0)

Replaced by a `fortran_max` helper written as `(a > b) ? a : b`, which is
decided by the language rather than merely recommended (`std::fmax` agrees at
the NaN and is only *recommended* to prefer `+0.0` at the tie).

## THE GATE'S REFUTED PREDICTION IS THE OTHER FINDING

`gate.redtest_predictions.txt` predicted the ELSE arm of the `VS_ControlMode`
chain to move a non-zero count in scenario 12 — the only scenario coverage
records for it, 15,999 calls. It moved **0**. A fourth red test forcing
`VS_RefSpd` by 1.0e5 on the always-run path moved 1,977,826 across **22 of the
23 calling scenarios, every one except 12**. Scenario 12 sets
`VS_ControlMode = 1`, K·Ω² torque, where the generator torque is
`Rgn2K·GenSpeedF²` and no speed reference is read: **the one scenario that
takes the arm is the one scenario in which this unit's principal output reaches
no gate channel at all.** Two zeros (RT2, RT3) with two different causes, and
only the pair (RT4, RT2) separates them.

## Files

| file | what it is |
|---|---|
| `callee_bridge_null_stash.txt` | the C12 record of the bridge defect, written before the fix |
| `callees.as-generated.f90` | the artifact it names |
| `inputs_census.txt` | the scored corpus's own inputs on three arms, and the divide-by-zero that explains eleven survivors |
| `mutation_survivors.txt` | one answer per survivor, the two equivalences and their control, and what 0.7342 does not cover |
| `harness.PINNED-CORPUS-RED.json` | the corpus repair that WORKS and goes RED — 577 of 22,434, kept because it is the next dispatch's starting point |
| `gate.redtest_predictions.txt` | four perturbations, predicted before the runs, with the refutation and the probe that explains it |
| `wrapper_redtest_prediction.txt` | the copy-back prediction, 25,398, written before the run |
| `revcheck.txt` | revcheck's one finding and its exact cause |
| `done_check.txt` | the done-condition, captured by `scripts/capture_done_check.sh` |

## Two things worth carrying

**A CALLEE'S `INTENT(IN)` VIEW ARGUMENT IS A DIFFERENT BRIDGE PATH FROM ITS
`INTENT(INOUT)` ONE, AND ONLY THE SECOND HAD BEEN EXERCISED.** Sixty-one units
called callees whose derived-type dummies were INOUT (`ErrVar`) or not view
types at all. The first `INTENT(IN)` one segfaulted on case 0 with an empty
stdout, which reads as a build problem. **Before bisecting a differential
harness that produces no JSON, `grep 'POINTER, SAVE'` the view module and ask
the generated bridge what it assumes** — unit #45 wrote that rule for the
INOUT path and it transferred exactly.

**A REFERENCE THAT WRITES PAST ITS OWN ARRAY IS A CORPUS PROBLEM WEARING A
TRANSLATION'S CLOTHES.** The failure presented as an absurd allocation size
inside the generated Fortran bridge, three layers from `objInst%instLPF`. The
probe that named it was the cheapest one: print one field either side of the
C++ call and see it change. **When a value the case file supplied is different
by the time the reference reads it, stop reading the corpus generator and start
bracketing the call.**
