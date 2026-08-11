# STATUS — rosco-r2

**This is the live state file. Read it first, then `RUNBOOK.md`.**
`DECISIONS.md` is the append-only record of *why*; this file is *where things
stand*. One copy of every count — do not duplicate them anywhere else.

**As of 2026-08-11: unit #3 `ColemanTransformInverse` is `integrated` and
CLOSED.** Every layer green with its own red test, and the unit's finding is
about the INSTRUMENTS rather than the translation. Three of them say less than
they appear to:

1. **RUNBOOK's kernel-liveness recipe cannot be run on this unit at all.** It
   counts non-zero `reference` values in `verify_fields.csv`; VIT logs an ARRAY
   field as one row with both value columns EMPTY. The recipe returns an empty
   answer that reads like a clean one. The zero-writing stub is the only
   instrument that answers the question here — and it does: 61 of 63 cases go
   `OUT_TOL`.
2. **A kernel `PASSED` is a TOLERANCE claim, not a bit-identity claim.** The
   generated comparison falls back to an ABSOLUTE RMS against `1.D-14`; outputs
   here are of order 1e-3 rad. The bit-exact claim is in the field log's
   `status` column, which reads `IDENTICAL` for all 63.
3. **`aziOffset` is 0 at all five call sites in all 27 scenarios.** A
   translation ignoring that argument passes the kernel 63/63 AND the gate
   5,252,000/5,252,000. Only the differential harness varies it; three mutants
   die in 81 of 257 cases each, and those 81 cases are the whole of what
   constrains it.

Unit #1 `AddToList` and unit #2 `ColemanTransform` remain closed. Escalation 1
is answered; **escalation 2 still stands** — a dead unit whose signature does
NOT cross has no path to harness, mutation or gate, and `loop/done.py` still has
no branch for `integrated_unexercised`.

Next: the lowest-order remaining unit in `plan.json`. Confirm it against
`plan.json` before starting, including its `phase` and `proposed_verification`
fields — those are hypotheses, not facts. Run
`python3.12 scripts/done_check.py <Unit>` before setting any disposition.

## Counts

3 attempted / 3 integrated / 0 integrated_unexercised / 0 out_of_scope /
0 deferred / 0 blocked.

69 units in `plan.json`; 66 remain.

## Evidenced

- **E1.2** — `rosco/controller/CMakeLists.txt` passes `-ffp-contract=off` to
  gfortran and to g++, and `scripts/assert_fp_contract.sh` asserts both the
  build line and **zero FMA instructions in the linked library**. Red-tested by
  blanking the flag: 0 build lines, 103 FMA, exit 1.
- **E3.1** — `scripts/gate.py` runs 27 scenarios, compares 5,252,000 values
  across 351 channels bit-for-bit against `baseline_arrays`, prints the count
  and persists `gate/__gate__.json`. Exits non-zero on mismatch and on comparing
  nothing.
- **E3.2** — gate observed red: perturbing `LPFilter`'s leading `1.0` moved
  1,604,573 of 5,252,000 values across 138 of 351 channels; reverting restored
  0. `gate/__gate__.redtest.json`.
- **E3.5** — `baseline_arrays/` regenerated from clean pre-integration source
  after E1.2 changed the flags; 20 of 27 scenario files moved. E3.2 re-run
  against the new set, so the red and green evidence describe the same
  baselines.

Both E3.1 and E3.2 were red-tested **as criteria**, not just satisfied:
corrupting the expected values in `phases.toml` turns them `[FAIL]`, restoring
them turns them `[ok]`.

### The campaign's VIT moved twice during unit #1: `d07a716` → `22086e8` → `37f8bdf`

Neither move was an upgrade; both are fixes this unit produced.

`22086e8` (first dispatch) made `interface_gen` REFUSE an ALLOCATABLE
INTENT(INOUT) dummy instead of emitting a wrapper that compiles and does
nothing, and gave the conformance matrix an `integrates` column measuring the
generators `vit integrate` actually ships.

`37f8bdf` (second dispatch) replaced the refusal with the feature it was
standing in for: the descriptor bridge, in all three generators
(`interface_gen`, `test_validate`, and the scaffold), plus two things found on
the way — `vit_translated.h` did not include `<ISO_Fortran_binding.h>` for a
declaration that names `CFI_cdesc_t`, and **every file `vit integrate`
generated carried `// After verification: <name> kernel PASSED`**, a verdict
the integrator never checked and which was flatly false here. The loop's
harness learned the same convention in `cf885e3`. VIT suite 935 passed; the
loop's 393, with 7 pre-existing `test_tiny_campaign` failures confirmed present
without these changes.

`tests/conformance/matrix.toml`'s `c_alloc_inout` has now been all three
things: `compiles = "no"` (measured on the wrong generator), `integrates =
"refused"`, and now `yes`/`yes`. Its `why` carries the history.

**`AddToList`'s first-dispatch evidence was deliberately measured under
`d07a716`** and stamps it. `evidence/AddToList/vit_interface.stdout.txt`,
`vit_translate.stdout.txt`, `addtolist.scaffold.cpp` and
`bridge_probe/mod_vit.f90` **cannot be regenerated** — that generator no longer
exists in either later form. They are the record of what it did.

`ColemanTransform`'s committed evidence was produced under `d07a716` and is
unaffected in substance. One caveat, recorded rather than left to be
discovered: `rosco/controller/src/colemantransform.cpp` was NOT regenerated, so
the "regenerates byte-identical" property below no longer holds for its three
header comment lines, which `37f8bdf` changed. Everything else about it is
untouched.

### ColemanTransform, second pass — every number re-measured

Every artifact for this unit now comes from ONE instrument pair, VIT `d07a716`
and loop `ebce989`, and each names its own producer in a `vit_rev`/`loop_rev`
field. The first pass used VIT `d85b33b` and loop `3c88913`.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 27 | 63/63, 14,175 fields IDENTICAL | zero-writing stub → 124 `OUT_TOL`; green restored on revert |
| differential harness vs clean Fortran | 199 checked, 0 failed | mutation, below |
| mutation score | 35/35 killed, 1.000 (33 by case mismatch, 2 by failing to compile) | baseline green first, else it refuses to score |
| post-integration harness (wrapper only) | 199 checked, 0 failed | wrapper args swapped → 199/199 failed; green restored on revert |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `2.0/3.0` → `2.000001/3.0` moved 124,353 values across 15 channels; revert → 0 |

`vit integrate` under the new VIT regenerated `Functions.f90`, `CMakeLists.txt`
and `colemantransform.cpp` **byte-identical** to the committed integration.

### ColemanTransformInverse — every layer, and its red test

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 27, `Controllers.f90:561` | 63/63, `pitcomipc_1p` `IDENTICAL` in every case | zero-writing stub → 61/63 `OUT_TOL`; green restored on revert |
| differential harness vs clean Fortran | 257 checked, 0 failed | the mutation score, below |
| mutation score | 24/24 behavioural killed, 1.000, 0 declared equivalent | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 257 checked, 0 failed | `axTIn`/`axYIn` swapped in the wrapper's CALL → 256/257 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `PitComIPC[0]` × 1.000001 moved 389,644 values; revert → 0 |

2 of the 24 mutation kills are CRASHES (`[2] -> [2 + 1]` and `'2' -> '3'`, both
writing past the caller's 3-element array), so the killed-by-case-mismatch count
is 22 of 24. 2 further `compare_op` mutants did not compile and are EXCLUDED
from the score rather than counted — `vit_mutate.py` changed this after unit #2
(loop `46a7f4f`), so that unit's `35/35` and this one's `24/24` are not the same
measurement.

The signature crosses whole: `PitComIPC(3)` `INTENT(OUT)` becomes
`REAL(C_DOUBLE), INTENT(OUT) :: PitComIPC(*)` — by reference, rank preserved, no
extent parameter emitted or needed — and the five scalars go by `VALUE`.
`vit interface` was read attribute by attribute before any C++ was written, per
the habit unit #1 installed; nothing was dropped.

## Open

- **`vit check -f <file>` attributes findings to the FILE, not the function.**
  It reported `minval-endpoints` and `array-section-row` against a 6-line
  translation containing neither; both came from `interp1d`/`interp2d` elsewhere
  in `Functions.f90`. `--function` only sets the report header. Every finding on
  a multi-procedure source has to be re-attributed by hand, and a checker that
  cries wolf twice per unit is a checker sessions will learn to skip. Fix belongs
  in VIT (X2), not here.
- **One argument of unit #3 is invisible to both bit-exact layers.**
  `aziOffset` is 0 at every `ColemanTransformInverse` call site in every
  scenario. The differential harness covers it and nothing else does. This is P9
  at ARGUMENT granularity — coverage counts a line as exercised while one of its
  inputs is a constant throughout — and the campaign's verification ledger (E5.2)
  should be able to say which arguments each unit's bit-exact layers bound.
- **ESCALATION 1 is ANSWERED: VIT learned the C descriptor.** `37f8bdf` +
  `cf885e3`, reaching all three generators the escalation said it would have to
  — `interface_gen`, `test_validate.generate_fortran_bridge`, and the loop's
  `vitbridge`/`emit`. `AddToList` is integrated on it with a mutation score of
  1.000. **It does not automatically unblock the other three** —
  `Read_OL_Input`, `ParseInAry_Opt`, `ParseDbAry_Opt` — whose `bridge_feasible`
  verdicts came from the same pre-`integrates` matrix and have to be
  re-measured rather than inferred from this one.
- **ESCALATION 2 STANDS, and is sharper.** `AddToList` closed without the gate
  ever seeing it, because the harness and the mutation score do not need it.
  That is the shape of an answer for a dead unit — but only for one whose
  signature crosses. A dead unit that cannot cross has no harness, no mutation
  score, and a vacuous P9, and `loop/done.py` still has no branch for
  `integrated_unexercised`. The vocabulary has the word; the verifier does not.
- **A translation's mutation score can be raised by REMOVING restatements, and
  that is not gaming it.** Two survivors here were a `malloc(isize+1)` and a
  `memcpy(..., isize+1)` restating a quantity the allocation already fixed;
  perturbing either produced a memory error no value comparison can see. Naming
  it once (`nsize`) left one site, and that site decides the extent the caller
  sees. Two others survive genuinely and are DECLARED equivalent with reasons
  (`mutation/AddToList.equivalences.json`). The distinction — removed vs
  declared — is the thing to preserve: declaring the first pair away would have
  recorded a blindness as a property of the mutants.
- **`harness/cppmutate.py` reads a C++ template bracket as a comparison.**
  `static_cast<int*>` mutates to `static_cast<=int*>`, which does not compile. 8
  of 33 mutants here (24%) were that, one point under the 25% at which
  `vit_mutate.py` REFUSES to score. A translation with a few more casts would
  have been unscoreable for a reason that has nothing to do with its harness.
- **The differential harness varies this unit's allocation status but not its
  length.** With a single extent parameter `_extent_plan` gives the same value
  (3) in every case, so a defect that only appears at another length is not
  covered by the 43 cases. Recorded in `plan.json`'s `observability`.
- **`bridge_feasible` verdicts in `plan.json` were derived from a column that
  measures the wrong generator.** VIT's conformance matrix fed `bridge` and
  `compiles` from `test_validate.generate_fortran_bridge` — the differential
  harness's Fortran side — while `vit integrate` ships the output of
  `interface_gen`. Fixed in VIT (an `integrates` column, and a refusal), but
  **the plan's 64 `yes` verdicts were computed before that**, and one of them —
  `c_complex_in`'s shape — is now known not to integrate. The plan's
  feasibility column should be re-derived against the new matrix before it is
  trusted again.
- **THREE SCENARIOS EXECUTE NO CONTROLLER CODE AT ALL.** Scenarios 10, 14 and
  24 run 0 lines of Controllers.f90, ControllerBlocks.f90, Filters.f90 and
  Functions.f90; scenario 13 runs 12 lines of Functions.f90 and 0 of
  Controllers.f90. They enter DISCON, read parameters, and stop —
  DISCON.F90 42%, ReadSetParameters 39%, everything downstream 0%. Scenario 14
  reports `PASSED` and 11 of its 13 channels are a single constant. They
  contribute their values to the gate's 5,252,000 while constraining nothing
  about the controller. This is **E3.3's exact failure mode** and E3.3 is still
  `manual`. Measured, not inferred: `coverage/line_coverage.json`.
- **E2.3 is not closed by `coverage/line_coverage.json`.** That file is real
  per-scenario line coverage and it is what C2 now selects call sites from, but
  it comes from a `--coverage -O0` build, not the campaign's Release build. The
  criterion asks for coverage from a clean build committed as phase evidence.
- **The launcher's protect glob has a hole.** `rosco/controller/src/*.f90`
  matches 11 files and NOT `DISCON.F90` — the one file `vit extract` modifies
  (strips CRLF, even under `--dry-run`). Add
  `--protect 'rosco/controller/src/*.F90'`.
- **`vit verify` and `vit integrate` rewrite `vit.yaml` and strip every
  comment.** The file is declared `derive` and its provenance lives in those
  comments; three runs deleted it three times. Currently restored by hand after
  each unit. Either VIT should round-trip comments or the provenance should live
  somewhere VIT does not write.
- **`regress.sh --baseline` does not restore `Examples/DISCON*.IN`.** Only
  `gate.py` snapshots and restores them. A baseline regeneration leaves two
  files modified and the tree dirty, which blocks `done.py`'s clean-tree check.
- **The harness Makefile's LIBS can carry `kgen_utils.f90.o`**, an object that
  exists only because an extraction left it in the build tree. A from-scratch
  build directory would not have it and the link would fail.
- `vit.yaml` is declared `derive` with `red_tested = false`. The planned test is
  to drop `assumed_size_arrays.avrSWAP` and confirm extraction breaks.
- Bootstrap otherwise incomplete: `phases.toml` still declares most criteria
  `manual`, which is NOT_EVALUABLE and never a pass.

## Closed

- **`reset_to_clean.sh` used to leave `CMakeLists.txt` integrated**, so "clean"
  meant the pre-integration SOURCE and not the pre-integration BUILD, and the
  differential harness link died on a second definition of its own translation.
  Closed in `6e614af`: the reset strips translated `.cpp` entries from
  CMakeLists and `restore_integrated.sh` puts them back. `harness.sh`'s LIBS
  workaround is kept — it is what makes the artifacts already committed for
  unit #2 reproducible.

- **`ColemanTransform`'s closure was asserted; now it is measured.**
  `scripts/done_check.py` runs the loop's own `DoneVerifier` with exactly the
  configuration `run_campaign.py` builds. At the start of the second pass it
  returned INCOMPLETE 10/13; it now returns COMPLETE 13/13. The failing
  predicate was P12: `vit_mutate.py` wrote `total`/`equivalent` and P12 reads
  `mutants`/`equivalent_declared`, so a genuine 35/35 was invisible to the check
  that requires one — and it failed with the WRONG REASON, "the operator set
  reached nothing in this unit". Fixed in the loop repo (`13265e7`, both
  spellings emitted) and the artifact regenerated by the tool, not hand-edited.
- **Three defects in this campaign's own scripts, each found by a run that
  should have worked.** All three were invisible while the stale artifacts sat
  on disk looking like results. See DECISIONS.md.
  1. `harness.sh` failed on VIT `d07a716`'s per-unit test directory — the skew
     it existed to reconcile had closed.
  2. Its post-integration LIBS extraction passed the literal words `LIBS` and
     `+=` to the linker, because the new VIT's Makefile has a second `LIBS +=`
     assignment its `^LIBS =` grep could not see. It now asks `make`.
  3. It aborted one line before stamping whenever `./test` exited non-zero — so
     the RED artifact, the one that most needs to say what it measured, was the
     only one that could not.

- **`AddToList` was `blocked`; it is now `integrated`.** The blockage was
  escalation 1, and the answer was to build the feature the refusal was standing
  in for rather than to close the unit as impossible. What made it closable is
  that its verification never needed the gate: a differential harness against
  the clean Fortran (43 cases, both branches red-tested), a mutation score of
  1.000, and a post-integration harness for the wrapper. The gate's green for it
  is still vacuous and is still committed beside the red test that says so.

- **The extraction blocker was a dead call site, not a tool defect.**
  `vit extract` printed `✓ Extraction successful` and `WARNING: No state data
  captured.` for `AddToList` three times. It was aimed at
  `ROSCO_IO.f90:1126`, which coverage now shows has **zero hits in all 27
  scenarios** — as do all five `AddToList` call sites. Pointed at a call site
  that executes, extraction works: `ColemanTransform` at `Controllers.f90:510`
  captured 63 state files, and kernel replay is available as a verification
  route. The tool was reporting success about instrumentation it had genuinely
  installed; the run never reached the pragma.
- **E1.2, previously "declared but unmet".** Closed 2026-08-10 — see Evidenced.
- **Reset-to-clean**, which did not exist: `scripts/reset_to_clean.sh` and
  `scripts/restore_integrated.sh`, red-tested.
