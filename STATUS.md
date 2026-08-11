# STATUS — rosco-r2

**This is the live state file. Read it first, then `RUNBOOK.md`.**
`DECISIONS.md` is the append-only record of *why*; this file is *where things
stand*. One copy of every count — do not duplicate them anywhere else.

**As of 2026-08-11: unit #5 `ExtController` is `blocked`, and it is the first
unit in this campaign that no instrument can verify.** Not because the gate
cannot see it — three units before it had that problem — but because **the
oracle cannot be run.**

**FORCING THE ORIGINAL FORTRAN TO EXECUTE MAKES IT SEGFAULT.** `Ext_Mode` is 0
in all 14 `Examples/*.IN`, so `ExtControl.f90` measures **0 of 28 executable
lines in all 27 scenarios**. Turning the guard on through `vit_sim.py`'s own
`write_discon(patches={'Ext_Mode': 1})` produces **exit status −11, signal 11**:
`DLL_FileName` is the literal string `"unused"`, no external Bladed-style
library is shipped anywhere in the tree, and `ExtController` never checks
`ErrVar%ErrStat` after `LoadDynamicLib` — so `dlopen` fails, the code prints
*"Library loaded successfully"*, and the next two statements are
`C_F_PROCPOINTER` on a null funptr and a CALL through it.
`evidence/ExtController/probe_ext_mode_1.json`.

That is what separates it from `AddToList`, which was equally dead and still
closed `integrated`: the harness could **run the clean Fortran** as an oracle.
Here P7 has nothing to point at. **This is a fourth shape of unobservability in
five units, and the first that is not about the instrument** — #1 was a line no
scenario reached, #3 an argument constant in every scenario, #4 an executed line
whose result is cancelled downstream, and this is a unit whose reference
implementation cannot be executed to completion on any input the campaign has.

Two further findings from this unit:

1. **VIT emitted a five-argument wrapper over a three-argument bridge, silently.**
   With no view strategy configured, `vit interface` produced
   `CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(ExtDLL_view))` —
   `LocalVar` and `ErrVar` accepted and not forwarded, no diagnostic anywhere.
   `LocalVar%iStatus` is the guard on the whole initialisation branch, so that
   bridge could never have loaded the library it exists to call, and it compiles
   and links. Same shape as unit #1's dropped ALLOCATABLE attribute, one level
   up. Fixed in VIT (`f8ab74f`), additively, red-tested both directions.
2. **`Examples/DISCON.IN` is gitignored, so a crashed run can reconfigure the
   gate where no clean-tree check can see it.** The probe's first draft restored
   it in a `finally`, and a `finally` does not run through SIGSEGV; it left
   `Ext_Mode = 1` in the gate's own input with `git status` clean. The probe now
   restores from a parent process.

Units #1–#4 remain closed. **Escalation 3 is raised** (`blocked_substrate`), and
escalation 2 still stands.

---

**Unit #4 `Conv2UC` is `integrated` and CLOSED.** Every layer green with its own
red test — except the gate, whose red test FAILED and is committed as the
finding.

**THE GATE IS BLIND TO A PROCEDURE CALLED 1,333,146 TIMES.** `Conv2UC` converts
4,558,823 characters across the 27 scenarios; it is among the hottest procedures
in the controller. Two perturbations of the integrated C++ were built and gated
and **each moved 0 of 5,252,000 values** — `ch - 32` → `ch - 31`, and the guard
forced false so no conversion happens at all. The cause is structural: every
value the unit produces is consumed by an equality test against another of its
own outputs (`FindLine` upper-cases both the expected parameter name and the
file's word before comparing them), so a perturbation lands on both operands and
equality survives.

This is the third distinct shape of P9 in four units and the sharpest: #1 was a
line no scenario reached, #3 was an argument constant in every scenario, and
this is an **executed line whose result is cancelled downstream**. A hit count
of 1.3M says nothing about observability, and `coverage/line_coverage.json`
cannot express the difference.

Two further findings:

1. **The differential harness had no string kind, and 20 of 69 units need one.**
   `vit_harness.py` reported the CHARACTER argument comparable-but-held-constant
   and then died on `C parameter 'Str' is not in the mapped signature`. With
   P11 and P12 mandatory, none of those 20 units had a path to close. Built
   rather than worked around: loop `9bee569`. A CHARACTER **array** dummy is
   still refused, with its reason.
2. **Removing a restatement raised the mutation score from 0.696 to 1.000, for
   the second time in the campaign.** Transcribing `LEN_TRIM` literally adds six
   mutable sites computing a quantity nothing downstream can read; five survived
   as out-of-bounds reads no value comparison can see. Unit #1 named a size
   once; this named a loop bound once.

Units #1 `AddToList`, #2 `ColemanTransform` and #3 `ColemanTransformInverse`
remain closed. Escalation 1 is answered; **escalation 2 still stands** — a dead
unit whose signature does NOT cross has no path to harness, mutation or gate,
and `loop/done.py` still has no branch for `integrated_unexercised`.

Next: the lowest-order remaining unit in `plan.json`. Confirm it against
`plan.json` before starting, including its `phase` and `proposed_verification`
fields — those are hypotheses, not facts. Run
`python3.12 scripts/done_check.py <Unit>` before setting any disposition.

## Counts

5 attempted / 4 integrated / 0 integrated_unexercised / 0 out_of_scope /
0 deferred / **1 blocked**.

69 units in `plan.json`; 64 remain.

**3 of the 5 attempted units are invisible to the gate**, for three different
reasons: `AddToList` is never called, `Conv2UC` is called constantly and
cancelled, and `ExtController` is never called *and* cannot be run at all. Each
carries a green gate artifact committed beside the red test that says it
constrains nothing.

**`ExtController` is the only unit so far with no `harness/` and no `mutation/`
artifact.** That absence is the finding, not an omission: P11 and P12 are
mandatory for every unit here, and neither can be produced without a bridge and
a runnable reference. `done_check.py` reports both FAIL and that verdict is kept.

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

### The campaign's VIT: `d07a716` → `22086e8` → `37f8bdf` (unit #1) → `f8ab74f` (unit #5)

**`f8ab74f` (unit #5)** makes `generate_fortran_wrapper` say, on stderr, which
derived-type arguments it accepted and did not forward. `dropped_derived_args`
already existed and only `test_validate` asked it, so `vit interface` — the
command that SHOWS you the wrapper — and `vit integrate` — the one that SHIPS it
— both emitted a bridge with fewer arguments than the wrapper, in silence. On
`ExtController` that was `LocalVar` and `ErrVar`, and `LocalVar%iStatus` is the
guard on the entire initialisation branch.

**Additive: no generated byte changes**, so no artifact already measured against
an earlier revision is invalidated — units #1–#4 are unaffected. Red-tested in
both directions (fires on `ExtController` with the strategies unset, silent on
`ColemanTransform` and on `ExtController` once they are set). 937 tests pass,
935 before plus the two added with it.

The three earlier moves happened during unit #1. Neither of the first two was an
upgrade; both are fixes that unit produced.

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

### Conv2UC — every layer, and the one red test that failed

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 1, `ROSCO_Helpers.f90:1118` | 63/63 `IDENTICAL` | no-op stub → 31/63 `OUT_TOL`; green restored |
| differential harness vs clean Fortran | 118 checked, 0 failed | no-op stub → 27/118 failed |
| mutation score | 14/14 behavioural killed, 1.000, 0 declared equivalent (4 nocompile EXCLUDED) | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 118 checked, 0 failed | `LEN(Str)` → `LEN(Str) - 1` → 24/118 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED, twice — 0 of 5,252,000 moved** |

2 of the 14 mutation kills are CRASHES rather than case mismatches (both index
before the buffer), so the killed-by-comparison count is 12 of 14. The 4
excluded nocompile mutants are all `harness/cppmutate.py` mangling
`static_cast<unsigned char>` — 22% against the 25% at which `vit_mutate.py`
refuses to score.

The signature crosses whole: `CHARACTER(*), INTENT(INOUT) :: Str` becomes
`CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: Str(*)` plus
`INTEGER(C_INT), VALUE :: len_Str`, with copy-in/copy-out in a wrapper that
declares the dummy exactly as the original does. `vit interface` was read
attribute by attribute before any C++ was written.

**For a CHARACTER output a kernel PASS *is* a bit-identity claim** — KGen's
generated comparison has no tolerance branch at all. Unit #3's caveat is a fact
about REAL fields, not about the verdict line; read the generated comparison for
each new element type.

### ExtController — every layer, and why none of them ran

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT ATTEMPTED** — no live call site, so nothing to capture | n/a; unit #1's lesson is that this is deadness, not a tool defect |
| differential harness vs clean Fortran | **IMPOSSIBLE** — VIT emits no bridge, and the oracle segfaults | n/a |
| mutation score | **IMPOSSIBLE** — no translation to mutate | n/a |
| post-integration harness | **IMPOSSIBLE** — nothing integrated | n/a |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED, twice — 0 of 5,252,000 moved** |

Both gate red tests carry `replacements: 1`, `perturbed: true`,
`revert_verified: true`, `residual_dirt: []`. The second perturbs toward
**absence** — first statement replaced by `RETURN`, making the unit a no-op —
which is the form that shows the gate is blind to the *unit*, not merely to the
perturbation chosen.

`gcov` says the same thing independently: **0 of 28 executable lines, in all 27
scenarios.** The denominator had to be regenerated to get it — see the coverage
gap in Open below.

## Open

- **ESCALATION 3, RAISED: `ExtController` is `blocked`, and the blockage is in
  the substrate and in the fixtures — not in the function.** Three independent
  things would have to change, and only the third decides:
  1. **VIT must map `CHARACTER(:), ALLOCATABLE` in a view struct**, with
     *reallocating* assignment from C++ (`ErrVar%ErrMsg = RoutineName//':'//
     TRIM(ErrVar%ErrMsg)` changes its length). `TYPE(ErrorVariables)` has four
     fields and exactly one blocks it. **37 of the 69 units take a
     `TYPE(ErrorVariables)` dummy**, so this must be built regardless of what
     happens to `ExtController` — the same argument that carried unit #1's
     descriptor bridge and unit #4's string kind. It does **not** unblock this
     unit on its own.
  2. **`TYPE(ExtDLL_Type)` must cross**, for the `LoadDynamicLib` permanent
     bridge `plan.json` already declares: five fields including `TYPE(C_PTR)`,
     `TYPE(C_FUNPTR) :: ProcAddr(3)` and `CHARACTER(1024) :: ProcName(3)` — a
     CHARACTER array, which the differential harness also still refuses. X1
     forbids inlining `LoadDynamicLib` to avoid it. Blocks this unit only.
  3. **An oracle must be constructed** — a minimal Bladed-style library
     exporting a `DISCON` entry point plus a scenario setting `Ext_Mode = 1`.
     That is an *addition*, which P5 permits, but it adds a gate scenario and so
     changes the gate's compared count and baseline set. SPEC §8.4 makes that
     the Driver's call, not a unit's. **Until it exists there would be a
     translation and nothing to check it against.**
- **`vit analyze-types --fix character` must not be used to close this gap.**
  It is what VIT's own error message suggests, and it rewrites ROSCO's type
  definition to a fixed length — changing `LEN(ErrVar%ErrMsg)`, which sizes
  `avcMSG`. That is a behavioural change to the oracle (P7). Recorded here
  because the suggestion is in the tool's output and will be read again.
- **A gitignored file can reconfigure the gate where no clean-tree check can
  see it.** `Examples/DISCON.IN` is ignored at `.gitignore:78`. A run that dies
  on a signal cannot restore it from a `finally`, and `git status` stays clean,
  so `done.py`'s P2 cannot catch it. `gate.py` already snapshots and restores
  these; nothing else did. The general form: **any restore that must survive a
  crash belongs in a parent process, not in the process that crashes.**
- **`coverage/line_coverage.json` cannot distinguish "never ran" from "never
  instrumented".** `scripts/coverage.py` stores only lines with a NON-ZERO hit
  count, so both are the same empty dictionary. Four files read as empty today —
  `ExtControl.f90`, `Constants.f90`, `ROSCO_Types.f90`, `ZeroMQInterface.f90` —
  and only re-running with the denominator printed tells them apart
  (`ExtControl.f90` is genuinely `0/28`, 27 times). The per-scenario log line
  already prints `N/M`; the *artifact* drops M. Storing the denominator would
  close it, and would be an addition.
- **A hot line is not an observable line, and nothing in the campaign's coverage
  data can say which is which.** `Conv2UC` runs 1.3M times and no gate
  perturbation of it moves any output, because its result is only ever compared
  against another of its own results. The verification ledger (E5.2) needs a
  column for this that is distinct from both "unexercised" and "argument held
  constant" — it is a *consumer* property, not a producer one, and neither
  coverage nor the gate's own red test in its usual form can detect it. The
  general instrument found here is to perturb toward ABSENCE (`if (false && …)`)
  rather than toward a different answer; see DECISIONS.md for the candidate
  method amendment this suggests, which is flagged and not made.
- **The differential harness refuses a CHARACTER ARRAY dummy.** `char[]` has one
  extent; `CHARACTER(*), DIMENSION(:)` has two — element width and element
  count. `FindLine`, `ParseInAry_Opt`, `ParseDbAry_Opt` and the four other
  `ParseInput_*_Opt` units all take one, so this is the next instrument gap on
  the critical path, and it is refused with a reason rather than approximated.
- **The harness varies a string's length over {1, 3, 8}, not over the 200
  `MaxParamLength` gives it.** `_extent_plan` assigns a lone free extent the
  single value 3; the string stage adds 1 and 8 without touching that plan,
  deliberately, because widening `_extent_plan` would change the case set of
  every unit already scored. A defect that only appears past the eighth
  character is not covered.
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
  of 33 mutants on `AddToList` (24%) were that, one point under the 25% at which
  `vit_mutate.py` REFUSES to score. **Seen a second time at unit #4**: 4 of 18
  (22%) on `Conv2UC`'s two `static_cast<unsigned char>`s. Two units, both within
  three points of the refusal threshold, for a reason with nothing to do with
  either translation. This is now a pattern rather than an anecdote and belongs
  in the loop repo's mutator: mask template argument lists the way `cppmutate`
  already masks string and character literals.
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

## Done-condition, unit #5

`python3.12 scripts/done_check.py ExtController --baseline 05a64cd` returns
**INCOMPLETE, 11 of 13**, on a clean tree. P11 and P12 FAIL because
`harness/ExtController.postintegration.json` and `mutation/ExtController.json`
do not exist and cannot be produced without a bridge and a runnable oracle.
**That verdict is correct and is kept** — a `blocked` unit cannot reach COMPLETE,
and `loop/driver.py` escalates `blocked_substrate` on the disposition separately
(SPEC §8.4). Transcript and reasoning: `evidence/ExtController/done_check.txt`.

Worth carrying: the first attempt piped the run into that file, and **creating
the file made the tree dirty**, so the artifact recorded `P2 FAIL dirty_tree` and
10/13 — describing itself rather than the unit. A done-condition capture has to
be taken before the file that captures it exists, or written as a transcript.
