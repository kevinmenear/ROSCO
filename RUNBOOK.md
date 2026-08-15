---
spec_version: 1
invariant_hash: sha256:e4e6b553a5588907
instantiated: 2026-08-10
campaign: rosco-r2
---

# Runbook — invariant layer

GENERATED from SPEC.md. **Do not paraphrase and do not edit casually.**

Editing this layer changes `invariant_hash`, which the Driver reads as a proposed amendment
to the method itself and escalates (`method_amendment_proposed`). That is the intended way
to promote a rule learned here into the spec — it is a signal, not a violation.

Target-specific procedure belongs in the target layer, never here.

`Status here` starts at `inherited, unexercised` for every rule. Update it to
`exercised at unit #N` the first time the rule actually bears on the work. Rules still
unexercised after N units are the P4 report: inherited, never tested against this target.

## Principles

### P1 — State lives on disk, not in context
**Implements:** P1
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P2 — The loop body is mechanical
**Implements:** P2
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P3 — A green result must be able to name what it compared, and be able to go red
**Implements:** P3
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P4 — Copy and hash-verify; never re-implement from prose
**Implements:** P4
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P5 — Close gaps by addition, never modification
**Implements:** P5
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P6 — Absence must not render as a value
**Implements:** P6
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P7 — The oracle is the original source
**Implements:** P7
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P8 — Dispositions, not blockers
**Implements:** P8
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P9 — Coverage is not visibility
**Implements:** P9
**Origin:** SPEC.md §3 (method)
**Status here:** inherited, unexercised

### P10 — A pass built from an empty set must name the set and prove it could have been non-empty
**Implements:** P10
**Origin:** SPEC.md §3 (method)
**Status here:** **EXERCISED — five instances in one session, all of them
ad-hoc probes rather than registered checks.** `git log --not --remotes`
reported nothing unreachable while the clone was 2 commits ahead; a scan reading
`Procedure.body` reported 0 file-valued units against an attribute that does not
exist; a C digest with a bad format string reported 740/740 identical having
compared nothing; `ahead=0` read against a remote-tracking ref whose remote was
a deleted `/private/tmp` path; and a bundle census filtered on branch names it
did not include, reporting the wrong sha for one repo and none for another. In
every case the campaign's own check was right and the ad-hoc probe was wrong.

A positive control is necessary and not sufficient, and one probe showed both
halves: `file_params_from` was repaired by a control, then still reported "0 of
52 pending units have file-valued inputs" and missed five, because its notion of
a filename is a bare dummy while the reachable ones are an array element, a
derived-type field and two locals. Naming the set is the other half.

## Contracts

### K1 — The last action of every unit is a commit that brings state current
**Implements:** K1
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### K2 — Every unit ends in exactly one disposition, which records its evidence
**Implements:** K2
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### K3 — Every evidence reference must resolve to an artifact that exists
**Implements:** K3
**Origin:** SPEC.md §4 (method)
**Status here:** inherited, unexercised

### K4 — All work outside a unit's own cycle is bracketed under a declared phase
**Implements:** K4
**Origin:** SPEC.md §12 (method)
**Status here:** inherited, unexercised

## The cycle — every unit, in order, no steps skipped

### C1 — Confirm the function's plan.json metadata against evidence (§6.4). Re-classify if wrong
**Implements:** C1
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C2 — Select the call site and test case from coverage data
**Implements:** C2
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C3 — Extract runtime state at a live call site
**Implements:** C3
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C4 — Scaffold the translation with the tool, never from scratch
**Implements:** C4
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C5 — Write the translation
**Implements:** C5
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C6 — Verify against captured state
**Implements:** C6
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C7 — Integrate, generating the wrapper
**Implements:** C7
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C8 — Rebuild
**Implements:** C8
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C9 — Gate
**Implements:** C9
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C10 — Commit the function
**Implements:** C10
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C11 — Commit the state (§8.3)
**Implements:** C11
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### C12 — If a defect passed a layer it should have failed, record it NOW, with the wrong artifact, before fixing it
**Implements:** C12
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

## Prohibitions

### X1 — Never inline a callee to route around infrastructure
**Implements:** X1
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### X2 — Never work around a tool bug you control
**Implements:** X2
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### X3 — Never change a verification default mid-run
**Implements:** X3
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### X4 — Never take a green result at face value on first use
**Implements:** X4
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

## Phase exit criteria

### E1.1 — Clean build from a pinned upstream commit
**Implements:** E1.1
**Origin:** SPEC.md §5 (method)
**Status here:** inherited, unexercised

### E1.2 — Translation-phase flags applied to every compiler and asserted in the build log
**Implements:** E1.2
**Origin:** SPEC.md §5 (method)
**Status here:** inherited, unexercised

### E1.3 — All five red tests in §5.3 observed red
**Implements:** E1.3
**Origin:** SPEC.md §5 (method)
**Status here:** inherited, unexercised

### E1.4 — Reset/restore cycle round-trips bit-identically
**Implements:** E1.4
**Origin:** SPEC.md §5 (method)
**Status here:** inherited, unexercised

### E1.5 — Copy manifest complete, every entry hashed or flagged as re-implemented
**Implements:** E1.5
**Origin:** SPEC.md §5 (method)
**Status here:** inherited, unexercised

### E1.6 — Telemetry records a start and an end event with a correct exit code for a deliberately failing
**Implements:** E1.6
**Origin:** SPEC.md §5 (method)
**Status here:** inherited, unexercised

### E2.1 — Inventory complete; every procedure classified in-scope, out-of-scope, or dead, with reasons
**Implements:** E2.1
**Origin:** SPEC.md §6 (method)
**Status here:** inherited, unexercised

### E2.2 — Dependency graph built; work order derived from it, not from file order
**Implements:** E2.2
**Origin:** SPEC.md §6 (method)
**Status here:** inherited, unexercised

### E2.3 — Coverage data generated from a clean build and committed
**Implements:** E2.3
**Origin:** SPEC.md §6 (method)
**Status here:** inherited, unexercised

### E2.4 — plan.json complete, with phase and proposed_verification marked explicitly as hypotheses
**Implements:** E2.4
**Origin:** SPEC.md §6 (method)
**Status here:** inherited, unexercised

### E2.5 — At least one non-leaf function identified as an early infrastructure probe
**Implements:** E2.5
**Origin:** SPEC.md §6 (method)
**Status here:** inherited, unexercised

### E3.1 — Gate runs, prints the compared count next to the pass count, and exits non-zero on mismatch
**Implements:** E3.1
**Origin:** SPEC.md §7 (method)
**Status here:** inherited, unexercised

### E3.2 — Gate observed red under a deliberate perturbation
**Implements:** E3.2
**Origin:** SPEC.md §7 (method)
**Status here:** inherited, unexercised

### E3.3 — Every scenario confirmed alive: no rejected runs, no all-constant channel sets
**Implements:** E3.3
**Origin:** SPEC.md §7 (method)
**Status here:** inherited, unexercised

### E3.4 — Branch coverage of the gate measured; gaps recorded in a known-gaps table
**Implements:** E3.4
**Origin:** SPEC.md §7 (method)
**Status here:** inherited, unexercised

### E3.5 — Baselines generated from clean pre-integration source, committed
**Implements:** E3.5
**Origin:** SPEC.md §7 (method)
**Status here:** inherited, unexercised

### E3.6 — Subset-baseline generation exists and does not disturb existing baselines
**Implements:** E3.6
**Origin:** SPEC.md §7 (method)
**Status here:** inherited, unexercised

### E4.1 — Disposition recorded, from the vocabulary, with the evidence it rests on
**Implements:** E4.1
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### E4.2 — Every evidence reference resolves to a committed artifact
**Implements:** E4.2
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### E4.3 — The gate ran, and its compared count is recorded in a committed artifact rather than in prose
**Implements:** E4.3
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### E4.4 — A state commit brings STATUS.md, DECISIONS.md and the dataset current
**Implements:** E4.4
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### E4.5 — The generated differential harness ran against the integrated build, not only the
**Implements:** E4.5
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### E4.6 — The mutation score for this unit is at or above the campaign's stated threshold, every
**Implements:** E4.6
**Origin:** SPEC.md §8 (method)
**Status here:** inherited, unexercised

### E5.1 — Claims audit run; every verification claim resolves to an artifact that exists
**Implements:** E5.1
**Origin:** SPEC.md §10 (method)
**Status here:** inherited, unexercised

### E5.2 — Verification ledger produced: one row per unit, naming the instrument and what it could not see
**Implements:** E5.2
**Origin:** SPEC.md §10 (method)
**Status here:** inherited, unexercised

### E5.3 — Every unit whose only verification ran pre-integration has a post-integration check or a
**Implements:** E5.3
**Origin:** SPEC.md §10 (method)
**Status here:** inherited, unexercised

### E5.4 — Branch coverage of the gate recorded as a table rather than claimed
**Implements:** E5.4
**Origin:** SPEC.md §10 (method)
**Status here:** inherited, unexercised

### E5.5 — Every translated unit turned red by a deliberate perturbation
**Implements:** E5.5
**Origin:** SPEC.md §10 (method)
**Status here:** inherited, unexercised

### E5.6 — Full gate re-run from a clean build against baselines regenerated from clean source
**Implements:** E5.6
**Origin:** SPEC.md §10 (method)
**Status here:** inherited, unexercised

---

# Runbook — target layer

Everything below is specific to **rosco-r2**. It is *earned, not written in
advance*: add an entry when something is learned, never in anticipation.

This layer is expected to change constantly. The invariant layer above is not —
editing it changes `invariant_hash`, which the Driver reads as a proposed
amendment to the method itself.

## Commands

Each entry below was RUN, in the `vit-dev` container, on 2026-08-10, and records
what happened -- not what documentation says should happen. `TODO` means nobody
has executed it yet.

The container mounts `~/Artifacts/vit_translation` at `/workspace`, so this tree
is `/workspace/ROSCO-r2`.

- **A GUARD THE REFERENCE DOES NOT HAVE IS A SITE NO INPUT CAN KILL, AND THE
  MUTATION SCORE IS WHAT FINDS IT.** Unit #43. Four of the first sweep's eleven
  survivors were at one line I wrote defensively:

  ```
  std::vector<double> row(cols > 0 ? (size_t)cols : 0);   cols >= 0 / > 1 / : 1
  for (int32_t j = 0; j < cols; ++j) row[j] = ...;        j <= cols
  interp1d_c(..., row.data(), cols, ...)                  <- hides the fourth
  ```

  `cols` is an extent: R5 never draws it below 3 and the view populator never
  makes it negative, so all three mutants of the guard spell the same number and
  no input can distinguish them. The fourth is worse than equivalent -- passing
  `cols` rather than the buffer's own length as `SIZE(yData)` means a widened
  loop bound writes past the vector and the extra element is never read.

  ASK BEFORE WRITING THE LINE, not after the sweep: *does this spelling offer a
  mutant no input can kill?* Here the repair was no guard, `push_back`, and
  `row.size()` as the length -- which is also what the reference's `SIZE(yData)`
  denotes. Unit #37's `std::max` standing is the same rule seen from the other
  side. Raised in DECISIONS.md as a proposed method amendment: it belongs beside
  X1/X2, being about how a translation is written rather than about ROSCO.

- **A DEAD ARM WHOSE SCENARIO EXISTS. READ THE SCENARIO, NOT THE GUARD.** Unit
  #43, and it changes what the zero means.

  ```
  Controllers.f90:1003-1012  StC_Mode == 2 arm      0 hits, all 27 scenarios
  gate red test on that arm  0 of 5,252,000         revert-verified
  vit_sim.py scenario 24     StC_Mode = 2, StC_Group_N = 1, Ind_StructControl 8
  DISCON.F90:136 under 24    0 hits   |  DISCON.F90:141 under 24   8,000
  ```

  "No scenario configures it" is FALSE: one does, and it never reaches DISCON's
  main block. Units #23 and #26 had already measured why -- `Read_OL_Input`
  returns on the absent `Examples/example_inputs/OL_Mode2_Input.dat` -- and
  STATUS.md has carried scenario 24 executing no controller code as an E3.3
  failure since phase 3. The difference decides whether widening the scenarios
  could help. Put BOTH sentences in the artifact with `--note`; a reader who
  opens a red test recording 0 needs the expectation beside the number.

- **WHEN A UNIT'S OUTPUTS ARE INOUT STATE THAT PERSISTS BETWEEN CALLS, A NO-OP
  PASSES EVERY STEADY-STATE CASE AND THE KERNEL'S POWER IS THE TRANSITIONS.**
  Unit #43, third instance of the window rule after #41 and #42, and the
  sharpest.

  ```
  the unit as a no-op            fails 1 of 62   StructuralControl.0.0.20002
  the avrSWAP copy loop deleted  passes 62 of 62      <- 'avrswap' is NOT a
  the first step constant moved  fails 35 of 62          compared field
  ```

  `avrSWAP` and `LocalVar%StC_Input` both persist in the driver, so from
  invocation 20,003 onward the values the unit writes are already there on entry.
  The single case that fails is the transition, and it is in the corpus because
  the window was aimed at `23,999 - 3,998 + 1` -- arithmetic over a coverage
  count, written into `vit.yaml` with the cost stated BEFORE the extraction ran.

  AND CHECK WHICH OUTPUTS THE KERNEL ACTUALLY COMPARES BEFORE TRUSTING ITS
  GREEN. Read the field-name column of `verify_fields.csv`; a name absent from
  it is a statement the kernel cannot fail on. Then MEASURE it with a stub --
  the field list is a claim about the comparison, and only a stub turns it into
  a claim about what can be caught.

- **AN ARM R6 CROSSES INTO IS NOT AN ARM R6 ENTERS, AND THE GENERATOR SAYS WHICH
  IS WHICH.** Unit #43. `StC_Mode = 2` with `StC_Group_N = 1` is in the corpus at
  the full cross product; what those cases lack is `Ind_StructControl(1) > 0`,
  and the generator prints the reason itself:

  ```
  note: CntrPar_Ind_StructControl[I_GROUP - 1]: subscript not traceable to a
        parameter -- its index is NOT exercised by R5
  ```

  A quantity whose subscript is the loop counter is not a knob, so no widening
  of R6 reaches it. That is an R11 baseline state's job. Read those `note:`
  lines in the generation output before explaining a survivor list -- they name
  the quantities no ladder will move.

- **A STUB RUNNER'S `git checkout` RESTORES THE COMMITTED TRANSLATION, SO AN
  UNCOMMITTED EDIT TO IT IS DESTROYED SILENTLY.** Unit #43.
  `evidence/<Unit>/run_harness_redtest.sh` -- the shape every unit copies --
  ends with `trap 'git checkout -- "$CPP"' EXIT`. A translation edit made and
  not committed before the red test runs is gone when it finishes, and nothing
  reports it: the red test's own number is correct, and only `grep` on the file
  afterwards shows the old code back. Same shape as `restore_integrated.sh`'s
  stated warning, one directory over. COMMIT BEFORE RUNNING ANYTHING WHOSE
  CLEANUP IS A CHECKOUT -- a translation edit is an artifact too, not a
  work-in-progress waiting for its measurement.

- **A STATE MACHINE'S CAPTURE WINDOW IS ARITHMETIC OVER THE SCENARIO'S PATCH
  DICT, AND THE CHEAPEST CONFIRMATION IS A STUB THAT FAILS ONE NAMED CASE.**
  Unit #42. `Startup` runs 11,999 times in scenario 9 and is finished by
  invocation 1,801; a start/middle/end window would spend two ranges on
  `SU_Stage == 0`, where the body writes nothing.

  ```
  Examples/vit_sim.py::run_scenario_9   dt 0.025, SU_FW_MinDuration 5,
                                        SU_LoadRampDuration 10 10,
                                        SU_LoadHoldDuration 10 10
  invocation  201  t = 5    stage 1 -> 2   coverage :594  1 hit  / :608  199
  invocation 1001  t = 25   stage 2 -> 3   coverage :602  2 hits / :610  800
  invocation 1801  t = 45   stage 3 -> 0   coverage :619  1 hit
  invocation: 0:0:1-20,0:0:195-215,0:0:995-1015,0:0:1795-1815      83 cases
  ```

  Compute the transitions from the scenario's OWN patch dict and cross-check
  each against a coverage hit count BEFORE writing the window. Then delete the
  arm that fires once and confirm the kernel fails exactly the case index the
  arithmetic named -- here `Startup.0.0.1801`, 82 of 83 passing. A midpoint
  window reports the same green over a corpus in which that arm is dead.

- **A SURVIVOR LIST IS NOT A LIST OF SITES. ASK WHICH ARM OF WHICH CALLEE THE
  CORPUS REACHED.** Unit #42, and it is the difference between one baseline
  state and five.

  ```
  first sweep   92 of 106, and six survivors at ONE call:
     arith 'LSST + SU_LoadRampDuration' -> '-'      swap 'S-1' -> '1-S'
     arith 'S - 1' -> 'S + 1'                       const the two -1s
     const '1.0' -> '2.0'   (sigma's y1)
  same sweep    drop_factor on sigma's y0   KILLED on 624 cases
  ```

  `sigma(x, x0, x1, y0, y1)` returns `y0` outright when `x < x0`, and every
  stage-2 case had `Time < SU_LoadStageStartTime`. Killing the argument a
  function RETURNS while missing every argument it does not is the signature of
  a corpus that reached one arm, not of six independent holes. Two states, not
  one: `x < x0` and `x > x1` are different arms and one `Time` cannot be in both.

- **`\&\&` INSIDE SINGLE QUOTES REACHES THE COMPILER AS BACKSLASHES, AND ON A
  RED TEST WHOSE EXPECTED RESULT IS ZERO THAT IS INDISTINGUISHABLE FROM THE
  ANSWER.** Unit #42.

  ```
  --perturb-to '(false \&\& LocalVar->SU_RotSpeedF < ...)'
  PERTURBED BUILD FAILED -- no red test was performed          exit 2
  --perturb-to '(false && LocalVar->SU_RotSpeedF < ...)'
  RED TEST FAIL: perturbation moved 0 of 5252000               exit 1
  ```

  Both end with the tree reverted and nothing moved. `scripts/gate.py`
  distinguishes them and prints which; a tool that reported the build failure as
  a zero would have manufactured a corroboration of the coverage data. Read the
  verdict line, not the count -- and note gate.py reverts unconditionally, so
  the failed attempt costs one second and no cleanup.

- **`vit_mutate.py`'s SURVIVOR IDS ARE COMPUTED FROM THE LOWERCASED UNIT NAME,
  SO `cppmutate.mutants('Startup', src)` MATCHES NONE OF THEM.** Unit #42, and
  it is 0-of-14 rather than a partial match, which is what made it obvious.
  `vit_mutate.py:296` calls `mutants(args.unit.lower(), original)` and `_mid`
  hashes the unit name in. To locate a survivor in the source:

  ```
  ms = cppmutate.mutants('startup', open(cpp).read())     # lower-cased
  {m.mid: (m.line, m.operator) for m in ms}
  ```

  The artifact records `id`, `operator`, `before` and `after` and NOT the line,
  so this is the only way to turn a survivor into a source position -- and the
  positions are what partition the list.

- **`harness/ranges.toml` NARROWED EVERY LADDER EXCEPT THE ONE READ OUT OF THE
  REFERENCE, AND THE COST WAS 31 CASES OF UNDEFINED BEHAVIOUR SCORED AS
  FAILURES.** Unit #41, `translation-loop f92fb9f`. `predicate_knobs_from`
  derives a knob's values from the reference's own predicates and never consults
  the signature, so a stated range was recorded, reviewed and ignored.

  ```
  LocalVar_SD_Stage = { lo = 0, hi = 3 }     stated
  PREDICATE KNOB: LocalVar_SD_Stage at [-1.0, 0.0, 1.0, 2.0]     generated
  HARNESS FAIL: checked 14621  failed 31
    case 9475 LocalVar.SD_MaxPitchRate: ref 176b14841ba98540 != got 0000000000000000
  ```

  The "reference" values are uninitialised memory: at `SD_Stage = -1` the Fortran
  reads one element before its allocation. **A red result that is really an
  undefined reference cannot be told from a translation defect**, and
  `vit_mutate.py` refuses to score against a red baseline, so nothing downstream
  could have closed either. Only a `stated:` bounds_source narrows — filtering on
  the +/-1e3 default would delete knob values for every unit already scored, and
  that is the positive control in the new test.

- **A CALLEE'S IDENTITY CASE TURNS A COMPUTED SIDE BACK INTO AN INPUT, AND THAT
  IS HOW A THRESHOLD BOUNDARY THAT NO LADDER CAN REACH GETS REACHED.** Unit #41,
  and it generalises to every unit in ROSCO that thresholds a filtered signal.

  ```
  IF (LocalVar%SD_GenSpeedF > CntrPar%SD_MaxGenSpd)   <- left side is COMPUTED
  R6's relational-pair rule sets one side FROM the other and needs BOTH to be
  inputs, so the equality is unreachable and `>` -> `>=` survives.

  LPFilter's init arm:  1/(2+c) * ( -(c-2)x + cx + cx ),  c = CornerFreq*DT
      = x in real arithmetic for every c
      = x BIT-EXACTLY at c = 0, where the coefficients are 2, -2, 0, 0
        and the expression is (1/2)*(2x) -- both operations exact
  ```

  `SD_GenSpdCornerFreq = 0` with `iStatus = 0` makes the filter the identity, and
  the mutant is KILLED rather than declared. Two of this unit's thirteen baseline
  states are that, and they are the difference between a declaration and a
  measurement.

- **`run_if_time_remains.sh` GUARDS THE DISPATCH DEADLINE AND KNOWS NOTHING ABOUT
  THE 600-SECOND TOOL CEILING, WHICH IS A DIFFERENT NUMBER WITH THE SAME
  CONSEQUENCE.** Unit #41. Asked for 450 seconds with 8,400 remaining, it started
  a 69-mutant sweep that needed ~800.

  ```
  Command timed out after 10m 0s
  mutate_guarded: REFUSING TO CLEAR THE MARKER.
    is       eff8060a8d49496e6e92876df53712f28f9108d8
    intended 07c2967fde7fa8318b3bebd7f1ea2eb361402e7c
  docker exec vit-dev pgrep -af vit_mutate.py     <- still running, after the
                                                     tool had returned
  ```

  **Estimate against 600, not against the deadline**, and split until every part
  fits: this unit's four parts are 460, 408, 273 and 185 seconds. `pkill` the
  orphan BEFORE restoring the file, or the sweep writes the mutant back.

- **A HOST WRITE CAN RACE A CONTAINER READ ACROSS THE BIND MOUNT, AND THE
  EVIDENCE IS AN OBJECT FILE THAT IS THE RIGHT SIZE AND HAS NO SYMBOLS.** Unit
  #41, and it is this file's mtime hazard met in a third form.

  ```
  /usr/bin/ld: ControllerBlocks.f90.o: undefined reference to `shutdown_c'
  nm vit_integration_shim.o    ->  .bss .comment .data .text and NO shutdown_c
  wc -c vit_integration_shim.cpp   ->  451
  recompile from the SAME bytes    ->  0000000000000000 T shutdown_c
  ```

  `harness.sh` writes that .cpp by redirecting one `docker exec`'s stdout to a
  HOST path and compiles it in a SECOND `docker exec`; the two are ordered and
  the write was still not visible. The failure names the unit under test, so it
  reads like a broken integration. **`nm` the object, not the source** -- only
  the object was wrong, and it takes seconds.

- **A PERTURBATION MUST BE SCOPED TO THE WRAPPER IT NAMES, BECAUSE THE GENERATED
  LINE IS IDENTICAL IN ALL OF THEM.** Unit #41.
  `CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)` stands FIVE
  times in `ControllerBlocks.f90` -- once per integrated unit whose view argument
  carries scalar outputs. A whole-file replace perturbs five wrappers, and the
  first attempt asserted on the count and edited nothing, after which the harness
  printed `POST-INTEGRATION RED TEST FAILED (stayed green)` against an
  UNPERTURBED tree. That verdict was correct about what it measured and would
  have been read as a defect in the wrapper. Scope the edit to the unit's own
  `SUBROUTINE ... END SUBROUTINE` block and exit non-zero if the line is not
  there exactly once.

- **A REFERENCE CAN HAVE NO ANSWER FOR ITS OWN RETURN VALUE, AND THE PIN THAT
  LOOKS OBVIOUS THROWS AWAY THE ARM INSTEAD OF THE ANSWER.** Unit #39,
  `translation-loop 04975cf`. `ResController` assigns its result in the ELSE arm
  only; on `IF (reset)` it returns whatever the slot holds.

  ```
  HARNESS FAIL: checked 3532  failed 1763  inadmissible 0
    case 17 vit_result: ref 777cb0d2ffff0000 != got 0000000000000000   <- a pointer
    case 18 vit_result: ref 10cf0af41fb70000 != got 0000000000000000
  ```

  `reset = { values = [0] }` makes that green and deletes four writes the
  harness is the ONLY instrument that reaches -- zero hits in all 27 scenarios,
  `restart` F in all 62 kernel cases, and the gate red test on the arm moves 0
  of 5,252,000. **Exclude the ANSWER, not the ARM**: `no_oracle` now names
  `vit_result`. The same corpus goes `failed 1763` -> `failed 0`, which is also
  the proof that all 1763 were that one output, since the mismatch list is
  truncated at sixteen.

  State the cost against THIS unit rather than in general: here the ELSE arm
  stores the returned value into a compared out-parameter, so the arithmetic is
  not excluded. A unit whose result is not mirrored anywhere pays much more.

- **THE MUTATION SWEEP'S OWN STDOUT IS A CENSUS AND IT IS FREE. REDIRECT IT EVEN
  ON A RUN YOU EXPECT TO DISCARD.** Unit #39. The first sweep scored 0.9275 and
  its per-mutant kill counts are bimodal:

  ```
  killed on    12 of 3532 cases   21 mutants   <- the ELSE arm's arithmetic
  killed on  1763 of 3532 cases    8 mutants   <- the reset arm's four writes
  killed on  3532 of 3532 cases    7 mutants   <- structural
  ```

  1766 cases run the ELSE arm and twelve of them let its arithmetic reach a
  compared output. Unit #34 built `clamp_census.csv` to learn that about
  `PIDController`; this run printed it and the first one was not captured to a
  file, so it had to be re-run to be quoted.

  The cause is `[PIDController]`'s cause 2 met a second time, and the pin
  differs in one place worth copying: `minValue = { lo = -1e9, hi = 0 }`, not
  `hi = -1e-3`, because all 14 `Examples/DISCON*.IN` carry `PC_MinPit = 0.000`
  and a pin that excludes the value the whole program uses is a narrowing
  nobody needed.

- **BEFORE CALLING A FLOATING-POINT REASSOCIATION EQUIVALENT, ASK WHERE THE
  ROUNDING GRID STOPS BEING RELATIVE.** Unit #39.
  `2.0*(omega*omega)` -> `(2.0*omega)*omega` survived two corpora and is not
  equivalent: multiplying by two is exact, so the two agree for every input
  whose result is NORMAL, and part company in the SUBNORMAL range where the
  quantum is absolute.

  ```
  u = omega*omega = 0.4 D        2.0*RN(u) = 0.0
     D = 4.94e-324               RN(2u)    = RN(0.8 D) = D
  ```

  It looks unreachable because reaching a compared output needs a SECOND
  quantity at its own extreme and every ladder moves ONE: the quantum must
  survive `-8 +`, so `DT*DT` has to be within a factor of two of DBL_MAX.
  Closed by addition, one baseline state, arithmetic rather than a guess --
  `freq = 1.77e-163`, `DT = 1e154`, `ki = 0` so `2*DT*ki` cannot swamp the term,
  every other input zero or one. 2.0 against 1.9999999999999998, one case, one
  kill.

- **A NO-OP RED-TEST STUB STILL NEEDS THE CALLEE BRIDGE, AND THE LINK ERROR
  NAMES OTHER UNITS.** Unit #39. `harness.sh` generates callee bridges by
  reading the translation for calls and drops the callee's own `.cpp.o` from
  LIBS when it keeps one, so a stub that calls nothing keeps nothing:

  ```
  rescontroller_test.cpp:(.text+0x388): undefined reference to `saturate_c'
  picontroller.cpp:(.text+0x74):        undefined reference to `saturate_c'
  ```

  Keep one call whose result is discarded and whose arguments are constants --
  `(void)saturate_c(0.0, minValue, maxValue);`. It cannot make the stub agree
  with the reference on any output, and it is the difference between a red test
  and a build failure that reads like a broken harness. First met here; it will
  recur on every unit with a translated callee.

- **THE ARM CENSUS IS THE CHEAPEST WAY TO TURN "THE HARNESS REACHES IT" FROM A
  CLAIM INTO A COUNT, AND IT COSTS ONE 19-SECOND RUN.** Unit #38, and it is unit
  #37's instrument used where there is no survivor list at all -- the score was
  already 1.000 and the question was what the 9033 cases run.

  ```
  calls 9033 | RETURNed on aviFAIL < 0 8740 | ran the body 293
  F_LPFType==1 42 | ==2 42 | NEITHER 209        <- ==2 has 0 hits in all 27 scenarios
  tower-top notch iterations 275                <- 0 iterations in all 27 scenarios
  Flp_Mode==2 90, its inner notch loop 171      <- scenario 4 only
  42+42+209 = 293      113+90+89 = 292 blade iterations
  ```

  The probe fails **0 of 9033**, so the run is a READING. The arithmetic closing
  is what says the counters are on the arms they name; the 8740 agreeing with
  the no-op red test's 293 from the other side is what says the two instruments
  measure one corpus.

- **AN INDEX CAN BE AN ARRAY, AND ALL THREE SITES THAT MATERIALISED ONE WROTE A
  SCALAR.** Unit #38, `translation-loop 3ac5b4a`. `F_GenSpdNotch_Ind` is an
  ALLOCATABLE INTEGER array every ELEMENT of which subscripts `F_NotchFreqs`, so
  it carries `role="index"` AND `dims` -- and `_case_impl` tests the role branch
  BEFORE the dims branch.

  ```
  TypeError: 'int' object is not iterable          <- four rules downstream
  PreFilterMeasuredSignals: CntrPar_F_GenSpdNotch_Ind is ALLOCATABLE and its
  case value is a scalar int (1), not a sequence   <- one pass, after the fix
  ```

  The `p.dims` branch ONE LINE BELOW already carries the same lesson for
  `Ind_CableControl` and kind-vs-dims. **A fix applied at one of two sites that
  share a code shape is a fix the other site escapes** -- here there were three
  (`_case_impl`, `_random_over`, R5's 1/interior/n sweep). Grep for the shape,
  not for the symptom.

- **AN UNRESTRICTED `USE M` PUTS *M's* NAMES IN SCOPE, NOT EVERY NAME.** Unit
  #38, `vit fe22383`. VIT's bridge generator gave up on adding a nested type's
  `USE` line the moment any copied statement lacked an `ONLY` list.

  ```
  the module carries   USE Constants / USE Functions        <- neither re-exports a type
  the procedure adds   USE ROSCO_Types, ONLY : <5 dummies>  <- not the 5 NESTED types
  gfortran             Derived type 'filterparameters' at (1) is being
                       used before it is defined
  ```

  Fortran use-association is transitive only through a module that USEs
  something unrestricted ITSELF. Reason about which module supplies the name,
  not about whether some USE is unrestricted.

- **A COUNTER PASSED INOUT TO A CALLEE THAT SUBSCRIBES A FIXED-SIZE ARRAY WITH
  IT IS THE SECOND AND THIRD INSTANCE OF ONE CLASS, AND `ranges.toml` IS STILL
  THE ONLY PLACE IT CAN BE SAID.** Unit #38, after unit #34.

  ```
  ./test                        exit 139, SIGSEGV, core dumped, 0 bytes of stdout
  the harness said              harness produced no JSON     <- also what a
                                                                PRINTING reference says
  after six pins, unchanged     HARNESS PASS: checked 8955 failed 0
  ```

  Five `objInst%inst*` counters index `FP`'s `DIMENSION(1024)` arrays INSIDE the
  filters; `LocalVar%NumBl` bounds a loop over `rootMOOP`, which is
  `DIMENSION(3)`. No compared out-parameter exists for an inference to attach a
  role to. **The bound is arithmetic, not a round number**: this unit advances
  `instNotch` up to eighteen times in ONE call, so 1000 and not 1024.

  And a stdout-buffered SIGSEGV is invisible: the process dies with its buffer
  unflushed, so the artifact is zero bytes and the message is the one a printing
  reference produces. Two `fprintf(stderr)` lines in the GENERATED test .cpp --
  the case index and the inputs the reference subscripts -- cost one rebuild and
  name the case.

- **AN OUT-OF-BOUNDS SUBSCRIPT IS NOT A SIGNAL, SO THE CASE THAT CRASHES IS NOT
  THE CASE THAT IS WRONG.** Unit #38. Case 15 ran the body with `NumBl = 195`
  against a `DIMENSION(3)` array and SURVIVED; case 17 died. The instrumented
  run names the case and does NOT identify the input -- what identifies it is
  the bisect over pins, and both are recorded
  (`evidence/PreFilterMeasuredSignals/harness.case17_probe.txt`).

- **A BASELINE STATE IS A CORPUS RULE'S OWN DATA, AND R5's "ARRAY ELEMENTS
  DISTINCT" APPLIES TO IT. NOTHING CHECKS THAT.** Unit #38.

  ```
  {"ramp": [1, 0]}, both notch counts 1     score 0.8254, 22 survivors
  of which                                  'ind - 1' -> '1 - ind'   x9
                                            'n - 1'   -> 'n + 1'     x1
  {"ramp": [1, 1]}, both notch counts 2     score 1.000, 0 survivors
  ```

  With every element 1, `ind - 1` and `1 - ind` are THE SAME NUMBER. R11's own
  implementation comment records this from unit #29 -- 49 of that unit's 89
  survivors -- and the baseline file reproduced it. The corpus stayed at 9033
  cases: two counts and two ramps moved, nothing else.

- **`_mutation_merge.py` STATED ONE IDENTITY TWICE AND THE TWO DISAGREED.** Unit
  #38, C12. `killed + survived != behavioural` refuses every SPLIT sweep with a
  declared equivalence in it, while `denom = behavioural - eq` five lines below
  already reads `mutants` as counting them.

  ```
  _mutation_merge: REFUSING -- 114 killed + 0 survived != 126 behavioural
  ```

  It refused rather than writing a wrong number, which is the right failure --
  and the campaign's first split sweep with an equivalence in it is what found
  it, thirty-eight units in.

- **A WRAPPER RED TEST'S REVERT REBUILT NOTHING, AND `git diff` SAID THE SOURCE
  WAS CLEAN.** Unit #38, and it is the bind-mount mtime hazard this file already
  records, met at the revert rather than at the edit.

  ```
  restore, rebuild, re-run the green     POST-INTEGRATION FAIL: 9033 / 293
  touch the source, rebuild, re-run      POST-INTEGRATION PASS: 9033 / 0
  ```

  **Re-take the green after a red test; do not infer it from the restore.** The
  293 is the red test's own count, so a reader who trusted the restore would
  have had a failing artifact that looks exactly like a translation defect.

- **A GATE RED TEST THAT FAILS IS A MEASUREMENT, AND A SECOND UNIT'S KERNEL CAN
  CORROBORATE IT.** Unit #38. Disabling the `Flp_Mode == 2` blade-root arm moves
  **0 of 5,252,000**, revert-verified; the whole unit as a no-op moves
  **1,751,360**. The arm is reached by scenario 4 alone -- 12,000 hits -- and
  scenario 4's docstring in `vit_sim.py` says *"blade root moments are
  near-zero"*. Unit #35 measured the same quantity from the other end: its
  `PIIController` kernel found `-LocalVar%rootMOOPF(K)` identically zero in all
  20 captured cases. **An arm can have a coverage number, a translation and a
  mutation kill count, and no simulation evidence at all.**

- **A STUB THAT PASSES EVERY CASE CAN BE AN EQUIVALENCE IN THE WHOLE PROGRAM,
  AND TWO GREPS SETTLE IT FASTER THAN ANY CASE COUNT.** Unit #38.

  ```
  Filters.f90, all five filters   IF ((iStatus == 0) .OR. reset)
  ReadSetParameters.f90:123-126   the ONLY assignment to LocalVar%restart:
                                  restart = (iStatus == 0)
  ```

  So `reset` is TRUE only where `iStatus == 0` already is, the `.OR.` makes it
  redundant at all nineteen call sites, and a stub forcing every `reset` to
  false passes 62 of 62 with ZERO rows moved -- at any window, in any scenario.
  That is a different claim from "the corpus does not vary it", and only the
  second one a wider corpus could fix. The differential harness draws `iStatus`
  and `restart` INDEPENDENTLY and is the one instrument that is not blind.

- **R6's ALL-PAIRS FALLBACK HOLDS EVERY OTHER KNOB AT ITS LADDER'S FIRST VALUE,
  SO A BRANCH THAT NEEDS A TRIPLE IS UNREACHABLE — AND THE UNIT'S OWN COVERAGE
  LINE SAYS WHICH MODE RAN.** Unit #37, and it is the first artifact in this
  campaign whose coverage line does NOT read `the full cross product`.

  ```
  6 knobs   Ind_R_Pitch/Speed/Torque 4 each, PRC_Comm 5, PRC_Mode 4, aviFAIL 4
  product   4*4*4*5*4*4 = 5120        _KNOB_CASE_LIMIT = 4096
  fallback  every knob not being crossed sits at v[0]  ->  PRC_Mode = 0
  needed    PRC_Mode==2 AND PRC_Comm==1 AND Ind_R_Speed>0   <- a TRIPLE
  ```

  1024 combinations over the bound cost the unit eleven of its twenty
  statements: the OpenLoop arm's three `interp1d` calls and its `WRITE(401,*)`
  ran **0 times in 3596 cases**. **Grep the artifact's `R6_reference_literals`
  line for `ALL PAIRS ONLY` before reading any green from it.**

  The fix is ADDITION, not a wider bound: `harness/baseline.<Unit>.json` states
  an admissible state and R11 walks each knob off it one at a time, which is
  exactly the shape a triple needs. 52 cases, the other 3596 untouched.

- **A GREEN IS A STATEMENT ABOUT THE ARMS ITS CASES RAN, AND A COUNTER PROBE
  THAT FAILS 0 IS THE CHEAPEST WAY TO ASK WHICH.** Unit #37. The probe is the
  shipped translation with one `++counter` per arm and an `atexit` that writes
  them; because it changes no behaviour it runs GREEN, so it is a READING of the
  corpus and not a perturbation of it.

  ```
  bash evidence/<Unit>/run_probe.sh probes/arm_census.cpp <out>.json "arm census"
  ```

  It cost one 14-second run and it is what caught both of this unit's corpus
  defects. Unit #33's rule was "make the interesting arm an output before
  explaining a survivor list"; this is the same instrument used BEFORE there is
  a survivor list to explain, which is cheaper still.

- **A MUTATION SCORE AND THE GREEN IT IS SCORED AGAINST MUST NAME THE SAME CASE
  COUNT, AND NOTHING IN THE PIPELINE CHECKS IT.** Unit #37, C12.

  ```
  vit_mutate.py  rebuilds and re-runs the harness against WHATEVER CASE FILE IS
                 ON DISK -- it does not regenerate one
  the last writer was the P10 control run: the baseline moved aside, 3596 cases
  the committed green is 3648
  mutation/<U>.json records `compared_against` and NO CASE COUNT AT ALL
  ```

  The wrong sweep was complete and internally consistent — 77 mutants, score
  0.2763 — and announced nothing. **Re-run the harness immediately before the
  sweep, and check the sweep's survivor list against the arm census**: it was
  caught because six survivors reproduced the census's zeros exactly. Unit #26's
  rule one instrument over.

- **A PROBE THAT GOES RED MUST SURVIVE ITS OWN `set -e`, OR ITS FILE AND ITS
  CALLER DISAGREE AND THE FILE IS THE ONE NOBODY READS.** Unit #37.

  ```
  { ...measure... } > "$OUT"     # a RED probe exits 1
  rc=$?                          # never reached under `set -e`
  cat "$OUT"                     # never reached
  ```

  **37 of 45 mutants were graded `nocompile` and every one was a kill.** The
  artifact was correct throughout. A caller reading stdout cannot tell a kill
  from a build failure, and "no output" defaults to the wrong one.

- **A UNIT CAN HAVE MORE KINDS OF OUTPUT THAN THE PIPELINE HAS ORACLES, AND THE
  HONEST SCORE IS THE UNION — WHICH `vit_mutate.py` CANNOT EXPRESS.** Unit #37,
  68 of 76 across three.

  ```
  differential harness, 3648 cases                29 of 76
  the formatter vs 22,526 gfortran records        36 of 45
  fort.401 vs fort.401.cpp                         3 of 3
  ```

  `--equivalences` is the only place a "not killed by me" can be recorded, so
  writing the 39 kills there makes `equivalent_declared` OVERSTATE its own name.
  Keep the undeclared run beside it (`mutation/<U>.harness-only.json`) and say
  the split in the artifact. A `killed_by` field naming the instrument is the
  fix and it is not in the tool.

- **REPRODUCE A FORTRAN WRITE BY MEASURING IT, AND SLICE THE TESTED CODE OUT OF
  THE SHIPPED TRANSLATION SO THE ORACLE CANNOT GO STALE.** Unit #37, 22,526
  values, 0 mismatched.

  ```
  every record        exactly 26 characters
  0.1 <= |v| < 1e17   F-form, 17 significant digits, right-justified in 21,
  or v == 0           then FIVE trailing blanks
  otherwise           E-form, 1 digit + 16 decimals + E<sign><3 digits>, in 26
  non-finite          "NaN" / "Infinity" / "-Infinity", right-justified in 26
  ```

  `%.0f` DROPS the decimal point and Fortran's `F21.0` keeps it — 488 of the
  22,526 records, and the whole difference between a model that mismatched and
  one that does not. The exponent is read off `%.16E` rather than from `log10`:
  17 significant digits identify a double uniquely, so there is no boundary
  where a rounding and a logarithm disagree.

  **And when the reference and the translation both write ONE file in ONE
  process, they are not being compared — they are overwriting each other.**
  Split the path (`fort.401` vs `fort.401.cpp`) and diff.

- **TWO LIVE ARMS WITH DISJOINT SCENARIO SETS NEED TWO GATE RED TESTS, AND THE
  FIRST ONE PROVES IT.** Unit #37.

  ```
  ELSE arm    +0.01 on PRC_Min_Pitch   1,781,601 / 5,252,000, 21 scenarios,
                                       and NOT scenario 25
  PRC_Mode==2 x1.000001 on PRC_R_Speed     22,660 / 5,252,000, ALL scenario 25
  ```

  Scenario 25 is the only one of 27 with `PRC_Mode=2`. Either red alone reads as
  "the gate sees this unit" while being blind to half of it. Unit #14's rule
  says to read the red test's per-scenario channel list; this is what to DO when
  that list turns out to be a partition.

- **A RED TEST THAT WILL NOT COMPILE IS NOT A RED TEST, AND THE TOOL SAYS IT
  IS.** Unit #36. `vit verify` printed `62/62 passed` and beside it *"Red test:
  the kernel failed to build with the return value scaled by 1.00001"*, then
  wrote `red_test: demonstrated` into `vit.yaml`.

  ```
  redtest.py rewrites EVERY `return <expr>;` in the FILE, not the function's
  the unit's helper returns a std::string  ->  `return (std::string(...)) * 1.00001;`
  _run_red_test: "a perturbation that will not build is a red ... and an honest
  one" -- and RETURNS, so the value-level perturbation is never tried
  ```

  No perturbed kernel ever ran. It is unit #10's `make`-never-built-it with the
  sign flipped, landing in the one field a later reader would trust for exactly
  this question. **Read what a red test perturbed, not that it went red** --
  and any translation carrying a non-arithmetic `return` in a helper is in this
  state, which here is at least `interp1d.cpp` and `sigma.cpp` too.

- **WHEN A DIFFERENTIAL HARNESS FAILS *EVERY* CASE, SUSPECT THE GENERATOR
  BEFORE THE TRANSLATION -- AND READ THE GENERATED `_test.cpp`, NOT THE
  ARTIFACT.** Unit #36, 1292 of 1292.

  ```
  CntrPar_a.PS_BldPitchMin_N = r.i();                 <- the buffer is sized from this
  ...
  &CntrPar_b.n_PS_BldPitchMin                         <- the bridge is passed this
  grep -c n_PS_BldPitchMin <stem>_test.cpp     ->  1  <- and nothing ASSIGNS it
  ```

  `expand_derived` prefers a real Fortran field as an array's extent
  (`_EXTENT_FIELD_PATTERNS`) while VIT's view struct always carries
  `n_<field>`, and the two halves disagreed. The reference allocated ZERO
  elements. **The grep is the diagnosis in one line**: a struct member that
  appears exactly once, at a call site, is a member nobody filled.

  Five fields in this campaign have a companion count -- `WE_CP`,
  `WE_FOPoles`, `PS_BldPitchMin`, `SU_LoadStages`, `ACC_INFILE` -- and
  `CheckInputs` is closed with its `n_SU_LoadStages` at 0 in every case.

- **A CALLEE THAT IS ALREADY INTEGRATED GIVES THE PRE-INTEGRATION REFERENCE A
  SECOND CAPACITY, AND R13 IS WHERE IT SHOWS.** Unit #36, 16 of 1292, and the
  16 is `len("PitchSaturation:")`.

  ```
  capacity < 14      neither prefix fits          both sides agree
  capacity 14..29    interp1d's fits, this unit's does not     <- the 16
  capacity >= 30     both fit                     both sides agree
  ```

  The reference calls the Fortran `interp1d`, which re-enters C++ through its
  own bridge over the MODULE staging buffer -- so the inner write is not gated
  by the case's capacity, both prefixes land in the unbounded Fortran string,
  and the stated capacity is applied once, at export, where it refuses
  everything. The shipped build gates once. **Any unit whose already-integrated
  callee writes a deferred-length CHARACTER output is in this state.**

- **`--disable <rule>` IS RECORDED AS `N/A` WITH A REASON THAT IS FALSE.** Unit
  #36. Ablating `R13_staging_capacity` makes the artifact say *"no
  deferred-length CHARACTER output"* about a unit whose un-ablated run reports
  `applied R13_staging_capacity 256 case(s)`. An ablation is not merely
  invisible in the machine-readable file, it is recorded as the one thing it is
  not. **Do not cite `rule_coverage` as evidence that a rule was applicable.**

- **AN INHERITED `.gitignore` PATTERN IS A HAZARD TO EVERY ARTIFACT THE
  CAMPAIGN INVENTS A NAME FOR.** Unit #36. `*build*` at `.gitignore:65` is
  upstream ROSCO's and matches by FILENAME anywhere in the tree; five evidence
  artifacts across four units were untracked by it, and every one records a
  FAILURE. `git add -A` prints nothing and the commit reads as complete.

  ```
  git ls-files --others --ignored --exclude-standard evidence
  ```

  Run it before believing an evidence directory is committed. Closed by
  addition, not by editing line 65.

- **A DEMONSTRATED RED TEST IS NOT A DISCRIMINATING CORPUS, AND THE TWO
  SENTENCES ARE ONE WORD APART.** Unit #35. `vit verify` printed `20/20 passed`
  and, beside it, `the kernel reported a mismatch with input 'error' offset by
  1e-05`. Every earlier unit would have read that as the 20/20 being certified.

  ```
  `error` forced to 0.0                        PASSES 20/20
  a wrong answer if EITHER input is non-zero   PASSES 20/20
  the whole unit as a no-op returning 0.0      fails ONE field of 234
  ```

  Both sentences are true at once: offsetting a zero makes it non-zero, so the
  kernel CAN go red on `error`, and the corpus it was given never does. **"The
  instrument is sensitive to X" and "the cases vary X" are different claims**,
  and only the second decides what a pass means. Unit #33 had to measure a
  refusal's stated reason because a refusal can be false; this is the mirror,
  and the mirror is the more dangerous one because nothing looks wrong.

- **THE TREE A PROBE NEEDS IS THE TREE ITS GREEN WAS TAKEN ON, AND THAT CAN BE
  NEITHER STATE THE RESET/RESTORE PAIR OFFERS.** Unit #35, caught by a P10
  control that cost one run.

  ```
  the no-op, this unit's Fortran body intact     4528 of 4528
  the no-op, this unit INTEGRATED                   0 of 4607
  ```

  Two independent things move on integration and either alone invalidates a
  probe number. `harness.sh` links `vit_integration_shim.o` so the wrapper's
  `<unit>_c` resolves -- and after integration that wrapper IS the reference, so
  both sides run the harness's own copy (unit #29). And the corpus changes SIZE,
  because the generator mines the reference's own literals and a marshalling
  wrapper has none (unit #32's note about R12's widths, one instrument over).

  ```
  fully integrated     the reference is a wrapper:  4607 cases, no-op scores 0
  this unit reverted   the reference is the body:   4528 cases     <- the one
  fully clean          every OTHER unit's body is restored too, so the C++
                       side's callees are different objects again
  ```

  `git checkout <the pre-integration commit> --` on the three
  integration-carrying paths, under an EXIT trap that restores from HEAD, lands
  on the middle state exactly and does not trip the reset marker. **Ask which
  tree the number has to be commensurable with, not which tree is cleanest** --
  and run the no-op FIRST, because a probe number from a one-sided comparison is
  not a number.

- **WHEN A RED TEST'S OUTPUT LIST IS MISSING A FIELD THE TYPE DECLARES, ASK
  WHETHER A PROBE CAN MAKE THAT FIELD AN OUTPUT BEFORE RECORDING THE ABSENCE.**
  Unit #35, and it is unit #30's "a path that writes NOTHING is invisible to a
  no-op by definition" reached from the other direction.

  ```
  the no-op names   vit_result, piP.ITerm, piP.ITermLast, piP.ITerm2, inst
  R4 compares       those five AND piP.ITermLast2 AND piP.ELast
  ```

  `ITermLast2` is absent because the REFERENCE does not write it in the ELSE arm
  either -- an upstream asymmetry, transcribed under P7. But absence-because-the
  -reference-agrees and absence-because-the-corpus-cannot-reach-it read
  identically in the artifact. The probe that separates them is the shipped file
  with the asymmetry REPAIRED: it is rejected on 2261 of 4528, every ELSE-arm
  case.

- **A PERTURBATION CONFINED TO ONE CHANNEL FAMILY IS NOT A WEAK RED TEST --
  CHECK ITS COUNT AGAINST THE COVERAGE FIRST.** Unit #35.

  ```
  gate/PIIController.redtest.json   11,997 of 5,252,000, 3 channels
                                    scenario_4:flp_angle_{1,2,3}, each 3999/4000
  coverage/line_coverage.json       11,997 calls, all in scenario 4,
                                    = 1 reset-arm + 3998 else-arm, per blade
  ```

  gcov's statement counter and a bit-exact comparison of simulation output have
  nothing in common and agree on the total AND on the decomposition. The
  containment is the shape of the loop -- the output is clamped and fed straight
  back as the next call's error input, so once it saturates every value of the
  channel moves and nothing outside it does. Two instruments, one number, is the
  same corroboration unit #33 got from a C++ mutant and a Fortran MERGE.

- **`scripts/harness.sh` COULD NOT RUN ITS OWN DOCUMENTED USAGE LINE FOR
  THIRTY-FOUR UNITS.** Unit #35, C12.

  ```
  scripts/harness.sh: line 382: ARGS[*]: unbound variable
  ```

  PRE mode with no optional arguments. `${ARGS[*]}` on an EMPTY array is unbound
  under `set -u` in bash 3.2 -- this machine's shell -- and bash 4.4 and later
  do not agree. Latent because every previous pre-mode invocation passed
  `--out`, which appends to that array. The failure is in the SAFE direction and
  that is why it is worth recording rather than only fixing: the shape to fear
  is the mirror, an unset variable expanding to nothing and a command running
  with one argument missing. Fixed additively: `${ARGS[*]-}`.

- **`harness.sh` REDIRECTS INTO `--out`, SO A DIED BUILD DESTROYS THE ARTIFACT
  THAT WAS ALREADY THERE -- AND `git add -A` WILL COMMIT THE DELETION.** Unit
  #35, C12, and the commit that did it (`b090ab2`) is left unamended.

  ```
  make: *** [Makefile:80: test] Error 1        <- nothing in the tree had changed
  the identical command a minute later:  POST-INTEGRATION PASS: 4528, 0 failed
  ```

  Unit #23's and unit #30's bind-mount transient at a new site; what is new is
  the blast radius. The commit message was written from the red tests' output
  one line above the failure and said the green had been re-taken while removing
  the file that green lives in. **Read the LAST line of a run before writing a
  commit message about it, and prefer `git add <paths>` to `git add -A` when the
  previous command may have deleted an artifact.** Making the script write to a
  temporary and rename is the repair NOT made here: it changes how every
  artifact in this campaign is written (X3).

- **AN INDEX CAN ITSELF BE A DERIVED-TYPE FIELD, AND WHEN THE HARNESS CANNOT SEE
  THAT, THE PROGRAM THAT SEGFAULTS IS THE REFERENCE.** Unit #34. Both index
  inferrers (`infer_indexes`, `infer_indexes_fortran`) split a subscript on
  non-word characters and look for a PARAMETER NAME. `piP%ITerm(objInst%instPI)`
  yields `objInst` and `instPI`; the parameter is `objInst_instPI`.

  ```
  instPI outside 1..1024     4741 of 4771 cases   before   (default -300)
                                0 of 4692 cases   after
  the symptom                harness produced no JSON, which is ALSO what a
                             printing reference produces
  ```

  Unit #11 met the same failure with a bare dummy and fixed it by reading the
  role off the reference as well as off the translation. This is one
  qualification level in, and it is the `file_params_from` shape P10 already
  records: a helper whose notion of a name is a bare identifier, meeting a
  codebase where the reachable ones are array elements and derived-type fields.
  Fixed additively in the loop (`e7d5583`), applied only where the
  bare-identifier rule found nothing, so no already-measured corpus moves.

  **AND THE SECOND INDEX OF THE SAME UNIT NEEDED THE OTHER ANSWER.**
  `objInst%instLPF` is subscripted inside the CALLEE, against a field of a
  NESTED type the generator does not expand into parameters -- so there is no
  compared out-parameter for any inference to attach a role to, and it takes a
  `harness/ranges.toml` pin instead. The question to ask of a fixed-extent
  array's index is not "is it a dummy" but **"is the array it indexes a compared
  out-parameter"**: if it is, infer; if it is not, no inference can ever reach
  it and a pin is the only mechanism.

- **A NO-OP RED TEST CHANGES THE GENERATED CALLEE SET, AND THEREFORE THE LINK --
  SO NAME THE CALLEES IN A DEAD BRANCH.** Unit #34, and it cost one confusing
  build failure naming a DIFFERENT unit's symbol.

  ```
  vit/cli.py   generate_callee_bridges(Path(args.cpp_file).read_text(), ...)
               callee_bridge=bool(callee_src)   -> the Makefile's `test:` deps
  the stub     no `_c(` call -> no bridges -> pidcontroller_callees.o unlinked
  the error    picontroller.cpp.o: undefined reference to `saturate_c'
  ```

  `reset_to_clean.sh` leaves every earlier unit's `.cpp.o` in the build tree and
  `vit test-validate` globs them into LIBS, so a stale object's callee becomes
  the stub's problem. A bare no-op does not measure a WEAKER instrument, it
  measures a DIFFERENT one, and unit #26's rule is that a red result and the
  green it certifies must name the same instrument. The fix is two lines:

  ```cpp
  if (false) { (void)saturate_c(0,0,0); (void)lpfilter_c(...); }
  ```

- **WHEN THE INTERESTING ARM IS REACHED IN 0 OF N CASES, SIX SURVIVORS ARE ONE
  NUMBER -- AND THE COUNTING CHANNEL SHOULD BE A FIELD THE UNIT NEVER TOUCHES.**
  Unit #34, and it is unit #30's counting-probe rule with the collision question
  answered instead of argued.

  ```
  the ELSE arm runs                      2307 of 4610   <- the control (P10)
  minValue < maxValue AT ALL              207 of 4610
  the ITerm clamp is INACTIVE               8 of 4610
  the OUTER clamp is INACTIVE               0 of 4610
  ```

  Unit #32 had to argue that `LineNum = -7` could not collide with a real
  answer. Here the sentinel goes into `piP%ITerm2` -- a field this unit never
  writes, belonging to a sibling procedure that shares the type, and compared by
  R4 as one of 174 out-parameters. The reference's value for it is the input's
  UNCONDITIONALLY, so the failing count IS the number of cases reaching the arm
  and no argument is needed. **Look for an unused field of an INOUT derived type
  before inventing an impossible value.**

  The cause is structural and general: `minValue` and `maxValue` are drawn
  INDEPENDENTLY from the same default, and R6's isolating pin sets every OTHER
  defaulted real to the SAME value -- so the rule meant to isolate a parameter
  collapses exactly the pair that has to be an INTERVAL. See DECISIONS.md,
  proposed rule `R15_bracketing_bounds`.

- **`--reverse-copy` IS REQUIRED WHEN A VIEW HOLDS A NESTED TYPE BY VALUE, EVEN
  THOUGH NOTHING THE UNIT WRITES IS A SCALAR.** Unit #34. The rule as inherited
  is "use it when the function modifies SCALAR fields in view-type INOUT args",
  and this unit modifies none: what it changes inside `LocalVar` is
  `LocalVar%FP`, six arrays a CALLEE writes.

  ```
  vit_types.h                 filterparameters_t FP;    <- BY VALUE in the view
  vit_populate_localvariables view%FP%lpf1_a1 = src%FP%lpf1_a1    (a copy)
  the red test                the copy-back call deleted -> 4610 of 4610 fail
  ```

  An ALLOCATABLE field crosses as a pointer and needs no copy-back; an embedded
  derived type is copied in and must be copied out. Read the VIEW STRUCT, not
  the unit's write set.

  **And VIT resolved an aliasing here that no earlier unit had.** At
  `Controllers.f90:346` the `piP` argument IS `LocalVar%piP`, so the generated
  wrapper tests `C_ASSOCIATED(C_LOC(piP), C_LOC(LocalVar%piP))` and routes the
  pointer through `LocalVar_view%piP` when they are the same object -- otherwise
  the view copy-back afterwards would overwrite the integrator state the unit
  had just written through the other pointer. Read the wrapper for this whenever
  an argument could be a component of another argument.

- **WHEN A VERIFICATION TOOL DECLINES TO CERTIFY ITS OWN RED TEST, THE DECLINE
  NAMES AN INPUT -- PERTURB THAT INPUT AND COUNT.** Unit #33. `vit verify`
  printed `62/62 passed` and, beside it, that both of its red test's INPUT
  perturbations had been ABSORBED (`input 'error' scaled by 1.00001`, `offset by
  1e-05`), "typically a saturated output". Quoting that is a hedge; one stub
  turns it into a number:

  ```
  the reference return value == minValue        62 of 62 captured cases
  a stub with `error` forced to 0.0             PASSES 45 of 62
  the same edit made at the GATE            moves 2,133,598 of 5,252,000
  ```

  This is unit #32's "a refusal's stated reason is a claim" with the refusal
  turning out to be RIGHT, which is the case where quoting feels safe and is
  not. "The kernel may be weak here" and "the kernel cannot see this argument in
  73% of its cases" go into `observability` as different sentences, and only the
  second tells the next reader which oracle to trust.

  And the gate figure is LARGER than the same unit's whole-unit no-op
  (1,780,508). Not a paradox and worth expecting: a no-op stops advancing the
  unit's state, while an argument-blind unit keeps running, so the controller
  stays in its loop and the trajectories diverge further. **A no-op is not an
  upper bound on what a perturbation can move.**

- **A COVERAGE ZERO AT A CALL SITE'S OWN LINE IS NOT A DEAD CALL SITE: gcov
  ATTRIBUTES A CONTINUED STATEMENT'S HITS TO ITS LAST CONTINUATION LINE.** Unit
  #33, caught one sentence before it was written into `observability`.

  ```
  Controllers.f90:945   LocalVar%CC_ActuatedL(I_GROUP) = PIController( &      0 hits
  Controllers.f90:947                     LocalVar%piP, ..., objInst%instPI)  127,994
  ```

  Four of this unit's 17 call sites read as dead at their own line and none of
  them is. Unit #1 established that a call site with no hits is not a tool
  failure; the converse needs saying too, because the report is the same zero.
  The check is one range over the same dict:

  ```python
  best = max(sum(hits.get(str(l), {}).values()) for l in range(site, site + 5))
  ```

- **QUOTE `verify_fields.csv`'s STATUS COLUMN AND NOTHING ELSE -- ITS VALUE
  COLUMNS ARE NOT ALWAYS THE COMPARED VALUES.** Unit #33, and it extends unit
  #4's rule ("the bit-exact claim lives in the status column, not the verdict
  line") one column further left.

  ```
  instpi   computed 3, reference 3, IDENTICAL   in all 62 rows
  the stub that deletes `inst = inst + 1`       moves objinst%instpi in all 62 cases
  the CSV carries 14,508 rows                   the kernel's own stdout carries 14,818
  ```

  A reader who quoted the values would report the increment as unconstrained
  when it is the most constrained thing in the unit. The verdicts agree
  everywhere, so this is not a wrong answer -- it is a column that reads like
  evidence and is not. Keep the kernel's own stdout beside the CSV
  (`run_stub.sh` writes it) and count `NOT IDENTICAL` lines from that.

- **A MARSHALLING CONSTRUCT IN THE WRAPPER IS A MUTANT OF THE SAME PREDICATE
  SEEN FROM THE OTHER SIDE OF THE BOUNDARY, AND IT COSTS ONE REBUILD.** Unit
  #33. `vit integrate` writes `MERGE(1_C_INT, 0_C_INT, reset)` for a LOGICAL
  dummy; inverting it is the Fortran-side spelling of the C++ `negate_cond`
  mutant on `if (reset != 0)`.

  ```
  negate_cond, the C++ translation           3112 of 3532 harness cases
  the MERGE inverted, the Fortran wrapper    3112 of 3532 harness cases
  the predicate inverted, the whole program  2,128,633 of 5,252,000 gate values
  ```

  The first two agreeing TO THE CASE is what says the predicate is constrained
  rather than merely covered -- two instruments, two languages, one number. Use
  it when a post-integration red test needs a second perturbation: the obvious
  one (transpose two INTENT(IN) arguments) moved only 145 of 3532 here, because
  a swap is a no-op wherever the corpus draws the two equal.

- **A MUTATION OPERATOR GATED ON A TABLE OF NAMES STOPS COVERING THE PROGRAM AS
  THE CAMPAIGN INTEGRATES ITS OWN UNITS -- AND THE ARTIFACT SAYS SO IF YOU READ
  BOTH LISTS.** Unit #33, the first unit whose body CALLS a translated callee.

  ```
  mutation/PIController.json   operators_offered  12    <- what was tried
                               operators           7    <- what found a site
  the two saturate_c calls     no mutant of any kind
  ```

  All three call operators are gated on `_VALUE_PRESERVING` / `_SIBLINGS`, whose
  every entry is a C standard-library name. That restriction is MEASURED and
  right -- letting `drop_call` fire everywhere made four units 38-73%
  unbuildable -- and `saturate_c` still satisfies exactly the property it
  guarantees. Amending it is a campaign-wide re-take (X3), so the measurement
  goes by hand instead, which is what unit #24 did before the operators existed:

  ```
  drop_call    ITerm clamp / output clamp     1761 and 383 of 3532
  swap_args    value <-> minValue, both sites    0 and   0   EQUIVALENT
  transposed   bounds swapped, both sites        63 and 135   <- NO operator produces this
  ```

  Two things to carry. The `swap_call_args` zeros are equivalences already
  proved at `evidence/saturate/minmax_probe.txt`, so the amendment's effect on
  this unit is +2 kills, +2 equivalences and a score unchanged at 1.000 -- **an
  amendment whose effect on its proposing unit is nil is the cheap case for the
  Driver to decide.** And the last row is outside the amendment entirely:
  `swap_call_args` exchanges arguments 1 and 2 only, so a three-argument
  selection function's BOUNDS transposition has no operator at any unit.

- **A PROPOSED INSTRUMENT CAN BE DEMONSTRATED WITHOUT BEING ADOPTED, AND ONE RUN
  ANSWERS MORE THAN THREE DISPATCHES OF ARGUMENT.** Unit #32, second dispatch.
  The sanitiser amendment had been carried across three units on prose. Pointing
  it at the one mutant it was proposed for costs about fifteen seconds:

  ```
  # the generated Makefile spells compile AND link with $(CXX), so one variable
  # covers both; the campaign's Fortran objects need no instrumenting, because
  # the buffer that overflows is the harness's own std::vector<char>
  make CXX='g++ -fsanitize=address -g -fno-omit-frame-pointer' test
  ASAN_OPTIONS=detect_leaks=0 ./test findline_cases.bin
  ```

  ```
  the shipped translation   exit 0,    0 bytes on stderr
  the mutant                exit 1, 4475 bytes: heap-buffer-overflow, WRITE of
                            size 1, 0 bytes after 2048-byte region, char_assign
  ```

  **RUN THE CORRECT PROGRAM FIRST AND KEEP ITS EMPTY REPORT AS AN ARTIFACT.**
  The question that decides whether an instrument is usable is not "does it fire
  on the mutant" but "is it silent on the program that is right" -- one that
  reports on both marks every mutant killed, which is a 1.000 that measured
  nothing. `detect_leaks` must be off or libgfortran's I/O buffers answer that
  question with a false yes.

  ADOPTING it is a different act and was not done: the score stays 0.960,
  because a mutation instrument changed for one unit is what X3 forbids. The
  demonstration is filed as evidence for the Driver, and it still does not
  answer the campaign-wide question -- whether the other 31 translations are
  clean under it -- which is a sweep.

- **A SURVIVOR LIST THAT ONE MEASUREMENT EXPLAINS IS ONE PIECE OF WORK, AND THE
  DISPATCH THAT MEASURES IT SHOULD WRITE THE RULE'S SPECIFICATION EVEN IF IT
  CANNOT BUILD IT.** Unit #32, second dispatch. The first dispatch measured the
  gap and wrote a paragraph naming the remedy; this one implemented the
  paragraph almost verbatim and five of six survivors died in one sweep.

  ```
  cases reaching the match arm      47 of 2370   ->   89 of 2514
  of those, on a BLANK key          47                47
  so on a NON-BLANK key              0                42
  mutation                          0.760        ->   0.960
  ```

  Four details in that specification were load-bearing and a simpler rule gets
  each of them wrong. Copy them into the next rule of this kind:

  ```
  key = the word CASE-INVERTED       'aA' in the line, 'Aa' as the key: both
                                     fold to 'AA', so a fold dropped on EITHER
                                     side breaks the match. A same-case plant
                                     leaves one of the two folds untested
  k sweeps 1..3 AND every free       the generator cannot know WHICH integer
    scalar int is set to k-1, k      picks the word, only that one usually does
  the element FIRST and LAST         a search that skips line 1 and one that
    and nowhere else                 stops at line 1 are different mutants
  carried to R12's narrow WIDTH      `'200' -> '201'` dies on 14 cases and on
                                     nothing else in the corpus
  ```

  And the kills came in on five DIFFERENT counts -- 14, 48, 18, 21, 72 -- which
  is what says they were five behaviours rather than one seen five ways. A rule
  whose kills all land on the same number has added one case, not a rule.

- **MEASURE A CORPUS RULE'S X3 COST IN BYTES. TWO CORPORA OF 1552 CASES CAN
  DIFFER IN ALL 1552.** Unit #32, second dispatch, and it supersedes the
  case-count comparison the first dispatch settled for.

  ```
  bash scripts/reset_to_clean.sh          # R12's widths come from the CLEAN
  bash evidence/FindLine/x3_check_r14/run.sh    # Fortran; a wrapper has none
  bash scripts/restore_integrated.sh
  ```

  ```
                 R14 off        R14 on        first N bytes identical
  ChkParseData   102,577 B      110,533 B     yes    1552 -> 1624
  GetWords        56,701 B       56,701 B     yes    N/A: its array is OUTPUT
  FindLine     4,990,604 B    5,370,716 B     yes    2370 -> 2514
  ```

  The R14-off column reproducing the committed counts EXACTLY is the half that
  makes the ablation trustworthy; without it the table is two runs of an unknown
  generator. And ChkParseData's +72 cases all PASS, which is a real finding
  about a closed unit and is left as one.

- **A POSITIVE CONTROL MUST SHARE THE SURVIVOR'S SITE, NOT MERELY ITS
  INSTRUMENT -- AND WHEN THE HARNESS CANNOT SEE A MUTANT, PUT IT IN FRONT OF THE
  GATE BEFORE CLASSIFYING IT.** Unit #32, second dispatch, and it is unit #31's
  "chosen against the program, not against the probe" one step further in.

  ```
  gate/FindLine.redtest.json          1,857,893 of 5,252,000   the NAME MATCH
    -- same file, same instrument, same unit, and it says NOTHING about
       whether the gate can see the constant the survivor moves
  gate.survivor-ca75abea.json         0 of 5,252,000           '2048' -> '2049'
  gate.linewidth-control.json         1,583,216 of 4,732,000   '2048' -> '5'
  harness.linewidth-control.json      60 of 2514               '2048' -> '2047'
  ```

  `scripts/gate.py --perturb-file/--perturb-from/--perturb-to` IS a
  mutant-scoring instrument for a second oracle and costs one gate run (~3 min).
  Using it converted a survivor from "unmeasured" to "measured against two
  oracles, each with a control at its own site", which is what decides (a) from
  (b) from (c). The asymmetry it exposed is the general fact: **one byte too FEW
  is a wrong answer, one byte too MANY is not an answer at all**, and no value
  oracle of any kind reads bytes outside the buffer it compares.

- **A REFUSAL'S STATED REASON IS A CLAIM AND CAN BE FALSE, AND A FALSE ONE COSTS
  MORE THAN NO REASON WOULD -- ASK THE OTHER GENERATOR INSTEAD OF READING IT.**
  Unit #32, three tool defects in one declaration (`CHARACTER(*) :: X(:)`), and
  the first is the one to recognise:

  ```
  UNOBSERVABLE [character-array] FileLines
    "carries its extent in a descriptor build_c_params does not emit"
  vit interface FindLine   ->  CALL findline_c(FileLines_c, SIZE(FileLines),
                                               LEN(FileLines), ...)
  ```

  It emits BOTH. The count was already in `extent_of` under the ordinary `n_`
  name and already emitted as an extent Param; only the CHARACTER branch never
  read `dims_of`. A refusal that names a MECHANISM reads as a fact about the
  boundary, so it is believed for as long as no unit has the shape -- here, 31
  units. `UNMEASURED here` and `impossible here` look alike in a report and only
  one of them is worth a dispatch.

  One command settles it, and it is the command unit #8's rule already names:

  ```
  vit interface <Unit> -f <the file>.f90 | sed -n '/Wrapper Function/,$p'
  #   what the SHIPPED wrapper actually passes IS the ABI; a mapper that
  #   disagrees with it is wrong about the mapper, not about the boundary
  ```

- **TWO GENERATORS SPELLING ONE CALL WILL DISAGREE EVENTUALLY, AND C LINKAGE
  CHECKS NEITHER -- SO THE SECOND ONE MUST ASK THE FIRST RATHER THAN RESTATE
  IT.** Unit #32, and the artifact it produced was a 98% PASS.

  ```
  build_c_params                char* X, int n_X, int len_X      count, width
  generate_fortran_bridge       char* X, int len_X, int n_X      width, count
  a 4x3 CHARACTER array read as 3x4    a well-formed array of the WRONG SHAPE
  differential harness          2311 of 2358 cases AGREED
  the 47 that did not           reference LineNum == len_FileLines, every one
  ```

  Nothing failed to build, nothing crashed, and the number looked like an
  ordinary translation defect. What identified it was reading the FAILING cases'
  inputs out of the case stream rather than the diff: `LineNum` equal to the
  element WIDTH is `DO I = 1, SIZE(FileLines)` iterating over the wrong extent,
  and no translation bug produces that.

  ```
  python3 - <<'EOF'   # decode the case stream; the layout is in <stem>_test.cpp's main()
  ...  print(f"case {c}: len={lf} n={nf}", [fl[j*lf:(j+1)*lf] for j in range(nf)])
  EOF
  ```

  The fix that matters is not the reorder. `generate_fortran_bridge` now calls
  `build_c_params` and REFUSES when the two extent sequences differ, naming both
  -- a restatement that cannot be checked is a defect waiting for the first
  input shape that distinguishes it. vit `d2de28c`.

- **A CHECK'S SCOPE IS PART OF ITS PRECISION: `vit check` READ THE WHOLE FILE.**
  Unit #32. Ten of fifteen checks set `needs_fortran` and every one asks "the
  reference does X, does the translation?" -- so a sibling procedure supplied
  the Fortran half of a finding about a unit that has nothing to do with it.

  ```
  FindLine        contains no SCAN, INDEX or VERIFY at all
  delimiter-set   fired on GetPath's INDEX( GivenFil, '\', BACK=.TRUE. )
                  900 lines away in the same file
  ```

  Latent for EIGHT units in ROSCO_Helpers.f90, because `GetPath` and `GetRoot`
  both carry `'\/'` themselves and were clean by luck. It is the expensive kind
  of false positive: the finding names a real literal and a real absence, and
  only reading the reference shows they belong to different procedures. Fixed in
  vit `c4eb0ad`; when the name is not found the CLI falls back to the whole file
  AND PRINTS THAT IT DID, because a slice that silently missed would turn ten
  checks off without a word.

- **MAKE "THE INTERESTING ARM RAN" AN OUTPUT BEFORE EXPLAINING A SURVIVOR LIST,
  AND FIVE SURVIVORS MAY TURN OUT TO BE ONE NUMBER.** Unit #32, and it is unit
  #30's counting-probe rule on its second use, paid for in one command.

  ```
  LineNum = -7 inside the match arm          47 of 2370 cases   <- the whole gap
  WordInd pinned at 2                         0 of 2370
  CALL Conv2UC(ParamNameUC) deleted           0 of 2370
  ```

  2323 of 2370 cases never find the name, so `FoundLine` is false, `LineNum` is
  0 and `Line` is never written -- nothing downstream of the predicate is
  distinguishable in 98% of the corpus. Five of six mutation survivors follow
  from that one number, and none of them needed a separate explanation.

  The cause is structural and worth naming in general: **the corpus draws the
  two sides of a comparison INDEPENDENTLY, so the predicate is true only where
  two independent draws coincide.** That is unit #30's "a rule aimed at a
  predicate must supply BOTH sides of it" read from the other end, and the
  remedy is a rule that plants one input INSIDE another rather than a wider
  ladder for either.

  And when an artifact names survivors by (operator, before, after) and several
  sites match, **build each candidate and run it** rather than reasoning about
  which one it is -- seven runs at six seconds each turned a guess into
  `evidence/FindLine/const_tweak_probes/RESULTS.md`.

- **A SITE WHOSE MUTANTS DIFFER ONLY IN MEMORY THE COMPARISON CANNOT READ IS A
  RESTATEMENT: DELETE IT AND WRITE THE PROOF.** Unit #32, and the fifth time
  this campaign has reached the same shape (Conv2UC 0.696 -> 1.000, GetPath,
  GetRoot, GetWords).

  ```
  n = std::min(len_src, len_dst); two loops     min(a,b) -> a        SURVIVED
                                                min(a,b) -> min(b,a) SURVIVED
  one loop to len_dst, `i <= len_src` inside    the new predicate    47 of 2370
  std::max(WordInd, 0)                          all three mutants    SURVIVED
                                                deleted; AryLen >= 0 is STATED
  0.667 -> 0.704, mutants 39 -> 27
  ```

  The `min` version's mutants write the SAME len_dst bytes and then continue
  past the end, so what differs is undefined rather than wrong -- and unit #7
  settled that such a mutant can be deleted with its site but not declared
  equivalent. The rewrite is also the safer program: the loop cannot leave
  `dst`.

  **The distinction that decides it is whether the site can be removed at all.**
  `'2048' -> '2049'` on this unit's `MaxLineLength` has exactly the same
  character -- one byte past the CALLER's buffer, invisible to any value
  comparison -- and it STAYS a survivor, because VIT emits no `len_Line` and the
  width has to be stated in the C++ once.

- **A CORPUS RULE AND A TRANSLATION SITE ARE TWO HALVES OF ONE MEASUREMENT; A
  RULE THAT FIRES AND KILLS NOTHING IS NOT A RULE THAT IS WRONG.** Unit #32.

  ```
  R12 widened to resolve a NAMED width (module PARAMETER, not a literal)
    +12 cases     score 0.667 before  ->  0.667 after
  the same 12 cases, after char_assign lost its `std::min`
                  three kills
  ```

  The boundary was in the corpus and the translation had no site at which it
  changed an answer. Neither half is worth anything alone, and reverting the
  rule on its own evidence would have been the wrong reading.

- **A POSITIVE CONTROL MUST BE CHOSEN AGAINST THE PROGRAM, NOT AGAINST THE
  PROBE.** Unit #32. Two declared equivalences rest on a canary in the byte a
  mutant adds to a fixed-width local; the probe reports 0 of 2370 disturbed, and
  a zero is worth nothing until the probe is shown able to be non-zero (P10).

  ```
  control 1   conv2uc_c(ParamNameUC, MaxParamLength + 1)      0 of 2370   DEAD
              Conv2UC writes a byte only if it is a LOWERCASE LETTER, and the
              sentinel is '\x7f'
  control 2   ParamNameUC[MaxParamLength] = 'X';           2370 of 2370   alive
  ```

  For one run the dead control and the passing probe were the same number. A
  control that routes through the program's own logic can be silenced by that
  logic; write the disturbance unconditionally.

- **A UNIT'S OBSERVABLE SET IS A LIST OF STREAMS, AND THE HARNESS COMPARES THE
  ONES SOMEBODY THOUGHT OF. ENUMERATE THEM BEFORE CHOOSING AN ORACLE.** Unit
  #31, second dispatch, and it is the entry below it -- "the gate reads a
  stream" -- turned on the instrument that replaced the gate. `dbgcheck.py` was
  built to read `*.RO.dbg` because the gate read `avrSWAP`. Neither read the
  third thing this unit writes:

  ```
  WRITE(*,100)                      unit 6, ~124 records a scenario
  9 of 57 mutation survivors        every one of them on that single line
  46 of 110 stdout records          DIFFERED on the first comparison ever made
  ```

  The 46 is the point. Adding the stream did not merely score nine mutants, it
  found a **real defect** (C12, `0dbf443` -> `106d170`): libgfortran emits a
  preconnected unit's record whole, while a fully-buffered `stdout` split one
  record mid-field and delivered eighteen more after the driver's own output.
  Deterministic across three runs, so it is comparable as BYTES rather than as
  a filtered projection.

  ```
  grep -nE 'WRITE *\( *\*|WRITE *\( *6|print \*|std::cout|printf|fputs' <the unit>
  docker exec ... python3 vit_sim.py --scenario N > /tmp/a.out   # twice: is it stable?
  ```

  Return values, arguments, files, `stdout`, `stderr`, sockets, the exit code.
  A stream nobody enumerated is not a gap in coverage, it is a gap in the
  definition of "green".

- **ASK WHAT THE CALLER GUARDS BEFORE ADDING AN INPUT FOR A GUARD INSIDE THE
  CALLEE -- AND A SCENARIO THAT EARNS NOTHING CAN BE WORTH MORE THAN ONE THAT
  EARNS KILLS.** Unit #31, second dispatch. Two of six new scenarios killed
  nothing, and both turned a survivor into a declared equivalence resting on a
  measurement:

  ```
  scenario 29  LoggingLevel = 0   wrote NO FILE AT ALL
    DISCON.F90:75 and :145        IF (CntrPar%LoggingLevel > 0) CALL Debug(...)
    -> `LoggingLevel > 0` -> `>= 0` INSIDE the unit cannot differ on any input
       that reaches it. The inner guard is dominated by the caller's.
  scenario 34  two Init calls     avrIndices unallocated at BOTH
    a second Init in ONE library load -> ReadAvrSWAP sets aviFAIL = -1, and
      DISCON.F90:145 does not call Debug when aviFAIL < 0
    a second Init after kill_discon -> the library is RELOADED, so every SAVE
      datum resets
  ```

  The general form: **an unreachable arm inside a unit is often unreachable
  because of its call site**, and the call site is one grep away. The same
  grep that finds it also produces the argument the declaration needs.

  ```
  grep -nE "CALL *<Unit> *\(" -B 4 <the caller>.f90     # what wraps the call?
  ```

- **THE MUTATION CORPUS IS FIXED THE MOMENT THE FIRST PART IS TAKEN, SO CHOOSE
  IT FROM THE SURVIVOR LIST BEFORE STARTING -- AND PUT THE EQUIVALENCES IN THE
  MERGE, NOT IN THE SWEEP.** Unit #31, second dispatch.

  `dbgmutate_merge.py` refuses parts that ran different scenarios, which is
  right and which means a scenario thought of at part 7 of 10 costs the six
  already taken. Two survivors were left alive for exactly that reason
  (`Fl_PitCom` and `NacVaneOffset` need a scenario ABOVE RATED; 33 runs at
  9 m/s).

  The equivalence declaration has the opposite property and used to be bound
  the same way. Moved to `--equivalences` on the MERGE:

  ```
  in the sweep    revising a claim  -> re-run 25 minutes of simulation
  in the merge    revising a claim  -> re-run the merge, and it can REFUSE
  control         declare a mutant that died on 99,214 records -> exit 2
  ```

  A mutant declared equivalent that the corpus KILLED is a wrong declaration,
  not a smaller denominator. And split the survivor list: P12 quotes the first
  six of `survivors`, and five of those six were settled questions until
  `survivors_declared_equivalent` was separated out.

- **THE GATE DOES NOT READ A UNIT, IT READS A STREAM -- AND FOR A UNIT WHOSE
  EFFECT IS I/O THAT IS THE FIRST QUESTION, NOT THE COVERAGE ONE.** Unit #31,
  and it is a blindness sharper than any this campaign had recorded. `Debug` is
  called ~408,000 times across 23 of 27 scenarios and does its job every time,
  so it is neither #27's dead branches nor #1/#21/#26's dead call sites:

  ```
  every DebugOutData value DOUBLED       0 of 5,252,000
  GetWords perturbed, SAME libdiscon     1,857,893      <- the chain is alive
  the same edit, seen by the .dbg files  21,792+ records
  ```

  `Examples/vit_sim.py` builds `baseline_arrays/*.npz` from the `avrSWAP`
  channels the controller RETURNS; the unit writes `<RootName>.RO.dbg`, which
  nothing in the gate path opens. **No input could change that**, which is what
  makes it structural rather than a corpus gap -- and the two look identical in
  a red test that returns 0. Ask it of any unit that writes a file, a socket or
  a console line:

  ```
  grep -n "\.dbg\|open(\|savez" Examples/vit_sim.py   # what does the gate READ?
  ```

- **A UNIT THAT ASSIGNS NOTHING IN ITS SIGNATURE HAS NO GENERATED HARNESS, AND
  THE ANSWER IS A DIFFERENT ORACLE RATHER THAN A WAIVED P13.** Unit #31.
  `vit test-validate` compares MAPPED OUTPUTS; `Debug` writes only files, so the
  generated harness would compare the empty set and `vit_mutate` would report
  every mutant surviving -- a 0.000 that is a fact about the instrument.

  What replaced it keeps P11's and P13's shape and changes only what is
  compared: `scripts/dbgcheck.py` diffs the BYTES the unit writes between two
  builds that differ in one thing, and `scripts/dbgmutate.py` scores mutants by
  compiling each into `libdiscon.so` and re-running scenarios against the same
  archive.

  **And the reference being DATA dissolves unit #29's clean-tree requirement.**
  That rule exists because `vit_mutate` on an integrated tree routes both sides
  through the harness's own copy of the mutant. Here the reference is a
  committed archive written by a build containing no C++ unit at all: fixed
  before the sweep starts, unperturbable by it. **Where an oracle can be frozen
  as bytes, the requirement is about the reference and not about the tree.**

- **A CONFIGURATION SCALAR THAT EVERY SHIPPED INPUT HOLDS AT ONE VALUE HIDES
  WHOLE ARMS, AND COVERAGE REPORTS THE GUARD AS EXECUTED.** Unit #31, and it is
  the cheapest large win this campaign has had: one parameter, one new scenario,
  a third of a unit.

  ```
  all 14 Examples/DISCON*.IN         LoggingLevel = 1
  IF (LoggingLevel > 1) / (> 2)      REACHED in 23 scenarios, FALSE in all
  ROSCO_IO.f90:1108 / :1116          hits          <- coverage says "executed"
  ROSCO_IO.f90:1109 / :1118          none
  a perturbation inside that half    0 of 408,072 records
  the same edit at LoggingLevel = 3  15,999 of 48,014
  compare_op kills                   12 of 40  ->  21 of 40
  ```

  The scenario is ADDED, not modified: `Examples/vit_sim.py::run_scenario_28` is
  scenario 3's configuration with one patch, registered OUTSIDE
  `scenario_order`, so `--scenario 0` and every gate run behave exactly as
  before and no committed baseline moves (X3, P5). Pick the base scenario on
  what the hidden arms NEED -- here only scenario 3 already sets `CC_Mode` and
  `StC_Mode`, without which the dbg3 path's two `AddToList` loops stay
  unreachable even at `LoggingLevel = 3`.

  ```
  grep -h '! LoggingLevel' Examples/DISCON*.IN | sort -u   # one value -> one arm
  ```

- **A NO-COMPILE CAN BE A HOLE IN COVERAGE WEARING A DENOMINATOR'S CLOTHES, AND
  A MUTATOR FIX IS NOT VERIFIED BY THE OPERATOR THAT MOTIVATED IT.** Unit #31,
  three `cppmutate` defects (X2, fixed in the loop repo, each costed across all
  31 translations).

  ```
  template <=int W>            compare_op on a USER-DECLARED template   12 of 40
  std::FILE / f, return / p    arith_op on a pointer decl / deref        6 of 25
  1E + 99                      arith_op on a float's EXPONENT SIGN       5 of 25
  buf.size - pos()             swap_operands moving a call's name        1 of 10
  ```

  The third row is the one to recognise. It is not noise in a ratio: it meant
  the unit's two clamp constants had **no `arith_op` mutant at all**, so an
  operator that appeared to be measuring them was measuring nothing there. Read
  the no-compile list before accepting a ratio under the limit.

  And the fourth defect was created by the fix for the second: `_not_arithmetic`
  read `drop_factor`'s pattern group 2 as its OPERATOR, when for that rule group
  2 is the right OPERAND -- so it fired for two rules of three, and 6 of 7
  no-compiles survived in the very operator it was written to fix. Invisible in
  the diff, obvious in the sweep.

  ```
  python3 -c "import json;d=json.load(open('mutation/<U>.<op>.json'));
      [print(r['line'],r['before'],'->',r['after']) for r in d['results']
       if r['outcome']=='nocompile']"
  ```

- **FORTRAN'S `TRn` IS POSITIONAL, SO A TRAILING TAB WRITES NOTHING -- AND THE
  RECORD LENGTH IS WHERE A C++ REIMPLEMENTATION OF FORMATTED OUTPUT FIRST GOES
  WRONG.** Unit #31. `(99(a20,TR5:))` with 27 items looks like 675 bytes and the
  reference writes **670**: `TRn` moves the write position and emits nothing, so
  blanks appear only where a later descriptor writes past them, and the colon
  sits AFTER the `TR5` so the last item's tab is processed and then discarded.

  Three more from the same unit, each measured against gfortran rather than read
  out of a standard (`evidence/Debug/fmt_probe.sh`, 54 records byte-identical,
  red-tested at 1 and 26):

  ```
  ESw.dEe            C's "%.*E" agrees, EXCEPT the exponent WIDTH: C emits at
                     least 2 digits and as many as needed, Ee demands exactly e
                     and fills the field with '*' above it
  a CHARACTER(15) local   'NacHeadingTarget' SHIPS as 'NacHeadingTarge'; five of
                     185 literals truncate, and the width must have ONE site or
                     its mutant moves nothing
  a group with no data descriptor   `N(TR22,"(-)")` runs its FULL repeat count
                     after the one data item is consumed -- format control stops
                     at a DATA descriptor with nothing left, not at the item
  ```

  Build the probe by INCLUDING the translation from the test, so the helpers
  under test are the ones that ship (P4), and run it before integrating: it
  costs one compile and it is the only layer that sees a formatting defect
  without a full sweep.

- **A RED TEST'S REVERT CAN BE A SILENT NO-OP, AND THEN THE GREEN IT CERTIFIES
  REPORTS THE RED TEST'S OWN NUMBER.** Unit #30, second dispatch, and it is
  unit #23's `cp`-across-a-bind-mount hazard at the one place nobody had put the
  guard. `run_wrapper_redtest.sh` perturbs, builds, runs red, reverts, builds,
  runs green:

  ```
  red test                     1552 checked   9 failed
  revert-verified GREEN        1552 checked   9 failed   <- the same 9
  grep '2 - vit_j_Words'       nothing        the source IS reverted
  source  23:34:47.715
  object  23:34:46.732                        a second OLDER than its source
  ```

  `make` stat'd the file while the container still showed the pre-revert
  content, called the object up to date, and the mount caught up afterwards.
  **It appears only when the cycle is FAST** -- three seconds here, where the
  same script had passed twice earlier in the session at slower rebuilds -- so
  it will bite harder as the campaign's builds warm up.

  Two halves, and the second is what makes it deterministic:

  ```
  docker exec ... md5sum <the file>     # the content ARRIVED  (run_harness_stub.sh's rule)
  docker exec ... touch  <the file>     # and the object cannot be older than it
  ```

  The IN direction needs it more than the OUT one: a perturbation the container
  has not seen builds the ORIGINAL and reports a red test that stayed green,
  which reads as "this layer is blind" rather than as "the edit did not arrive".

- **A QUANTITY THE HARNESS SUPPLIES AND THE UNIT BRANCHES ON IS AN INPUT, AND A
  CONSTANT ONE NARROWS THE DOMAIN WITHOUT APPEARING IN ANY COVERAGE LINE.** Unit
  #30, second dispatch, and it is the widest of this campaign's corpus findings
  because it was true for every unit at once. A `CHARACTER(:), ALLOCATABLE`
  output crosses as a POINTER and a CAPACITY; both sides branch on the capacity
  and both refuse an assignment that does not fit -- the translation and the
  generated Fortran bridge, each with its own diagnostic. `emit.ALLOC_HEADROOM`
  added a constant 4096 to whatever the case supplied:

  ```
  cap = max(n_ErrMsg, 0) + 4096      the message is 127 bytes
  the refusal arm                    unreachable, in every case of every corpus
  '>' -> '>=' on the refusal         SURVIVED   <- differs only at cap == len
  ```

  It is not a range in `ranges.toml`, not a rule, and not in `rule_coverage`, so
  nothing reports it as a narrowing. Once it travels in the case stream and a
  rule sweeps it, the mutant dies at exactly one case. **Ask of any harness:
  what does IT choose that the unit READS?** Then ask whether a rule owns that
  choice.

- **A RULE CAN FIRE, VISIBLY CHANGE THE PROGRAM'S BEHAVIOUR, AND STILL KILL
  NOTHING -- BECAUSE IT VARIES THE RIGHT QUANTITY AGAINST A MOVING BASELINE.**
  Unit #30, second dispatch, and it cost one whole sweep before it was seen.
  R13's first version called the case builder once per rung, and the case
  builder draws its strings from `rng`:

  ```
  256 capacities x 256 different messages   boundary met by luck   MISSED
  the same block, one base case copied      boundary met exactly   killed by 1
  ```

  The block was ALIVE while it missed, and the proof was sitting in the other
  mutants: negating the refusal went from a mismatch to a SEGFAULT -- a 136-byte
  message written into a 6-byte buffer, which only a tight capacity can produce.
  **A sweep must hold everything else fixed**, which is R11's one-at-a-time
  principle applied to a quantity R11 does not reach. And the tell that a sweep
  is aimed correctly is that its kill count is PREDICTABLE: one boundary, one
  case.

- **A CORPUS RULE MUST PUT THE BOUNDARY WHERE IT CHANGES AN ANSWER, NOT MERELY
  WHERE IT EXISTS.** Unit #30, second dispatch, and it is why the first
  dispatch's proposed rule -- "put the declared width in the length ladder" --
  would have closed nothing on its own. The reference truncates `Words(1)` and
  `ExpVarName` into `CHARACTER(20)` locals and then COMPARES them, so a long
  string alone is invisible; what is visible is two strings that agree to
  character 20 and differ at 21.

  ```
  lengths {20, 21, 25}, strings drawn independently   the arms never match
  the same lengths, every string from ONE body        arm 1 reached, 9 cases
  with a per-parameter mark at index 20              '20'->'21' killed by 6
  ```

  The general form: **a rule aimed at a predicate must supply BOTH sides of it.**
  Two independently drawn strings are equal only by accident -- this unit's arm
  1 was reached 52 times in 1284 cases and every one was the all-blank shape,
  i.e. equality between two empty strings.

- **A BRIDGE-VS-OBJECT RULE LEARNED ON AN INTEGRATED TREE IS SILENTLY WRONG ON A
  CLEAN ONE, AND THE FAILURE IS A GREEN NUMBER.** Unit #30, and it is unit #29's
  entry two bullets down read in the other tree state. That rule asks LIBS -- if
  `<callee>.cpp.o` is in the link, drop the bridge -- and on a clean tree the
  same condition holds for the OPPOSITE reason: the object is a stale artifact
  of an earlier integrated build that `vit test-validate` globs in, and the
  Fortran callee is the real body.

  ```
  integrated tree   bridge -> Fortran <callee> -> <callee>_c -> bridge   SIGSEGV
  clean tree        C++ side -> <callee>.cpp.o      <- the TRANSLATION
                    Fortran side -> __mod_MOD_<callee>   <- the REFERENCE
                    links clean, runs, reports 1284 of 1284
  ```

  Two different implementations, one per side, so VIT's one-implementation
  property is gone and a defect in the callee is attributed to the unit under
  test. **Ask the TREE, not LIBS** -- it is `reset_to_clean.sh`'s own test and it
  maintains itself as units are integrated:

  ```
  grep -nE '(^|[^A-Za-z0-9_])<callee>_c *\(' rosco/controller/src/*.f90
  #   a hit -> a wrapper was integrated over it: DROP THE BRIDGE, keep the object
  #   none   -> the Fortran is the real body:    KEEP THE BRIDGE, drop the object
  ```

  And the rule has to run in BOTH harness modes with the link set computed per
  mode. Post mode adds every `.o` in the build tree on its own `make test LIBS=`
  line, so an object absent from the Makefile is still in the link -- the first
  post-integration run after the repair died on
  `multiple definition of int2lstr_c`.

- **A PROBE THAT WRITES TO AN OUTPUT THE REFERENCE NEVER TOUCHES IS THE CHEAPEST
  COUNTING INSTRUMENT HERE, AND IT NEEDS NO NEW TOOLING.** Unit #30. The
  differential harness reports a COUNT and not a SET (unit #27), so "how many
  cases satisfy P" has no direct answer -- unless P is made to move an output.
  `ErrVar%ErrStat` is compared by R4 and written by neither `CheckInputs` nor
  `ChkParseData`, so a stub that bumps it under P fails exactly the cases where
  P holds:

  ```
  ErrStat += 1 where Words(1) != Words(2)        990 of 1284
  ErrStat += 1 where Words(2) IS the name         50 of 1284
  the SILENT arm made to write aviFAIL = -7        2 of 1284
  ```

  Each is a stub through `run_harness_stub.sh` like any other. The third is the
  one that matters most: a path that writes NOTHING is invisible to a no-op stub
  by definition, so making it write is the only way to count it -- and
  `1284 - 1282 = 2` would have been an inference. **Before explaining a count,
  ask whether one stub can make the quantity an output.** Unit #30 explained one
  from an argument first and was wrong (`57b2f37`), which is unit #27's rule
  paid for twice.

- **A CORPUS WHOSE STRING LENGTHS ALL FALL SHORT OF A TRUNCATION BOUNDARY CANNOT
  SEE THE TRUNCATION, AND THE MUTANTS SAY SO THREE TIMES.** Unit #30. R6's
  ladder is `sorted({1, 2, ex0, ex0 + 5})`; the reference assigns `CHARACTER(*)`
  dummies into `CHARACTER(20)` locals.

  ```
  R6 lengths                    [1, 2, 6, 11]      every one below 20
  the truncation removed         0 of 1284 fail    <- the only red test that stays green
  '20' -> '21'                   SURVIVED
  min(len_src,len_dst)->len_src  SURVIVED
  ```

  Ask it of any unit whose Fortran declares a fixed-width CHARACTER local and
  assigns a `CHARACTER(*)` dummy into it -- `vit check`'s `narrowing-local`
  finds exactly that shape statically, so the two instruments answer the same
  question and only one of them is running:

  ```
  grep -nE 'CHARACTER\( *[0-9]+ *\)' <the unit's Fortran>   # then read what is assigned in
  ```

- **A MUTATION SWEEP CAN FAIL EVERY BUILD BECAUSE OF A `cp` THAT FINISHED
  SECONDS EARLIER, AND THE ARTIFACT IT WRITES IS A SCORE.** Unit #30, and it is
  unit #23's bind-mount hazard at sweep scale rather than at one file.

  ```
  31 of 31 mutants  "no compile"   score 0.000   <- run right after a stub restore
  the identical command, 90s later  score 0.742   31 of 31 compiled
  ```

  Nothing in the run says "the build was broken"; `nocompile` is reported per
  mutant and the summary is a number. What made it recoverable is
  `mutate_guarded.sh`: it refused to clear its marker, printed the intended hash
  and the live one, and the translation was put back from the evidence copy
  before anything was built. **The guard built for a hard kill caught a race**,
  which is worth saying because the argument for building it was about timeouts.
  Re-run the sweep before believing a `nocompile` ratio near 1.

- **TWO GENERATOR REFUSALS STOOD BETWEEN THIS CAMPAIGN AND ANY HARNESS FOR A
  UNIT TAKING A CHARACTER ARRAY, AND BOTH NAMED THEIR OWN WAY OUT.** Unit #30,
  fixed in the repo that owns each (X2).

  ```
  UNOBSERVABLE [character-array] Words          loop a1d76b0   map_signature
    -> EmitError: C parameter 'Words' is not in the mapped signature
  NotImplementedError: Callee bridge for 'Int2LStr' returns CHARACTER
    -> chkparsedata.hpp:117: error: 'int2lstr_c' was not declared in this scope
                                                  vit f4a711d   interface_gen
  ```

  The second is the one to recognise: a REFUSAL in the bridge generator becomes
  a compile error inside the TRANSLATION's own body, because `test_validate`
  catches the exception, notes it, and drops the callee from the declarations
  header. The note is printed and easy to miss. Read the callee notes before
  reading the C++:

  ```
  vit test-validate <U> <cpp> -f <file>.f90 --force 2>&1 | grep -i 'callee'
  ```

  Both refusals were narrow and stayed narrow: only a rank-1 CHARACTER array
  whose element count is a LITERAL moves, and only a SCALAR CHARACTER result.
  The X3 cost of the first was measured rather than argued -- exactly one dummy
  of that shape exists in this campaign's clean source, `ChkParseData`'s own.

- **THE CLEAN-TREE RE-TAKE OF AN INVALID MUTATION RUN LANDED NEXT TO IT, NOT AT
  THE OTHER END OF THE RANGE -- SO THE ENTRY BELOW IS RIGHT ABOUT THE DEFECT AND
  WRONG ABOUT ITS SIGNATURE.** Unit #29, second dispatch. Added rather than
  edited (P5); the bullet it corrects is the next one down, which says "the same
  mutants give scores at opposite ends of the range".

  ```
  integrated tree, mutant vs itself   4 killed of 173   0.0231   INVALID
  clean tree, mutant vs Fortran       8 killed of 173   0.0462   valid
  ```

  So the shape-check that entry proposes -- `mutation/<U>.json`'s kills against
  `harness/<U>.json`'s pass count -- fires IDENTICALLY on the valid run and
  would have been read as "still invalid". A number cannot answer a question
  about a configuration. **Read the reference object instead**, which is one
  command and the thing `_mutation_stamp.py` writes into the artifact:

  ```
  nm <build>/<Source>.f90.o | grep -E '_MOD_<unit>|<unit>_c'
  #   defines _MOD_<unit>, no undefined <unit>_c  -> the Fortran body: VALID
  #   undefined <unit>_c                          -> the wrapper: the mutant
  ```

- **A VALIDATOR WITH NO EARLY RETURN HAS ONE OBSERVABLE OUTPUT NO MATTER HOW
  MANY CHECKS IT HAS, AND A CORPUS THAT TRIPS TWO OF THEM AT ONCE MEASURES ONE.**
  Unit #29, second dispatch, and it is that unit's kernel finding one layer up:
  the kernel is blind because the capture is a VALID configuration and nothing
  fails; the differential harness is blind because in all 16,769 cases something
  ALWAYS fails, twice.

  `CheckInputs` writes only `ErrVar%aviFAIL` and `ErrVar%ErrMsg`, ~180 checks
  assign both, and none returns -- so the last failing check wins. Three
  one-line perturbations of the single message sink say what that costs:

  ```
  aviFAIL = -1 -> -7 everywhere   16,769 of 16,769 FAILED   <- control: alive
  no ErrMsg ever written          16,769 of 16,769 FAILED   <- >=1 error always
  first-writer-wins               16,769 of 16,769 FAILED   <- >=2 errors always
  the translation                          0 FAILED
  mutation score                  8 of 173, 0.0462
  ```

  The second and third are the measurement: `aviFAIL` is `-1` in every case, so
  it distinguishes nothing, and the first failing check differs from the last in
  every case, so no case isolates one. 165 of 173 mutants are above the last
  check and invisible. **`first-writer-wins` is the probe that separates "the
  corpus does not reach it" from "the corpus reaches it and cannot see it"**,
  and it is one line at the message sink:

  ```cpp
  if (ErrVar->aviFAIL != 0) return;   // in assign_errmsg
  ```

  Ask it of any unit whose outputs are an error code and a message. Both probes
  are meaningless on an integrated tree, for the same reason the mutation run
  was: `bash evidence/CheckInputs/errmsg_masking.sh` (exit 0 == masking holds)
  opens and closes the reset window itself.

- **A SWEEP THAT DOES NOT FIT IN ONE FOREGROUND COMMAND IS SPLIT BY OPERATOR,
  NEVER BACKGROUNDED, AND THE UNION MUST BE ABLE TO REFUSE.** Unit #29, second
  dispatch. 192 mutants at ~9.5 s is 32 minutes against a 600-second command.

  ```
  vit_mutate.py <U> --operator <op> [--operator <op>...] --out mutation/<U>.clean.<name>.json
  python3 scripts/_mutation_merge.py --unit <U> --cpp <the .cpp> --part ... --out mutation/<U>.json
  ```

  The merge derives the operator population from `harness.cppmutate`, not from
  the parts -- **a filtered run's own `operators` field is computed AFTER the
  filter**, so a union checked against it is a tautology, and the first version
  of the merge was written that way and caught by its own refusal. The
  per-operator COUNT then also settles that each part scored every mutant its
  operators produce:

  ```
  arith_op 17  compare_op 40  const_tweak 40  drop_call 2  drop_factor 6
  index_offset 33  negate_cond 40  swap_callee 2  swap_operands 12   = 192
  ```

- **A MUTATION RUN ON AN INTEGRATED TREE COMPARES THE MUTANT AGAINST ITSELF,
  AND THE ARTIFACT CANNOT SAY SO.** Unit #29, and it is unit #21's and unit
  #24's "a green that measured nothing" with the sign flipped -- here it is a
  RED that measured nothing, which is no better.

  ```
  173 mutants   4 killed   score 0.0231
  the same corpus, against real Fortran:   16,769 of 16,769, 0 failed
  ```

  169 survivors on a corpus that clean is the tell: 40 `compare_op` flips
  cannot all be equivalent. The build is why. `harness.sh` drops
  `<stem>.cpp.o`, so the integrated wrapper's `<unit>_c` is undefined and the
  integration shim supplies it -- and then

  ```
  <unit>_f90 -> the WRAPPER -> <unit>_c -> vit_integration_shim.o
             -> the harness's own copy of the translation   <- the MUTANT
  ```

  Both sides run the mutant. **Run the mutation sweep on the CLEAN tree**, the
  same configuration the pre-integration harness runs in, where the caller's
  object carries the real Fortran body.

  `harness/<U>.postintegration.json` carries a `measures:` field naming exactly
  what its run can and cannot see. `mutation/<U>.json` has no equivalent, and
  the same mutants give scores at opposite ends of the range depending on an
  answer it does not record. Check the number's SHAPE against the unit's own
  harness before believing it:

  ```
  python3 -c "import json;m=json.load(open('mutation/<U>.json'));\
      h=json.load(open('harness/<U>.json'));\
      print(m['killed'],'/',m['mutants'],' vs harness',h['checked'],h['failed'])"
  ```

- **AN ERROR THAT NAMES ITS PARAMETER COSTS ONE PASS; ONE THAT DOES NOT COSTS
  THREE. SAME FILE, SAME UNIT, SAME AFTERNOON.** Unit #29, and it is the
  cheapest rule this campaign has produced because it changes nothing you run,
  only what you write when you fix a generator.

  ```
  EmitError: CheckInputs: CntrPar_SU_LoadStages has 17 element(s)
             but its extents say 0            <- diagnosed and fixed in ONE pass
  TypeError: 'float' object is not iterable   <- THREE passes, three distinct causes
  ```

  Both come out of `harness/emit.py::write_cases`, four rules downstream of
  every one of their causes. The second names no parameter, no rule and no
  case, and it was raised by three different defects in a row -- a knob naming
  a whole array in R7, the same block again in R7b, and a knobbed body against
  a knobbed extent. Each round cost a full generate-and-emit.

  **The way out is not to read the traceback again, it is to scan the artifact
  the generator produced**: every array parameter holding a scalar, counted,
  with the first offending case index. That found the R7b copy in one run and
  named all seven parameters:

  ```
  CntrPar_F_NotchFreqs   scalar in 3078 case(s), first at case 18726
  ```

  3,078 is R7b's own reported case count and 18726 is where its block begins,
  so the attribution is the count rather than an argument.

- **"OTHER UNITS' OBJECTS STAY, THEY ARE PART OF THE REFERENCE BUILD" WAS TRUE
  FOR 28 UNITS BECAUSE NONE OF THEM HAD A CALLEE -- AND THE OBVIOUS REPAIR
  BUILDS A LOOP.** Unit #29, `harness.sh`. `checkinputs_callees.f90` defines
  `addtolist_c`; so does the integrated `addtolist.cpp.o`. The link says
  `multiple definition`, and there are two ways out of it:

  ```
  drop the OBJECT, keep the bridge   -> bridge -> Fortran AddToList -> WRAPPER
                                        -> addtolist_c -> ... SIGSEGV, case 0,
                                        no message, a core file
  drop the BRIDGE, keep the object   -> one definition, and it is the one the
                                        shipped program calls
  ```

  On an integrated tree the Fortran callee IS a wrapper around its own C++, so
  VIT's "both sides share one callee implementation" holds trivially once the
  bridge is gone. Read the integrated source before choosing which duplicate to
  delete; the comment in the script said the opposite and had never been tested.

  Two more fell out of the same corner. `vit test-validate` writes bridges
  DEFINING `<callee>_c` and declared them nowhere, so the build failed inside
  the translation's own body and read like the translation's fault. And
  `vit_mutate` then refused to score -- `baseline is not green (nocompile)` --
  because `<stem>.cpp.o` is dropped and the integrated wrapper calls
  `<unit>_c`: the integration shim has to be in the MAKEFILE, which is
  `harness.sh`'s own 2b argument one object over.

  ```
  grep -c '<callee>_c' <unit>_test/*.f90 <unit>_test/*.cpp   # who DEFINES it
  nm -g --defined-only <the integrated .o> | grep '<callee>_c'
  ```

- **A UNIT WHOSE ONLY OUTPUT IS AN ERROR SIGNAL IS INVISIBLE TO ANY CAPTURE
  TAKEN ON A WORKING CONFIGURATION.** Unit #29, and it is a different blindness
  from #27's dead branches: here the body RUNS, broadly, and answers "fine"
  every time.

  ```
  the translation            1 of 1, 426 of 426 IDENTICAL
  wrong constant aviFAIL=-7  0 of 1     <- the chain is alive
  the WHOLE UNIT deleted     1 of 1     <- and it sees none of it
  ```

  `errmsg` does not appear among the 426 rows at all: KGen guards its
  comparison with `IF (ALLOCATED(var%errmsg))` and a passing configuration
  never allocates it. **Check the field log for the unit's own output before
  reading a kernel pass as evidence** -- a field that is absent is not a field
  that matched.

  And the one case is not a window that was too narrow. The site is called once
  per scenario at invocation index 1 of its OWN counter, so all 24 scenarios
  write `<Unit>.0.0.1` and overwrite each other. A name collision does not
  respond to widening `kgen.invocation`:

  ```
  ls kernel/<Unit>/<Unit>.*.*.*     # one file -> every scenario overwrote the last
  ```

- **A DEFERRED-LENGTH STRING HAS NO BYTES PAST ITS LENGTH, AND TWO RED RUNS DO
  NOT TELL YOU WHICH WAY TO GO.** Unit #29. `ErrVar%ErrMsg` is
  `CHARACTER(:), ALLOCATABLE`; the assignment reallocates to exactly `LEN`. The
  view crosses on a wider staging buffer, and what belongs in the region past
  `n_ErrMsg` is decided by whoever allocated the buffer, not by ROSCO.

  ```
  leave the previous message's tail   16,729 of 16,769 FAILED
  blank-fill it                       16,769 of 16,769 FAILED
  clear it to NUL                              0 FAILED
  ```

  The two red counts differ by 40 and both say `ErrVar.ErrMsg`; neither points
  anywhere. **The first differing BYTE does**: `a=0x20 b=0x00` at exactly index
  `n_ErrMsg`, because the oracle side is a zeroed vector the bridge writes
  `n_ErrMsg` bytes into. Unit #27 says to compute the two SETS rather than
  explain two equal counts; this is that rule one representation lower down.
  Print the offset and the two bytes before choosing a fill:

  ```
  for (k...) if (a[k] != b[k]) { first diff at k; break; }
  ```

- **A PROBE THAT READS A CONSTANT FROM MATHEMATICS RATHER THAN FROM THE PROGRAM
  IS MEASURING A DIFFERENT PROGRAM, AND IT CAN REPORT FAILURE ON A CORRECT RUN.**
  Unit #28, and it is P7 applied to the instrument instead of to the translation.

  `wrap_360`'s call site is `StrAzimuth = wrap_360(...)*D2R`, so a kernel stub
  that deletes the wrap moves each affected case by 360 degrees *in the caller's
  post-multiplied units*. A check written against `2*math.pi` said
  `DOES NOT MATCH` on a green run:

  ```
  the kernel's own difference line        6.2831853036
  360 * D2R, Constants.f90:23             6.2831853036   <- eleven digits
  2 * PI                                  6.2831853072
  ```

  ROSCO defines `D2R = 0.01745329251` and `PI = 3.14159265359` as truncated
  literals. The disagreement is in the TENTH digit -- above the printed
  precision of the line being checked, so the probe was neither obviously right
  nor obviously wrong until the constant was parsed out of the source. Parse it:

  ```
  grep -nE ':: *(D2R|R2D|PI) ' rosco/controller/src/Constants.f90
  ```

  Ask it of any check that converts units, compares an angle, or reproduces an
  arithmetic identity the reference also computes.

- **A MODEL OF A CALL SITE IS AN ARGUMENT; THE STUB RUNS YOU ALREADY OWE ARE A
  MEASUREMENT.** Unit #28. The kernel's field log records the CALLER's local,
  not the unit's argument, so "what domain did the capture cover" has no direct
  answer in it. The first attempt modelled the site -- `x = 0.45*(n-1)` degrees,
  read off the first two cases -- which put every case in the right arm and
  reproduced **6 of 41** captured values bit for bit, because `LocalVar%Time`
  accumulates by `DT` rather than being multiplied out.

  What replaced it costs no extra build, because C6's red tests are these runs:

  ```
  passthrough  (both arms deleted)   20 IDENTICAL  21 NOT   <- IDENTICAL == did not wrap
  no-low-branch (x<0 arm deleted)    41 IDENTICAL   0 NOT   <- excludes the low arm
  no-high-branch(x>=360 deleted)     20 IDENTICAL  21 NOT   <- 0 + 21 = 21, the partition
  ```

  A case the pass-through stub matches is a case the reference did not wrap, by
  definition rather than by inference; the no-low run then makes the other 21
  unambiguous. Read the red tests for the second thing they can say before
  writing a model of the caller.

- **WHEN TWO UNITS IN ONE FILE DIFFER ONLY IN A COMPARISON SPELLING, THE SIBLING
  IS THE RED TEST.** Unit #28. `wrap_180` is `.le.` low and `.gt.` high --
  `(-180, 180]`. `wrap_360`, one screen down and otherwise the same three
  statements, is `.lt.` low and `.ge.` high -- `[0, 360)`. Reading either across
  into the other moves `x = 0.0`, `x = -0.0` and `x = 360.0` and **nothing else
  in the real line**, so it is invisible to both bit-exact layers (the kernel's
  41 cases hold neither boundary) and it is the perturbation a translator is
  most likely to actually produce.

  ```
  the sibling's comparison spelling    7 of 134 differential cases FAILED
  ```

  Those 7 come from R6's predicate knob and unit #14's signed-zero rung -- the
  second unit in a row with that rung inside its margin. Grep the unit's own
  file for a neighbour of the same shape before writing the translation:

  ```
  grep -nE '\.(lt|le|gt|ge)\.' <the unit's Fortran>   # then read the neighbours' pairs
  ```

- **A UNIT CAN HAVE ONE LIVE ARM AND ONE DEAD ONE, AND THE PER-UNIT GATE NUMBER
  REPORTS NEITHER.** Unit #28, the fourth unit with a second gate number worth
  carrying (after `saturate`'s upper clamp, `sigma`'s two clamps and
  `wrap_180`'s two branches) and **the first where one of the two is non-zero**.

  ```
  whole unit -> 0.0            84,477 of 5,252,000     <- what STATUS.md's list carries
  BOTH arms deleted            31,579
  the HIGH arm alone           31,579
  the LOW arm alone                 0                  <- and this is invisible in it
  ```

  So `wrap_360` reads as a gate-VISIBLE unit while one of its two arms is
  invisible, which is worse than the incompleteness unit #27 recorded: there the
  reader is told nothing, here the reader is told something true that points the
  wrong way. Run the arms singly and check the partition -- the arms are
  disjoint, so `low + high` must equal `both`, and unit #28 got that identity
  from three instruments (kernel 0+21=21, harness 36+15=51, gate 0+31,579).

  The two zeros are blindness and not breakage because their control is on the
  SAME BUILD. And the CAUSE matters: unit #27's dead arms are dead because the
  simulation harness discards the injection aimed at them, which is repairable;
  this one is dead because `360*Time*AWC_freq(1)` is non-negative by
  construction, which is not. **Say which kind before logging it as a gap.**

- **A SCENARIO'S INJECTED INPUT CAN BE OVERWRITTEN BEFORE THE CONTROLLER READS
  IT, SO THE SIM SOURCE SAYS WHAT WAS INTENDED AND THE COMMITTED `.dbg` SAYS WHAT
  ARRIVED.** Unit #27, and it is the reason two of five layers are blind for that
  unit. `Examples/vit_sim.py` writes, under its own comment:

  ```python
  # Inject avrSWAP values not handled by call_controller
  controller_int.avrSWAP[23] = nac_vane_rad      # avrSWAP(24) NacVane
  controller_int.avrSWAP[36] = nac_heading_rad   # avrSWAP(37) NacHeading = 350 deg
  ```

  Two of the six indices under that comment ARE handled by `call_controller`
  (`control_interface.py:209` and `:211`), which runs after the injection and
  before `call_discon`. Index 23 survives only because `Y_MeasErr` is set to the
  same value two lines above; **index 36 is silently replaced by the accumulated
  yaw position**, so the one input in this whole corpus aimed at `wrap_180`'s
  branches never reaches the controller.

  Refutable from committed artifacts with nothing run, which is what makes it
  cheap: `Yaw_Err` and `NacHeadingTarget` are both `.dbg` channels and
  `Yaw_Err = wrap_180(NacHeadingTarget - NacHeading)`, so a heading of 350 forces
  an interval and **10,632 of 23,999 timesteps sit outside it**.

  ```
  grep -n 'avrSWAP\[<N>\]' rosco/toolbox/control_interface.py   # a hit -> injection lost
  python3 evidence/wrap_180/heading_injection_discarded.py      # exit 0 == refuted
  ```

  NOT FIXED (X3): repairing it moves every baseline and every compared count.
  Ask it of any unit whose inputs come from an injected `avrSWAP` index, BEFORE
  reading the scenario's comment as true.

- **TWO RED TESTS WITH THE SAME FAILURE COUNT ON THE SAME CORPUS ARE NOT THE SAME
  MEASUREMENT, AND NOTHING IN THIS CAMPAIGN CAN TELL.** Unit #27, found by
  getting it wrong in a commit message first. `wrap_180`'s pre-integration no-op
  and its post-integration sign flip both report **130 of 136**:

  ```
  no-op (return 0.0)  blind on 6: four at x = 0.0, plus x = +/-360.0  (ref(x) IS 0.0)
  sign flip (-x)      blind on 6: the FOUR BOUNDARY cases +/-180.0, plus +/-360.0
  overlap                      2, not 6
  -0.0                         in NEITHER set
  ```

  The negation set is the point: `.le.` low and `.gt.` high send BOTH endpoints to
  `+180`, so a sign flip is invisible at exactly the cases that pin the unit's
  asymmetry. Unit #26's `redtest_corpus_skew.py` compares counts ACROSS corpora
  and says `0 SKEWED` here, correctly -- the same-corpus form is invisible to it
  because a red-test artifact records a COUNT and not a SET.

  ```
  python3 evidence/wrap_180/the_six_insensitive_cases.py   # exit 0 == sets differ
  ```

  Until an artifact carries the failing case indices, do not explain two equal
  counts -- compute the two sets. An explanation from an argument was wrong here
  in both of its halves.

- **A UNIT CAN BE EXERCISED 675,987 TIMES WITH EVERY BRANCH DEAD, WHICH IS NOT
  THE SAME BLINDNESS AS AN UNREACHED CALL SITE.** Unit #27, the fourth blind unit
  after #1, #21 and #26 and the first that is not dead. The check is an identity
  over three counts derived three different ways, and it needs no new tooling:

  ```
  hits at the FUNCTION line              675,987
  hits summed over the six CALL SITES    675,987   EQUAL
  hits on the ELSE line (pass-through)   675,987   EQUAL   <- so both branches: 0
  ```

  The P10 control belongs in the same run and for this unit it was in the same
  FILE: `wrap_360`, two screens down, is the same shape with a live branch
  (`x >= 360`, 15,199 hits), so an absent coverage key means zero rather than
  un-instrumented.

  What it changes is the reading of the gate. A whole-unit no-op moves 206,976
  values, so `wrap_180` lands on STATUS.md's gate-visible list -- and the stub
  deleting BOTH branches moves 0 of 5,252,000. **One number per unit answers "can
  the gate see this unit", never "which of its branches", and three units now have
  a second number sitting in their evidence** (`saturate`'s upper clamp, `sigma`'s
  two clamps, these two branches).

  ```
  python3 evidence/wrap_180/coverage_branch_deadness.py    # exit 0 == branches dead
  ```

- **A RED TEST AND THE GREEN IT CERTIFIES MUST NAME THE SAME CASE COUNT, AND SIX
  OF THIS CAMPAIGN'S TWENTY-ONE COMPARABLE PAIRS DO NOT.** Unit #26, second
  dispatch. The unit's own README tabulated three stub red tests as "of 403";
  all three were taken at **377** and their committed JSON said so. The corpus
  was widened by the constant-step block, only the GREEN was re-taken, and the
  three red numbers were carried down as if a wider corpus could not change them.
  Re-run at 403 the no-op fails **373**, not 363.

  A red test on a subset still shows the green can fail, so this invalidates no
  disposition. What it invalidates is the PAIR: the two artifacts sit in the same
  directory, each recording the one number that would settle it, related by
  nothing. Census it -- read-only, over artifacts that already exist:

  ```
  python3 evidence/unwrap/redtest_corpus_skew.py    # pre AND post, per unit
  ```

  ```
  NotchFilter 2652/1380   SecLPFilter 2884/2284   Int2LStr 144/103
  interp1d 497/473        HPFilter 832/829        StateMachine 2890/3610  <- +720
  post-integration                                26 of 26 EQUAL
  ```

  Three things the census says that an argument could not. It runs **both ways**
  -- StateMachine's red test saw 720 cases its green never did, so there the
  GREEN is the stale artifact -- which "re-take the red test after widening"
  would not catch. Five units are **not comparable at all** (no pre-integration
  red test), which is a different answer from "equal" and is reported as one.
  And post-integration is clean STRUCTURALLY, not luckily: post mode reuses the
  generating run's case file, so that pair cannot drift. **Regenerating the
  corpus is the only step that can break it**, so ask it after any widening, of
  the unit that widened and of every unit whose detectors the widening reaches.

- **A GATE ARTIFACT TAKEN BEFORE `vit integrate` MEASURED A LIBRARY WITH NO C++
  IN IT, AND IT PASSES WITH THE IDENTICAL NUMBER.** Unit #26, second dispatch,
  and it is unit #23's re-take rule one build earlier -- there the wrapper was
  missing `--reverse-copy`, here the wrapper is missing entirely.

  ```
  gate/unwrap.json @ aabf439   pre-integration    5,252,000 / 0
  gate/unwrap.json @ 45e7daf   integrated build   5,252,000 / 0
  ```

  Nothing in the artifact distinguishes them and the number never will. If a
  gate result predates the `vit integrate` that put the unit in the library,
  RE-TAKE IT; the cost is one gate run and the alternative is a green that
  measured a program the campaign is not shipping. Check before believing any
  inherited gate artifact:

  ```
  git log --oneline -1 -- gate/<U>.json ; git log --oneline -1 -- rosco/controller/src/<u>.cpp
  ```

- **A GENERATED WRAPPER LINE IS IDENTICAL ACROSS UNITS, SO A WRAPPER
  PERTURBATION MUST BE ANCHORED TO THE UNIT AND NEVER TO THE STRING.** Unit #26,
  second dispatch. `CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)`
  -- the line the post-integration red test deletes -- occurs **three times** in
  `Functions.f90` (interp1d:198, sigma:511, unwrap:529), because a generator
  wrote all three. A `str.replace` perturbs three units, measures none of them,
  and produces a red artifact indistinguishable from the right one.

  Assert the neighbours, cut by index, and verify the others survive:

  ```
  grep -c '<the generated line>' <the .f90>        # 1 -> safe. >1 -> anchor it.
  ```

  This is the `--reverse-copy` finding read from the other side: what makes
  generated code convenient to write is what makes it dangerous to perturb by
  matching.

- **A UNIT CAN BE DEAD BECAUSE ITS CALLER STOPS ONE STATEMENT SHORT, AND THE
  GUARD-BESIDE-THE-CALL READING IS WHAT SAYS SO.** Unit #26. `unwrap` has two
  call sites and both are dead in all 27 scenarios -- the third such unit after
  #1 (`AddToList`) and #21 (`UpdateZeroMQ`) -- but only one of them is dead for
  #21's reason.

  ```
  Controllers.f90:322  IF ((OL_Mode > 0) .AND. (Ind_GenTq > 0))   407,976 hits
  Controllers.f90:339  LocalVar%AzBuffer = UNWRAP(...)                  0 hits
  ReadSetParameters.f90:778  CALL Read_OL_Input(...)          3 hits (10,14,24)
  ReadSetParameters.f90:780  RETURN                           3 hits (10,14,24)
  ReadSetParameters.f90:807  CntrPar%OL_Azimuth = Unwrap(...)           0 hits
  ```

  The first is an unentered guard. The second is not: the three scenarios that
  set `OL_Mode > 0` DO reach the block, and then `Read_OL_Input` fails --
  `Examples/example_inputs/OL_Mode2_Input.dat` is not in this tree (unit #17) --
  and the `RETURN` two statements later takes them out. **Reading the guard
  alone would have said "no scenario configures it"; reading the RETURN says
  "three scenarios configure it and a missing FILE stops them".** Print the
  whole path from the guard to the call, not the guard and the call:

  ```
  python3 evidence/unwrap/coverage_deadness.py     # exit 0 == the body is dead
  ```

- **A `DO WHILE` WHOSE ONLY PROGRESS IS AN ADDITION DOES NOT TERMINATE ONCE THE
  ADDITION ROUNDS TO NOTHING, AND THE TRANSLATION MUST NOT TERMINATE EITHER.**
  Unit #26, the sixth upstream ROSCO defect of the "the reference has no answer"
  family (#17 non-termination, #21 indeterminate, #23 abort).

  ```
  smallest 2**k with v + 2*PI == v :  7.2057594037927936e16   (2**56)
  y(2) = 1e17    still looping after 17,900,000,000 iterations, y(2) unmoved
  y(2) = 1e300   still looping after 17,900,000,000 iterations, y(2) unmoved
  y(2) = 1e3     TERMINATED after 159 iterations
  ```

  Nothing in this campaign's corpus reaches it, and that is a fact about the
  CORPUS: `_bounds` defaults an array to +/-1e3 and `_real_magnitude_ladder` is
  gated on `not q.dims`, so no hot rung can land in an array. **No
  `harness/ranges.toml` entry was written**, deliberately -- a pin narrows a
  domain, and this one is already narrow enough that the branch is unreachable
  without one. Ask it of any `DO WHILE` whose body is `x = x + <constant>`.

- **AN ORDER LADDER VARIES THE SIGN OF A DIFFERENCE AND PINS ITS MAGNITUDE AT
  ONE, SO A PREDICATE ON THE SIZE OF A STEP IS UNREACHABLE BY IT.** Unit #26,
  and it is unit #12's `NonDecreasing` finding one predicate over.

  Unit #12 added the ORDER LADDER because `_fill_array` returns a strictly
  ascending ramp in every case and `A(i+1) - A(i) <= 0` therefore had one
  answer. Its bodies are `[1.0, 2.0, ..., L]` and its permutations, which is
  exactly right for a predicate on the SIGN of a difference and reaches nothing
  at all for one on its MAGNITUDE. `unwrap` tests
  `y(i) - y(i-1) .LE. -PI`, and the reversed unit-step run gives `-1`:

  ```
  order ladder off (355 cases)      the `+2*PI` branch DELETED   0 of 355 fail
  order ladder, unit steps (366)                                 0 of 366 fail
  + steps spanning +/-1e3   (377)                                4 of 377 fail
  + steps at the reference's own constants (403)   the `<=`/`<` mutant DIES
  ```

  Three scales, and the third is a different rule from the second: spanning the
  range makes the branch REACHED, naming the reference's own constant makes the
  branch's BOUNDARY reached. `-PI` is not a literal anywhere in `Functions.f90`
  -- it is a named `PARAMETER` in `Constants.f90` -- so `literals_from`, which
  mines numbers out of the unit's own file, cannot see it and
  `named_constants_from`, which mines named PARAMETERs campaign-wide, can. Ask
  it of any predicate whose left side is an EXPRESSION rather than a name:

  ```
  grep -nE '\([Ii][^)]*\) *[-+] *[A-Za-z_]+\([Ii]' <the unit's Fortran>
  ```

- **AN ORDER PREDICATE WRITTEN AGAINST A LOCAL COPY OF A PARAMETER IS INVISIBLE
  TO A RULE THAT MATCHES PARAMETER NAMES.** Unit #26. `order_arrays_from` looks
  for the same NAME subscripted twice in one statement, and `unwrap` writes

  ```fortran
  y = x                                ! whole array, no subscripts
  DO while (y(i) - y(i-1) .LE. -PI)    ! the order predicate, on `y`
  ```

  so the detector returned the empty set and the ladder never fired. Closed by
  ONE HOP through a whole-array copy -- `LHS = RHS`, both bare names, RHS a
  parameter -- and no further: a copy of a SLICE, of an EXPRESSION, or a second
  hop are each a claim about the reference the scan cannot check. Measured
  against the CLEAN baseline source and not the working tree, which matters
  more than it sounds: 21 of the 25 scored units have a WRAPPER for a body now,
  a wrapper contains no subscript at all, and a sweep over the tree reports
  `old=[] new=[]` for every one of them and looks like a proof.

  ```
  bash evidence/unwrap/x3_cost_order_alias.sh    # 4 detectors fire, 1 moves
  ```

- **THE THREE GENERATORS DISAGREE ABOUT `DIMENSION(SIZE(x))` ON A FUNCTION
  RESULT, AND ONLY ONE OF THE THREE IS RIGHT BY ACCIDENT.** Unit #26, the
  fourth such disagreement (units #8, #17, #22) and the second on a RESULT.

  ```
  vit interface (the SHIPPING wrapper)   fine -- its `x` is the original `x(:)`
  test_validate (the harness bridge)     `REAL(8), DIMENSION(SIZE(x))` on `x(*)`
                                         -> "The upper bound in the last dimension
                                            must appear in the reference to the
                                            assumed size array 'x'"
  harness map_signature                  REFUSED the result outright, then
                                         `EmitError: C parameter 'unwrap_result'
                                         is not in the mapped signature`
  ```

  All three carry the FUNCTION's own dimension text across, and the text is
  legal in exactly the scope the reference wrote it in. Fixed in both (X2, VIT
  and loop, `_bridge_result_dims` and `_result_extents`' `size_of`), each
  resolving `SIZE(<arg>)` to the extent parameter `build_c_params` already
  emits rather than evaluating anything. `SIZE(x) + 1` still falls to the same
  refusal, on purpose.

  And one level down: **one extent that sizes TWO arrays was written into the
  case stream twice and declared twice**, `error: redeclaration of 'int32_t
  n_x_a'`. That one needs no X3 argument -- a duplicate declaration is a
  COMPILE error, so no already-scored unit can have been carrying one.

- **THE BOUND A UNIT COMPARES AGAINST CAN BE A CONSTANT OF THE CONFIGURATION,
  AND THEN NO INVOCATION WINDOW REACHES THE BRANCH IT GUARDS.** Unit #25, and
  it is unit #22's constant-argument finding with one level of indirection --
  there the argument was a literal in the SOURCE, here it is a literal in the
  fourteen `.IN` files.

  `sigma` clamps to `y0` below `x0` and to `y1` above `x1`. At
  `Controllers.f90:526` the bounds are `CntrPar%IPC_Vramp(1:2)`, which is
  `9.120  11.400` in **all 14** `Examples/DISCON*.IN`, and the 62 captured
  values of `x` run 9.5804 .. 10.7998:

  ```
  the LOWER clamp deleted   62 of 62 PASSED      the differential harness: 116 of 1069
  the UPPER clamp deleted   62 of 62 PASSED      the differential harness:  94 of 1069
  the cubic replaced by y0  32 of 62, 30 rows    the differential harness: 812 of 1069
  ```

  Not a small kernel and a large harness -- on those two branches the kernel's
  answer is an ABSENCE. Widening the window cannot help, for unit #22's reason:
  a window varies the surrounding state, never a constant. Ask it of any
  predicate whose other side is a `CntrPar%` field, before choosing a site:

  ```
  grep -h '<the field>' Examples/*.IN | sort | uniq -c    # one line -> a constant
  ```

  What reached them is R6's two-sided-predicate block, added at unit #20. A
  corpus rule added five units earlier is the entire reason two of this unit's
  three branches are tested at all -- the same shape as unit #24's signed-zero
  margin, and the second time in three units that a rule's value showed up long
  after the unit that paid for it.

- **A CALL SITE WITH FEWER CALLS THAN THE WINDOW'S LATER RANGES CANNOT BE
  CAPTURED THERE, AND THAT IS A REJECTION ON A COUNT RATHER THAN ON A STUB.**
  Unit #25. Unit #24 says to run the branch-deleting stub at EACH candidate site
  before spending the cycle on one; this is the case where the committed
  coverage settles it one step earlier and cheaper.

  ```
  ControllerBlocks.f90:612   800 hits, one scenario
  kgen.invocation            0:0:1-20, 0:0:12000-12020, 0:0:23900-23920
  ```

  **Two of the three ranges are past that site's last call**, so 42 of the 62
  slots do not exist and the capture is the first 20 invocations -- for a ramp,
  the 20 timesteps where `x` is a hair above `x0`. Coverage says the same about
  the branches: the `sigma = y0` line has no hits at all in that site's only
  scenario. Do the arithmetic before extracting; it costs one query, and unit
  #24's rule is served by its PURPOSE (choose on what the kernel can see) even
  where its procedure is not followed literally. **Say which you did.**

- **THE DETERMINATE-WRONG CONSTANT IS CHOSEN AGAINST THE CALL SITE'S ARGUMENTS,
  NOT AGAINST THE TYPE.** Unit #25, and it is one turn past unit #22's rule that
  a liveness stub must be determinate and finite.

  `sigma`'s `y0` is the literal `0.0_DbKi` at `Controllers.f90:526`, so a ZERO
  stub returns the right answer for every case that takes the lower clamp -- the
  campaign's default liveness stub, measuring this unit's argument list instead
  of its kernel. `-7.25` passes **0 of 62**. Read the actual arguments at the
  chosen call site and pick a constant none of them can be:

  ```
  grep -n '<Unit>(' <the caller>.f90     # any LITERAL actual argument?
  ```

- **A CAPACITY GUARD IS UNREACHABLE FOR TWO DIFFERENT REASONS AND ONLY ONE OF
  THEM IS BY CONSTRUCTION.** Unit #25, the fifth unit to carry
  `assign_errmsg`'s `s.size() > n_ErrMsg_cap` and the first whose message is not
  a fixed literal.

  ReadAvrSWAP, ExtController, UpdateZeroMQ and interp1d assign literals of known
  length into a 4 KiB staging buffer, so the guard cannot fire in any
  configuration this campaign can build. `sigma` assigns
  `'sigma:' // TRIM(ErrMsg)`, whose length **grows with its input**, so the
  guard is unreachable only because the generator supplies no `ErrMsg` within
  six characters of the buffer -- a claim about the CORPUS, not about the
  program. Same site, same declaration, weaker evidence. Write the weaker one
  down as the weaker one; a declaration inherited by its site name is a
  declaration nobody re-checked.

- **A TRANSLATION WHOSE BODY IS A CALL GETS ZERO MUTANTS, AND ZERO IS A
  DIFFERENT FAILURE FROM A LOW SCORE.** Unit #24, and it is unit #22's
  `cppmutate` finding one step past displacement into absence.

  All nine of the original operators need an arithmetic operator, a comparison,
  a subscript or a numeric literal. `std::fmin(std::fmax(inputValue, minValue),
  maxValue)` has none, and neither its ARGUMENT ORDER nor its CALLEE NAME was a
  site. `done.py`'s P12 fails `total <= 0` by name, which is right: the operator
  set reached nothing, so it says nothing about the instrument.

  ```
  python3 -c "import sys;sys.path.insert(0,'/workspace/translation-loop');\
      from harness.cppmutate import mutants;\
      print(len(mutants('<U>', open('<cpp>').read())))"    # 0 -> P12 cannot pass
  ```

  Ask it BEFORE integrating, not after: it decides the disposition and it costs
  one command. **CLOSED, on unit #24's second dispatch, by adding the operator
  and paying its price in the same cycle** -- `drop_call`, `swap_call_args`,
  `swap_callee`, loop `b9fb5ee`. Still NOT closable by hand-writing
  `mutation/<U>.json`: authoring both the mutants and the artifact that grades
  them is exactly what `min_mutation_score: 1.0` exists to shut, and the hand
  measurement's role is to CROSS-CHECK the tool (it reports the same 151 and 100
  the tool's two `drop_call` mutants do), never to replace its artifact.

- **THE X3 COST OF A NEW OPERATOR IS TWO SEPARATE QUESTIONS AND ONLY ONE OF THEM
  IS EXPENSIVE.** Unit #24, second dispatch, and it is the resolution of the
  refusal units #21 and #22 both recorded.

  Unit #22 declined to add an operator because *"a new operator changes the
  mutant set of every unit already scored, and unlike a corpus addition it can
  produce a SURVIVOR in a unit that closed at 1.000."* Both halves are true.
  They are also both MEASURABLE, and the first one is free -- `_mid` is derived
  from `unit|operator|before|after|nth`, so an addition cannot move an existing
  id:

  ```
  # old tool vs new, over every already-scored translation. Run BEFORE deciding.
  git show HEAD:harness/cppmutate.py > /tmp/oldmut/cppmutate_old.py
  # ... mutants() from each, compare the id SETS per unit
  #   0 lost, 35 gained, 7 of 24 units affected  -> every committed artifact and
  #   every declared equivalence still resolves; the gap is what is UNMEASURED
  ```

  Lost ids would be a re-take. **Zero lost is not a re-take, it is a debt**, and
  the debt is only the new mutants in the affected units -- 6 units here, scored
  in one sweep into `mutation/<U>.call_operators.json` beside their untouched
  artifacts (`vit_mutate.py --operator`, which stamps `operators_filter` so a
  filtered run cannot be read as a full one).

  And unit #22's fear was correct: **8 survivors in three units that had closed
  at 1.000** -- GetPath 0.500, GetRoot 0.333, GetWords 0.667, every one the same
  `std::min(len_src, len_dst)` bounded-copy clamp and its argument swap. That is
  a newly VISIBLE defect class, not a newly created one, and their committed
  scores stay true about what they scored.

- **A TOKEN-LEVEL OPERATOR THAT REWRITES A CALL IS TYPE-BLIND, AND THE
  NOCOMPILE LIMIT IS WHAT TELLS YOU SO.** Unit #24, second dispatch, and it cost
  two full sweeps to learn.

  `drop_call` replaces a call with its first argument and `swap_call_args`
  exchanges two -- both well typed only when the types agree, which no regex and
  no scanner can know. Unrestricted, this campaign's string units came back
  **38%, 43%, 55%, 60%, 73%, 80%, 89%, 91% and 100% unbuildable**:
  `std::strtod(c, &stop) -> c` is a `char*` where a `double` is wanted.

  ```
  REFUSING TO SCORE: 21 of 23 mutants (91%) failed to COMPILE, above the 25% limit
  ```

  **Do not raise the limit.** It is the guard that catches a genuinely broken
  build -- a second definition of the unit in the link, every mutant failing,
  1.000 measured on nothing -- and spending it on punctuation is unit #21's
  finding repeated. Restrict the OPERATOR instead: `_VALUE_PRESERVING` is a
  table of callees whose result type is their first argument's type and whose
  arguments share one type. 231 mutants across 13 units became 35 across 7, and
  every one of them compiles.

  Same shape one level down: a `{` after the close paren does not identify a
  definition head, because a constructor's member-initialiser list starts with
  `:`. The test that needs no list of type names is that **a PARAMETER is two
  bare identifiers in a row**, which no C++ expression is.

- **THE LIVENESS STUB AND THE DOES-IT-REACH-THE-BRANCH STUB ARE DIFFERENT STUBS,
  AND ONLY THE SECOND CHOOSES A CALL SITE.** Unit #24, and it is interp1d's
  no-boundary-branches finding turned into a step of C2 rather than a note
  written afterwards.

  Coverage recommended `ControllerBlocks.f90:332` -- 407,976 hits, 23 scenarios,
  and NON-ALIASED, which units #7 and #20 both say to prefer. Its kernel is alive
  and blind to the whole point of the unit:

  ```
  ControllerBlocks.f90:332   zero stub 23 of 62 rows IDENTICAL -> ALIVE
                             passthrough (no saturation)  62 of 62 PASSED
  Controllers.f90:102        zero stub  21 of 62 cases   -> ALIVE
                             passthrough  40 of 62 -- a clamp fires in 22
                             the MIN deleted  62 of 62, 14,260 of 14,260 IDENTICAL
  ```

  So the first site sees NEITHER clamp and the second sees ONE, and no hit count
  distinguishes them. Build the stub that deletes the branch the unit EXISTS for
  and run it at each candidate site before spending the cycle on one. Keep the
  rejected site's artifact -- it is the measurement that justifies the choice.

- **gfortran's `MIN`/`MAX` ARE `fmin`/`fmax`, AND BOTH BRANCH SPELLINGS ARE WRONG
  AT A SIGNED ZERO.** Unit #24. Unit #14 says an explicit `IF (CornerFreq < 0)`
  must NOT be written as `fmax` -- true, and it does not generalise to the
  INTRINSIC, which is what a careless reading of it would do here.

  ```
  # evidence/saturate/saturate_expr_sweep.sh -- 12,167 triples, bits not values
  fmin(fmax(v, lo), hi)                       0 differ
  t = (v > lo) ? v : lo; (t < hi) ? t : hi  789 differ
  t = (lo > v) ? lo : v; (hi < t) ? hi : t  561 differ
  ```

  Read the values AT RUNTIME. A probe with literal operands measures gfortran's
  front end folding them, and the shipped controller runs the back end. The rule
  is the same one in both places -- transcribe the shape the reference has -- and
  in both places the discriminating input is a negative zero: the branch spelling
  dies on **exactly 1 of 451** corpus cases, the rung unit #14 added after its own
  `dict.fromkeys` dedup absorbed `-0.0`. Two units later that block is the entire
  margin.

- **A CALL SITE INSIDE A `FUNCTION` BODY PRODUCES A KERNEL THAT WILL NOT COMPILE,
  AND THE FIRST CAUSE IS A TRANSPOSED TYPO IN KGEN.** Unit #24.

  `gen_kernel_callsite_file.set_args` sets `node.tosubr = True` to convert the
  hoisted parent block into a SUBROUTINE the driver can `CALL`, and
  `EndFunction.tokgen` reads `tosubr`. `SubProgramStatement.tokgen` read
  **`tosurb`** -- in `kgen/parser/block_statements.py` AND `base/`, the two-site
  shape this file has recorded four times. So the kernel came out with a
  `FUNCTION` header, an `END SUBROUTINE`, and a `CALL`: 29 errors, opening with
  `Function result 'picontroller' has no IMPLICIT type`. Fixed in KGen (X2,
  `d3d6516`); inert for a SUBROUTINE parent block by construction, not by
  sampling -- `clsname` is `'SUBROUTINE'` on both branches, and the two
  `if not tosubr:` blocks read `typedecl` and `result`, which a Subroutine
  statement has not got.

  **The fix gets one error further and does not make it work.** The parent block
  becomes `SUBROUTINE picontroller`, so the callsite statement
  `PIController = saturate(...)` is an assignment to the subroutine's OWN NAME --
  *"has already been host associated"* -- and the function RESULT is neither
  declared as a local nor captured as state, so `!local output variables` is
  empty and the kernel would compare nothing even if it built. NOT FIXED: it is
  a change to the state analysis. Ask the question before extracting:

  ```
  # what encloses the call site -- a SUBROUTINE, or a FUNCTION?
  awk 'NR<=<CALL_LINE>' <caller>.f90 | grep -nE '^ *(RECURSIVE )?(REAL|INTEGER|LOGICAL|TYPE)?.*\b(SUBROUTINE|FUNCTION)\b' | tail -1
  ```

  It blocked nothing here -- 8 of `saturate`'s 17 call sites are in SUBROUTINE
  scope, and picking one of those is C2, not an X2 evasion. It WILL block a unit
  whose only usable call sites sit inside a FUNCTION body.

- **A BIND-MOUNTED FILE DOES NOT RELIABLY `stat()` NEWER THAN WHAT WAS BUILT FROM
  IT, SO `make` KEEPS A STALE HEADER AND THE RUN REPORTS THE PREVIOUS MUTANT.**
  Unit #24, and it is unit #18's symptom with a DIFFERENT cause -- worth knowing
  because unit #18's fix does not prevent it.

  The generated harness Makefile's prerequisites are CORRECT here
  (`saturate_test.o: saturate_test.cpp saturate.hpp` and
  `saturate.hpp: <the translation>`), so there is nothing to fix in it. The file
  is written on the Mac and `stat()`ed from inside the container, and the mtime
  does not always come back newer than the artifact built from it seconds
  earlier. A first hand-mutant run read `passthrough 100 of 451` where
  `scripts/harness.sh` had independently measured **202**, and adjacent mutants
  came back in EQUAL PAIRS -- which is the tell, and it is more legible than any
  single number.

  ```
  rm -f <stem>.hpp <stem>_test.o test && make -s test     # delete, do not trust a timestamp
  ```

  Cross-check one hand-run figure against the same measurement taken through
  `scripts/harness.sh`. Two instruments agreeing on one number is what says the
  recipe is sound; here `271` appears independently as the hand mutant for
  swapped bounds and as the post-integration red test for the same defect.

- **`vit integrate` WITHOUT `--reverse-copy` SILENTLY DROPS EVERY WRITE A UNIT
  MAKES TO A SCALAR FIELD OF A VIEW-TYPE INOUT DUMMY, AND BOTH BIT-EXACT
  LAYERS PASS IT.** Unit #23. This is the case the post-integration harness
  exists for, and the first time in this campaign it has caught something.

  `interp1d` writes `ErrVar%aviFAIL` and `ErrVar%ErrMsg`. The generated wrapper
  populated the view, called the C++ and returned. **The kernel scored 62 of 62
  and the gate compared 5,252,000 values with 0 mismatched on that build** --
  the kernel marshals the view itself so the wrapper is not on its path, and
  the 27 scenarios hand the unit well-formed tables with `aviFAIL == 0` so the
  dropped fields are never written. A perturbed return value moves 1,341,803
  values: the gate is not blind to the UNIT, it is blind to an OUTPUT of it.

  ```
  grep -n 'INTENT(INOUT)' <the unit's Fortran>   # then, for each view-type one:
  grep -n '<Dummy>%<scalar field> *=' <the unit's Fortran>   # any hit -> --reverse-copy
  ```

  The post-integration run is what says so, and it says it in the mismatch
  LIST rather than the count -- 454 of 497 cases, naming `ErrVar.ErrMsg` and
  `ErrVar.n_ErrMsg` and nothing else. Re-integrating is cheap and does NOT need
  a reset: `git checkout HEAD -- <the .f90> CMakeLists.txt vit_translated.h`,
  then `vit integrate ... --apply --reverse-copy`.

  RE-RUN THE GATE AFTERWARDS. The first gate artifact measured the build with
  the broken wrapper; it passes either way, which is exactly why it has to be
  re-taken rather than kept.

  NOT FIXED IN VIT HERE: inferring the flag changes the wrapper every unit
  ships (X3) and 12 integrated units take a `TYPE(ErrorVariables)` dummy.
  DECISIONS.md, as a candidate.

- **A REFERENCE CAN ABORT ON ITS OWN ERROR PATH, AND THEN EVERY GENERATED CASE
  KILLS THE ORACLE BEFORE ANY COMPARISON HAPPENS.** Unit #23, and it is unit
  #17's non-termination finding one exit code over.

  `interp1d`'s size-mismatch branch assigns a 38-character literal to
  `CHARACTER(:), ALLOCATABLE :: ErrMsg` -- which REALLOCATES it to 38 -- and
  then formats 53 characters into that record with an internal WRITE:

  ```
  LEN after the assignment = 38     characters the WRITE formats = 53
  Fortran runtime error: End of record        exit 2
  ```

  `_extent_plan` makes extents PAIRWISE DISTINCT on purpose, so the very first
  case had `n_xData = 3, n_yData = 4` and the run reported only
  *harness produced no JSON*. The tell is the Fortran runtime error ABOVE that
  line -- read the whole stderr, not the last line.

  `harness/ranges.toml` now takes a THIRD kind of entry beside its ranges and
  `no_oracle`: `{ same_as = "<other extent>" }`, which says two INPUTS are only
  JOINTLY admissible. A range narrows one parameter alone and `no_oracle`
  excludes an OUTPUT; neither can state this. Never silent -- it prints, it
  lands in R5's own detail line as `TIED, so NOT pairwise distinct`, it lands
  in the artifact's `tied_extents`, and a name matching no extent is an error.

  **And a tie has to survive the rules that build their own extent map.** The
  character ladder and the ORDER ladder both write `{**ex0, d: L}`; here the
  order ladder fires for BOTH arrays, so the tie held everywhere except in the
  one rule most likely to break it. `_extents_with` re-applies it, and
  propagates it in BOTH directions -- shortening the follower shortens the
  leader, rather than silently undoing the ladder.

- **A PREDICATE WHOSE OTHER SIDE IS A REDUCTION OF A VARIED ARRAY HAS NO NAME
  TO PIN, AND ITS CROSSING VALUE IS NOT IN THE CASE STREAM AT ALL.** Unit #23,
  the fifteenth corpus blind spot (R10, `reduction_pairs_from` in
  `vit_harness.py`, the block in `generate.py`).

  `IF (xq <= MINVAL(xData))` and `ELSEIF (xq >= MAXVAL(xData))` are the whole
  shape of a table lookup outside its own table. Unit #20's two-sided rule
  needs both sides to be PARAMETERS; here one side is a FUNCTION of one, whose
  value is whatever `_fill_array` drew. `<=` against `<` and `>=` against `>`
  differ on exactly one input each and **473 cases produced neither**.

  What makes it load-bearing rather than tidy is that the KERNEL is blind to
  the same two branches, and the check is one stub:

  ```
  # the shipped translation with the two ENDPOINT branches deleted
  cp evidence/<U>/<u>.no-boundary-branches-stub.cpp kernel/<U>/<u>.hpp
  cd kernel/<U> && rm -f <u>.o kernel.exe && make -s build && ./kernel.exe
  # 62 of 62 PASSED, 248 of 248 IDENTICAL -- every captured query is interior
  ```

  R10 draws the body FIRST, computes the reduction, sets the scalar to it and
  to its two neighbouring representable numbers -- over the ordinary body AND
  ITS REVERSE, so the extremum is once the first element and once the last. The
  reverse is not decoration: a translation substituting `xData(1)` for
  `MINVAL(xData)` agrees on an ascending body and disagrees on a descending one.

- **`cp` ONTO A BIND-MOUNTED FILE IS READ HALF-WRITTEN OFTEN ENOUGH TO NEED A
  GUARD, NOT A WARNING.** Unit #23, and the RUNBOOK has carried the warning
  since unit #4. It happened twice in one cycle here, and the first time it
  looked like a syntax error in a file that has none:

  ```
  interp1d.hpp:25:11: error: 'ch' does not name a type; did you mean 'char'?
     25 | constexpr char MsgSizeMismatch[] = " xData and ...";
  ```

  gcc printed the line from the file it could read a moment later and the
  compiler had read a truncated one. Hash it from INSIDE the container before
  building anything:

  ```bash
  cp "$src" kernel/<U>/<u>.hpp
  want=$(md5 -q "$src")
  for i in 1 2 3; do
    got=$(docker exec vit-dev bash -lc "md5sum /workspace/.../<u>.hpp | cut -d' ' -f1")
    [ "$want" = "$got" ] && break; cp "$src" kernel/<U>/<u>.hpp
  done
  [ "$want" = "$got" ] || { echo "HASH MISMATCH"; exit 1; }
  ```

  Print the hash into the artifact. A stub run whose input nobody verified is
  a measurement of an unknown program.

- **A SURVIVOR THAT IS UNREACHABLE AND A SURVIVOR THAT IS EQUIVALENT ARE TWO
  DIFFERENT CLAIMS, AND THE ARTIFACT MUST SAY WHICH.** Unit #23, five declared.

  Three are EQUIVALENT and the proof is not the obvious one. Loosening the
  MINVAL reduction's `<` to `<=` makes it re-assign on a tie, and that DOES
  change the stored bits -- **1,666 of 69,905 swept tuples** -- because
  `-0.0 <= 0.0` is true where `-0.0 < 0.0` is false. What makes the mutant
  equivalent is the weaker, sufficient claim: the CONSUMERS cannot tell, and
  the probe reports **0** disagreeing (tuple, xq) pairs. Sweep the consumer,
  not the intermediate.

  Two are UNREACHABLE, and the honest form of that claim is a COUNT over the
  corpus rather than an argument about it. The pattern is the shipped
  translation plus counters and nothing else changed, run through the harness's
  own Makefile:

  ```
  evidence/interp1d/errmsg_extremes_probe.cpp
  #   n_ErrMsg 1 .. 10   cap 4097 .. 4106   largest message 42
  #   s.size() == cap: 0        n_ErrMsg <= 0: 0
  ```

  Note `make -s test` overwrites `<u>.hpp` from the translation -- the FIRST
  Makefile rule is `<u>.hpp: <the translation>` -- so swap the probe over
  `translations/<Module>/<u>.cpp` itself and restore afterwards.

- **`NaN > kgen_tolerance` IS FALSE, SO A NaN OUTPUT SCORES `IN_TOL` AND THE
  KERNEL PRINTS PASSED.** Unit #22, and it is a SECOND kernel-scoring hole
  beside unit #19's, with a different cause and the same symptom.

  Unit #19's is about MAGNITUDE -- an absolute tolerance against an output near
  1e-52. This one is about the COMPARISON ITSELF: KGen's tolerance branch is
  `IF (rmsdiff > kgen_tolerance) -> OUT_TOL ELSE -> IN_TOL`, and IEEE says a
  NaN is not greater than anything. A translation that leaves an automatic array
  uninitialised makes the downstream field NaN and the kernel scores it PASSED.

  ```
  # the tell is in the CSV, not the verdict -- grep the rms, not the status
  grep ',array,IN_TOL' kernel/<U>/verify_fields.csv | grep -i nan
  ```

  Telling the two apart costs one stub and the reading is opposite each way.
  Build one that is DETERMINATE and FINITE and wrong -- here the 3x3 identity
  with `2.0` on the diagonal -- and score it:

  ```
  no-op stub              62 of 62 PASSED,  p IN_TOL, rms = NaN
  wrong-constant stub      0 of 62 passed,  p OUT_TOL, rms = 0.37807928263323742
  ```

  The second line says the window is fine and `p` is of order 1. Reading the
  first alone gives "the kernel cannot see this unit", which is false.

  NOT FIXED HERE: a NaN guard changes the pass basis of every kernel this
  campaign has run. X3 and SPEC 8.4, the Driver's call; DECISIONS.md.

- **A UNIT WHOSE ONLY ARGUMENT IS A LITERAL AT ITS ONLY CALL SITE HAS A KERNEL
  THAT IS A ONE-ROW LOOKUP TABLE, AND THE WIDENED INVOCATION WINDOW CANNOT
  HELP.** Unit #22, and it is unit #6's `GetPath` shape reached from the
  argument side rather than the call-count side.

  `MATMUL(identity(3) - ...)` -- the argument is written in the SOURCE. 62
  captured cases, one input value, and a constant stub reading nothing scores
  **13,950 of 13,950 IDENTICAL**. Widening `kgen.invocation` varies the
  SURROUNDING state, never the argument, so no window setting reaches it. Ask
  the call site before choosing a window:

  ```
  grep -n '<Unit>(' <every caller>     # is every actual argument a LITERAL?
  ```

- **THE DIFFERENTIAL HARNESS MAPPED AN ARRAY-VALUED FUNCTION RESULT AS A SCALAR
  INPUT, AND THE ONLY REASON ANYONE FOUND OUT IS THAT THE FORTRAN SIDE WOULD NOT
  COMPILE.** Unit #22, both halves fixed rather than worked around (X2), VIT
  `ab75fa0` and loop `20b0dbb`.

  `FUNCTION identity(n) RESULT(A)` with `REAL(DbKi), DIMENSION(n, n) :: A`.
  `vit interface` has crossed it since the campaign began --
  `REAL(C_DOUBLE), INTENT(OUT) :: identity_result(*)`, the wrapper declaring the
  automatic array and passing it. `test_validate.generate_fortran_bridge`
  declared the result a SCALAR: *Incompatible ranks 0 and 2 in assignment*.

  The loop's `map_signature` had the matching gap and it is the one that
  matters: `build_c_params` emits `double* <Func>_result` with NO extent, and
  `arg_by_name` has no entry for a RESULT, so it fell past even the "treated as
  a SCALAR" note an array DUMMY gets -- **the unit's only output was varied as
  an input on the +/-1e3 default and eight bytes of it were compared.** Had the
  bridge compiled, that harness would have passed and been committed.

  Third disagreement between VIT's two generators (units #8, #17, #22). The
  RUNBOOK already says to ask both; what this adds is that **the loud one is not
  the dangerous one**.

- **R5 EMITTED ONE SHAPE FOR EVERY UNIT WITH ONE FREE EXTENT, AND REPORTED IT AS
  A VARIED ONE.** Unit #22, the fourteenth corpus blind spot, in `generate.py`'s
  `_extent_plan` (loop `20b0dbb`).

  R5's second shape comes from `rotate=1`, which PERMUTES the sizes among the
  extents -- and a permutation of one element is the identity. So `ex1 == ex0`,
  the append was skipped in silence, every case ran at the same shape, and the
  coverage line read `1 varied extent(s) at [3]`. `bump=1` shifts the size
  instead; the line now reads `at [3] and [4]`.

  Load-bearing immediately: a hardcoded stride (`* n` written `* 3`) dies on
  exactly two of 29 cases here and **one of them is the n = 4 case this fix
  adds**. Read the rule's own detail line rather than its `applied` flag:

  ```
  python3 -c "import json;print([r for r in json.load(open('harness/<U>.json'))\
      ['rule_coverage'] if 'R5' in r])"     # one bracket or two?
  ```

- **`cppmutate` CANNOT MUTATE A PARENTHESISED OPERAND, AND VIT'S OWN CHECK
  REGISTRY REQUIRES THE PARENTHESES.** Unit #22. `_OPERAND` is
  `identifier | number`, so `(j - 1) * n` produces no `arith_op`, no
  `drop_factor` and no `swap_operands` mutant -- the stride multiplier of a
  column-major index expression is unmutated, in every translation that follows
  the `exponent-grouping` check.

  NOT FIXED HERE, and for a stronger reason than the KGen entry above: widening
  the operand pattern changes the MUTANT SET of every unit already scored, and
  unlike a corpus addition -- which can only kill more -- a new operator can
  produce a SURVIVOR in a unit that closed at 1.000. Campaign-wide re-take, the
  Driver's call.

  What to do meanwhile is one unit's worth of the missing measurement, by hand
  and committed. `evidence/identity/stride_probes.sh` is the pattern -- and note
  `make -s test`, not `make -s`: the generated harness Makefile's FIRST target is
  `<stem>.hpp`, so a bare `make` copies the header, relinks nothing, and `./test`
  reports the PREVIOUS translation's verdict. All three probes read `failed 0`
  that way, which is unit #18's kernel finding in the harness Makefile. And
  `|| true` on the run: `./test` exits non-zero when cases fail, which under
  `set -e` kills the script and leaves an EMPTY artifact.

- **ASK WHETHER A DEFECT CLASS IS UNFALSIFIABLE ON THIS UNIT BEFORE COUNTING ITS
  ABSENCE AS COVERAGE.** Unit #22. Transposing the index expression --
  `(i - 1) * n + (j - 1)` -- moves **0 of 29 cases at every n**, because the
  identity matrix is SYMMETRIC and `A(i,j)` equals `A(j,i)` in every element. No
  corpus can distinguish column-major from row-major here.

  That is not a gap in the corpus and it is not a mutant to declare equivalent;
  it is a property of the unit's output, and it means VIT's column-major rule is
  ENFORCED on this translation and UNTESTABLE by it. Worth asking of any unit
  whose output has a symmetry: a diagonal matrix, a sorted array, a set.

- **A REFERENCE CAN HAVE NO ANSWER, AND THE TELL IS A HANDFUL OF CASES THAT
  DISAGREE WHILE THOUSANDS AGREE.** Unit #21, and it is the first output in this
  campaign that no instrument could constrain because the ORACLE is absent
  rather than because an instrument is blind.

  `UpdateZeroMQ` declares `real(C_DOUBLE) :: setpoints(8)` and copies it into
  eight `LocalVar%ZMQ_*` fields. Nothing writes it: the `call zmq_client(...)`
  that would is inside `#ifdef ZMQ_CLIENT`, and this campaign's build does not
  define it. The harness said so before anyone read the source that way --
  **4,175 of 4,179 cases agreed on every output and 4 disagreed on those fields
  alone**, the reference answering `68bb11a718b90000`, a leftover pointer.

  A near-green with a tiny, field-clustered failure set is the shape. Ask which
  fields, then ask what writes them:

  ```
  python3.12 -c "import json;d=json.load(open('harness/<U>.json'));\
      import collections;print(collections.Counter(m['output'] for m in d['mismatches']))"
  grep -n '<the local the fields are copied from>' <the unit's Fortran>   # assigned where?
  ```

  Three calls in one process settle it beyond argument -- fresh frame, then
  after a routine that fills the same stack region with a recognisable value:
  `evidence/UpdateZeroMQ/setpoints_indeterminate_probe.f90` returns `NaN`, then
  `1.0`, then `-7.25`. **Six of the eight track the previous frame verbatim.**

  `harness/ranges.toml` now takes `{ no_oracle = "why" }` beside its ranges, and
  the two are different judgements on purpose: a range narrows an INPUT domain,
  `no_oracle` says the reference is not a function of its arguments on an
  OUTPUT. Never silent -- it prints, it lands in the artifact's
  `no_oracle_outputs`, and a stated name matching no compared field is an error,
  because an exclusion that excludes nothing reads as a cost that was paid.

  The translation still WRITES those fields. The write happens in the reference
  and destroys what was there; only the VALUE is undefined. What it must not do
  is spell one undefined value eight times -- see the next entry.

- **A RATE GATE IS UNREACHABLE BY EVERY LADDER, AND A SAMPLE OF THE CORPUS IS
  NOT A SAMPLE OF ITS CONJUNCTIONS.** Unit #21, the thirteenth corpus blind spot
  and the largest by reach: **4,175 of 4,179 cases entered nothing at all.**

  `MOD(LocalVar%n_DT, CntrPar%n_DT_ZMQ) == 0 .OR. LocalVar%iStatus == -1` is the
  whole procedure. `MOD(A, B) == r` is true only on the multiples of B, so two
  independent ladders hit it by accident or not at all -- and unit #20's
  two-sided-predicate rule cannot cross it either, because its crossing value is
  the other side's value and here it is a MULTIPLE of the other side.
  `predicate_knobs_from` cannot see it: its left-hand side must be a NAME.

  R9 (`divisibility_pairs_from` in `vit_harness.py`, the block in
  `generate.py`). **Two wrong versions came first and each is the lesson:**

  ```
  120 fresh cases, one per knob combination per divisor   score 0.2593 -> 0.2593
  a strided sample of the corpus, re-run with A from B    score 0.2593 -> 0.2593
  EVERY case re-run with the gate satisfied               score 0.2593 -> 0.600
  ```

  A gate does not need cases of its own; it needs **every other rule's cases to
  arrive at a procedure that does something**. And the stride is what killed the
  second version: 20 of 4,179 cases are the knob combinations, and
  `CntrPar%ZMQ_Mode > 0` against `>= 0` differ only at `ZMQ_Mode == 0`, which
  only a knob produces, and it has to meet the gate IN THE SAME CASE. With every
  case re-run that mutant dies on 7 of 8,334.

  The no-op red test is the cheap way to see reach before scoring anything:

  ```
  # it must fail every case that reaches the body -- here 12 of 4,179 before,
  # 4,167 of 8,334 after, and the second number is half the corpus by construction
  ```

- **ONE INDETERMINATE VALUE SPELLED EIGHT TIMES IS SEVENTEEN UNKILLABLE
  MUTANTS.** Unit #21, and it is unit #1's "name a size once" and unit #4's
  restatement rule meeting an output no comparison can see.

  Written as `double setpoints[8] = {0.0, ...}` with eight subscripted copies,
  the translation offered eight initialisers, eight subscripts and an extent --
  seventeen sites on a quantity no input can change and no oracle covers.
  One `const double no_setpoint = 0.0` leaves three, which the reference itself
  has (`ZMQ_PitOffset(1:3)`), and those are declared with their measurement.

  The third restatement in the same file was found by the score rather than by
  reading, and is worth the habit:

  ```fortran
  ErrVar%ErrMsg = ' >> ...'                               ! statement 1
  ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)   ! TRIM of statement 1
  ```

  **TRIM applied to a value the line above just assigned is TRIM of a literal.**
  Its four transcribed sites -- the bound, the decrement, the `n > 0` guard and
  the `<= 0` early return -- all survived. Ask of every intrinsic: does its
  argument come from an INPUT, or from a statement in this same procedure?

- **THE DECOMPOSED BRIDGE DECLARES TYPES THE REFERENCE NEVER NAMES, AND EVERY
  EARLIER UNIT COMPILED BY ACCIDENT.** Unit #21, fixed in VIT rather than worked
  around (X2).

  `vit test-validate` copies the reference's own USE statements into the bridge.
  `UpdateZeroMQ` carries `USE ROSCO_Types, ONLY : LocalVariables,
  ControlParameters, ErrorVariables` -- correct for the reference, and the
  bridge then declares `TYPE(rlParams), POINTER :: vit_nested_LocalVar_rlP` for
  every nested-type FIELD, names no `ONLY` list written for the reference can
  contain. gfortran: *"Derived type 'rlparams' is being used before it is
  defined"* plus five *"has no IMPLICIT type"*.

  `ReadAvrSWAP` decomposes the SAME `LocalVariables` and builds -- because
  `ReadSetParameters` USEs `ROSCO_Types` at module scope and re-exports it, so
  its bridge reaches the nested types through `USE ReadSetParameters`. **A module
  that keeps its imports inside its procedures has no such route.** Check the
  file, not the type:

  ```
  grep -n 'USE ROSCO_Types' <the unit's Fortran>   # at MODULE scope, or inside?
  ```

- **`compare_op` WAS MUTATING `static_cast<...>`, AND IT WAS EATING THE GUARD
  THAT CATCHES A BROKEN MUTATION RUN.** Unit #21 (X2).

  Ten of this unit's thirty mutants were `static_cast<=int32_t>` and cannot
  compile -- 33%, above the 25% at which `vit_mutate` REFUSES to score. That
  refusal exists for a real failure: after integration the build tree can hold a
  second definition of the function, every mutant fails to LINK, and the old
  arithmetic scored 1.000 while measuring nothing. Spending it on punctuation
  is how a real broken run would slip under it.

  ```
  python3 -c "import sys;sys.path.insert(0,'/workspace/translation-loop');\
      from harness.cppmutate import mutants;\
      print([str(m) for m in mutants('<U>', open('<cpp>').read()) if 'cast' in m.before])"
  ```

  Fixed by masking template brackets in `harness/cppmutate.py`, the same
  discipline the comment and literal masks already use. **A capped operator hides
  it**: on a large file `compare_op` stops at 40 sites, so the brackets do not
  raise the mutant count, they DISPLACE real comparisons. `Read_OL_Input`'s
  committed artifact carries 29 of 135 no-compile, 21% -- under the limit, same
  cause, and its 135 include brackets in place of comparisons nobody scored.

- **A DEAD UNIT'S GATE RED TEST IS ITS CONTROL'S JOB, AND THE CALLER'S COVERAGE
  IS WHAT PROVES THE UNIT DEAD.** Unit #21, and it is the answer to the case
  where `coverage/line_coverage.json` holds an EMPTY dictionary for the unit's
  own file -- which the RUNBOOK already says cannot tell "never ran" from "never
  instrumented".

  Read the CALL SITE's file instead, which is instrumented and has hits:

  ```
  python3 -c "import json;d=json.load(open('coverage/line_coverage.json'));\
      h=d['hits']['DISCON.F90'];print([(l, sum(h[str(l)].values()) if str(l) in h else 0) \
      for l in (103,104,141,142)])"
  # 103 the guard 407,976   104 the CALL 0   141 the guard 36,024   142 the CALL 0
  ```

  A guard with hundreds of thousands of hits and a CALL with none is a
  measurement, not an absence -- and it says in advance that the gate red test
  will move 0, so take the same-build control in the same session.

- **A UNIT WHOSE OUTPUT IS A FIELD IT ALSO READS IS SELF-ALIASING THROUGH TIME,
  AND ITS NO-OP STUB PASSES 60 OF 62.** Unit #20, and it is unit #7's
  `CALL GetRoot(RootName,RootName)` finding reached from the other direction --
  there two dummies aliased one variable, here ONE FIELD aliases its own
  previous value.

  `StateMachine` writes `LocalVar%PC_State` and `LocalVar%VS_State`, both fields
  of the `INTENT(INOUT)` argument, so they arrive carrying the previous
  timestep's answer. Holding its state is what a state machine is FOR, so on
  every call where the state does not change, "write nothing" and "write the
  right answer" are the same bytes:

  ```
  no-op stub            60 of 62 PASSED,   3 rows move (cases 1 and 2 only)
  right-constant stub   61 of 62 PASSED,   2 rows move (case 1 only)
  wrong-constant stub    0 of 62 PASSED, 123 rows move (all 62 cases)
  ```

  So the WRONG-constant stub is the liveness test, exactly as unit #7 said, and
  the right-constant stub says how little is left: **61 of the 62 captured cases
  have the SAME ANSWER.** Ask it of the outputs before choosing the stubs:

  ```
  grep -n 'INTENT(INOUT)' <the unit's Fortran>   # then: does the unit READ a
  # field it also WRITES, or is the write unconditional on every path?
  ```

  A unit whose every path writes every output is NOT exempt -- this one's are,
  and the mirror is about the VALUE being unchanged, not about the write being
  skipped.

- **A RED TEST PROVES VISIBILITY; IT DOES NOT MEASURE IT, AND THE TWO DIFFER
  HERE BY A FACTOR OF FORTY.** Unit #20.

  `VS_State_Region_1_5` written as `VS_State_Region_2` moved **36,577 of
  5,252,000**, and every moved channel was in **scenario 12** -- while the five
  scenarios that write `Region_1_5` hundreds of thousands of times moved
  nothing at all. Read forwards that is a nearly-invisible unit. The whole-unit
  no-op, which unit #4 already recommends as a habit, moves **1,526,538 across
  22 of 27 scenarios and 135 channels.**

  ```
  # the narrow red says the gate CAN see the unit; the no-op says HOW MUCH
  python3.12 scripts/gate.py <U> --perturb-file rosco/controller/src/<u>.cpp \
      --perturb-from 'void <U>(<the signature>) {' \
      --perturb-to   'void <U>(<the signature>) { if (<args>) return;' \
      --out evidence/<U>/gate.whole-unit-no-op-MOVES.json
  ```

  Run BOTH when the unit's output is an enumerated STATE. Which state you
  perturb to decides everything: two states whose downstream control law
  coincides on a scenario's operating point produce a green that is about the
  perturbation and not about the unit.

- **SEVEN OF THIRTEEN ASSIGNMENT SITES DEAD IN ALL 27 SCENARIOS, AND THE UNIT
  IS STILL THE MOST GATE-VISIBLE ONE SO FAR.** Unit #20. Both facts are true at
  once and they answer different questions -- coverage of the WRITE SITES says
  what the simulation exercises, the no-op perturbation says what it propagates.

  ```
  python3.12 -c "import json;d=json.load(open('coverage/line_coverage.json'));\
      h=d['hits']['<File>.f90'];print([(l, sum(h[str(l)].values()) if str(l) in h else 0) \
      for l in range(<lo>,<hi>)])"
  ```

  For `StateMachine`: the whole region-3 INITIALISATION sub-tree is dead
  because every scenario's first call takes the Region-2 leaf; `PC_ControlMode`
  is 1 in all 14 `Examples/*.IN`; `Region_2` is written in scenario 12 alone.
  A unit that enumerates MODES has this shape by construction, and the
  differential harness is the only layer that reaches the modes the campaign's
  inputs do not select.

- **A PREDICATE THE REFERENCE WRITES AGAINST A NAMED PARAMETER, OR BETWEEN TWO
  VARIED QUANTITIES, WAS UNREACHABLE BY EVERY CORPUS RULE.** Unit #20, and it
  is the twelfth corpus blind spot -- two of them, closed together in the loop
  repo (`21ed899`), 0.769 -> 1.000.

  Nine of 39 mutants survived a 2028-case green, and NOT ONE was a
  transcription defect. What said so was a branch-reachability probe, which is
  twenty lines and is worth building before reasoning about any survivor:

  ```
  # the shipped translation with a counter per leaf, run through the harness
  # evidence/StateMachine/statemachine.branch_probe.cpp is the pattern
  ```

  All nine sat inside two sub-trees that ZERO cases entered.

  1. `predicate_knobs_from` matched `NAME <op> NUMERIC LITERAL` only, so
     `IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch)` produced no knob at all --
     the value is an `INTEGER(IntKi), PARAMETER` in `Constants.f90`. **A
     reference that names its magic numbers is being CLEARER than one that
     inlines them, and it was the clearer spelling the corpus could not read.**
  2. A predicate whose two sides are BOTH varied has no crossing value in any
     ladder: every rule sets one parameter and leaves the rest at base, so
     **whichever base value is larger is larger in every case this generator
     has ever produced.** `PitCom(1) >= VS_Rgn3Pitch` was false in all 2028.
     The crossing point is the OTHER SIDE's value in that same case.

  And the half that was not obvious: **a two-sided predicate can be GATED by
  another one.** `GenArTq >= VS_MaxOMTq*1.01` sits inside the ELSE of
  `BlPitchCMeas >= VS_Rgn3Pitch`, true in 2451 of 2890 cases under the ordinary
  draw, so pinning the inner pair alone still left its leaf unwritten. Each
  pair now runs with every OTHER pair pinned just below its crossing point and
  again just above.

- **`restore_integrated.sh` TAKES `vit.yaml`'s HAND-ADDED ENTRY WITH IT, AND SO
  DOES THE `git checkout --` THAT REPAIRS THE FILE.** Unit #20, and it is the
  entry units #14/#16/#18 wrote, one loop further round: a unit that
  re-integrates after a second `reset_to_clean` runs `vit integrate` TWICE, so
  the comment-stripping and the hand re-add both happen twice. Re-check the
  entry after the LAST `git checkout -- vit.yaml`, not after the first:

  ```
  grep -c 'StateMachine' vit.yaml     # 0 means the checkout ate it
  ```

- **THE PRE-INTEGRATION HARNESS CANNOT BE RE-RUN ON AN INTEGRATED TREE, AND IT
  FAILS BY WRITING AN EMPTY FILE.** Unit #20. Re-taking a probe artifact after
  integration produced a zero-byte evidence file and no error anyone would
  read -- the rule the RUNBOOK already states ("both link the campaign's
  Fortran objects, and after integration `<File>.f90.o` IS the wrapper"), met
  as a silent empty artifact rather than as a link error. The fix is the
  documented one and it costs about four minutes: `reset_to_clean.sh` -> re-run
  -> `restore_integrated.sh` -> `vit integrate --apply` -> rebuild -> re-run
  the gate. Do that rather than hand-copying the numbers out of a transcript.

- **A STUB WHOSE WHOLE RETURN VALUE IS `0.0` CAN PASS THE KERNEL VERDICT 62/62,
  AND THE TELL IS IN THE ROWS RATHER THAN ANYWHERE IN THE RUN.** Unit #19, and
  it is unit #3's tolerance rule reaching the case that costs something.

  KGen scores an array field `IN_TOL` when `SQRT(SUM((var-ref)**2)/n)` is under
  `kgen_tolerance = 1.D-14` -- **ABSOLUTE**, and `n` is the DECLARED extent, 1024
  for every `FP` array. So a unit whose output has decayed small enough is
  compared against nothing: `SecLPFilter_Vel`'s reference output is
  `-2.97e-52` in the captured window, one element of 1024, giving an RMS of
  1e-53. A stub identical to the translation in every state write with the
  filter expression replaced by `0.0` moved **63 rows in 21 of 62 cases** and
  the kernel printed `Number of verification-passed cases : 62`.

  **A zero stub is not enough to establish that a kernel can see the RESULT.**
  The ordinary zero stub also zeroes the coefficients, which the reference
  writes in one case and leaves alone in the other 61 -- so it fails every case
  for a reason that has nothing to do with the returned value. Build the second
  stub, the one that perturbs ONLY the thing you are asking about:

  ```
  # identical in every state write; the filter expression -> 0.0
  # then count the ROWS, not the cases:
  grep -c "is IDENTICAL" evidence/<U>/<log>          # and
  grep -c "NOT IDENTICAL"  evidence/<U>/<log>        # (within tolerance) counts as PASSED
  ```

  `evidence/SecLPFilter_Vel/kernel_field_rows.py` prints the verdict line and
  the row statuses next to each other for every run log, which is the smallest
  thing that makes the disagreement impossible to miss. Copy it, not the
  SecLPFilter version -- that one has no tolerance column.

  The same tolerance softens a HARDCODED-ARGUMENT stub into a weaker claim than
  it looks: this unit's passes 62/62 and still moves 5 within-tolerance rows,
  because `2*PI/20` written by hand differs in the last bits from
  `2*PI/CntrPar%CC_ActTau`. "The kernel cannot constrain this argument" and
  "the kernel would not have seen a last-bit error in it either" are two claims
  and only the row table contains the second.

- **A HAND-RUN `make` IN A KERNEL DIRECTORY REPORTS THE PREVIOUS
  TRANSLATION'S VERDICT, AND A ZERO STUB "PASSED 62/62" ON IT.** Unit #18, and
  it is a defect in a recipe TWO entries below this one already teach.

  VIT writes the TRANSLATION to `<stem>.hpp` and a three-line `extern "C"`
  wrapper to `<stem>.cpp`. The generated rule named only the .cpp, so `make`
  reused an object built from the real translation and never compiled the stub:

  ```make
  seclpfilter.o: seclpfilter.cpp        # the translation is in the .hpp
  ```

  `vit verify` is NOT affected -- `_build_and_run_kernel` runs `make clean`
  first, so every kernel green in this campaign came through a full rebuild.
  What is affected is every recipe here that hand-runs `make`: the stub re-run,
  and unit #17's original-Fortran replay. Both are used exactly when the
  question is whether the comparison can fail at all.

  Fixed in VIT `5ba5e7e` (X2), and the tell was nearly missed -- the "passing"
  stub printed `Number of output variables: 2`, identical to the real run.
  Until a kernel is regenerated with that VIT, force it:

  ```
  cd kernel/<Unit> && rm -f <stem>.o kernel.exe && make -s build && ./kernel.exe
  ```

- **A KERNEL RUN'S OWN STDOUT SURVIVES THE CYCLE AND `verify_fields.csv` DOES
  NOT.** Unit #18, and it is the entry below about `kernel/` being untracked,
  one step further on: knowing to copy something out is no help once
  `reset_to_clean.sh` has already run. The harness and mutation steps of a
  normal cycle BOTH need a clean tree, so `kernel/` is gone by the time the
  evidence directory is written.

  The kernel prints `<field> is IDENTICAL.` per case per field -- the same
  verdicts the CSV carries -- so redirect every run to a file AT THE TIME and
  cite that. `evidence/SecLPFilter/kernel_field_rows.py` parses the three logs
  back into the per-field table.

  ```
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/kernel/<U> && ./kernel.exe" \
      > evidence/<U>/kernel.<what-this-run-was>.run.txt 2>&1
  ```

  What the CSV has and the stdout does not is the VALUES, so a non-vacuity claim
  of the form "62 distinct reference values" cannot be made from the log. Make
  it from the ZERO STUB instead, which is committed and is stronger: the count
  of rows it moves is the unit's footprint, and the count of CASES each row
  moves in tells you how many reach the branch that writes it. Here the six
  coefficient arrays move in 1 of 62 -- so 61 cases never read `CornerFreq` or
  `Damp`, which is unit #11's "not read" rather than "has one value", derived
  from an artifact that survives.

- **TWO ISOLATING VALUES ARE NOT ENOUGH, AND WHICH ONE WORKS IS DECIDED BY WHAT
  THE EXPRESSION DOES TO THE RUNG AFTERWARDS.** Unit #18, and it is unit #13's
  joint magnitude block meeting the case it was not shaped for.

  `0.0` and `1e300` isolate by REMOVING the other parameter -- to nothing, or to
  Inf by overflowing its square. Right for a SUM (`NotchFilter`), useless for a
  PRODUCT: `2.0*DT**2.0*CornerFreq**2.0` is 0 under both spellings at
  `CornerFreq = 0.0` and Inf under both at `1e300`. **A product has no term to
  remove.** And the plain ladder cannot reach it either -- it carries `1e-155`,
  which unit #13 already measured as NOT distinguishing the regrouping, while
  the rungs that do (`1e-156`, `1e-158`, `1e-160`) live only in the pinned
  block.

  Then the same mutant one subtraction later needs a THIRD value:
  `2.0*DT**2.0*CornerFreq**2.0 - 8.0` is exactly `-8.0` under both spellings at
  any ordinary `CornerFreq`. `sqrt(DBL_MAX)` -- the largest x whose `x*x` is
  still FINITE -- is the only isolating value that amplifies rather than
  annihilates. Of **1,936** ladder-by-ladder pairs, exactly **SIX** kill it and
  all six are `(hot rung, +/-sqrt(DBL_MAX))`. 0.975 -> 0.988 -> 1.000, 0
  declared equivalent.

  **Search the corpus's own values against each other before believing a
  survivor is out of reach.** The witness was already in the ladder; what was
  missing was the pairing, and the search is twenty lines:

  ```
  # evidence/SecLPFilter/assoc_reorder_ladder_pair_search.py
  # every (ladder value, ladder value) pair through both spellings, bits compared
  ```

  And ask the question the other way round: not "is the rung present" but
  **what does the rest of the expression do to the rung's difference before it
  reaches an output.** Powers of two never distinguish a regrouping at all --
  `x*x` is exact there -- so an exact-mantissa probe grid answers nothing, which
  cost one wrong "0 witnesses" reading here.

- **THE OTHER GENERATOR CROSSES IT TOO, AND THE MATRIX'S CELL WAS A REAL
  DEFECT IN A REAL GENERATOR.** Unit #17, second dispatch, and it is the half
  of the refutation the first dispatch could not reach.

  `test_validate.generate_fortran_bridge` -- the DIFFERENTIAL HARNESS's Fortran
  side, which is what the matrix's `bridge`/`compiles` columns measure -- really
  did emit

  ```fortran
  REAL(C_DOUBLE), INTENT(OUT) :: Channels(*)
  CALL Read_OL_Input(..., Channels(1:n_Channels), ...)   ! rank-1 section ->
  ```                                                    ! rank-2 ALLOCATABLE

  and really did not compile. Fixed in VIT rather than worked around (X2): the
  bridge now mirrors `interface_gen`, and it names its extents with
  `_alloc_return_extent_names` -- **the same helper** -- so the two spellings of
  one ABI cannot drift. A second disagreement rode with it: `build_c_params`
  emits `int len_<x>` only for `CHARACTER(*)`, and the bridge was emitting one
  for `CHARACTER(1024)` too. **One argument more than the prototype, on a C
  linkage no linker checks.**

  Ask BOTH generators before believing a `bridge_feasible`, and ask them with
  the same command:

  ```
  python3 -c "from vit.interface_gen import build_c_params; ..."     # the ABI
  vit test-validate <Unit> <cpp> -f <f90> -m <Module> --force        # the bridge
  diff <(the first) <(grep 'SUBROUTINE <unit>_f90' -A 20 the second)
  ```

- **THE CORPUS CAN WRITE THE FILE, AND THE FIRST DISPATCH WAS WRONG THAT IT
  COULD NOT.** Unit #17, second dispatch. R8_file_contents, and it is the first
  rule here that varies an input the signature only NAMES.

  `file_params_from` reads the `FILE=` specifier out of the REFERENCE and the
  corpus writes 82 fixtures -- absent, blank, 1/2/3 records, no trailing
  newline, CRLF, four separator forms, the null value, the `/` terminator,
  `r*value` repeats, the e/d and letter-less exponents, -0.0, the representable
  extremes, short and long records, malformed first and second records, a record
  past the 1024-byte `A` edit, and comment lines led by **each character literal
  the reference itself names**. Crossed with the io-list trip count read out of
  the reference's own `READ (X(I), I=1,N)`, because two ladders that never cross
  cannot reach a branch that needs both (unit #16).

  **And the file parameter is HELD everywhere else**, which is the half that
  costs nothing and buys the most. A file NAME is dereferenced, not read, so
  R6's character ladder produces several hundred names for one answer -- and one
  of them was 1024 `/` characters, which resolve to the ROOT DIRECTORY.

  ```
  grep -n "FILE *=" <the unit's Fortran>     # is a DUMMY the specifier?
  ```

- **THE REFERENCE DOES NOT TERMINATE ON THREE INPUTS, AND A DIFFERENTIAL
  HARNESS CAN ONLY COMPARE INPUTS BOTH SIDES RETURN FROM.** Unit #17, second
  dispatch, and it is a fourth upstream ROSCO defect of the same kind as the two
  already in this file.

  ```fortran
  DO WHILE ( INDEX(LINE,'!') > 0 .OR. INDEX(LINE,'#') > 0 .OR. INDEX(LINE,'%') > 0 )
      NumComments = NumComments + 1
      READ(Unit_OL_Input,'( A )',IOSTAT=IOS) LINE
      ! NWTC_IO has some error catching here that we'll skip for now
  ```

  A failed READ leaves `LINE` UNCHANGED under gfortran. So an EMPTY file, a
  COMMENTS-ONLY file and a name resolving to a DIRECTORY all loop for ever --
  measured, three exit-124s at an 8-second timeout
  (`evidence/Read_OL_Input/reference.does-not-terminate-empty-allcomment-directory.txt`).
  The translation stops instead, so the two DISAGREE there and no case can be
  written for it. `file_shapes()` states the exclusion in the rule's own
  detail line rather than omitting the shapes in silence.

  **The symptom is a 100%-CPU `./test` with no output**, and the generated
  program prints only at the end. Two lines find it:

  ```
  # a CASE print at the top of the loop, and one between the two calls
  sed -i 's/        r.marker(c);/&\n        fprintf(stderr,"CASE %d\\n",c);fflush(stderr);/' <unit>_test.cpp
  ```

- **A CASE THAT DEPENDS ON ITS PREDECESSORS IS NOT A DIFFERENTIAL CASE, AND AN
  OPEN FORTRAN UNIT IS HOW.** Unit #17, second dispatch.

  `Read_OL_Input` CLOSEs its unit on both error branches and not on the success
  branch. Two cases sharing a fixture then fail two different ways, and both
  were seen here:

  ```
  the same UNIT again   positioned at end of file -> every READ fails -> the
                        comment loop above never exits
  the same FILE again   Fortran forbids connecting one file to two units, so the
                        OPEN fails and the reference answers "Cannot open" where
                        the C++ reads the file   (68 of 739 cases)
  ```

  R8 gives every file case its own file AND its own unit number
  (`io_unit_params_from`). It is the first time this campaign has had to choose
  an input VALUE to make a unit a function of its arguments.

- **THE DIFFERENTIAL HARNESS SAW A DEFECT THE KERNEL AND THE GATE COULD NOT,
  AND IT WAS AN ALLOCATION EXTENT.** Unit #17, second dispatch.

  `ALLOCATE(Channels(NumDataLines, NumChannels))` with `NumChannels < 0` is an
  extent of **ZERO** in Fortran -- a bound `1:n` with `n < 0` -- not a negative
  extent. The translation returned the raw argument, which would size a
  `C_F_POINTER` shape by a negative number. **80 of 657 cases.** The kernel sees
  1 case and the gate's three scenarios all take `.NOT. FileExists`, so neither
  reaches a path on which this unit allocates at all.

  ```
  # every ALLOCATE whose bound is an argument, and whether anything bounds it
  grep -n 'ALLOCATE(' <the unit's Fortran>
  ```

- **31 OF 47 SURVIVORS WERE ONE REDUNDANT SCANNER, AND DELETING IT WAS THE
  FIX.** Unit #17, second dispatch, and it is unit #15's `INDEX` finding in a
  second shape.

  `scan_real` walked a list-directed datum by hand -- sign, digits, point,
  exponent letter, exponent sign, exponent digits -- and then handed **the same
  text to `strtod` anyway**. Two implementations of one grammar, of which only
  the second produces a value anything reads, so every `rec[i]` -> `rec[i + 1]`
  and every `i < n` -> `i <= n` inside the walk moved a position `strtod`
  recomputes four lines later. Replacing the walk with "the datum ends at a
  separator, and `strtod` must consume all of it" took **0.708 -> 0.726 and 47
  survivors -> 37**, and the same treatment on `scan_repeat` (`strtoul`) removed
  four more.

  The check is one question asked of every hand-written scanner:

  ```
  # does anything READ the position this loop computes, or is it recomputed?
  ```

- **A MUTANT CAN FAIL TO TERMINATE, AND WITHOUT A WATCHDOG THE WHOLE SCORING
  RUN STOPS WITH NOTHING WRITTEN.** Unit #17, second dispatch.
  `vit_mutate.py` had no timeout on `./test`, and this unit is a parser with
  four `while` loops. The watchdog is **self-calibrating** -- 20x the baseline
  run, floor 60s -- so it is derived from the unit rather than picked, and a
  hang is counted as a kill and **reported separately** (`killed_by_timeout`, 4
  here): it was killed by a watchdog, not by a value the corpus supplied. Same
  rule the file already had for `killed (no compile)`.

- **AN EVIDENCE REFERENCE INTO `kernel/` DOES NOT SURVIVE THE RESET/RESTORE
  CYCLE.** Unit #17, second dispatch. `kernel/` is UNTRACKED and
  `reset_to_clean.sh` removes it, so `P5 evidence resolves` passed only while the
  directory happened to be on disk -- and E4.2 asks for a COMMITTED artifact.
  Copy what you need into `evidence/<Unit>/` and cite that.

  ```
  git ls-files kernel | head -1     # empty: nothing under kernel/ is committed
  ```

- **`bridge_feasible: no` MEASURED THE WRONG GENERATOR, AND THE SIGNATURE
  CROSSED.** Unit #17, and it is the first prediction this campaign has
  REFUTED rather than confirmed.

  The plan said `channels: c_assumed_shape_2d does not cross`. `vit interface`
  crosses it, in a shape nothing in the plan names: **ALLOCATE-ON-RETURN.**
  `REAL(DbKi), INTENT(OUT), DIMENSION(:,:), ALLOCATABLE :: Channels` becomes
  three C parameters --

  ```
  double** Channels, int* n_Channels_rows, int* n_Channels_cols
  ```

  -- the C++ mallocs and hands back a pointer, and the wrapper does
  `C_F_POINTER(...,[rows,cols])`, `ALLOCATE`, copy, `vit_free`. `C_NULL_PTR`
  means NOT ALLOCATED, so P6 survives. It builds, links and passes the gate.

  The prediction is not merely wrong, it is wrong about **which generator**, and
  the RUNBOOK already said so two hundred lines down: the matrix's
  `bridge`/`compiles` columns measure `test_validate.generate_fortran_bridge` --
  the DIFFERENTIAL HARNESS's Fortran side -- and `vit integrate` ships
  `interface_gen`. Both answers were true of the thing each measured. Ask the
  shipping generator first; it costs one command:

  ```
  vit interface <Unit> -f <file> -o /tmp/<u>_iface && cat /tmp/<u>_iface/*_wrapper.f90
  ```

- **A DEFERRED-LENGTH `CHARACTER(:), ALLOCATABLE` FIELD THAT ARRIVES
  UNALLOCATED HAD NO REPRESENTATION ON THE C SIDE, AND THE KERNEL SCORED IT
  200 OF 200 IDENTICAL.** Unit #17. C12: the wrong artifact is committed
  (`evidence/Read_OL_Input/kernel.errmsg-never-written-still-PASSES.verify_fields.csv`)
  and the fix came after it.

  `vit_populate_errorvariables` published `C_NULL_PTR / n = 0 / cap = 0` for an
  unallocated field. That carries P6 correctly and leaves the C++ **with no
  buffer at all** -- so a reference whose own statement is
  `ErrVar%ErrMsg = <expr>`, a REALLOCATING assignment that allocates, cannot be
  transcribed. `Read_OL_Input`'s only reachable path is exactly that statement.

  The translation wrote nothing, printed its refusal to stderr 6 times, and the
  kernel **passed**: KGen guards the comparison on `IF (ALLOCATED(var%errmsg))`
  -- the KERNEL's own value -- so **an output the translation fails to allocate
  DELETES ITS OWN COMPARISON.** Read the stderr of a kernel run, not only its
  verdict, and count the fields:

  ```
  ./kernel.exe 2>&1 | grep -c "^VIT:"          # a refusal the verdict cannot see
  python3.12 -c "import csv; r=list(csv.DictReader(open('kernel/<U>/verify_fields.csv'))); \
      print([x['field'] for x in r if x['field'] in ('<the outputs>',)])"
  ```

  Fixed by ADDITION in `vit/view_populator.py`: the staging buffer is supplied
  either way and the not-allocated signal moves onto the LENGTH, using the
  convention this codebase already had for an ALLOCATABLE array extent --
  **`n < 0` unallocated, `n == 0` allocated-empty, `n > 0` allocated**.
  `vit_copy_scalars_to_*` already guarded `n >= 0`, so a field the C++ does not
  touch stays unallocated exactly as before, and no already-measured unit moves
  (gate re-run: 5,252,000 / 0). With the fix the field appears and the case goes
  `OUT_TOL` -- which is the next entry.

- **KGEN DOES NOT ROUND-TRIP A `CHARACTER(:), ALLOCATABLE` FIELD: THE REFERENCE
  COMES BACK EMPTY.** Unit #17, and it is why the fix above turned a false
  green into a red that is also not about the translation.

  The generated reader is `READ (UNIT = kgen_unit) var%errmsg` into a field it
  has just DEALLOCATED -- no length record anywhere. So `errmsg`'s reference
  column is `''` while the computed column is the exact string
  `ROSCO_Helpers.f90` constructs. Measured, not read: the **ORIGINAL FORTRAN**,
  rebuilt into this same kernel, fails the same case with the same message
  (`evidence/Read_OL_Input/kernel.original-fortran-replay-FAILS-errmsg-empty-reference.txt`).

  ```
  cd kernel/<U> && cp ROSCO_Helpers.f90 /tmp/bridge.f90 && cp ROSCO_Helpers.f90.kgen ROSCO_Helpers.f90 \
      && make -s && ./kernel.exe 2>&1 | grep VIT_FIELD ; cp /tmp/bridge.f90 ROSCO_Helpers.f90 && make -s
  ```

  NOT FIXED HERE. Writing a length record changes the STATE FILE FORMAT for
  every type carrying such a field -- `ErrorVariables` is one, and
  `ReadAvrSWAP`'s and `ExtController`'s committed kernels both use it -- so it
  re-takes their artifacts. X3 and SPEC 8.4, the Driver's call; recorded in
  DECISIONS.md as a candidate.

- **A UNIT'S PRINCIPAL INPUT CAN BE ABSENT FROM ITS SIGNATURE, AND THEN NO
  GENERATOR IN THIS CAMPAIGN CAN REACH IT.** Unit #17, and it is the eleventh
  corpus blind spot -- the one no widening of any ladder can close.

  `Read_OL_Input`'s behaviour is a function of **a file on disk**. The signature
  carries the file's NAME. `harness/generate.py` varies numbers, strings and
  array shapes; it has no notion of a file-valued input, so a corpus could vary
  the name for ever and never vary the bytes the unit reads. That is a third
  blocker underneath two ordinary ones, and it is the one that decides the
  disposition.

  The two ordinary ones, both measured:

  ```
  bash scripts/harness.sh Read_OL_Input ROSCO_Helpers read_ol_input <file>.f90 --against translation
  # -> EmitError: C parameter 'OL_InputFileName' is not in the mapped signature
  ```

  1. `CHARACTER(1024), INTENT(IN)` is a width **compiled into both sides**, so
     `build_c_params` emits `char*` with no `len_` and `map_signature` refuses
     what it cannot size -- unit #10's CHARACTER-function-result shape, one
     declaration over.
  2. `Channels` maps as an INPUT `real[]` varied on the +/-1e3 default, when the
     shipping bridge makes it ALLOCATE-ON-RETURN. The report says so in one
     line: `UNCONSTRAINED: 1 varied parameter(s) ... Channels`.

- **THE GATE CAN SEE A UNIT BY KILLING EXACTLY THE SCENARIOS THAT CALL IT.**
  Unit #17, and it is unit #15's `perturbation_broke_scenarios` shape reached
  from the opposite end -- there 24 of 27 died and it was noise; here **3 of 27
  died and they are the entire footprint.**

  The clean call site `ReadSetParameters.f90:778` runs **exactly three times in
  all 27 scenarios** -- once each in 10, 14 and 24, which coverage already
  records as executing NO controller code -- and all three take
  `.NOT. FileExists`, because `Examples/example_inputs/OL_Mode2_Input.dat` is
  not in this tree. Every line from the `OPEN` to the final read loop is dead in
  the whole campaign. The whole-unit no-op stopped precisely [10, 14, 24]
  (compared 5,252,000 -> 4,992,000) because without `aviFAIL = -1` the caller
  does not `RETURN` and reads an unallocated `CntrPar%OL_Channels`.

  So `went_red` reads false and the unit is not invisible. Read the field:

  ```
  python3.12 -c "import json; d=json.load(open('gate/<U>.redtest.json')); \
      print(d.get('perturbation_broke_scenarios'), d['compared'], d['mismatched'])"
  ```

  And the first perturbation attempt is worth keeping as the lesson: forcing
  `FileExists = false` moved 0, because the branch it forced is the branch every
  scenario **already takes**. Perturb toward the behaviour the scenarios do NOT
  produce.

- **A LADDER AIMED AT A NAME IS NOT A LADDER OVER THE QUANTITY -- ASK WHERE THE
  TESTED VALUE ENTERS, NOT WHAT IT IS CALLED.** Unit #16, and it is the tenth
  corpus blind spot: the first where the corpus DID contain the value and the
  branch was still unreachable.

  `ReadAvrSWAP` branches four times on `LocalVar%iStatus == 0`.
  `LocalVar_iStatus` IS a scalar integer parameter, so the integer decade ladder
  (unit #10) gave it 0 -- and every one of those cases was DEAD, because the
  unit's first statement is `LocalVar%iStatus = NINT(avrSWAP(1))`. The value that
  decides the branch arrives as ONE ELEMENT of an array, and `_fill_array` gives
  an array a single ascending ramp, so `avrSWAP(1)` is about **-999 in every case
  this generator has ever produced**. Same for `NINT(avrSWAP(61))` -> `NumBl`,
  the trip count of the unit's only loop, and `avrSWAP(2)` -> `Time`.

  The second half is separate and cost as much: **two ladders that never cross
  cannot reach a branch that needs both.** Every stage sets ONE parameter and
  leaves the rest at base; `flag_variants` crosses only DECLARED flags, and a
  scalar integer compared against a literal is not one. The loop body needs
  `PF_Mode == 1` AND `NumBl >= 2` AND `iStatus /= 0` **in the same case**. Each
  was individually reachable; the conjunction was not.

  Together: **14 of 111 mutants survived a 26,198-case green, 0.874**, and all
  fourteen sit in the two blocks those conjunctions guard. The check is two
  greps against the reference and it costs a minute:

  ```
  # 1. is the tested NAME reassigned before the branch reads it?
  grep -n 'iStatus\|NumBl' <the unit's Fortran> | grep '='
  # 2. how many predicates must hold AT ONCE for the survivor's line to run?
  ```

  Closed by addition: `predicate_knobs_from` (in `vit_harness.py`) reads the
  reference's own predicates, follows `LHS = ... arr(lit) ...` back to the
  element the quantity ENTERS by, and `generate.py`'s R7 block emits the CROSS
  PRODUCT of them -- 729 combinations of six quantities here, bounded at 4096
  with an all-pairs fallback that reports itself. 26,198 -> 27,656 cases.

- **AN INPUT SURFACE NO SCENARIO DRIVES IS INVISIBLE TO EVERY BIT-COMPARING
  LAYER AT ONCE, AND `avrSWAP` HAS ONE.** Unit #16, and it is the largest
  single blind spot this campaign has measured: **23 of `ReadAvrSWAP`'s 43
  output fields**, in one unit.

  A stub with those 23 assignments DELETED passes the kernel **62 of 62
  IDENTICAL, 14,135 field rows**
  (`evidence/ReadAvrSWAP/kernel.zero-fed-outputs-deleted-stub-PASSES.verify_fields.csv`),
  and disabling the same block at the gate moves **0 of 5,252,000** on a build
  whose `GenSpeed` perturbation moves 1,487,557. The differential harness fails
  it **26,198 of 26,198** and names the fields, so the blindness is the
  simulation's and not the method's.

  Three separate mechanisms, and asking which one applies is the whole check:

  ```
  # 1. NO DATA. Who writes the index the unit READS?
  grep -n 'avrSWAP\[' rosco/toolbox/control_interface.py Examples/vit_sim.py
  # 2. NO READER THE GATE READS -- unit #7's question, asked of each output
  grep -n '<the output field>' rosco/controller/src/*.f90 | grep -v vit_
  # 3. and the value itself, straight out of the green field log
  python3.12 -c "import csv;r=list(csv.DictReader(open('kernel/<U>/verify_fields.csv')));\
      print({x['computed'] for x in r if x['field']=='<field>'})"
  ```

  Here: `avrSWAP(1001..1018)` -- the extended Bladed interface that
  `Ext_Interface = 1` turns on in all 14 inputs and coverage shows running
  443,972 times -- is **read in exactly one place in the whole tree and written
  in none**. `avr_size` is 3000 zeros. Those 18 fields also have no consumer in
  the controller at all: every other occurrence is a `ROSCO_IO` debug `WRITE`,
  a restart-file `READ`, or the `LocalVarOutData` table, and `gate.py` never
  opens a `.RO.dbg`. Double blindness -- no data AND no reader -- so no widening
  of the scenario set alone would fix it.

- **A SCENARIO CAN INJECT A SIGNAL AND THE DRIVER CAN OVERWRITE IT BEFORE THE
  DLL SEES IT.** Unit #16, and it is the second mechanism above -- new, cheap
  to check, and it makes a scenario's stated purpose false.

  `Examples/vit_sim.py`'s scenario 27 sets `controller_int.avrSWAP[52]` (the
  tower-top acceleration, for `TD_Mode = 1`) and `[82]` (the nacelle IMU, for
  `Fl_Mode = 2`) and then calls `call_controller`, which opens with a block of
  its own `avrSWAP[...] =` assignments -- including

  ```python
  try:    self.avrSWAP[82] = turbine_state["NacIMU_FA_RAcc"]
  except KeyError: self.avrSWAP[82] = 0
  ```

  and `turbine_state` carries neither key. Both injections are **overwritten
  with 0**, and the green field log confirms it: `fa_acc_tt`, `ss_acc_tt`,
  `nacimu_fa_racc` and `fa_acc_nac` have exactly **one distinct value, 0.0, in
  all 62 cases**. The scenarios that inject `[23]`, `[29..31]`, `[36]` and
  `[59]` are unaffected -- `call_controller` either does not touch those indices
  or writes the same signal through `turbine_state`.

  ```
  # the injections, then the driver's own writes; any index in BOTH is dead
  grep -n 'controller_int.avrSWAP\[' Examples/vit_sim.py
  grep -n 'self.avrSWAP\[' rosco/toolbox/control_interface.py
  ```

  **NOT FIXED HERE.** Repairing it changes what the 27 scenarios feed the
  controller, which moves the baselines and the compared count -- X3 and SPEC
  §8.4, the Driver's call. Recorded in DECISIONS.md as a candidate.

- **`vit verify` PRINTS ITS OWN OVERRIDE, NOT THE KERNEL'S COUNT -- and the
  green field log it commits still carries the red row.** Unit #16.

  The kernel's own summary reads `Total number of verification cases : 62 /
  Number of verification-passed cases : 61 / kernel: ReadAvrSWAP: FAILED
  verification`
  (`evidence/ReadAvrSWAP/vit_defects/kernel.run-says-FAILED-61-of-62.txt`).
  `vit verify` printed `✓ VERIFICATION PASSED: 62/62 passed`.

  The override is CORRECT and is the design: `run_kernel_verify` runs the
  ORIGINAL FORTRAN through the same kernel first, and when the two field logs
  match it declares the kernel's failures state-capture artifacts. Measured
  rather than read -- the original Fortran, rebuilt into this kernel, produces
  the identical wrong answer on the identical case
  (`evidence/ReadAvrSWAP/kernel.original-fortran-replay-FAILS-case2.txt`). What
  is missing is that `run_kernel_verify` COMPUTES a `Note: N state file
  artifacts detected` line and a field-coverage line into `result.output`, and
  `cmd_verify` never prints `result.output` at all.

  So read the artifact, not the verdict, before recording a kernel green:

  ```
  python3.12 -c "import csv,collections;r=list(csv.DictReader(open('kernel/<U>/verify_fields.csv')));\
      print(collections.Counter(x['status'] for x in r));\
      [print(x) for x in r if x['status']!='IDENTICAL']"
  ```

  The cause here is this campaign's OWN `vit.yaml`. `kgen.dll_persistence`
  inserts `LocalVar%AlreadyInitialized = 0` before every line matching
  `CALL ReadAvrSWAP` in the INSTRUMENTED source -- which puts it between KGen's
  input capture and the call -- and the generated kernel does not carry it. So
  the reference output was produced by a statement the replay does not execute,
  on exactly the one case where it is observable (case 1's input is already 0;
  from case 3 on the previous call left it 0). A `dll_persistence` reset that
  touches a variable the unit itself READS is this shape.

- **A GATE RED TEST CAN BE SEEN BY KILLING THE RUN, AND `went_red` COUNTS ONLY
  VALUES.** Unit #15, and it is the first `gate/*.json` in this campaign with a
  non-empty `scenarios_failed`.

  Forcing `PathIsRelative` to answer `.TRUE.` prefixes `PriPath` onto an
  already-absolute `PerfFileName`, and the scenarios do not finish wrong -- they
  ABORT, in the Fortran runtime, before writing anything. 24 of 27 died,
  `compared` fell 5,252,000 -> 260,000, `mismatched` stayed 0, and
  `went_red` (`mismatched > 0`) read **false** under the message *"either the
  line is never executed by these scenarios, or the gate cannot observe it"* --
  which is false of that run. The most visible failure available, filed as
  invisibility.

  `verdict_of` already treats `scenarios_failed` as broken for an ORDINARY gate
  run; only the red-test branch ignored it. `gate.py` now also writes
  `perturbation_broke_scenarios` and prints an accurate line; `went_red` and the
  exit code are UNCHANGED on purpose (X3 -- fourteen committed artifacts mean
  what they say). Read the field, and read it before writing an observability
  note:

  ```
  python3.12 -c "import json,glob; [print(f, json.load(open(f)).get('scenarios_failed')) \
      for f in sorted(glob.glob('gate/*redtest*.json'))]"
  ```

  A perturbation that can make the controller REJECT or FAIL TO READ its own
  input file is this shape -- anything touching a file name, a unit number, or a
  validity check. Diagnose it rather than reporting it: one scenario, run by
  hand, prints the runtime error
  (`evidence/PathIsRelative/gate.always-true-scenario1-runtime-error.log`), and
  that is the difference between "24 scenarios failed" and a cause.

- **ASK WHICH DIRECTION OF WRONGNESS THE GATE COULD SEE -- a unit can answer
  both ways in the campaign and still have neither answer observable.** Unit
  #15, and it is unit #6's `GetPath` finding from the other side.

  Coverage of the clean source is what says it, in one line: the function is
  entered 56 times and `PathIsRelative = .TRUE.` has 25 hits, so the two call
  sites one line apart take DIFFERENT branches. And each is invisible for its
  own reason -- `PerfFileName` is absolute everywhere, so a wrong `.FALSE.`
  changes nothing; `OL_Filename` is `"unused"` and the name the branch builds is
  read only when `OL_Mode > 0`, where the path is absolute and the answer is
  `.FALSE.` again. Two red tests, opposite directions, 0 values moved by either.

  ```
  # the branch BODIES, not the branch -- unit #7's recipe, and it costs seconds
  python3.12 -c "import json;d=json.load(open('coverage/line_coverage.json'));\
      h=d['hits']['<File>.f90'];print([(l, sum(h[str(l)].values()) if str(l) in h else 0) \
      for l in range(<lo>,<hi>)])"
  ```

  **Beware the line numbers.** Coverage is indexed against the CLEAN source, and
  an integrated tree has moved every line below each wrapper -- here by two. A
  first reading of this unit's coverage landed one statement off and said the
  `.TRUE.` branch was dead, which would have been the wrong observability note
  entirely. Re-`grep` the function in the file the coverage was generated from.

- **A POSITION NOTHING READS IS AN UNOBSERVABLE SITE, AND `INDEX` IS FULL OF
  THEM.** Unit #15, and it is the first score gap since unit #4 that was the
  TRANSCRIPTION rather than the corpus.

  `PathIsRelative` calls `INDEX` three times and compares all three against `0`.
  Written literally -- a function returning the POSITION -- it scores **0.938**,
  and both survivors are its loop bound `len_s - len_sub + 1`
  (`-` -> `+`, and `+ 1` -> `+ 2`). Neither is a wrong answer: both read PAST
  THE END of the buffer, so they are undefined behaviour that no value
  comparison can see and that cannot honestly be declared equivalent. The
  position is also a quantity nothing downstream reads, so every site computing
  WHICH position matched is unobservable by construction.

  Rewritten as predicates -- `contains_pair`, indexed on the position of the
  SECOND character so the bound is a bare `i <= len_s`, and `contains_char` --
  the SAME 387 cases kill **26 of 26**.

  Two checks, and the second is what tells corpus from code:

  ```
  # 1. does the reference ever READ the value, or only test it against 0?
  grep -n 'INDEX(' <the unit's Fortran>
  # 2. do the two runs report the SAME case count? then the corpus did not move
  python3.12 -c "import json; [print(f, json.load(open(f))['checked']) for f in \
      ('harness/<Unit>.json','evidence/<Unit>/harness.<first form>.json')]"
  ```

  Keep the first form's mutation artifact. `mutation/PathIsRelative.survivors_index_position.json`
  is the measurement that makes the departure a finding rather than a preference.

- **A CORPUS CANNOT CONTAIN A VALUE ITS OWN DEDUP COLLAPSES, and `-0.0` is the
  first one.** Unit #14, and it is the ninth corpus blind spot -- the one where
  the value was not missing from the ladder, it was unrepresentable in it.

  `NotchFilterSlopes` opens with `IF (CornerFreq < 0) THEN CornerFreq_ = 0`, and
  `compare_op '<' -> '<='` was the only mutant alive at 0.988. The two
  predicates disagree on exactly one input, `CornerFreq == 0`, and at `+0.0`
  they are bit-identical -- so no magnitude rung, at any exponent, can separate
  them. `harness/generate.py`'s `_real_magnitude_ladder` and `_ladder` both end
  in `list(dict.fromkeys(out))`, and `0.0 == -0.0`, so a `-0.0` written into
  either list disappears into the `0.0` already there. Every earlier rung could
  be added where it belonged; this one had to be a block appended after the
  dedup. 4468 -> 4508 cases, **0.988 -> 1.000**, and the mutant dies on **2 of
  4508** (`evidence/NotchFilterSlopes/negative_zero_block_kills_the_saturation_mutant.txt`).

  Two checks, and the first is five seconds:

  ```
  python3 -c "from harness.generate import _real_magnitude_ladder as L; \
      import math; print(any(math.copysign(1,v)<0 and v==0 for v in L()))"
  grep -n 'CornerFreq < 0\|<= *0\|>= *0' <the unit's Fortran>   # a zero-guard?
  ```

  A unit with a comparison against zero on a value that then flows into a
  PRODUCT is the shape: the sign survives multiplication and is visible in the
  bits of the result.

- **A MUTANT'S DIFFERENCE CAN CANCEL IN THE RETURN VALUE AND SURVIVE ONLY IN AN
  OUT-PARAMETER.** Same unit, and it is the half worth more than the fix.

  At `CornerFreq = -0.0` the reference carries the sign into `2.0*DT*CornerFreq_`
  and the mutant substitutes `+0.0`, so `FP%nfs_b2` differs -- 36 of 36 draws.
  In **all 36 the returned value is bit-identical**, because `(+0.0) + (-0.0)`
  is `+0.0` under round-to-nearest and the two signed zeros meet inside the
  filter expression. A differential harness comparing only the unit's result
  could not have killed this mutant at any input, at any magnitude, ever.

  `R4_compare_all_outputs` has until now read like thoroughness. Here it is the
  whole of the discrimination, and the check before trusting any survivor
  analysis is to ask WHICH output the witness moves:

  ```
  # the probe pattern: evidence/NotchFilterSlopes/negative_zero_survivor_probe.cpp
  # compare EVERY field the unit writes, not the result -- and print which one
  # differed FIRST, because "the result agrees" is not "the mutant is equivalent"
  ```

- **TRANSCRIBE THE BRANCH, NOT `fmax` -- the shape rule buys an OBSERVABLE
  defect, not merely a matching one.** Unit #14. `IF (CornerFreq < 0) THEN
  CornerFreq_ = 0 ELSE CornerFreq_ = CornerFreq` is not `std::fmax(CornerFreq,
  0.0)`: `fmax(-0.0, 0.0)` returns `+0.0`, which is exactly the mutant's answer,
  and `fmax(NaN, 0.0)` returns `0.0` where the Fortran takes the ELSE. Written
  as `fmax` the translation would have been indistinguishable from the mutant --
  the mutation score would have read 1.000 for the wrong reason, and there would
  have been nothing to find.

- **THE GATE RED TEST'S PER-SCENARIO CHANNEL LIST SAYS WHICH SCENARIOS DRIVE THE
  UNIT WITH REAL DATA.** Unit #14, and it is a cheaper route to unit #2's
  finding than decoding a kernel window.

  `gate/<Unit>.redtest.json`'s `mismatched_channels` is `scenario_N:channel
  k/total`. `NotchFilterSlopes` runs in six scenarios and the perturbation moved
  channels in three -- 8, 26 and 27. One grep says why:

  ```
  grep -n 'rootMOOP\|<the unit's input signal>' Examples/vit_sim.py   # who INJECTS it
  ```

  Those three are exactly the scenarios `vit_sim.py` injects a synthetic
  per-blade `rootMOOP` into. The other three execute the call site 108,000 times
  between them on a zero input, where the output is zero and scaling it is the
  identity. Read the scenario numbers out of the red test before writing an
  observability note; they partition the hit count into the part that can carry
  a defect and the part that cannot.

- **A LADDER THAT VARIES ONE PARAMETER AT A TIME IS BLIND TO ANY DEFECT THE
  OTHER PARAMETERS CAN DOMINATE.** Unit #13, and it is the eighth corpus blind
  spot -- the one where reaching the right magnitude bought nothing at all.

  Four `assoc_reorder` mutants survived `NotchFilter` at 0.968:
  `2.0 * (x*x)` regrouped as `(2.0*x) * x`. Adding a REAL magnitude ladder --
  unit #10's integer decade ladder, one type over, and the obvious fix -- moved
  the case count 1380 -> 2172 and **the score not at all**. The rung puts `omega`
  at 1e-158; the coefficient is

  ```
  (2.0*(omega*omega) - 2.0*(K*K)) / ((K*K) + 2.0*omega*betaDen*K + (omega*omega))
  ```

  and the second term is still at the +/-1e3 default, so a 2^-1074 difference is
  annihilated in the subtraction. The fix is to run each rung again with every
  OTHER defaulted scalar real **pinned out of the way** -- `0.0`, and `1e300` so
  that a reciprocal like `K = 2.0/DT` underflows. 2652 cases, **126 of 126,
  1.000, 0 declared equivalent**
  (`evidence/NotchFilter/joint_magnitude_block_kills_the_assoc_mutants.txt`).

  Two things generalise, and the second is the cheaper one:

  1. Before adding corpus values for a survivor, ask what else appears in **the
     same expression** and whether it will drown them.
  2. **A round decimal is a bad rung for a rounding defect.** Measured:
     `1e-155`, `1e-161` and `sqrt(DBL_MIN)` do NOT distinguish the regrouping;
     `1e-156`, `1e-158`, `1e-160` do, and at exponent -535 so do 126 of 256
     mantissas. `evidence/NotchFilter/hot_mantissa_probe.cpp` is three loops.

- **PROVE the survivor before declaring it, and prove it over the REACHABLE
  inputs -- not over the expression's free variables.** Unit #13, and the second
  half is what nearly produced a wrong answer.

  The first equivalence probe drew `K` directly and found witnesses at
  `K = 7.4e-313`. `K` is not an input: the reference computes `K = 2.0/DT`, so
  `|K|` below `2.0/DBL_MAX` is unreachable from any finite timestep and every
  such witness is fiction. Drawing `DT` and deriving `K` gives a REAL witness
  (`DT = 2.44e204`, `omega = 1.34e-154`) -- which is what makes the mutant
  killable rather than equivalent, and so decides the whole disposition.

  ```
  # draw the ARGUMENTS the signature has, then compute the locals the way the
  # reference computes them; a local's own range is not an input domain
  ```

  `equivalent_declared` stayed **0**. A declaration would have been false, and
  `min_mutation_score` is 1.0 precisely so that the cheap way out is closed.

- **A REAL exponent of exactly 2.0 is `x*x`; 3.0 and up is a libm call. Measure
  the one you have.** Unit #13. `NotchFilter` writes `K**2.0` and `omega**2.0`
  five times, and the check registry's own line says gfortran emits `pow` for a
  real exponent of 3.0 or more.

  ```
  gfortran -O3 -fdefault-real-8 -fdefault-double-8 -ffp-contract=off probe.f90
  # evidence/NotchFilter/pow_two_probe.f90 -- TRANSFER both to INTEGER(8), compare bits
  ```

  Identical on 200,010 values including the overflow-to-Inf and
  underflow-to-zero endpoints, so `K * K` is exact here. Two minutes, and it is
  the difference between transcribing and hoping. **`**` still binds tighter
  than `*`**: `2.0*omega**2.0` is `2.0 * (omega*omega)`, and it was the four
  parentheses that made those the only surviving mutants in the first place.

- **A post-integration red test can stay green because the perturbation matched
  TWO sites and so was applied to NEITHER.** Unit #13, and it is a second cause
  for unit #7's symptom -- upstream of the rebuild rather than downstream.

  `notchfilter_c(InputSignal, DT, omega, betaNum, betaDen,` occurs twice in the
  integrated `Filters.f90`: once in the `BIND(C)` interface block VIT emits and
  once at the call. A single-occurrence replacement asserted out, the source was
  left untouched, the rebuild rebuilt nothing, and the run reported
  `checked 2652  failed 0`. Anchor on the call: the interface has no
  `C_LOC(FP)` in it.

  ```
  grep -n '<unit>_c(<first arg>' rosco/controller/src/<File>.f90   # expect TWO
  ```

- **Build:** WORKS.
  ```
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/rosco/controller && \
      mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && \
      cmake --build . -j4"
  cp rosco/controller/build/libdiscon.so rosco/lib/libdiscon.so
  ```
  Pristine Fortran at `974290e` builds clean; `libdiscon.so` is 810,776 bytes.

- **Gate:** CARRIED IN, PARTIALLY WORKING -- see the blocker below.
  ```
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
      python3 Examples/vit_sim.py --scenario N --output-dir <dir>"
  ```
  `Examples/vit_sim.py` and `scripts/regress.sh` are NOT in upstream ROSCO at
  this commit. They were copied from the previous replication's setup commit
  `bf25e35` and hash-verified (`6e6b2623446d`, `c87a74e194d0`). Each scenario
  must run in its own process -- the DLL holds SAVE state across calls.

  `regress.sh` prints `channels compared: N  mismatched: M` and writes no
  artifact. Superseded for gating by `scripts/gate.py` below; still the way to
  regenerate baselines (`bash scripts/regress.sh --baseline`).

- **Gate (use this one):** WORKS, 2026-08-10.
  ```
  python3.12 scripts/gate.py <unit>          # -> gate/<unit>.json, exit != 0 on mismatch
  ```
  Runs all 27 scenarios, compares every channel against `baseline_arrays` on
  the exact bit pattern, prints the count next to the verdict and PERSISTS it.
  Measured on the pristine build: **5,252,000 values across 351 channels, 0
  mismatched, 55 s.**

  Three things it does that `regress.sh` does not, each because of something
  observed here rather than anticipated:

  1. **It counts VALUES, not channels.** 351 against 5,252,000. P9 only asserts
     `compared > 0`, so a channel-counted artifact passes while saying four
     orders of magnitude less than it appears to -- and would not be evidence
     about the same instrument E3.2 observed failing, since the red test is
     recorded in values.
  2. **It compares bit patterns via a uint8 view, not `==`.** `==` calls two
     identical NaNs different; `np.array_equal` collapses a whole channel to one
     bool.
  3. **It restores `Examples/DISCON*.IN` afterwards.** `vit_sim.py`'s
     `write_discon()` rewrites those files in place per scenario and never puts
     them back -- measured here as `IPC_ControlMode` 1->2 and `F_NumNotchFilts`
     0->1 left in the tree. `loop/done.py` runs `require_clean_tree=True`, so a
     unit session that ran the gate could not close; a session that resolved the
     dirt with `git add -A` would commit a silently reconfigured gate for every
     unit after it. `gate.py` snapshots and restores them, records which it put
     back in `inputs_restored`, and reports anything still dirty in
     `residual_dirt` rather than assuming its own list is complete.

  Exiting non-zero when it compares NOTHING is not part of E3.1 as written. It
  follows from SPEC §7's reasoning that the count is what catches the vacuous
  case; recorded in DECISIONS.md as a local decision.

- **Baseline capture:** 26 of 27 scenarios. Scenario 4 does not run -- below.

- **Red-test the gate:** DONE 2026-08-10, and it took two attempts. Perturb a
  line, rebuild, re-run, compare against `baseline_arrays`, revert, re-run.

    attempt 1  `RootMOOPF(K) = rootMOOP(K)` (the ELSE branch) x 1.000001
               ->  624,000 compared, 0 differ.  NOT a gate failure: scenarios
                   1/4/6 do not take that branch, so the line was never run.
    attempt 2  `LPFilter = 1.0/...` -> `1.000001_DbKi/...`
               ->  624,000 compared, 198,892 differ across all 3 scenarios. RED.
    revert     ->  624,000 compared, 0 differ. GREEN restored.

  **The first attempt is the lesson.** A red test that picks an unexercised line
  produces a green and proves nothing -- it looks exactly like a gate that
  cannot fail. Perturb something the scenarios certainly reach (`LPFilter` runs
  many times per timestep in every scenario) and confirm green returns after the
  revert, or the red is not attributable to the perturbation.

- **A generated corpus can contain ONE ORDERING, and the check is three lines
  of decoding, not a reading of the generator.** Unit #12, and it is the seventh
  corpus blind spot -- the one that made the differential harness green for the
  same reason the kernel was.

  `harness/generate.py`'s `_fill_array` returns `lo + span*(k+1) + jitter`: a
  strictly ASCENDING ramp, in every case, for every array parameter this
  generator has ever filled. `NonDecreasing`'s whole body is
  `Array(I+1) - Array(I) <= 0`, so all 25 cases answered `.TRUE.` and a
  translation reading no argument and returning `.TRUE.` passes -- which is the
  identical green the KERNEL gives, because `NonDecreasing = .FALSE.` has zero
  hits in all 27 scenarios. **Two instruments, one blindness**, and the second
  exists to not share the first's.

  Decode the case file and count the DISTINCT ANSWERS before believing any green
  on a unit whose output is a predicate:

  ```
  python3.12 - <<'EOF'
  import struct
  b = open('translations/<Module>/<unit>_test/<unit>_cases.bin','rb').read()
  # decode per the unit's own _order; evaluate the reference's predicate in
  # Python and count how many cases give each answer
  EOF
  ```

  An ORDER LADDER now exists: reversed, one adjacent inversion at first /
  interior / last, one adjacent EQUAL pair at interior and last, the constant
  array, and lengths 1 and 2. Fired only for an array the REFERENCE ITSELF
  subscripts twice in one statement (`order_arrays_from`, read out of the
  Fortran, which is the oracle -- a red-test stub contains no subscript).
  Deliberately NOT "any array": an interpolation table's reference SEARCHES a
  sorted array, one subscript per statement, and an unsorted one would put it
  outside its own admissible domain. Appended last, and the block draws no
  random numbers when it does not fire, so no earlier unit's corpus moves.

  The number that says it bought something: both `<= 0.0` -> `< 0.0` mutants are
  killed by 4 of 36 cases, and all 4 are ladder cases
  (`evidence/NonDecreasing/order_ladder_kills_the_le_mutant.txt`). Without it the
  score is 0.875. Those same 4 are the only cases the post-integration
  `SIZE(Array) - 1` red test can reach.

- **A NUMERIC ARRAY's extent is an ORDINARY C PARAMETER that `vit interface`
  puts AFTER the buffer, and the harness emitter declared in signature order.**
  Unit #12. `std::vector<double> Array_a(n_Array_a);` came out one line before
  `n_Array_a` existed -- a compile error, not a wrong answer, and so cheap to
  spot that it is worth knowing it is a fixed shape rather than a new bug:

  ```
  grep -n 'was not declared in this scope' <the harness build log>
  ```

  The CHARACTER path had already solved it, with a `predeclared` placeholder
  that reads the length WITH the buffer and passes it at its own argument
  position. **A fix applied at one of two sites that share a code shape is a fix
  the other site escapes** -- the fifth instance here. Fixed by reusing that
  machinery, gated on `names.index(d) > i`, which is exactly the condition under
  which the old emitter produced the error, so nothing that ever compiled moves.
  A second defect rode along: at its own position the extent was typed from
  `elem_ctype`, which defaults to `double`.

  And `scripts/_integration_shim.py` emitted `extern "C" int32_t <unit>_c(...)`
  with **no `#include <cstdint>`**. The shim compiles STANDALONE and includes
  nothing the translation includes; it already carries two conditional includes
  for this reason, and a LOGICAL RESULT is the first fixed-width return this
  campaign has produced.

- **A LOGICAL FUNCTION RESULT crosses as `INTEGER(C_INT)`, gfortran accepts the
  assignment as an EXTENSION, and what the extension DOES has to be measured.**
  Unit #12. `vit interface` emits
  `NonDecreasing_result = nondecreasing_c(Array, SIZE(Array))` -- `LOGICAL =
  INTEGER`, which is not standard Fortran. gfortran compiles it with one warning
  and exit 0, so the build says nothing useful; a bit-copy and a normalisation
  give different answers for a return value of 2, and only one of them is safe.

  ```
  # evidence/NonDecreasing/logical_result_conversion_probe.f90 is the pattern
  gfortran probe.f90 -o /tmp/probe && /tmp/probe
  ```

  It NORMALISES: `0` -> `.FALSE.`, and `1`, `2`, `-1`, `256` all -> `.TRUE.` with
  `TRANSFER(L,0) == 1`. Returning 1/0 from the C++ is exact. Also read the
  wrapper for what it ADDS: it declares `INTENT(IN)` on a dummy the original
  declares with no intent at all.

- **A gate red test that reproduces ANOTHER unit's red number exactly is one
  finding, not two.** Unit #12. Forcing `NonDecreasing` to answer `.FALSE.` moved
  **1,857,893 of 5,252,000** -- byte-identical to `gate/GetWords.redtest.json`,
  the figure the campaign already designates as its same-build control.

  Read forwards that is a fifth gate-visible unit. Read properly it says the two
  perturbations END IN THE SAME PLACE: every live call site is
  `.NOT. NonDecreasing(...)` -> `ErrVar%aviFAIL = -1`, so the controller rejects
  its own input file, which is exactly what breaking GetWords' word parser does,
  and a rejected input file has ONE output signature. The red is real and
  attributable, and what it constrains is a SINGLE BOOLEAN -- nothing about the
  array, and nothing about the answer no scenario produces.

  Two things follow, both worth keeping. Such a red test IS its own control -- it
  has just reproduced the control figure. And before writing "the gate sees this
  unit", compare the moved count against every committed redtest artifact:

  ```
  python3.12 -c "import json,glob; [print(f, json.load(open(f)).get('mismatched')) \
      for f in sorted(glob.glob('gate/*redtest*.json'))]"
  ```

- **When the unit is called inside an IF CONDITION, KGen instruments the IF and
  the compared fields are the BRANCH's outputs.** Unit #12, and it is the shape
  unit #7's `!local verify variables` check is for. `IF (... .AND. .NOT.
  NonDecreasing(CntrPar%PC_GS_angles)) THEN ErrVar%aviFAIL = -1` gives a kernel
  comparing `cntrpar` and `errvar` and naming the unit's result nowhere.

  That is not automatically dead -- the result reaches `errvar` by deciding
  whether the branch runs -- but it makes the pair of stubs mandatory and it
  makes WHICH constant you pick load-bearing:

  ```
  the constant the scenarios ACTUALLY produce   -> if it PASSES, the kernel is a
                                                   lookup table
  the OTHER constant                            -> if it FAILS, the comparison is
                                                   alive
  ```

  Here `.TRUE.` passes 200 of 200 and `.FALSE.` moves exactly 2 rows, `avifail`
  and `errmsg`. Pick the constants from the COVERAGE of the branch bodies, not
  from the signature: clean `ROSCO_Helpers.f90:1569` has zero hits in all 27
  scenarios, which said in advance that `.TRUE.` was the only answer the kernel
  would ever have seen.

  **An input VALIDATOR is this shape by construction.** All 27 scenarios read
  valid input files, so the branch the unit exists for is the branch none of them
  takes -- the same argument unit #10 recorded about a unit whose job is
  rendering error messages, one layer up.

- **`vit_mutate.py` mutates the TRANSLATION FILE IN PLACE, so an interrupted
  mutation run leaves a MUTANT in the tree and nothing says so.** Unit #11.

  A run was stopped mid-flight (`pkill`) to correct a comment. It restores the
  file when it finishes; killed, it does not. The file left behind read
  `*inst = *inst + 2;` — four characters, in a file whose own header still says
  `Status: unverified`, sitting in the exact place the next `vit integrate` would
  have taken it from. `git status` showed it as the same untracked path it had
  been all along.

  What caught it was re-running the differential harness for an unrelated reason
  and getting `checked 996  failed 996` where the identical command had passed
  ten minutes earlier. So the rule is cheap and it is not optional:

  ```
  cp translations/<Module>/<stem>.cpp /tmp/<stem>.intended.cpp    # BEFORE mutating
  # ... run mutation ...
  diff /tmp/<stem>.intended.cpp translations/<Module>/<stem>.cpp  # AFTER, always
  ```

  **Do not read, copy, or edit the translation while a mutation run is live**,
  and treat any harness/kernel artifact taken during one as measuring a mutant.
  The window is minutes wide and the failure is silent in both directions: a
  mutant that happens to pass the layer you are running looks like a green.

- **The differential harness read the INDEX ROLE off the translation only, so
  the one run that proves it can fail was the one run it could not complete.**
  Unit #11, fixed by addition rather than worked around.

  `map_signature(..., cpp_source=...)` promotes a scalar to `role="index"` by
  finding `FP->lpf1_a1[i]` in the C++. The no-op red test contains no such
  subscript, so `inst` stayed an ordinary scalar integer, the generator drew it
  from unit #10's decade ladder up to `INT_MAX`, and the REFERENCE — which
  subscripts `FP%lpf1_a1(inst)` on a `DIMENSION(1024)` array — segfaulted.
  `harness.sh` reported **"harness produced no JSON"**, which is the same
  sentence a reference that PRINTS produces, and wrote no artifact at all.

  The input domain of a differential harness must not be a function of the code
  under test. The REFERENCE states the role too, and it is the oracle (P7):
  `infer_indexes_fortran` reads `arg%field(expr)` out of the unit's own Fortran
  body and fills only what the translation left ordinary, so no already-measured
  unit's corpus moves. Loop repo; `unit_body()` allows the TYPE-prefixed
  declaration (`REAL(DbKi) FUNCTION LPFilter(...)`) that `char_literals_from`'s
  line-anchored copy of the same search silently misses.

  The check, and the number that says the fix worked: the red test's case count
  must MATCH the green run's. 1157 before (an `inst` nobody constrained), **996
  after — the green run's own figure**, with `996 of 996 failed`.

  ```
  grep -E 'note: inst|case\(s\)$|HARNESS' <the red run's log>
  ```

- **A gate red test that comes back GREEN needs a CONTROL taken on the SAME
  BUILD, or it is not a finding.** Learned at unit #9, and it is owed to the five
  units before it that recorded `0 of 5,252,000` without one.

  "The gate cannot see this unit" and "the chain from build to install to 27
  simulations to bit comparison is broken today" produce the IDENTICAL artifact.
  Nothing in `gate.py` distinguishes them: `revert_verified: true` only says the
  revert also moved nothing, which is equally true in both worlds.

  Re-run a perturbation that is ALREADY KNOWN RED, from a unit already committed,
  against the build in front of you:

  ```
  python3.12 scripts/gate.py <ThisUnit> --perturb-file rosco/controller/src/getwords.cpp \
      --perturb-from 'if (NextWhite > 1) {' --perturb-to 'if (false && NextWhite > 1) {' \
      --out evidence/<ThisUnit>/gate.control-getwords-perturbed-MOVES.json
  ```

  It must reproduce `gate/GetWords.redtest.json`'s **1,857,893 of 5,252,000**. A
  control that moves a DIFFERENT number is its own finding. Costs one gate run
  (~3 min) and it is the difference between a measurement and a shrug.

- **Ask what SCALES the unit's result, not what CONSUMES it.** Unit #9, and it is
  the sixth P9 shape -- the one where unit #6's "follow the OUTPUT" and unit #7's
  "does the live reader put it somewhere the gate READS" both answer YES and are
  both wrong.

  `HPFilter` runs 892,000 times across 23 of 27 scenarios and all four of its
  readers run. It is still invisible, because every call site is multiplied by
  something zero, and no two sites share the mechanism:

  ```
  grep -h '<the gain the reader multiplies by>' Examples/*.IN | sort -u
  grep -n '<that gain>' Examples/vit_sim.py        # is it PATCHED by a scenario?
  ```

  `Fl_Kp` is `0.0000` in all 14 inputs and never patched, so `Kp_Float` is 0 and
  `FloatingFeedback` is 0. `FA_KI` likewise, with the proportional gain the
  literal `0.0_DbKi` at the call. Both greps are seconds; tracing the readers by
  hand took most of an hour and answered the wrong question.

  The third site needed a different check -- the consumer is live and the gain is
  non-zero, and the CHANNEL is constant anyway:

  ```
  python3.12 -c "import numpy as np; d=np.load('baseline_arrays/scenario_<N>.npz'); \
      print({c: len(set(d[c].tolist())) for c in d.files})"
  ```

  A channel with **1 distinct value** cannot carry any result out of any unit.
  Check it before writing an observability note that blames the unit.

- **A derived type can cross as `C_LOC(FP)` onto a flat C struct with no view
  populator, and then the FIELD ORDER is load-bearing and unchecked.** Unit #9.

  `vit integrate` emits `C_LOC(FP)` for `TYPE(FilterParameters)` and the C++ does
  `FP->hpf_InputSignalLast[i]`. That is sound only because the type is 46
  fixed-size `REAL(DbKi), DIMENSION(1024)` fields -- no ALLOCATABLE, no CHARACTER,
  no padding possible. **A reordered or short struct compiles, links, and reads
  the wrong array in silence**, and no layer here would catch it: the kernel, the
  harness and the mutation run all include the same `vit_types.h`. Verify against
  the Fortran declaration, not against the header:

  ```
  python3.12 - <<'EOF'
  import re
  blk = open('rosco/controller/src/ROSCO_Types.f90').read() \
        .split('TYPE, PUBLIC :: <Type>')[1].split('END TYPE')[0]
  fort = re.findall(r'::\s*(\w+)', blk)
  cb = open('rosco/controller/src/vit_types.h').read().split('} <type>_t;')[0]
  c = re.findall(r'double\s+(\w+)\[<N>\];', cb[cb.rfind('typedef struct'):])
  print(len(fort), len(c), 'ORDER+NAMES MATCH:', fort == c)
  EOF
  ```

- **A CHARACTER FUNCTION RESULT is a `char*` whose WIDTH IS NOT A PARAMETER, and
  three separate places assumed a width always is one.** Unit #10, and it is the
  same disagreement-between-generators shape unit #8 recorded, one type over.

  `CHARACTER(11) :: Int2LStr` is not a C return value. `result_is_out_param`
  makes it a trailing `char* <Func>_result` the caller owns and the callee
  blank-fills, and `vit interface` emits that with the width COMPILED INTO BOTH
  SIDES (`DO vit_i_result = 1, 11`). `build_c_params` therefore emits no
  `len_<x>` -- there is nothing to emit, the width is a literal in the
  FUNCTION's own declaration -- and the differential harness refused the unit's
  ONLY output for its absence.

  Ask both generators, and for a FUNCTION ask a third question of the harness
  script itself:

  ```
  vit interface <Unit> -f <file> -o /tmp/<u>_iface        # does the result cross?
  # ... map_signature as in the unit #8 entry above ...
  grep -n 'result_ctype' /workspace/translation-loop/scripts/vit_harness.py
  ```

  **`vit_harness.py` set `result_ctype` for ANY function.** A result already
  crossing as an out-parameter would have been emitted a SECOND time as a return
  value: `char ret_a = int2lstr(...)` against a wrapper returning `void`, plus a
  `&ret_b` argument the Fortran bridge has no dummy for, through a C linkage that
  checks neither. It asked `is_function` where the question is *does the result
  come back through the RETURN VALUE* -- which VIT already answers, in one
  predicate. Fixed in the loop repo; the check before believing any FUNCTION's
  harness is that the generated test declares the bridge with the same arity VIT's
  `vit interface` printed:

  ```
  grep -n 'extern "C" void <unit>_f90' translations/<Module>/<unit>_test/<unit>_test.cpp
  ```

- **`intent="out"` on a buffer the harness must SUPPLY is a stream with nothing
  in it.** Same unit, thirty seconds after the mapping was fixed:
  `Signature.inputs` excludes an `out`, so nothing filled the result buffer and
  `write_cases` died with *"has 0 element(s) but its extents say 11"*.
  `_arg_intent` had already drawn this line -- *"an OUT struct is still supplied
  (allocated) and compared"* -- and `inout` is also the STRONGER choice: the
  bytes a translation fails to write are the ones the harness put there, and they
  are compared. Read the no-op red test's `got` column to confirm it: it should
  show the harness's own fill, not zeros.

- **A scalar INTEGER reaches no value this generator CHOSE -- it is a uniform
  draw over a range nobody declared, and for a width-dependent unit that is the
  whole blindness.** Unit #10, and it is the third corpus blind spot after unit
  #7's three and unit #8's two.

  `_ladder` and `_literal_values` are both driven off `reals`. The only thing
  that has ever set a scalar int is `rng.randint` over `_bounds`' DEFAULT
  `+/-1e3`. A Fortran default INTEGER is the whole 32-bit domain; `+/-1e3` is
  0.00005% of it, all of it in the middle.

  `Int2LStr` formats a number into an 11-character field, so every branch it has
  is a branch about the WIDTH of the number -- and at `|Num| < 1000` the width is
  1 to 4 in every case ever generated. Nine mutants survived a 144-case green for
  want of a value the generator could not produce. Check it before believing any
  green on a unit that formats, pads, justifies or field-widths a number:

  ```
  python3.12 - <<'EOF'
  import struct
  b = open('translations/<Module>/<unit>_test/<unit>_cases.bin','rb').read()
  # decode per the unit's own _order; print the distinct values of each int
  EOF
  ```

  An integer DECADE ladder now exists (`_int_magnitude_ladder`): `9/10`,
  `99/100`, ... `10^9`, both signs, plus `INT_MAX` and `INT_MIN`. Those are the
  only places a width-dependent predicate changes its answer. `-2147483648` is
  listed rather than derived -- it is the one value whose magnitude has no
  positive counterpart, and it is exactly 11 characters wide. Appended last and
  fired only for a DEFAULTED scalar int, so it can only ADD cases and no earlier
  unit's draws move. **Not an X3 change**: no default is loosened, no existing
  case is altered, and the effect is strictly more input and strictly more
  mutants killed.

- **The KERNEL may compare the CALLER's variable, not the unit's result, and then
  three stubs are not enough.** Unit #10. `Int2LStr` is called inside an
  expression -- `OL_String = TRIM(OL_String)//' Cable'//TRIM(Int2LStr(I))//' '`
  -- so KGen instruments the ASSIGNMENT and the compared field is `ol_string`.
  The generated `!local verify variables` names the unit's own output NOWHERE,
  which is exactly the check unit #7 said to run before trusting a kernel.

  Run FOUR stubs when the result reaches the compared field through `TRIM`,
  concatenation, or any other narrowing:

  ```
  no-op            must FAIL   -- the comparison is alive
  WRONG constant   must FAIL   -- alive on the VALUE, not just on presence
  the SHAPE stub   ?           -- right digits, wrong PADDING (use a non-blank)
  RIGHT constant   if it PASSES the kernel is a lookup table
  ```

  The third one is the trap and is worth keeping even when it fails. `'X'`
  padding fails here, which reads like proof the kernel sees all 11 bytes -- and
  it only survives `TRIM` because `'X'` is not a blank. A translation that padded
  with a different BLANK would be invisible. **A stub that fails does not tell
  you why it failed.**

- **Ask whether the only reader is a `PRINT` -- it is one grep, and it settles
  the observability note before any tracing.** Unit #10; unit #7's shape reached
  in seconds rather than an afternoon.

  ```
  grep -n '<the out-variable>' rosco/controller/src/*.f90    # EVERY reader
  ```

  If every reader is a `PRINT`, a `WRITE` to a log unit, or a filename, the gate
  cannot see the unit and no amount of hit count changes that: `gate.py` compares
  `baseline_arrays/scenario_N.npz`, built from the arrays crossing the DLL
  boundary, and never reads stdout.

  And ask what the DEAD call sites have in common before calling them incidental.
  Eighteen of `Int2LStr`'s twenty sites build `ErrVar%ErrMsg` strings or a
  debug-file FORMAT. **A unit whose job is rendering numbers into human-readable
  messages has output a numerical gate structurally does not look at**, and all
  27 scenarios read valid input files, so no error message is ever built. That is
  a property of the unit's PURPOSE, not an accident of coverage.

- **A CHARACTER ARRAY dummy is TWO extents, and two of this campaign's generators
  disagreed about it in opposite directions.** Learned at unit #8, and the
  disagreement is the transferable part.

  `CHARACTER(*), INTENT(OUT) :: Words(NumWords)` is an array of fixed-width
  strings: an element WIDTH and an element COUNT where every earlier unit's
  CHARACTER dummy had one length. `vit interface` handles it -- it stages the
  argument column-major, `Words_c((j-1)*LEN(Words)+i) = Words(j)(i:i)`, and the
  wrapper compiles and works. The DIFFERENTIAL HARNESS refused it outright. So
  the unit had a shipping bridge and no P11/P12 route at all, and the refused
  argument was its ONLY output.

  ```
  # ask BOTH generators before writing C++, not just the one that ships
  vit interface <Unit> -f <file> -o /tmp/<u>_iface
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && python3 -c \"
  import sys; sys.path.insert(0,'/workspace/translation-loop')
  from vit.fortran_parser import find_function_in_file
  from harness.vitbridge import map_signature
  m = map_signature(find_function_in_file('<file>','<Unit>'), {...})
  print(m.unobservable)\""
  ```

  A non-empty `unobservable` naming an OUTPUT is a unit with no differential
  harness, whatever `vit interface` says. Built in the loop repo (`6d13949`);
  rank >= 2 is still refused. The trap inside the fix, if it needs touching
  again: the element count is an ORDINARY dummy of the original, so nothing in
  the emitted C signature says it sizes anything -- it comes off the Fortran
  declaration, and VIT's parser UPPER-CASES the dimension text while
  `build_c_params` keeps the argument's declared spelling. Match case-folded or
  the lookup finds nothing and reads like a declaration naming no parameter.

- **An extent that is also a semantic input must be `role="extent"`, and getting
  that wrong is worse than a wrong length.** Same unit. `NumWords` is both the
  element count of `Words` and the value the loop tests `IW ==` against. Left as
  an ordinary integer it is varied over +/-1e3 -- and it is the extent of a
  buffer BOTH SIDES WRITE THROUGH. A wrong length reads past a buffer; a wrong
  count writes past one.

- **The corpus cannot contain a character the reference cannot write, and the
  characters a reference cannot write are the ones that matter most.** Unit #8,
  and it is the third and fourth corpus blind spot after unit #7's two.

  `GetWords` separates words on `' ,!;''"'//Tab` -- seven characters. The corpus
  contained five.

  1. `''` inside a `'...'` literal is ONE APOSTROPHE in Fortran. `'([^']*)'`
     reads it as two literals and drops the character. An apostrophe is a word
     separator, so it is exactly what a SET literal exists to name.
  2. `Tab = CHAR(9)`, because a tab cannot be written as a source literal at
     all -- and `char_corpus` filtered `32 <= v <= 126`, so it would have been
     discarded even if mined. The rule that filter was reaching for is *do not
     INVENT unprintable characters*, not *the reference cannot have meant one*.

  Both fixed in the loop repo (`5b40e1c`); a mined character is admitted, its
  neighbours must still be printable, NUL stays out. Check before believing a
  string unit's corpus:

  ```
  python3 -c "from harness.generate import char_corpus; print(sorted(char_corpus(<mined>)))"
  ```

- **`padded` makes the END of a string interesting and pins its START.** Unit #8.
  The three shapes were mixed / word-then-blanks / all-blank, and all three put
  a word at position 1 -- so every predicate about where a word BEGINS had one
  answer in every case, and `Ch = Ch + 1` -> `Ch + 2` survived 978 of them. A
  fourth shape, `leading`, is `padded`'s mirror. ROSCO's own `DISCON*.IN` files
  are full of it.

- **A mutation score can FLAP, and the run that reads 1.000 can be the one that
  measured least.** Unit #8. Three IDENTICAL runs -- same translation, same 1370
  cases, same command -- scored 0.983, 1.000, 0.983. The mutant responsible
  differs from the original only at addresses past the end of a buffer, so
  whether it dies depends on the heap.

  Two rules follow, and the campaign now depends on both:

  1. **Before declaring a survivor equivalent on an out-of-bounds argument,
     PROVE it exhaustively rather than reasoning about it.** Enumerate the input
     space at small sizes with the bytes past the buffer set to the value most
     able to disagree, and count in-bounds disagreements:
     `evidence/GetWords/mutant_91681005_probe.cpp` is the pattern -- 3,279
     strings x every reachable offset, `differ-IN-BOUNDS 0`.
  2. **A declaration is a statement about the MUTANT, not about one run.**
     `vit_mutate.py` drew its equivalence set from the mutants that SURVIVED, so
     a declared mutant that happened to die was counted an ordinary kill and the
     declaration did nothing. Fixed (`5b40e1c`); a declared mutant that is
     killed anyway now lands in the artifact's `declared_but_killed` rather than
     being absorbed.

- **A revision stamp read from a hand-written pin file goes stale in silence, and
  it took the campaign's OWN script down with it.** Unit #8. `vit-dev` has no
  git, so `loop_rev`/`vit_rev` fell through to `.loop_rev` and `.vit_rev` --
  gitignored, hand-maintained, and both wrong: unit #7's artifacts stamp
  `99b57ab-pinned` against a tree at `0e92a72`, which is the commit that unit's
  own corpus fix went in as, and every artifact since unit #5 says
  `8c34ceb-pinned` against a VIT at `87a3847`.

  `.git/HEAD` and the ref it names are plain text and need no binary. Both loop
  scripts read it before the pin now and report `-nogit`, which cannot see a
  dirty tree and does not claim to. **And `scripts/_harness_stamp.py` was a
  third site with the same logic that OVERWROTE the correct read with the stale
  pin** -- the pre-integration run stamped `57c6fe3-nogit` and the red-test run,
  passing through that script, came back `6d13949-pinned`. A measured value
  clobbered by a claimed one. Read the stamp of every artifact before believing
  its numbers, and check that the two artifacts of a green/red PAIR agree about
  which instrument produced them.

- **A KGen kernel that will not COMPILE is a different failure from one that
  captures nothing, and this one is on the critical path.** Learned at unit #6.
  `vit extract` reported success, and `vit verify` died in the kernel build:

  ```
  CHARACTER(LEN=accinfile_size), DIMENSION(:), ALLOCATABLE :: accinfile
  Error: Variable 'accinfile_size' cannot appear in the expression at (1)
  ```

  KGen hoists the call site's ENCLOSING procedure's dummies into the kernel
  driver's PROGRAM scope, and `ReadControlParameterFileSub` declares
  `CHARACTER(accINFILE_size) :: accINFILE(accINFILE_size)` — an AUTOMATIC length:
  legal on a dummy, illegal on a local. That procedure also encloses the
  `ParseInput_*`, `ParseAry`, `FindLine` and `GetWords` call sites, so a large
  share of the remaining units reach the same wall.

  FIXED IN KGEN (X2), additively — `is_automatic_length_char` beside the existing
  `is_assumed_length_char`, reusing the deferred-length machinery `c839e1a`
  already built for `CHARACTER(*)`. Two things to know before touching it again:

  1. The predicate is deliberately narrow. It fires only when the length names an
     entity THIS SCOPE declares as a non-PARAMETER variable, so
     `CHARACTER(MaxParamLength)` — a `USE`d constant, all over `ROSCO_Helpers` —
     is untouched and no already-measured kernel changes shape.
  2. The read and write sides must decide it IDENTICALLY. The element length is
     one record at the front of the state file; one side emitting it without the
     other shifts every field after it. Only the caller knows whether it is
     hoisting an ARGUMENT (becomes a PROGRAM local → defer) or a LOCAL (stays in
     a subroutine → legal, leave it), so the flag is passed in.

  Red-test a KGen change in both directions before believing it, and pick the
  control by SHAPE: re-extracting `Conv2UC` exercises `FindLine`, which has both
  a `CHARACTER(*)` dummy and `CHARACTER(MaxParamLength)` locals. Its generated
  kernel must come back byte-identical apart from the timestamp comment:

  ```
  diff <(sed 3d kernel/Conv2UC/ROSCO_Helpers.f90) \
       <(sed 3d evidence/Conv2UC/kernel-generated-ROSCO_Helpers.f90)
  ```

  **A committed `kernel-generated-*.f90` is a POST-`vit verify` file, not KGen's
  output.** Learned at unit #7 while trying to use unit #6's `GetPath` artifact
  as that control. `vit verify` reports "Modified 10 files" and one of them is
  the generated callsite file: it collapses the `USE kgen_utils_mod` lines to
  `ONLY:` lists, lowers `verboseLevel` from 100 to 1, adds a NaN branch and the
  `[VIT_FIELD]` prints. Diffing a fresh pre-verify extraction against it shows
  five differences that are all VIT's and none KGen's. Control a KGen change
  against a run of the SAME stage — stash the patch, re-extract, `diff` the two
  fresh kernels.

- **A KGen kernel can COMPILE, RUN, print `62/62 passed`, and compare NOTHING.**
  Learned at unit #7, and VIT is what caught it:

  ```
  ✗ VERIFICATION FAILED: 62/62 passed
    FAILED: kernel compared 0 output variables — nothing was verified
  ```

  `CALL GetRoot(RootName,RootName)` passes one variable to both an INTENT(IN)
  and an INTENT(OUT) dummy. `kganalyze.update_state_info` promotes a variable to
  STATE_OUT by finding its position in the argument list and reading the matching
  dummy's INTENT — with `arglist.items.index(argobj)`, and `Fortran2003.Base`
  compares nodes by CONTENT, so both occurrences resolved to argument 0
  (INTENT(IN)) and the variable stayed an input. Measured rather than read:

  ```
  Actual_Arg_Spec_List('RootName, RootName')  ->  index(items[1]) == 0
  Actual_Arg_Spec_List('A, B')                ->  index(items[1]) == 1
  ```

  FIXED IN KGEN (X2), `4457cd2`, by searching for the argument node by IDENTITY.
  **The dangerous version is a call site with a SECOND out-argument**: the kernel
  compares that one, drops this one, and says nothing at all. Before trusting a
  kernel, read the generated callsite file's `!local verify variables` section
  and check that every output the unit writes is named there.

  Fixing it exposed a second defect one file over: `get_typedecl_subpname` builds
  a procedure name out of the declaration's selector, so
  `CHARACTER(LEN=size(avcoutname))` produced
  `SUBROUTINE kv_discon_character_size(avcoutname)_(...)`, which gfortran cannot
  read. `c839e1a` had already fixed exactly this on the GENCORE side; the
  VERIFICATION side has the same two lines and never got it. **A fix applied at
  one of two sites that share a code shape is a fix the other site escapes** —
  the same lesson unit #5 recorded about the 132-column bridge generator. The
  sanitiser now lives in `kgutils` and both import it.

- **The captured case count is NOT stable across runs of the identical command.**
  Unit #4 recorded 63 cases for `Conv2UC`; the same command on the same clean
  source now yields 62, with and without unit #6's KGen change. The configured
  window `0:0:1-20,0:0:12000-12020,0:0:23900-23920` is 20 + 21 + 21 = **62**, and
  #4's committed list carries one extra outside the first range
  (`Conv2UC.0.0.21`) — most likely a stale state file swept up from an earlier
  attempt. Compare the INDICES against the window, not the totals against a
  previous run.

- **Extract / capture:** WORKS. Point it at a call site that RUNS.
  ```
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
      vit extract ColemanTransform --file rosco/controller/src/Controllers.f90 \
      --line 510 --run-args ' --scenario 27'"
  ```
  63 state files, first attempt, 2026-08-10.

  **The earlier entry here said extraction was broken. It was not.** Three runs
  on `AddToList` printed `✓ Extraction successful` and `WARNING: No state data
  captured.`, and the instrumented library, the run command and the invocation
  window were each ruled out by measurement. What was never measured was the
  premise underneath all three: `ROSCO_IO.f90:1126` has **zero hits in all 27
  scenarios**, as do the other four `AddToList` call sites. There was no state
  to capture. KGen had instrumented a region the process never entered and said
  so.

  So the order matters more than the checks. Before diagnosing a capture
  failure, query `coverage/line_coverage.json` for the call site's hit counts.
  A call site with no hits is not a tool failure.

  **Hit counts are necessary and NOT sufficient.** See the invocation entry
  below: scenario 6 executes this call site 3,999 times, on zeros, and produces
  a kernel that a zero-writing stub passes.

  **Extraction is not read-only.** Each run strips CRLF from
  `rosco/controller/src/DISCON.F90` (162 lines, content identical, 7956 -> 7794
  bytes) -- including under `--dry-run` -- and leaves `include.ini`,
  `strace.log`, `kgen.log`, `model/`, `elapsedtime/` and `.vit_build_wrapper.sh`
  behind. Those are now gitignored; DISCON.F90 must be checked out afterwards.
  Note the launcher's `--protect 'rosco/controller/src/*.f90'` does NOT cover
  `DISCON.F90` -- lowercase glob, uppercase extension, verified by running
  `Path.glob` against the tree: 11 files matched, DISCON.F90 not among them.
  Add `--protect 'rosco/controller/src/*.F90'` as well.

  It also leaves the scenario's `Examples/DISCON_<name>.IN` modified — extraction
  runs `vit_sim.py`, whose `write_discon()` rewrites the `File written using
  ROSCO version ... on MM/DD/YY` header to today's date. Content-free and it
  still blocks `done.py`'s clean-tree predicate. `git checkout --` it;
  `reset_to_clean.sh` deliberately does not, because that script owns the SOURCE
  tree and these files are the gate's input, which `gate.py` already restores.

- **An argument can be a constant in every scenario, and neither the kernel nor
  the gate can say so.** Learned at unit #3. `ColemanTransformInverse`'s
  `aziOffset` is 0 at all five call sites in all 27 scenarios — `IPC_aziOffset`
  is `0.000` in all 14 `Examples/DISCON*.IN`, `AWC_phaseoffset` likewise and the
  one scenario patching it patches it to `'0.0'`, and the fifth site passes the
  literal `0.0_DbKi`. A translation ignoring that argument passes the kernel
  63/63 and the gate 5,252,000 of 5,252,000, and the gate's red test does not
  catch it either: perturbing the OUTPUT proves the unit is seen, not that every
  argument is.

  Only the differential harness varies it. So before writing the observability
  note, grep the scenario inputs for each argument that comes from `CntrPar` or
  from a literal:

  ```
  grep -h '<ParamName>' Examples/*.IN | sort -u
  grep -n "'<ParamName>'" Examples/vit_sim.py
  ```

  A parameter with one distinct value across all of them is a parameter the two
  bit-exact layers cannot constrain, and the harness's mutant kill counts are
  the evidence that anything does.

- **A line executed 1.3 million times can still be one the gate cannot see.**
  Learned at unit #4. `Conv2UC` is among the hottest procedures in the
  controller — 1,333,146 calls, 4,558,823 characters converted — and BOTH of
  these moved 0 of 5,252,000 values:

  ```
  python3.12 scripts/gate.py <Unit> --perturb-file rosco/controller/src/<u>.cpp \
      --perturb-from '<the write>' --perturb-to '<a wrong write>'
  python3.12 scripts/gate.py <Unit> --perturb-file rosco/controller/src/<u>.cpp \
      --perturb-from 'if (<guard>) {' --perturb-to 'if (false && <guard>) {'
  ```

  The second is the one to reach for, and it is worth a habit of its own: it
  makes the unit a NO-OP, so a gate that still cannot move is blind to the unit
  entirely, not merely to the perturbation you chose. Confirm `replacements: 1`
  and `revert_verified: true` in the artifact before believing either.

  The cause here is a shape to look for before writing an observability note:
  **the unit's output is only ever compared against another of its own
  outputs.** `FindLine` upper-cases the expected parameter name and the file's
  word with the same function and tests them equal, so any perturbation lands
  on both sides and equality survives. Grep the call sites for what consumes
  the result; if every consumer is symmetric in this way, no gate built on
  simulation output can constrain it, and the harness plus the mutation score
  are the whole of the evidence.

- **Before anything else, ask whether the ORACLE can be RUN — not just whether
  the unit is reached.** Learned at unit #5, and it is the question the first
  four units never had to ask because the answer was always yes.

  `ExtController` is 0/28 lines in all 27 scenarios, which by unit #1's rule
  makes it a dead unit that the harness and the mutation score should still
  carry. They cannot. Forcing the guard on — `Ext_Mode = 1` through
  `vit_sim.py`'s own `write_discon(patches=...)` — makes **the original Fortran
  segfault**, signal 11: `DLL_FileName` is the literal string `"unused"` in all
  14 inputs, no external Bladed-style library is shipped anywhere in the tree,
  and `ExtController` never checks `ErrVar%ErrStat` after `LoadDynamicLib`, so
  `dlopen` fails, the code prints *"Library loaded successfully"*, and the next
  two statements are `C_F_PROCPOINTER` on a null funptr and a CALL through it.

  P7 makes the original the oracle for **every** layer here, so a unit whose
  original cannot be run to completion has no verification route at all — not a
  weaker one. The check is cheap and belongs before the translate step:

  ```
  grep -h '<the guard parameter>' Examples/*.IN | sort -u    # one value = dead
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
      python3 evidence/ExtController/probe_ext_mode_1.py"    # can it RUN?
  ```

  `evidence/ExtController/probe_ext_mode_1.py` is the pattern: patch the guard
  through the campaign's own `write_discon`, make the `iStatus == 0` call, and
  record the child's exit status. **Run the crashing part in a CHILD process.**
  The first draft restored `Examples/DISCON.IN` in a `finally`, and a `finally`
  does not run through SIGSEGV — it left `Ext_Mode = 1` in the gate's own input,
  and because `Examples/DISCON.IN` is GITIGNORED (`.gitignore:78`) `git status`
  stayed clean and `done.py`'s P2 could not have caught it. Any restore that
  must survive a crash goes in the parent.

- **A kernel window can be VACUOUS WITH NO WIDER SETTING, and the check is a
  CONSTANT stub, not a zero stub.** Learned at unit #6. `GetPath`'s only call
  site is in initialisation, so it runs ONCE PER PROCESS: `vit extract` captures
  1 case whatever `vit.yaml`'s `invocation` says, and every scenario builds the
  argument the same way, so the expected answer is one string in all 27.

  Unit #2's recipe — count non-zero references, then widen the window — cannot
  reach this. The zero-stub test cannot either, because the reference values are
  not zero; they are *constant*. The test that works:

  ```
  # a stub that reads NO argument and writes the captured answer as a literal
  cp evidence/<Unit>/<unit>.constant-stub.cpp translations/<Module>/<unit>.cpp
  vit verify <Unit> ... --kernel-dir kernel/<Unit>      # if this PASSES, the kernel is a lookup table
  ```

  It passed 1/1 here, and that artifact is committed
  (`evidence/GetPath/kernel.constant-stub-PASSES.verify_fields.csv`) beside the
  no-op stub's `OUT_TOL`, which shows the instrument can still move. Run BOTH:
  the no-op says the kernel is alive, the constant stub says whether being alive
  buys anything.

  **Do not skip C6 on that account.** Running it is what produced the
  measurement. A mandatory step's whole value can be the red test that
  discredits its own green.

- **Before writing an observability note, ask whether the unit's result is
  CONSUMED — the line running is not the question, and neither is the line that
  reads it.** Learned at unit #6, and it is a fourth distinct shape after
  unexercised (#1), constant argument (#3) and cancelled-downstream (#4).

  `GetPath` runs 28 times across the 27 scenarios. `PriPath`, its output, has
  exactly two readers, and BOTH of those run 28 times too. Making the unit a
  no-op still moved 0 of 5,252,000. The readers are guarded:

  ```fortran
  IF (PathIsRelative(CntrPar%PerfFileName)) CntrPar%PerfFileName = TRIM(PriPath)//...
  IF (PathIsRelative(CntrPar%OL_Filename))  CntrPar%OL_Filename  = TRIM(PriPath)//...
  ```

  and the guard is false exactly where the answer would matter. So the recipe is
  to follow the OUTPUT, not the call:

  ```
  grep -n '<the out-argument>' rosco/controller/src/*.f90     # every reader
  grep -h '<each parameter those readers test>' Examples/*.IN | sort -u
  ```

  A reader whose guard has one value across all 14 inputs is a reader that
  cannot carry the result out. Hit counts on the reader's own line say nothing:
  here they are 28 of 28.

- **When the call site ALIASES an INTENT(IN) argument with an INTENT(OUT) one,
  the no-op stub stops being a liveness test and becomes a vacuity test.**
  Learned at unit #7, and it INVERTS the recipe unit #6 wrote three entries up.

  `DISCON.F90:67` is `CALL GetRoot(RootName,RootName)`. KGen captures the input
  and the reference output of the same variable, so they are the same bytes —
  the state file is literally `vit_sim1vit_sim1` — and a stub that reads nothing
  and writes nothing scores **62/62 IDENTICAL**. Unit #6's rule ("the no-op says
  the kernel is alive, the constant stub says whether being alive buys
  anything") read forwards here gives the wrong answer twice over.

  Run a THIRD stub, and let it write a WRONG constant:

  ```
  # no-op          -> if this PASSES, the kernel is a mirror, not a comparison
  # right constant -> if this PASSES, the kernel is a lookup table
  # WRONG constant -> if this FAILS, the comparison is alive. This is the
  #                   liveness test whenever the arguments alias.
  ```

  62/62 `OUT_TOL` for the wrong constant is what makes the other two artifacts
  statements about the UNIT rather than about a broken check
  (`evidence/GetRoot/kernel.wrong-constant-stub.verify_fields.csv`).

  The same aliasing makes the GATE's standard no-op perturbation degenerate: a
  no-op returns the caller's own bytes, which on this campaign's inputs is the
  correct answer, so `0 moved` says nothing. Perturb the unit to return a
  WRONG value instead. Both artifacts are committed, and the degenerate one is
  labelled as such.

- **A unit's result can be consumed by a line that runs, and still be outside
  what the gate measures.** Learned at unit #7; a FIFTH shape of P9, after
  unexercised (#1), constant argument (#3), cancelled-downstream (#4) and
  produced-but-never-consumed (#6).

  `GetRoot`'s output names a FILE. Five of its six reader sites are dead in all
  27 scenarios; the sixth runs 24 times and is
  `OPEN(unit=UnDb, FILE=TRIM(RootName)//'.RO.dbg')`. `gate.py` compares
  `baseline_arrays/scenario_N.npz`, which `vit_sim.py` builds from the arrays
  crossing the DLL boundary — it never opens a `.RO.dbg`. Perturbing the unit
  renames a file.

  So unit #6's "follow the OUTPUT, not the call" needs one more question after
  it: **does the live reader put the value somewhere the gate READS?** A reader
  that opens a file, writes a log, or sets a name is a live consumer and an
  invisible one.

  ```
  grep -n '<the out-argument>' rosco/controller/src/*.f90   # every reader
  python3.12 -c "import json;d=json.load(open('coverage/line_coverage.json'));\
      print({k: sum(d['hits']['<File>.f90'].get(k,{}).values()) for k in ['<lines>']})"
  grep -n 'npz\|savez\|arrays\[' Examples/vit_sim.py        # what the gate reads
  ```

- **A whole procedure can be the IDENTITY on the exercised domain, and then
  every instrument built on those inputs is measuring nothing.** Also unit #7,
  and it is the other half of that gate blindness.

  `GetRoot` strips a file extension. Every scenario hands it `vit_sim<N>`, which
  contains no `'.'`, so all 444,000 calls fall through to `RootName = GivenFil`.
  Coverage states it precisely: clean `ROSCO_Helpers.f90:1280` is evaluated
  4,304,000 times and **every assignment inside it — 1283, 1284, 1285, 1287,
  1290 — has zero hits**, as does the special case at 1270.

  The check is to read the coverage of the branch BODIES, not of the branch:

  ```
  python3.12 -c "import json;d=json.load(open('coverage/line_coverage.json'));\
      h=d['hits']['<File>.f90'];print([(l, sum(h[str(l)].values()) if str(l) in h else 0) \
      for l in range(<lo>,<hi>)])"
  ```

  A procedure whose every interesting leaf is 0 is one the differential harness
  and the mutation score have to carry alone — and see the harness-corpus entry
  below, because the generator may not reach those leaves either.

- **A green differential harness can mean the corpus never reached the branch
  the procedure exists for, and R6's own rule is what prevents it.** Unit #7's
  first harness reported `224 checked, 0 failed` and had not once executed
  `RootName = GivenFil(:I-1)` — extension stripping, the whole point of the
  unit. Three independent gaps, each found from a SURVIVING MUTANT and not from
  reading the generator:

  1. **The literal miner read single-character literals only, so a character SET
     was invisible.** `INDEX( '\/', ... )` is one two-character literal; the
     corpus contained **no backslash at all**. Fixed by mining every character of
     every literal handed to `INDEX`/`SCAN`/`VERIFY` — the three intrinsics whose
     argument is a set by definition, which is what keeps error messages out.
  2. **The corpus is each literal plus its COLLATING NEIGHBOURS, laid down in
     corpus order — so `'.'` was always followed by `'/'`, which is `'.'+1`.**
     The rule that makes the corpus relevant is the rule that blinded it: the
     character after a matched literal could only ever be that literal's own
     neighbour. Fixed by planting a corpus character at an interior position of
     an ordinary string, and by planting PAIRS of the reference's own literals.
  3. **The length ladder `{1, N, N+5}` has no 2.** Length 1 is degenerate — the
     first character is also the last — and at 4 both sides of an `I == 1` test
     are false together. `{1, 2, N, N+5}` now.

  224 cases / mutation 0.648 → 726 / 0.968. Fixed in the loop repo (`0e92a72`).
  **The verdict did not find any of this; the mutation score did.** A harness
  green is a claim about the cases that were generated, and the survivors are the
  only thing that says which cases those were.

- **A post-integration red test can stay green because the perturbation never
  reached the binary.** Unit #7, first attempt: `LEN(RootName)` was changed to
  `LEN(RootName) - 1` in the wrapper and `harness.sh --post-integration`
  reported `checked 726  failed 0`, which reads exactly like a harness that
  cannot fail. The harness links the campaign's **prebuilt Fortran objects**; it
  compiles the C++ test and nothing else. Rebuild the controller between the edit
  and the run:

  ```
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/rosco/controller/build && \
      cmake --build . -j4"
  ```

  With the rebuild it fails 596 of 726. Revert, rebuild AGAIN, and confirm green
  returns before believing either number.

- **`restore_integrated.sh` restores from HEAD, so run it before the unit's
  commit and you lose the unit's own wrapper.** Paid for once at unit #6: the
  script prints the warning, it was run anyway, and the result was an orphaned
  `getpath.cpp`, a library with no GetPath in it, and a gate artifact no longer
  reproducible from the tree. Every post-integration artifact taken before that
  point has to be DELETED and re-taken after `vit integrate ... --apply` is
  re-run — which is what the warning says. The existing rule covers it: commit
  before exercising the reset/restore pair.

- **Before believing a differential harness's green, ask WHICH FIELDS its red
  test named.** Learned at unit #5's third dispatch, and it is the entry that
  would have saved two dispatches if it had existed.

  `vit test-validate` decomposes a derived-type argument into one bridge
  parameter per field. It copied every field IN and copied back only fixed-size
  arrays, nested types and fixed-width CHARACTERs. A rank-1 **ALLOCATABLE**
  field and a **SCALAR** field of an INOUT argument were INPUT-ONLY -- so the
  reference wrote into a Fortran allocation the bridge then deallocated, and
  the two sides were never compared on that field at all.

  `ExtDLL%avrSWAP` is the whole of what `ExtController` produces.
  `ErrVar%ErrStat` is where ROSCO puts every error status. Both were invisible,
  and **every unit in this campaign taking a derived type was affected.**

  Fixed in VIT `8c34ceb`: the extent goes BY REFERENCE, because the callee
  CHOOSES it, plus a capacity; `n < 0` unallocated, `n == 0` allocated-empty,
  `n > 0` allocated (P6); over-capacity is refused and reported, never
  truncated. A SCALAR of an INOUT argument crosses by reference and is copied
  back.

  The check is one command and it belongs before believing any harness green:

  ```
  # replace the translation body with a no-op, re-run, and READ THE FIELD NAMES
  bash scripts/harness.sh <Unit> <Module> <stem> <file>.f90 \
       --out harness/<Unit>.redtest.json --red-test "the unit as a no-op"
  ```

  A no-op must fail every case AND the mismatch list must name every output the
  unit is supposed to write. `163/163 failed` says the harness can move; the
  LIST is what says it was ever looking at the right things.

- **`harness/ranges.toml` is where a judgement goes, and this campaign's first
  four entries are all about a reference that CRASHES on its own domain.**
  Created at unit #5's third dispatch; it did not exist before.

  `ExtController` never checks `ErrVar%ErrStat` after `LoadDynamicLib` and then
  calls through `DLL_Ext%ProcAddr(1)`. Any generated case that fails to load a
  library reaches a null function pointer, so the admissible domain really is
  narrow -- and pinning it is still a NARROWING of the input space, which is the
  blindness the generator exists to remove. So each entry carries the
  measurement that forces it, and the COST is written into `plan.json`'s
  `observability` rather than left to be discovered.

  Two mechanisms were needed and both are additions: `text = "..."` in the TOML
  (a stated string written as the string it is), and `lo = hi = -1` on an
  ALLOCATABLE field's extent, which is how the bridge is told the field arrives
  UNALLOCATED. A `values` list on a CHARACTER parameter is NOT an enumeration of
  alternatives -- three places in the generator read it as one, and one of them
  turned a pinned library path into 1024 semicolons.

- **`vit integrate --auto-allocate` does not work on this codebase, and the one
  ALLOCATE it should have generated is in the committed wrapper instead.**
  Learned at unit #5's third dispatch. Three defects, artifact kept at
  `evidence/ExtController/vit_defects/integrate_auto_allocate.wrapper.f90`:

  1. Its copy-call scan matches **any** `CALL x(..., arg%field, ...)`, not just
     Registry Copy calls as its docstring says. It hoisted BOTH
     `LoadDynamicLib` and the external DLL call into the wrapper -- each would
     have run a second time.
  2. A size expression that is a local PARAMETER classifies `local` and is
     declined.
  3. Its error handling emits `SetErrStat(...)` and `CHARACTER(ErrMsgLen)`,
     OpenFAST names that exist NOWHERE in ROSCO. The generated wrapper cannot
     compile here at all, so the path has never been exercised outside OpenFAST.

  Read the dry run before applying, and read it for the WRAPPER, not just the
  file list:

  ```
  vit integrate <Unit> <cpp> -f <file>.f90 --reverse-copy --auto-allocate --dry-run
  ```

- **`vit integrate` also writes `verification: simulation` into `vit.yaml`**,
  having run no simulation and read no result. It is the same shape as the
  `// After verification: <name> kernel PASSED` comment `37f8bdf` deleted from
  every generated file, one file over. Remove it, or leave it only where it is
  true. Nothing in the toolchain checks a claim a generator makes about
  verification.

- **A REFERENCE THAT PRINTS breaks three different readers of stdout.**
  `ExtController` has five `PRINT *` statements. `vit_harness.py` and
  `vit_mutate.py` both did `json.loads(run.stdout)`, and `harness.sh`'s post
  mode did `./test > $OUT`. A clean 163-of-163 run therefore reported "harness
  produced no JSON", and `vit_mutate.py` reported the BASELINE as a crash. The
  second is the dangerous one: had its baseline check not refused to score,
  every mutant would have scored `killed (no compile)`/`killed (crash)` -- a
  1.000 that measured nothing. All three now take the last parseable JSON
  object in the stream.

- **After integrating a unit that takes a derived type, the harness's generated
  LIBS list is STALE.** `vit test-validate` derives LIBS from the CMake target
  as it stood BEFORE integration, and `vit integrate` then adds one
  view-populator `.f90` per type -- four, for `ExtController`. The
  post-integration link dies on eight undefined `__vit_*_view_MOD_vit_populate_*`
  symbols, in the wrapper the run exists to measure. `harness.sh` now repairs it
  by CONTENTS (every object CMake built for this target that is not already
  there) rather than by a list, the same shape as `reset_to_clean.sh`'s "remove
  any object that DEFINES kgen symbols". Every earlier unit took scalars, arrays
  and strings, so integration added only the `.cpp`, which the existing drop
  removes anyway.

- **Two things about a Fortran original that must be MEASURED and not read.**
  Both found at unit #5's third dispatch, both transcribed rather than corrected
  (P7), and both would have produced a plausible wrong answer.

  1. `LEN` of a CHARACTER ARRAY is the ELEMENT length, not the array size.
     `ExtDLL%avrSWAP(49) = LEN(avcMSG) + 1` where
     `CHARACTER(KIND=C_CHAR) :: avcMSG(LEN(ErrVar%ErrMsg)+1)` is therefore the
     **constant 2**, not the size of the message buffer the record's own comment
     claims. `evidence/ExtController/len_probe.f90`.
  2. `TRANSFER(SOURCE, MOLD)` with an ARRAY mold returns an array just large
     enough for SOURCE -- ONE element for one character -- and gfortran assigns
     a nonconforming right-hand side by copying min(size) elements. So
     `avcMSG = TRANSFER(C_NULL_CHAR, avcMSG)` writes byte 1 and leaves the rest
     INDETERMINATE. `evidence/ExtController/transfer_probe.f90` prints
     `0 90 90 ...`.

  The second has a consequence for the ORACLE, not just for the translation: a
  fixture that folded those bytes into its answer would make the two sides of
  the comparison disagree for a reason about neither implementation. Extend a
  stub to read an input ONLY where the input is defined.

- **A survivor can be a blindness in the ORACLE, and then the oracle is what to
  change.** Third move at unit #5, alongside REMOVE-the-restatement and
  DECLARE-with-a-reason. `int aviFAIL = 0;` was write-only -- the local is
  handed to the external library and never read again -- so no mutation of it
  could be caught. The stub now reports the INCOMING value back in an output
  record before overwriting it, and the mutant dies on every case. Before
  declaring a survivor equivalent, ask whether the instrument could see it if
  the instrument were better.

- **A unit whose original CRASHES is not a unit with no oracle. Ask what it
  crashed ON.** Learned at unit #5's second dispatch, and it reverses that
  unit's first conclusion.

  The measurement was right: `Ext_Mode = 1` on this campaign's own inputs makes
  the original Fortran die on signal 11. The reading -- "no runnable oracle
  exists anywhere in this campaign" -- was one step too far. The crash is a
  property of the INPUT. `DLL_FileName` is the literal string `"unused"` in all
  14 `Examples/*.IN`, no external Bladed-style library was shipped anywhere in
  the tree, `dlopen` fails, and `ExtController` does not check
  `ErrVar%ErrStat`. Ship a library that loads and every step succeeds:

  ```
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
      bash fixtures/bladed_stub/build.sh && \
      python3 evidence/ExtController/probe_ext_mode_1_with_oracle.py"
  ```
  `exit_status: 0`, `returned_normally: true`. 60 lines of C.

  **And building it is not a verification-default change.** That was the other
  half of the wrong conclusion. Constructing an oracle sounds like adding a gate
  scenario, which would move the compared count and the baseline set and so
  belongs to the Driver (SPEC 8.4). The DIFFERENTIAL HARNESS DOES NOT RUN
  SCENARIOS -- it calls the unit directly -- so a fixture it links against is an
  ADDITION under P5 and touches nothing the gate measures. Before escalating
  "an oracle must be constructed", ask which instrument needs it.

  The probe is an ADDITION beside `probe_ext_mode_1.py`, not an edit of it. The
  SIGSEGV artifact is still true of the campaign's own inputs and the two are
  meant to be read together.

- **`vit test-validate` emitted a bridge no compiler could read, for MOST OF
  THIS CAMPAIGN, in silence.** Learned at unit #5's second dispatch and worth
  checking before any unit that takes a derived type.

  Decomposing a derived type emits one bridge dummy per FIELD.
  `ControlParameters` has 214 and `LocalVariables` 168, so the `SUBROUTINE`
  statement came out **11,747 characters long on one line**. Free-form Fortran
  stops at 132 columns; gfortran truncated it and then reported 1,153
  diagnostics, the first of which is `Unexpected junk in formal argument list`
  and none of which name the cause.

  So every unit here taking either type -- most of ROSCO's controllers -- was
  outside `vit test-validate` entirely, and P11 and P12 are mandatory for all of
  them. The first four units took scalars, arrays and strings and never touched
  a derived type, which is the only reason it went unmeasured this long.

  Fixed in VIT `83d25f9`. The check is one line and belongs before believing any
  generated Fortran:

  ```
  awk '{ if (length($0)>132) c++ } END { print "over 132: " c+0 }' <generated>.f90
  ```

  **Wrapping only the dummy list was not enough**, measured rather than
  predicted: the array copy-in statements were still 133-153 columns, one
  statement holding two decomposed field names, neither shortenable. A fix that
  names the sites emitting long lines is a fix the next site escapes.

- **A `.gitignore` pattern broad enough to catch a directory is broad enough to
  catch a file somebody needs.** `.gitignore:65` is `*build*`, which silently
  swallowed `fixtures/bladed_stub/build.sh` -- so the first commit of the oracle
  fixture added the `.c` and left the script that builds it untracked, leaving
  an evidence file naming a library nothing committed could produce (K3). `git
  status` was clean throughout. Same shape as the gitignored `Examples/DISCON.IN`
  that let a crashed probe reconfigure the gate. After committing any new
  fixture, run `git check-ignore -v <each file>` rather than trusting a clean
  `git status`.

- **`coverage/line_coverage.json` cannot tell "never ran" from "never
  instrumented", and four files read the same either way.** `scripts/coverage.py`
  stores only lines with a NON-ZERO hit count, so a fully-dead file is an empty
  dictionary — as are `Constants.f90`, `ROSCO_Types.f90` and `ZeroMQInterface.f90`
  today. The per-scenario log line prints `N/M`; the artifact drops `M`.

  So before calling a unit dead from the committed artifact, regenerate it for
  that file and read the denominator:

  ```
  python3.12 scripts/coverage.py --files <File>.f90,Functions.f90 \
      --out /tmp/<unit>_coverage.json          # then read the printed N/M
  docker exec vit-dev bash -lc "rm -rf /workspace/ROSCO-r2/rosco/controller/build_cov"
  bash scripts/reset_to_clean.sh
  ```

  `ExtControl.f90` came back `0/28` on all 27, which is a measurement;
  "no entry in the JSON" was not. Keep `Functions.f90` in the `--files` list as a
  live control — a run where BOTH are 0/0 is an instrumentation failure, not a
  finding.

- **VIT can emit a wrapper with more arguments than its bridge, and until unit #5
  it did so in silence.** With no `strategy: view` configured for a derived type,
  `vit interface` accepts the argument in the wrapper's dummy list and does not
  forward it. On `ExtController` that produced
  `CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(ExtDLL_view))` —
  three of five — dropping `LocalVar`, whose `%iStatus` is the guard on the whole
  initialisation branch. It compiles, it links, and nothing said a word.

  Fixed in VIT `f8ab74f`: `generate_fortran_wrapper` now prints what it dropped,
  so every path that ships a wrapper reports it. **Read the wrapper anyway** —
  that is what caught this one, and the habit from unit #1 is the thing that
  generalises past whatever the tool currently checks.

  Two follow-ons measured here, both worth having before the next derived-type
  unit:
  1. `vit translate` AUTO-ADDS `strategy: view` to `vit.yaml` for any type with
     ALLOCATABLE fields, then may still refuse for a different reason. It also
     rewrites the file and strips every comment, like `verify` and `integrate`.
  2. Once the strategy IS configured, the wrapper `vit interface` emits `USE`s a
     view module that `vit translate` may be unable to generate — so the tool has
     two answers and neither is a working bridge. `ErrorVariables` is that case:
     `CHARACTER(:), ALLOCATABLE :: ErrMsg`, refused, and **37 of the 69 units take
     a `TYPE(ErrorVariables)` dummy**.

- **`cp` onto a bind-mounted file can be read half-written from inside the
  container.** Restoring `vit.yaml` on the Mac and immediately running a `vit`
  command produced a YAML parse traceback from a file that parses fine on both
  sides a second later. Re-run before believing a config error; if it repeats,
  it is real.

- **A kernel PASS *is* a bit-identity claim for a CHARACTER output, and this is
  a per-TYPE fact, not a per-unit one.** Unit #3 recorded that KGen falls back
  to an ABSOLUTE RMS against `1.D-14` for a REAL array. Read the generated
  comparison again for each new element type: for `CHARACTER(len)` KGen emits

  ```
  IF (var == kgenref_var) -> IDENTICAL  ELSE -> OUT_TOL
  ```

  with no `rmsdiff`, no tolerance and no `IN_TOL` branch at all. Quoting the
  status counts is still the right habit; the point is that the caveat belongs
  to REAL fields, and repeating it about a CHARACTER field would be hedging
  about an instrument that is exact.

- **`vit verify` will report `NON_DISCRIMINATING` for a unit with no by-value
  floating-point argument, and it is right to.** Its automatic red test
  perturbs such an argument; a CHARACTER, array or pointer-only signature has
  none, so it declines to construct one and says the kernel's discriminating
  power is UNMEASURED. That verdict is not a failure of the translation and
  must not be argued away — run the zero/no-op stub by hand, record the count,
  and keep VIT's line in the evidence beside it.

- **A CHARACTER argument crosses, and until unit #4 the HARNESS did not.**
  `vit interface` emits `CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: Str(*)` plus
  `INTEGER(C_INT), VALUE :: len_Str`, with copy-in/copy-out in the wrapper, and
  the wrapper declares the dummy exactly as the original does. What failed was
  the differential harness: `harness/generate.py` had kinds real / int / real[]
  and no string, so `vit_harness.py` reported the argument
  comparable-but-held-constant and then died on
  `C parameter 'Str' is not in the mapped signature`.

  **20 of this campaign's 69 units take a CHARACTER dummy argument**, and P11
  and P12 are mandatory for every unit, so none of the 20 could close. Fixed in
  the loop repo (`9bee569`), not worked around. Two things to carry:

  1. `len_<x>` is an EXTENT. It was falling through to the scalar branch and
     being varied over ±1e3 — two implementations reading that many bytes from
     a buffer sized by a different number.
  2. A string needs THREE SHAPES, not just three lengths: mixed,
     word-then-blanks, and all-blank. `LEN_TRIM` and `LEN` are the same number
     on a string with no trailing blank in it, so without a padded case a unit
     that trims cannot be told from one that does not.

  Still REFUSED, for being unmeasured: a CHARACTER **array** dummy
  (`FileLines(:)`), whose element width and element count are two extents where
  `char[]` has one. `FindLine`, `ParseInAry_Opt` and the `ParseInput_*_Opt`
  family all take one, so that is the next thing this feature will need.

- **Removing a restatement raised a mutation score from 0.696 to 1.000, for the
  second time.** Unit #1 named a SIZE once; unit #4 named a LOOP BOUND once.
  `Conv2UC`'s Fortran reads `DO IC=1,LEN_TRIM(Str)`, and transcribing LEN_TRIM
  literally adds six mutable sites computing a quantity nothing downstream can
  read — every character past `LEN_TRIM` is a blank, and a blank is never
  converted, so the trimmed bound and the full length agree on every input. All
  six survived, five of them as out-of-bounds reads no value comparison can
  see.

  The test to apply before declaring a survivor equivalent: **can any input
  make this quantity change an output?** If not, the sites are not equivalent
  mutants, they are unobservable ones — and the fix is to delete the
  restatement and write the proof in the translation, not to declare the
  blindness away. A departure from literal transcription needs that proof in
  the file; without one, transcribe literally.

- **`vit check -f <file>` scopes its cross-source checks to the FILE, not the
  function**, and `--function` only sets the report header. On `Functions.f90` it
  reported `minval-endpoints` and `array-section-row` against a 6-line
  translation containing neither — both findings came from `interp1d`/`interp2d`
  hundreds of lines away. Re-attribute every finding to the unit's own line range
  before acting on it, or the checker teaches you to ignore it.

- **Before writing any C++, ask whether the SIGNATURE crosses — and ask the
  generator that ships, not the one the matrix measures.** Learned at unit #1.

  ```
  vit interface <Unit> -f <file> -o /tmp/<unit>_iface     # read the wrapper it emits
  ```

  Compare the emitted wrapper's dummy declarations against the original's,
  attribute by attribute. `AddToList`'s `list` is
  `dimension(:), allocatable, intent(inout)`; the wrapper declared
  `INTEGER(4), INTENT(INOUT) :: list(:)`. **VIT parsed `is_allocatable=True` and
  the generator dropped it with no diagnostic**, producing a bridge that
  compiles and cannot do the one thing the function does.

  VIT first REFUSED that case (`UnbridgeableSignature`, exit 1, nothing
  written) and now GENERATES it correctly — see the descriptor entry below.
  The habit is what generalises, and it is the reason this entry stays: an
  attribute missing from the generated wrapper is a silent semantic change, and
  the build cannot catch it. Read the wrapper, whatever the tool's current
  answer is.

  **`plan.json`'s `bridge_feasible` is not the answer to this question.** Its
  basis strings come from VIT's conformance matrix, whose `bridge`/`compiles`
  columns measured `test_validate.generate_fortran_bridge` — the differential
  harness's Fortran side — while `vit integrate` ships `interface_gen`'s output.
  Those are different code paths and 5 of 39 cells disagree. The matrix now has
  an `integrates` column; the plan was derived before it existed.

- **An ALLOCATABLE INTENT(INOUT) dummy crosses. VIT generates the descriptor
  bridge as of `37f8bdf`; the loop's harness understands it as of `cf885e3`.**
  Learned at unit #1's second dispatch, replacing the refusal the first
  dispatch installed.

  The interface declares the dummy exactly as the original does, emits NO
  extent parameter, and the C++ takes `CFI_cdesc_t*`. Two things to carry into
  the next such unit, both measured here:

  1. **Write the body the way the Fortran writes it.** `do i=1,isize`
     transcribed as a 1-based C loop makes the boundary mutant observable
     (`<=` -> `<` leaves an element uncopied); the same loop written 0-based
     makes it invisible (`<` -> `<=` writes a slot the next statement
     overwrites). The literal transcription is the observable one.
  2. **Name a size once.** `SIZE(clist)` restated in the allocation, the new
     upper bound and the copy length left two of the three sites unobservable,
     and each surviving mutant there was a memory error no value comparison can
     see. One name, one site, and that site decides the extent the caller sees.

  Still refused, each for being UNMEASURED: a scalar allocatable, rank >= 2, a
  CHARACTER or derived-type element, LOGICAL and COMPLEX. The exception says
  which.

- **A dead unit can still close — on the harness and the mutation score, not
  on the gate.** `AddToList` is `integrated` while the gate is blind to it. The
  order that works:

  ```
  vit translate -> write -> harness (pre, against the CLEAN Fortran) ->
  mutation -> vit integrate -> rebuild -> gate + gate red test ->
  harness --post-integration -> commit
  ```

  **The pre-integration harness and the mutation score must be taken against a
  CLEAN tree.** Both link the campaign's Fortran objects, and after integration
  `ROSCO_Helpers.f90.o` IS the wrapper — there is no independent reference left,
  and the link fails outright once `harness.sh` drops this unit's own `.cpp.o`.
  If either has to be re-run later (a stamp correction, say), it is
  `reset_to_clean.sh` -> re-run -> `restore_integrated.sh` -> rebuild, and the
  post-integration pair re-run after that.

- **`vit integrate` used to write `// After verification: <name> kernel PASSED`
  into every file it generated**, having run no kernel and read no kernel
  result. Fixed in `37f8bdf`. It is worth reading a generated artifact for
  claims like that: nothing in the toolchain checks a comment.

- **A unit the gate cannot see is not a unit the gate passed.** Two checks, and
  the second is the one that cannot be argued with:

  ```
  python3.12 -c "import json;d=json.load(open('coverage/line_coverage.json'));\
      print(sum(sum(v.values()) for k,v in d['hits']['<File>.f90'].items() if int(k) in range(<lo>,<hi>)))"
  python3.12 scripts/gate.py <Unit> --perturb-file <file> \
      --perturb-from '<a line the body certainly runs>' --perturb-to '<perturbed>'
  ```

  For `AddToList`, perturbing BOTH branches of the body moved **0 of 5,252,000**
  values. The gate's green for that unit compares 5,252,000 values and
  constrains none of them. Commit the pair — green and failed red test — never
  the green alone.

  On a `--perturb-*` run `gate.py` writes `verdict: RED_TEST_PASS` or
  `RED_TEST_FAIL` and keeps the comparison's own verdict under
  `comparison_verdict`. Artifacts written before unit #1 carry the old spelling
  (`PASS`/`FAIL`), which is INVERTED relative to the run's meaning; `went_red`
  means the same thing in both and is the field to read on any redtest file.

- **Coverage:** WORKS, 2026-08-10. This is C2's input; there was none before.
  ```
  python3 scripts/coverage.py --out coverage/line_coverage.json
  python3 scripts/coverage.py --no-build --files Functions.f90,Controllers.f90
  ```
  Builds `--coverage -O0` in `rosco/controller/build_cov` (its own directory, so
  the Release build the gate measures is untouched), runs each of the 27
  scenarios with the counters cleared, gcovs after each. ~6 min for all 12
  sources. Remove `build_cov` and run `reset_to_clean.sh` afterwards.

  Two things it taught immediately, both by measurement:

  1. All five `AddToList` call sites are dead in all 27 scenarios. That is the
     whole of the "extraction is broken" blocker.
  2. Scenarios 10, 14 and 24 execute **no controller code at all**, and 13
     executes 12 lines. They enter DISCON, read parameters, and stop. See
     STATUS.md; this is E3.3's failure mode, measured.

  `gcov -o <dir> Functions.f90` does NOT work: gcov strips the extension, looks
  for `Functions.gcno`, prints "No executable lines" and **exits 0**. Name the
  notes file: `gcov <dir>/Functions.f90.gcno`. The first run of this script
  reported 0/185 lines for all 27 scenarios and looked like a clean answer;
  printing the denominator next to the numerator is what showed it.

- **Choosing the invocation window:** hit counts are not enough.
  `vit.yaml`'s `kgen.invocation` at setup is `0:0:1-20`, which for a call site
  in the control loop means the first 21 timesteps -- simulation start, where
  most state is still zero. For `ColemanTransform` every captured case had
  `Azimuth = 0` and `rootMOOPF = 0`, and a translation reading none of its
  inputs and writing `0.0` passed **21/21 with 4725/4725 IDENTICAL**.

  Widening the window on the same scenario did not help: scenario 6 is a 1-DOF
  sim that never drives azimuth or blade root moments, so invocations 2000-2020
  and 3900-3920 are all-zero too. **The line ran 7,998 times on zeros.**

  So the check is on the CAPTURED VALUES, not on the counts:

  ```
  python3 - <<'EOF'
  import csv, collections
  rows = list(csv.DictReader(open('kernel/<Unit>/verify_fields.csv')))
  print(collections.Counter(r['status'] for r in rows))
  for fld in ('<the fields the unit writes>',):
      v = [float(r['reference']) for r in rows if r['field'] == fld]
      print(fld, sum(1 for x in v if x != 0.0), '/', len(v), 'non-zero')
  EOF
  ```

  A field that is 0.0 in every reference case is a field the kernel cannot see.
  Then run the stub test below; if the stub passes, the window is vacuous
  whatever it says.

  **THAT RECIPE ONLY WORKS FOR SCALAR OUTPUTS.** Learned at unit #3. VIT logs a
  scalar field with its computed and reference values; it logs an ARRAY field as
  ONE row with BOTH value columns EMPTY:

  ```
  ColemanTransformInverse.0.0.1,pitcomipc_1p,array,IDENTICAL,,,size=           3
  ```

  So for a unit whose outputs are arrays the loop above yields nothing at all —
  it does not error, it produces an empty answer that reads like "no zeros
  found". For such a unit the stub test below is not a second opinion, it is the
  only one. Check `type` in the field log before trusting the recipe.

- **A kernel `✓ VERIFICATION PASSED` is NOT a bit-identity claim.** Read out of
  the generated comparison at unit #3, not assumed. For an array field KGen
  generates:

  ```
  IF (ALL(var == kgenref_var)) -> IDENTICAL
  ELSE rmsdiff = SQRT(SUM((var-ref)**2)/n)
       IF (rmsdiff > kgen_tolerance) -> OUT_TOL  ELSE -> IN_TOL
  ```

  `kgen_tolerance` is `1.D-14` and `rmsdiff` is ABSOLUTE. A unit whose outputs
  are of order 1e-3 can be wrong at ~1e-11 relative, score `IN_TOL`, leave
  `numOutTol` at 0, and print `63/63 passed`. **The bit-exact claim lives in the
  field log's `status` column (`IDENTICAL`), not in the verdict line.** Quote the
  status counts in evidence, never the verdict alone.

- **Red-test every green on first use, including the tool's own.** For a kernel:
  replace the translation body with one that reads no argument and writes
  constants, re-run `vit verify`, confirm it FAILS, restore. Two minutes, and it
  is the only thing that distinguishes `62/62 IDENTICAL` from `62/62 IDENTICAL
  on inputs that were all zero`. Keep the passing-stub artifact when you find
  one -- `evidence/<Unit>/`.

- **The done-condition:** RUNNABLE HERE, 2026-08-10. Run it before believing a
  unit is finished, and again before the state commit.
  ```
  python3.12 scripts/done_check.py <Unit>          # exit 0 only on COMPLETE
  ```
  It imports the loop's own `DoneVerifier` with exactly the config
  `scripts/run_campaign.py` builds, and prints all 13 predicates with reasons.
  It reads; it never writes.

  **This is what unit #2 was re-dispatched for.** The first pass wrote
  `disposition: integrated` while the condition stood at 10 of 13 — not because
  the work was bad but because nothing in the session could ask. A disposition
  set without running this is a prediction of the verdict, and predictions of
  green have been wrong here before.

  The two predicates a session most often gets wrong on its own:
  `P2 dirty_tree` (any untracked file counts, including one you just wrote) and
  `P12`, which reads `mutation/<Unit>.json` and needs the keys `mutants` and
  `equivalent_declared`.

- **Differential harness (P11) and mutation score (P12):** WORK, 2026-08-10.
  ```
  bash scripts/harness.sh <Unit> <Module> <stem> rosco/controller/src/<File>.f90 \
       --against translation --out harness/<Unit>.json
  bash scripts/harness.sh <Unit> <Module> <stem> rosco/controller/src/<File>.f90 \
       --post-integration --out harness/<Unit>.postintegration.json
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
      python3 /workspace/translation-loop/scripts/vit_mutate.py <Unit> \
        --root /workspace/ROSCO-r2 --cpp translations/<Module>/<stem>.cpp \
        --module <Module> --out mutation/<Unit>.json"
  ```
  `--red-test` works in BOTH modes as of unit #4. It used to be **accepted and
  silently dropped in pre mode** — that path returns before the stamping block
  — so a pre-integration red test wrote an artifact saying `failed: 27` and
  nothing about what was perturbed, indistinguishable from a run that failed by
  accident. It now stamps with `--pre`, which adds the `red_test` record and
  none of post mode's fields.

  Red-test the post-integration run with the same script, which records the
  perturbation into the artifact and inverts the verdict:
  ```
  bash scripts/harness.sh <Unit> <Module> <stem> rosco/controller/src/<File>.f90 \
       --post-integration --out harness/<Unit>.postintegration.redtest.json \
       --red-test "wrapper: the two INTENT(OUT) arguments swapped"
  ```

  `scripts/harness.sh` rather than `vit_harness.py` directly. It now does four
  things, three of them learned by a run failing on 2026-08-10:

  1. Handles BOTH test-directory layouts and prints which one it used. VIT
     `d07a716` writes to `translations/<Module>/<stem>_test/`, the same place
     `vit_harness.py` looks; the pre-merge VIT wrote one level up. The skew the
     script was written to reconcile has closed.
  2. **Drops this unit's own `<stem>.cpp.o` from the generated LIBS.** After a
     unit has been integrated once, `reset_to_clean.sh` leaves CMakeLists
     integrated, so the build tree holds a compiled copy of the same function
     the harness compiles itself: `multiple definition of ...`, link dead. This
     hits EVERY unit from #2 onward and could not have happened on the first
     one. The edit goes into the Makefile, not onto a `make` command line,
     because `vit_mutate.py` runs `make` itself.
  3. Asks `make` what LIBS is instead of grepping `^LIBS =`. The new VIT's
     Makefile ends with a second `LIBS +=` assignment, and the old grep handed
     the literal words `LIBS` and `+=` to the linker.
  4. Stamps the artifact even when the run went RED. Under `set -e` a non-zero
     `./test` aborted one line short of the stamp, so a red artifact carried
     `against: "translation"` and no revisions.

  ColemanTransform, second pass (VIT `d07a716`, loop `ebce989`): 199
  differential cases 0 failed; 35/35 mutants killed, score 1.000; 199/199
  post-integration; the wrapper-swap red test 199 of 199 failed.

  **Check the artifact's `loop_rev`/`vit_rev` before believing its numbers.** A
  run that fails to write leaves the PREVIOUS run's artifact in place, and it
  reads exactly like a fresh pass. Three runs today failed and two were read as
  successes off a stale file; the stamp naming a revision that no longer exists
  is what exposed it.

  Case counts differ between passes and that is expected: post-integration
  reuses whatever case file the last generating run left, and the literals it
  draws from differ before and after integration (217 first pass, 199 second).
  The artifact records what it actually checked.

  **`--post-integration` measures the WRAPPER, not the arithmetic.** After
  integration the Fortran body IS the translation, so both sides run the same
  code by construction. A failure means the marshalling corrupted an argument --
  which is worth checking, since two generated bridges in this campaign dropped
  an array's rank. Red-test it by swapping the wrapper's two INTENT(OUT)
  arguments; it must fail every case.

  **A `killed (no compile)` mutant is not a behavioural kill.** ColemanTransform
  scores 35/35, but 2 of those are `compare_op` mutants on a translation
  containing no comparison, killed by the build failing. Counting them is
  honest; reading 35/35 as 35 wrong implementations caught is not. Report the
  behavioural count beside the score.

  **The mutation score is MANDATORY for every unit, not just `respecify`.**
  `done.py:344` returns `_mutation(...)` unconditionally when `--mutation-glob`
  is set -- the red-test fallback exists only when it is UNSET -- and
  `min_mutation_score` is 1.0 and is never overridden. Do not unset the flag to
  make a unit close: that converts a hard requirement into a red test and makes
  that unit's evidence permanently weaker than every other unit's, silently.

- **`vit verify` and `vit integrate` rewrite `vit.yaml` and delete every
  comment.** Observed three times on 2026-08-10. The file is declared `derive`
  and its provenance lives entirely in those comments. Restore them from
  `git show HEAD:vit.yaml` after the last VIT command of the unit, keeping VIT's
  own `translations:` block, and check `yaml.safe_load` still parses it.

  **"Keeping VIT's own `translations:` block" does NOT hold for a STUB run.**
  Unit #11, second dispatch. A `vit verify` run on the hardcoded-`CornerFreq`
  stub wrote `cases_passed: 62`, `red_test: demonstrated` under
  `translations.LPFilter` — a machine-readable claim, in the config every later
  unit reads, that the SHIPPED translation was verified by a run of something
  else. When the verify you just ran was on a stub, `git checkout -- vit.yaml`
  wholesale; there is nothing in the new block worth keeping.

- **A stub `.cpp` committed as evidence is the INPUT to a measurement, not the
  measurement — and a unit's evidence directory can look complete while the
  measurement is missing.** Unit #11, and `done_check.py` is what found it, not a
  reader: `P5:unresolved_evidence` named a `verify_fields.csv` that `plan.json`
  and the evidence README both described in prose and neither had produced. The
  hard-to-write half of the artifact (the stub) was there, which is exactly why
  it read as done.

  **Re-running a kernel verify after integration needs no source reset.** The
  kernel directory carries its own generated Fortran, and `-f` supplies only the
  signature, so a gitignored copy of the pre-integration file is enough:

  ```
  mkdir -p .vit && git show HEAD~1:rosco/controller/src/Filters.f90 > .vit/Filters.clean.f90
  cp translations/<Module>/<unit>.cpp /tmp/<unit>.intended.cpp        # the stub overwrites it
  cp evidence/<Unit>/<the-stub>.cpp translations/<Module>/<unit>.cpp
  vit verify <Unit> translations/<Module>/<unit>.cpp \
      -f .vit/Filters.clean.f90 --kernel-dir kernel/<Unit>
  cp /tmp/<unit>.intended.cpp translations/<Module>/<unit>.cpp        # restore, then diff
  git checkout -- vit.yaml
  ```

  Three minutes, and it turns "delete the unsupported claim" into "run it". Here
  the claim held AND got sharper: the stub passes 14,508 of 14,508, and the
  reason is not the one the prose gave. `CornerFreq` is read only in the
  initialisation branch and **exactly 1 of the 62 captured cases has
  `istatus == 0`**, so 61 cases never read the argument at all. *Ask how many
  cases REACH the branch that reads the argument before calling a
  constant-argument stub's pass a constant-argument blindness* — they produce the
  same green and they are not the same weakness:

  ```
  python3.12 -c "import csv,collections; \
    r=list(csv.DictReader(open('evidence/<Unit>/<the-stub-csv>'))); \
    d=collections.defaultdict(dict); [d[x['case']].__setitem__(x['field'],x['reference']) for x in r]; \
    print(sum(1 for v in d.values() if v.get('istatus')=='0'), 'of', len(d))"
  ```

- **Reset to clean source:** WORKS, 2026-08-10.
  ```
  bash scripts/reset_to_clean.sh              # sources + leftovers + rebuild + verify
  bash scripts/reset_to_clean.sh --no-build   # sources only
  bash scripts/restore_integrated.sh          # put the wrappers back
  ```
  **The pair must be used together.** Leaving the tree clean and then gating
  builds a library with no C++ in it at all, which passes 27/27 and means
  nothing.

  **Restoring the SOURCE is not enough, and the script could detect that
  before it could repair it.** At unit #4 the reset reported success on every
  step and then exited 1: `kgen symbols in rosco/lib/libdiscon.so: 1`.
  `vit extract` restores its own pragma-carrying source when it finishes, so
  `ROSCO_Helpers.f90` came back textually clean, step 1 skipped it (no `_c(`
  wrapper to revert) and its mtime never moved — and make, seeing an object
  newer than its source, did not recompile the INSTRUMENTED
  `ROSCO_Helpers.f90.o`. Step 3c now removes any object in the build tree that
  DEFINES kgen symbols, which is a rule about contents rather than a list of
  files and so covers whatever the next extraction instruments. Measured in
  both directions on the run that found it: 2 objects removed and the assertion
  went 1 → 0; the immediate re-run removed 0 and stayed at 0.

  It reverts more than the replication's version because `vit extract` on this
  tree damages more than wrappers. Five things, each measured here:

  1. **wrapper-carrying sources** back to the pinned clean baseline, selected by
     whether the file calls a `<name>_c(` bridge -- a rule that maintains itself
     as units are added. Files WITHOUT a wrapper are left at HEAD: some sources
     differ for reasons that are not integrations, and reverting those breaks
     the kernel.
  2. **files whose ONLY change is line endings.** DISCON.F90 comes back
     CRLF-stripped from every extraction, including `--dry-run`. Narrow on
     purpose: reverting any file that merely DIFFERS would destroy in-flight
     work, but a change with no content in it cannot lose any.
  3. **extraction leftovers** -- `kernel/ state/ model/ elapsedtime/`, the logs,
     `.kgen_org` backups, `src/kgen_utils.f90`, and CMakeLists' `kgen_utils`
     patch plus its `.vit_backup`. Note `make recover` in `state/` does NOT do
     this: it restores the PRAGMA-CARRYING source and leaves the rest.
  4. **rebuild, install, and ASSERT the installed library carries no kgen
     symbols**, exiting non-zero if it does. Extraction installs an instrumented
     `libdiscon.so` into `rosco/lib/`; any gate run afterwards measures the
     instrumented build, and nothing announces it.

  Red-tested 2026-08-10 against synthetic damage rather than waiting for a real
  integration: a planted `synthetic_c(` wrapper is reverted, a planted
  non-wrapper edit SURVIVES, CRLF is restored to 162 lines, every leftover is
  removed, and a deliberately instrumented `libdiscon.so` makes the script exit
  1. 10 of 10 assertions.

  **The verification had a bug of its own, worth keeping.** `grep -c` exits 1 on
  a count of zero, so an outer `|| echo "?"` appended a second line to a correct
  `0` and the check reported FAILED on a clean library. Wrong in the safe
  direction, but a verification that misreports is not a verification. `|| true`
  belongs inside the container command.

### Two upstream ROSCO bugs must be fixed before the gate can run

Scenario 4 (`Flp_Mode=2`) segfaulted on the pristine build -- exit 139, core
dumped, reproducible. Both causes are **defects in upstream ROSCO**, not in the
harness or this setup, and both were fixed here on 2026-08-10:

1. **`Filters.f90`, `PreFilterMeasuredSignals`** -- the `Flp_Mode==2` branch
   indexes `CntrPar%F_GenSpdNotch_Ind(n)` with a loop counter `n` left over from
   an earlier `DO n = 1,CntrPar%F_GenSpdNotch_N`. With `F_GenSpdNotch_N=0` that
   loop never runs and `n` is never assigned, so the index reads undefined
   memory. Fix: wrap the NotchFilter call in the same loop, which is skipped
   when there are no notch filters.

2. **`Controllers.f90`, `FlapControl`** -- the `Flp_Mode==2` initialisation
   block (`iStatus==0`) indexes `LocalVar%Flp_Angle(K)` with an unassigned local
   `K`. The steady-state block a few lines below loops `DO K = 1,LocalVar%NumBl`
   correctly. Fix: the same loop around the initialisation call.

Both were introduced in ROSCO v2.9.0 and are documented in the previous
campaign's `ROSCO_BUG_REPORT.md`. `Flp_Mode=2` appears to be exercised by no
test in the ROSCO repository, which is why they survived.

After both fixes, `--scenario 4` exits 0 and the full baseline captures 27/27.

**Two things worth keeping from how this was found.** Two candidate fixes
carried from the previous campaign were tried FIRST and neither helped -- the
FITPACK stack scrubber, and raising `avr_size` from 500 to 3000. Carrying a fix
across because it is nearby is not the same as diagnosing. And the real cause
was already written down in the previous campaign's own bug report; reading it
would have been faster than reproducing it.

## Do not background a build and poll for it

Run builds, gates and mutation runs in the **foreground**. Backgrounded work is
tracked and you are re-invoked when it finishes, so a sleep-loop waiting on its
output file buys nothing and spends the unit's budget doing it.

Never:

```bash
for i in $(seq 1 20); do
  if [ -s .../tasks/<id>.output ]; then break; fi
  sleep 15
done
```

**Measured on unit HPFilter.** It backgrounded the kernel build and then wrote
11 polling loops against the harness's own task-output file. The slowest
inter-call gaps cluster at 606, 608, 610, 610, 611 and 742 seconds -- `seq 1 60`
x `sleep 10` is exactly 600 -- so those loops ran to EXHAUSTION rather than
breaking early. Roughly 100 of its 120 minutes were spent asleep, and the 7200s
timeout killed it mid-cycle with its work uncommitted.

It was not a hard unit. Its 216 assistant turns sit beside GetRoot's 263 and
GetWords' 252, and it cost about $12.55 against GetWords' $40.20 -- it was
sleeping, not thinking. Cost and wall clock are not the same axis here, and a
unit can exhaust the second while barely touching the first.

## Finishing a unit

0. Before extracting: query `coverage/line_coverage.json` for the call site's
   hit counts AND check that the scenario actually drives the unit's inputs.
   A line that runs on zeros produces a kernel that cannot fail.
1. Red-test the gate for this unit; confirm it fails and writes its artifact.
2. Run the gate. Non-zero compared count, zero mismatched.
3. If this unit needed a purpose-built harness, **re-run it against the
   integrated build**, not only against the translation, and commit the result.
4. Score the mutants and confirm the artifact carries `mutants` and
   `equivalent_declared`. `mutation/<Unit>.json` is what P12 reads; the
   `total`/`equivalent` spelling is invisible to it.
5. Set `disposition` and `evidence` in `plan.json`. Evidence must name artifacts
   that exist.
6. Commit the translation and its gate artifacts.
7. Commit `STATUS.md`, `DECISIONS.md` and `plan.json` brought current. That
   second commit is the hand-off.
8. **Run `python3.12 scripts/done_check.py <Unit>` and read the verdict.** Not
   before the commits -- P2 requires a clean tree and P6/P7 require the commits
   to exist, so the condition can only be true at the end. If it is not
   COMPLETE, the unit is not finished no matter what the work looked like; fix
   what it names and amend. Unit #2 recorded `integrated` at 10 of 13 because
   this step did not exist.
