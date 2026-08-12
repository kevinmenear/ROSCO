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
