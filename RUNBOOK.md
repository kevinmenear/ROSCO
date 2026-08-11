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
