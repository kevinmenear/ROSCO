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

- **Red-test every green on first use, including the tool's own.** For a kernel:
  replace the translation body with one that reads no argument and writes
  constants, re-run `vit verify`, confirm it FAILS, restore. Two minutes, and it
  is the only thing that distinguishes `62/62 IDENTICAL` from `62/62 IDENTICAL
  on inputs that were all zero`. Keep the passing-stub artifact when you find
  one -- `evidence/<Unit>/`.

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
  `scripts/harness.sh` rather than `vit_harness.py` directly: the pinned loop
  repo writes to `translations/<Module>/<stem>_test/` and this workspace's VIT
  writes the Makefile and bridge to `translations/<Module>/`, so the raw command
  dies on `No rule to make target 'test'`. The script reconciles them and
  explains why VIT was not upgraded instead.

  ColemanTransform: 199 differential cases, 0 failed; 35/35 mutants killed,
  score 1.000; 217/217 post-integration.

  **`--post-integration` measures the WRAPPER, not the arithmetic.** After
  integration the Fortran body IS the translation, so both sides run the same
  code by construction. A failure means the marshalling corrupted an argument --
  which is worth checking, since two generated bridges in this campaign dropped
  an array's rank. Red-test it by swapping the wrapper's two INTENT(OUT)
  arguments; it must fail every case.

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

  It reverts more than the replication's version because `vit extract` on this
  tree damages more than wrappers. Four things, each measured here:

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
4. Set `disposition` and `evidence` in `plan.json`. Evidence must name artifacts
   that exist.
5. Commit the translation and its gate artifacts.
6. Commit `STATUS.md`, `DECISIONS.md` and `plan.json` brought current. That
   second commit is the hand-off.
