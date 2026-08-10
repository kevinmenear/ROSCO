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

  It does not yet print a compared count or exit non-zero on comparing nothing.
  That is an E3.1 exit criterion and is TODO.

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

- **Extract / capture:** TODO.

- **Reset to clean source:** not applicable yet. This tree has no integration
  wrappers, so there is nothing to reset. Once the first unit is integrated,
  this becomes a real requirement -- extraction needs clean Fortran.

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

1. Red-test the gate for this unit; confirm it fails and writes its artifact.
2. Run the gate. Non-zero compared count, zero mismatched.
3. If this unit needed a purpose-built harness, **re-run it against the
   integrated build**, not only against the translation, and commit the result.
4. Set `disposition` and `evidence` in `plan.json`. Evidence must name artifacts
   that exist.
5. Commit the translation and its gate artifacts.
6. Commit `STATUS.md`, `DECISIONS.md` and `plan.json` brought current. That
   second commit is the hand-off.
