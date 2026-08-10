# STATUS — rosco-r2

**This is the live state file. Read it first, then `RUNBOOK.md`.**
`DECISIONS.md` is the append-only record of *why*; this file is *where things
stand*. One copy of every count — do not duplicate them anywhere else.

**As of 2026-08-10, Phase 1 partly closed. No units attempted, and the first
unit should NOT be dispatched yet — see Open.** Next: the lowest-order unit in
`plan.json`. Confirm it against `plan.json` before starting, including its
`phase` and `proposed_verification` fields — those are hypotheses, not facts.

## Counts

0 attempted / 0 integrated / 0 integrated_unexercised / 0 out_of_scope /
0 deferred / 0 blocked.

## Evidenced

- **E3.1** — `scripts/gate.py` runs 27 scenarios, compares 5,252,000 values
  across 351 channels bit-for-bit against `baseline_arrays`, prints the count
  and persists `gate/__gate__.json`. Exits non-zero on mismatch and on comparing
  nothing.
- **E3.2** — gate observed red: perturbing `LPFilter`'s leading `1.0` moved
  1,634,643 of 5,252,000 values across 138 of 351 channels; reverting restored
  0. Identical counts on two separate runs, so it is deterministic.
  `gate/__gate__.redtest.json`.

Both were red-tested as criteria, not just satisfied: corrupting the expected
values in `phases.toml` turns both `[FAIL]`, and restoring them turns both
`[ok]`.

## Open

- **`AddToList` is unit #1 and its signature cannot cross the bridge.** Now
  `bridge_feasible: no` after the plan was re-derived. It must not be the first
  unit dispatched, and `next_unit` does not yet consult the field — it selects
  on `disposition` alone. Decide the policy (skip / escalate / pre-dispose)
  before any unit is dispatched.
- **BLOCKER — extraction reports success and captures no state.** `vit extract`
  prints `✓ Extraction successful` alongside `WARNING: No state data captured.`
  Zero state files against the first replication's `kernel/AddToList.0.0.1`.
  Instrumented library, run command and invocation window all ruled out by
  measurement. **Kernel replay is unavailable until this is understood**, so
  units must close on the generated harness. See RUNBOOK.md → Extract / capture.
- **The launcher's protect glob has a hole.** `rosco/controller/src/*.f90`
  matches 11 files and NOT `DISCON.F90` — which is the one file `vit extract`
  modifies (strips CRLF, even under `--dry-run`). Add
  `--protect 'rosco/controller/src/*.F90'`.
- **E1.2 is declared but unmet.** `vit.yaml` carries `-ffp-contract=off` as the
  method requires, but `rosco/controller/CMakeLists.txt` does not pass it — the
  gfortran branch sets `-ffree-line-length-0 -fdefault-real-8 -fdefault-double-8
  -cpp`. Closing this means recapturing the baseline, so it is a deliberate
  open item rather than a silent one.
- `vit.yaml` is declared `derive` with `red_tested = false`. The planned test is
  to drop `assumed_size_arrays.avrSWAP` and confirm extraction breaks; it is
  blocked behind the extraction blocker above.
- Reset-to-clean does not exist yet, and is needed sooner than expected: an
  extraction leaves an INSTRUMENTED `libdiscon.so` installed in `rosco/lib/`.
- Bootstrap otherwise incomplete: `phases.toml` still declares most criteria
  `manual`, which is NOT_EVALUABLE and never a pass.
