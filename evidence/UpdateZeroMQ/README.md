# UpdateZeroMQ — evidence

Unit #21. `rosco/controller/src/ZeroMQInterface.f90:7`, the whole module's only
procedure. Disposition **integrated**; the gate is blind to it and says so.

## What this unit is, and which of its two bodies was translated

`ZeroMQInterface.f90` is written twice, under `#ifdef ZMQ_CLIENT` and `#else`.
Only one of them is in the library this campaign compares against:
`rosco/controller/CMakeLists.txt:131` gates the definition on
`pkg_check_modules(PC_ZeroMQ libzmq)`, and this tree's
`rosco/controller/build/CMakeCache.txt` records `PC_ZeroMQ_FOUND:INTERNAL=` —
empty. **`ZMQ_CLIENT` is undefined, `zmq_client` is neither compiled nor linked,
and the `#else` branch is the object under translation.** Everything below about
dead code and indeterminate values is a statement about that configuration.

## The layers, and what each could see

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT AVAILABLE** — no live call site anywhere in the campaign | n/a |
| differential harness vs clean Fortran | **8,334 checked, 0 failed, 0 inadmissible**, 6 outputs declared no-oracle | the unit as a no-op fails **4,167 of 8,334** — every case that reaches the body |
| mutation score | **12 of 12 behavioural killed, 1.000**, 8 declared equivalent, 0 no-compile | the inherited corpus scores **0.2593 with 40 survivors** — see below |
| post-integration harness (wrapper only) | **8,334 checked, 0 failed** | the `ErrVar` reverse copy removed from the wrapper, rebuilt between the edit and the run: **4,167 of 8,334**; revert, rebuild, green returns |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | whole-unit no-op moves **0** — the unit is DEAD in all 27 scenarios; control on the same build moves **1,857,893** |

## Why there is no kernel

Both call sites are guarded by `CntrPar%ZMQ_Mode > 0`, and `ZMQ_Mode` is `0` in
all 14 `Examples/DISCON*.IN` with no scenario patching it. Coverage of the
CALLER — an instrumented file with 47 live lines — states it exactly:

```
DISCON.F90:103  IF (CntrPar%ZMQ_Mode > 0) THEN      407,976 hits
DISCON.F90:104      CALL UpdateZeroMQ(...)                0 hits
DISCON.F90:141  ELSEIF ((iStatus == -1) .AND. (ZMQ_Mode > 0))  36,024 hits
DISCON.F90:142      CALL UpdateZeroMQ(...)                0 hits
```

That is a measurement rather than an absence: `coverage/line_coverage.json`
holds an empty dictionary for `ZeroMQInterface.f90`, which cannot tell "never
ran" from "never instrumented" (RUNBOOK), and the guard's own hit count settles
it from the caller's side. The gate red test moving 0 is the same fact.

## The finding: the reference is not a function of its arguments

`real(C_DOUBLE) :: setpoints(8)` is never assigned in this configuration — the
`call zmq_client(...)` that would write it is inside the `#ifdef` — and the
procedure copies it into eight `LocalVar%ZMQ_*` fields.

* `setpoints_indeterminate_probe.f90` / `setpoints_indeterminate.run.txt` —
  three calls in one process. Fresh frame → `NaN`, `9.75e-310`, `2.62e-322`;
  after a routine that fills the same stack region with `1.0` → `1.0`; after
  `-7.25` → `-7.25`. **Six of the eight track the previous frame verbatim.**
* `harness.setpoints-indeterminate-FAILS-4-of-4179.json` — the same thing from
  the other side, before the exclusion was stated: 4,175 of 4,179 cases agreed
  on every output and **4 disagreed on these fields alone**, the reference
  answering `68bb11a718b90000` — a leftover pointer — where the translation
  answers 0.

So the disagreement is not between two implementations of one thing; it is
between one implementation and an absence. The six fields are declared
`no_oracle` in `harness/ranges.toml`, which is reported in the run's own
artifact (`no_oracle_outputs`) rather than applied silently, and seven of the
eight declared-equivalent mutants are unkillable for that one reason.

## Two more upstream ROSCO defects, both measured

`domain_aborts_probe.f90` / `domain_aborts.run.txt`:

1. **A full-width `ZMQ_CommAddress` kills the controller.** The field and the
   local it is written into are both `CHARACTER(256)`, so
   `write (zmq_address,'(A,A)') TRIM(...), C_NULL_CHAR` needs 257 characters of
   a 256-character record: `Fortran runtime error: End of record`, exit 2. The
   generated harness hit it too — the first run died at `ZeroMQInterface.f90:53`
   and wrote no JSON. Pinned in `harness/ranges.toml` with the cost stated.
2. **`MOD(n_DT, 0)`.** `n_DT_ZMQ = NINT(ZMQ_UpdatePeriod / DT)` and
   `ZMQ_UpdatePeriod` is only parsed when `ZMQ_Mode /= 0`, so on every input
   this campaign carries the divisor is 0 — and the `ZMQ_Mode > 0` guard on the
   call is the only thing standing between the shipped controller and a
   division by zero. Measured on this build (aarch64): it does not trap, it
   returns with the branch not taken, which is an artifact of the divide
   instruction and not a semantic.

## Files

| file | what it is |
|---|---|
| `updatezeromq.final.cpp` | the shipped translation |
| `updatezeromq.noop-stub.cpp` | the harness red test's input |
| `setpoints_indeterminate_probe.f90`, `.run.txt` | the reference has no answer on 8 outputs |
| `domain_aborts_probe.f90`, `domain_aborts.run.txt` | the two shapes on which the reference stops |
| `harness.setpoints-indeterminate-FAILS-4-of-4179.json` | the same indeterminacy, seen by the harness |
| `gate.control-getwords-perturbed-MOVES.json` | same-build control, 1,857,893 — the gate chain works |
| `vit_defects/bridge.before-nested-type-import.f90` | the generated bridge that would not compile |

## Three instrument defects fixed on the way, none worked around (X2)

1. **`vit test-validate` emitted a bridge that would not compile**, and every
   earlier unit escaped it by accident. The bridge copies the reference's own
   `USE ROSCO_Types, ONLY : ...` list and then declares `POINTER`s to the NESTED
   types it decomposes — names the reference never mentions.
   `ReadAvrSWAP` decomposes the same `LocalVariables` and builds only because
   `ReadSetParameters` re-exports `ROSCO_Types` at module scope; a module that
   keeps its imports inside its procedures has no such route. Fixed in
   `vit/test_validate.py::_nested_type_use_statements` — additive, and silent
   when an unrestricted `USE` already makes the types visible.
2. **The corpus could not reach a rate gate.** `MOD(A, B) == 0` is true only on
   the multiples of `B`; no ladder over either name reaches it, and
   `relational_pairs_from` cannot either, since the crossing value is a MULTIPLE
   of the other side rather than the other side. 4,175 of 4,179 cases entered
   nothing. New rule R9 (`harness/generate.py`, `divisibility_pairs_from`) re-runs
   **every** case with the gate satisfied — a sample was tried first and moved
   the score not at all, because 20 of 4,179 cases are the knob combinations and
   a sample of a corpus is not a sample of its conjunctions.
3. **`compare_op` was mutating the angle brackets of `static_cast<...>`.** Ten
   of thirty mutants could not compile, 33%, and `vit_mutate` refuses to score
   above 25% — a guard that exists to catch a build holding two definitions of
   the function was being spent on punctuation. Fixed in
   `harness/cppmutate.py::_mask_template_brackets`, same discipline as the
   existing comment and literal masks. Unit #17's committed artifact carries
   29 of 135, 21% — just under the limit.
