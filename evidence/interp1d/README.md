# evidence/interp1d — unit #23

Five layers, **all five alive**, and one of them went RED on a defect the other
four passed. Everything below is a file in this directory or in
`harness/`, `mutation/`, `gate/`.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1, clean `ControllerBlocks.f90:421` | 248 field rows, **all IDENTICAL** | zero stub **0 of 62**, moving `a_op` in all 62 — but the **no-boundary-branches stub PASSES 62 of 62** |
| differential harness vs clean Fortran | **497** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **497 of 497** |
| mutation score | **66 of 66 behavioural killed, 1.000**, 5 declared equivalent, 1 no-compile | the inherited corpus scores **0.865, thirteen survivors** |
| post-integration harness (wrapper only) | **497 checked, 0 failed** | **it FAILED FIRST** — see below — and the deliberate red re-take fails **454 of 497** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `interp1d_result × 1.000001` moves **1,341,803 of 5,252,000**; revert verified |

## The wrapper `vit integrate` generated first threw away every write this unit makes

`vit integrate ... --apply` without `--reverse-copy` emits

```fortran
CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
interp1d_result = REAL(interp1d_c(..., C_LOC(ErrVar_view)), 8)
END FUNCTION
```

and nothing copies the view back. `interp1d` writes `ErrVar%aviFAIL` and
`ErrVar%ErrMsg` on both of its error paths and in the `RoutineName` prefix, so
after integration those writes were **lost**.

**The kernel passed it (62/62) and the gate passed it (5,252,000 / 0.)** The
kernel marshals the view itself, so the wrapper is not in its path at all; the
gate's 27 scenarios hand `interp1d` well-formed monotone tables with
`aviFAIL == 0`, so the fields the wrapper drops are never written. The
post-integration harness is the only layer that could see it, and it is the
layer that exists for exactly this (`--post-integration` measures the WRAPPER,
not the arithmetic). **454 of 497 cases**, the mismatches naming
`ErrVar.ErrMsg` and `ErrVar.n_ErrMsg` and nothing else.

The wrong artifact is committed as
`harness.postintegration.NO-REVERSE-COPY-FAILS-454-of-497.json` (C12), taken
before the fix. The fix is `--reverse-copy` on `vit integrate`, and the red
re-take — the `CALL vit_copy_scalars_to_errorvariables` line deleted from the
wrapper, rebuilt between the edit and the run — reproduces the same
454 of 497.

## The kernel cannot see two of the unit's three interpolation branches

`kernel.no-boundary-branches-stub.run.txt`: the shipped translation with
`IF (xq <= MINVAL(xData))` and `ELSEIF (xq >= MAXVAL(xData))` **deleted**
scores **62 of 62 PASSED, 248 of 248 IDENTICAL**. Every captured query at
`ControllerBlocks.f90:421` — the wind-speed estimator's pole lookup — is
interior, so the two endpoint branches are unreachable by the kernel at any
invocation window.

That is what made the corpus addition below load-bearing rather than tidy: on
those two branches the differential harness is the ONLY instrument.

The zero stub is what says the kernel is alive at all: **0 of 62 passed, `a_op`
moves in all 62 cases**, and it is DETERMINATE and finite, which unit #22
established is the property that matters — a stub leaving an output NaN scores
`IN_TOL`, because `NaN > kgen_tolerance` is false.

## The reference ABORTS on `SIZE(xData) /= SIZE(yData)`

`reference.size-mismatch-aborts.txt` — the branch transcribed in isolation and
run:

```
LEN after the assignment = 38
characters the WRITE formats = 53
Fortran runtime error: End of record          exit 2
```

`ErrVar%ErrMsg` is `CHARACTER(:), ALLOCATABLE`, so the literal assignment
reallocates it to 38 characters and the internal `WRITE` on the next line
formats 53 into that record. The fifth upstream ROSCO defect this campaign has
measured.

`harness.untied-extents-KILLS-the-reference.txt` is the same fact from the
other side: `_extent_plan` makes extents pairwise distinct on purpose, so the
FIRST generated case had `n_xData = 3, n_yData = 4`, the reference died at
`Functions.f90:118`, and the run produced no JSON at all.
`harness/ranges.toml` now ties the two extents, with the cost stated.

## The five declared-equivalent mutants, and what proves each

`mutation/interp1d.equivalences.json` carries the reason per id. The
measurements they rest on are here:

- `equivalence_probe.cpp` / `.txt` — the MINVAL/MAXVAL reduction (`20f2a843`,
  `23097b47`) and the view-length guard (`b61327db`). The reduction result is
  **not** identical under the loosened comparison — **1,666 of 69,905 swept
  tuples have different stored bits**, because `-0.0 <= 0.0` is true where
  `-0.0 < 0.0` is false. What makes them equivalent is that the two consumers
  cannot tell: **(tuple, xq) pairs whose consumer differs = 0**. `b61327db` is
  swept over all **4,294,967,296** values of a 32-bit int: 0 differ.
- `errmsg_extremes_probe.cpp` / `.txt` — the shipped translation with three
  counters and nothing else changed, run over all 497 harness cases:
  `n_ErrMsg 1 .. 10`, `n_ErrMsg_cap 4097 .. 4106`, largest message 42,
  `s.size() == cap: 0`, `n_ErrMsg <= 0: 0`. That makes `e34d5cc4` and
  `55e42d36` **unreachable rather than equivalent**, and the two are recorded
  differently for that reason.

## Files

| file | what it is |
|---|---|
| `interp1d.final.cpp` | the shipped translation, byte-for-byte |
| `interp1d.zero-stub.cpp` | INPUT to the kernel liveness measurement |
| `interp1d.no-boundary-branches-stub.cpp` | INPUT to the kernel blindness measurement |
| `kernel.translation.run.txt` | the kernel's own stdout, shipped translation |
| `kernel.translation.verify_fields.csv` | VIT's field log for the same run |
| `kernel.zero-stub.run.txt` | 0 of 62 |
| `kernel.no-boundary-branches-stub.run.txt` | 62 of 62 — the blindness |
| `kernel_field_rows.py` / `.txt` | the three logs parsed into one table |
| `size_mismatch_probe.f90` | INPUT to the abort measurement |
| `reference.size-mismatch-aborts.txt` | the abort, measured |
| `harness.untied-extents-KILLS-the-reference.txt` | the same fact through the harness |
| `equivalence_probe.cpp` / `.txt` | proof for three declared mutants |
| `errmsg_extremes_probe.cpp` / `.txt` | corpus counts for two declared mutants |
| `harness.postintegration.NO-REVERSE-COPY-FAILS-454-of-497.json` | the wrong artifact (C12) |
| `done_check.txt` | the done condition, run after the commits |
