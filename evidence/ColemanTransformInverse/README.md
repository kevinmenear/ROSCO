# ColemanTransformInverse — what each artifact here measured

Unit #3. Every number below was RUN on 2026-08-11, in `vit-dev`, under VIT
`37f8bdf` and loop `5edf80d`, and every artifact carries those revisions in a
`vit_rev`/`loop_rev` field. Check the stamp before believing a number: a run
that fails to write leaves the previous run's file in place and it reads
exactly like a fresh pass.

## The window, and the stub that proves it is alive

`vit.yaml`'s `kgen.invocation` is `0:0:1-20,0:0:12000-12020,0:0:23900-23920` —
the window unit #2 widened. It was NOT re-derived for this unit; it was
re-tested for it, which is not the same thing and is why the stub run below
exists.

Extraction: `Controllers.f90:561`, scenario 27, 63 state files, first attempt.
Call-site selection came from `coverage/line_coverage.json` — 561 has 23,999
hits in scenario 27, the highest of the five call sites in any scenario.

| file | run | result |
|---|---|---|
| `kernel-window-27.verify_fields.csv` | the translation | 63/63 cases, `pitcomipc_1p` IDENTICAL in every one |
| `kernel-window-27.stub-fails.verify_fields.csv` | a stub reading no argument and writing `0.0` | **61 of 63 `OUT_TOL`** |
| `kernel-window-27.statefiles.lst` | — | the 63 captured state files, by name |

The two cases the stub still passes are `...0.0.1` and `...0.0.2` — simulation
start, where the axis inputs are still zero. That is the 1-20 window's failure
mode surviving as two cases out of 63 rather than as all 21, and it is visible
here only because the stub was run. Green was re-confirmed after the stub was
reverted (63/63), so the red is attributable to the stub.

## THE FIELD LOG IS VALUE-BLIND FOR THIS UNIT, and the recipe in RUNBOOK.md is not

`RUNBOOK.md`'s "check the CAPTURED VALUES, not the counts" recipe reads
`reference` out of `verify_fields.csv` and counts non-zeros. **That column is
EMPTY here.** This unit's only output is an array, and VIT logs an array field
as one row carrying `type=array`, `status=IDENTICAL`, `diff=size=3` — no
values at all:

    ColemanTransformInverse.0.0.1,pitcomipc_1p,array,IDENTICAL,,,size=           3

So for a unit whose outputs are arrays the recipe returns nothing and cannot
distinguish a live window from a dead one. The stub run is not a supplement to
it here; it is the only instrument that answers the question.

## WHAT "PASSED" MEANS IN THIS KERNEL, read out of the generated code

`kernel/ColemanTransformInverse/Controllers.f90`'s `kv_ipc_real__dbki_dim1`:

    IF (ALL(var == kgenref_var)) THEN            -> IDENTICAL
    ELSE ... rmsdiff = SQRT(SUM((var-ref)**2)/n)
             IF (rmsdiff > kgen_tolerance) -> OUT_TOL   ELSE -> IN_TOL

`kgen_tolerance` is `1.D-14`, and `rmsdiff` is ABSOLUTE. The kernel's PASS
condition is therefore "no field out of an absolute RMS tolerance of 1e-14",
NOT bit-identity — and this unit's outputs are of order 1e-3 rad, so a
translation wrong in the low bits could pass as `IN_TOL` and the run would
still print `VERIFICATION PASSED`.

What makes this unit's kernel evidence bit-exact is not the pass condition. It
is that all 63 fields landed in the `ALL(var == kgenref_var)` branch —
`status=IDENTICAL` in the field log, which is the bit-exact one. Read the
status column, not the verdict line.

## What nothing here sees: aziOffset is 0 in every scenario

`IPC_aziOffset` is `0.000 0.000` in all 14 `Examples/DISCON*.IN`, no scenario
patches it, and `AWC_phaseoffset` is `0.000000000000` in all of them and is
patched to `'0.0'` by the one scenario that sets it (15). The remaining call
site passes the literal `0.0_DbKi`. So **every one of the five call sites, in
all 27 scenarios, calls this function with `aziOffset = 0`** — the kernel and
the gate are both blind to that argument.

The differential harness is not: `R6_reference_literals` varies `aziOffset`
across 257 cases, and the three `aziAngle + aziOffset` → `-` mutants are killed
in 81 of 257 cases each. Those 81 cases are the whole of what constrains
`aziOffset` in this campaign, and they exist only in the harness.

`nHarmonic` is likewise never anything but 1 or 2 in the scenarios (NP_1, NP_2,
`AWC_harmonic = 1`), and the kernel's window sees only `NP_1 = 1`.

## Mutation

24 of 24 behavioural mutants killed, score 1.000, 0 declared equivalent. 2 more
did not compile (`compare_op` on a translation containing no comparison) and
are EXCLUDED from the score rather than counted as kills — loop `46a7f4f`
changed that after unit #2, whose 35/35 included 2 such.

Two of the 24 kills are crashes, not case mismatches: `[2] -> [2 + 1]` and
`'2' -> '3'` both write one past the end of the caller's 3-element array. They
are honest kills — the harness died — but they are memory errors, and a value
comparison alone would not have seen them. 22 of 24 were killed by a case
mismatch.
