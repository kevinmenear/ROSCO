# NonDecreasing — evidence

Unit #12. `LOGICAL FUNCTION NonDecreasing(Array)`, `ROSCO_Helpers.f90`, clean
line 1558. Disposition `integrated`.

The first **LOGICAL function result** in this campaign, and the first unit whose
argument is a bare `REAL(DbKi), DIMENSION(:)` assumed-shape array.

## What each artifact is

| file | what it measures |
|---|---|
| `nondecreasing.final.cpp` | the shipped translation, as integrated |
| `kernel.verify_fields.csv` | the green kernel run: 1 case, **200 field rows, all `IDENTICAL`** |
| `nondecreasing.constant-false-stub.cpp` + `kernel.constant-false-stub-FAILS.verify_fields.csv` | LIVENESS. Returning `.FALSE.` makes the caller's `.NOT.` guard true, the error branch runs, and **2 of 201 rows go `OUT_TOL` — `avifail` (`0` → `-1`) and `errmsg` (blank → `PC_GS_angles must be non-decreasing`)**. The comparison is alive and it is alive on this unit's result. |
| `nondecreasing.constant-true-stub.cpp` + `kernel.constant-true-stub-PASSES.verify_fields.csv` | VACUITY, and it is the important one. A stub that reads **no argument** and returns `.TRUE.` **passes 1/1, 200 of 200 `IDENTICAL`**. The kernel is a lookup table for this unit: it cannot constrain the array at all. |
| `nondecreasing.inverted-result-stub.cpp` | the differential harness's red test. The unit's only output is a boolean and the corpus now contains BOTH answers, so no constant can fail every case; the complement can, and does — `harness/NonDecreasing.redtest.json`, **36 of 36 failed**, mismatch list naming `vit_result`. |
| `order_ladder_kills_the_le_mutant.txt` | the corpus repair, measured rather than argued: `'<= 0.0' → '< 0.0'` differs on cases `[4, 8, 10, 11]`, **all four of them cases the new order ladder appended, none of them among the 25 base draws.** Without the repair both `compare_op` mutants survive and the score is 0.875. |
| `logical_result_conversion_probe.f90` / `.out.txt` | gfortran's `INTEGER → LOGICAL` extension NORMALISES (`0 → .FALSE.`; `1`, `2`, `-1`, `256` all → `.TRUE.` with `TRANSFER(L,0) == 1`), so the wrapper's assignment is exact for a C++ that returns 1/0. Measured, because a bit-copy and a normalisation give different answers for a return value of 2. |
| `gate.redtest.json` | the gate red test — see below |
| `loop_defects/` | two generator defects this unit found, recorded with the wrong artifact before either was fixed (C12) |

## The kernel is alive and vacuous at the same time

Both stubs are needed and they say different things. The `.FALSE.` stub proves
the comparison moves; the `.TRUE.` stub proves that moving buys nothing here,
because **`NonDecreasing = .FALSE.` has zero hits in all 27 scenarios**
(`coverage/line_coverage.json`, clean `ROSCO_Helpers.f90:1569`). All 74 calls
across the three live call sites answer `.TRUE.`. The unit is the constant
`.TRUE.` on the exercised domain — unit #7's identity-on-the-exercised-domain
shape, one predicate over.

The kernel compares `cntrpar` and `errvar`, not the unit's result: the call is
inside an `IF` condition, so KGen instruments the enclosing statement. That is
the check unit #7's entry asks for, run and answered — the result reaches the
compared fields only by deciding whether the error branch runs.

## The gate

`gate/NonDecreasing.json` — 5,252,000 values / 351 channels / 27 scenarios,
0 mismatched.

`gate.redtest.json` — forcing the unit to answer `.FALSE.` moved **1,857,893 of
5,252,000**, and the revert returned 0.

That number is **byte-identical to `gate/GetWords.redtest.json`**, which is the
campaign's designated same-build control. So this red test is simultaneously its
own control: the chain from build to install to 27 simulations to bit comparison
reproduced a known-red figure exactly. It also says what the gate sees — not
arithmetic, but the controller refusing to start: both perturbations end in a
rejected input file, and a rejected input file has one output signature.

**One bit is what the gate constrains.** Nothing about the array reaches it.

## Loop-repo defects found here

1. `harness/emit.py` emitted `std::vector<double> Array_a(n_Array_a);` one line
   before `n_Array_a` was declared, because a numeric array's extent is an
   ordinary C parameter that `vit interface` places AFTER the buffer. The
   CHARACTER path had solved this years-equivalent ago with a `predeclared`
   placeholder; the numeric path never got it. Fixed by using the same
   mechanism, gated on `names.index(d) > i` — which is exactly the condition
   under which the old emitter produced the compile error, so no unit that ever
   produced a compiling harness can enter the new branch.
2. `scripts/_integration_shim.py` emitted `extern "C" int32_t nondecreasing_c(...)`
   with no `#include <cstdint>`. Third conditional include in that generator,
   keyed the same way as the two before it.
