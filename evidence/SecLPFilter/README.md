# SecLPFilter — unit #18

`REAL(DbKi) FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue)`
in `rosco/controller/src/Filters.f90`. A second-order discrete low-pass filter:
on the first call or a reset, set four history slots and six coefficients from
`DT**2.0*CornerFreq**2.0` and `4.0*Damp*CornerFreq*DT`; then five products
accumulated in source order over a reciprocal; then shift the two input and two
output histories and advance the instance counter.

**Five layers ran and all five are alive** — the third unit in this campaign of
which that is true, after `LPFilter` (#11) and `NotchFilter` (#13).

| layer | result | red test |
|---|---|---|
| kernel replay | 62/62 cases, 14,818 compared field rows, ALL `IDENTICAL` | zero stub 0/62, moves 404 rows — and it took a Makefile fix to see, below |
| differential harness (clean Fortran) | **2,884 checked, 0 failed, 0 inadmissible** | the unit as a no-op fails **2,884 of 2,884** |
| mutation score | **80 of 80** behavioural killed, **1.000**, **0 declared equivalent** | reached by two corpus additions, not by declaration |
| post-integration harness (the wrapper) | 2,884 checked, 0 failed | `CornerFreq`/`Damp` swapped at the `seclpfilter_c` call: **1,040 of 2,884** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 1,349,326 of 5,252,000**, 0 after revert |

## The zero stub PASSED 62/62, and the reason was a missing Makefile prerequisite

**C12 — the wrong artifact first.** `kernel.zero-stub-FAILS.run.txt` is the
correct measurement; it exists only because the first run of the same stub
reported `kernel: SecLPFilter: PASSED verification`, `62 / 62`.

VIT writes the TRANSLATION to `<stem>.hpp` and a three-line `extern "C"` wrapper
to `<stem>.cpp`, and the generated rule was

```make
seclpfilter.o: seclpfilter.cpp        # <- not the file the translation is in
	$(CXX) $(CXXFLAGS) -c $< -o $@
```

so `make` reused an object built from the REAL translation and the stub was
never compiled. `vit verify` never sees this — `_build_and_run_kernel` runs
`make clean` first — but the RUNBOOK teaches two recipes that hand-run `make` in
the kernel directory (re-running a stub, and replaying the original Fortran
through an existing kernel), and both are exposed.

Fixed in VIT (`5ba5e7e`), not worked around (X2):
`seclpfilter.o: seclpfilter.cpp $(wildcard seclpfilter.hpp vit_types.h)`.
`$(wildcard ...)` because the Makefile is patched before those files are
necessarily on disk and a named prerequisite make cannot build is a hard error.
Measured in both directions: with the old rule, editing the .hpp did not
recompile and the stub "passed"; with the new rule the same edit recompiles and
the real translation is back to 62/62 (`kernel.verification-PASSES-62of62.run.txt`).

## What the kernel could NOT constrain

`kernel_field_rows.txt` (regenerate with `kernel_field_rows.py`) is the table,
derived from the three run logs rather than from `kernel/verify_fields.csv` —
`kernel/` is untracked and `reset_to_clean.sh` removes it, so citing the CSV
would be citing a file that no longer exists (unit #17's rule).

Two readings the verdict line does not give:

1. The zero stub's 404 moved rows are **exactly this unit's outputs** —
   `vs_lastgentrqf`, the four `lpf2_*SignalLast*` histories, the six coefficient
   arrays, `instseclpf`. The other ~14,400 rows are the enclosing procedure's
   untouched state and read `IDENTICAL` for any translation at all.
2. The six coefficient arrays move in **1 of 62 cases**. They are the only
   fields written inside `IF ((iStatus == 0) .OR. reset)`, so the
   initialisation branch is reached exactly once in the window and **61 of 62
   cases never read `CornerFreq` or `Damp` at all**. That is unit #11's
   distinction — not "the argument has one value" but "the argument is not
   read" — and it is what makes the next artifact unsurprising:

`seclpfilter.hardcoded-arguments-stub.cpp` ignores four of the ten arguments and
writes the literal the simulation happens to carry (`CornerFreq` → 1.57080,
`Damp` → 0.7, `has_InitialValue` → 0, `InitialValue` unread). It scores
**14,818 of 14,818 `IDENTICAL`, 62/62 PASSED**. Only the differential harness
varies those four.

## The mutation score reached 1.000 by two corpus additions, and neither mutant was equivalent

The inherited corpus scored **0.975** on 2,284 cases with two survivors, both
`assoc_reorder: '2.0 * (DT * DT)' -> '(2.0 * DT) * DT'` — unit #13's shape in a
sibling expression. Both were PROVEN KILLABLE before anything was changed, over
the reachable inputs (`DT` and `CornerFreq` are both direct arguments, so their
own ranges ARE the domain — unit #13's second lesson).

`assoc_reorder_reachability_probe.cpp` — powers of two NEVER distinguish the
regrouping (`DT*DT` is exact there), and of the plain ladder's rungs only
`1e-156` and `1e-158` do. `assoc_reorder_hot_rung_probe.cpp` — at `DT = 1e-156`
with `CornerFreq` at its ordinary `1e3` default the two spellings give
`1.9999999999969307e-306` against `2.0000000000018713e-306`.

**Addition 1 — the hot rungs, UNPINNED.** `1e-156`/`1e-158`/`1e-160` lived ONLY
in the joint block, which pins every OTHER defaulted real to `0.0` or `1e300`.
That is right for `NotchFilter`, whose coefficient is a SUM: removing a term is
what exposed its rung. This unit's `lpf2_b1` is a bare PRODUCT and has no term
to remove — at `CornerFreq = 0.0` it is 0 under both spellings, at `1e300` it is
Inf under both. The plain ladder cannot reach it either: it carries `1e-155`,
which does not distinguish. **2,284 → 2,484 cases, 0.975 → 0.988.**

**Addition 2 — `±sqrt(DBL_MAX)` as a third isolating pin.** The `a1` mutant
survived addition 1 too, because

```
FP%lpf2_a1(inst) = 2.0*DT**2.0*CornerFreq**2.0 - 8.0
```

is `~1e-306 - 8.0`, which is exactly `-8.0` under both spellings — the
subtraction annihilates the difference. `sqrt(DBL_MAX)` is the largest x whose
`x*x` is still FINITE, so it amplifies the rung to the top of the exponent range
instead of overflowing it away. `assoc_reorder_ladder_pair_search.py`: of the
**1,936** ladder-by-ladder pairs exactly **SIX** separate the two spellings of
`a1`, and all six are `(hot rung, ±sqrt(DBL_MAX))`. **2,484 → 2,884 cases,
0.988 → 1.000.**

Both are additive blocks in `translation-loop/harness/generate.py` (`9ee71de`),
appended in place, drawing no random numbers when they do not fire.
`equivalent_declared` stayed **0** — a declaration would have been false twice.

## The gate sees this unit, and the number is its own

`gate/SecLPFilter.redtest.json` — scaling the returned value by 1.000001 moves
**1,349,326 of 5,252,000**, revert returns 0, `scenarios_failed: []`. Unit #12's
check applied, against every committed redtest artifact:

```
ColemanTransform 124,353   ColemanTransformInverse 389,644   GetWords 1,857,893
LPFilter 1,592,059   NonDecreasing 1,857,893   NotchFilter 551,278
NotchFilterSlopes 128,918  ReadAvrSWAP 1,487,557  __gate__ 1,604,573
SecLPFilter 1,349,326   <- matches none of them
```

`LocalVar%VS_LastGenTrqF` is the filtered generator torque the wind-speed
estimator is built on, and three of the unit's live call sites feed channels the
gate reads.

## Files

| file | what it is |
|---|---|
| `seclpfilter.final.cpp` | the shipped translation |
| `kernel.verification-PASSES-62of62.run.txt` | the green, 14,818 rows all IDENTICAL |
| `kernel.zero-stub-FAILS.run.txt` | 0/62, 404 rows moved — the comparison is alive |
| `seclpfilter.zero-stub.cpp` | the stub that produced it |
| `kernel.hardcoded-arguments-stub-PASSES.run.txt` | 62/62 — what the kernel cannot constrain |
| `seclpfilter.hardcoded-arguments-stub.cpp` | the stub that produced it |
| `kernel_field_rows.txt` / `.py` | per-field verdicts derived from the three logs |
| `assoc_reorder_reachability_probe.cpp` / `.out.txt` | which rungs distinguish the regrouping at all |
| `assoc_reorder_hot_rung_probe.cpp` / `.out.txt` | the witness at `DT=1e-156`, ordinary `CornerFreq` |
| `assoc_reorder_witness_search.cpp` / `.out.txt` | full-mantissa search; the `a1` form needs more than a hot rung |
| `assoc_reorder_ladder_pair_search.py` / `.out.txt` | 6 of 1,936 ladder pairs kill `a1`, all `(hot rung, ±sqrt(DBL_MAX))` |

## Call site and window

Clean `Filters.f90:426` — `LocalVar%VS_LastGenTrqF = SecLPFilter(LocalVar%VS_LastGenTrq, ...)`,
**408,000 calls across 23 of the 27 scenarios**, scenario 1 carrying 80,000 of
them. Chosen over the tower-top pair at 371/372 because unit #16 measured
`nacimu_fa_racc` and `fa_acc_nac` at exactly one distinct value, 0.0, in every
scenario — a call site that runs on zeros produces a kernel that cannot fail
(P9). Lines 350/351 (`F_LPFType == 2`) are dead in all 27 scenarios.

Window is exactly the configured `0:0:1-20,12000-12020,23900-23920` = 62 with no
stray index, checked against `kgen_statefile.lst` at extraction time.
