# SecLPFilter_Vel — unit #19

`REAL(DbKi) FUNCTION SecLPFilter_Vel(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue)`
in `rosco/controller/src/Filters.f90`. The VELOCITY form of unit #18's
second-order filter: the same ten arguments, the same three `a` coefficients,
and a numerator carrying a single `DT` and a sign —
`b2 = 2.0*DT*CornerFreq**2.0`, `b1 = 0.0`, `b0 = -2.0*DT*CornerFreq**2.0` —
which makes it a differentiator rather than a low-pass, and that fact decides
what every layer below can see.

**Five layers ran and all five are alive.** The fourth unit in this campaign of
which that is true, after `LPFilter` (#11), `NotchFilter` (#13) and
`SecLPFilter` (#18).

| layer | result | red test |
|---|---|---|
| kernel replay | 62/62 cases, **14,818 compared field rows, ALL `IDENTICAL`** | zero stub 0/62, moves 477 rows |
| differential harness (clean Fortran) | **2,884 checked, 0 failed, 0 inadmissible** | the unit as a no-op fails **2,884 of 2,884** |
| mutation score | **79 of 79** behavioural killed, **1.000**, **0 declared equivalent** | 2 excluded as `killed (no compile)` |
| post-integration harness (the wrapper) | 2,884 checked, 0 failed | `CornerFreq`/`Damp` swapped at the `seclpfilter_vel_c` call: **1,100 of 2,884** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED: 14,140 of 5,252,000**, 0 after revert |

## A stub whose entire return value is the constant 0.0 PASSED the kernel verdict

**C12 — the wrong artifact is committed, and it is committed because it is
right about the instrument.**
`kernel.zero-output-stub-PASSES-verdict-63-rows-move.run.txt` is the log of
`seclpfilter_vel.zero-output-stub.cpp`: identical to the shipped translation in
every state write — the four history slots, all six coefficients, the history
shift, the instance increment — with the filter expression replaced by `0.0`.

```
    Total number of verification cases  :    62
    Number of verification-passed cases :    62
    kernel: SecLPFilter_Vel: PASSED verification
```

`vit verify` prints `✓ VERIFICATION PASSED: 62/62 passed` on it.

The reason is arithmetic, not tooling. The reference output has decayed to
**-2.97e-52** by the captured window (below), and KGen's `kgen_tolerance` is
`1.D-14` **ABSOLUTE**, so the 63 field rows that DO move are classified
`NOT IDENTICAL(within tolerance)` and **counted as passes**:

```
 localvar%cc_actuateddl is NOT IDENTICAL(within tolerance).
           1  of           12  elements are different.
 Average - reference   -2.9734115494462705E-052
```

This is unit #3's rule in its starkest sighting so far: *the bit-exact claim
lives in the field log's `status` column, not in the verdict line.* Read that
way the instrument is not blind at all — the shipped translation's **14,818
rows are all `IDENTICAL`**, which is a claim the zero-output stub fails and the
verdict cannot express. What is committed here is the verdict being wrong, so
that the next unit does not have to rediscover it from prose.

`kernel_field_rows.txt` reports the verdict line and the row statuses side by
side for all four runs, for exactly this reason. Regenerate it with
`python3 kernel_field_rows.py`.

## What the kernel could NOT constrain, and how much of the window is vacuous

Three stubs, three different answers, all in `kernel_field_rows.txt`:

1. **Zero stub** — reads no argument, writes zeros. `0 of 62 PASSED`, 477 rows
   move. The comparison is alive. Its shape is worth reading: the five
   coefficient rows move in **62 of 62** cases (the stub writes them
   unconditionally, where the reference writes them only under
   `IF ((iStatus == 0) .OR. reset)`), while every history and output row moves
   in **21 of 62** — and 21 is the whole of the live window.
2. **Zero-output stub** — above. 21 of 62 cases move, 41 do not. The 41 are
   exactly the cases with `Time <= 500`: the only call site is inside
   `IF (CntrPar%CC_Mode == 1)`, whose `CC_DesiredL` assignments are guarded by
   `IF (LocalVar%Time > 500)`, so before that the filter is handed 0 with all
   histories 0 and returns 0 for **any** coefficients. Two thirds of this
   kernel cannot tell the translation from a constant.
3. **Hardcoded-argument stub** — `CornerFreq` pinned to the literal `2*PI/20`,
   `Damp` to `1.0`, `InitialValue` ignored. `62 of 62 PASSED`, and it moves
   **5 rows, in case 1 only, all within tolerance**. Exactly **1 of the 62
   captured cases has `istatus == 0`** (and `restart == T`), so 61 cases never
   read `CornerFreq` or `Damp` at all — unit #11's distinction, "the argument is
   not read" rather than "the argument has one value". The five moved rows are
   the residue of the one case that does read them, and they are under tolerance
   because a hand-written `2*PI/20` differs from `2*PI/CntrPar%CC_ActTau` only in
   the last bits. Only the differential harness varies those arguments.

## Call site and window

There is **exactly one call site in the tree**: clean `Controllers.f90:941`,
inside `CableControl` —

```fortran
LocalVar%CC_ActuatedDL(I_GROUP) = SecLPFilter_Vel(LocalVar%CC_DesiredL(I_GROUP), &
        LocalVar%DT, 2*PI/CntrPar%CC_ActTau, REAL(1.0,DbKi), &
        LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instSecLPFV)
```

Coverage puts it in three scenarios: **3 (15,999 hits), 7 (23,999) and 27
(23,999)**. Scenario 3 was rejected on its own file's evidence rather than on
its hit count — `Examples/vit_sim.py` says of it, *"with CC_Group_N=1 and
tlen=400 < 500, the filter processes zero input the entire run"* — which is P9
in the source comments. Scenario 27 was chosen; `vit.yaml`'s window
`0:0:1-20,12000-12020,23900-23920` was set at unit #2 against a call site with
the same 23,999 hit count, and the captured indices are exactly those 62 with no
stray index (`kgen_statefile.lst`). Captured `Time` runs 0.0 → 597.975, and
**21 of the 62 are past `Time > 500`.**

## The gate sees this unit, and the number is its own

`gate/SecLPFilter_Vel.redtest.json` — the returned value scaled by 1.000001
moves **14,140 of 5,252,000**, revert returns 0, `scenarios_failed: []`,
`perturbation_broke_scenarios: []`. The moved channels are the unit's entire
footprint and nothing else:

```
scenario_7:cc_actuated_dl  3071/24000     scenario_27:cc_actuated_dl  3071/24000
scenario_7:cc_actuated_l   3999/24000     scenario_27:cc_actuated_l   3999/24000
```

3,999 of 24,000 is the tail after `Time > 500`. Scenario 3 does not appear,
which is the same fact as its rejection above, measured from the other end.
Unit #12's check applied — 14,140 matches no other committed redtest artifact
(the nearest are `NotchFilterSlopes` 128,918 and `ColemanTransform` 124,353), so
this is a distinct sighting and not a shared refuse-to-start.

## Mutation reached 1.000 on the corpus unit #18 left behind

**79 of 79 behavioural mutants killed, 0 declared equivalent**, 2 excluded as
`killed (no compile)`. The `assoc_reorder` operator ran and left nothing alive.

That is the direct dividend of unit #18's two additions to
`harness/generate.py` — the hot magnitude rungs run UNPINNED, and `±sqrt(DBL_MAX)`
as a third isolating pin — which were made because `SecLPFilter`'s
`2.0*DT**2.0*CornerFreq**2.0` is a bare product with no term to remove. This
unit's `a1` is the identical expression. The corpus this unit inherited already
contained the pairs that kill it, and the rule_coverage line in
`harness/SecLPFilter_Vel.json` names both blocks by their reasons.

**What the harness HELD, so cannot test:** 72 parameters, of which 36 held reals
are zero in every case — a scaling defect confined to one of those cannot move
any output. `R4_compare_all_outputs` reports the return value plus 47
out-parameters, the six `lpfV_` coefficient arrays among them.

## Files

| file | what it is |
|---|---|
| `seclpfilter_vel.final.cpp` | the shipped translation |
| `kernel.verification-PASSES-62of62.run.txt` | the green, 14,818 rows all IDENTICAL |
| `kernel.zero-stub-FAILS.run.txt` | 0/62, 477 rows moved — the comparison is alive |
| `seclpfilter_vel.zero-stub.cpp` | the stub that produced it |
| `kernel.zero-output-stub-PASSES-verdict-63-rows-move.run.txt` | the verdict passing a constant-0.0 return |
| `seclpfilter_vel.zero-output-stub.cpp` | the stub that produced it |
| `kernel.hardcoded-arguments-stub-PASSES.run.txt` | 62/62 — what the kernel cannot constrain |
| `seclpfilter_vel.hardcoded-arguments-stub.cpp` | the stub that produced it |
| `kernel_field_rows.txt` / `.py` | verdict line vs row statuses, for all four runs |
