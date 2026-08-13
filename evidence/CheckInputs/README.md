# CheckInputs — evidence

Unit #29. `ReadSetParameters.f90:946-1802` in the pinned clean source (`54dd134`),
857 lines: about 180 validity checks over `CntrPar`, `LocalVar%DT` and two
`avrSWAP` elements, whose only effect is `ErrVar%aviFAIL = -1` and a message
into `ErrVar%ErrMsg`.

## Call site and capture (C2, C3)

ONE call site, `ReadSetParameters.f90:269`, inside `SUBROUTINE SetParameters` —
a SUBROUTINE parent block, which unit #24 established a KGen call site must be.
Coverage gives it **25 hits across 24 of 27 scenarios** (scenario 1 twice).

The three scenarios that do NOT reach it are **10, 14 and 24** — the `OL_Mode > 0`
scenarios. That is unit #26's finding one level up: `Read_OL_Input` fails on the
missing `OL_Mode2_Input.dat`, `SetParameters` RETURNs at line 224 on
`ErrVar%aviFAIL < 0`, and CheckInputs is never called. So **the whole
`IF (CntrPar%OL_Mode > 0)` block — the ALLOCATE, both AddToList loops and nine
checks — is unreachable in every scenario this campaign can run**, and only the
generated differential corpus tests it.

**One case, and the arithmetic says why.** The window is
`0:0:1-20,0:0:12000-12020,0:0:23900-23920`, but the site is called once per
scenario at invocation index 1 of its own counter, so all 24 scenarios write the
same statefile `CheckInputs.0.0.1` and overwrite one another. Widening the window
cannot change that; it is not a range that is empty, it is a name collision.

The surviving case is **scenario 27, `Examples/DISCON_stress27.IN`** ("11
controller modes active simultaneously"), identified from the captured values
rather than from the run order: `Fl_Mode=2, PA_Mode=2, PF_Mode=1, CC_Mode=1,
StC_Mode=1` matches `DISCON_stress27.IN` and separates it from
`DISCON_ipc_awc.IN`, the only other file with `AWC_Mode=4`.

## Kernel replay (C6) — ALIVE and BLIND, measured both ways

| run | result | file |
|---|---|---|
| the translation | **1 of 1, 426 of 426 field rows IDENTICAL** | `kernel.translation.run.txt`, `kernel.translation.verify_fields.csv` |
| determinate wrong constant `aviFAIL = -7` | **0 of 1** | `kernel.stub_wrong_constant.run.txt` |
| the WHOLE UNIT deleted (no-op) | **1 of 1 PASSED** | `kernel.stub_whole_unit_noop.run.txt` |

VIT declined to construct its own red test here (`Red test: NOT CONSTRUCTED — no
by-value floating-point parameter and no floating-point result`) and reported
`NON_DISCRIMINATING`. It was right to, and the two stubs above are what replace
it: the wrong constant proves the chain is alive, and the no-op then makes the
blindness unambiguous rather than suspected.

**Why the no-op passes.** The unit's only outputs are `aviFAIL` and `ErrMsg`, and
the captured case is a VALID configuration: the reference raises no error, so
`aviFAIL` is `0` on both sides and `ErrMsg` is never allocated — it does not
appear among the 426 compared fields at all, because KGen guards its comparison
with `IF (ALLOCATED(var%errmsg))`. The body is not skipped in that case; it runs
broadly (`AWC_Mode=4`, `CC_Mode=1`, `StC_Mode=1`, `PS_Mode=1`, `Fl_Mode=2`,
`IPC_ControlMode=1`) and every check it reaches answers "fine". **A unit whose
output is an error signal is invisible to any capture taken on a working
configuration**, however many of its branches that configuration executes.
Both zeros have their control on the SAME build.

## Tool defects found and fixed (C12, X2)

`kernel_callees_header_defect.txt` records the first one with the artifact it
produced, before the fix. Both were in the KERNEL side of the callee bridge and
both were already right on the INTEGRATION side; CheckInputs is the first unit
whose kernel calls `AddToList`. Fixed in VIT `d29bfc2`.
