# Bug Report: Two Uninitialized Variables in Flp_Mode=2 Code Path

## Summary

Two uninitialized local variables in the Flp_Mode=2 (proportional trailing edge flap control) code path cause undefined behavior. One causes a segfault with the default BAR_10 configuration. Both were introduced in ROSCO v2.9.0 (commit `b994e4d`, January 2024).

These bugs are unlikely to have affected anyone's results — Flp_Mode=2 does not appear to be exercised by any simulation test in the ROSCO repo. They were discovered while building a Fortran→C++ translation verification tool against ROSCO as a test case.

## Environment

- ROSCO v2.10 (commit `e8010f0`)
- gfortran (GNU Fortran) on Ubuntu 24.04 (aarch64)
- 1-DOF Python simulation (`rosco.toolbox.sim`)
- BAR_10 configuration: `Flp_Mode=2`, `F_GenSpdNotch_N=0`, `F_FlpCornerFreq=0.5 0.7`

## Bug 1: Uninitialized `n` in PreFilterMeasuredSignals (Segfault)

**File:** `rosco/controller/src/Filters.f90`
**Severity:** Crash (segfault) with default BAR configuration

### Description

In `PreFilterMeasuredSignals`, the `Flp_Mode==2` branch applies a NotchFilter to blade root bending moments using `CntrPar%F_GenSpdNotch_Ind(n)` as an array index. The variable `n` is a loop counter from an earlier `DO n = 1,CntrPar%F_GenSpdNotch_N` loop (~line 355). When `F_GenSpdNotch_N=0` (no generator speed notch filters configured), the loop never executes and `n` is never assigned. The subsequent use of `n` as an array index reads undefined memory.

### Code (approximately lines 402-416)

```fortran
! Blade root bending moment for IPC
DO K = 1,LocalVar%NumBl
    IF ((CntrPar%IPC_ControlMode > 0) .OR. (CntrPar%Flp_Mode == 3)) THEN
        LocalVar%RootMOOPF(K) = NotchFilterSlopes(...)
    ELSEIF ( CntrPar%Flp_Mode == 2 ) THEN
        LocalVar%RootMOOPF(K) = SecLPFilter(...)
        LocalVar%RootMOOPF(K) = HPFilter(...)

        ! Use same as generator speed signal because that's how it was before
        LocalVar%RootMOOPF(K) = NotchFilter(LocalVar%RootMOOPF(K), LocalVar%DT, &
                                            CntrPar%F_NotchFreqs(CntrPar%F_GenSpdNotch_Ind(n)), &     ! ← n is undefined
                                            CntrPar%F_NotchBetaNum(CntrPar%F_GenSpdNotch_Ind(n)), &   !   when F_GenSpdNotch_N=0
                                            CntrPar%F_NotchBetaDen(CntrPar%F_GenSpdNotch_Ind(n)), &
                                            LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch)
    ELSE
        LocalVar%RootMOOPF(K) = LocalVar%rootMOOP(K)
    ENDIF
END DO
```

### How to Reproduce

Run a simulation with `Flp_Mode=2` and `F_GenSpdNotch_N=0` (the default in `Examples/Test_Cases/BAR_10/BAR_10_DISCON.IN`). The crash occurs in `PreFilterMeasuredSignals` on the first DISCON call.

With the 1-DOF Python sim, a minimal reproduction is:

```python
from rosco.toolbox import control_interface as ROSCO_ci
from rosco.toolbox.utilities import write_DISCON
# ... load turbine, tune controller, write DISCON.IN with Flp_Mode=2,
#     F_FlpCornerFreq > 0, IPC_ControlMode=0 ...
ci = ROSCO_ci.ControllerInterface(lib_name, param_filename='DISCON.IN', sim_name='test_flp')
# Segfaults during init_discon (first call to DISCON)
```

Building with `-fcheck=bounds` confirms the issue:
```
At line 412 of file Filters.f90
Fortran runtime error: Array bound mismatch
```

### Suggested Fix

Wrap the NotchFilter call in a loop matching the gen-speed notch pattern. When `F_GenSpdNotch_N=0`, the loop body is skipped entirely (no notch filters to apply):

```fortran
                ! Apply gen speed notch filters to blade root signal
                DO n = 1,CntrPar%F_GenSpdNotch_N
                    LocalVar%RootMOOPF(K) = NotchFilter(LocalVar%RootMOOPF(K), LocalVar%DT, &
                                                        CntrPar%F_NotchFreqs(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        CntrPar%F_NotchBetaNum(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        CntrPar%F_NotchBetaDen(CntrPar%F_GenSpdNotch_Ind(n)), &
                                                        LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instNotch)
                END DO
```

---

## Bug 2: Uninitialized `K` in FlapControl Initialization

**File:** `rosco/controller/src/Controllers.f90`
**Severity:** Undefined behavior (out-of-bounds array access with undefined index)

### Description

In `FlapControl`, the initialization block for `Flp_Mode==2` (executed when `iStatus==0`) calls `PIIController` using `K` as an array index, but `K` is a local variable that has not been assigned. The steady-state block for the same mode correctly uses `DO K = 1,LocalVar%NumBl`.

### Code (approximately lines 658-672)

```fortran
SUBROUTINE FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
    ...
    INTEGER(IntKi)              :: K                    ! ← local, no default initialization
    ...
    IF (CntrPar%Flp_Mode > 0) THEN
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%Flp_Angle(1) = CntrPar%Flp_Angle  ! ← hardcoded 1,2,3
            LocalVar%Flp_Angle(2) = CntrPar%Flp_Angle
            LocalVar%Flp_Angle(3) = CntrPar%Flp_Angle
            ! Initialize controller
            IF (CntrPar%Flp_Mode == 2) THEN
                LocalVar%Flp_Angle(K) = PIIController(RootMyb_VelErr(K), ...)  ! ← K undefined
            ENDIF

        ...

        ELSEIF (CntrPar%Flp_Mode == 2) THEN
            DO K = 1,LocalVar%NumBl                                             ! ← K properly looped
                LocalVar%Flp_Angle(K) = PIIController(...)
            END DO
```

### Practical Impact

Lower severity than Bug 1. The init call passes `I0=0.0` and triggers PIIController's reset branch, which writes `piP%ITerm(inst) = 0.0` over already-zero memory (SAVE semantics). The `instPI` counter is reset to 1 at the start of every DISCON call. So the init call is effectively a no-op for controller state — but `Flp_Angle(K)` and `RootMyb_VelErr(K)` are accessed at an undefined index, which is undefined behavior per the Fortran standard.

Additionally, PIIController uses `inst` as an instance index (incremented per call). The init block calls it once, consuming one inst slot. The steady-state block calls it `NumBl` times (typically 3). Without the loop in the init block, only 1 of 3 blade instances is explicitly initialized via the reset path. The other 2 rely on zero-initialized memory being equivalent to the reset state (which it is when `I0=0.0`).

### Suggested Fix

Match the steady-state pattern:

```fortran
            IF (CntrPar%Flp_Mode == 2) THEN
                DO K = 1,LocalVar%NumBl
                    LocalVar%Flp_Angle(K) = PIIController(RootMyb_VelErr(K), 0 - LocalVar%Flp_Angle(K), &
                        CntrPar%Flp_Kp, CntrPar%Flp_Ki, 0.05_DbKi, -CntrPar%Flp_MaxPit, CntrPar%Flp_MaxPit, &
                        LocalVar%DT, 0.0_DbKi, LocalVar%piP, LocalVar%restart, objInst%instPI)
                END DO
            ENDIF
```

---

## Additional Note: F_FlpCornerFreq Validation

When `Flp_Mode > 0`, `CheckInputs` validates `F_FlpCornerFreq(1) > 0` and sets `aviFAIL=-1` on failure. The default DISCON.IN generated by the ROSCO toolbox for the NREL 5MW has `F_FlpCornerFreq = 0.0000  1.0000`. This means any simulation that sets `Flp_Mode > 0` without also patching `F_FlpCornerFreq` will silently fail — the controller returns an error on the first call and never executes any control logic, but the simulation continues without crashing. This is technically correct behavior (the validation works as designed), but it can mask the bugs above since the crash in `PreFilterMeasuredSignals` is never reached.

## Context

These bugs were discovered during development of VIT (Verified Incremental Translator), a tool for verified Fortran→C++ translation. VIT uses KGen to extract runtime state from ROSCO functions during simulation, then verifies C++ translations against the captured state. Extracting the `PIIController` function (Flp_Mode=2 code path) required running the 1-DOF simulation with Flp_Mode=2 — the first time this code path was exercised in simulation.

Both bugs were introduced in the same commit (`b994e4d`, ROSCO v2.9.0, January 2024). The Flp_Mode=2 feature appears to have been implemented but never end-to-end simulation-tested — example `09_distributed_aero.py` tunes BAR flap controller gains but does not run a simulation, and no test in `rosco/test/` exercises Flp_Mode=2.
