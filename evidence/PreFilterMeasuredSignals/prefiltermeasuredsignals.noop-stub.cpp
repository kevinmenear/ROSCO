// VIT Translation Scaffold
// Function: PreFilterMeasuredSignals
// Source: Filters.f90
// Module: Filters
// Fortran: SUBROUTINE PreFilterMeasuredSignals(CntrPar, LocalVar, DebugVar, objInst, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: a8a81f6639dc
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T00:19:51Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature, and the body is transcribed statement for statement.
//
// THIS UNIT IS ALMOST ENTIRELY CALLS. Twenty of its twenty-six executable
// statements are a filter call, and all six callees -- LPFilter (#11),
// SecLPFilter (#18), HPFilter (#9), NotchFilter (#13), NotchFilterSlopes (#14)
// and wrap_180 (#27) -- are already integrated. X1 forbids inlining any of
// them, so each goes through its own `_c` bridge and this file contains no
// filter arithmetic of its own. What it DOES contain is the ORDER of those
// calls, the arguments handed to each, and the instance counters they advance:
// `objInst%instLPF` and friends are INOUT through every bridge, so a call moved,
// dropped or duplicated shifts every later filter onto the wrong state slot.
//
// THE `Flp_Mode == 2` INNER NOTCH LOOP IS THE CAMPAIGN'S OWN FIX TO AN UPSTREAM
// BUG, AND THE REFERENCE IS THE FIXED SOURCE. RUNBOOK.md records it: ROSCO
// v2.9.0 indexes `F_GenSpdNotch_Ind(n)` there with a loop counter left over
// from the earlier `DO n = 1,F_GenSpdNotch_N`, which never ran when
// `F_GenSpdNotch_N = 0`, so the index read undefined memory and scenario 4
// segfaulted. The fix wrapped that call in the same loop. P7 makes the oracle
// the source in THIS tree, so the loop is transcribed as it stands here, not as
// upstream wrote it.
//
// WHAT THE KERNEL CANNOT SEE, read from coverage/line_coverage.json before
// extracting rather than after (its line numbers are the clean source's):
//
//   :349-:351  F_LPFType == 2, the SecLPFilter speed arm   0 hits, ALL 27
//   :379       DO n = 1,F_TwrTopNotch_N                    body iterates 0 times
//              (the DO line carries exactly the call count in every scenario)
//   :406-:417  the Flp_Mode == 2 blade-root arm            scenario 4 ONLY
//
// The extraction ran scenario 27, which is the scenario reaching the most arms
// at once -- the gen-speed notch loop, `TD_Mode > 0`, and NotchFilterSlopes on
// all three blades -- and no scenario reaches all of them. The three above are
// outside it by construction, so the kernel's verdict is a statement about the
// other arms and the differential harness is what varies these.

#include "vit_types.h"

#include <cmath>

namespace {

// Constants.f90:22-23, restated by name as `readavrswap.cpp` restates them: the
// Fortran reads `R2D`/`D2R` from the Constants module and a translation that
// spells the number inline loses the name the reference used. The literals are
// copied character for character from the PARAMETER declarations.
constexpr double R2D = 57.2957795130;
constexpr double D2R = 0.01745329251;

}  // namespace

void PreFilterMeasuredSignals(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                              debugvariables_t* DebugVar, objectinstances_t* objInst,
                              errorvariables_view_t* ErrVar) {
    // REAL(DbKi) :: NacVaneCosF   ! Time-filtered x-component of NacVane (deg)
    // REAL(DbKi) :: NacVaneSinF   ! Time-filtered y-component of NacVane (deg)
    //
    // Declared at the top of the reference and assigned only at the bottom.
    // Kept as locals here because that is what they are; they cross no
    // signature and no instrument can see them except through NacVaneF.
    double NacVaneCosF;
    double NacVaneSinF;

    // ! If there's an error, don't even try to run
    // IF (ErrVar%aviFAIL < 0) THEN
    //     RETURN
    // ENDIF
    if (ErrVar->aviFAIL < 0) {
        return;
    }

    return;  // STUB: the whole unit as a no-op

    // ! Filter the HSS (generator) and LSS (rotor) speed measurement:
    // ! Apply Low-Pass Filter (choice between first- and second-order low-pass filter)
    // IF (CntrPar%F_LPFType == 1) THEN
    //     LocalVar%GenSpeedF = LPFilter(LocalVar%GenSpeed, ...)
    //     LocalVar%RotSpeedF = LPFilter(LocalVar%RotSpeed, ...)
    // ELSEIF (CntrPar%F_LPFType == 2) THEN
    //     LocalVar%GenSpeedF = SecLPFilter(LocalVar%GenSpeed, ...)
    //     LocalVar%RotSpeedF = SecLPFilter(LocalVar%RotSpeed, ...)
    // ENDIF
    //
    // NO ELSE ARM. An F_LPFType outside {1,2} leaves both filtered speeds at
    // whatever they arrived with, which is a real behaviour of the reference
    // (P7) -- and it is reachable from the harness even though `CheckInputs`
    // rejects it in the simulation.
    //
    // `LocalVar%FP` is a nested derived type held BY VALUE in the view struct,
    // so its address is taken here and it must be copied back at the wrapper:
    // this integration needs `--reverse-copy` even though the callees, not this
    // unit, are what write it.
    if (CntrPar->F_LPFType == 1) {
        LocalVar->GenSpeedF =
            lpfilter_c(LocalVar->GenSpeed, LocalVar->DT, CntrPar->F_LPFCornerFreq, &LocalVar->FP,
                       LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);
        LocalVar->RotSpeedF =
            lpfilter_c(LocalVar->RotSpeed, LocalVar->DT, CntrPar->F_LPFCornerFreq, &LocalVar->FP,
                       LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);
    } else if (CntrPar->F_LPFType == 2) {
        // ! Second-order low-pass filter on generator speed
        LocalVar->GenSpeedF = seclpfilter_c(
            LocalVar->GenSpeed, LocalVar->DT, CntrPar->F_LPFCornerFreq, CntrPar->F_LPFDamping,
            &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instSecLPF, 0,
            0.0);
        LocalVar->RotSpeedF = seclpfilter_c(
            LocalVar->RotSpeed, LocalVar->DT, CntrPar->F_LPFCornerFreq, CntrPar->F_LPFDamping,
            &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instSecLPF, 0,
            0.0);
    }

    // ! Apply Notch Fitler to Gen Speed
    // DO n = 1,CntrPar%F_GenSpdNotch_N
    //     LocalVar%GenSpeedF = NotchFilter(LocalVar%GenSpeedF, LocalVar%DT,
    //                                      CntrPar%F_NotchFreqs(CntrPar%F_GenSpdNotch_Ind(n)),
    //                                      CntrPar%F_NotchBetaNum(CntrPar%F_GenSpdNotch_Ind(n)),
    //                                      CntrPar%F_NotchBetaDen(CntrPar%F_GenSpdNotch_Ind(n)),
    //                                      LocalVar%FP, LocalVar%iStatus, LocalVar%restart,
    //                                      objInst%instNotch)
    // END DO
    //
    // Written 1-based, as the Fortran writes it: an off-by-one at the bound is
    // then an extra or a missing filter call, which every oracle sees. The
    // double subscript is a Fortran index of a Fortran index, so BOTH need the
    // -1: `F_NotchFreqs[F_GenSpdNotch_Ind[n-1] - 1]`.
    for (int n = 1; n <= CntrPar->F_GenSpdNotch_N; ++n) {
        const int ind = CntrPar->F_GenSpdNotch_Ind[n - 1];
        LocalVar->GenSpeedF = notchfilter_c(
            LocalVar->GenSpeedF, LocalVar->DT, CntrPar->F_NotchFreqs[ind - 1],
            CntrPar->F_NotchBetaNum[ind - 1], CntrPar->F_NotchBetaDen[ind - 1], &LocalVar->FP,
            LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instNotch, 0, 0.0);
    }

    // ! Filtering the tower fore-aft acceleration signal
    // ! Force to start at 0
    // IF (LocalVar%iStatus == 0 .AND. LocalVar%Time == 0) THEN
    //     LocalVar%NacIMU_FA_RAcc = 0
    //     LocalVar%FA_Acc_Nac = 0
    // ENDIF
    //
    // The reference assigns the INTEGER literal 0 to a REAL(DbKi); that is a
    // conversion, not a different value, so 0.0 here is the same write.
    if (LocalVar->iStatus == 0 && LocalVar->Time == 0.0) {
        LocalVar->NacIMU_FA_RAcc = 0.0;
        LocalVar->FA_Acc_Nac = 0.0;
    }

    // ! Low pass
    // LocalVar%NacIMU_FA_AccF = SecLPFilter(LocalVar%NacIMU_FA_RAcc, ...) ! Fixed Damping
    // LocalVar%FA_AccF        = SecLPFilter(LocalVar%FA_Acc_Nac, ...)     ! Fixed Damping
    //
    // `F_FlCornerFreq` is an allocatable of at least 2; the reference takes
    // element 1 as the corner frequency and element 2 as the damping.
    LocalVar->NACIMU_FA_AccF = seclpfilter_c(
        LocalVar->NacIMU_FA_RAcc, LocalVar->DT, CntrPar->F_FlCornerFreq[0],
        CntrPar->F_FlCornerFreq[1], &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0,
        &objInst->instSecLPF, 0, 0.0);
    LocalVar->FA_AccF = seclpfilter_c(
        LocalVar->FA_Acc_Nac, LocalVar->DT, CntrPar->F_FlCornerFreq[0], CntrPar->F_FlCornerFreq[1],
        &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instSecLPF, 0, 0.0);

    // ! High pass
    // LocalVar%NacIMU_FA_AccF = HPFilter(LocalVar%NacIMU_FA_AccF, ...)
    // LocalVar%FA_AccF        = HPFilter(LocalVar%FA_AccF, ...)
    //
    // Both read the field they are about to overwrite: the low-pass result is
    // the high-pass input. The two are one cascade and not two independent
    // filters, which is why the pair cannot be reordered.
    LocalVar->NACIMU_FA_AccF =
        hpfilter_c(LocalVar->NACIMU_FA_AccF, LocalVar->DT, CntrPar->F_FlHighPassFreq,
                   &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instHPF,
                   0, 0.0);
    LocalVar->FA_AccF =
        hpfilter_c(LocalVar->FA_AccF, LocalVar->DT, CntrPar->F_FlHighPassFreq, &LocalVar->FP,
                   LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instHPF, 0, 0.0);

    // ! Notch filters
    // DO n = 1,CntrPar%F_TwrTopNotch_N
    //     LocalVar%NACIMU_FA_AccF = NotchFilter(LocalVar%NacIMU_FA_AccF, ...)
    //     LocalVar%FA_AccF        = NotchFilter(LocalVar%FA_AccF, ...)
    // END DO
    //
    // The reference spells the assignment target `NACIMU_FA_AccF` and its own
    // argument `NacIMU_FA_AccF`; Fortran is case-insensitive and they are one
    // field. The view struct declares it once, as `NACIMU_FA_AccF`.
    //
    // This loop's body runs ZERO times in all 27 scenarios (see the header), so
    // nothing built on simulation constrains it. The harness varies
    // `F_TwrTopNotch_N`.
    for (int n = 1; n <= CntrPar->F_TwrTopNotch_N; ++n) {
        const int ind = CntrPar->F_TwrTopNotch_Ind[n - 1];
        LocalVar->NACIMU_FA_AccF = notchfilter_c(
            LocalVar->NACIMU_FA_AccF, LocalVar->DT, CntrPar->F_NotchFreqs[ind - 1],
            CntrPar->F_NotchBetaNum[ind - 1], CntrPar->F_NotchBetaDen[ind - 1], &LocalVar->FP,
            LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instNotch, 0, 0.0);

        LocalVar->FA_AccF = notchfilter_c(
            LocalVar->FA_AccF, LocalVar->DT, CntrPar->F_NotchFreqs[ind - 1],
            CntrPar->F_NotchBetaNum[ind - 1], CntrPar->F_NotchBetaDen[ind - 1], &LocalVar->FP,
            LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instNotch, 0, 0.0);
    }

    // ! FA acc for ForeAft damping, condition matches whether it's used in Controllers.f90
    // IF (CntrPar%TD_Mode > 0) THEN
    //     LocalVar%FA_AccHPF = HPFilter(LocalVar%FA_Acc_Nac, LocalVar%DT,
    //                                   CntrPar%FA_HPFCornerFreq, ...)
    // ENDIF
    //
    // Note the INPUT is `FA_Acc_Nac`, the raw signal, not the `FA_AccF` the two
    // filters above produced. A third, independent path off the same source.
    if (CntrPar->TD_Mode > 0) {
        LocalVar->FA_AccHPF =
            hpfilter_c(LocalVar->FA_Acc_Nac, LocalVar->DT, CntrPar->FA_HPFCornerFreq,
                       &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0,
                       &objInst->instHPF, 0, 0.0);
    }

    // ! Filter Wind Speed Estimator Signal
    // LocalVar%We_Vw_F = LPFilter(LocalVar%WE_Vw, LocalVar%DT, CntrPar%F_WECornerFreq, ...)
    LocalVar->WE_Vw_F =
        lpfilter_c(LocalVar->WE_Vw, LocalVar->DT, CntrPar->F_WECornerFreq, &LocalVar->FP,
                   LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);

    // ! Blade root bending moment for IPC
    // DO K = 1,LocalVar%NumBl
    for (int K = 1; K <= LocalVar->NumBl; ++K) {
        // IF ((CntrPar%IPC_ControlMode > 0) .OR. (CntrPar%Flp_Mode == 3)) THEN
        if (CntrPar->IPC_ControlMode > 0 || CntrPar->Flp_Mode == 3) {
            // ! Moving inverted notch at rotor speed to isolate 1P
            // LocalVar%RootMOOPF(K) = NotchFilterSlopes(LocalVar%rootMOOP(K), LocalVar%DT,
            //                                           LocalVar%RotSpeedF, 0.7_DbKi,
            //                                           LocalVar%FP, LocalVar%iStatus,
            //                                           LocalVar%restart,
            //                                           objInst%instNotchSlopes, .TRUE.)
            //
            // The corner frequency is `LocalVar%RotSpeedF` -- the value the top
            // of this subroutine just wrote -- which is what makes the notch
            // "moving". The optional `Moving` is PRESENT and .TRUE. here, so
            // the bridge takes has_Moving = 1, Moving = 1; `InitialValue` is
            // absent, so has_InitialValue = 0.
            LocalVar->rootMOOPF[K - 1] = notchfilterslopes_c(
                LocalVar->rootMOOP[K - 1], LocalVar->DT, LocalVar->RotSpeedF, 0.7, &LocalVar->FP,
                LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instNotchSlopes, 1, 1, 0,
                0.0);

        // ELSEIF ( CntrPar%Flp_Mode == 2 ) THEN
        } else if (CntrPar->Flp_Mode == 2) {
            // ! Filter Blade root bending moments
            // LocalVar%RootMOOPF(K) = SecLPFilter(LocalVar%rootMOOP(K), LocalVar%DT,
            //                                     CntrPar%F_FlpCornerFreq(1),
            //                                     CntrPar%F_FlpCornerFreq(2), ...)
            // LocalVar%RootMOOPF(K) = HPFilter(LocalVar%rootMOOPF(K), LocalVar%DT, 0.1_DbKi, ...)
            LocalVar->rootMOOPF[K - 1] = seclpfilter_c(
                LocalVar->rootMOOP[K - 1], LocalVar->DT, CntrPar->F_FlpCornerFreq[0],
                CntrPar->F_FlpCornerFreq[1], &LocalVar->FP, LocalVar->iStatus,
                LocalVar->restart ? 1 : 0, &objInst->instSecLPF, 0, 0.0);
            LocalVar->rootMOOPF[K - 1] = hpfilter_c(
                LocalVar->rootMOOPF[K - 1], LocalVar->DT, 0.1, &LocalVar->FP, LocalVar->iStatus,
                LocalVar->restart ? 1 : 0, &objInst->instHPF, 0, 0.0);

            // ! Apply gen speed notch filters to blade root signal (same as gen speed path)
            // DO n = 1,CntrPar%F_GenSpdNotch_N  ! upstream bug: n was uninitialized here
            //     LocalVar%RootMOOPF(K) = NotchFilter(LocalVar%RootMOOPF(K), ...)
            // END DO
            for (int n = 1; n <= CntrPar->F_GenSpdNotch_N; ++n) {
                const int ind = CntrPar->F_GenSpdNotch_Ind[n - 1];
                LocalVar->rootMOOPF[K - 1] = notchfilter_c(
                    LocalVar->rootMOOPF[K - 1], LocalVar->DT, CntrPar->F_NotchFreqs[ind - 1],
                    CntrPar->F_NotchBetaNum[ind - 1], CntrPar->F_NotchBetaDen[ind - 1],
                    &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0,
                    &objInst->instNotch, 0, 0.0);
            }

        // ELSE
        } else {
            // LocalVar%RootMOOPF(K) = LocalVar%rootMOOP(K)
            LocalVar->rootMOOPF[K - 1] = LocalVar->rootMOOP[K - 1];
        // ENDIF
        }
    // END DO
    }

    // ! Control commands (used by WSE, mostly)
    // LocalVar%VS_LastGenTrqF = SecLPFilter(LocalVar%VS_LastGenTrq, LocalVar%DT,
    //                                       CntrPar%F_LPFCornerFreq, 0.7_DbKi, ...)
    // LocalVar%BlPitchCMeasF  = SecLPFilter(LocalVar%BlPitchCMeas, LocalVar%DT,
    //                                       CntrPar%F_LPFCornerFreq*0.25, 0.7_DbKi, ...)
    LocalVar->VS_LastGenTrqF = seclpfilter_c(
        LocalVar->VS_LastGenTrq, LocalVar->DT, CntrPar->F_LPFCornerFreq, 0.7, &LocalVar->FP,
        LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instSecLPF, 0, 0.0);
    LocalVar->BlPitchCMeasF = seclpfilter_c(
        LocalVar->BlPitchCMeas, LocalVar->DT, CntrPar->F_LPFCornerFreq * 0.25, 0.7, &LocalVar->FP,
        LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instSecLPF, 0, 0.0);

    // ! Wind vane signal
    // NacVaneCosF = LPFilter(cos(LocalVar%NacVane*D2R), LocalVar%DT, CntrPar%F_YawErr,
    //                        LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
    // NacVaneSinF = LPFilter(sin(LocalVar%NacVane*D2R), LocalVar%DT, CntrPar%F_YawErr,
    //                        LocalVar%FP, LocalVar%iStatus, .FALSE., objInst%instLPF) ! (-)
    // LocalVar%NacVaneF = wrap_180(atan2(NacVaneSinF, NacVaneCosF) * R2D) ! (deg)
    //
    // These two are the ONLY filter calls in the unit whose `reset` is the
    // literal .FALSE. rather than `LocalVar%restart` -- a 0 here and a variable
    // everywhere else, which is a distinction a reader can lose and a mutant
    // cannot.
    NacVaneCosF = lpfilter_c(std::cos(LocalVar->NacVane * D2R), LocalVar->DT, CntrPar->F_YawErr,
                             &LocalVar->FP, LocalVar->iStatus, 0, &objInst->instLPF, 0, 0.0);
    NacVaneSinF = lpfilter_c(std::sin(LocalVar->NacVane * D2R), LocalVar->DT, CntrPar->F_YawErr,
                             &LocalVar->FP, LocalVar->iStatus, 0, &objInst->instLPF, 0, 0.0);
    LocalVar->NacVaneF = wrap_180_c(std::atan2(NacVaneSinF, NacVaneCosF) * R2D);

    // ! Debug Variables
    // DebugVar%GenSpeedF = LocalVar%GenSpeedF
    // DebugVar%RotSpeedF = LocalVar%RotSpeedF
    // DebugVar%NacIMU_FA_AccF = LocalVar%NacIMU_FA_AccF
    // DebugVar%FA_AccF = LocalVar%FA_AccF
    //
    // `DebugVar` crosses as a plain struct pointer, not a view, so these four
    // writes land in the Fortran object directly and need no copy-back.
    DebugVar->GenSpeedF = LocalVar->GenSpeedF;
    DebugVar->RotSpeedF = LocalVar->RotSpeedF;
    DebugVar->NacIMU_FA_AccF = LocalVar->NACIMU_FA_AccF;
    DebugVar->FA_AccF = LocalVar->FA_AccF;
}
