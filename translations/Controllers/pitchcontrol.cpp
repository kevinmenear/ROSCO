// VIT Translation Scaffold
// Function: PitchControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 56aee7350bdb
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-20T18:44:05Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature, and the body is transcribed statement for statement.
//
// ELEVEN CALLEES, ALL ELEVEN ALREADY TRANSLATED AND ALL ELEVEN CALLED RATHER
// THAN INLINED (X1). This is the campaign's widest call fan-out:
//
//   interp1d          (#23)  x4 unconditional, +x0..3 on the OL_Mode arm
//   PIController      (#25)  x1 unconditional
//   IPC               (#52)  x0..1
//   ForeAftDamping    (#50)  x0..1
//   PitchSaturation   (#38)  x0..1
//   FloatingFeedback  (#51)  x0..1
//   saturate          (#24)  x1 unconditional, +x1..2 per blade, +x1 per blade
//   ratelimit         (#30)  x1 unconditional, +x1 per blade, +x1 per blade
//   ActiveWakeControl (#47)  x0..1
//   LPFilter          (#12)  x0..1 per blade  (PA_Mode == 1)
//   SecLPFilter       (#13)  x0..1 per blade  (PA_Mode == 2)
//
// FOUR OF THOSE CALLEES MUTATE `objInst` COUNTERS AND THE ORDER IS THEREFORE
// LOAD-BEARING. `PIController` post-increments `objInst%instPI`, `ratelimit`
// post-increments `objInst%instRL`, `LPFilter` `objInst%instLPF`,
// `SecLPFilter` `objInst%instSecLPF` -- and `IPC`, `ForeAftDamping`,
// `PitchSaturation` and `FloatingFeedback` each increment several of them
// internally. Every counter is a 1-based subscript into a `DIMENSION(1024)`
// member of `LocalVar%piP` / `%rlP` / `%FP`, so the sequence of calls decides
// which history slot each filter and each rate limiter owns. Reordering any
// two calls here is a different program even where the arithmetic commutes.
//
// ARM COVERAGE, read from coverage/line_coverage.json over all 27 scenarios,
// against the CLEAN source (54dd134) line numbers. 407,976 calls in 23 of the
// 27 scenarios, from one unguarded call site, DISCON.F90:119:
//
//   :47  PC_State == PC_State_Enabled     407,952   (the ELSE, :50, is 24)
//   :56  SU_Mode > 0 .AND. SU_Stage == -1       1   <- ONE call, in 27 scenarios
//   :72  IPC_ControlMode >= 1 .OR. Y_..== 2  63,995
//   :79  TD_Mode > 0                       63,997
//   :86  PS_Mode > 0                      391,977   (the ELSE, :90, is 15,999)
//   :95  Fl_Mode > 0                       79,996
//   :113 IPC_SatMode == 1                        0  <- NO SCENARIO
//   :126 OL_Mode > 0                             0  <- NO SCENARIO (body :127-139)
//   :143 AWC_Mode > 0                     119,993
//   :148 SD_Trigger == 0                  407,976   (the ELSE, :152, is 1,998)
//   :165 PA_Mode > 0                      167,991
//   :166 PA_Mode == 1                      47,997
//   :168 PA_Mode == 2                     119,994
//   :185 PF_Mode == 1                     159,992
//   :191 PF_Mode == 2                      63,996
//   :207 aviFAIL < 0                             3  <- three calls, in 27 scenarios
//
// So TWO of this unit's arms are outside everything the 27-scenario simulation
// gate drives -- the hard IPC saturation (`IPC_SatMode == 1`) and the whole
// open-loop pitch override (`OL_Mode > 0`, four statements including three
// more `interp1d` calls). Only the differential harness reaches them, and that
// asymmetry is the reason P11 is not optional here.

#include "vit_types.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// gfortran's two-argument `MAX` for REAL(8), TRANSCRIBED FROM WHAT IT DOES.
//
// COPIED BYTE FOR BYTE from `translations/ControllerBlocks/
// computevariablessetpoints.cpp` (unit #62), which established it (P4). NOT
// `std::max`, which is `(a < b) ? b : a` and therefore returns the FIRST
// operand both at a NaN and at a tie. gfortran lowers the MAX intrinsic to a
// MAX_EXPR, which on x86-64 is `maxsd`, and `maxsd`'s own definition returns
// the SECOND operand in exactly those two cases -- when either input is a NaN,
// and when the two are equal (which for `+0.0` against `-0.0` is a visible
// difference, since they are equal and differently signed).
//
// Unit #62 measured the difference on its own corpus: 3,300 of 25,398 cases,
// `MAX(NaN, x)` and `MAX(-0.0, +0.0)`. This unit has ONE `max` site, at :88,
// and it is on the 391,977-call path -- the most-exercised arm in the body.
//
// Written as the conditional rather than as `std::fmax`: `fmax` agrees at the
// NaN and is only *recommended* (not required) to prefer `+0.0` at the tie,
// while `(a > b) ? a : b` is decided by the language and reproduces both.
inline double fortran_max(double a, double b) { return (a > b) ? a : b; }

// Constants.f90:69-70. Copied as the reference declares them; the Fortran
// compares `LocalVar%PC_State` against this NAME, so the translation compares
// against the same value rather than against a bare literal -- a `const_tweak`
// on it is then a mutant on the same quantity the reference names.
constexpr int PC_State_Enabled = 1;

// CHARACTER(*), PARAMETER :: RoutineName = 'PitchControl'
//
// Twelve characters, the LONGEST `RoutineName` reached on any error path in
// this campaign, so the prefix this unit writes is thirteen bytes. See the
// composition note at the ErrMsg tail: four callees on this path prefix the
// same field before it does.
constexpr std::string_view RoutineName = "PitchControl";

// THE TWO HELPERS BELOW ARE COPIED FROM `translations/Controllers/ipc.cpp`
// (unit #52), which took them from `cablecontrol.cpp`, which took them from
// `structuralcontrol.cpp`, which took them from `pitchsaturation.cpp`, which
// took them from `interp1d.cpp` -- the unit that established the shape. They
// are not re-derived from prose (P4). The only edit is the diagnostic string,
// which names THIS unit so a reader of stderr knows which translation refused.
//
// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: PitchControl: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: PitchControl: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// TRIM(ErrVar%ErrMsg): trailing blanks only, off the field's CURRENT length.
// `find_last_not_of` rather than a hand-written backward scan, for unit #15's
// and unit #17's reason: a loop written `while (n > 0 && s[n-1] == ' ')` offers
// a `> 0` -> `>= 0` mutant that reads the byte BEFORE the buffer, which is
// undefined behaviour rather than a wrong answer and which no value comparison
// can be relied on to catch.
//
// A NEGATIVE length is the view's NOT-ALLOCATED convention, and it collapses to
// the same empty string a zero LENGTH gives -- correct for both, because the
// reference's `RoutineName//':'//TRIM(ErrMsg)` on a zero-length ErrMsg is
// exactly `'PitchControl:'`. No `== npos` branch: `find_last_not_of` returns
// `npos`, which is `SIZE_MAX`, and `npos + 1` is 0 by the defined wraparound of
// an unsigned type -- so the all-blank string falls out of the same expression.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

void PitchControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                  localvariables_view_t* LocalVar, objectinstances_t* objInst,
                  debugvariables_t* DebugVar, errorvariables_view_t* ErrVar) {
    // INTEGER(IntKi) :: K   ! Index used for looping through blades.
    //
    // Declared once at the top of the reference and reused by five separate DO
    // loops. It is NOT `SAVE`d (no initialiser) and every loop writes it before
    // reading it, so its value does not carry between loops and each loop gets
    // its own C++ `K`. The loops are written 1-based, the way the Fortran
    // writes them, which is what makes the loop-bound mutants observable
    // (RUNBOOK: "write the body the way the Fortran writes it").

    // ! ------- Blade Pitch Controller --------
    // ! Load PC State
    // IF (LocalVar%PC_State == PC_State_Enabled) THEN ! PI BldPitch control
    //     LocalVar%PC_MaxPit = CntrPar%PC_MaxPit
    // ELSE ! debug mode, fix at fine pitch
    //     LocalVar%PC_MaxPit = CntrPar%PC_FinePit
    // END IF
    if (LocalVar->PC_State == PC_State_Enabled) {
        LocalVar->PC_MaxPit = CntrPar->PC_MaxPit;
    } else {
        LocalVar->PC_MaxPit = CntrPar->PC_FinePit;
    }

    // ! Hold blade pitch at last value
    // ! If:
    // !   In pre-startup mode (before freewheeling)
    // IF ((CntrPar%SU_Mode > 0) .AND. (LocalVar%SU_Stage == -1)) THEN
    //     LocalVar%PC_MaxPit = LocalVar%BlPitchCMeas
    //     LocalVar%PC_MinPit = LocalVar%BlPitchCMeas
    // END IF
    //
    // THIS IS THE ONLY WRITE OF `PC_MinPit` BEFORE THE `PS_Mode` BLOCK BELOW
    // OVERWRITES IT ON EVERY PATH, so on its own it is a dead store -- but the
    // `PC_MaxPit` write beside it is not, and the mirror contract is about the
    // reference's behaviour rather than about the part of it some instrument
    // can distinguish (P7). Both stay. Coverage records this arm taken ONCE in
    // 407,976 calls across all 27 scenarios.
    //
    // `.AND.` in Fortran is not required to short-circuit and `&&` is; both
    // operands are pure struct reads with no side effect, so the two agree.
    if ((CntrPar->SU_Mode > 0) && (LocalVar->SU_Stage == -1)) {
        LocalVar->PC_MaxPit = LocalVar->BlPitchCMeas;
        LocalVar->PC_MinPit = LocalVar->BlPitchCMeas;
    }

    // ! Compute (interpolate) the gains based on previously commanded blade
    // ! pitch angles and lookup table:
    // LocalVar%PC_KP = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KP,
    //                           LocalVar%BlPitchCMeasF, ErrVar) ! Proportional
    // LocalVar%PC_KI = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KI, ...)
    // LocalVar%PC_KD = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_KD, ...)
    // LocalVar%PC_TF = interp1d(CntrPar%PC_GS_angles, CntrPar%PC_GS_TF, ...)
    //
    // Four calls with the SAME abscissa array and the SAME query point,
    // differing only in the ordinate. `interp1d`'s deferred-shape dummies
    // become a pointer plus its extent across the bridge, and the extents are
    // the four fields' OWN -- ROSCO's reader allocates all five at
    // `PC_GS_n` (ReadSetParameters.f90), but nothing here asserts that, so
    // each array carries its own `n_` rather than sharing one.
    LocalVar->PC_KP = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                 CntrPar->PC_GS_KP, CntrPar->n_PC_GS_KP,
                                 LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_KI = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                 CntrPar->PC_GS_KI, CntrPar->n_PC_GS_KI,
                                 LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_KD = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                 CntrPar->PC_GS_KD, CntrPar->n_PC_GS_KD,
                                 LocalVar->BlPitchCMeasF, ErrVar);
    LocalVar->PC_TF = interp1d_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles,
                                 CntrPar->PC_GS_TF, CntrPar->n_PC_GS_TF,
                                 LocalVar->BlPitchCMeasF, ErrVar);

    // ! Compute the collective pitch command associated with the proportional
    // ! and integral gains:
    // LocalVar%PC_PitComT = PIController(LocalVar%PC_SpdErr, LocalVar%PC_KP,
    //     LocalVar%PC_KI, LocalVar%PC_MinPit, LocalVar%PC_MaxPit, LocalVar%DT,
    //     LocalVar%BlPitch(1), LocalVar%piP, LocalVar%restart, objInst%instPI)
    //
    // `LocalVar%PC_MinPit` IS READ HERE AND IS WRITTEN AFTERWARDS. The value
    // this call sees is the one left by the `SU_Mode` block above, or --
    // because `LocalVar` persists across DLL calls -- whatever the PREVIOUS
    // timestep's `PS_Mode` block stored. It is the reference's own ordering and
    // it is transcribed, not repaired.
    //
    // `I0` is `LocalVar%BlPitch(1)`, the FIRST blade's measured pitch, which is
    // the integrator's reset value. `LocalVar%restart` is a Fortran LOGICAL
    // crossing into the `int32_t` the bridge declares; `? 1 : 0` is the
    // campaign's standing spelling.
    LocalVar->PC_PitComT = picontroller_c(LocalVar->PC_SpdErr, LocalVar->PC_KP,
                                          LocalVar->PC_KI, LocalVar->PC_MinPit,
                                          LocalVar->PC_MaxPit, LocalVar->DT,
                                          LocalVar->BlPitch[0], &LocalVar->piP,
                                          LocalVar->restart ? 1 : 0, &objInst->instPI);
    // DebugVar%PC_PICommand = LocalVar%PC_PitComT
    DebugVar->PC_PICommand = LocalVar->PC_PitComT;

    // ! Find individual pitch control contribution
    // IF ((CntrPar%IPC_ControlMode >= 1) .OR. (CntrPar%Y_ControlMode == 2)) THEN
    //     CALL IPC(CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    // ELSE
    //     LocalVar%IPC_PitComF = 0.0 ! THIS IS AN ARRAY!!
    // END IF
    //
    // The reference's own comment is right and the extent is the DECLARED one.
    // `LocalVar%IPC_PitComF` is `REAL(DbKi), DIMENSION(3)` in ROSCO_Types, so
    // the whole-array assignment writes THREE elements regardless of
    // `LocalVar%NumBl` -- which is the loop bound used a few statements down.
    // A loop to `NumBl` here would leave element 3 unwritten on a two-bladed
    // rotor, and the difference is observable through `PitCom(3)` and hence
    // `avrSWAP(44)`. Three, literally.
    //
    // `0.0` is a default-REAL literal under `-fdefault-real-8`, so it is the
    // REAL(8) zero and the assignment is exact either way.
    if ((CntrPar->IPC_ControlMode >= 1) || (CntrPar->Y_ControlMode == 2)) {
        ipc_c(CntrPar, LocalVar, objInst, DebugVar, ErrVar);
    } else {
        for (int K = 1; K <= 3; ++K) {
            LocalVar->IPC_PitComF[K - 1] = 0.0;
        }
    }

    // ! Include tower fore-aft tower vibration damping control
    // IF (CntrPar%TD_Mode > 0) THEN
    //     CALL ForeAftDamping(CntrPar, LocalVar, objInst)
    // ELSE
    //     LocalVar%FA_PitCom = 0.0 ! THIS IS AN ARRAY!!
    // ENDIF
    //
    // Same shape as the block above, same declared extent 3.
    if (CntrPar->TD_Mode > 0) {
        foreaftdamping_c(CntrPar, LocalVar, objInst);
    } else {
        for (int K = 1; K <= 3; ++K) {
            LocalVar->FA_PitCom[K - 1] = 0.0;
        }
    }

    // ! Pitch Saturation
    // IF (CntrPar%PS_Mode > 0) THEN
    //     LocalVar%PC_MinPit = PitchSaturation(LocalVar,CntrPar,objInst,DebugVar, ErrVar)
    //     LocalVar%PC_MinPit = max(LocalVar%PC_MinPit, CntrPar%PC_FinePit)
    // ELSE
    //     LocalVar%PC_MinPit = CntrPar%PC_FinePit
    // ENDIF
    //
    // `max` is `fortran_max`, the helper at the top of this file, and NOT
    // `std::max`. This is the unit's only two-argument MAX and it sits on the
    // 391,977-call arm.
    if (CntrPar->PS_Mode > 0) {
        LocalVar->PC_MinPit = pitchsaturation_c(LocalVar, CntrPar, objInst, DebugVar, ErrVar);
        LocalVar->PC_MinPit = fortran_max(LocalVar->PC_MinPit, CntrPar->PC_FinePit);
    } else {
        LocalVar->PC_MinPit = CntrPar->PC_FinePit;
    }
    // DebugVar%PC_MinPit = LocalVar%PC_MinPit
    DebugVar->PC_MinPit = LocalVar->PC_MinPit;

    // ! FloatingFeedback
    // IF (CntrPar%Fl_Mode > 0) THEN
    //     LocalVar%Fl_PitCom = FloatingFeedback(LocalVar, CntrPar, objInst, ErrVar)
    //     DebugVar%FL_PitCom = LocalVar%Fl_PitCom
    //     LocalVar%PC_PitComT = LocalVar%PC_PitComT + LocalVar%Fl_PitCom
    // ENDIF
    //
    // `DebugVar%FL_PitCom` and the struct's `Fl_PitCom` are the same field:
    // Fortran identifiers are case-insensitive and the reference spells it with
    // a capital L here and a lower-case one in ROSCO_Types. The C struct has
    // one spelling.
    //
    // NO `ELSE`. On `Fl_Mode == 0` `LocalVar%Fl_PitCom` keeps whatever the
    // previous call left in it -- it is a persistent field, not a local -- and
    // `DebugVar%FL_PitCom` is not written at all on that path. Both are the
    // reference's behaviour.
    if (CntrPar->Fl_Mode > 0) {
        LocalVar->Fl_PitCom = floatingfeedback_c(LocalVar, CntrPar, objInst, ErrVar);
        DebugVar->Fl_PitCom = LocalVar->Fl_PitCom;
        LocalVar->PC_PitComT = LocalVar->PC_PitComT + LocalVar->Fl_PitCom;
    }

    // ! Saturate collective pitch commands:
    // LocalVar%PC_PitComT = saturate(LocalVar%PC_PitComT, LocalVar%PC_MinPit,
    //                                CntrPar%PC_MaxPit)
    //
    // THE UPPER BOUND IS `CntrPar%PC_MaxPit`, THE PARAMETER, NOT
    // `LocalVar%PC_MaxPit`, THE FIELD THE FIRST BLOCK OF THIS ROUTINE JUST
    // COMPUTED. The two differ on the `PC_State /= Enabled` arm and on the
    // `SU_Mode` arm, and the reference uses the parameter at every saturate
    // call in this routine while using the LocalVar copy only as
    // `PIController`'s upper limit. Transcribed as written.
    LocalVar->PC_PitComT =
        saturate_c(LocalVar->PC_PitComT, LocalVar->PC_MinPit, CntrPar->PC_MaxPit);
    // LocalVar%PC_PitComT = ratelimit(LocalVar%PC_PitComT, CntrPar%PC_MinRat,
    //     CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,
    //     objInst%instRL, LocalVar%BlPitchCMeas)
    //
    // EIGHT actual arguments, so `ResetValue` IS supplied and the bridge's
    // `has_ResetValue` is 1 with `LocalVar%BlPitchCMeas` in the value slot.
    // (Contrast `variablespeedcontrol.cpp:454`, which passes seven and gets a
    // 0 flag.) The reset value is the MEASURED collective pitch here and the
    // per-blade `BlPitch(K)` at the two calls inside the loops below.
    LocalVar->PC_PitComT = ratelimit_c(LocalVar->PC_PitComT, CntrPar->PC_MinRat,
                                       CntrPar->PC_MaxRat, LocalVar->DT,
                                       LocalVar->restart ? 1 : 0, &LocalVar->rlP,
                                       &objInst->instRL, 1, LocalVar->BlPitchCMeas);
    // LocalVar%PC_PitComT_Last = LocalVar%PC_PitComT
    LocalVar->PC_PitComT_Last = LocalVar->PC_PitComT;

    // ! Combine and saturate all individual pitch commands in software
    // DO K = 1,LocalVar%NumBl
    for (int K = 1; K <= LocalVar->NumBl; ++K) {
        // LocalVar%PitCom(K) = LocalVar%PC_PitComT + LocalVar%FA_PitCom(K)
        LocalVar->PitCom[K - 1] = LocalVar->PC_PitComT + LocalVar->FA_PitCom[K - 1];
        // LocalVar%PitCom(K) = saturate(LocalVar%PitCom(K), LocalVar%PC_MinPit,
        //                               CntrPar%PC_MaxPit)
        LocalVar->PitCom[K - 1] =
            saturate_c(LocalVar->PitCom[K - 1], LocalVar->PC_MinPit, CntrPar->PC_MaxPit);
        // LocalVar%PitCom(K) = LocalVar%PitCom(K) + LocalVar%IPC_PitComF(K)
        LocalVar->PitCom[K - 1] = LocalVar->PitCom[K - 1] + LocalVar->IPC_PitComF[K - 1];

        // ! Hard IPC saturation by peak shaving limit
        // IF (CntrPar%IPC_SatMode == 1) THEN
        //     LocalVar%PitCom(K) = saturate(LocalVar%PitCom(K),
        //         LocalVar%PC_MinPit, CntrPar%PC_MaxPit)
        // END IF
        //
        // A SECOND saturate with the IDENTICAL three arguments as the one four
        // lines up -- and it is NOT a no-op, because the addition of
        // `IPC_PitComF(K)` sits between them and can push the sum back outside
        // the window. Zero hits in all 27 scenarios; only the differential
        // harness reaches this.
        if (CntrPar->IPC_SatMode == 1) {
            LocalVar->PitCom[K - 1] =
                saturate_c(LocalVar->PitCom[K - 1], LocalVar->PC_MinPit, CntrPar->PC_MaxPit);
        }

        // ! Add ZeroMQ pitch commands
        // LocalVar%PitCom(K) = LocalVar%PitCom(K) + LocalVar%ZMQ_PitOffset(K)
        LocalVar->PitCom[K - 1] = LocalVar->PitCom[K - 1] + LocalVar->ZMQ_PitOffset[K - 1];

        // ! Rate limit
        // LocalVar%PitCom(K) = ratelimit(LocalVar%PitCom(K), CntrPar%PC_MinRat,
        //     CntrPar%PC_MaxRat, LocalVar%DT, LocalVar%restart, LocalVar%rlP,
        //     objInst%instRL, LocalVar%BlPitch(K))
        LocalVar->PitCom[K - 1] = ratelimit_c(LocalVar->PitCom[K - 1], CntrPar->PC_MinRat,
                                              CntrPar->PC_MaxRat, LocalVar->DT,
                                              LocalVar->restart ? 1 : 0, &LocalVar->rlP,
                                              &objInst->instRL, 1, LocalVar->BlPitch[K - 1]);
    // END DO
    }

    // ! Open Loop control, use if
    // !   Open loop mode active         Using OL blade pitch control
    // IF (CntrPar%OL_Mode > 0) THEN
    //     IF (LocalVar%Time >= CntrPar%OL_Breakpoints(1)) THEN
    //         IF (CntrPar%Ind_BldPitch(1) > 0) THEN
    //             LocalVar%PitCom(1) = interp1d(CntrPar%OL_Breakpoints,
    //                 CntrPar%OL_BldPitch1, LocalVar%OL_Index, ErrVar)
    //         ENDIF
    //         ... 2, 3 ...
    //     ENDIF
    // ENDIF
    //
    // THREE NESTED IFs AND FOUR STATEMENTS THAT NO SCENARIO REACHES. The
    // abscissa is `OL_Breakpoints` and the query point is `LocalVar%OL_Index`
    // -- which is an INDEX into the open-loop table, not a time, and the
    // reference interpolates the pitch series against the breakpoints at it
    // anyway. Transcribed as written (P7): the oracle is the source, not what
    // the source appears to have meant.
    //
    // The subscripts `OL_Breakpoints(1)` and `Ind_BldPitch(1..3)` are on
    // ALLOCATABLE fields with no ALLOCATED test and no bound check in the
    // reference, so an extent below the subscript is an out-of-bounds read in
    // the REFERENCE. `harness/ranges.toml` states the extents rather than
    // letting the generator draw them.
    if (CntrPar->OL_Mode > 0) {
        if (LocalVar->Time >= CntrPar->OL_Breakpoints[0]) {
            if (CntrPar->Ind_BldPitch[0] > 0) {
                LocalVar->PitCom[0] =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_BldPitch1, CntrPar->n_OL_BldPitch1,
                               LocalVar->OL_Index, ErrVar);
            }

            if (CntrPar->Ind_BldPitch[1] > 0) {
                LocalVar->PitCom[1] =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_BldPitch2, CntrPar->n_OL_BldPitch2,
                               LocalVar->OL_Index, ErrVar);
            }

            if (CntrPar->Ind_BldPitch[2] > 0) {
                LocalVar->PitCom[2] =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_BldPitch3, CntrPar->n_OL_BldPitch3,
                               LocalVar->OL_Index, ErrVar);
            }
        }
    }

    // ! Active wake control
    // IF (CntrPar%AWC_Mode > 0) THEN
    //     CALL ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
    // ENDIF
    //
    // Note the argument ORDER: `DebugVar` before `objInst`, which is the
    // opposite of `IPC`'s. The `_c` signature carries the reference's order.
    if (CntrPar->AWC_Mode > 0) {
        activewakecontrol_c(CntrPar, LocalVar, DebugVar, objInst);
    }

    // ! Shutdown
    // IF (LocalVar%SD_Trigger == 0) THEN
    //     LocalVar%PitCom_SD = LocalVar%PitCom
    // ! If shutdown is not triggered, PitCom_SD tracks PitCom.
    // ELSE
    //     IF (CntrPar%SD_Method == 1 .OR. CntrPar%SD_Method == 2) THEN
    //         DO K = 1,LocalVar%NumBl
    //             LocalVar%PitCom_SD(K) = LocalVar%PitCom_SD(K)
    //                 + LocalVar%SD_MaxPitchRate*LocalVar%DT
    //         END DO
    //     ENDIF
    //     LocalVar%PitCom = LocalVar%PitCom_SD
    // ENDIF
    //
    // TWO WHOLE-ARRAY ASSIGNMENTS AT THE DECLARED EXTENT 3, and one accumulate
    // loop at `NumBl`. The asymmetry is the reference's: on the ELSE path
    // elements above `NumBl` are copied back into `PitCom` UNCHANGED from
    // whatever `PitCom_SD` holds, which on a two-bladed rotor is the third
    // element of the last state that wrote it. Both extents are transcribed
    // literally -- 3 for the copies, `NumBl` for the loop.
    if (LocalVar->SD_Trigger == 0) {
        for (int K = 1; K <= 3; ++K) {
            LocalVar->PitCom_SD[K - 1] = LocalVar->PitCom[K - 1];
        }
    } else {
        if (CntrPar->SD_Method == 1 || CntrPar->SD_Method == 2) {
            for (int K = 1; K <= LocalVar->NumBl; ++K) {
                LocalVar->PitCom_SD[K - 1] =
                    LocalVar->PitCom_SD[K - 1] + LocalVar->SD_MaxPitchRate * LocalVar->DT;
            }
        }
        for (int K = 1; K <= 3; ++K) {
            LocalVar->PitCom[K - 1] = LocalVar->PitCom_SD[K - 1];
        }
    }

    // ! Place pitch actuator here, so it can be used with or without open-loop
    // DO K = 1,LocalVar%NumBl
    //     IF (CntrPar%PA_Mode > 0) THEN
    //         IF (CntrPar%PA_Mode == 1) THEN
    //             LocalVar%PitComAct(K) = LPFilter(LocalVar%PitCom(K),
    //                 LocalVar%DT, CntrPar%PA_CornerFreq, LocalVar%FP,
    //                 LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
    //         ELSE IF (CntrPar%PA_Mode == 2) THEN
    //             LocalVar%PitComAct(K) = SecLPFilter(LocalVar%PitCom(K),
    //                 LocalVar%DT, CntrPar%PA_CornerFreq, CntrPar%PA_Damping,
    //                 LocalVar%FP, LocalVar%iStatus, LocalVar%restart,
    //                 objInst%instSecLPF)
    //         END IF
    //     ELSE
    //         LocalVar%PitComAct(K) = LocalVar%PitCom(K)
    //     ENDIF
    // END DO
    //
    // THE INNER CHAIN HAS NO `ELSE`: on `PA_Mode > 2` neither filter runs and
    // `PitComAct(K)` IS NOT WRITTEN AT ALL on that trip -- it keeps the value
    // the previous timestep's hardware-saturation loop left there. That hole is
    // the reference's and it is transcribed rather than closed.
    //
    // NEITHER filter is passed `InitialValue` (seven and eight actual arguments
    // against eight and nine dummies), so both presence flags are 0 and the
    // value slots are unread.
    //
    // ONE COUNTER PER MODE, INCREMENTED PER BLADE: `objInst%instLPF` advances
    // on each trip under mode 1 and `objInst%instSecLPF` under mode 2, so the
    // three blades own three consecutive filter-history slots and the mode
    // decides which array they come from.
    for (int K = 1; K <= LocalVar->NumBl; ++K) {
        if (CntrPar->PA_Mode > 0) {
            if (CntrPar->PA_Mode == 1) {
                LocalVar->PitComAct[K - 1] = lpfilter_c(
                    LocalVar->PitCom[K - 1], LocalVar->DT, CntrPar->PA_CornerFreq,
                    &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0,
                    &objInst->instLPF, 0, 0.0);
            } else if (CntrPar->PA_Mode == 2) {
                LocalVar->PitComAct[K - 1] = seclpfilter_c(
                    LocalVar->PitCom[K - 1], LocalVar->DT, CntrPar->PA_CornerFreq,
                    CntrPar->PA_Damping, &LocalVar->FP, LocalVar->iStatus,
                    LocalVar->restart ? 1 : 0, &objInst->instSecLPF, 0, 0.0);
            }
        } else {
            LocalVar->PitComAct[K - 1] = LocalVar->PitCom[K - 1];
        }
    }

    // ! Hardware saturation: using CntrPar%PC_MinPit
    // DO K = 1,LocalVar%NumBl
    //     ! Saturate the pitch command using the overall (hardware) limit
    //     LocalVar%PitComAct(K) = saturate(LocalVar%PitComAct(K),
    //         CntrPar%PC_MinPit, CntrPar%PC_MaxPit)
    //     ! Saturate the overall command of blade K using the pitch rate limit
    //     LocalVar%PitComAct(K) = ratelimit(LocalVar%PitComAct(K),
    //         CntrPar%PC_MinRat, CntrPar%PC_MaxRat, LocalVar%DT,
    //         LocalVar%restart, LocalVar%rlP, objInst%instRL, LocalVar%BlPitch(K))
    // END DO
    //
    // THE LOWER BOUND HERE IS `CntrPar%PC_MinPit`, THE HARDWARE PARAMETER, not
    // `LocalVar%PC_MinPit`, the pitch-saturation result the two saturate calls
    // in the first loop used. The reference's own comment says so and the two
    // are different quantities.
    for (int K = 1; K <= LocalVar->NumBl; ++K) {
        LocalVar->PitComAct[K - 1] =
            saturate_c(LocalVar->PitComAct[K - 1], CntrPar->PC_MinPit, CntrPar->PC_MaxPit);
        LocalVar->PitComAct[K - 1] =
            ratelimit_c(LocalVar->PitComAct[K - 1], CntrPar->PC_MinRat, CntrPar->PC_MaxRat,
                        LocalVar->DT, LocalVar->restart ? 1 : 0, &LocalVar->rlP,
                        &objInst->instRL, 1, LocalVar->BlPitch[K - 1]);
    }

    // ! Add pitch actuator fault for blade K
    // IF (CntrPar%PF_Mode == 1) THEN
    //     DO K = 1, LocalVar%NumBl
    //         ! This assumes that the pitch actuator fault overides the
    //         ! Hardware saturation
    //         LocalVar%PitComAct(K) = LocalVar%PitComAct(K) + CntrPar%PF_Offsets(K)
    //     END DO
    // ! Blade pitch stuck at last value
    // ELSEIF (CntrPar%PF_Mode == 2) THEN
    //     DO K = 1, LocalVar%NumBl
    //         IF (LocalVar%Time > CntrPar%PF_TimeStuck(K)) THEN
    //             LocalVar%PitComAct(K) = LocalVar%BlPitch(K)
    //         END IF
    //     END DO
    // END IF
    //
    // `PF_Offsets` and `PF_TimeStuck` are ALLOCATABLE and subscripted to
    // `NumBl` with no ALLOCATED test; ROSCO's reader allocates both at 3.
    // `harness/ranges.toml` states the extents.
    if (CntrPar->PF_Mode == 1) {
        for (int K = 1; K <= LocalVar->NumBl; ++K) {
            LocalVar->PitComAct[K - 1] = LocalVar->PitComAct[K - 1] + CntrPar->PF_Offsets[K - 1];
        }
    } else if (CntrPar->PF_Mode == 2) {
        for (int K = 1; K <= LocalVar->NumBl; ++K) {
            if (LocalVar->Time > CntrPar->PF_TimeStuck[K - 1]) {
                LocalVar->PitComAct[K - 1] = LocalVar->BlPitch[K - 1];
            }
        }
    }

    // ! Command the pitch demanded from the last
    // ! call to the controller (See Appendix A of Bladed User's Guide):
    // avrSWAP(42) = LocalVar%PitComAct(1)
    // avrSWAP(43) = LocalVar%PitComAct(2)
    // avrSWAP(44) = LocalVar%PitComAct(3)
    // avrSWAP(45) = LocalVar%PitComAct(1)
    //
    // `avrSWAP` is `REAL(ReKi)` and `ReKi` is `REAL(4)` (vit.yaml
    // `kind_aliases`), so each of these four is a NARROWING assignment from the
    // REAL(8) `PitComAct` -- one rounding to single precision, at the
    // boundary. `static_cast<float>` is the same conversion the Fortran
    // performs and is written explicitly so the narrowing is visible rather
    // than implicit.
    //
    // ALL THREE ELEMENTS ARE READ UNCONDITIONALLY while every loop above is
    // bounded by `NumBl`, so on a rotor with fewer than three blades this
    // publishes elements the routine did not write on this call. That is the
    // reference's behaviour.
    avrSWAP[41] = static_cast<float>(LocalVar->PitComAct[0]);
    avrSWAP[42] = static_cast<float>(LocalVar->PitComAct[1]);
    avrSWAP[43] = static_cast<float>(LocalVar->PitComAct[2]);
    avrSWAP[44] = static_cast<float>(LocalVar->PitComAct[0]);

    // ! Add RoutineName to error message
    // IF (ErrVar%aviFAIL < 0) THEN
    //     ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    // ENDIF
    //
    // *** COUNT THE STAGED ASSIGNMENTS ON THE PATH BEFORE READING A RED ON A
    // CHARACTER OUTPUT (unit #48's rule). *** `aviFAIL < 0` is a property of
    // the state, and four of this unit's callees test the same predicate and
    // prefix the same field before control reaches here:
    //
    //     interp1d x4       'interp1d:'          9 bytes each
    //     PitchSaturation   'PitchSaturation:'  16 bytes
    //     FloatingFeedback  'FloatingFeedback:' 17 bytes
    //     IPC               'IPC:'               4 bytes  (itself over 4x'sigma:')
    //     here              'PitchControl:'     13 bytes
    //
    // so with an entry message of trimmed length m and `aviFAIL < 0` on entry
    // the worst-case chain is m -> m+36 (four interp1d) -> ... -> m+13 more.
    // The reference chain is Fortran throughout with ONE capacity gate, in the
    // generated bridge, on the FINAL value; the translation chain gates at
    // every C boundary. Where the two disagree is a window in R13's capacity
    // ladder -- and it is measured rather than assumed: see
    // `harness/ranges.toml` `[PitchControl]` and `evidence/PitchControl/`.
    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }
}
