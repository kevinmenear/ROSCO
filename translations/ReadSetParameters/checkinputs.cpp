// VIT Translation Scaffold
// Function: CheckInputs
// Source: ReadSetParameters.f90
// Module: ReadSetParameters
// Fortran: SUBROUTINE CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 62a91cdbc20c
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-13T04:05:39Z

#include "vit_types.h"

#include <ISO_Fortran_binding.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace {

// CHARACTER(*), PARAMETER :: RoutineName = 'CheckInputs'
constexpr std::string_view RoutineName = "CheckInputs";

// Constants.f90:44-45. The two values `TRA_Mode > 1` compares VS_ControlMode
// against, read out of the source rather than recalled -- P7 reaches a named
// constant as well as an expression.
constexpr int VS_Mode_WSE_TSR = 2;
constexpr int VS_Mode_Power_TSR = 3;

// ErrVar%ErrMsg is CHARACTER(:), ALLOCATABLE. A Fortran assignment to it
// REALLOCATES it to exactly the length of the right-hand side -- it does not
// blank-fill a fixed width -- so `n_ErrMsg` is the new LEN and the bytes past
// it are not part of the value. Same helper shape as sigma/interp1d/unwrap.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: CheckInputs: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: CheckInputs: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    // AND CLEAR THE REST OF THE STAGING BUFFER.
    //
    // `ErrVar%ErrMsg` is CHARACTER(:), ALLOCATABLE. The assignment REALLOCATES
    // it to exactly LEN of the right-hand side, so in the reference there are
    // no bytes past the new length at all -- the previous message's storage is
    // gone. Leaving the previous message's tail in this buffer renders a value
    // the reference does not have.
    //
    // Invisible after integration, where the reverse copy reads only
    // `buffer(1:n_ErrMsg)` -- and the whole margin in the differential
    // harness, which compares the buffer. MEASURED, in both directions:
    // leaving the tail alone fails 16,729 of 16,769 and blank-filling it fails
    // 16,769 of 16,769, every one on `ErrVar.ErrMsg` and none on `n_ErrMsg`.
    // The oracle side is a freshly zeroed buffer that the bridge writes
    // `n_ErrMsg` bytes into, so the discarded region reads as NUL and not as
    // blank -- the first differing byte is `a=0x20 b=0x00` at exactly index
    // `n_ErrMsg`, which is what said so.
    if (ErrVar->n_ErrMsg_cap > static_cast<int32_t>(s.size())) {
        std::memset(ErrVar->ErrMsg + s.size(), 0,
                    static_cast<size_t>(ErrVar->n_ErrMsg_cap) - s.size());
    }
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// TRIM(ErrVar%ErrMsg): drop TRAILING blanks only.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    if (ErrVar->ErrMsg == nullptr || n <= 0) return std::string();
    const std::string_view v(ErrVar->ErrMsg, static_cast<size_t>(n));
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

// --- Fortran array intrinsics over the view's pointer+size pairs -------------
//
// Every array the reference reduces here is CntrPar's, i.e. ALLOCATABLE, and
// the view carries it as a pointer that is null when the Fortran component is
// unallocated. The zero-size answers are the STANDARD's, not a convenience:
// ANY of an empty set is .FALSE., ALL of an empty set is .TRUE., MAXVAL is
// -HUGE and MINVAL is +HUGE. Writing them out is what keeps an unallocated
// component from silently taking the other branch.
template <typename T>
bool any_lt(const T* a, int32_t n, T v) {
    if (a == nullptr) return false;
    for (int32_t i = 0; i < n; ++i)
        if (a[i] < v) return true;
    return false;
}

template <typename T>
bool any_le(const T* a, int32_t n, T v) {
    if (a == nullptr) return false;
    for (int32_t i = 0; i < n; ++i)
        if (a[i] <= v) return true;
    return false;
}

template <typename T>
bool any_gt(const T* a, int32_t n, T v) {
    if (a == nullptr) return false;
    for (int32_t i = 0; i < n; ++i)
        if (a[i] > v) return true;
    return false;
}

template <typename T>
bool all_lt(const T* a, int32_t n, T v) {
    if (a == nullptr) return true;
    for (int32_t i = 0; i < n; ++i)
        if (!(a[i] < v)) return false;
    return true;
}

double maxval(const double* a, int32_t n) {
    // MAXVAL of a zero-sized array is -HUGE(a) -- the identity of MAX, not 0.
    double r = -std::numeric_limits<double>::max();
    if (a == nullptr) return r;
    for (int32_t i = 0; i < n; ++i)
        if (a[i] > r) r = a[i];
    return r;
}

double minval(const double* a, int32_t n) {
    // MINVAL of a zero-sized array is +HUGE(a).
    double r = std::numeric_limits<double>::max();
    if (a == nullptr) return r;
    for (int32_t i = 0; i < n; ++i)
        if (a[i] < r) r = a[i];
    return r;
}

}  // namespace

void CheckInputs(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
                 float* avrSWAP, errorvariables_view_t* ErrVar, int size_avcMSG) {
    // size_avcMSG is declared INTENT(IN) and never read in the body. Named here
    // so the parameter is not an unused-argument warning and so the fact is on
    // the record rather than inferred from its absence.
    (void)size_avcMSG;

    //..............................................................................................................................
    // Check validity of input parameters:
    //..............................................................................................................................

    //------- DEBUG ------------------------------------------------------------

    // LoggingLevel
    if ((CntrPar->LoggingLevel < 0) || (CntrPar->LoggingLevel > 3)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "LoggingLevel must be 0 - 3.");
    }

    if (CntrPar->DT_Out <= 0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "DT_Out must be greater than 0");
    }

    if (CntrPar->DT_Out < LocalVar->DT) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "DT_Out must be greater than or equal to DT in OpenFAST");
    }

    // n_DT_Out is INTEGER(IntKi); the product is formed in REAL(DbKi) because
    // the other factor is.
    if (std::fabs(CntrPar->DT_Out - LocalVar->DT * static_cast<double>(CntrPar->n_DT_Out)) > 0.001) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "DT_Out must be a factor of DT in OpenFAST");
    }

    if (CntrPar->ZMQ_Mode > 0) {
        if (std::fabs(CntrPar->ZMQ_UpdatePeriod - LocalVar->DT * static_cast<double>(CntrPar->n_DT_ZMQ)) > 0.001) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "ZMQ_UpdatePeriod must be a factor of DT in OpenFAST");
        }
    }
    //------- CONTROLLER FLAGS -------------------------------------------------

    // F_LPFType
    if ((CntrPar->F_LPFType < 1) || (CntrPar->F_LPFType > 2)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "F_LPFType must be 1 or 2.");
    }

    // IPC_ControlMode
    if ((CntrPar->IPC_ControlMode < 0) || (CntrPar->IPC_ControlMode > 2)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "IPC_ControlMode must be 0, 1, or 2.");
    }

    // VS_ControlMode
    if ((CntrPar->VS_ControlMode < 0) || (CntrPar->VS_ControlMode > 4)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_ControlMode must be between 0 and 4.");
    }

    // VS_ConstPower
    if ((CntrPar->VS_ConstPower < 0) || (CntrPar->VS_ConstPower > 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_ConstPower must be 0 or 1.");
    }

    // VS_FBP
    if ((CntrPar->VS_FBP < 0) || (CntrPar->VS_FBP > 3)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_FBP must be between 0 and 3.");
    }
    if ((CntrPar->VS_FBP > 0) && (CntrPar->PC_ControlMode > 0)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_FBP and PC_ControlMode cannot both be greater than 0.");
    }

    if ((CntrPar->VS_FBP > 0) && (CntrPar->PRC_Mode > 0)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      "Fixed blade pitch control (VS_FBP) and power reference control (PRC_Mode) cannot both be enabled.");
    }

    if ((CntrPar->VS_FBP > 0) && (CntrPar->VS_ConstPower > 0)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      "Fixed blade pitch control (VS_FBP) and constant power torque control (VS_ConstPower) cannot both be enabled.");
    }

    // PC_ControlMode
    if ((CntrPar->PC_ControlMode < 0) || (CntrPar->PC_ControlMode > 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PC_ControlMode must be 0 or 1.");
    }

    // Y_ControlMode
    if ((CntrPar->Y_ControlMode < 0) || (CntrPar->Y_ControlMode > 2)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "Y_ControlMode must be 0, 1 or 2.");
    }

    if ((CntrPar->IPC_ControlMode > 0) && (CntrPar->Y_ControlMode > 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      "IPC control for load reductions and yaw-by-IPC cannot be activated simultaneously");
    }

    // SS_Mode
    if ((CntrPar->SS_Mode < 0) || (CntrPar->SS_Mode > 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "SS_Mode must be 0 or 1.");
    }

    // WE_Mode
    if ((CntrPar->WE_Mode < 0) || (CntrPar->WE_Mode > 2)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "WE_Mode must be 0, 1, or 2.");
    }

    // PS_Mode
    if ((CntrPar->PS_Mode < 0) || (CntrPar->PS_Mode > 3)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PS_Mode must be 0 or 1.");
    }

    // SU_Mode
    if ((CntrPar->SU_Mode < 0) || (CntrPar->SU_Mode > 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "SU_Mode must be 0 or 1.");
    }

    // SD_Mode
    if ((CntrPar->SD_Mode < 0) || (CntrPar->SD_Mode > 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "SD_Mode must be 0 or 1.");
    }

    // Fl_Mode
    if ((CntrPar->Fl_Mode < 0) || (CntrPar->Fl_Mode > 2)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "Fl_Mode must be 0, 1, or 2.");
    }

    // Flp_Mode
    if ((CntrPar->Flp_Mode < 0) || (CntrPar->Flp_Mode > 3)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "Flp_Mode must be 0, 1, 2, or 3.");
    }

    if ((CntrPar->IPC_ControlMode > 0) && (CntrPar->Flp_Mode > 0)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "ROSCO does not currently support IPC_ControlMode and Flp_Mode > 0");
    }
    //------- FILTERS ----------------------------------------------------------

    // F_LPFCornerFreq
    if (CntrPar->F_LPFCornerFreq <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "F_LPFCornerFreq must be greater than zero.");
    }

    // F_LPFDamping
    if (CntrPar->F_LPFType == 2) {
        if (CntrPar->F_LPFDamping <= 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_LPFDamping must be greater than zero.");
        }
    }

    // Notch Filter Params
    if (CntrPar->F_NumNotchFilts > 0) {
        // F_NotchCornerFreq
        if (any_le(CntrPar->F_NotchFreqs, CntrPar->n_F_NotchFreqs, 0.0)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_NotchFreqs must be greater than zero.");
        }

        // F_NotchBetaDen
        if (any_le(CntrPar->F_NotchBetaDen, CntrPar->n_F_NotchBetaDen, 0.0)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_NotchBetaDen must be greater than zero.");
        }
    }

    // F_SSCornerFreq
    if (CntrPar->F_SSCornerFreq <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "F_SSCornerFreq must be greater than zero.");
    }

    // F_WECornerFreq
    if (CntrPar->F_WECornerFreq <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "F_WECornerFreq must be greater than zero.");
    }

    if (CntrPar->Fl_Mode > 0) {
        // F_FlCornerFreq(1)  (frequency)
        if (CntrPar->F_FlCornerFreq[0] <= 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_FlCornerFreq(1) must be greater than zero.");
        }

        // F_FlCornerFreq(2)  (damping)
        if (CntrPar->F_FlCornerFreq[1] <= 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_FlCornerFreq(2) must be greater than zero.");
        }

        // F_FlHighPassFreq
        if (CntrPar->F_FlHighPassFreq <= 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_FlHighPassFreq must be greater than zero.");
        }
    }

    if (CntrPar->Flp_Mode > 0) {
        // F_FlpCornerFreq(1)  (frequency)
        if (CntrPar->F_FlpCornerFreq[0] <= 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_FlpCornerFreq(1) must be greater than zero.");
        }

        // F_FlpCornerFreq(2)  (damping). `<` here, not `<=`: the reference
        // spells this one predicate differently from its neighbour and the
        // message says so.
        if (CntrPar->F_FlpCornerFreq[1] < 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "F_FlpCornerFreq(2) must be greater than or equal to zero.");
        }
    }

    //------- BLADE PITCH CONTROL ----------------------------------------------

    // PC_GS_n -- INTEGER compared against a REAL literal, so the comparison is
    // made in REAL. Written the way the reference wrote it.
    if (static_cast<double>(CntrPar->PC_GS_n) <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PC_GS_n must be greater than 0");
    }

    // PC_GS_angles
    if (CntrPar->PC_ControlMode != 0 &&
        !(nondecreasing_c(CntrPar->PC_GS_angles, CntrPar->n_PC_GS_angles) != 0)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PC_GS_angles must be non-decreasing");
    }

    // PC_GS_KP and PC_GS_KI
    // I'd like to throw warnings if these are positive

    // PC_MinPit and PC_MaxPit
    if (CntrPar->PC_MinPit >= CntrPar->PC_MaxPit) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PC_MinPit must be less than PC_MaxPit.");
    }

    // PC_RefSpd
    if (CntrPar->PC_RefSpd <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PC_RefSpd must be greater than zero.");
    }

    // PC_MaxRat
    if (CntrPar->PC_MaxRat <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PC_MaxRat must be greater than zero.");
    }

    // PC_MinRat
    if (CntrPar->PC_MinRat >= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PC_MinRat must be less than zero.");
    }

    //------- INDIVIDUAL PITCH CONTROL -----------------------------------------

    if (CntrPar->IPC_CornerFreqAct < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      "Corner frequency of IPC actuator model must be positive, or set to 0 to disable.");
    }

    if (CntrPar->IPC_SatMode < 0 || CntrPar->IPC_SatMode > 3) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "IPC_SatMode must be 0, 1, 2, or 3.");
    }

    if (CntrPar->IPC_KI[0] < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "IPC_KI(1) must be zero or greater than zero.");
    }

    if (CntrPar->IPC_KI[1] < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "IPC_KI(2) must be zero or greater than zero.");
    }

    // The next two predicates read IPC_KI, not IPC_KP, while their messages
    // name IPC_KP -- ReadSetParameters.f90:1240-1258. Transcribed as written
    // (P7): the reference's own guard on IPC_KP does not exist, and repairing
    // it here would move the answer of the program the campaign is shipping.
    if (CntrPar->IPC_KI[0] < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "IPC_KP(1) must be zero or greater than zero.");
    }

    if (CntrPar->IPC_KI[1] < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "IPC_KP(2) must be zero or greater than zero.");
    }

    //------- VS TORQUE CONTROL ------------------------------------------------

    if (CntrPar->VS_MaxRat <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_MaxRat must be greater than zero.");
    }

    // VS_Rgn2K
    if (CntrPar->VS_Rgn2K < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_Rgn2K must not be negative.");
    }

    // VS_RtTq
    if (CntrPar->VS_MaxTq < CntrPar->VS_RtTq) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_RtTq must not be greater than VS_MaxTq.");
    }

    // VS_RtPwr
    if (CntrPar->VS_RtPwr < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_RtPwr must not be negative.");
    }

    // VS_RtTq
    if (CntrPar->VS_RtTq < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_RtTq must not be negative.");
    }

    // VS_KP
    if (CntrPar->VS_KP[0] > 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_KP must be less than zero.");
    }

    // VS_KI
    if (CntrPar->VS_KI[0] > 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_KI must be less than zero.");
    }

    // VS_TSRopt
    if (CntrPar->VS_TSRopt < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "VS_TSRopt must be greater than zero.");
    }

    //------- SETPOINT SMOOTHER ---------------------------------------------

    // SS_VSGain
    if (CntrPar->SS_VSGain < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "SS_VSGain must be greater than zero.");
    }

    // SS_PCGain
    if (CntrPar->SS_PCGain < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "SS_PCGain must be greater than zero.");
    }

    if ((CntrPar->PRC_Mode < 0) || (CntrPar->PRC_Mode > 2)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PRC_Mode must be 0, 1, or 2.");
    }

    if (CntrPar->PRC_Mode == 2) {
        // PRINT *, "Note: PRC Mode = ", CntrPar%PRC_Mode, ", which will affect ..."
        //
        // gfortran list-directed output: the record opens with one blank, a
        // CHARACTER item is written verbatim with no separator, a default
        // INTEGER occupies a 12-column right-justified field, and the item
        // after it is preceded by one blank. Measured, not assumed --
        // evidence/CheckInputs/print_format.f90.
        std::printf(" Note: PRC Mode = %12d , which will affect VS_RefSpeed, VS_TSRopt, and PC_RefSpeed\n",
                    CntrPar->PRC_Mode);

        if (CntrPar->PRC_Comm == 0) {
            if (CntrPar->PRC_R_Pitch < 0) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "PRC_R_Pitch must be greater than or equal to zero.");
            }

            if (CntrPar->PRC_R_Speed < 0) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "PRC_R_Speed must be greater than or equal to zero.");
            }

            if (CntrPar->PRC_R_Torque < 0) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "PRC_R_Torque must be greater than or equal to zero.");
            }
        }

        if ((CntrPar->PRC_Comm == 1) && (CntrPar->OL_Mode != 1)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          "OL_Mode must be 1 to use open loop inputs for power control (PRC_Comm = 1).");
        }

        if ((CntrPar->PRC_Comm == 2) && (CntrPar->ZMQ_Mode != 1)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          "ZMQ_Mode must be 1 to use ZeroMQ inputs for power control (PRC_Comm = 2).");
        }
    }

    //------- WIND SPEED ESTIMATOR ---------------------------------------------

    // WE_BladeRadius
    if (CntrPar->WE_BladeRadius < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "WE_BladeRadius must be greater than zero.");
    }

    // WE_GearboxRatio
    if (CntrPar->WE_GearboxRatio < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "WE_GearboxRatio must be greater than zero.");
    }

    // WE_Jtot
    if (CntrPar->WE_Jtot < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "WE_Jtot must be greater than zero.");
    }

    // WE_RhoAir
    if (CntrPar->WE_RhoAir < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "WE_RhoAir must be greater than zero.");
    }

    // PerfTableSize(1) -- INTEGER against a REAL literal, as the reference wrote it.
    if (static_cast<double>(CntrPar->PerfTableSize[0]) < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PerfTableSize(1) must be greater than zero.");
    }

    // PerfTableSize(2)
    if (static_cast<double>(CntrPar->PerfTableSize[1]) < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "PerfTableSize(2) must be greater than zero.");
    }

    // WE_FOPoles_N
    if (static_cast<double>(CntrPar->WE_FOPoles_N) < 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "WE_FOPoles_N must be greater than zero.");
    }

    // WE_FOPoles_v
    if (CntrPar->WE_Mode == 2 &&
        !(nondecreasing_c(CntrPar->WE_FOPoles_v, CntrPar->n_WE_FOPoles_v) != 0)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "WE_FOPoles_v must be non-decreasing.");
    }

    // ---- Yaw Control ----
    if (CntrPar->Y_ControlMode > 0) {
        if (CntrPar->Y_ControlMode == 1) {
            if (CntrPar->Y_ErrThresh[0] <= 0.0) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "Y_ErrThresh must be greater than zero.");
            }

            if (CntrPar->Y_Rate <= 0.0) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "CntrPar%Y_Rate must be greater than zero.");
            }
        }
    }

    // ---- Tower Control ----
    if (CntrPar->TD_Mode < 0 || CntrPar->TD_Mode > 1) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "TD_Mode must be 0 or 1.");
    }

    if (CntrPar->TRA_Mode < 0 || CntrPar->TRA_Mode > 1) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "TRA_Mode must be 0 or 1.");
    }

    if (CntrPar->TRA_Mode > 1) {  // Frequency avoidance is active
        if (CntrPar->TRA_ExclSpeed < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "TRA_ExclSpeed must be greater than 0.");
        }

        if (CntrPar->TRA_ExclBand < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "TRA_ExclBand must be greater than 0.");
        }

        if (CntrPar->TRA_RateLimit < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "TRA_RateLimit must be greater than 0.");
        }

        if (!((CntrPar->VS_ControlMode == VS_Mode_WSE_TSR) ||
              (CntrPar->VS_ControlMode == VS_Mode_Power_TSR))) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "VS_ControlMode must be 2 or 3 to use frequency avoidance control.");
        }

        if (CntrPar->PRC_Mode == 1) {
            std::printf(" ROSCO Warning: Note that frequency avoidance control (TRA_Mode > 1) will affect PRC set points\n");
        }
    }

    //------- MINIMUM PITCH SATURATION -------------------------------------------
    if (CntrPar->PS_Mode > 0) {
        // PS_BldPitchMin_N
        if (static_cast<double>(CntrPar->PS_BldPitchMin_N) < 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "PS_BldPitchMin_N must be greater than zero.");
        }

        // PS_WindSpeeds
        if (!(nondecreasing_c(CntrPar->PS_WindSpeeds, CntrPar->n_PS_WindSpeeds) != 0)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "PS_WindSpeeds must be non-decreasing.");
        }
    }

    // --- Startup ---
    if (CntrPar->SU_Mode > 0) {
        // SU_FW_MinDuration
        if (CntrPar->SU_FW_MinDuration < 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SU_FW_MinDuration must be greater than zero.");
        }

        // SU_RotorSpeedThresh
        if (CntrPar->SU_RotorSpeedThresh < 0.0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SU_RotorSpeedThresh must be greater than zero.");
        }

        // SU_RotorSpeedCornerFreq
        if (CntrPar->SU_RotorSpeedCornerFreq < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SU_RotorSpeedCornerFreq must be greater than or equal to 0.");
        }

        // SU_LoadStages_N
        if (CntrPar->SU_LoadStages_N < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SU_LoadStages_N must be greater than or equal to 0.");
        }

        // SU_LoadStages
        if (any_lt(CntrPar->SU_LoadStages, CntrPar->n_SU_LoadStages, 0.0)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SU_LoadStages must be positive.");
        }

        // SU_LoadRampDuration
        if (any_lt(CntrPar->SU_LoadRampDuration, CntrPar->n_SU_LoadRampDuration, 0.0)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SU_LoadRampDuration must be positive.");
        }

        // SU_LoadHoldDuration
        if (any_lt(CntrPar->SU_LoadHoldDuration, CntrPar->n_SU_LoadHoldDuration, 0.0)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SU_LoadHoldDuration must be positive.");
        }
    }

    // --- Shutdown ---
    if (CntrPar->SD_Mode > 0) {
        // SD_Method
        if (CntrPar->SD_Method < 1 || CntrPar->SD_Method > 2) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SD_Method must be 1 or 2.");
        }

        // SD_MaxPitchRate
        if (maxval(CntrPar->SD_MaxPitchRate, CntrPar->n_SD_MaxPitchRate) > CntrPar->PC_MaxRat) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SD_MaxPitchRate(s) should be less than or equal to PC_MaxRat.");
        }

        // SD_MaxTorqueRate
        if (maxval(CntrPar->SD_MaxTorqueRate, CntrPar->n_SD_MaxTorqueRate) > CntrPar->VS_MaxRat) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SD_MaxTorqueRate(s) should be less than or equal to VS_MaxRat.");
        }

        if (CntrPar->SD_Stage_N < 1) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "SD_Stage_N must be greater than or equal to 1.");
        }

        if (CntrPar->SD_Method == 1) {
            // SD_StageTime must be greater than zero
            if (minval(CntrPar->SD_StageTime, CntrPar->n_SD_StageTime) < 0.0) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "SD_StageTime(s) must be greater than or equal to zero.");
            }
        } else if (CntrPar->SD_Method == 2) {
            // SD_StagePitch must be increasing
            if (!(nondecreasing_c(CntrPar->SD_StagePitch, CntrPar->n_SD_StagePitch) != 0)) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "SD_StagePitch must be non-decreasing.");
            }
        }
    }

    // --- Open loop control ---
    if (CntrPar->OL_Mode > 0) {
        // Get all open loop indices.
        //
        // All_OL_Indices is INTEGER(IntKi), ALLOCATABLE :: (:) and is grown by
        // AddToList, which takes an ALLOCATABLE INTENT(INOUT) dummy -- so what
        // it needs is a real Fortran descriptor, not a C array. CFI_establish
        // makes one; ALLOCATE/DEALLOCATE become CFI_allocate/CFI_deallocate on
        // it. Calling addtolist_c through the descriptor is the point: the
        // callee is translated, and re-implementing its growth here would be
        // the inlining the campaign forbids.
        CFI_CDESC_T(1) All_OL_Indices_storage;
        CFI_cdesc_t* All_OL_Indices = reinterpret_cast<CFI_cdesc_t*>(&All_OL_Indices_storage);
        CFI_establish(All_OL_Indices, nullptr, CFI_attribute_allocatable, CFI_type_int,
                      sizeof(int), 1, nullptr);

        // ALLOCATE(All_OL_Indices(5))   ! Will need to increase to 5 when IPC
        {
            CFI_index_t lower = 1, upper = 5;
            CFI_allocate(All_OL_Indices, &lower, &upper, 0);
        }

        // All_OL_Indices = (/CntrPar%Ind_BldPitch, CntrPar%Ind_GenTq, ... /)
        //
        // The constructor's FIRST item is the whole ALLOCATABLE array
        // Ind_BldPitch, so the right-hand side has SIZE(Ind_BldPitch) + 5
        // elements -- eight for the three-blade shape this controller builds,
        // against the five just allocated. gfortran is at F2003 defaults here
        // (no -fno-realloc-lhs and no -std=f95 in rosco/controller/CMakeLists.txt),
        // so the assignment REALLOCATES the left-hand side to the shape of the
        // right and the ALLOCATE(5) above decides nothing. Transcribed with the
        // reallocation, not with the 5.
        std::vector<int> rhs;
        rhs.reserve(static_cast<size_t>(CntrPar->n_Ind_BldPitch) + 5);
        for (int32_t i = 0; i < CntrPar->n_Ind_BldPitch; ++i)
            rhs.push_back(CntrPar->Ind_BldPitch[i]);
        rhs.push_back(CntrPar->Ind_GenTq);
        rhs.push_back(CntrPar->Ind_YawRate);
        rhs.push_back(CntrPar->Ind_R_Speed);
        rhs.push_back(CntrPar->Ind_R_Torque);
        rhs.push_back(CntrPar->Ind_R_Pitch);

        if (static_cast<CFI_index_t>(rhs.size()) != All_OL_Indices->dim[0].extent) {
            CFI_deallocate(All_OL_Indices);
            CFI_index_t lower = 1, upper = static_cast<CFI_index_t>(rhs.size());
            CFI_allocate(All_OL_Indices, &lower, &upper, 0);
        }
        std::memcpy(All_OL_Indices->base_addr, rhs.data(), sizeof(int) * rhs.size());

        // DO I = 1,SIZE(CntrPar%Ind_CableControl) ; Call AddToList(...)
        for (int32_t I = 1; I <= CntrPar->n_Ind_CableControl; ++I) {
            addtolist_c(All_OL_Indices, CntrPar->Ind_CableControl[I - 1]);
        }

        // DO I = 1,SIZE(CntrPar%Ind_StructControl) ; Call AddToList(...)
        for (int32_t I = 1; I <= CntrPar->n_Ind_StructControl; ++I) {
            addtolist_c(All_OL_Indices, CntrPar->Ind_StructControl[I - 1]);
        }

        // Read the extent back out of the descriptor AFTER the two loops: each
        // AddToList call replaces the data pointer and the extent.
        const int* ol_idx = static_cast<const int*>(All_OL_Indices->base_addr);
        const int32_t n_ol_idx = static_cast<int32_t>(All_OL_Indices->dim[0].extent);

        if (any_lt(ol_idx, n_ol_idx, 0)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "All open loop control indices must be greater than zero");
        }

        if (CntrPar->Ind_Breakpoint < 1) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "Ind_Breakpoint must be non-zero if OL_Mode is non-zero");
        }

        if (all_lt(ol_idx, n_ol_idx, 1)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "At least one open loop input channel must be non-zero");
        }

        if (CntrPar->OL_Mode == 2) {
            if ((CntrPar->Ind_BldPitch[0] == 0) || (CntrPar->Ind_BldPitch[1] == 0) ||
                (CntrPar->Ind_BldPitch[2] == 0) || (CntrPar->Ind_GenTq == 0) ||
                (CntrPar->Ind_Azimuth == 0)) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar,
                              "If OL_Mode = 2, Ind_BldPitch, Ind_GenTq, and Ind_Azimuth must be greater than zero");
            }
        }

        if (any_gt(CntrPar->Ind_CableControl, CntrPar->n_Ind_CableControl, 0) &&
            CntrPar->CC_Mode != 2) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          "CC_Mode must be 2 if using open loop cable control via Ind_CableControl");
        }

        if (any_gt(CntrPar->Ind_StructControl, CntrPar->n_Ind_StructControl, 0) &&
            CntrPar->StC_Mode != 2) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          "CC_Mode must be 2 if using open loop struct control via Ind_StructControl");
        }

        if ((CntrPar->OL_BP_Mode < 0) || (CntrPar->OL_BP_Mode > 1)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "OL_BP_Mode must be 0 or 1.");
        }

        if (CntrPar->OL_BP_FiltFreq < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "OL_BP_FiltFreq must be greater than or equal to 0.");
        }

        if ((CntrPar->OL_BP_Mode == 1) && (CntrPar->OL_Mode == 2)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          "Rotor position control (OL_Mode = 2) is not compatible with wind speed breakpoints (OL_BP_Mode = 1)");
        }

        // The Fortran local is deallocated on return from the subroutine.
        CFI_deallocate(All_OL_Indices);
    }

    // ---- AWC vs. IPC
    if (CntrPar->AWC_Mode > 0 && CntrPar->IPC_ControlMode > 0) {
        std::printf(" ROSCO WARNING: Individual pitch control and active wake control are both enabled. Performance may be compromised.\n");
    }

    // --- Pitch Actuator ---
    if (CntrPar->PA_Mode > 0) {
        if ((CntrPar->PA_Mode < 0) || (CntrPar->PA_Mode > 2)) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "PA_Mode must be 0, 1, or 2");
        }
        if (CntrPar->PA_CornerFreq < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "PA_CornerFreq must be greater than 0");
        }
        if (CntrPar->PA_Damping < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "PA_Damping must be greater than 0");
        }
    }

    // --- Active Wake Control ---
    if (CntrPar->AWC_Mode > 0) {
        if (CntrPar->AWC_NumModes < 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "AWC_NumModes must be a positive integer if AWC_Mode = 1");
        }
        for (int Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode) {
            if (CntrPar->AWC_freq[Imode - 1] < 0.0) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "AWC_freq cannot be less than 0");
            }
        }
        if (CntrPar->AWC_Mode == 1) {
            for (int Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode) {
                if ((CntrPar->AWC_clockangle[Imode - 1] > 360.0) ||
                    (CntrPar->AWC_clockangle[Imode - 1] < 0.0)) {
                    ErrVar->aviFAIL = -1;
                    assign_errmsg(ErrVar, "AWC_clockangle must be between 0 and 360 in AWC_Mode = 1");
                }
            }
        }

        if (CntrPar->AWC_Mode == 2) {
            if ((CntrPar->AWC_NumModes > 2) || (CntrPar->AWC_NumModes < 1)) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "AWC_NumModes must be either 1 or 2 if AWC_Mode = 2");
            }
            for (int Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode) {
                if ((CntrPar->AWC_clockangle[Imode - 1] > 360.0) ||
                    (CntrPar->AWC_clockangle[Imode - 1] < -360.0)) {
                    ErrVar->aviFAIL = -1;
                    assign_errmsg(ErrVar,
                                  "AWC_clockangle must be between -360 and 360 in AWC_Mode = 2");
                }
                if (CntrPar->AWC_harmonic[Imode - 1] < 0) {
                    ErrVar->aviFAIL = -1;
                    assign_errmsg(ErrVar, "AWC_harmonic must be a positive integer");
                }
            }
        }
    }

    if ((CntrPar->CC_Mode < 0) || (CntrPar->CC_Mode > 2)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "CC_Mode must be 0 or 1");
    }

    if (CntrPar->CC_Mode > 0) {
        // Extended avrSWAP must be used
        if (CntrPar->Ext_Interface == 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          "The OpenFAST extended bladed interface must be used with Ext_Interface > 0 in the DISCON");
        }

        if (CntrPar->CC_ActTau <= 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar, "CC_ActTau must be greater than 0.");
        }

        for (int I = 1; I <= CntrPar->CC_Group_N; ++I) {
            if (CntrPar->CC_GroupIndex[I - 1] < 2601) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "CC_GroupIndices must be greater than 2601.");
            }
        }
    }

    if (CntrPar->StC_Mode > 0) {
        // Extended avrSWAP must be used
        if (CntrPar->Ext_Interface == 0) {
            ErrVar->aviFAIL = -1;
            assign_errmsg(ErrVar,
                          "The OpenFAST extended bladed interface must be used with Ext_Interface > 0 in the DISCON");
        }

        // Check indices
        for (int I = 1; I <= CntrPar->StC_Group_N; ++I) {
            if (CntrPar->StC_GroupIndex[I - 1] < 2801) {
                ErrVar->aviFAIL = -1;
                assign_errmsg(ErrVar, "StC_GroupIndices must be greater than 2801.");
            }
        }
    }

    // Check that open loop control active if using open loop cable/struct control
    if (CntrPar->CC_Mode == 2 && CntrPar->OL_Mode != 1) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "OL_Mode must be 1 if using CC_Mode = 2 (open loop)");
    }

    if (CntrPar->StC_Mode == 2 && CntrPar->OL_Mode != 1) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "OL_Mode must be 1 if using StC_Mode = 2 (open loop)");
    }

    // Abort if the user has not requested a pitch angle actuator (See Appendix A
    // of Bladed User's Guide):
    //
    // avrSWAP is REAL(ReKi) = REAL(4) (Constants.f90), so each element is a
    // float widened to double. Fortran NINT is round-half-away-from-zero:
    // std::lround, not a cast and not rint.
    if (std::lround(static_cast<double>(avrSWAP[9])) != 0) {  // .TRUE. if a pitch angle actuator hasn't been requested
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "Pitch angle actuator not requested.");
    }

    if ((std::lround(static_cast<double>(avrSWAP[27])) == 0) &&
        ((CntrPar->IPC_ControlMode > 0) || (CntrPar->Y_ControlMode > 1) ||
         (CntrPar->Ind_BldPitch[1] > 0) || (CntrPar->Ind_BldPitch[2] > 0))) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      "IPC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.");
    }

    // PF_Mode = 1
    if (std::lround(static_cast<double>(avrSWAP[27])) == 0 && (CntrPar->PF_Mode == 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      "Pitch offset fault enabled (PF_Mode = 1), but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.");
    }

    if (std::lround(static_cast<double>(avrSWAP[27])) == 0 && (CntrPar->AWC_Mode > 1)) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar,
                      "AWC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1 for individual pitch control.");
    }

    // DT
    if (LocalVar->DT <= 0.0) {
        ErrVar->aviFAIL = -1;
        assign_errmsg(ErrVar, "DT must be greater than zero.");
    }

    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }
}
