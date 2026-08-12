// RED TEST STUB -- the 23 outputs fed by all-zero avrSWAP entries DELETED.
// Not a translation.
// VIT Translation Scaffold
// Function: ReadAvrSWAP
// Source: ReadSetParameters.f90
// Module: ReadSetParameters
// Fortran: SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar, CntrPar, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 5b2dcf10ae7b
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-12T02:12:58Z

#include "vit_types.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace {

// Constants.f90:22-23. Both are declared REAL(DbKi), and the literal that
// initialises each carries NO kind suffix -- so under this campaign's
// -fdefault-real-8 the literal is itself parsed at kind 8 and no narrowing
// happens anywhere. Written here as the same decimal text.
constexpr double R2D = 57.2957795130;
constexpr double D2R = 0.01745329251;

// The message the already-loaded branch assigns. ErrVar%ErrMsg is
// CHARACTER(:), ALLOCATABLE, so the Fortran assignment REALLOCATES it to
// exactly this length -- it does not blank-fill a fixed width.
const char ALREADY_LOADED_MSG[] =
    "ERROR: This ROSCO dynamic library has already been loaded.";
constexpr int ALREADY_LOADED_LEN = (int)(sizeof(ALREADY_LOADED_MSG) - 1);

}  // namespace

void ReadAvrSWAP(float* avrSWAP, localvariables_view_t* LocalVar,
                 controlparameters_view_t* CntrPar, errorvariables_view_t* ErrVar) {
    // Index used for looping through blades. INTEGER(IntKi) :: K
    int K;

    // Load variables from calling program (See Appendix A of Bladed User's Guide):
    //
    // avrSWAP is REAL(ReKi) and ReKi is C_FLOAT (Constants.f90:18), so every
    // read below is a float widened to double -- exact, and NOT the same thing
    // as reading a double. NINT is round-half-away-from-zero, which is
    // std::lround and not a cast and not rint.
    LocalVar->iStatus = (int)std::lround((double)avrSWAP[0]);
    LocalVar->Time = avrSWAP[1];
    LocalVar->DT = avrSWAP[2];
    LocalVar->VS_GenPwr = avrSWAP[14];
    LocalVar->GenSpeed = avrSWAP[19];
    LocalVar->RotSpeed = avrSWAP[20];
    LocalVar->GenTqMeas = avrSWAP[22];
    LocalVar->NacVane = (double)avrSWAP[23] * R2D;
    LocalVar->HorWindV = avrSWAP[26];
    LocalVar->rootMOOP[0] = avrSWAP[29];
    LocalVar->rootMOOP[1] = avrSWAP[30];
    LocalVar->rootMOOP[2] = avrSWAP[31];
    LocalVar->NacHeading = (double)avrSWAP[36] * R2D;
    // This is the translational acceleration of the tower top in the non-rotating frame
    // This is the translational acceleration of the tower top in the non-rotating frame
    // This is the rotational aceleration of the nacelle in the shaft frame
    LocalVar->Azimuth = avrSWAP[59];
    LocalVar->NumBl = (int)std::lround((double)avrSWAP[60]);

    // RED TEST STUB: extended-interface block (18 Ptfm* assignments) DELETED

    // Check that we haven't already loaded this dynamic library
    if (LocalVar->iStatus == 0) {
        if (LocalVar->AlreadyInitialized == 0) {
            LocalVar->AlreadyInitialized = 1;
        } else {
            ErrVar->aviFAIL = -1;
            // ErrVar%ErrMsg = '...' on a CHARACTER(:), ALLOCATABLE field is a
            // REALLOCATING assignment: the new length is the literal's, not the
            // old one's. The staging buffer's capacity is finite, so an
            // assignment that does not fit is REFUSED and reported rather than
            // truncated -- a shortened message is the one wrong answer a byte
            // comparison cannot tell from a right one.
            if (ALREADY_LOADED_LEN <= (int)ErrVar->n_ErrMsg_cap) {
                std::memcpy(ErrVar->ErrMsg, ALREADY_LOADED_MSG,
                            (size_t)ALREADY_LOADED_LEN);
                ErrVar->n_ErrMsg = (int32_t)ALREADY_LOADED_LEN;
            } else {
                std::fprintf(stderr,
                             "VIT: ReadAvrSWAP: ErrVar%%ErrMsg needs %d bytes, "
                             "staging buffer holds %d; field left unchanged\n",
                             ALREADY_LOADED_LEN, (int)ErrVar->n_ErrMsg_cap);
            }
            return;
        }
    }

    // --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does not...
    if (LocalVar->iStatus == 0) {
        LocalVar->BlPitch[0] = avrSWAP[3];
        LocalVar->BlPitch[1] = avrSWAP[32];
        LocalVar->BlPitch[2] = avrSWAP[33];
    } else {

        // Subtract pitch actuator fault for blade K - This in a sense would make the controller blind to the pitch fault
        if (CntrPar->PF_Mode == 1) {
            for (K = 1; K <= LocalVar->NumBl; K++) {
                // This assumes that the pitch actuator fault is hardware fault
                LocalVar->BlPitch[K - 1] =
                    LocalVar->PitComAct[K - 1] - CntrPar->PF_Offsets[K - 1];  // why is PitCom used and not PitComAct??
            }
        } else {
            LocalVar->BlPitch[0] = LocalVar->PitComAct[0];
            LocalVar->BlPitch[1] = LocalVar->PitComAct[1];
            LocalVar->BlPitch[2] = LocalVar->PitComAct[2];
        }

    }

    // `REAL(LocalVar%NumBl)` names no kind, so under -fdefault-real-8 it is
    // kind 8 and the reciprocal is formed in double. The grouping is the
    // reference's: a reciprocal FIRST, then one multiply, and a left-to-right
    // sum inside the parentheses.
    LocalVar->BlPitchCMeas =
        (1.0 / (double)LocalVar->NumBl) *
        ((LocalVar->BlPitch[0] + LocalVar->BlPitch[1]) + LocalVar->BlPitch[2]);

    // TODO: Technically, LocalVar%Time > 0, too, but this restart is in many places as a reset
    if (LocalVar->iStatus == 0) {
        LocalVar->restart = 1;
    } else {
        LocalVar->restart = 0;
    }

    // FA_Acc_TT is in the non-rotating tower-top frame, so we need to convert it to the rotating (nacelle) frame of reference

    // Increment timestep counter
    if (LocalVar->iStatus == 0 && LocalVar->Time == 0) {
        LocalVar->n_DT = 0;
    } else {
        LocalVar->n_DT = LocalVar->n_DT + 1;
    }
}
