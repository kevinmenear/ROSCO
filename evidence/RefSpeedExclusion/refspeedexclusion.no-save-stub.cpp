// STUB -- NOT THE SHIPPED TRANSLATION. Unit #59 harness red test.
// THE `! Save last reference speed` STATEMENT DELETED. TRA_LastRefSpd keeps
// whatever the restart block left, or whatever the case carried.
//
// Built by hand from translations/ControllerBlocks/refspeedexclusion.cpp at
// commit 40d869e3; every comment of the original is dropped so that what is
// CUT is visible at a glance.

#include "vit_types.h"

void RefSpeedExclusion(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar) {
    double VS_RefSpeed_LSS = LocalVar->VS_RefSpd / CntrPar->WE_GearboxRatio;

    if ((VS_RefSpeed_LSS > CntrPar->TRA_ExclSpeed - CntrPar->TRA_ExclBand / 2) &&
        (VS_RefSpeed_LSS < CntrPar->TRA_ExclSpeed + CntrPar->TRA_ExclBand / 2)) {
        LocalVar->FA_Hist = 1;
    } else {
        LocalVar->FA_Hist = 0;
    }

    if (LocalVar->restart != 0) {
        if (LocalVar->FA_Hist > 0) {
            if (VS_RefSpeed_LSS > CntrPar->TRA_ExclSpeed) {
                LocalVar->TRA_LastRefSpd = CntrPar->TRA_ExclSpeed + CntrPar->TRA_ExclBand / 2;
            } else {
                LocalVar->TRA_LastRefSpd = CntrPar->TRA_ExclSpeed - CntrPar->TRA_ExclBand / 2;
            }
        } else {
            LocalVar->TRA_LastRefSpd = VS_RefSpeed_LSS;
        }
    }

    if (LocalVar->FA_Hist > 0) {
        LocalVar->VS_RefSpd_TRA = LocalVar->TRA_LastRefSpd;
    } else {
        LocalVar->VS_RefSpd_TRA = VS_RefSpeed_LSS;
    }


    LocalVar->VS_RefSpd_RL =
        ratelimit_c(LocalVar->VS_RefSpd_TRA, -CntrPar->TRA_RateLimit, CntrPar->TRA_RateLimit,
                    LocalVar->DT, LocalVar->restart ? 1 : 0, &LocalVar->rlP, &objInst->instRL,
                    0, 0.0);

    LocalVar->VS_RefSpd = LocalVar->VS_RefSpd_RL * CntrPar->WE_GearboxRatio;
}
