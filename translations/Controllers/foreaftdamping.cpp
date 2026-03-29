// VIT Translation
// Function: ForeAftDamping
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE ForeAftDamping(CntrPar, LocalVar, objInst)
// VIT: 0.1.0
// Status: unverified

#include <algorithm>

// PIController entry point — provided by vit_translated.h (integration)
// or vit_kernel_callees.h (kernel verification).
extern "C" {
    double picontroller_c(double error, double kp, double ki,
                          double minValue, double maxValue, double DT,
                          double I0, piparams_t* piP, int reset, int* inst);
}

void ForeAftDamping(controlparameters_view_t* CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst) {
    // Fore-aft damping controller: reduces tower vibrations using pitch

    // PI controller on fore-aft acceleration (high-pass filtered)
    LocalVar->FA_AccHPFI = picontroller_c(
        LocalVar->FA_AccHPF,        // error: filtered fore-aft acceleration
        0.0,                         // kp: no proportional gain
        CntrPar->FA_KI,             // ki: integral gain (from view struct)
        -CntrPar->FA_IntSat,        // minValue: negative saturation
        CntrPar->FA_IntSat,         // maxValue: positive saturation
        LocalVar->DT,               // time step
        0.0,                         // I0: initial integrator value
        &LocalVar->piP,             // PI parameters struct
        (LocalVar->restart != 0),    // reset flag
        &objInst->instPI             // instance counter
    );

    // Store the fore-aft pitch contribution for all blades
    for (int K = 0; K < LocalVar->NumBl; K++) {
        LocalVar->FA_PitCom[K] = LocalVar->FA_AccHPFI;
    }
}
