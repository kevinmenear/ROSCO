#include "../include/vit_types.h"
#include "../include/vit_translated.h"

double PIDController(double error, double kp, double ki, double kd, double tf, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, objectinstances_t* objInst, localvariables_t* LocalVar) {
    int piIdx = objInst->instPI - 1;  // Fortran 1-based -> C 0-based
    double result;

    // Always filter error for derivative calculation
    double EFilt = LPFilter(error, DT, tf, &LocalVar->FP, LocalVar->iStatus, reset, &objInst->instLPF, 0, 0.0);

    if (reset) {
        piP->ITerm[piIdx] = I0;
        piP->ITermLast[piIdx] = I0;
        piP->ELast[piIdx] = 0.0;
        result = I0;
    } else {
        // Proportional
        double PTerm = kp * error;

        // Integrate and saturate
        piP->ITerm[piIdx] = piP->ITerm[piIdx] + DT * ki * error;
        piP->ITerm[piIdx] = saturate(piP->ITerm[piIdx], minValue, maxValue);

        // Derivative (filtered)
        double DTerm = kd * (EFilt - piP->ELast[piIdx]) / DT;

        // Saturate all
        result = saturate(PTerm + piP->ITerm[piIdx] + DTerm, minValue, maxValue);

        // Save lasts
        piP->ITermLast[piIdx] = piP->ITerm[piIdx];
        piP->ELast[piIdx] = EFilt;
    }

    objInst->instPI = objInst->instPI + 1;

    return result;
}
