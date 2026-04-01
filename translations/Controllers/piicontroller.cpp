// VIT Translation
// Function: PIIController
// Source: Controllers.f90
// Module: Controllers
// Fortran: FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst)
// Status: unverified

#include "vit_types.h"
#include <algorithm>

double PIIController(double error, double error2, double kp, double ki, double ki2, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst) {
    int idx = *inst - 1;  // Fortran 1-based -> C 0-based
    double result;

    if (reset) {
        piP->ITerm[idx] = I0;
        piP->ITermLast[idx] = I0;
        piP->ITerm2[idx] = I0;
        piP->ITermLast2[idx] = I0;

        result = I0;
    } else {
        double PTerm = kp * error;
        piP->ITerm[idx] = piP->ITerm[idx] + DT * ki * error;
        piP->ITerm2[idx] = piP->ITerm2[idx] + DT * ki2 * error2;
        piP->ITerm[idx] = std::min(std::max(piP->ITerm[idx], minValue), maxValue);
        piP->ITerm2[idx] = std::min(std::max(piP->ITerm2[idx], minValue), maxValue);
        result = PTerm + piP->ITerm[idx] + piP->ITerm2[idx];
        result = std::min(std::max(result, minValue), maxValue);

        piP->ITermLast[idx] = piP->ITerm[idx];
    }
    *inst = *inst + 1;

    return result;
}
