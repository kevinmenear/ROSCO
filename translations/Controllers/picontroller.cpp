// VIT Translation
// Function: PIController
// Source: Controllers.f90
// Module: Controllers
// Fortran: FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst)
// Status: unverified

#include "vit_types.h"
#include <algorithm>

double PIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int reset, int* inst) {
    int idx = *inst - 1;  // Fortran 1-based -> C 0-based
    double result;

    if (reset) {
        piP->ITerm[idx] = I0;
        piP->ITermLast[idx] = I0;

        result = I0;
    } else {
        double PTerm = kp * error;
        piP->ITerm[idx] = piP->ITerm[idx] + DT * ki * error;
        piP->ITerm[idx] = std::min(std::max(piP->ITerm[idx], minValue), maxValue);
        result = std::min(std::max(PTerm + piP->ITerm[idx], minValue), maxValue);

        piP->ITermLast[idx] = piP->ITerm[idx];
    }
    *inst = *inst + 1;

    return result;
}
