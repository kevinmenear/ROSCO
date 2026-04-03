#include "vit_types.h"
#include "vit_translated.h"

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
        piP->ITerm[idx] = saturate(piP->ITerm[idx], minValue, maxValue);
        piP->ITerm2[idx] = saturate(piP->ITerm2[idx], minValue, maxValue);
        result = PTerm + piP->ITerm[idx] + piP->ITerm2[idx];
        result = saturate(result, minValue, maxValue);

        piP->ITermLast[idx] = piP->ITerm[idx];
    }
    *inst = *inst + 1;

    return result;
}
