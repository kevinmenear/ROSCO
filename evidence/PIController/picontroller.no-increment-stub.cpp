// STUB -- NOT the translation. `inst = inst + 1` deleted. Asks whether the
// kernel's compared state includes the instance counter this unit advances.
#include "vit_types.h"
double PIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, int* inst) {
    const int i = *inst - 1;
    double PIController_result;
    if (reset != 0) {
        piP->ITerm[i] = I0;
        piP->ITermLast[i] = I0;
        PIController_result = I0;
    } else {
        const double PTerm = kp * error;
        piP->ITerm[i] = piP->ITerm[i] + DT * ki * error;
        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);
        PIController_result = saturate_c(PTerm + piP->ITerm[i], minValue, maxValue);
        piP->ITermLast[i] = piP->ITerm[i];
    }
    /* *inst = *inst + 1; deleted */
    return PIController_result;
}
