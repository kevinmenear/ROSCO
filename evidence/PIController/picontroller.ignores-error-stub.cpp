// STUB -- NOT the translation. `error` forced to 0.0 on entry, so both the
// proportional term and the integrator increment vanish. VIT's own red test
// reported that scaling and offsetting `error` were ABSORBED; this asks the
// stronger question -- can the kernel see the argument at all?
#include "vit_types.h"
double PIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, int* inst) {
    (void)error;
    const double error_ = 0.0;                                // <-- argument ignored
    const int i = *inst - 1;
    double PIController_result;
    if (reset != 0) {
        piP->ITerm[i] = I0;
        piP->ITermLast[i] = I0;
        PIController_result = I0;
    } else {
        const double PTerm = kp * error_;
        piP->ITerm[i] = piP->ITerm[i] + DT * ki * error_;
        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);
        PIController_result = saturate_c(PTerm + piP->ITerm[i], minValue, maxValue);
        piP->ITermLast[i] = piP->ITerm[i];
    }
    *inst = *inst + 1;
    return PIController_result;
}
