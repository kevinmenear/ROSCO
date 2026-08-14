// STUB -- NOT the translation. Reads no argument, writes no state, returns the determinate
// constant -7.25, which no argument at either call site can produce.
#include "vit_types.h"
double PIIController(double error, double error2, double kp, double ki, double ki2, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, int* inst) {
    (void)error; (void)error2; (void)kp; (void)ki; (void)ki2;
    (void)minValue; (void)maxValue; (void)DT; (void)I0;
    (void)piP; (void)reset; (void)inst;
    if (false) { (void)saturate_c(0.0, 0.0, 0.0); }  // link parity, unit #34
    return -7.25;
}
