// STUB -- NOT the translation. The unit as a complete no-op: reads no argument, writes no output,
// returns 0.0. P3's red test for the differential harness.
#include "vit_types.h"
double PIIController(double error, double error2, double kp, double ki, double ki2, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, int* inst) {
    (void)error; (void)error2; (void)kp; (void)ki; (void)ki2;
    (void)minValue; (void)maxValue; (void)DT; (void)I0;
    (void)piP; (void)reset; (void)inst;
    if (false) { (void)saturate_c(0.0, 0.0, 0.0); }  // link parity, unit #34
    return 0.0;
}
