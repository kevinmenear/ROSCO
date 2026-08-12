// STUB — NOT THE TRANSLATION. The whole unit as a no-op: it reads no argument,
// writes nothing to ErrVar and returns a determinate, finite, wrong constant.
// This is the differential harness's liveness test (P3 / X4).
#include "vit_types.h"

double sigma(double x, double x0, double x1, double y0, double y1,
             errorvariables_view_t* ErrVar) {
    (void)x; (void)x0; (void)x1; (void)y0; (void)y1; (void)ErrVar;
    return -7.25;
}
