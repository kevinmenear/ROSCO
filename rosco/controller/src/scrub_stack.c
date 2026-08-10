/*
 * Stack scrubber for scipy FITPACK bispev non-determinism workaround.
 *
 * FITPACK's fpbisp (bivariate spline evaluator) reads an uninitialized local
 * variable from the stack. The value depends on residual stack contents from
 * prior Fortran calls (e.g., ROSCO's DISCON controller), which vary with ASLR
 * across process invocations. This causes RectBivariateSpline.__call__ to
 * produce 1-ULP different results with identical inputs.
 *
 * Calling scrub_stack() before each spline evaluation zeros 64KB of stack,
 * ensuring deterministic results regardless of prior call history.
 *
 * See dev note: 202603261512-scipy-fitpack-bispev-uninitialized-variable.md
 *
 * Build: gcc -shared -fPIC -o rosco/lib/libscrub.so rosco/controller/src/scrub_stack.c
 */
#include <string.h>

void scrub_stack(void) {
    volatile char buf[65536];
    memset((char*)buf, 0, sizeof(buf));
}
