// KERNEL RED TEST — INPUT to a measurement, not the measurement.
//
// A determinate, finite stub that reads no argument and writes nothing. Zero
// is well outside the reference's range for this call site (A_op is of order
// -1e-2 to -5e-1), so `rmsdiff` is of order 1e-2 against an absolute
// `kgen_tolerance` of 1.D-14 -- this stub is DETERMINATE, which unit #22
// showed is the property that matters: a stub that leaves an output NaN scores
// IN_TOL, because `NaN > tol` is false.
//
// Run with: vit verify interp1d <this file> -f <clean Functions.f90>
//           --kernel-dir kernel/interp1d
// The result belongs in evidence/interp1d/kernel.*.run.txt.

#include "vit_types.h"

double interp1d(double* xData, int n_xData, double* yData, int n_yData, double xq,
                errorvariables_view_t* ErrVar) {
    (void)xData;
    (void)n_xData;
    (void)yData;
    (void)n_yData;
    (void)xq;
    (void)ErrVar;
    return 0.0;
}
