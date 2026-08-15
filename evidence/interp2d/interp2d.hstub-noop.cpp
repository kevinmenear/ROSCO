// INPUT to the differential harness's red test: unit #45 `interp2d` as a NO-OP.
//
// It reads no argument, writes no view field and returns a constant. If the
// harness's green can be moved at all, this moves it; a stub that cannot is a
// harness comparing nothing.
//
// Zero rather than a NaN, for unit #22's reason: a stub leaving an output NaN
// scores as agreement under a tolerance comparison, because `NaN > tol` is
// false. The harness compares stored bits rather than a tolerance, but the
// habit is cheap and the alternative is a red test that reports green for the
// wrong reason.

#include "vit_types.h"

double interp2d(double* xData, int n_xData, double* yData, int n_yData,
                double* zData, int n_zData_rows, int n_zData_cols, double xq,
                double yq, errorvariables_view_t* ErrVar) {
    (void)xData; (void)n_xData; (void)yData; (void)n_yData;
    (void)zData; (void)n_zData_rows; (void)n_zData_cols;
    (void)xq; (void)yq; (void)ErrVar;
    return 0.0;
}
