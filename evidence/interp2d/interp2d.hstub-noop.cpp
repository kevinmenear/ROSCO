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
//
// THE UNREACHABLE `interp1d_c` CALL IS LOAD-BEARING, AND IT IS ABOUT THE LINK
// RATHER THAN ABOUT THE ARITHMETIC. `vit/test_validate.generate_callee_bridges`
// decides whether to emit a callee bridge -- and whether the Makefile compiles
// and links `interp2d_callees.o` at all -- by scanning THE TRANSLATION'S TEXT
// for `<name>_c(`. A stub with no such call is built with no callee bridge, and
// the link then dies on `undefined reference to interp1d_c` raised by
// `pitchsaturation.cpp.o` and `powercontrolsetpoints.cpp.o` -- two OTHER
// integrated units that call it -- because `scripts/harness.sh` has dropped
// `interp1d.cpp.o` from LIBS in favour of the bridge that no longer exists.
//
// A red test that fails to BUILD is not a red test (unit #42: a build failure
// and a perturbation that moves nothing end the same way). So the call stands,
// behind a guard the harness cannot satisfy: `zData` is `std::vector::data()`
// on a vector of `n_zData_rows * n_zData_cols >= 9` elements in every case.

#include "vit_types.h"

double interp2d(double* xData, int n_xData, double* yData, int n_yData,
                double* zData, int n_zData_rows, int n_zData_cols, double xq,
                double yq, errorvariables_view_t* ErrVar) {
    (void)n_zData_rows; (void)n_zData_cols; (void)yq;
    if (zData == nullptr) {
        return interp1d_c(xData, n_xData, yData, n_yData, xq, ErrVar);
    }
    return 0.0;
}
