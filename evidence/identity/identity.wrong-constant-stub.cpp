// STUB, not a translation. The INPUT to a measurement (X4).
//
// The no-op stub leaves the automatic array uninitialised, so the difference it
// produces is NaN and KGen's `IF (rmsdiff > kgen_tolerance)` is FALSE -- IN_TOL,
// counted as PASSED. This stub is DETERMINATE and FINITE: the 3x3 identity with
// 2.0 on the diagonal. It is the liveness test the no-op cannot be.
void identity(int n, double* identity_result) {
    (void)n;
    identity_result[0] = 2.0;
    identity_result[1] = 0.0;
    identity_result[2] = 0.0;
    identity_result[3] = 0.0;
    identity_result[4] = 2.0;
    identity_result[5] = 0.0;
    identity_result[6] = 0.0;
    identity_result[7] = 0.0;
    identity_result[8] = 2.0;
}
