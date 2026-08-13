// STUB -- NOT the shipped translation. A DETERMINATE WRONG constant, chosen against the call site's arguments (unit #25): Controllers.f90:845 passes 360*Time*AWC_freq(1), which is >= 0 for every invocation, so no case can legitimately return a negative number and -7.25 is wrong everywhere.
double wrap_360(double x) {
    (void)x;
    return -7.25;
}
