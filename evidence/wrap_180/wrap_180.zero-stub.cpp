// STUB, not the translation. The campaign's DEFAULT liveness stub, kept to show
// why unit #25 says not to use it blind: case wrap_180.0.0.1 captures
// x = 0.0 exactly, so a zero stub returns the RIGHT answer there.
double wrap_180(double x) {
    (void)x;
    return 0.0;
}
