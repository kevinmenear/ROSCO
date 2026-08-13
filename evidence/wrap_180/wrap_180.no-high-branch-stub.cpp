// STUB, not the translation. Only the `x .gt. 180.0 -> x - 360.0` branch deleted.
double wrap_180(double x) {
    if (x <= -180.0) { return x + 360.0; }
    return x;
}
