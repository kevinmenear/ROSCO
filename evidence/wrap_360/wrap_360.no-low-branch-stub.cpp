// STUB -- NOT the shipped translation. Only the LOW branch (x < 0 -> x + 360) deleted; the high branch kept.
double wrap_360(double x) {
    if (x >= 360.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
