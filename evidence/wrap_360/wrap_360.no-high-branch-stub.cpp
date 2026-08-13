// STUB -- NOT the shipped translation. Only the HIGH branch (x >= 360 -> x - 360) deleted; the low branch kept.
double wrap_360(double x) {
    if (x < 0.0) {
        return x + 360.0;
    } else {
        return x;
    }
}
