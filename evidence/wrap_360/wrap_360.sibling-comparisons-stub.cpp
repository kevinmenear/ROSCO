// STUB -- NOT the shipped translation. The two comparisons spelled as wrap_180 spells them -- '<=' low and '>' high instead of '<' and '>='. Moves exactly the two endpoints x = 0.0 and x = 360.0.
double wrap_360(double x) {
    if (x <= 0.0) {
        return x + 360.0;
    } else if (x > 360.0) {
        return x - 360.0;
    } else {
        return x;
    }
}
