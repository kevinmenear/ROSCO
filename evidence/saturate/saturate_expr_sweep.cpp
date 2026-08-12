// The C++ side of saturate_expr_sweep.f90: the SHIPPED spelling, and the two
// branch spellings it was chosen over, on the same triples.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

static uint64_t bits(double x) { uint64_t u; std::memcpy(&u, &x, 8); return u; }

// shipped: intrinsic -> intrinsic
static double shipped(double v, double lo, double hi) { return std::fmin(std::fmax(v, lo), hi); }
// branch spelling A: first argument wins a tie / a NaN
static double branchA(double v, double lo, double hi) {
    double t = (v > lo) ? v : lo;
    return (t < hi) ? t : hi;
}
// branch spelling B: second argument wins a tie / a NaN
static double branchB(double v, double lo, double hi) {
    double t = (lo > v) ? lo : v;
    return (hi < t) ? hi : t;
}

int main(int argc, char** argv) {
    const char* which = (argc > 1) ? argv[1] : "shipped";
    double v, lo, hi;
    while (std::scanf("%lf %lf %lf", &v, &lo, &hi) == 3) {
        double r;
        if (std::strcmp(which, "branchA") == 0)      r = branchA(v, lo, hi);
        else if (std::strcmp(which, "branchB") == 0) r = branchB(v, lo, hi);
        else                                         r = shipped(v, lo, hi);
        std::printf("%016llX\n", (unsigned long long)bits(r));
    }
    return 0;
}
