// Are the four surviving mutants EQUIVALENT, or merely unreached?
//
//   original: 2.0 * (x * x)        mutant: (2.0 * x) * x
//
// Multiplying by 2 is exact in IEEE-754, and rounding is invariant under
// scaling by a power of two -- IN THE NORMAL RANGE. So the two agree
// everywhere the generator has ever drawn from. They cannot agree in the
// SUBNORMAL range, where the grid is absolute rather than relative:
// 2*round(y) lands on an even multiple of 2^-1074 while round(2y) may land on
// an odd one.
//
// This probe does not reason -- it searches, and it prints the first witness.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <random>

static bool differ(double x, double& a, double& b) {
    a = 2.0 * (x * x);
    b = (2.0 * x) * x;
    uint64_t ia, ib; std::memcpy(&ia, &a, 8); std::memcpy(&ib, &b, 8);
    return ia != ib;
}

int main() {
    double a, b;
    long long n_normal = 0, n_sub = 0, tried_normal = 0, tried_sub = 0;
    double first_witness = 0.0; bool have = false;

    // 1. the range the differential harness actually draws from: |v| <= 1e3,
    //    plus the reference's own literals.
    std::mt19937_64 rng(20260811);
    std::uniform_real_distribution<double> u(-1000.0, 1000.0);
    for (long long i = 0; i < 20000000; ++i) {
        double x = u(rng); ++tried_normal;
        if (differ(x, a, b)) { ++n_normal; if (!have) { first_witness = x; have = true; } }
    }

    // 2. the whole exponent range, uniform in the BIT PATTERN of the exponent.
    std::uniform_int_distribution<int> ue(-1074, 1023);
    std::uniform_real_distribution<double> um(1.0, 2.0);
    long long n_wide = 0, tried_wide = 0;
    for (long long i = 0; i < 20000000; ++i) {
        double x = std::ldexp(um(rng), ue(rng)); ++tried_wide;
        if (differ(x, a, b)) { ++n_wide; if (!have) { first_witness = x; have = true; } }
    }

    // 3. the subnormal-product region explicitly: x such that x*x is subnormal
    //    (|x| below ~1.49e-162).
    for (long long i = 0; i < 20000000; ++i) {
        double x = std::ldexp(um(rng), -537 - (int)(i % 500)); ++tried_sub;
        if (differ(x, a, b)) { ++n_sub; if (!have) { first_witness = x; have = true; } }
    }

    printf("|x| <= 1e3            : %lld of %lld differ\n", n_normal, tried_normal);
    printf("full exponent range   : %lld of %lld differ\n", n_wide, tried_wide);
    printf("x*x subnormal         : %lld of %lld differ\n", n_sub, tried_sub);
    if (have) {
        uint64_t ix; std::memcpy(&ix, &first_witness, 8);
        differ(first_witness, a, b);
        uint64_t ia, ib; std::memcpy(&ia, &a, 8); std::memcpy(&ib, &b, 8);
        printf("FIRST WITNESS x = %.17g  (bits %016llx)\n", first_witness,
               (unsigned long long)ix);
        printf("  2.0*(x*x)  = %.17g  (bits %016llx)\n", a, (unsigned long long)ia);
        printf("  (2.0*x)*x  = %.17g  (bits %016llx)\n", b, (unsigned long long)ib);
        printf("VERDICT: the mutant is NOT equivalent -- a distinguishing double exists.\n");
    } else {
        printf("VERDICT: no witness found in 60,000,000 draws.\n");
    }
    return 0;
}
