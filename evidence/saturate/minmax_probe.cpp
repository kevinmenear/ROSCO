// The two candidate C++ spellings of Fortran MAX/MIN, plus fmax/fmin, over the
// same runtime-supplied inputs as minmax_probe.f90. Bits, not values.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

static uint64_t bits(double x) { uint64_t u; std::memcpy(&u, &x, 8); return u; }

// spelling A: the FIRST argument wins a tie / a NaN
static double maxA(double a, double b) { return (a > b) ? a : b; }
static double minA(double a, double b) { return (a < b) ? a : b; }
// spelling B: the SECOND argument wins a tie / a NaN
static double maxB(double a, double b) { return (b > a) ? b : a; }
static double minB(double a, double b) { return (b < a) ? b : a; }

int main() {
    double a, b;
    int n = 0;
    while (std::scanf("%lf %lf", &a, &b) == 2) {
        ++n;
        std::printf("case%3d %016llX %016llX  maxA=%016llX maxB=%016llX fmax=%016llX"
                    "  minA=%016llX minB=%016llX fmin=%016llX\n",
                    n,
                    (unsigned long long)bits(a), (unsigned long long)bits(b),
                    (unsigned long long)bits(maxA(a, b)), (unsigned long long)bits(maxB(a, b)),
                    (unsigned long long)bits(std::fmax(a, b)),
                    (unsigned long long)bits(minA(a, b)), (unsigned long long)bits(minB(a, b)),
                    (unsigned long long)bits(std::fmin(a, b)));
    }
    return 0;
}
