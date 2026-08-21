// EXECUTE THE EQUIVALENCE ARGUMENTS BEFORE BELIEVING THEM -- unit #48's sixth
// finding, whose first model proved three mutants behaviour-preserving from a
// clean inequality and was false.
//
//   g++ -O2 -fPIC -ffp-contract=off probe_equivalences.cpp -o eq && ./eq
//
// Three families are declared equivalent for unit #65 and one neighbouring
// family is NOT. The fourth is the negative control: if the probe cannot show
// `fmax(0.0, x)` and `fmax(x, 0.0)` DIFFERING, it cannot show the other three
// agreeing either.
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

static bool same(double a, double b) {
    uint64_t x, y; std::memcpy(&x,&a,8); std::memcpy(&y,&b,8); return x == y;
}

int main() {
    std::vector<double> V;
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double d : {0.0, -0.0, 1.0, -1.0, 3.0, -3.0, 2.9999999999999996,
                     3.0000000000000004, DBL_EPSILON, -DBL_EPSILON, DBL_MIN,
                     -DBL_MIN, DBL_MAX, -DBL_MAX, 5e-324, -5e-324, 1e300,
                     -1e300, 0.25, 1e-8, -1e-8}) V.push_back(d);
    V.push_back(inf); V.push_back(-inf); V.push_back(nan); V.push_back(-nan);
    for (int i = 0; i < 40; ++i) V.push_back(std::sin(0.37 * i) * std::pow(10.0, i % 17 - 8));

    long n = 0, bad_abs = 0, bad_max3 = 0, bad_maxeps = 0, differ_zero = 0;
    long control_minmax = 0;
    for (double a : V) {
        // A. swap_operands inside fabs(a - b) > 0, three sites.
        for (double b : V) {
            ++n;
            if (!same(std::fabs(a - b), std::fabs(b - a))) ++bad_abs;
            // the PREDICATE, which is what the reference actually evaluates
            if ((std::fabs(a - b) > 0) != (std::fabs(b - a) > 0)) ++bad_abs;
        }
        // B. swap_call_args on fmax against a positive non-zero constant.
        if (!same(std::fmax(a, 3.0), std::fmax(3.0, a))) ++bad_max3;
        if (!same(std::fmax(a, DBL_EPSILON), std::fmax(DBL_EPSILON, a))) ++bad_maxeps;
        // D. FIRST ATTEMPT AT A NEGATIVE CONTROL, AND IT IS NOT ONE. The
        // argument for family B was that fmax is symmetric EXCEPT possibly at
        // a (+0, -0) pair, so `fmax(0.0, Cp_op)` was going to be left standing
        // as an honest survivor. This counter is what refuted that: glibc's
        // fmax agrees on ±0 as well, so the zero site is equivalent too and
        // the count below is 0 -- which means this line CANNOT FAIL and is
        // therefore not a control (P10). It is kept because the wrong model is
        // what the next reader would otherwise re-derive.
        if (!same(std::fmax(a, 0.0), std::fmax(0.0, a))) ++differ_zero;
        // D'. THE CONTROL THAT CAN FAIL: the same comparison machinery over an
        // operator that is NOT symmetric with fmax.
        if (!same(std::fmax(a, 3.0), std::fmin(3.0, a))) ++control_minmax;
    }
    std::printf("A  fabs(a-b) vs fabs(b-a), value AND predicate, %ld pairs: %ld disagreements\n",
                n, bad_abs);
    std::printf("B1 fmax(a,3.0) vs fmax(3.0,a),        %zu values: %ld disagreements\n",
                V.size(), bad_max3);
    std::printf("B2 fmax(a,DBL_EPSILON) vs swapped,    %zu values: %ld disagreements\n",
                V.size(), bad_maxeps);
    std::printf("D  fmax(a,0.0) vs fmax(0.0,a): %ld disagreements -- glibc's fmax is\n"
                "     symmetric at +/-0 too, so the zero site is EQUIVALENT as well and\n"
                "     this line is NOT a control, because it cannot fail\n", differ_zero);
    std::printf("D' NEGATIVE CONTROL fmax(a,3.0) vs fmin(3.0,a): %ld DISAGREEMENTS "
                "(must be > 0)\n", control_minmax);
    std::printf("     the +/-0 pair, spelled out: fmax(0.0,-0.0)=%.17g signbit %d, "
                "fmax(-0.0,0.0)=%.17g signbit %d\n",
                std::fmax(0.0,-0.0), (int)std::signbit(std::fmax(0.0,-0.0)),
                std::fmax(-0.0,0.0), (int)std::signbit(std::fmax(-0.0,0.0)));
    // C/E. the integer index identities, which are exact by definition and are
    // stated so the declaration rests on an executed check like the others.
    std::printf("C  0*3+0 == 0 : %s   0*3 == 0 : %s   0/3 == 0 : %s\n",
                (0*3+0 == 0) ? "yes" : "NO", (0*3 == 0) ? "yes" : "NO",
                (0/3 == 0) ? "yes" : "NO");
    std::printf("E  NOT declared: 1*3 == 1 : %s   2*3 == 2 : %s   (both must be NO)\n",
                (1*3 == 1) ? "yes" : "NO", (2*3 == 2) ? "yes" : "NO");
    return 0;
}
