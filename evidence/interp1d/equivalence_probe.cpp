// PROOF for three of interp1d's five surviving mutants. Unit #8's rule: a
// survivor is DECLARED only against a measurement, never against an argument.
//
//   20f2a843  compare_op  `if (x < xData_min)`  -> `<=`
//   23097b47  compare_op  `if (x > xData_max)`  -> `>=`
//   b61327db  compare_op  `n > 0 ? (size_t)n : 0` -> `n >= 0 ? ...`
//
// The first two are the MINVAL/MAXVAL reduction. Loosening the comparison makes
// the loop RE-ASSIGN on a tie, and the only way a re-assignment can change the
// stored bits is a signed zero -- `-0.0 <= 0.0` is true where `-0.0 < 0.0` is
// false. So the claim to prove is not "the value is the same" but the weaker,
// sufficient one: **the two consumers cannot tell the difference**, because
// `xData_min` and `xData_max` are read ONLY by `xq <= xData_min` and
// `xq >= xData_max`, and IEEE says `+0.0 == -0.0`.
//
// Swept exhaustively over every ordered tuple drawn from a value set that
// contains both zeros, duplicates, the infinities, a NaN and the extremes --
// which is exhaustive over the only structure the reduction has, since its
// answer depends on nothing but the multiset of values and their signs.
//
// The third is arithmetic on a 32-bit int and is swept over the WHOLE type.
//
//   g++ -O2 -ffp-contract=off -o eqprobe equivalence_probe.cpp && ./eqprobe

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

namespace {

const double VALUES[] = {
    0.0, -0.0, 1.0, -1.0, 3.0, 3.0, -3.0, 1e-300, -1e-300,
    DBL_MAX, -DBL_MAX, DBL_MIN, -DBL_MIN,
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN(),
};
constexpr int NV = static_cast<int>(sizeof(VALUES) / sizeof(VALUES[0]));

// The shipped reduction, and the two mutants of it.
void reduce(const double* a, int n, bool loose_min, bool loose_max,
            double& mn, double& mx) {
    mn = DBL_MAX;
    mx = -DBL_MAX;
    for (int I = 1; I <= n; ++I) {
        const double x = a[I - 1];
        if (loose_min ? (x <= mn) : (x < mn)) mn = x;
        if (loose_max ? (x >= mx) : (x > mx)) mx = x;
    }
}

}  // namespace

int main() {
    // ---- 20f2a843 and 23097b47 -------------------------------------------
    long long tuples = 0, bits_differ = 0, consumer_differs = 0;
    double a[4];
    for (int n = 0; n <= 4; ++n) {
        std::vector<int> idx(n, 0);
        for (;;) {
            for (int k = 0; k < n; ++k) a[k] = VALUES[idx[k]];
            double mn0, mx0, mn1, mx1;
            reduce(a, n, false, false, mn0, mx0);
            reduce(a, n, true, true, mn1, mx1);
            ++tuples;
            uint64_t u0, u1;
            __builtin_memcpy(&u0, &mn0, 8);
            __builtin_memcpy(&u1, &mn1, 8);
            bool differ = (u0 != u1);
            __builtin_memcpy(&u0, &mx0, 8);
            __builtin_memcpy(&u1, &mx1, 8);
            differ = differ || (u0 != u1);
            if (differ) ++bits_differ;
            // The two consumers, over every xq in the same set.
            for (int q = 0; q < NV; ++q) {
                const double xq = VALUES[q];
                if ((xq <= mn0) != (xq <= mn1)) ++consumer_differs;
                if ((xq >= mx0) != (xq >= mx1)) ++consumer_differs;
            }
            int k = n - 1;
            while (k >= 0 && ++idx[k] == NV) idx[k--] = 0;
            if (k < 0) break;
        }
    }
    std::printf("20f2a843 / 23097b47  reduction `<`/`>` against `<=`/`>=`\n");
    std::printf("  tuples swept (lengths 0..4 over %d values): %lld\n", NV, tuples);
    std::printf("  tuples whose stored min/max BITS differ:    %lld\n", bits_differ);
    std::printf("  (tuple, xq) pairs whose CONSUMER differs:   %lld\n", consumer_differs);

    // ---- b61327db ---------------------------------------------------------
    long long swept = 0, len_differs = 0;
    for (int64_t v = INT32_MIN; v <= INT32_MAX; ++v) {
        const int n = static_cast<int>(v);
        const size_t tight = n > 0 ? static_cast<size_t>(n) : 0;
        const size_t loose = n >= 0 ? static_cast<size_t>(n) : 0;
        ++swept;
        if (tight != loose) ++len_differs;
    }
    std::printf("\nb61327db  `n > 0 ? (size_t)n : 0` against `n >= 0 ? ...`\n");
    std::printf("  every value of a 32-bit int swept: %lld\n", swept);
    std::printf("  values at which the LENGTH differs: %lld\n", len_differs);
    return 0;
}
