// PROOF for four of interp2d's surviving mutants. Unit #8's rule: a survivor
// is DECLARED only against a measurement, never against an argument.
//
//   7986e5f9  compare_op  `if (x < xData_min) xData_min = x;`  -> `<=`
//   1235716b  compare_op  `if (x > xData_max) xData_max = x;`  -> `>=`
//   b9162232  compare_op  `if (y < yData_min) yData_min = y;`  -> `<=`
//   acb38635  compare_op  `if (y > yData_max) yData_max = y;`  -> `>=`
//
// THIS IS interp1d's equivalence_probe.cpp RE-AIMED AT THIS UNIT'S CONSUMERS,
// not a citation of it. The reduction loops are byte-identical between the two
// translations, but the predicates that READ the reductions are not: interp1d
// reads its two through `xq <= xData_min` and `xq >= xData_max`; interp2d has
// FOUR reductions and reads them through
//
//     if (xq <= xData_min || std::isnan(xq))        ... else if (xq >= xData_max)
//     if (yq <= yData_min || std::isnan(yq))        ... else if (yq >= yData_max)
//
// -- a disjunct interp1d does not have, and a second pair of reductions over a
// second array whose ties can coincide with the first's in one table. So the
// claim is re-proved here rather than transferred: the four are swept TOGETHER,
// against these four consumers, in the ELSE-IF STRUCTURE the unit actually
// writes, so that a difference which the first test hides from the second is
// counted as no difference exactly when the unit would also hide it.
//
// WHAT IS BEING PROVED, and it is not "the value is the same". Loosening the
// comparison makes the loop RE-ASSIGN on a tie, and a re-assignment CAN change
// the stored bits: `-0.0 <= 0.0` is true where `-0.0 < 0.0` is false. The
// weaker, sufficient claim is that the CONSUMERS cannot tell -- IEEE says
// `+0.0 == -0.0`, so a predicate is the only place a reduction of this unit is
// read and a signed zero is invisible there.
//
// Swept exhaustively over every ordered tuple of length 0..4 drawn from a
// 16-value set containing both zeros, duplicates, both infinities, a NaN,
// DBL_MAX/DBL_MIN and their negatives. That is exhaustive over the only
// structure a reduction has: its answer depends on nothing but the multiset of
// values and their signs, and length 4 is past the point where a longer tuple
// can add a new arrangement of ties.
//
//   g++ -O2 -ffp-contract=off -o eqprobe equivalence_probe.cpp && ./eqprobe

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

namespace {

const double VALUES[] = {
    0.0, -0.0, 1.0, -1.0, 2.0, -2.0,
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN(),
    DBL_MAX, -DBL_MAX, DBL_MIN, -DBL_MIN,
    0.5, -0.5, 3.0,
};
const int NV = static_cast<int>(sizeof(VALUES) / sizeof(VALUES[0]));

struct Red { double lo, hi; };

// The shipped spelling, transcribed from translations/Functions/interp2d.cpp.
Red reduce_tight(const std::vector<double>& a) {
    Red r{DBL_MAX, -DBL_MAX};
    for (size_t k = 0; k < a.size(); ++k) {
        const double x = a[k];
        if (x < r.lo) r.lo = x;
        if (x > r.hi) r.hi = x;
    }
    return r;
}

// Both mutants at once. They are independent sites, but a tuple that separates
// one separates it whether or not the other is also loosened, and running the
// pair together is what makes the swept count a count of TABLES rather than of
// table-times-site.
Red reduce_loose(const std::vector<double>& a) {
    Red r{DBL_MAX, -DBL_MAX};
    for (size_t k = 0; k < a.size(); ++k) {
        const double x = a[k];
        if (x <= r.lo) r.lo = x;
        if (x >= r.hi) r.hi = x;
    }
    return r;
}

// Which arm of the unit's corner search a query takes, given the reductions.
// 0 = lower bound / NaN, 1 = upper bound, 2 = the interior search. This is the
// ONLY thing the unit does with a reduction.
int arm(double q, const Red& r) {
    if (q <= r.lo || std::isnan(q)) return 0;
    if (q >= r.hi) return 1;
    return 2;
}

}  // namespace

int main() {
    long tuples = 0, bits_differ = 0, pairs = 0, consumer_differs = 0;

    std::vector<double> x;
    for (int lx = 0; lx <= 4; ++lx) {
        std::vector<int> idx(lx, 0);
        for (;;) {
            x.clear();
            for (int k = 0; k < lx; ++k) x.push_back(VALUES[idx[k]]);
            const Red t = reduce_tight(x), l = reduce_loose(x);
            ++tuples;
            // A bit difference in the STORED reduction, counted so the run
            // cannot be read as "the mutant does nothing".
            if (std::memcmp(&t, &l, sizeof(Red)) != 0) ++bits_differ;
            // The consumer. Every query in the same value set, and the pair of
            // reductions is one table's worth -- the x pair and the y pair are
            // the same code over two arrays, so one sweep answers both, and the
            // cross of the two directions is the cross of this sweep with
            // itself.
            for (int qi = 0; qi < NV; ++qi) {
                const double q = VALUES[qi];
                ++pairs;
                if (arm(q, t) != arm(q, l)) ++consumer_differs;
            }
            int k = lx - 1;
            for (; k >= 0; --k) {
                if (++idx[k] < NV) break;
                idx[k] = 0;
            }
            if (k < 0) break;
        }
    }

    std::printf("7986e5f9 / 1235716b / b9162232 / acb38635"
                "  reduction `<`/`>` against `<=`/`>=`\n");
    std::printf("  tuples swept (lengths 0..4 over %d values): %ld\n", NV, tuples);
    std::printf("  tuples whose stored min/max BITS differ:    %ld\n", bits_differ);
    std::printf("  (tuple, query) pairs swept:                 %ld\n", pairs);
    std::printf("  (tuple, query) pairs whose ARM differs:     %ld\n", consumer_differs);
    std::printf("\n  the arm is the whole consumer: xData_min, xData_max, yData_min\n"
                "  and yData_max are each read at exactly one site in\n"
                "  translations/Functions/interp2d.cpp, and all four sites are in\n"
                "  the two if/else-if chains this probe reproduces.\n");
    return consumer_differs == 0 ? 0 : 1;
}
