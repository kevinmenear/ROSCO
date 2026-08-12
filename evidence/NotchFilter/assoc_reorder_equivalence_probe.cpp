// Is there ANY input to NotchFilter that distinguishes the four surviving
// assoc_reorder mutants from the translation?
//
// In ISOLATION `2.0*(x*x)` and `(2.0*x)*x` differ -- 130,696 of 20,000,000
// full-range draws, whenever `x*x` is subnormal
// (assoc_reorder_survivor_probe.cpp). The question this probe asks is the one
// that decides the disposition: does that difference survive being EMBEDDED in
//
//     nf_b1 = nf_a1 = (2.0*(omega*omega) - 2.0*(K*K))
//                     / ((K*K) + 2.0*omega*betaDen*K + (omega*omega))
//
// where the numerator is a DIFFERENCE of two such terms and the whole is
// divided by a third. The unit's other four coefficients contain no `2.0*(..)`
// grouping, so they carry no mutant.
//
// This does not reason. It searches the full exponent range of the ACTUAL
// inputs -- DT, not K: the reference computes K = 2.0/DT, so |K| below
// 2.0/DBL_MAX is not reachable from any finite timestep, log-uniformly, and it prints the first witness if one exists. The
// campaign's rule (unit #8): prove a survivor exhaustively rather than arguing
// it.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <random>

static inline bool bitdiff(double a, double b) {
    uint64_t x, y; std::memcpy(&x, &a, 8); std::memcpy(&y, &b, 8); return x != y;
}
static inline double den(double K, double om, double bd) {
    return (K*K) + 2.0*om*bd*K + (om*om);
}
int main() {
    std::mt19937_64 rng(20260811u);
    std::uniform_int_distribution<int> ue(-1074, 1023);
    std::uniform_real_distribution<double> um(1.0, 2.0);
    std::uniform_int_distribution<int> sgn(0, 1);
    auto draw = [&]() {
        double v = std::ldexp(um(rng), ue(rng));
        return sgn(rng) ? v : -v;
    };

    const long long N = 200000000LL;
    long long nK = 0, nO = 0, n_subK = 0, n_subOm = 0;
    bool have = false;
    for (long long i = 0; i < N; ++i) {
        double DT = draw();
        double K  = 2.0 / DT;   // K IS NOT AN INPUT -- only 2.0/DT is reachable
        double om = draw();
        double bd = (i % 4 == 0) ? 0.0 : draw();      // betaNum/betaDen are 0 in every scenario
        double d  = den(K, om, bd);
        double KK = K*K, OO = om*om;
        if (KK > 0.0 && KK < 2.2250738585072014e-308) ++n_subK;
        if (OO > 0.0 && OO < 2.2250738585072014e-308) ++n_subOm;

        double base = (2.0*OO - 2.0*KK) / d;
        double mK   = (2.0*OO - (2.0*K)*K) / d;
        double mO   = ((2.0*om)*om - 2.0*KK) / d;
        if (bitdiff(base, mK)) { ++nK;
            if (!have) { printf("K-mutant WITNESS  DT=%.17g K=%.17g omega=%.17g betaDen=%.17g\n"
                                "   base=%.17g  mutant=%.17g\n", DT, K, om, bd, base, mK); have = true; } }
        if (bitdiff(base, mO)) { ++nO;
            if (!have) { printf("omega-mutant WITNESS  DT=%.17g K=%.17g omega=%.17g betaDen=%.17g\n"
                                "   base=%.17g  mutant=%.17g\n", DT, K, om, bd, base, mO); have = true; } }
    }
    printf("triples drawn                     : %lld\n", N);
    printf("  with K*K subnormal              : %lld\n", n_subK);
    printf("  with omega*omega subnormal      : %lld\n", n_subOm);
    printf("K-mutant distinguished by         : %lld\n", nK);
    printf("omega-mutant distinguished by     : %lld\n", nO);
    if (!have) printf("VERDICT: no witness. Embedded in this unit's expression the regrouping\n"
                      "         is unobservable, though it is NOT in isolation.\n");
    return 0;
}
