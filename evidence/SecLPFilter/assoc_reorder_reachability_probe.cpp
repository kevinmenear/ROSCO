// Is `2.0*(DT*DT)*(CF*CF)` distinguishable from `(2.0*DT)*DT*(CF*CF)` over the
// REACHABLE domain? DT and CornerFreq are both direct arguments, so their own
// ranges ARE the input domain -- no local to derive (unit #13's second lesson).
#include <cstdio>
#include <cmath>
#include <cstring>
#include <initializer_list>
static bool bits_differ(double a, double b){ unsigned long long x,y; std::memcpy(&x,&a,8); std::memcpy(&y,&b,8); return x!=y; }
int main(){
    // b1 = 2.0*DT**2.0*CornerFreq**2.0 ; a1 = that - 8.0
    long nb1=0, na1=0; double wDT=0,wCF=0;
    for (int e = -1080; e <= 1080; ++e) {
        double DT = std::ldexp(1.0, e);
        for (int ce = -60; ce <= 60; ++ce) {
            double CF = std::ldexp(1.0, ce);
            double ref_b1 = 2.0 * (DT*DT) * (CF*CF);
            double mut_b1 = (2.0*DT) * DT * (CF*CF);
            double ref_a1 = 2.0 * (DT*DT) * (CF*CF) - 8.0;
            double mut_a1 = (2.0*DT) * DT * (CF*CF) - 8.0;
            if (bits_differ(ref_b1,mut_b1)) { if(!nb1){wDT=DT;wCF=CF;} ++nb1; }
            if (bits_differ(ref_a1,mut_a1)) ++na1;
        }
    }
    printf("power-of-two sweep: b1 differs in %ld pairs, a1 differs in %ld pairs\n", nb1, na1);
    printf("first witness: DT=%.17g CF=%.17g\n", wDT, wCF);
    // and the decimal rungs the corpus actually ships
    const double rungs[] = {1e-155,1e-156,1e-158,1e-160,1e-161,1e-162,1e-170,1e-200,1e-300,
                            std::sqrt(2.2250738585072014e-308), 1e300, 1e150, 1.0, 0.1, 0.025};
    for (double DT : rungs) {
        int hit = 0;
        for (double CF : {0.0, 1.0, 1e3, -1e3, 1.5708, 1e300, 1e-3}) {
            if (bits_differ(2.0*(DT*DT)*(CF*CF), (2.0*DT)*DT*(CF*CF))) ++hit;
        }
        printf("  DT=%-12.4g distinguishing CornerFreq values: %d of 7\n", DT, hit);
    }
    return 0;
}
