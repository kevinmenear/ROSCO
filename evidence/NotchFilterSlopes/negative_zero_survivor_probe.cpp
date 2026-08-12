// Is the surviving mutant equivalent?
//
//   38bbc289  compare_op   'if (CornerFreq < 0)'  ->  'if (CornerFreq <= 0)'
//
// The two predicates disagree on exactly one input: CornerFreq == 0. For +0.0
// both branches produce +0.0 and nothing can tell them apart. For NEGATIVE zero
// they do not: the reference takes the ELSE and carries -0.0 forward, the mutant
// takes the THEN and substitutes +0.0.
//
// Whether that difference REACHES AN OUTPUT is the question, and it is not
// obvious -- IEEE addition collapses (+0.0) + (-0.0) to +0.0, so the returned
// value may well agree. This probe asks it of every output the unit writes, not
// only of the return value, because `FP%nfs_b2` and `FP%nfs_b0` are outputs the
// differential harness compares on the exact bit pattern.
//
// Per unit #13's rule, the draw is over the ARGUMENTS the signature has. DT,
// Damp and CornerFreq are all dummies of the original -- no local is being given
// a domain it cannot reach.
#include <cstdio>
#include <cstring>
#include <cstdint>

static uint64_t bits(double x) { uint64_t u; std::memcpy(&u, &x, 8); return u; }

struct Out { double b2, b0, a2, a1, a0, res, il1, il2, ol1, ol2; };

// The unit, reduced to the coefficient block and the filter expression, with
// the saturation predicate as a parameter. State starts at the reset values so
// the case is the one the harness generates for iStatus == 0.
static Out run(double InputSignal, double DT, double CornerFreq, double Damp, bool le) {
    double CornerFreq_;
    if (le ? (CornerFreq <= 0) : (CornerFreq < 0)) CornerFreq_ = 0; else CornerFreq_ = CornerFreq;

    double IL1 = InputSignal, IL2 = InputSignal, OL1 = InputSignal, OL2 = InputSignal;
    Out o;
    o.b2 = 2.0 * DT * CornerFreq_;
    o.b0 = -o.b2;
    o.a2 = Damp * (DT * DT) * (CornerFreq_ * CornerFreq_) + 2.0 * DT * CornerFreq_ + 4.0 * Damp;
    o.a1 = 2.0 * Damp * (DT * DT) * (CornerFreq_ * CornerFreq_) - 8.0 * Damp;
    o.a0 = Damp * (DT * DT) * (CornerFreq_ * CornerFreq_) - 2.0 * DT * CornerFreq_ + 4.0 * Damp;
    o.res = 1.0 / o.a2 * (o.b2 * InputSignal + o.b0 * IL1 - o.a1 * OL1 - o.a0 * OL2);
    o.il2 = IL1; o.il1 = InputSignal; o.ol2 = OL1; o.ol1 = o.res;
    return o;
}

int main() {
    const double zeros[2] = { 0.0, -0.0 };
    const char* zname[2] = { "+0.0", "-0.0" };
    const double dts[4]  = { 0.0125, 1.0, -0.5, 1e300 };
    const double damps[3] = { 0.7, 1.0, 0.0 };
    const double ins[3]  = { 1.0, -3.25, 0.0 };

    int differ_total = 0;
    for (int z = 0; z < 2; ++z) {
        int differ = 0, checked = 0;
        const char* first_field = nullptr;
        for (int a = 0; a < 4; ++a) for (int b = 0; b < 3; ++b) for (int c = 0; c < 3; ++c) {
            Out r = run(ins[c], dts[a], zeros[z], damps[b], false);
            Out m = run(ins[c], dts[a], zeros[z], damps[b], true);
            ++checked;
            const char* f = nullptr;
            if (bits(r.b2)  != bits(m.b2))  f = "nfs_b2";
            else if (bits(r.b0)  != bits(m.b0))  f = "nfs_b0";
            else if (bits(r.a2)  != bits(m.a2))  f = "nfs_a2";
            else if (bits(r.a1)  != bits(m.a1))  f = "nfs_a1";
            else if (bits(r.a0)  != bits(m.a0))  f = "nfs_a0";
            else if (bits(r.res) != bits(m.res)) f = "the return value";
            else if (bits(r.ol1) != bits(m.ol1)) f = "nfs_OutputSignalLast1";
            if (f) { ++differ; if (!first_field) first_field = f; }
        }
        std::printf("CornerFreq = %-5s  differ %d of %d", zname[z], differ, checked);
        if (first_field) std::printf("   first differing output: %s", first_field);
        std::printf("\n");
        differ_total += differ;
    }

    // The single witness, spelled out.
    Out r = run(1.0, 0.0125, -0.0, 0.7, false);
    Out m = run(1.0, 0.0125, -0.0, 0.7, true);
    std::printf("\nwitness  InputSignal=1.0  DT=0.0125  CornerFreq=-0.0  Damp=0.7\n");
    std::printf("  reference nfs_b2 = %.17g  bits %016llx\n", r.b2, (unsigned long long)bits(r.b2));
    std::printf("  mutant    nfs_b2 = %.17g  bits %016llx\n", m.b2, (unsigned long long)bits(m.b2));
    std::printf("  reference nfs_b0 = %.17g  bits %016llx\n", r.b0, (unsigned long long)bits(r.b0));
    std::printf("  mutant    nfs_b0 = %.17g  bits %016llx\n", m.b0, (unsigned long long)bits(m.b0));
    std::printf("  return value      %s\n", bits(r.res) == bits(m.res) ? "AGREES -- the return value alone cannot kill this mutant" : "differs");
    std::printf("\nVERDICT: %s\n", differ_total > 0
        ? "NOT EQUIVALENT -- a reachable argument value distinguishes the two"
        : "no witness found by this probe");
    return 0;
}
