// SECOND EQUIVALENCE PROBE -- unit #65 `WindSpeedEstimator`, added at the
// dispatch that re-took P12. Additive to `probe_equivalences.cpp` (P5): that
// file's four families are unchanged and are not re-run here.
//
//   g++ -O2 -fPIC -ffp-contract=off probe_equivalences2.cpp -o eq2 && ./eq2
//
// THREE FAMILIES, EACH WITH A CONTROL THAT CAN FAIL. Unit #48's sixth finding
// is the rule: an equivalence argument that has not been executed is a guess,
// and a probe whose confirming line cannot fail is not evidence (P10). Every
// family below is paired with the NEIGHBOURING mutant the same argument does
// NOT cover, and that neighbour must disagree.
//
//   G  `drop_call` on `ABS(...)` at the TORQUE and SPEED range tests
//        068d3be3   fabs(WE_Inp_Torque - VS_LastGenTrqF) > 0  ->  ... > 0
//        cd428761   fabs(WE_Inp_Speed  - RotSpeedF)      > 0  ->  ... > 0
//      CONTROL: the PITCH test one line above, which the sweep KILLED.
//
//   H  `arith_op` on `v_m + v_t` in the EKF RESTART arm
//        d3f2e7f0   LocalVar%WE%v_m + LocalVar%WE%v_t  ->  v_m - v_t
//      CONTROL: the same +/- pair over an UNCONSTRAINED left operand.
//
//   I  `compare_op` on the length guard in `errmsg_trim`
//        a1db7de8   n > 0 ? (size_t)n : 0   ->   n >= 0 ? (size_t)n : 0
//      CONTROL: the sibling `const_tweak` 5f847ebc (`n > 0` -> `n > 1`),
//      which is NOT declared and must disagree.
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

static bool same(double a, double b) {
    uint64_t x, y; std::memcpy(&x, &a, 8); std::memcpy(&y, &b, 8); return x == y;
}

// The reference's own two one-sided saturations, transcribed from
// ControllerBlocks.f90 (clean, 54dd134) rather than from the translation.
static double sat_torque(double last, double rttq) {
    return (last < 0.0001 * rttq) ? 0.0001 * rttq : last;
}
static double sat_speed(double rotspeedf, double minomspd, double gbr) {
    const double b = 0.25 * minomspd / gbr;
    return (rotspeedf < b) ? b + DBL_EPSILON : rotspeedf;
}
// `saturate` is `MIN(MAX(v,lo),hi)`, measured as `fmin(fmax(..))` by unit #24.
static double sat_pitch(double v, double lo, double hi) {
    return std::fmin(std::fmax(v, lo), hi);
}

int main() {
    std::vector<double> V;
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double d : {0.0, -0.0, 1.0, -1.0, 3.0, -3.0, 2.9999999999999996,
                     3.0000000000000004, DBL_EPSILON, -DBL_EPSILON, DBL_MIN,
                     -DBL_MIN, DBL_MAX, -DBL_MAX, 5e-324, -5e-324, 1e300,
                     -1e300, 0.25, 1e-8, -1e-8, 1e5, -1e5}) V.push_back(d);
    V.push_back(inf); V.push_back(-inf); V.push_back(nan); V.push_back(-nan);
    for (int i = 0; i < 40; ++i) V.push_back(std::sin(0.37 * i) * std::pow(10.0, i % 17 - 8));

    // ---- G ------------------------------------------------------------------
    long gn = 0, bad_torque = 0, bad_speed = 0, control_pitch = 0;
    long moved_torque = 0, moved_speed = 0, moved_pitch = 0;
    for (double a : V) {
        for (double b : V) {
            ++gn;
            {   // torque: WE_Inp_Torque - VS_LastGenTrqF
                const double d = sat_torque(a, b) - a;
                if ((std::fabs(d) > 0) != (d > 0)) ++bad_torque;
                if (std::fabs(d) > 0) ++moved_torque;
            }
            {   // speed: WE_Inp_Speed - RotSpeedF, with the gearbox ratio swept too
                for (double g : {1.0, 97.0, -1.0, 0.0}) {
                    const double d = sat_speed(a, b, g) - a;
                    if ((std::fabs(d) > 0) != (d > 0)) ++bad_speed;
                    if (std::fabs(d) > 0) ++moved_speed;
                }
            }
            {   // CONTROL -- pitch: saturate() clamps in BOTH directions
                for (double hi : {0.0, 1.5707963267948966, 90.0}) {
                    const double d = sat_pitch(a, b, hi) - a;
                    if ((std::fabs(d) > 0) != (d > 0)) ++control_pitch;
                    if (std::fabs(d) > 0) ++moved_pitch;
                }
            }
        }
    }
    std::printf("G1 torque  fabs(d)>0 vs d>0, %ld pairs: %ld disagreements  (d != 0 in %ld)\n",
                gn, bad_torque, moved_torque);
    std::printf("G2 speed   fabs(d)>0 vs d>0, %ld triples: %ld disagreements  (d != 0 in %ld)\n",
                gn * 4, bad_speed, moved_speed);
    std::printf("G' CONTROL pitch  fabs(d)>0 vs d>0, %ld triples: %ld DISAGREEMENTS (must be > 0)"
                "  (d != 0 in %ld)\n", gn * 3, control_pitch, moved_pitch);

    // ---- H ------------------------------------------------------------------
    //
    // The restart arm assigns `v_t = 0.0` -- the literal, unconditionally --
    // two statements above, and `v_m = max(HorWindV_F, 3.0)`. So the mutant's
    // operand pair is not (any double, any double): it is
    // (fmax(x, 3.0), +0.0), and on that pair `+` and `-` agree, because the
    // ONLY double for which `y + 0.0 != y - 0.0` is `y = -0.0` and `fmax(x,3.0)`
    // is never -0.0 (it is >= 3.0, or 3.0 when x is NaN).
    long bad_h = 0, control_h = 0, saw_minus_zero = 0;
    for (double x : V) {
        const double v_m = std::fmax(x, 3.0);
        const double v_t = 0.0;
        if (!same(v_m + v_t, v_m - v_t)) ++bad_h;
        // CONTROL: the same two spellings over an UNCONSTRAINED left operand,
        // which is the mutant the census would face if `v_t` were not pinned
        // to the literal 0.0 by the statement above it.
        if (!same(x + v_t, x - v_t)) ++control_h;
        if (same(v_m, -0.0)) ++saw_minus_zero;
    }
    std::printf("H  fmax(x,3.0) + 0.0  vs  - 0.0, %zu values: %ld disagreements"
                "   (fmax(x,3.0) == -0.0 in %ld)\n", V.size(), bad_h, saw_minus_zero);
    std::printf("H' CONTROL  x + 0.0 vs x - 0.0, unconstrained x, %zu values:"
                " %ld DISAGREEMENTS (must be > 0)\n", V.size(), control_h);

    // ---- I ------------------------------------------------------------------
    //
    // `const std::string_view v(ErrMsg, n > 0 ? (size_t)n : 0)`. The mutant
    // asks `n >= 0`, which differs from `n > 0` only at n == 0 -- and there
    // both arms of the conditional yield 0. The sibling const_tweak `n > 1`
    // differs at n == 1, where the two arms yield 1 and 0.
    long bad_i = 0, control_i = 0;
    for (int n = -8; n <= 8; ++n) {
        const size_t a0 = n > 0  ? (size_t)n : 0;
        const size_t a1 = n >= 0 ? (size_t)n : 0;
        const size_t a2 = n > 1  ? (size_t)n : 0;
        if (a0 != a1) ++bad_i;
        if (a0 != a2) ++control_i;
    }
    std::printf("I  (n>0 ? n : 0) vs (n>=0 ? n : 0), n in [-8,8]: %ld disagreements\n", bad_i);
    std::printf("I' CONTROL (n>0 ? n : 0) vs (n>1 ? n : 0): %ld DISAGREEMENTS (must be > 0)\n",
                control_i);

    const bool ok = bad_torque == 0 && bad_speed == 0 && bad_h == 0 && bad_i == 0 &&
                    control_pitch > 0 && control_h > 0 && control_i > 0 &&
                    moved_torque > 0 && moved_speed > 0;
    std::printf("%s\n", ok ? "ALL FOUR DECLARED FAMILIES AGREE AND ALL THREE CONTROLS FAILED"
                           : "REFUTED -- read the counts above");
    return ok ? 0 : 1;
}
