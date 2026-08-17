// EXHAUSTIVE PROBE OVER THE COMPOSED CHARACTER DOMAIN OF `AeroDynTorque`.
//
// It answers ONE question, for the four mutants unit #48 left standing:
//
//     is there ANY (entry ErrMsg, entry n_ErrMsg, staging capacity, callee
//     result) at which the mutated `AeroDynTorque` tail and the original tail
//     leave DIFFERENT bytes in `ErrVar%ErrMsg`/`n_ErrMsg`?
//
// This is not a corpus. It does not run the harness, it does not call interp2d,
// and it does not need either: the three `const_tweak` survivors live entirely
// in the two CHARACTER helpers, whose only inputs are the buffer, its stated
// length and its stated capacity. Sweeping those three EXHAUSTIVELY over the
// range in which the two helpers can differ at all is a stronger statement than
// any sample of 1131 cases, and it is the statement the mutation census could
// not make: the census measured that the corpus does not contain the killing
// input; this measures whether one EXISTS.
//
// ------------------------------------------------------------------ the model
//
// The tail of `AeroDynTorque` is
//
//     if (ErrVar->aviFAIL < 0)
//         assign_errmsg(ErrVar, "AeroDynTorque:" + errmsg_trim(ErrVar));
//
// and it runs AFTER the one call, `interp2d`. So the state `errmsg_trim` reads
// is not the unit's input -- it is whatever `interp2d` left. What interp2d can
// leave is read off the reference (clean `rosco/controller/src/Functions.f90`)
// and it has exactly two shapes:
//
//   (1) THE CALLEE WROTE. Every exit of `interp2d` on which `aviFAIL < 0` holds
//       passes through exactly one reallocating assignment of the shape
//       `RoutineName//':'//TRIM(ErrVar%ErrMsg)`: the five early RETURNs go
//       through `interp1d`'s (`Functions.f90:294..296` in the clean file), and
//       the bilinear fall-through goes through `interp2d`'s own. Both
//       RoutineNames are eight characters, so the result is a string of length
//       L = 9 + LEN(TRIM(entry)) >= 9 whose LAST CHARACTER IS NOT BLANK (`TRIM`
//       removed the blanks; when TRIM is empty the last character is the ':').
//       The staged write-back is refused when L exceeds the capacity.
//
//   (2) THE CALLEE'S WRITE WAS REFUSED, and the buffer still holds the unit's
//       own input. Modelled below as L = 0.
//
// L is therefore swept over {0} U [9, 40] rather than being computed from the
// entry string: a probe that derived L from the entry would be asserting the
// callee's arithmetic instead of covering it. L = 0 is the COUNTERFACTUAL
// CALLEE -- a callee that leaves ErrMsg alone. No such path exists in interp2d;
// it is here as the probe's own red test (see CONTROLS below).
//
// ------------------------------------------------------------- the four sites
//
//     assign_errmsg    if (static_cast<int>(s.size()) >  cap)      88466711: >=
//     errmsg_trim      view(ErrMsg, n >  0 ? (size_t)n : 0)        11c1e326: n > 1
//     errmsg_trim      view(ErrMsg, n > 0 ? (size_t)n : 0 )        06f7d2c8: : 1
//     errmsg_trim      substr(0, find_last_not_of(' ') + 1)        26021804: + 2
//
// Rendered from ONE template with four integer knobs rather than five copied
// bodies, so that "the variants differ in exactly one token" is true by
// construction and not by a reader's diff.
//
// ------------------------------------------------------------------- CONTROLS
//
// A probe that reports zero differences has to be able to report a difference.
// Two controls, both printed:
//
//   POSITIVE   88466711 -- the capacity-boundary mutant. It MUST be found
//              different, and the configurations that find it are the input the
//              corpus was missing. If this row reads 0 the probe is broken.
//   L = 0      the counterfactual callee. The three trim mutants MUST be found
//              different there, which is what identifies the callee's write as
//              the load-bearing fact rather than the sweep being too narrow.
//
// Build and run (container toolchain, campaign flags):
//   docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
//     g++ -O2 -ffp-contract=off -std=c++17 \
//     evidence/AeroDynTorque/aerodyntorque.trim-composition-probe.cpp \
//     -o /tmp/trimcomp && /tmp/trimcomp"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr int BUFCAP = 256;   // the real allocation; `cap` is the STATED capacity

struct View {
    char* ErrMsg;
    int   n_ErrMsg;
    int   n_ErrMsg_cap;
};

// ---- the two helpers, verbatim from translations/Functions/aerodyntorque.cpp
// except for the four knobs. CAP_GE, TRIM_GUARD, TRIM_ELSE and TRIM_PLUS are 0
// / 0 / 0 / 1 in the shipped translation.
template <int CAP_GE, int TRIM_GUARD, int TRIM_ELSE, int TRIM_PLUS>
void assign_errmsg(View* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) return;
    const bool refuse = CAP_GE ? (static_cast<int>(s.size()) >= ErrVar->n_ErrMsg_cap)
                               : (static_cast<int>(s.size()) >  ErrVar->n_ErrMsg_cap);
    if (refuse) return;
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    ErrVar->n_ErrMsg = static_cast<int>(s.size());
}

template <int CAP_GE, int TRIM_GUARD, int TRIM_ELSE, int TRIM_PLUS>
std::string errmsg_trim(const View* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg,
                             n > TRIM_GUARD ? static_cast<size_t>(n)
                                            : static_cast<size_t>(TRIM_ELSE));
    return std::string(v.substr(0, v.find_last_not_of(' ') + TRIM_PLUS));
}

// The tail of AeroDynTorque, reached only when aviFAIL < 0.
template <int CAP_GE, int TRIM_GUARD, int TRIM_ELSE, int TRIM_PLUS>
void tail(View* ErrVar) {
    assign_errmsg<CAP_GE, TRIM_GUARD, TRIM_ELSE, TRIM_PLUS>(
        ErrVar, std::string("AeroDynTorque:")
                    + errmsg_trim<CAP_GE, TRIM_GUARD, TRIM_ELSE, TRIM_PLUS>(ErrVar));
}

// ---- the input space ------------------------------------------------------
//
// SHAPES of the entry ErrMsg. Every one of them is a value a `CHARACTER(:),
// ALLOCATABLE` field can hold; none is narrowed away.
const char* const SHAPES[] = {
    "",                       // (n is applied on top; content beyond n is fill)
    "ABCDEFGHIJKLMNOPQRSTUVWX",   // no blank anywhere
    "AB CD EF GH IJ KL MN OPQ",   // embedded blanks, non-blank tail
    "ABCDEFGHIJ              ",   // trailing blanks
    "                        ",   // all blanks
    " A                      ",   // one non-blank, deep trailing blank run
};
constexpr int N_SHAPES = sizeof(SHAPES) / sizeof(SHAPES[0]);

// The bytes of the buffer that lie OUTSIDE [0, n) -- what a mutant that reads
// past the stated length would see. Both fills are swept because 06f7d2c8 reads
// ErrMsg[0] at n <= 0 and the answer depends on this byte.
const char FILLS[] = {'\0', ' ', 'Z'};
constexpr int N_FILLS = sizeof(FILLS) / sizeof(FILLS[0]);

struct Config { int shape, fill, n, cap, L; };

void setup(char* buf, const Config& c) {
    std::memset(buf, FILLS[c.fill], BUFCAP);
    const int keep = c.n > 0 ? c.n : 0;
    const int have = static_cast<int>(std::strlen(SHAPES[c.shape]));
    for (int i = 0; i < keep && i < BUFCAP; ++i)
        buf[i] = i < have ? SHAPES[c.shape][i] : FILLS[c.fill];
}

// The callee, as read off the reference. L = 0 is the counterfactual.
void callee(View* v, int L) {
    if (L == 0) return;                    // the callee that does not write
    if (L > v->n_ErrMsg_cap) return;       // the staged write-back, refused
    // "interp1d:" then non-blank filler, so the last character is not blank --
    // which is what `RoutineName//':'//TRIM(...)` guarantees.
    static const char P[] = "interp1d:";
    for (int i = 0; i < L; ++i) v->ErrMsg[i] = i < 9 ? P[i] : 'q';
    v->n_ErrMsg = L;
}

struct Variant {
    const char* id;
    const char* site;
    void (*run)(View*);
};

void run_orig(View* v) { tail<0, 0, 0, 1>(v); }
void run_8846(View* v) { tail<1, 0, 0, 1>(v); }   // compare_op  '>'  -> '>='
void run_11c1(View* v) { tail<0, 1, 0, 1>(v); }   // const_tweak n > 0 -> n > 1
void run_06f7(View* v) { tail<0, 0, 1, 1>(v); }   // const_tweak  : 0  -> : 1
void run_2602(View* v) { tail<0, 0, 0, 2>(v); }   // const_tweak  + 1  -> + 2

}  // namespace

int main() {
    const Variant vars[] = {
        {"88466711", "assign_errmsg  '>' -> '>='",          run_8846},
        {"11c1e326", "errmsg_trim    'n > 0' -> 'n > 1'",   run_11c1},
        {"06f7d2c8", "errmsg_trim    ': 0' -> ': 1'",       run_06f7},
        {"26021804", "errmsg_trim    '+ 1' -> '+ 2'",       run_2602},
    };

    std::vector<Config> configs;
    for (int s = 0; s < N_SHAPES; ++s)
        for (int f = 0; f < N_FILLS; ++f)
            for (int n = -2; n <= 24; ++n)
                for (int cap = 0; cap <= 80; ++cap) {
                    configs.push_back({s, f, n, cap, 0});          // counterfactual
                    for (int L = 9; L <= 40; ++L)
                        configs.push_back({s, f, n, cap, L});      // the real callee
                }

    std::printf("TRIMCOMP configs=%zu  shapes=%d fills=%d  n=[-2,24] cap=[0,80] "
                "L={0}U[9,40]\n\n", configs.size(), N_SHAPES, N_FILLS);
    std::printf("%-10s %-34s %10s %10s %10s\n",
                "mutant", "site", "diff L>=9", "diff L=0", "first L>=9");
    std::printf("%-10s %-34s %10s %10s %10s\n",
                "----------", "----------------------------------",
                "----------", "----------", "----------");

    int rc = 0;
    for (const Variant& var : vars) {
        long diff_real = 0, diff_cf = 0;
        Config first{}; bool have_first = false;
        char a[BUFCAP], b[BUFCAP];
        for (const Config& c : configs) {
            View va{a, c.n, c.cap}, vb{b, c.n, c.cap};
            setup(a, c); setup(b, c);
            callee(&va, c.L); callee(&vb, c.L);
            run_orig(&va);
            var.run(&vb);
            const int la = va.n_ErrMsg > 0 ? va.n_ErrMsg : 0;
            const int lb = vb.n_ErrMsg > 0 ? vb.n_ErrMsg : 0;
            const bool differ = (va.n_ErrMsg != vb.n_ErrMsg)
                                || (la != lb) || std::memcmp(a, b, la) != 0;
            if (!differ) continue;
            if (c.L == 0) { ++diff_cf; continue; }
            ++diff_real;
            if (!have_first) { first = c; have_first = true; }
        }
        std::printf("%-10s %-34s %10ld %10ld   ",
                    var.id, var.site, diff_real, diff_cf);
        if (have_first)
            std::printf("shape=%d fill=%d n=%d cap=%d L=%d\n",
                        first.shape, first.fill, first.n, first.cap, first.L);
        else
            std::printf("(none)\n");
    }

    // The two controls, asserted rather than left to a reader.
    {
        long p = 0;
        char a[BUFCAP], b[BUFCAP];
        for (const Config& c : configs) {
            if (c.L == 0) continue;
            View va{a, c.n, c.cap}, vb{b, c.n, c.cap};
            setup(a, c); setup(b, c);
            callee(&va, c.L); callee(&vb, c.L);
            run_orig(&va); run_8846(&vb);
            const int la = va.n_ErrMsg > 0 ? va.n_ErrMsg : 0;
            if (va.n_ErrMsg != vb.n_ErrMsg || std::memcmp(a, b, la) != 0) ++p;
        }
        std::printf("\nCONTROL positive  88466711 differs at %ld configuration(s) "
                    "with the real callee -- %s\n", p, p ? "the probe can go red" :
                    "PROBE IS BROKEN");
        if (!p) rc = 1;
    }
    {
        long q = 0;
        char a[BUFCAP], b[BUFCAP];
        for (const Config& c : configs) {
            if (c.L != 0) continue;
            for (const Variant& var : vars) {
                if (std::strcmp(var.id, "88466711") == 0) continue;
                View va{a, c.n, c.cap}, vb{b, c.n, c.cap};
                setup(a, c); setup(b, c);
                run_orig(&va); var.run(&vb);
                const int la = va.n_ErrMsg > 0 ? va.n_ErrMsg : 0;
                if (va.n_ErrMsg != vb.n_ErrMsg || std::memcmp(a, b, la) != 0) ++q;
            }
        }
        std::printf("CONTROL counterfactual  the three trim mutants differ at %ld "
                    "configuration(s) when the callee does NOT write -- %s\n", q,
                    q ? "the callee's write is what makes them equivalent" :
                        "PROBE IS BROKEN");
        if (!q) rc = 1;
    }
    return rc;
}
