// PROBE — INPUT to a measurement, not the measurement.
//
// One survivor of `mutation/sigma.json`, proved EXHAUSTIVE over the whole type
// rather than over the corpus:
//
//   532e4d37  compare_op '>' -> '>=' at
//             `const std::string_view v(ErrVar->ErrMsg, n > 0 ? (size_t)n : 0);`
//
// The two spellings can differ only where the predicate does, `n == 0`, and
// there the tight form yields the literal `0` and the loose form yields
// `static_cast<size_t>(0)` — the same length. That is an argument; this is the
// measurement. Every one of the 4,294,967,296 values a 32-bit `int` admits is
// put through both spellings and the resulting VIEW LENGTHS are compared, so
// the claim covers every input the C type has, not merely the ones this
// campaign's generator draws.
//
//   g++ -O2 -o equivalence_probe equivalence_probe.cpp && ./equivalence_probe
//
// This is the same site interp1d (unit #23) declared as `b61327db` and the same
// proof; it is re-run here rather than cited, because a measurement taken on
// another unit's source is a measurement of that unit.

#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <initializer_list>

int main() {
    long long differ = 0;
    long long checked = 0;
    int first_differ = 0;
    bool have_first = false;

    // int64 counter so the loop terminates: an `int` one wraps at INT_MAX.
    for (int64_t k = INT32_MIN; k <= INT32_MAX; ++k) {
        const int n = static_cast<int>(k);
        const size_t tight = n > 0 ? static_cast<size_t>(n) : 0;
        const size_t loose = n >= 0 ? static_cast<size_t>(n) : 0;
        ++checked;
        if (tight != loose) {
            ++differ;
            if (!have_first) { first_differ = n; have_first = true; }
        }
    }

    std::printf("532e4d37  `n > 0 ? (size_t)n : 0`  vs  `n >= 0 ? (size_t)n : 0`\n");
    std::printf("  int values swept                 %lld\n", checked);
    std::printf("  values at which the LENGTH differs %lld\n", differ);
    if (have_first) std::printf("  first differing n                %d\n", first_differ);
    // The three interesting witnesses, printed so the run says what it did.
    for (int n : {-2147483647 - 1, -1, 0, 1, 2147483647}) {
        std::printf("  n = %12d   tight = %zu   loose = %zu\n", n,
                    n > 0 ? static_cast<size_t>(n) : 0,
                    n >= 0 ? static_cast<size_t>(n) : 0);
    }
    return differ == 0 ? 0 : 1;
}
