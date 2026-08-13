// `n > 0 ? static_cast<size_t>(n) : 0` against `n >= 0 ? ... : 0`, EXHAUSTIVE.
//
// Mutant bf2ce388 (compare_op, unwrap.cpp:135). The two spellings can differ
// only where the predicate does, at `n == 0` -- and there the tight form yields
// the literal `0` and the loose form `static_cast<size_t>(0)`, the same length.
// This sweeps all 4,294,967,296 values a 32-bit int admits rather than arguing
// it, because a claim proved over the whole TYPE is stronger than one proved
// over a corpus, and it costs a minute.
//
// Same site and same proof as sigma's 532e4d37 and interp1d's b61327db. RE-RUN
// here rather than cited: a measurement taken on another unit's source is a
// measurement of that unit.
#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <initializer_list>

static inline size_t tight(int n) { return n >  0 ? static_cast<size_t>(n) : 0; }
static inline size_t loose(int n) { return n >= 0 ? static_cast<size_t>(n) : 0; }

int main() {
    long long differ = 0;
    for (int64_t v = INT32_MIN; v <= INT32_MAX; ++v) {
        int n = static_cast<int>(v);
        if (tight(n) != loose(n)) { if (differ < 4) std::printf("  DIFFER at n = %d\n", n); differ++; }
    }
    std::printf("values of a 32-bit int swept        : 4294967296\n");
    std::printf("values at which the LENGTH differs  : %lld\n", differ);
    for (int n : {INT32_MIN, -1, 0, 1, INT32_MAX})
        std::printf("  witness n = %-12d tight = %zu  loose = %zu\n", n, tight(n), loose(n));
    return differ == 0 ? 0 : 1;
}
