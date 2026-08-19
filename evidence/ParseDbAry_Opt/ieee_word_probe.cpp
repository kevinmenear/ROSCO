// The translation half of `ieee_word_probe.f90`: the same ten IEEE words
// through the SHIPPED translation's own `list_read_reals`, printing the 64 bits
// each one leaves in the array element and the IOSTAT it models.
//
// TEXTUAL INCLUDE, for the reason `print_conformance.cpp` states: `parse_real`
// and `list_read_reals` live in an anonymous namespace, and a copy of them here
// would be a copy that goes stale silently. The three callee bridges are
// stubbed and abort if reached -- `main` never enters `ParseDbAry_Opt`.
//
//   build: g++ -O0 -ffp-contract=off -I<test dir> \
//              -DVIT_TRANSLATION='"<path to parsedbary_opt.cpp>"' \
//              ieee_word_probe.cpp -o ieee_word_probe
//   run:   ./ieee_word_probe

#include <ISO_Fortran_binding.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "parsedbary_opt_callees.h"

extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int) {
    std::fprintf(stderr, "ieee_word_probe: findline_c reached\n");
    std::abort();
}
void getwords_c(char*, int, char*, int, int) {
    std::fprintf(stderr, "ieee_word_probe: getwords_c reached\n");
    std::abort();
}
void int2lstr_c(int, char*) {
    std::fprintf(stderr, "ieee_word_probe: int2lstr_c reached\n");
    std::abort();
}
}

#include VIT_TRANSLATION

int main() {
    static const char* words[] = {"inf",  "-inf", "INF", "Infinity", "-Infinity",
                                  "nan",  "nAn",  "NAN", "-nan",     "NaN"};
    for (const char* w : words) {
        // The record the corpus will produce: the word in a 2048-byte
        // blank-padded internal-file record, exactly as `Line` arrives.
        std::vector<char> rec(static_cast<std::size_t>(MaxLineLength), ' ');
        std::memcpy(rec.data(), w, std::strlen(w));

        double v = -987.654;
        const int ios = list_read_reals(rec.data(), MaxLineLength, &v, 1);

        std::uint64_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        std::printf("GOT %-32s iostat=%d bits=%016llX\n", w, ios,
                    static_cast<unsigned long long>(bits));
    }
    return 0;
}
