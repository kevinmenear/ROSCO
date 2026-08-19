// The translation half of `record_tail_probe.f90`: the same four records
// through the SHIPPED translation's own `list_read_reals`.
//
// Textual include, for the reason `print_conformance.cpp` states. The three
// callee bridges are stubbed and abort if reached.
//
//   build: g++ -O0 -ffp-contract=off -I<test dir> \
//              -DVIT_TRANSLATION='"<path to parsedbary_opt.cpp>"' \
//              record_tail_probe.cpp -o record_tail_probe -lgfortran

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
    std::fprintf(stderr, "record_tail_probe: findline_c reached\n");
    std::abort();
}
void getwords_c(char*, int, char*, int, int) {
    std::fprintf(stderr, "record_tail_probe: getwords_c reached\n");
    std::abort();
}
void int2lstr_c(int, char*) {
    std::fprintf(stderr, "record_tail_probe: int2lstr_c reached\n");
    std::abort();
}
}

#include VIT_TRANSLATION

namespace {

std::string rep(char c, int n) { return std::string(static_cast<std::size_t>(n), c); }

}  // namespace

int main() {
    const int L = MaxLineLength;
    const std::pair<const char*, std::string> recs[] = {
        {"tail:digits", rep('0', L - 1) + "1"},
        {"tail:frac", "1." + rep('0', L - 3) + "5"},
        {"tail:inf", rep('0', L - 4) + ",inf"},
        {"tail:nan", rep('0', L - 4) + ",nAn"},
    };
    for (const auto& r : recs) {
        if (static_cast<int>(r.second.size()) != L) {
            std::fprintf(stderr, "record_tail_probe: %s is %d characters, not %d "
                                 "-- the record would not end at the boundary\n",
                         r.first, static_cast<int>(r.second.size()), L);
            return 2;
        }
        std::vector<char> rec(r.second.begin(), r.second.end());
        double v[2] = {-987.654, -987.654};
        const int ios = list_read_reals(rec.data(), L, v, 2);
        std::uint64_t b1 = 0, b2 = 0;
        std::memcpy(&b1, &v[0], sizeof b1);
        std::memcpy(&b2, &v[1], sizeof b2);
        std::printf("GOT %-16s iostat=%d b1=%016llX b2=%016llX\n", r.first, ios,
                    static_cast<unsigned long long>(b1),
                    static_cast<unsigned long long>(b2));
    }
    return 0;
}
