#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "parseinput_dbl_opt_callees.h"
extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int) { std::abort(); }
void getwords_c(char*, int, char*, int, int) { std::abort(); }
void int2lstr_c(int, char*) { std::abort(); }
}
#include VIT_TRANSLATION
int main() {
    for (int L = 0; L <= 196; ++L) {
        std::string w = "nan(" + std::string(L, 'a') + ")";
        std::vector<char> words(2 * MaxParamLength, ' ');
        std::memcpy(words.data(), w.data(), std::min<std::size_t>(w.size(), MaxParamLength));
        std::memcpy(words.data() + MaxParamLength, "Aa", 2);
        double v = -987.654;
        const int ios = list_read_reals(words.data(), MaxParamLength, &v, 1);
        std::uint64_t bits = 0; std::memcpy(&bits, &v, sizeof bits);
        std::printf("GOT %5d iostat=%6d bits=%016llX\n", L, ios,
                    static_cast<unsigned long long>(bits));
    }
    return 0;
}
