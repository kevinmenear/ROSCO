// The TRANSLATION half of `record_form_probe.f90`: the same eighteen records
// through the SHIPPED translation's own `list_read_reals`, out of the same
// 400-byte two-element storage `Words` is.
//
// TEXTUAL INCLUDE, for the reason `survivor_record_search.cpp` states:
// `parse_real` and `list_read_reals` live in an anonymous namespace, and a copy
// of them here would be a copy that goes stale silently. The three callee
// bridges are stubbed and abort if reached -- `main` never enters
// `ParseInput_Dbl_Opt` itself.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -I<test dir> \
//              -DVIT_TRANSLATION='"<path to parseinput_dbl_opt.cpp>"' \
//              record_form_probe.cpp -o rfp_got -lgfortran
//   run:   ./rfp_got

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "parseinput_dbl_opt_callees.h"

extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int) {
    std::fprintf(stderr, "record_form_probe: findline_c reached\n");
    std::abort();
}
void getwords_c(char*, int, char*, int, int) {
    std::fprintf(stderr, "record_form_probe: getwords_c reached\n");
    std::abort();
}
void int2lstr_c(int, char*) {
    std::fprintf(stderr, "record_form_probe: int2lstr_c reached\n");
    std::abort();
}
}

#include VIT_TRANSLATION

namespace {

struct Form {
    const char* label;
    std::string word;      // the WORD, not the record: GetWords left-justifies
    const char* neighbour; // Words(2), whose first byte is record byte 201
};

std::string digits200() { return std::string(199, '0') + "1"; }
std::string frac200() { return "1." + std::string(197, '0') + "5"; }

}  // namespace

int main() {
    const std::vector<Form> forms = {
        {"dig", digits200(), "7E"},   {"frac", frac200(), "7E"},
        {"expl", digits200(), "E7"},  {"expq", digits200(), "Q7"},
        {"sign", digits200(), "+7"},  {"neg", digits200(), "-7"},
        {"point", digits200(), ".7"}, {"fracE", frac200(), "E7"},
        {"rep", "3*7", "Aa"},         {"repbad", "0*7", "Aa"},
        {"expsign", "1.5E+2", "Aa"},  {"expneg", "1.5E-2", "Aa"},
        {"bare", "1.5+2", "Aa"},      {"baren", "1.5-2", "Aa"},
        {"dot", ".", "Aa"},           {"dotdot", "..", "Aa"},
        {"dote", ".e", "Aa"},
        {"nanpay", "nan(" + std::string(58, 'a') + ")", "Aa"},
        // A repeat count wide enough to start the value at byte 198, which is
        // where `match_word`'s `p + LEN(word) > len` bound finally has two
        // sides. Two mutants of that bound are declared equivalent on the
        // premise that `p` is 0 or 1, and the premise is the corpus's, not the
        // program's.
        {"repwide", std::string(196, '9') + "*nan", "Aa"},
        {"repwide2", std::string(195, '9') + "*nan", "Aa"},
        {"repbig", std::string(21, '9') + "*7", "Aa"},
        {"repceil0", "199999999*7", "Aa"},
        {"repceil", "200000000*7", "Aa"},
        {"repover", "200000001*7", "Aa"},
        {"repzeros", std::string(195, '0') + "3*nan", "Aa"},
        {"fracsig", std::string(185, '0') + "3*1.23456789012", "7E"},
        {"expsig", std::string(193, '0') + "3*1.5e2", "7E"},
        {"pointend", std::string(189, '0') + "200000000*-", ".7"},
        {"over", "1" + std::string(199, '0') + "1", "7E"},
        {"repone", "1*7", "Aa"},
    };
    for (const Form& f : forms) {
        // `CHARACTER(MaxParamLength) :: Words(2)` -- ONE 400-byte object. The
        // record is the first 200 bytes; byte 201 is Words(2)(1:1), which is
        // what the `p <= len` mutants read.
        std::vector<char> words(2 * MaxParamLength, ' ');
        std::memcpy(words.data(), f.word.data(),
                    std::min<std::size_t>(f.word.size(), MaxParamLength));
        std::memcpy(words.data() + MaxParamLength, f.neighbour,
                    std::strlen(f.neighbour));

        double v = -987.654;
        const int ios = list_read_reals(words.data(), MaxParamLength, &v, 1);

        std::uint64_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        std::printf("GOT %16s iostat=%6d bits=%016llX\n", f.label, ios,
                    static_cast<unsigned long long>(bits));
    }
    return 0;
}
