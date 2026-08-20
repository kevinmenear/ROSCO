// The TRANSLATION half of `record_form_probe.f90`: the same thirty-four records
// through the SHIPPED translation's own `list_read_ints`, out of the same
// 400-byte two-element storage `Words` is.
//
// TEXTUAL INCLUDE, for the reason unit #55's `survivor_record_search.cpp`
// states: `parse_int` and `list_read_ints` live in an anonymous namespace, and
// a copy of them here would be a copy that goes stale silently. The three
// callee bridges are stubbed and abort if reached -- `main` never enters
// `ParseInput_Int_Opt` itself.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -I<dir with vit_types.h> \
//              -DVIT_TRANSLATION='"<path to parseinput_int_opt.cpp>"' \
//              record_form_probe.cpp -o rfp_got -lgfortran
//   run:   ./rfp_got

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int);
void getwords_c(char*, int, char*, int, int);
void int2lstr_c(int, char*);

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

std::string zeros(int n) { return std::string(static_cast<std::size_t>(n), '0'); }

}  // namespace

int main() {
    const std::vector<Form> forms = {
        {"plain", "42", "Aa"},
        {"signed", "+42", "Aa"},
        {"neg", "-42", "Aa"},
        {"imax", "2147483647", "Aa"},
        {"imaxp1", "2147483648", "Aa"},
        {"imin", "-2147483648", "Aa"},
        {"iminm1", "-2147483649", "Aa"},
        {"zeros", "0000000000000000042", "Aa"},
        {"zerosbig", "002147483648", "Aa"},
        {"realpt", "1.5", "Aa"},
        {"realexp", "1e2", "Aa"},
        {"trailing", "5abc", "Aa"},
        {"signonly", "-", "Aa"},
        {"dot", ".", "Aa"},
        {"word", "nan", "Aa"},
        {"slash", "/", "Aa"},
        {"slashaft", "7/9", "Aa"},
        {"slashpre", "/7", "Aa"},
        {"rep", "3*7", "Aa"},
        {"repone", "1*7", "Aa"},
        {"repzero", "0*7", "Aa"},
        {"repnull", "3*", "Aa"},
        {"repceil0", "199999999*7", "Aa"},
        {"repceil", "200000000*7", "Aa"},
        {"repover", "200000001*7", "Aa"},
        {"repwide", std::string(21, '9') + "*7", "Aa"},
        {"tailmax", zeros(190) + "2147483647", "Aa"},
        {"tailover", zeros(190) + "2147483648", "Aa"},
        {"tailrep", zeros(197) + "3*7", "Aa"},
        {"tailstar", zeros(199) + "*", "Aa"},
        // THE BYTE PAST A FULL-WIDTH RECORD IS AN INPUT -- unit #56's finding.
        // The same 200-byte lead with four different `Words(2)` first
        // characters, each one what a boundary guard TESTS.
        {"nbdigit", zeros(190) + "2147483647", "7X"},
        {"nbsign", zeros(190) + "2147483647", "-7"},
        {"nbstar", zeros(197) + "3*7", "*7"},
        {"nbslash", zeros(190) + "2147483647", "/7"},
        // The three forms the mutation search named, priced before planting.
        {"repnull1", "1*", "Aa"},
        {"repnull3", "3*", "Aa"},
        {"junkend", "7x", "Aa"},
        // The three forms the second round of survivors named.
        {"repzslash", "0*/", "Aa"},
        {"repstar", zeros(189) + "3*123456789", "*7"},
        {"over1", zeros(190) + "21474836470", "Aa"},
        // The correction to `repstar`: a record with NO star, so the repeat
        // lookahead's digit scan reaches the record's last byte, paired with a
        // star NEIGHBOUR.
        {"digstar", zeros(199) + "1", "*7"},
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

        std::int32_t v = -987654;
        const int ios = list_read_ints(words.data(), MaxParamLength, &v, 1);

        std::printf("GOT %16s iostat=%6d value=%12d\n", f.label, ios,
                    static_cast<int>(v));
    }
    return 0;
}
