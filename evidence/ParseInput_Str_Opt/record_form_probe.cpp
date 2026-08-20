// The TRANSLATION half of `record_form_probe.f90`: the same sixteen record
// forms, crossed with the same nine item lengths, through the SHIPPED
// translation's own `read_a_edit`, out of the same 400-byte two-element
// storage `Words` is.
//
// TEXTUAL INCLUDE, for the reason unit #55's `survivor_record_search.cpp`
// states: `char_assign` and `read_a_edit` live in an anonymous namespace, and a
// copy of them here would be a copy that goes stale silently. The three callee
// bridges are stubbed and abort if reached -- `main` never enters
// `ParseInput_Str_Opt` itself.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -I<dir with vit_types.h> \
//              -DVIT_TRANSLATION='"<path to parseinput_str_opt.cpp>"' \
//              record_form_probe.cpp -o rfp_str_got -lgfortran
//   run:   ./rfp_str_got

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

constexpr int kRec = 200;

struct Form {
    const char* tag;
    std::string word;       // the WORD; the element is blank-padded to 200
    std::string neighbour;  // Words(2), whose first byte is object byte 201
};

std::string rep(char c, int n) { return std::string(static_cast<std::size_t>(n), c); }

// The same escape the Fortran half writes: printable ASCII raw, everything
// else as \xNN with UPPERCASE hex, and '\' and '"' escaped too.
std::string esc(const char* s, int n) {
    std::string out;
    for (int i = 0; i < n; ++i) {
        const unsigned char v = static_cast<unsigned char>(s[i]);
        if (v >= 32 && v < 127 && v != '\\' && v != '"') {
            out += static_cast<char>(v);
        } else {
            char b[5];
            std::snprintf(b, sizeof b, "\\x%02X", v);
            out += b;
        }
    }
    return out;
}

long long fnv31(const char* s, int n) {
    long long h = 0;
    for (int i = 0; i < n; ++i) {
        h = (h * 31 + static_cast<unsigned char>(s[i])) % 2147483647LL;
    }
    return h;
}

int len_trim(const char* s, int n) {
    int i = n;
    while (i > 0 && s[i - 1] == ' ') {
        --i;
    }
    return i;
}

}  // namespace

int main() {
    const int lens[] = {1, 5, 6, 7, 199, 200, 201, 256, 1024};

    const std::vector<Form> forms = {
        {"plain", "unused", "Aa"},
        {"path", "Cp_Ct_Cq.txt", "Aa"},
        {"digits", "12345", "Aa"},
        {"slash", "a/b", "Aa"},
        {"star", "3*7", "Aa"},
        {"nul", std::string("ab") + '\0' + "cd", "Aa"},
        {"cr", std::string("ab") + '\r' + "cd", "Aa"},
        {"lf", std::string("ab") + '\n' + "cd", "Aa"},
        {"full", rep('Z', 200), "Aa"},
        {"full199", rep('Y', 199), "Aa"},
        {"fullcr", rep('\r', 200), "Aa"},
        {"fullnul", rep('\0', 200), "Aa"},
        {"blankrec", "", "Aa"},
        {"blankrecN", "", rep('N', 200)},
        {"fn:tab", std::string("a") + '\t' + "b", "Aa"},
        {"fn:lead", "   x", "Aa"},
    };

    for (const Form& f : forms) {
        // The reference's own storage: ONE 400-byte object, blank-filled, with
        // the word left-justified into element 1 and the neighbour into
        // element 2.
        std::vector<char> Words(2 * kRec, ' ');
        std::memcpy(Words.data(), f.word.data(),
                    std::min<std::size_t>(f.word.size(), kRec));
        std::memcpy(Words.data() + kRec, f.neighbour.data(),
                    std::min<std::size_t>(f.neighbour.size(), kRec));

        for (const int L : lens) {
            std::vector<char> item(static_cast<std::size_t>(L), '~');
            const int e = read_a_edit(Words.data(), kRec, item.data(), L);
            const int show = L < 24 ? L : 24;
            std::printf("R %s %d iostat= %d lentrim= %d h= %lld p=\"%s\"\n",
                        f.tag, L, e, len_trim(item.data(), L),
                        fnv31(item.data(), L), esc(item.data(), show).c_str());
        }
    }
    return 0;
}
