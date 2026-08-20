// The TRANSLATION half of `print_record_probe.f90`: the same nine
// (VarName, LEN(Variable)) pairs through the SHIPPED translation's own
// `char_assign` and `print_default_warning`.
//
// TEXTUAL INCLUDE, same reason as `record_form_probe.cpp`. The three callee
// bridges are stubbed and abort if reached -- `main` never enters
// `ParseInput_Str_Opt` itself.
//
//   build: g++ -O0 -ffp-contract=off -std=c++17 -I<dir with vit_types.h> \
//              -DVIT_TRANSLATION='"<path to parseinput_str_opt.cpp>"' \
//              print_record_probe.cpp -o prp_str_got -lgfortran
//   run:   ./prp_str_got

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
    std::fprintf(stderr, "print_record_probe: findline_c reached\n");
    std::abort();
}
void getwords_c(char*, int, char*, int, int) {
    std::fprintf(stderr, "print_record_probe: getwords_c reached\n");
    std::abort();
}
void int2lstr_c(int, char*) {
    std::fprintf(stderr, "print_record_probe: int2lstr_c reached\n");
    std::abort();
}
}

#include VIT_TRANSLATION

int main() {
    struct Case { const char* vn; int L; };
    const Case cases[] = {
        {"PerfFileName", 1024},
        {"ZMQ_CommAddress", 256},
        {"X", 6},
        {"X", 5},
        {"X", 1},
        {"X", 0},
        {"", 7},
        {"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ", 7},
        {"DLL_ProcName", 1024},
    };

    for (const Case& c : cases) {
        const int lvn = static_cast<int>(std::strlen(c.vn));
        std::vector<char> VarName(static_cast<std::size_t>(lvn) + 1, ' ');
        std::memcpy(VarName.data(), c.vn, static_cast<std::size_t>(lvn));

        // `Variable` arrives blank-padded and holding whatever the caller had;
        // the sentinel makes an untouched byte visible, as in the READ probe.
        std::vector<char> Variable(static_cast<std::size_t>(c.L) + 1, '~');

        // Variable = 'unused'
        char_assign(Variable.data(), c.L, DefaultValue.data(),
                    static_cast<int>(DefaultValue.size()));
        print_default_warning(ftrim(VarName.data(), lvn),
                              ftrim(Variable.data(), c.L));
    }
    return 0;
}
