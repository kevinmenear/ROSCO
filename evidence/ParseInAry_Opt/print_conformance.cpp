// Replay the cases `print_conformance.f90` wrote through the SHIPPED
// translation's own `print_default_warning`, and print each record it produces.
//
// COPIED CHARACTER FOR CHARACTER from
// `evidence/ParseDbAry_Opt/print_conformance.cpp` (P4) with the unit's names
// changed, the callee header changed, and the item type changed from `double`
// to `std::int32_t`. Nothing else differs, because nothing else needs to: the
// two units' `print_default_warning` take the same shape and the binary format
// the Fortran side writes is the same file with one declaration changed.
//
// THE TEXTUAL INCLUDE IS THE LOAD-BEARING CHOICE, and it is unit #55's
// (`parser_conformance.cpp`). `print_default_warning` lives in an anonymous
// namespace, where it belongs. A COPY of it here would test a copy and go stale
// silently; including the .cpp puts `main` in the same translation unit, so the
// replay exercises the function that ships.
//
// The path is a macro so a MUTATED copy of the translation can be replayed
// without touching the tree: -DVIT_TRANSLATION='"/tmp/mutant.cpp"'.
//
// The three callee bridges are stubbed because the linker wants them. They are
// never called: `print_default_warning` reaches none of them, and `main` never
// enters `ParseInAry_Opt`. If one is ever reached the stub aborts rather than
// returning a plausible answer.
//
//   build:  g++ -O0 -I<test dir> -DVIT_TRANSLATION='"..."' print_conformance.cpp -o print_conformance
//   run:    ./print_conformance print_conformance.bin > got.txt

#include <ISO_Fortran_binding.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// The DECLARATIONS come from the generated header, not from a copy of the three
// signatures typed here: a stub whose parameter list has drifted from the
// header's is a stub that compiles and forwards the wrong arguments.
#include "parseinary_opt_callees.h"

extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int) {
    std::fprintf(stderr, "print_conformance: findline_c reached -- the replay "
                         "is supposed to enter the PRINT path only\n");
    std::abort();
}
void getwords_c(char*, int, char*, int, int) {
    std::fprintf(stderr, "print_conformance: getwords_c reached\n");
    std::abort();
}
void int2lstr_c(int, char*) {
    std::fprintf(stderr, "print_conformance: int2lstr_c reached\n");
    std::abort();
}
}

#include VIT_TRANSLATION

namespace {

// The reference wrote `INTEGER` and `INTEGER(4)` with an unformatted STREAM
// WRITE, so each is its own bytes with no record markers.
template <typename T>
bool rd(std::FILE* f, T& out) {
    return std::fread(&out, sizeof(T), 1, f) == 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: print_conformance <cases.bin>\n");
        return 2;
    }
    std::FILE* f = std::fopen(argv[1], "rb");
    if (!f) {
        std::fprintf(stderr, "cannot open %s\n", argv[1]);
        return 2;
    }
    int cases = 0;
    for (;;) {
        std::int32_t len = 0;
        if (!rd(f, len)) break;
        if (len < 0 || len > 4096) {
            std::fprintf(stderr, "case %d: implausible name length %d\n", cases, len);
            return 2;
        }
        std::string name(static_cast<std::size_t>(len), '\0');
        if (len > 0 && std::fread(name.data(), 1, static_cast<std::size_t>(len), f)
                       != static_cast<std::size_t>(len)) {
            std::fprintf(stderr, "case %d: short name\n", cases);
            return 2;
        }
        std::int32_t n = 0;
        if (!rd(f, n)) { std::fprintf(stderr, "case %d: no n\n", cases); return 2; }
        std::vector<std::int32_t> v(static_cast<std::size_t>(n < 0 ? 0 : n));
        for (int i = 0; i < n; ++i) {
            if (!rd(f, v[static_cast<std::size_t>(i)])) {
                std::fprintf(stderr, "case %d: short values\n", cases);
                return 2;
            }
        }
        print_default_warning(name, v.data(), n);
        ++cases;
    }
    std::fclose(f);
    std::fprintf(stderr, "print_conformance: replayed %d case(s)\n", cases);
    // A replay of nothing must not look like a replay that agreed (P10).
    return cases > 0 ? 0 : 3;
}
