// Replay every record of `parser_conformance.txt` through the SHIPPED
// translation's own `list_read_ints`, and compare the iostat and every element
// against what gfortran's runtime returned for the same record.
//
//   g++ -std=c++17 -O2 -I<vit headers> parser_conformance.cpp -o pc_replay
//   ./pc_replay parser_conformance.txt
//
// WHY IT INCLUDES THE .cpp RATHER THAN LINKING IT. `list_read_ints` lives in an
// anonymous namespace in the translation, which is where it belongs -- it is
// not part of the unit's interface and nothing outside the file may call it. A
// textual include puts this `main` in the same translation unit, so the replay
// tests the SHIPPED FUNCTION and not a copy of it. If the translation changes,
// this changes with it; a copy would go stale silently, which is the failure
// mode a positive control exists to avoid.
//
// The three callee bridges are stubbed below because `ParseInAry_Opt` itself
// references them and the linker wants definitions. They are never called: this
// program's only entry into the translation is the parser.
//
// EXIT 0 = every record agrees. Non-zero = the count that did not, and each is
// printed with the record, gfortran's answer and the translation's.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// The view struct and the CFI descriptor the translation includes.
#include <ISO_Fortran_binding.h>
#include "vit_types.h"

extern "C" {
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int) {}
void getwords_c(char*, int, char*, int, int) {}
void int2lstr_c(int, char*) {}
}

#include "parseinary_opt.cpp"   // NOLINT -- see the note above

namespace {

constexpr int kRecordLen = 2048;   // MaxLineLength
constexpr int32_t kSentinel = -987654;

struct Case {
    std::string text;              // the SUPPLIED text, before blank padding
    int n = 0;
    int iostat = 0;
    std::vector<int32_t> values;   // gfortran's answer, sentinels included
};

std::string printable(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '\t') { out += "<TAB>"; }
        else if (c == '\n') { out += "<LF>"; }
        else if (c == '\r') { out += "<CR>"; }
        else if (static_cast<unsigned char>(c) < 32) { out += '?'; }
        else { out += c; }
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "parser_conformance.txt";
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", path);
        return 2;
    }

    std::vector<Case> cases;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("CASE", 0) != 0) {
            continue;
        }
        std::istringstream is(line);
        std::string tag;
        Case c;
        is >> tag >> c.n >> c.iostat;
        c.values.resize(static_cast<std::size_t>(c.n));
        for (int i = 0; i < c.n; ++i) {
            is >> c.values[static_cast<std::size_t>(i)];
        }
        std::string len_tag;
        int m = 0;
        is >> len_tag >> m;
        if (len_tag != "LEN") {
            std::fprintf(stderr, "malformed line: %s\n", line.c_str());
            return 2;
        }
        for (int i = 0; i < m; ++i) {
            int code = 0;
            is >> code;
            c.text += static_cast<char>(code);
        }
        cases.push_back(std::move(c));
    }

    if (cases.empty()) {
        // P10: a pass built from an empty set is not a pass.
        std::fprintf(stderr, "no CASE records in %s -- refusing to report a pass\n", path);
        return 2;
    }

    int bad = 0;
    for (std::size_t k = 0; k < cases.size(); ++k) {
        const Case& c = cases[k];
        std::vector<char> rec(static_cast<std::size_t>(kRecordLen), ' ');
        std::memcpy(rec.data(), c.text.data(),
                    std::min<std::size_t>(c.text.size(), kRecordLen));

        std::vector<int32_t> got(static_cast<std::size_t>(c.n), kSentinel);
        const int s = list_read_ints(rec.data(), kRecordLen, got.data(), c.n);

        if (s != c.iostat || got != c.values) {
            ++bad;
            std::printf("MISMATCH  case %zu  n=%d  record \"%s\"\n",
                        k, c.n, printable(c.text).c_str());
            std::printf("   gfortran  iostat=%d  ", c.iostat);
            for (int32_t v : c.values) { std::printf(" %d", v); }
            std::printf("\n   c++       iostat=%d  ", s);
            for (int32_t v : got) { std::printf(" %d", v); }
            std::printf("\n");
        }
    }

    std::printf("parser_conformance: %zu records, %d mismatched\n", cases.size(), bad);
    return bad == 0 ? 0 : 1;
}
