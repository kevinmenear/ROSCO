// C++ SIDE of the three-question semantics control. See semantics_probe.f90.
//
// The helpers are not re-implemented here: this file INCLUDES the translation
// itself, so the code under test is the code that ships (P4). `WriteRestartFile`
// is never called; only `wr_log`, `i0d0` and `fixed_width_trim`/`ftrim` are.
#include <cstdio>
#include <cstring>
#include <string>

#include "writerestartfile.cpp"

int main() {
    // Q1 -- the raw bytes, through the shipped `wr_log`, at the same widths.
    Stream u;
    u.f = std::fopen("semantics_probe.c.bin", "wb");
    wr_log(u, 1);
    wr_log(u, 0);
    wr_log(u, 1);
    std::fclose(u.f);

    std::FILE* o = std::fopen("semantics_probe.c.txt", "w");

    // Q2 -- i0d0 across the sign and the zero.
    const long vals[7] = {0, 1, -1, 7, 40, 21560, -21560};
    for (int i = 0; i < 7; ++i) {
        const std::string s = i0d0(vals[i]);
        std::fprintf(o, "I0.0(%ld) = [%s] len=%d\n", vals[i], s.c_str(),
                     static_cast<int>(s.size()));
    }

    // Q3 -- the CHARACTER(128) truncation, through fixed_width_trim.
    for (int i = 116; i <= 124; ++i) {
        const std::string root(static_cast<size_t>(i), 'R');
        const std::string InFile =
            fixed_width_trim(root + i0d0(7) + ".RO.chkp", INFILE_LEN);
        std::fprintf(o, "root=%d lenTrim=%d [%s]\n", i,
                     static_cast<int>(InFile.size()), InFile.c_str());
    }
    std::fclose(o);
    return 0;
}
