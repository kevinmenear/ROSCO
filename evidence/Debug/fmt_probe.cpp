// C++ side of the format-fidelity control. See fmt_probe.f90.
//
// The helpers are not re-implemented here: this file INCLUDES the translation
// itself, so the code under test is the code that ships (P4). `Debug` is never
// called; only its record machinery is exercised.
#include <ISO_Fortran_binding.h>
#include <cstring>

extern "C" void addtolist_c(CFI_cdesc_t* list, int element);

#include "debug.cpp"

// The wall-clock bridge, stubbed to the constant the Fortran probe writes.
extern "C" void vit_curdate_c(char* b) { std::memcpy(b, "01-Jan-2026", 11); }
extern "C" void vit_curtime_c(char* b) { std::memcpy(b, "00:00:00", 8); }
extern "C" void addtolist_c(CFI_cdesc_t*, int) {}

int main() {
    const int N = 26;
    const double v[N] = {
        0.0, -0.0, 1.0, -1.0, 0.5,
        1.0e-99, 1.0e+99, -1.0e-99, -1.0e+99, 9.99999,
        1.234567890123e5, -1.234567890123e-5, 3.14159265358979, 1.0e-308, 1.0e-320,
        123456.789, -123456.789, 1.0 / 3.0, 2.0 / 3.0, 1.0e10,
        -1.0e10, 9.999995e-1, 1.0e-1, 6.02214076e23, -6.02214076e-23,
        99999.999999};

    std::FILE* f = std::fopen("fmt_probe.c.out", "w");

    for (int i = 0; i < N; ++i) {
        Rec r;
        put_f(r, v[i], 20, 5);
        r.tr(5);
        put_es(r, v[i], 20, 5, 2);
        r.tr(5);
        put_es(r, -v[i], 20, 5, 2);
        r.tr(5);
        r.write_to(f);
    }

    {
        const char* names[4] = {"WE_Cp", "NacHeadingTarget",
                                "SU_LoadStageStartTime", "[-]"};
        Rec r;
        put_a(r, "Time", 4, 20);
        r.tr(5);
        for (int i = 0; i < 4; ++i) {
            put_a(r, FChar<15>(names[i]), 20);
            r.tr(5);
        }
        r.write_to(f);
    }

    for (int i = 0; i < N; ++i) {
        Rec r;
        r.put("Generator speed: ", 17);
        put_f(r, v[i] * 9.5492966, 6, 1);
        r.put(" RPM, Pitch angle: ", 19);
        put_f(r, v[i] * 57.2957795130, 5, 1);
        r.put(" deg, Power: ", 13);
        put_f(r, v[i] / 1000.0, 7, 1);
        r.put(" kW, Est. wind Speed: ", 22);
        put_f(r, v[i], 5, 1);
        r.put(" m/s", 4);
        r.write_to(f);
    }

    write_generated_on(f);

    std::fclose(f);
    return 0;
}
