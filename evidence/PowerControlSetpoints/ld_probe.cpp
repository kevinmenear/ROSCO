// Generated wrapper. Do not edit -- run_ld_probe.sh rewrites it.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
namespace {
#include "ld_probe.slice.cpp"
}  // namespace
int main() {
    std::FILE* fb = std::fopen("vals.bin", "rb");
    std::FILE* fr = std::fopen("fort.401", "rb");
    if (!fb || !fr) { std::fprintf(stderr, "probe: corpus missing\n"); return 2; }
    long n = 0, bad = 0;
    double v;
    char ref[64];
    while (std::fread(&v, sizeof v, 1, fb) == 1) {
        if (std::fgets(ref, sizeof ref, fr) == nullptr) {
            std::fprintf(stderr, "probe: fewer records than values at %ld\n", n);
            return 2;
        }
        size_t len = std::strlen(ref);
        while (len && (ref[len - 1] == '\n' || ref[len - 1] == '\r')) ref[--len] = '\0';
        const std::string got = list_directed_real(v);
        if (got != std::string(ref, len)) {
            if (bad < 10) {
                std::printf("MISMATCH %ld  value %.17g\n  gfortran [%s]\n  C++      [%s]\n",
                            n, v, ref, got.c_str());
            }
            ++bad;
        }
        ++n;
    }
    if (std::fgets(ref, sizeof ref, fr) != nullptr) {
        std::fprintf(stderr, "probe: more records than values\n");
        return 2;
    }
    std::printf("values compared %ld\nmismatched     %ld\n", n, bad);
    return bad == 0 ? 0 : 1;
}
