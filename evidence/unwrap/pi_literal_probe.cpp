// The same three constants as the C++ translation spells them, plus M_PI --
// the mistake this probe exists to make impossible to make silently.
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
static uint64_t bits(double v){ uint64_t u; std::memcpy(&u,&v,8); return u; }
int main(){
    constexpr double PI = 3.14159265359;
    constexpr double TWO_PI = 2.0 * PI;
    std::printf("PI      bits = %016llX\n", (unsigned long long)bits(PI));
    std::printf("2*PI    bits = %016llX\n", (unsigned long long)bits(TWO_PI));
    std::printf("-PI     bits = %016llX\n", (unsigned long long)bits(-PI));
    std::printf("PI      dec  = %.17E\n", PI);
    std::printf("2*PI    dec  = %.17E\n", TWO_PI);
    std::printf("M_PI    bits = %016llX   <- NOT what Constants.f90 declares\n",
                (unsigned long long)bits(M_PI));
    std::printf("M_PI    dec  = %.17E\n", M_PI);
    std::printf("PI - M_PI    = %.6E   (%.0f ULP at this magnitude)\n",
                PI - M_PI, (double)(bits(PI) - bits(M_PI)));
    return 0;
}
