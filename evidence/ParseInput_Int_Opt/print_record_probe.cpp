// The TRANSLATION half: the same six records through the SHIPPED
// translation's own `print_default_warning`.
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
void findline_c(char*, int, int, char*, int, int32_t*, char*, int*, int, int) { std::abort(); }
void getwords_c(char*, int, char*, int, int) { std::abort(); }
void int2lstr_c(int, char*) { std::abort(); }
}
#include VIT_TRANSLATION
int main() {
    struct C { const char* vn; std::int32_t v; };
    const C cs[] = { {"PC_KP",0}, {"X",0}, {"",0}, {"WE_Mode",-2147483647-1},
                     {"A",2147483647}, {nullptr,7} };
    for (int i = 0; i < 6; ++i) {
        std::string vn = cs[i].vn ? std::string(cs[i].vn) : std::string(60, 'Z');
        std::vector<char> buf(200, ' ');
        std::memcpy(buf.data(), vn.data(), vn.size());
        print_default_warning(ftrim(buf.data(), 200), cs[i].v);
    }
    return 0;
}
