// STUB, not a translation. The SHIPPED translation (predicate form) with its
// answer INVERTED and nothing else changed. The differential harness must fail
// EVERY case and name vit_result, the unit's only output.
#include <cstdint>
static bool contains_pair(const char* s, int len_s, char c1, char c2) {
    for (int i = 2; i <= len_s; ++i) {
        if (s[(i - 1) - 1] == c1 && s[i - 1] == c2) { return true; }
    }
    return false;
}
static bool contains_char(const char* s, int len_s, char c) {
    for (int i = 1; i <= len_s; ++i) {
        if (s[i - 1] == c) { return true; }
    }
    return false;
}
static const char Separators[] = { '/', '\\' };
int32_t PathIsRelative(char* GivenFil, int len_GivenFil) {
    int32_t PathIsRelative = 1;                       // INVERTED
    if (!contains_pair(GivenFil, len_GivenFil, ':', '/') &&
        !contains_pair(GivenFil, len_GivenFil, ':', '\\')) {
        if (!contains_char(Separators, (int)sizeof Separators, GivenFil[1 - 1])) {
            PathIsRelative = 0;                       // INVERTED
        }
    }
    return PathIsRelative;
}
