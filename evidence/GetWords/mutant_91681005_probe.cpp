// Does the surviving mutant differ from the original only past the end of Line?
// Enumerates every Line over a 3-letter alphabet (a separator, a non-separator,
// a blank) at lengths 1..7, runs both scan_first call shapes on every reachable
// Ch, and reports whether any DIFFERING answer was produced without reading
// past the buffer.  A guard byte after the buffer is set to a separator so the
// mutant's out-of-bounds region is maximally able to disagree.
#include <cstdio>
#include <cstring>
#include <vector>
static const char WS[] = { ' ', ',', '!', ';', '\'', '"', '\t' };
static int scan_first(const char* s, int len, const char* set, int nset) {
    for (int i = 1; i <= len; ++i)
        for (int k = 1; k <= nset; ++k)
            if (s[i - 1] == set[k - 1]) return i;
    return 0;
}
int main() {
    const char alpha[3] = { ' ', 'X', ',' };
    long differ_inbounds = 0, differ_oob = 0, same = 0;
    for (int len = 1; len <= 7; ++len) {
        long total = 1; for (int i = 0; i < len; i++) total *= 3;
        for (long n = 0; n < total; ++n) {
            std::vector<char> buf(len + 64, ',');     // guard bytes are separators
            long m = n;
            for (int i = 0; i < len; ++i) { buf[i] = alpha[m % 3]; m /= 3; }
            const char* Line = buf.data();
            for (int Ch = 0; Ch <= len; ++Ch) {
                int ok  = scan_first(Line + Ch, len - Ch, WS, (int)sizeof WS);
                int bad = scan_first(Line + Ch, len + Ch, WS, (int)sizeof WS);
                if (ok == bad) { same++; continue; }
                // Where did the mutant have to look to disagree?  It answers
                // `bad`, at 1-based offset Ch+bad; in bounds iff <= len.
                if (bad != 0 && Ch + bad <= len) differ_inbounds++; else differ_oob++;
            }
        }
    }
    printf("same %ld  differ-only-past-the-end %ld  differ-IN-BOUNDS %ld\n",
           same, differ_oob, differ_inbounds);
    return differ_inbounds != 0;
}
