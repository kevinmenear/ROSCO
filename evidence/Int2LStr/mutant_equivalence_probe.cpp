// Int2LStr: are the five surviving mutants equivalent ON THE IN-BOUNDS RESULT?
//
// RUNBOOK, unit #8: "Before declaring a survivor equivalent on an out-of-bounds
// argument, PROVE it exhaustively rather than reasoning about it. Enumerate the
// input space at small sizes with the bytes past the buffer set to the value
// most able to disagree, and count in-bounds disagreements." That unit also
// measured why reasoning is not enough: three identical runs of the same
// mutation command scored 0.983, 1.000, 0.983, because whether such a mutant
// dies depends on the heap.
//
// So this probe does not argue. It runs the original and each mutant over the
// domain below, on a 12-byte buffer whose 12th byte -- the one past the
// CHARACTER(11) result -- is POISONED to a value no correct output contains,
// and it counts disagreements in bytes 0..10 only.
//
// The five survivors, by line of translations/ROSCO_Helpers/int2lstr.cpp:
//   A  line 39  (Num <  0) -> (Num <= 0)   in the magnitude ternary
//   B  line 39  (Num <  0) -> (Num <  1)   in the magnitude ternary
//   C  line 67  first + i <  W -> <=       the ADJUSTL shift bound
//   D  line 70  i <  W -> <=               the trailing blank-fill bound
//   E  line 24  const int W = 11 -> 12     the declared width
//
// DOMAIN. The function's behaviour is a function of the DECIMAL STRING of Num,
// so the only places it can change are the decade boundaries -- where the digit
// count changes -- and the sign. The domain is therefore exhaustive where the
// structure lives:
//   * every value with |Num| <= 2,000,000            (widths 1..7, all of them)
//   * every value within 1000 of each decade boundary 10^k, both signs, k<=9
//   * every value within 1000 of INT_MAX and of INT_MIN
// Every width and every width TRANSITION is covered by an exhaustive sweep, not
// a sample.

#include <cstdio>
#include <cstdint>
#include <cstdlib>

static const char POISON = '\x7f';   // no correct output byte is ever this

#define BODY(NAME, LT_A, LT_B, LT_C, LT_D, WIDTH)                        \
static void NAME(int Num, char* Int2LStr_result) {                       \
    const int W = WIDTH;                                                 \
    long long mag = (Num LT_A) ? -(long long)Num : (long long)Num;       \
    int first = W;                                                       \
    do {                                                                 \
        Int2LStr_result[--first] = (char)('0' + (int)(mag % 10));        \
        mag /= 10;                                                       \
    } while (mag != 0);                                                  \
    if (Num LT_B) {                                                      \
        Int2LStr_result[--first] = '-';                                  \
    }                                                                    \
    for (int i = 0; first + i LT_C W; ++i) {                             \
        Int2LStr_result[i] = Int2LStr_result[first + i];                 \
    }                                                                    \
    for (int i = W - first; i LT_D W; ++i) {                             \
        Int2LStr_result[i] = ' ';                                        \
    }                                                                    \
}

BODY(orig,     < 0,  < 0, <,  <,  11)
BODY(mutant_A, <= 0, < 0, <,  <,  11)
BODY(mutant_B, < 1,  < 0, <,  <,  11)
BODY(mutant_C, < 0,  < 0, <=, <,  11)
BODY(mutant_D, < 0,  < 0, <,  <=, 11)
BODY(mutant_E, < 0,  < 0, <,  <,  12)

typedef void (*Fn)(int, char*);
static const char* NAMES[] = {"A <=0 ternary", "B <1 ternary",
                              "C shift bound", "D fill bound", "E width 12"};
static Fn FNS[] = {mutant_A, mutant_B, mutant_C, mutant_D, mutant_E};

static long long differ[5];
static long long checked;

static void probe(int Num) {
    char ref[16], got[16];
    for (int i = 0; i < 16; ++i) ref[i] = POISON;
    orig(Num, ref);
    checked++;
    for (int m = 0; m < 5; ++m) {
        for (int i = 0; i < 16; ++i) got[i] = POISON;
        FNS[m](Num, got);
        for (int i = 0; i < 11; ++i) {          // IN BOUNDS ONLY
            if (got[i] != ref[i]) { differ[m]++; break; }
        }
    }
}

int main(void) {
    for (long long v = -2000000; v <= 2000000; ++v) probe((int)v);

    long long d = 10;
    for (int k = 1; k <= 9; ++k) {
        for (long long v = d - 1000; v <= d + 1000; ++v) {
            probe((int)v);
            probe((int)-v);
        }
        d *= 10;
    }
    for (long long v = 2147483647LL - 1000; v <= 2147483647LL; ++v) probe((int)v);
    for (long long v = -2147483648LL; v <= -2147483648LL + 1000; ++v) probe((int)v);

    printf("checked %lld input(s)\n", checked);
    for (int m = 0; m < 5; ++m)
        printf("  mutant %-16s differ-IN-BOUNDS %lld\n", NAMES[m], differ[m]);
    return 0;
}
