# `ParseDbAry_Opt` — seventeen mutants declared EQUIVALENT, with the argument for each

An equivalence is a claim about the two PROGRAMS: they agree on every admissible
input, not merely on the 16,512 the corpus supplies. That is a different and
stronger claim than `unreachable`, which is about the CORPUS and is declared
separately in `mutation/ParseDbAry_Opt.unreachable.json` against a measured
zero-coverage line.

Written here rather than in the JSON so the reasons can be disputed. Each is
stated as something a reader can check against
`translations/ROSCO_Helpers/parsedbary_opt.cpp`.

Three of these (`b76a0c84`, `d764720b`, `412b5d5b`) sit on lines the corpus also
never executes. They are declared EQUIVALENT rather than UNREACHABLE on purpose:
the stronger claim is available, and a mutant declared under the weaker one
would come back the moment the corpus grew.

---

## A. A NONZERO IOSTAT IS A NONZERO IOSTAT — 8 mutants

    parsedbary_opt.cpp:676   ErrStatLcl = list_read_reals(Line.data(), MaxLineLength, values.data(), n);
    parsedbary_opt.cpp:681   if (ErrStatLcl != 0) {          <- the only read of it

`list_read_reals` has exactly one call site, and the value it returns has
exactly one use: the test at L681. `ErrStatLcl` is reused earlier in the unit
(L582, the ALLOCATE emulation) but that is a different assignment; nothing reads
the value stored at L676 after L681, and the function returns. The value itself never reaches an output — the error
message the arm builds is the reference's own text and does not name the status
(`ROSCO_Helpers.f90`, the READ arm, quotes `FileName`, `ParamName`, `LineNum`
and `Line` and nothing else). So every mutant that changes one nonzero return
into a different nonzero return is behaviour-preserving.

    37ba45eb   arith_op     L327  `return -1`   -> `return + 1`
    cf85bd96   const_tweak  L327  `return -1`   -> `return -2`
    81b6a619   const_tweak  L366  `return 5010` -> `5011`
    49ce3a9d   const_tweak  L377  `return 5010` -> `5011`
    1bbaf2d2   const_tweak  L380  `return 5010` -> `5011`
    b76a0c84   const_tweak  L338  `return 5010` -> `5011`
    d764720b   const_tweak  L349  `return 5010` -> `5011`
    412b5d5b   const_tweak  L374  `return 5010` -> `5011`

**HOW TO DISPUTE IT:** find a second use of `ErrStatLcl` after line 676, or an
output that carries the status value. `grep -n 'ErrStatLcl' translations/
ROSCO_Helpers/parsedbary_opt.cpp` is the whole check. Note the value 5010 is
NOT arbitrary — it is libgfortran's own `LIBERROR_READ_VALUE`, measured in
`fortran_io_probe.txt` — but the reference does not expose it either.

## B. ENLARGING A LOCAL BUFFER NOBODY MEASURES — 4 mutants

    7302f2f5   index_offset  L108  `char buf[Int2LStrLen]` -> `[Int2LStrLen + 1]`
    00d622d9   const_tweak   L231  `char buf[64]`          -> `char buf[65]`
    057b4549   index_offset  L231  `char buf[64]`          -> `char buf[64 + 1]`
    fa5926ac   index_offset  L457  `char tmp[512]`         -> `char tmp[512 + 1]`

A larger automatic array changes behaviour only if the program reads its SIZE
or overruns the original. Taken one at a time:

* **L108.** `int2lstr_c(Num, buf)` writes the callee's `CHARACTER(11)` and
  `ftrim(buf, Int2LStrLen)` reads exactly `Int2LStrLen` = 11 bytes. Both bounds
  are the named constant, not `sizeof`, so 11 bytes are written and 11 read at
  either array size.
* **L231.** The fill loop is `while (q < len && n < 62 && !is_separator(rec[q]))`
  followed by `buf[n] = '\0'`, so at most index 62 is written. 64 and 65 both
  admit it, and `sizeof buf` appears nowhere.
* **L457.** `sizeof tmp` IS used — `std::snprintf(tmp, sizeof tmp, "%.16E", v)`
  — so this one is not equivalent merely because the array grew. It is
  equivalent because the bound cannot bind at either size: `%.16E` of a finite
  double is sign + one digit + `.` + sixteen digits + `E` + sign + at most three
  exponent digits = 24 characters, and the non-finite values return at L453
  before reaching this line. 25 bytes against 512 or 513.

**HOW TO DISPUTE IT:** show a `sizeof` on `buf` at L108 or L231, or a `%.16E`
conversion of a finite double longer than 511 characters.

## C. `lower()` IS ONLY EVER COMPARED AGAINST NINE LETTERS, AND `z` IS NOT ONE — 1 mutant

    5bfdaacf   compare_op   L170  `(c >= 'A' && c <= 'Z')` -> `(c >= 'A' && c < 'Z')`

The mutant stops lower-casing exactly one character, `'Z'`, and returns it
unchanged. `lower` has three call sites and every one feeds a comparison:

    L179   lower(rec[p + k]) != word[k]      word ∈ {"infinity", "inf", "nan"}
    L267   const char c = lower(rec[p]);  c == 'e' || c == 'd' || c == 'q'
    L306   lower(exp[0]) == 'e' || == 'd' || == 'q'

The letters compared against are `i n f t y a e d q`. `'z'` is not among them,
so `'Z'` and `'z'` are both unequal to every one of them and every comparison
has the same value under both programs. The result is never stored, printed or
returned.

**HOW TO DISPUTE IT:** a fourth call site of `lower`, or a comparison against
`'z'`. `grep -n 'lower(' translations/ROSCO_Helpers/parsedbary_opt.cpp`.

NOTE THE SIBLING MUTANT IS **NOT** DECLARED. `6b5fc8ce` moves the OTHER bound,
`c >= 'A'` -> `c > 'A'`, and stops lower-casing `'A'` — and `'a'` IS the middle
letter of `"nan"`. That one is a live survivor, not an equivalence, and the
difference between the two is the whole reason this is written per mutant rather
than per line.

## D. A GUARD THE CALLER ALREADY ESTABLISHED — 1 mutant

    0a3516d1   compare_op   L223  `if (p < len && (rec[p] == '+' || rec[p] == '-'))`
                                  -> `if (p <= len && ...)`

`parse_real` has one call site, `list_read_reals:364`, and on the path to it
`p < len` is already established and not invalidated:

    L326   if (p >= len) { return -1; }        <- p < len from here on
    L329   rec[p] == ','   -> ++p; continue    <- restarts the loop, re-guarded
    L334   rec[p] == '/'   -> return 0
    L337   rec[p] == ';'   -> return 5010
    L351   p = q + 1       -> L353 `if (p >= len || ...) { ...; continue; }`
                              so a p that reached len continues rather than
                              falling through
    L364   parse_real(rec, len, p, value)      <- p < len

`p < len` and `p <= len` therefore have the same value at L223 on every
execution, and the short-circuit means `rec[p]` is evaluated at the same
subscripts. This is the `p < len` -> `p <= len` shape the sanitiser was run for,
and it is the one instance of it that is provably harmless rather than merely
unobserved — the other five (`2763d449`, `2f05620e`, `56c1dd4d`, `a98553ab`,
`a0007207`) are at points where `p` CAN have advanced to `len`, and they are
left as live survivors.

**HOW TO DISPUTE IT:** a second call site of `parse_real`, or a path from L326
to L364 that advances `p` to `len` without returning or continuing.

## E. A DEAD INITIALISER — 1 mutant

    269b76b7   const_tweak  L363  `double value = 0.0;` -> `double value = 1.0;`

`value` is passed by reference to `parse_real` on the very next line and is read
only afterwards, on the two verdicts that assign it:

    RealParse::Ok             -> L313 `out = std::strtod(...)`  (or L245 for inf/nan)
    RealParse::BadStoreZero   -> L291 `out = 0.0;`
    RealParse::BadNoStore     -> L366 returns 5010 without reading `value`

So no read of `value` can observe the initialiser.

**HOW TO DISPUTE IT:** a `return RealParse::Ok` or `BadStoreZero` in `parse_real`
that does not assign `out` first. The three `BadNoStore` returns are L243, L259
and L284, and none of them is preceded by an assignment -- which is the point:
that verdict never reads `value` either.

## F. `std::max` IS SYMMETRIC — 1 mutant

    dd749c6b   swap_call_args  L656  `std::max(NumWords, 0)` -> `std::max(0, NumWords)`

`std::max(a, b)` returns `a < b ? b : a`; `std::max(b, a)` returns `b < a ? a : b`.
For `int` these differ only in WHICH object is returned when `a == b`, and a
copy of two equal ints is indistinguishable. The result is immediately cast to
`std::size_t` and used as a length.

**HOW TO DISPUTE IT:** this argument fails for types whose equal values are
distinguishable (e.g. a comparator on a key alone). `NumWords` is `int`.

Its sibling `b2bbedf3`, which DROPS the clamp entirely, is **not** declared. It
is behaviour-preserving only while `NumWords >= 0`, which is a fact about the
stated domain `AryLen = { lo = 0, hi = 32 }` and not about the program.

## G. A CONSTANT THAT SIZES A BUFFER NOTHING READS — 1 mutant

    4c9f1d23   const_tweak  L50  `constexpr int MaxParamLength = 200;` -> `201`

`MaxParamLength` has exactly two uses, and they are the two halves of one
allocation:

    L655-657   std::vector<char> Words_Ary(NumWords * MaxParamLength)
    L661       getwords_c(Line.data(), MaxLineLength, Words_Ary.data(),
                          MaxParamLength, NumWords)

The element width the callee is told and the width the buffer is sized at are
the SAME constant, so the two move together and the call stays consistent.
`Words_Ary` is then never read: the reference's only consumer is the
`IF (DEBUG_PARSING)` block, and `DEBUG_PARSING` is a `.FALSE.` PARAMETER in
every build of this tree (evidence README §2). `grep -n 'Words_Ary'` shows a
write and no read.

**HOW TO DISPUTE IT:** a third use of `MaxParamLength`, or any read of
`Words_Ary` after L661.

---

## What is NOT here, and why

Nothing in the PRINT region is declared. Those mutants are reached — the default
arm runs — and they change what the record says; nothing in this campaign
compares that record. That is an INSTRUMENT gap, and calling it an equivalence
would be the exact shape of a green that established nothing.

Nothing in `match_word` is declared either. The seven survivors there change
whether an `inf`/`nan` word matches, and no case in this corpus spells one — but
that is a statement about the corpus, and the corpus can grow.
