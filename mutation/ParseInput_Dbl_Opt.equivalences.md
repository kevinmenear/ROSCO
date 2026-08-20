# `ParseInput_Dbl_Opt` — declared equivalences, with the argument for each

Twenty-six mutants of `translations/ROSCO_Helpers/parseinput_dbl_opt.cpp` are
declared `equivalent` in `mutation/ParseInput_Dbl_Opt.equivalences.json`. An
equivalence is a claim about the **programs** — that they agree on every
admissible input — and not about the corpus, so each one is written out here
where it can be disputed rather than left as an id in a list.

Seven families. The first five turn on a fact about **this unit's single call
site into the parser**; the sixth turns on what `GetWords` can put in the buffer
at all, and the seventh (§7) on the repeat-count ceiling added at the fourth
dispatch.

    ParseInput_Dbl_Opt calls   list_read_reals(Words1, MaxParamLength, &value, 1)

  * `n == 1`. The reference reads ONE item — `READ (Words(1),*) Variable` —
    where both siblings read `SIZE(Ary)`.
  * `len == 200`, always. `MaxParamLength` is a Fortran PARAMETER and this is
    the only call.
  * ~~`p == 0` whenever `parse_real` is entered.~~ **WITHDRAWN AT THE FOURTH
    DISPATCH, AND IT IS THE MOST IMPORTANT CORRECTION IN THIS FILE (C12).** The
    argument was: `Words1` is `GetWords`'s output, and `GetWords` assigns
    `Words(IW) = Line(Ch+1:Ch+NextWhite-1)` into a `CHARACTER(200)` —
    left-justified, blank-padded — so `list_read_reals`, which skips leading
    blanks, can only reach `parse_real` at position 0.

    The left-justification half is true and is used all over this file. The
    conclusion is FALSE, because `list_read_reals` reaches `parse_real` from
    **two** places: the top of the item loop, where `p` is indeed 0 or 1, and
    the fall-through from the **repeat count**, where `p == q + 1` and `q` is
    wherever the `*` sat. `3*7` enters `parse_real` at `p == 2`; a count behind
    leading zeros enters it near the record's last byte.

    The corpus that carried the premise had no `*` in any record a search key
    matched, so nothing contradicted it for three dispatches. The corpus that
    has one **killed two mutants this file had declared equivalent on it**
    (`ec2c7596`, `2a6d08a0`), and `vit_mutate.py` named them in
    `declared_but_killed`. Two more (`575c6151`, `52112be9`) rested on the same
    premise and were withdrawn in the same edit; the corpus then killed
    `52112be9` too, and `575c6151` is re-declared in §3 on an argument that does
    not use the premise.

    **A fact about where a corpus enters a function is not a fact about the
    function.** This entry is left struck through rather than deleted because
    the four ids above are what it cost.
  * `*Variable == +0.0` at the PRINT. `Variable = 0` is the statement
    IMMEDIATELY above it and is unconditional.

Each family below names the mutant ids, the site, and what makes the two
programs agree. `list_read_reals` and `parse_real` are `static` in an anonymous
namespace and are each called from exactly one place, which is what lets a
whole-program argument be made about them at all.

---

## 1. The IOSTAT family — 8 mutants

The status `list_read_reals` returns is read in exactly one place:

```cpp
const int ErrStatLcl = list_read_reals(Words1, MaxParamLength, &value, 1);
*Variable = value;
if (ErrStatLcl != 0) {
```

and the reference does the same thing — `IF ( ErrStatLcl /= 0 )`. The value is
never printed, never compared against a literal, never returned. So **any
nonzero status is the same program as any other nonzero status**, and every
mutant below turns one nonzero into another.

| id | line | edit | resulting status |
|---|---|---|---|
| `60b5717a` | 288 | `return -1` → `return + 1` | `+1` |
| `0bf7868b` | 288 | `-1` → `-2` | `-2` |
| `e97b4153` | 299 | `5010` → `5011` | `5011` |
| `c62206c9` | 310 | `5010` → `5011` | `5011` |
| `9762906a` | 327 | `5010` → `5011` | `5011` |
| `2068cffd` | 335 | `5010` → `5011` | `5011` |
| `8bb3596d` | 338 | `5010` → `5011` | `5011` |
| `5da57481` | 341 | `5010` → `5011` | `5011` |

**What is deliberately NOT claimed.** The three spellings 0 / 5010 / -1 are
reproduced in the translation because the *next* unit in this family may read
the value rather than its sign, and because a model that returned "1" for both
failure kinds would be a model of the TEST rather than of the READ (the comment
above `parse_real` says so). That is a reason to keep them, not a reason to
call them observable HERE. The sibling `ParseInAry_Opt` declares the identical
family for the identical reason.

**The disputable half.** `return 0` at line 296 — the `/` terminator — is NOT
declared, and it is the control on this family: zero is the one status the
`!= 0` reader separates, and the mutant that changes it is expected to be
killable.

## 2. The `n == 1` family — 4 mutants

The reference transfers ONE item, so `list_read_reals`'s loop body runs at most
once and everything the body does to `p` after the store is dead:

```cpp
for (long k = 0; k < repeat && i < n; ++k) {
    v[i++] = value;          // i: 0 -> 1 == n
}
while (p < len && is_blank(rec[p])) { ++p; }     // <- dead
if (p < len && rec[p] == ',') { ++p; }           // <- dead
```

After the store `i == n == 1`, the `while (i < n)` condition is false, the
function returns `0`, and `p` is a local that nothing outside reads.

| id | line | edit |
|---|---|---|
| `8c4c86d0` | 346 | `rec[p]` → `rec[p + 1]` in the trailing blank skip |
| `70cae959` | 347 | `rec[p]` → `rec[p + 1]` in the trailing comma skip |
| `ffd9ba9f` | 347 | that same `if` negated |

and one more that is `n == 1` seen from the repeat count:

| `99f9997a` | 303 | `long repeat = 1` → `= 2` |

`repeat` is assigned only at line 307, which no case executes, so it is `1` on
every path that reaches the store; and with `n == 1` the store loop stops on
`i < n` after one iteration whatever `repeat` holds. `repeat`'s other reader,
`if (repeat <= 0)` at line 309, is inside the block line 307 belongs to.

**The disputable half.** `for (long k = 0; ...)` at line 343 — the store loop's
own induction variable — is NOT declared: `k = 1` would skip the store
entirely and change `Variable`.

## 3. The `match_word` family — 3 mutants, re-argued from scratch

**This section replaces the `p == 0` family, whose premise is withdrawn above.**
Two of its five members were killed by the corpus, one more was killed after its
declaration was withdrawn, and the two that remain are re-argued here without
using `p == 0` at all.

```cpp
bool match_word(const char* rec, int len, int p, std::string_view word) {
    if (p + static_cast<int>(word.size()) > len) { return false; }   // the bound
    for (std::size_t k = 0; k < word.size(); ++k) {                  // the loop
        if (lower(rec[p + k]) != word[k]) { return false; }
    }
    return true;
}
```

It is called from exactly one place, with `word` one of `"infinity"`, `"inf"`,
`"nan"`, and only to decide whether to enter the IEEE-word branch. That branch
copies the field up to the next separator, prefixes the sign, and calls
`strtod`; if `strtod` does not consume the whole field it returns `BadNoStore`,
and the caller then returns `5010` **with the item unassigned**.

| id | line | edit | why the two agree |
|---|---|---|---|
| `575c6151` | 156 | `p + size > len` → `p - size > len` | the mutant's guard is never true (`p <= len` and `size > 0`), so it accepts a strict SUPERSET. The only extra acceptances are where the word would run PAST the record — and there the copy loop stops at `len`, so `strtod` is handed a TRUNCATED `inf`/`nan`, rejects it, and returns `BadNoStore`. The original, on the same record, sees `rec[p] ∈ {i,n}`, which is not a digit and not a `.`, so its scan gives `digits == 0 && !saw_point` and returns `BadNoStore` too. Both reach `return 5010` with nothing stored |
| `8e796788` | 159 | `k = 0` → `k = 1` | skipping `k = 0` again accepts a strict SUPERSET: fields whose characters 2..n spell `nf` / `an` / `nfinity` but whose FIRST character is not `i`/`n`. For such a field `strtod` must consume the whole string for `Ok`, and the only tokens `strtod` accepts containing an `n` are the IEEE words, which begin with `i`/`n` after the sign — excluded by construction. So the mutant's extra acceptances all end in `BadNoStore`. The original on those same records reaches `BadNoStore` (first character not a digit, sign or point), or `BadStoreZero` at a leading `.` whose terminator test then fails, or `Ok` on a leading digit whose terminator test then fails — every one of which is `return 5010` with the item unassigned |

**The disputable half, and it is a KILL on the same two lines.** `52112be9`
(`>` → `>=` at 156) is NOT declared: it makes the guard REJECT where the
original accepts, at `p + size == len` exactly, and the record
`0…03*nan` — a legal count of 3 behind 195 leading zeros, so the value starts at
byte 197 — distinguishes it. It is **killed** on the 13,802-case corpus. So line
156 is executed at the boundary, the branch is observable there, and these two
declarations are claims about the VALUE at that site rather than about
reachability. The `compare_op` mutants of the loop bound at 159 are killed too.

**And the second instrument was asked.** Both were run through the 27-scenario
gate, character for character out of the mutation artifact, each predicted
before its run in `evidence/ParseInput_Dbl_Opt/gate.equivalence_predictions.txt`:
**0 of 5,252,000 each**
(`gate/ParseInput_Dbl_Opt.equivalence.{8e796788,575c6151}.json`), against a
perturbation of the parsed value on the same build that moves 1,857,893. A
non-zero would have refuted the declaration.

## 3b. The two sign tests — 2 mutants, and they are the same edit twice

`parse_real` tests for a sign in exactly two places, and the SAME mutation of
`p < len` is behaviourally different at each. Both are declared, for two
different reasons, and neither reason is `p == 0`.

```cpp
RealParse parse_real(const char* rec, int len, int& p, double& out) {
    const int start = p;
    if (p < len && (rec[p] == '+' || rec[p] == '-')) { ++p; }   // 184  8475def2
    ...
    if (has_exp) {
        if (p < len && (rec[p] == '+' || rec[p] == '-')) { ++p; }   // 254  209527b9
        int edigits = 0;
        while (p < len && is_digit(rec[p])) { ++p; ++edigits; }
        if (edigits == 0) { return RealParse::BadNoStore; }
    }
```

| id | line | edit | why the two agree |
|---|---|---|---|
| `8475def2` | 184 | `p < len` → `p <= len` | `p < len` is an INVARIANT at entry, so the two guards are the same guard. `list_read_reals` reaches `parse_real` from two places and both establish it: the item loop `return -1`s when `p >= len` after the blank scan, and the repeat-count fall-through `continue`s when `p >= len`. There is no third caller — the function is `static` in an anonymous namespace |
| `209527b9` | 254 | `p < len` → `p <= len` | at `p == len` the mutant may consume a sign the original does not, making `p == len + 1`. Both then run `while (p < len && ...)`, which is false either way, so `edigits == 0` and BOTH return `BadNoStore` — and on `BadNoStore` the caller returns `5010` immediately without reading `p`, so the one value the mutant changed is dead. `edigits` cannot be non-zero here: its own scan requires `p < len` |

**The disputable half.** Every other `p < len` → `p <= len` in `parse_real` — the
integer scan, the point test, the fraction scan, the exponent-letter test, the
exponent-digit scan — is NOT declared, and **all five are killed** on the
13,802-case corpus by the `ntail` records whose byte one past the record is the
character that scan tests. The declaration here is not "the guard is
unreachable at `p == len`"; it is that at `p == len` these two particular sites
compute the same answer.

## 4. The buffer-size family — 2 mutants

A local buffer made one byte LARGER, where the writes into it are bounded by
something else.

| id | line | edit | the bound |
|---|---|---|---|
| `c09407e0` | 107 | `char buf[Int2LStrLen]` → `[12]` | `int2lstr_c` writes exactly 11 and `ftrim(buf, 11)` reads exactly 11 |
| `965d4416` | 410 | `char tmp[512]` → `[513]` | `snprintf(tmp, sizeof tmp, ...)`; `sizeof tmp` grows with it, and the longest output of `%.16E` or of `%.*f` with `d <= 17` on a `double` is under 340 bytes |

**TWO ROWS WERE DELETED FROM THIS FAMILY AT THE FOURTH DISPATCH, AND THE REASON
IS THE POINT.** `44f8fb11` and `85dacdff` declared the `char buf[64]` of the
IEEE-word branch equivalent at 65 bytes, "because the copy loop stops at
`n < 62`". Both arguments were sound; the CODE was not. `n < 62` truncated any
IEEE word of 63 characters or more, so `nan(<58 chars>)` — which gfortran reads
as a NaN — was handed to `strtod` without its closing parenthesis and rejected.
The buffer and its bound are gone; the word is now bounded by the record. The
open survivor `4c451ef0` (`62` → `63`) went with them: it was a mutant that made
the translation LESS wrong, which is exactly why no corpus had killed it.
See `evidence/ParseInput_Dbl_Opt/record_form_probe.FIRST.txt`.

## 5. The dead-value family — 4 mutants

| id | line | edit | why the two agree |
|---|---|---|---|
| `d95b5244` | 324 | `double value = 0.0` → `1.0` | `parse_real` assigns `out` on both paths that return `Ok` or `BadStoreZero`, and the `BadNoStore` path returns before `value` is read |
| `10118ab3` | 151 | `c <= 'Z'` → `c < 'Z'` | `lower()`'s result is compared only against `i n f t y a e d q` — never `z` — so whether `'Z'` maps to `'z'` or stays `'Z'`, every comparison is false either way |
| `47d7f309` | 414 | `s.c_str() + epos + 1` → `- 1` | `v` is `+0.0` at the only call, so `s` is `"0.0000000000000000E+00"`, `epos == 18`, and `atoi("+00") == atoi("0E+00") == 0` |
| `27d9c8af` | 417 | `16 - decexp` → `16 + decexp` | `decexp == 0` for the same reason |

The last two are the reason this unit declares nothing about `list_directed_real`'s
E-form: **it is not reachable by the program**, not merely by the corpus. Every
line from 426 to 436 is `#####` in `evidence/ParseInput_Dbl_Opt/line_coverage.txt`,
and those mutants are declared `unreachable` rather than equivalent, because
"the corpus never runs the line" is the weaker claim and it is the one the
measurement supports directly.

**The disputable half.** `nonfinite_text`'s `isnan` at 395 is NOT declared even
though `+0.0` is neither NaN nor infinite: line 395 RUNS on every record, and
its mutant is killed.

## 6. The GetWords family — 2 mutants, and they were misfiled first

**ADDED AFTER THE FIRST SWEEP AND AFTER `mutation_survivors.txt` had already
called them corpus levers, which was wrong.** Recorded here as a correction
rather than a silent edit (C12): §A of that file says of every row "a corpus
that contained the record would kill the mutant", and for these two **no corpus
can contain the record.**

`rec` is `Words`, and every byte of it is written by `GetWords`, whose
separator set is a literal in the reference:

```fortran
NextWhite = SCAN( Line(Ch+1:) , ' ,!;''"'//Tab )
Words(IW) = Line(Ch+1:Ch+NextWhite-1)
```

So a word is a run of characters containing **none** of space, comma, `!`,
semicolon, `'`, `"` or tab; `GetWords` blank-fills all `NumWords` elements
before it writes anything; and the translation passes it a 400-byte buffer with
`len_Words = 200, NumWords = 2`, so all 400 bytes are its output. **No byte of
`rec[0..399]` is ever a comma or a semicolon.**

| id | line | edit | why the two programs agree |
|---|---|---|---|
| `4cbc3c66` | 290 | `rec[p] == ','` → `rec[p + 1] == ','` | neither byte is ever a comma. `p <= 199` here (the `p >= len` return is above it) so `p + 1 <= 200 < 400` and the read is in bounds |
| `8d81269b` | 298 | `rec[p] == ';'` → `rec[p + 1] == ';'` | the same, for the semicolon |

**The disputable half is the strongest in this file, because it is a KILL.**
Each of these two lines carries a `negate_cond` sibling —
`96b773cb` at 290 and `139ff398` at 298 — and **both were KILLED**. So the line
is executed, the branch is observable, and the equivalence is a claim about the
VALUE at that site rather than about reachability. Neither sibling is declared.

**And the second instrument DISTINGUISHES them, which is not a contradiction.**
`evidence/ParseInput_Dbl_Opt/survivor_record_search.txt` reports `4cbc3c66`
differing on 6,391 records and `8d81269b` on 924, because the search feeds
`list_read_reals` directly with records containing commas and semicolons.
**Those records are admissible inputs to the FUNCTION and not to the UNIT**:
`ParseInput_Dbl_Opt` cannot be handed one, because `GetWords` sits between its
inputs and that buffer. The gate agrees, and it agrees on the one that was
actually asked: `8d81269b` was run through all 27 scenarios and moved
**0 of 5,252,000** (`gate/ParseInput_Dbl_Opt.survivor.8d81269b.json`), against a
sibling perturbation on the same build that moved 1,583,216.

**The general shape, and it is why this section exists.** A search that drives
an internal function directly has a WIDER input space than the unit does, and
for a survivor whose site sits behind a callee that filter is the whole answer.
Ask what the callee can produce before reading a search's `differs` as a corpus
lever.

---

## 7. The repeat-count ceiling — 3 mutants, on code this dispatch ADDED

```cpp
long count = 0;
bool count_over = false;
for (int k = p; k < q; ++k) {
    count = count * 10 + (rec[k] - '0');
    if (count > MaxRepeat) { count_over = true; count = MaxRepeat + 1; }
}
if (count <= 0 || count_over || count > MaxRepeat) { return 5010; }
```

The clamp exists so that a 200-digit count cannot overflow a `long` during the
accumulation; `count_over` is what the guard actually reads, and it is
monotone — once set, nothing clears it.

| id | line | edit | why the two agree |
|---|---|---|---|
| `9c669e12` | 356 | `MaxRepeat + 1` → `MaxRepeat - 1` | the clamp VALUE. `count_over` is already `true` at this point, so the guard below returns `5010` whatever `count` holds, and `count` is dead after the guard. The only other reader is the next iteration's `count * 10 + digit`, which re-clamps: both `MaxRepeat ± 1` keep the accumulation bounded well inside a 64-bit `long` for a 200-digit record |
| `2cd50aab` | 356 | `MaxRepeat + 1` → `MaxRepeat + 2` | the same site, the same argument |
| `c62206c9` | 360 | `return 5010` → `5011` | the IOSTAT family (§1) one site over: `ErrStatLcl` is read once, as `ErrStatLcl /= 0`, and is not an output of the unit |

**The disputable half.** `ea396111` at 359 — `count <= 0` → `count <= 1` — is
NOT declared even though it sits in the same expression: it rejects `1*7`, which
is a legal record and a real behavioural difference. It is **killed** by the
`repone` entry the same dispatch added to the generator.

---

## What the sweep checks about all of this

`vit_mutate.py` builds and runs every declared mutant anyway and reports
`declared_but_killed`. An empty list there is the control on this file: a
mutant this document calls equivalent that the corpus kills would fail P12
outright, and the tool would name it.
