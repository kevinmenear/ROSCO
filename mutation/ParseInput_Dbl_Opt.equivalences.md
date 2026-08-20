# `ParseInput_Dbl_Opt` — declared equivalences, with the argument for each

Twenty-seven mutants of `translations/ROSCO_Helpers/parseinput_dbl_opt.cpp` are
declared `equivalent` in `mutation/ParseInput_Dbl_Opt.equivalences.json`. An
equivalence is a claim about the **programs** — that they agree on every
admissible input — and not about the corpus, so each one is written out here
where it can be disputed rather than left as an id in a list.

Six families. Five of them turn on a fact about **this unit's single call site
into the parser**, and those facts are stated first; the sixth (§6, added after
the first sweep) turns on what `GetWords` can put in the buffer at all.

    ParseInput_Dbl_Opt calls   list_read_reals(Words1, MaxParamLength, &value, 1)

  * `n == 1`. The reference reads ONE item — `READ (Words(1),*) Variable` —
    where both siblings read `SIZE(Ary)`.
  * `len == 200`, always. `MaxParamLength` is a Fortran PARAMETER and this is
    the only call.
  * `p == 0` whenever `parse_real` is entered. `Words1` is `GetWords`'s output,
    and `GetWords` assigns `Words(IW) = Line(Ch+1:Ch+NextWhite-1)` into a
    `CHARACTER(200)` — left-justified, blank-padded. `list_read_reals` skips
    leading blanks and returns `-1` if the record is all blank, so the only
    position at which it can reach `parse_real` is 0.
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

## 3. The `p == 0` family — 5 mutants

`parse_real` is entered at `p == 0` on every call (see the facts above), so
`start == 0`, and `match_word` is called at `p ∈ {0, 1}` against words of
length 3 or 8 with `len == 200`.

| id | line | edit | why the two agree |
|---|---|---|---|
| `8475def2` | 184 | `p < len` → `p <= len` | `p == 0`, `len == 200`; both true |
| `575c6151` | 156 | `p + size` → `p - size` | `1 - 8 > 200` and `1 + 8 > 200` are both false |
| `52112be9` | 156 | `>` → `>=` | `9 >= 200` is false, as `9 > 200` is |
| `ec2c7596` | 263 | `rec + start` → `rec - start` | `start == 0` |
| `2a6d08a0` | 263 | `exp_start - start` → `exp_start + start` | `start == 0` |

The gcov measurement is a second, independent witness for the two at line 156:
`return false` at line 157 is `#####` over all 11,562 cases, so the guard is
false on every call, which is what both mutants preserve.

**The disputable half.** The `p < len` guards at 213, 214, 217, 227 and 237 are
NOT declared even though they are the same operator on the same variable — by
then `p` has been advanced past digits and can equal `len`, and `rec[200]` is
the first byte of `Words(2)`, which is usually not a blank. Those five are
reported as open survivors below.

## 4. The buffer-size family — 4 mutants

A local buffer made one byte LARGER, where the writes into it are bounded by
something else.

| id | line | edit | the bound |
|---|---|---|---|
| `c09407e0` | 107 | `char buf[Int2LStrLen]` → `[12]` | `int2lstr_c` writes exactly 11 and `ftrim(buf, 11)` reads exactly 11 |
| `44f8fb11` | 192 | `char buf[64]` → `[65]` | the copy loop stops at `n < 62`, so at most `buf[62]` is written |
| `85dacdff` | 192 | `64` → `65` | same site, same bound |
| `965d4416` | 410 | `char tmp[512]` → `[513]` | `snprintf(tmp, sizeof tmp, ...)`; `sizeof tmp` grows with it, and the longest output of `%.16E` or of `%.*f` with `d <= 17` on a `double` is under 340 bytes |

**The disputable half.** `62` → `63` at line 195 (`4c451ef0`) is NOT declared:
it changes how many characters of an IEEE word are copied, which is a real
behavioural difference at a 62-character field. It is an open survivor, not an
equivalence.

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

## What the sweep checks about all of this

`vit_mutate.py` builds and runs every declared mutant anyway and reports
`declared_but_killed`. An empty list there is the control on this file: a
mutant this document calls equivalent that the corpus kills would fail P12
outright, and the tool would name it.
