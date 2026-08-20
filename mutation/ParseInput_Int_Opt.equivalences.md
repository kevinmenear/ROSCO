# `ParseInput_Int_Opt` — 37 declared equivalences, in seven families

An EQUIVALENCE is a claim about the PROGRAMS: the shipped translation and the
mutant agree on every input admissible to **this unit**. It is not a claim about
the corpus — that is what `mutation/ParseInput_Int_Opt.unreachable.json` is for,
and a mutant already declared here is never re-declared there
(`evidence/ParseInput_Int_Opt/make_unreachable.py` skips four for exactly that
reason: the equivalence is the stronger claim and survives a corpus that later
reaches the line).

Ids are `cppmutate` OCCURRENCE INDICES, so they move when the translation moves.
Nothing in this file may be carried to another dispatch without re-deriving it
against `(operator, before, after, source line)` — unit #55's `declared_but_killed`
finding.

**The measurement every family rests on** is
`evidence/ParseInput_Int_Opt/survivor_record_search.txt`: 51,590 records driven
directly through `list_read_ints`, split into records **admissible to the unit**
(`GetWords` produces `<word><blanks>`, left-justified, with no interior blank, no
comma and no semicolon) and records only the **function** can be handed. Every
mutant declared below came back `NONE` over the admissible half, or `differs`
only on function-only records.

**And the premise every caller-based family needs is enumerable.**
`list_read_ints` is `static` inside an anonymous namespace and has exactly ONE
call site, `parseinput_int_opt.cpp:538`:

```
const int ErrStatLcl = list_read_ints(Words1, MaxParamLength, &value, 1);
```

`grep -c 'list_read_ints('` on the translation is 2 — the definition and that
call. So "the caller always passes `n == 1`" and "the record is always
`Words(1)`" are claims about the PROGRAM and not about the corpus, which is the
condition unit #56 recorded after four of its declarations rested on a premise
that had a second call site nobody had looked for.

---

## Family 1 — IOSTAT: the reference reads the status only through `/= 0` (6)

`33e905e4` `c7d7da2a` `9b309eac` `715eaff6` `972ae933` `27b68fda`

```fortran
READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
IF ( ErrStatLcl /= 0 )  THEN            ! <- the ONLY use
```

`ErrStatLcl` is a local of the reference, is written by the READ and is read by
exactly one expression, `/= 0`. It does not cross the signature and it is not
compared by anything. So a mutant that turns one NON-ZERO status into another
non-zero status is invisible to every layer, and to the program.

| id | site | edit | still non-zero? |
|---|---|---|---|
| `33e905e4` | `list_read_ints:364` | `5010` → `5011` | yes |
| `c7d7da2a` | `list_read_ints:361` | `5010` → `5011` | yes |
| `9b309eac` | `list_read_ints:343` | `5010` → `5011` | yes |
| `715eaff6` | `list_read_ints:312` | `5010` → `5011` | yes |
| `972ae933` | `list_read_ints:289` | `return -1` → `return + 1` | yes |
| `27b68fda` | `list_read_ints:289` | `return -1` → `return -2` | yes |

**THE SIBLING ON THE SAME SITE THAT IS NOT DECLARED** is `6a04d0f7`, the `0` of
`return 0;` at `:293` — the `/` terminator's SUCCESS. Its `const_tweak`
`0` → `1` turns a zero status into a non-zero one, which the reference's own
`/= 0` sees immediately, and the sweep **KILLED** it. That is the proof that
this family is a family and not a bucket: the discriminator is
zero-versus-non-zero, and the corpus can see it.

## Family 2 — `n == 1`: everything after the single item's transfer is dead (13)

`7d03c743` `461972fc` `a2131605` `f665d4be` `f8d17fd5`
`5a181633` `938cc91c` `0902d9ce` `f1bbee38`
`6713e5fa` `ed5b251a` `10e4743a` `e589ae82`

`eat_separator` is a lambda that writes only `p`, `sep` and its own local
`crossed_eol`. It is called from exactly two places, `:354` and `:369`, and
BOTH are immediately followed by the loop test `while (i < n)`:

```cpp
if (p >= len || is_value_terminator(rec[p]) || rec[p] == ';') {
    for (long k = 0; k < repeat && i < n; ++k) { ++i; }   // repeat >= 1 -> i = 1
    eat_separator();
    continue;                                             // -> while (i < n) is FALSE
}
...
for (long k = 0; k < repeat && i < n; ++k) { v[i++] = value; }   // -> i = 1
eat_separator();
}                                                          // -> while (i < n) is FALSE
```

With `n == 1` the item count is already `n` at both call sites (`repeat >= 1`
because `count <= 0` returns 5010 above), so the loop exits and **neither `p`
nor `sep` is ever read again**. No edit inside the lambda can reach an output.

The twelve lambda-body mutants are `p < len` → `p <= len` and
`rec[p]` → `rec[p + 1]` at `:270`, `:271`, `:276` and `:278`, plus the two
`negate_cond` edits at `:271` and `:276`, plus `'=='` → `'!='` at `:276`.

Reads stay in bounds: `p <= len == 200` and the buffer is the 400-byte `Words`
object, so `rec[p + 1]` is at worst `Words(2)(1:1)`. The sanitised sweep
confirms it — none of the twelve aborts.

`e589ae82` (`long repeat = 1` → `2`) is the same argument from the other side:
`repeat` is used only as the bound of those two `k < repeat && i < n` loops, and
`i < n` caps them at one iteration whatever `repeat` is.

**THE SIBLING NOT DECLARED**: `08ce3b56`, the `k = 0` → `k = 1` of the null-fill
loop at `:351`, is inside the same block and is **KILLED**. It changes how many
times `++i` runs, which decides whether the item is consumed — an output. The
family is about what happens AFTER the count reaches `n`, not about the count.

## Family 3 — `GetWords`' separator set: no byte of `Words` is ever `,` or `;` (3)

`7b2897c8` `400277bd` `27a6fe82`

```fortran
NextWhite = SCAN( Line(Ch+1:) , ' ,!;''"'//Tab )
Words(IW) = Line(Ch+1:Ch+NextWhite-1)
```

`GetWords` blank-fills all `NumWords` elements and then copies runs that contain
none of space, comma, `!`, semicolon, apostrophe, quote or tab. The translation
hands it the whole 400-byte buffer, so **no byte of `rec[0..399]` is ever a comma
or a semicolon** — including byte 200, which is `Words(2)(1:1)`.

| id | site | edit |
|---|---|---|
| `7b2897c8` | `:296` | `rec[p] == ','` → `rec[p + 1] == ','` |
| `400277bd` | `:307` | `rec[p] == ';'` → `rec[p + 1] == ';'` |
| `27a6fe82` | `:350` | `rec[p] == ';'` → `rec[p + 1] == ';'` |

Both spellings compare a byte that is never the character tested, so both take
the same branch. Reads are in bounds (`p <= 199` at those lines).

**MEASURED, not argued**: the record search distinguishes all three, and every
distinguishing record it found is tagged `FUNCONLY` —
`FUNCONLY:sep|0,0|` for `7b2897c8` and `FUNCONLY|full:repsemi|` /
`FUNCONLY|head:semifirst|` for the other two. Records admissible to the FUNCTION
and not to the UNIT. That is the shape unit #56 misread twice, and the tag exists
because of it.

**THE SIBLINGS NOT DECLARED, and there are four of them.** Each of `:296` and
`:307` carries a `negate_cond` and a `compare_op` beside the `index_offset`
declared above, and all four were **KILLED**:

    b4cd7967  :296  if (rec[p] == ',')  -> if (!(rec[p] == ','))   KILLED
    f124baa3  :296  '=='                -> '!='                    KILLED
    3bd0f136  :307  if (rec[p] == ';')  -> if (!(rec[p] == ';'))   KILLED
    f9e3e074  :307  '=='                -> '!='                    KILLED

So both lines are EXECUTED and both branches are observable — the claim here is
about the VALUE at the site (the byte read is never a comma or a semicolon
whichever index is used), not about reachability. `27a6fe82`'s own line, `:350`,
carries no such sibling -- its `rec[p]` occurrences are split between
`is_value_terminator(rec[p])` and `rec[p] == ';'` and only one mutant is offered
-- so the nearest control for it is `b6d16b4d` at `:332`, `rec[q]` -> `rec[q + 1]`
inside the repeat lookahead, which is **KILLED**. The repeat block IS entered and
its byte reads ARE observable.

## Family 4 — a value that is never read, or a buffer byte that is never touched (5)

| id | site | edit | why |
|---|---|---|---|
| `f45f53d4` | `:359` | `int32_t value = 0` → `1` | `value` is passed by reference to `parse_int`, which writes it on success and leaves it untouched on failure — and on failure the caller `return 5010`s before `v[i] = value`. The initialiser is never read |
| `965f5b6e` | `:138` | `char buf[Int2LStrLen]` → `[Int2LStrLen + 1]` | `int2lstr_c` writes exactly 11 bytes and `ftrim(buf, Int2LStrLen)` reads exactly 11. The twelfth byte is written by nothing and read by nothing |
| `1bd7a9a4` | `:423` | `char tmp[32]` → `[32 + 1]` | `snprintf(tmp, sizeof tmp, "%d", int32_t)` writes at most 12 bytes; the array is a scratch buffer with no other reader |
| `9ea1d982` | `:423` | `char tmp[32]` → `[33]` | the same site by the other operator, and the same argument — including `sizeof tmp`, which moves with the declaration |
| `9a30c578` | `:525` | `if (DEBUG_PARSING)` → `if (!(DEBUG_PARSING))` | `DEBUG_PARSING` is `constexpr bool = false` and the block it guards is EMPTY, so both spellings execute nothing |

`9a30c578` is the one this campaign would normally answer by DELETING the
restatement rather than declaring it (unit #32's `LEN_TRIM`, unit #55's
`negate_cond 9381bdef`). It is kept because the reference has the block —
`IF (DEBUG_PARSING) THEN ... END IF` at `ROSCO_Helpers.f90:151-153` — and P7
says the oracle is the original source. The empty `if` is the transcription of a
statement that exists; the sibling `parseinput_dbl_opt.cpp` carries the same one.

## Family 5 — an `INTENT(OUT)` dummy, or an OPTIONAL that is absent (3)

| id | site | edit | why |
|---|---|---|---|
| `5db2df45` | `:474` | `int32_t FoundLine_l = 0` → `1` | `FoundLine` is `LOGICAL, INTENT(OUT)` in `FindLine`, which assigns `FoundLine = .FALSE.` unconditionally before its search loop (`ROSCO_Helpers.f90:1109`). The initialiser is dead |
| `d841f125` | `:475` | `int CurLine = 0` → `1` | `LineNum` is `INTEGER(IntKi), INTENT(OUT)` in the same callee and is assigned `LineNum = 0` on the line below it (`:1110`). The initialiser is dead |
| `22d97f58` | `:477` | the SECOND `0` of `..., &CurLine, 0, 0)` → `1` | the two trailing arguments are `has_AryLen` and `AryLen`. This edit is on `AryLen`, and the generated bridge reads it only under `IF (has_AryLen /= 0)` (`parseinput_int_opt_callees.f90:63`), which is FALSE here because the reference calls `FindLine` without its OPTIONAL argument |

The column is checked rather than assumed: the two zeros are at columns 52 and
55 of that line, and `22d97f58`'s edit is at column 55.

**THE SIBLING NOT DECLARED** is `ac5c80e4`, the FIRST zero on `:477` (column 52),
`has_AryLen` itself. Setting it to 1 makes `FindLine` compare word
`AryLen + 1 = 1` instead of word 2 — a different search entirely — and the sweep
**KILLED** it. The two zeros are three characters apart and only one of them is
dead.

## Family 6 — the overflow clamp: `count_over` alone decides (3)

`ad55bdab` `ef24c75c` `9fd036b6`

```cpp
for (int k = p; k < q; ++k) {
    count = count * 10 + (rec[k] - '0');
    if (count > MaxRepeat) { count_over = true; count = MaxRepeat + 1; }
}
if (count <= 0 || count_over || count > MaxRepeat) { return 5010; }
```

The assignment `count = MaxRepeat + 1` exists only to stop `count` overflowing
`long` over a 200-digit run; the VERDICT is carried by `count_over`, which is
sticky and is a disjunct of the test. So `MaxRepeat + 2` (`ad55bdab`) and
`MaxRepeat - 1` (`ef24c75c`) both reach the same `return 5010`, and neither can
overflow `long` (each iteration re-clamps).

`9fd036b6` is the same shape one function over: `if (acc > 4294967296LL)` →
`>=` sets `overflow` one value earlier. At `acc == 4294967296` exactly, the
original leaves `overflow` false and then rejects on
`v > 2147483647LL`; the mutant rejects on `overflow`. Both `return false` and
neither writes `out`. Any `acc` that reaches 4294967296 is already outside
`INTEGER(4)`, so the two tests cannot disagree about the VERDICT.

**THE SIBLINGS NOT DECLARED**: the `2147483647LL` and `-2147483648LL` bounds on
`:240` were both open survivors until R14 gained the two limit words, and the
corpus then **KILLED** both (`cb35366f` at column 22, `9e4b76b0` at column 42),
along with the `negate_cond` of the whole test (`9aace307`). Those are the comparisons
that decide; this family is the bookkeeping around them.

## Family 7 — a boundary the record can reach but a digit cannot (4)

`94933624` `31376350` `26bfb57b` `75fadd11`

Each of these widens a bound by one at a place where the extra byte cannot change
the answer, and each is argued from what the code does with `p` afterwards rather
than from what the corpus contains.

| id | site | edit | why |
|---|---|---|---|
| `94933624` | `:332` | `q > p` → `q >= p` | `q == p` means no digits before the `*`. The original skips the repeat block and `parse_int` then finds no digits and returns false → `5010`. The mutant enters the block, accumulates over an EMPTY digit range → `count == 0` → `count <= 0` → `5010`. Same status, and neither writes the item |
| `31376350` | `:215` | `p < len` → `p <= len` | at `p == len` the mutant may consume a sign, making `p == len + 1`; the digit loop's own `p < len` is then false, so `digits == 0` and `parse_int` returns false exactly as the original does. A sign it consumes can never be followed by a digit it can read |
| `26bfb57b` | `:261` | `p < len` → `p <= len` | the leading-blank scan. Reaching `p == len` requires all 200 bytes blank or end-of-line; the mutant may then advance to `p == len + 1` and stops (`201 <= 200` is false). The only consumer is `if (p >= len) return -1` on the next line, and `200 >= 200` and `201 >= 200` are both true |
| `75fadd11` | `:329` | `q < len` → `q <= len` | the repeat-count lookahead's digit scan. At `q == len` the mutant may advance to `q == len + 1`; the very next line tests `q < len`, which is false for both 200 and 201, so the repeat block is skipped either way and `q` is not read again |

**THE SIBLING NOT DECLARED**, and it is on the same line as the last one:
`ab7420d6` (`:332`, `q < len` → `q <= len`) is **OPEN**, not declared. It reads
`rec[q]` at `q == len`, so unlike `75fadd11` its extra byte is USED — and
`evidence/ParseInput_Int_Opt/mutation_survivors.txt` names the record that would
distinguish it. Two `q < len` → `q <= len` mutants, one line apart, and only one
of them is equivalent.
