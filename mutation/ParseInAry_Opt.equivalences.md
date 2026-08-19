# `ParseInAry_Opt` — declared EQUIVALENT, seventeen mutants, one argument each

An equivalence is a claim about the two **PROGRAMS**: the mutated one and the
shipped one agree on every admissible input, so no corpus can ever tell them
apart and the mutant belongs out of the denominator rather than in the survivor
list. It is the stronger of this campaign's two declarations and it survives a
corpus that later widens, which is why a mutant argued here is deliberately
**not** re-declared `unreachable` even when it sits on a line the corpus never
runs (`evidence/ParseInAry_Opt/make_unreachable.py` skips it, and
`vit_mutate.py` refuses a mutant carrying both claims).

**Every entry ends in the grep that would refute it.** A file that cannot be
disputed is not evidence. And the set is checkable a second way, which is the
sibling's rule (`RUNBOOK`, unit #54): **three lines here carry a mutant that is
declared beside one that is not**, and the difference is the whole argument.

```
L 276  `return -1` -> `return + 1`   DECLARED   nonzero IOSTAT, one reader, `!= 0`
       `return -1` -> `1 - return`   NOT        it does not COMPILE
L 358  `return 0`  -> `return 1`     NOT        this one is the SUCCESS value, and
                                                the single reader tests `!= 0`
L 217  `acc > 4294967296LL` -> `>=`  DECLARED   a stop-ACCUMULATING guard; the
                                                validity test is L227
L 227  `v > 2147483647LL`   -> `>=`  NOT        the validity test itself, and the
                                                corpus KILLS it on one record
L 319  `q > p`     -> `q >= p`       DECLARED   `q == p` means no digits, and both
                                                spellings end in the same 5010
       `q < len`   -> `q <= len`     NOT        it reads `rec[len]`; the sanitiser
                                                kills it
L 326  `MaxRepeat + 1` -> `- 1`      DECLARED   only reached with `count_over`
                                                already set, which decides L329
L 329  `count > MaxRepeat` -> `>=`   NOT        killed on 18 cases by `200000000*7`
L 443  `: 0` (ary_size)  -> `: 1`    DECLARED   the arm needs a FAILED CFI_allocate
       `d->dim[0]` -> `d->dim[0 + 1]`  NOT      the sanitiser kills it
```

The four `L 587` rows the third dispatch listed here are gone: that site --
`std::max(NumWords, 0)` -- was DELETED from the translation at the fourth
dispatch rather than argued about, together with `field()`'s `*` overflow fill
and the empty `if (DEBUG_PARSING) {}`. A restatement no input can make fire is
removed, not declared.

`26d402b2` on L358 is the one that keeps the IOSTAT argument honest: the claim
below is `nonzero -> nonzero under a `!= 0` reader`, not `any return value of
`list_read_ints` is unobservable`. Declaring it would have been the same
sentence applied one value too far, and the corpus can kill it.

---

## 1. A return value with exactly one reader, and the reader is `!= 0`

`list_read_ints` reproduces gfortran's own `IOSTAT` spelling — `0`, `5010`,
`-1` — because a model of the TEST is not a model of the READ and the three
sibling units in this family read the value too. **In THIS program the value has
one assignment and one read:**

```
607:        ErrStatLcl = list_read_ints(Line.data(), MaxLineLength, values.data(), n);
612:        if (ErrStatLcl != 0) {
```

so any edit that carries one **nonzero** value to another nonzero value is
invisible to every input. The value never reaches an output: `Ary`,
`ErrVar%aviFAIL`, `ErrVar%ErrStat` and `ErrVar%ErrMsg` are all written from the
BRANCH, not from the number.

| id | line | edit |
|---|---|---|
| `56160d94` | 299 | `return 5010` → `5011` — the semicolon guard |
| `6b5c6a0f` | 330 | `return 5010` → `5011` — the repeat-count ceiling |
| `24e73513` | 348 | `return 5010` → `5011` — `parse_int` failed |
| `3683a56a` | 351 | `return 5010` → `5011` — a bad terminator |
| `80184bd0` | 276 | `return -1` → `return + 1` — the record ran out |
| `c8a37018` | 276 | `return -1` → `return -2` — the same statement |

**REFUTED BY:** `grep -n ErrStatLcl translations/ROSCO_Helpers/parseinary_opt.cpp`
showing a second read of the value, or any read that is not `!= 0`. It reports
lines 504, 507, 511, 513 (the ALLOCATE status, a different variable's life) and
607/612 (this one). Nothing else.

## 2. A local buffer nothing measures, and one that IS measured but cannot be reached

| id | line | edit |
|---|---|---|
| `b54ac6eb` | 114 | `char buf[Int2LStrLen]` → `[Int2LStrLen + 1]` |
| `34a3b929` | 407 | `char tmp[32]` → `char tmp[33]` |
| `f5386894` | 407 | `char tmp[32]` → `[32 + 1]` — the same program by another operator |

`buf` is filled by `int2lstr_c(Num, buf)`, which writes exactly `Int2LStrLen`
bytes (`Int2LStr`'s result is `CHARACTER(11)`, blank-padded, not
NUL-terminated), and it is read by `ftrim(buf, Int2LStrLen)`. **Both bounds are
the named constant; neither is `sizeof`.** Enlarging the allocation is therefore
behaviour-preserving — and this holds under `--sanitize` too, since a larger
allocation cannot turn a legal access illegal.

`tmp` is the one that **is** measured: `std::snprintf(tmp, sizeof tmp, "%d",
...)`. So the argument is not "nothing reads the size" but the FORMAT's maximum
width — `"%d"` of an `int32_t` is at most 11 bytes (`-2147483648`), against 32
and against 33. No value of the type can select either bound.

**REFUTED BY:** `grep -n 'sizeof buf\|sizeof(buf)' …` (none), or an `snprintf`
whose format could produce 32 bytes from an `int32_t`.

## 3. Dead initialisers

| id | line | edit |
|---|---|---|
| `4f1acf76` | 346 | `int32_t value = 0;` → `= 1` |

`value` is read at exactly one place, L354 `v[i++] = value;`, and the only path
to L354 runs through `if (!parse_int(rec, len, p, value)) return 5010;` —
`parse_int` assigns `out` on its single `return true` (L230) and on no other
path. So the initialiser is dead on the one path that reads the variable.

**THE SECOND ROW OF THIS TABLE IS GONE, AND ITS REMOVAL IS NOT A RETRACTION.**
`int32_t FoundLine_l = 0;` → `= 1` was declared here on the same argument —
`FoundLine_l` is passed by address to `findline_c` and `FindLine` writes
`FoundLine = .FALSE.` unconditionally before any statement that can return, so
the initialiser cannot be observed. The argument still holds. What changed is
that **the mutant no longer exists**: `cppmutate` caps `const_tweak` at 40 of
its 61 sites, the zero-length-`Ary` repair added a `'0' → '1'` site at
`if (n > 0)`, and that pushed `FoundLine_l`'s off the end of the sample. A
declaration whose mutant is not offered cannot be checked and is not carried.

**REFUTED BY:** a path to L354 that does not pass L347.

## 4. A symmetric call

| id | line | edit |
|---|---|---|
| `a61c0a99` | 587 | `std::max(NumWords, 0)` → `std::max(0, NumWords)` |

`std::max` on `int` returns one of two equal values when they are equal and the
larger otherwise; `int` has no way to tell two equal values apart. The
**undeclared** mutant on the same expression is what makes this checkable:
`b1c839d7` drops the call entirely (`std::max(NumWords, 0)` → `NumWords`), which
is equivalent only while `NumWords >= 0` is guaranteed — a fact about the
DOMAIN, not about the operator — and it is left alive.

**REFUTED BY:** a `std::max` overload here on a type with distinguishable equal
values (it is `int`), or a corpus that kills it — `declared_but_killed` in
`mutation/ParseInAry_Opt.json` is the check, and it is empty.

---

## What is NOT declared, and why the list stops here

* **`8464d14d`, `MaxParamLength` 200 → 201.** It is both the allocation width of
  `Words_Ary` and the `len_Words` the `GetWords` bridge is handed, so the two
  uses move together and the mutant may well be equivalent. "May well be" is not
  an argument, and a declaration this campaign cannot dispute is worth less than
  a survivor it can count. Left alive.
* **`b93bad70`, `MaxRepeat` 200000000 → 200000001.** Measured non-equivalent:
  `'199999999*7'` and `'200000000*7'` give `7 7` and `'200000001*7'` gives 5010
  (`evidence/ParseInAry_Opt/survivor_replay.txt`). The corpus does not reach it;
  that is an `unreachable` claim, not an equivalence, and it is not declared as
  either here.
* **The PRINT record.** Its mutants are measured MOVING by the sibling's
  instrument class, so they are the opposite of equivalent — they are differences
  the scoring oracle cannot see. Escalated, not declared.

---

## 6. A guard on the ACCUMULATOR is not the validity test, and only the validity test decides

| id | line | edit |
|---|---|---|
| `e6bc72cb` | 217 | `if (acc > 4294967296LL)` → `>=` |

`parse_int` stops accumulating once the magnitude can no longer matter, so that
a twenty-digit record cannot overflow `long long`:

```
209:    while (p < len && is_digit(rec[p])) {
212:        acc = acc * 10 + (rec[p] - '0');
217:        if (acc > 4294967296LL) { overflow = true; }
227:    if (overflow || v > 2147483647LL || v < -2147483648LL) { return false; }
```

The two spellings differ on exactly one state: `acc == 4294967296` after some
digit. Take it.

* If that digit is the LAST, `v` is `±4294967296`, which fails L227's range test
  in both programs — `> 2147483647` in the positive case, `< -2147483648` in the
  negative — and both return `false` having written nothing to `out`.
* If a digit FOLLOWS, the shipped program accumulates once more to
  `42949672960 + d`, which is `> 4294967296`, and sets `overflow` on the very
  next iteration; the mutant sets it one iteration earlier. Both leave the loop
  with `overflow == true`, the same `digits` count, the same `p`, and both
  return `false`.

So the mutant can only make the program reject sooner something it already
rejects. **`4294967296` is more than twice `2147483647`: no value that reaches
this guard at all can pass L227.**

**REFUTED BY:** a path on which `overflow` is read for anything other than
L227's `return false` (it is not: `grep -n overflow` gives four lines, all
inside `parse_int`), or a corpus that kills it. The mutant on the line below —
`51771298`, `v > 2147483647LL` → `>=`, the VALIDITY test — is **not** declared
and this dispatch's corpus **kills it on one case**, which is what makes the
distinction between the two lines a measurement rather than a preference.

---

## 7. A comparison whose only caller has already established it

| id | line | edit |
|---|---|---|
| `2f8001ae` | 202 | `if (p < len && (rec[p] == '+' || rec[p] == '-'))` → `p <= len` |

`parse_int` has exactly ONE call site, at line 347, and `p` is `p` on entry:

```
274:    while (i < n) {
275:        if (p >= len) { return -1; }          <- p < len from here on
...
333:            p = q + 1;                        <- the only assignment between
337:            if (p >= len || is_value_terminator(rec[p]) || rec[p] == ';') {
342:                continue;                     <- and it CONTINUES when p >= len
347:        if (!parse_int(rec, len, p, value)) {
```

Every path from L275 to L347 either `continue`s, `return`s, or leaves `p < len`:
the repeat-count block is the only code that advances `p`, and its own guard at
L337 sends `p >= len` back to the top of the loop. So `parse_int` is never
entered with `p == len`, the only state at which `p < len` and `p <= len`
differ, and `rec[len]` is never read by either program.

**REFUTED BY:** a second call site (`grep -n 'parse_int(' ` gives the definition
and one call), an assignment to `p` between L275 and L347 that this misses, or a
corpus that kills it. Note that this is a claim about the CALLER's control flow
and not about the operator, which is why the campaign's other `p < len` →
`p <= len` mutants are **not** declared: on this dispatch's corpus every one of
them is killed by a sanitiser abort at `rec[2048]`.

---

## 8. A guard whose two outcomes converge on the same return

| id | line | edit |
|---|---|---|
| `a4bdfbcc` | 319 | `if (q > p && q < len && rec[q] == '*')` → `q >= p` |

`q` starts at `p` and only advances over digits, so `q >= p` always holds and
the mutant's condition differs from the original's on exactly one state:
`q == p`, meaning **no digit at the start of the item**. In that state the
mutant enters the repeat-count block; the loop `for (int k = p; k < q; ++k)`
does not execute, `count` stays `0`, and L329's `count <= 0` returns `5010`
having stored nothing. The shipped program skips the block and calls
`parse_int`, which finds `digits == 0`, returns `false`, and L348 returns
`5010` having stored nothing.

**The two programs return the same value, write the same elements — none — and
leave `i` unchanged.** The third conjunct `rec[q] == '*'` narrows the state
further (the mutant only enters when the item starts with `*`) and does not
change the argument.

**REFUTED BY:** a path on which the repeat-count block's `count == 0` exit
differs from `parse_int`'s zero-digit exit (both are `return 5010` with no
store), or a corpus that kills it. The mutant beside it on the same line —
`q < len` → `q <= len` — is **not** declared, and the sanitiser kills it on the
`reptail` record where `q` reaches `len`.

---

## 9. A value written only after the decision that reads it has been made

| id | line | edit |
|---|---|---|
| `e905195c` | 326 | `count = MaxRepeat + 1` → `MaxRepeat - 1` |
| `5477861c` | 326 | the same statement, `+ 1` → `+ 2` |

```
322:        for (int k = p; k < q; ++k) {
323:            count = count * 10 + (rec[k] - '0');
324:            if (count > MaxRepeat) {
325:                count_over = true;
326:                count = MaxRepeat + 1;
327:            }
329:        if (count <= 0 || count_over || count > MaxRepeat) { return 5010; }
```

L326 executes only under L324, which sets `count_over` on the same pass and
never clears it. `count_over` is the **second** disjunct of L329, so from L326
onward the function returns `5010` whatever `count` holds — and `count` is read
nowhere else: `repeat = count` at L332 is downstream of that return. The clamp
exists to stop `count` from overflowing `long` on a long digit string, and any
value that keeps it bounded does the same job.

**REFUTED BY:** a read of `count` that is reachable with `count_over` true
(`grep -n 'count' ` gives L320–L332 and nothing after), a path that clears
`count_over`, or a corpus that kills it. The mutant on the line below —
`9120d215`, L329's `count > MaxRepeat` → `>=` — is **not** declared and is
killed on 18 cases by the `200000000*7` record, which is the exact boundary
this clamp is one side of.

---

## 10. A width that is the allocation AND the stride, moving together, into an object nothing reads

| id | line | edit |
|---|---|---|
| `8464d14d` | 60 | `MaxParamLength` `200` → `201` |

`MaxParamLength` has exactly two uses in the program, and they are the two
halves of one call:

```
607:        std::vector<char> Words_Ary(static_cast<std::size_t>(NumWords)
608:                                    * static_cast<std::size_t>(MaxParamLength));
612:        getwords_c(Line.data(), MaxLineLength, Words_Ary.data(), MaxParamLength, NumWords);
```

The constant is the element WIDTH of the buffer and the `len_Words` the callee
is told to use, so a mutant moves both: the vector is `NumWords * 201` bytes and
`GetWords` writes `NumWords` words of 201 bytes into it. There is no
out-of-bounds write, and **`Words_Ary` is read by nothing** — the reference
allocates it, passes it to `GetWords`, and reads it only inside the
`DEBUG_PARSING` block, which is a `constexpr false` and is not translated. No
output of this unit depends on the buffer's contents or its width.

**REFUTED BY:** a third use of `MaxParamLength` (`grep -n MaxParamLength` gives
the declaration, two comments and these two lines), a read of `Words_Ary`, or a
corpus that kills it. This is the mutant `FindLine` (unit #32) recorded as
surviving for the same reason one function over; here the argument is written
down rather than left as a survivor.

---

## 11. A ternary arm that needs a failure the input domain cannot produce

| id | line | edit |
|---|---|---|
| `73975430` | 453 | `return ary_allocated(d) ? d->dim[0].extent : 0;` → `: 1` |

`ary_size` has one call site:

```
528:    const int n = static_cast<int>(ary_size(Ary));
```

and it is downstream of the allocation block, which leaves `Ary` allocated on
every path an INPUT can select: either the caller supplied an allocated array
(`was_allocated`, the reachable ALLOCATE failure) or `CFI_allocate` succeeded.
The `: 0` arm is selected only when `CFI_allocate` FAILED and left `base_addr`
null — a genuine out-of-memory, which is a property of the machine and not of
any input in this unit's domain (`AryLen` is stated `[0, 32]`, so the request is
at most 33 four-byte elements).

This is deliberately filed as an EQUIVALENCE and not as `unreachable`: an
`unreachable` declaration is a claim about the CORPUS and is derived from
`line_coverage.txt`, and L443 *is* executed — 19,536 times. What is unreachable
is the ternary's false arm, which gcov's line granularity cannot report, so the
claim has to be made about the two programs and the admissible inputs instead.

**REFUTED BY:** a call to `ary_size` before the allocation block (`grep -n
ary_size` gives the definition and L528), an input that makes `CFI_allocate`
fail, or a corpus that kills it. The mutant beside it on the same line —
`d->dim[0]` → `d->dim[0 + 1]`, reading the descriptor's second dimension of a
rank-1 array — is **not** declared and the sanitiser kills it.

**AND THIS ROW'S ID CHANGED WITHOUT THE SITE CHANGING, WHICH IS WORTH THE
WARNING.** It read `6fb2c552` until the zero-length-`Ary` repair. `cppmutate`
identifies a mutant by an OCCURRENCE INDEX over identical edits, so inserting a
`'0' → '1'` site earlier in the file renumbers every later one — `6fb2c552` now
names `d->dim[0]`, the mutant this section says is **not** declared, and the
corpus kills it. `declared_but_killed` caught it. **Re-derive a declaration from
its SITE after any edit that adds or removes an identical edit; an enumeration
diff of the id SET will report "none lost" and be useless.**
