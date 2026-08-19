# `ParseInAry_Opt` — declared EQUIVALENT, twelve mutants, one argument each

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
L 587  `std::max(NumWords, 0)` swapped   DECLARED   std::max is symmetric on int
       `std::max(NumWords, 0)` dropped   NOT        holds only while NumWords >= 0,
                                                    which is a fact about the DOMAIN
```

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
| `e9786fd2` | 474 | `int32_t FoundLine_l = 0;` → `= 1` |

`value` is read at exactly one place, L354 `v[i++] = value;`, and the only path
to L354 runs through `if (!parse_int(rec, len, p, value)) return 5010;` —
`parse_int` assigns `out` on its single `return true` (L230) and on no other
path. So the initialiser is dead on the one path that reads the variable.

`FoundLine_l` is passed by address to `findline_c` at L477 and read at L478.
`FindLine` writes `FoundLine = .FALSE.` unconditionally before any other
statement that can return (`translations/ROSCO_Helpers/findline.cpp:170`, and
the clean Fortran it bridges to at `ROSCO_Helpers.f90`), so the initialiser
cannot be observed either.

**REFUTED BY:** a path to L354 that does not pass L347, or an early `return` in
`FindLine` before its `FoundLine = .FALSE.` — `sed -n '110,175p'
translations/ROSCO_Helpers/findline.cpp | grep -n 'return\|FoundLine'` reports
the assignment and no return above it.

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
