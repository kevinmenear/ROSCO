# `ParseInput_Str_Opt` — declared EQUIVALENCES, and the sibling on each site that is not declared

`mutation/ParseInput_Str_Opt.equivalences.json` carries the ids;
`vit_mutate.py --equivalences` reads them and a declared mutant LEAVES THE
DENOMINATOR, so every entry here is a claim somebody has to be able to answer.
The undeclared runs are committed beside it —
`evidence/ParseInput_Str_Opt/sweep_probe0.json` (52 mutants, 38 killed, 0.731,
before the `char_assign` repair) and `sweep_probe1.json` (50 mutants, 39 killed,
0.780, after it) — so what survived is on the record before any of it was
excused.

**FIVE equivalences, and every one of them has a NAMED SIBLING ON THE SAME SITE
OR THE SAME LINE THAT IS NOT DECLARED AND THAT THE CORPUS KILLED.** Each sibling
id below was checked against the scored artifact rather than asserted.

**AND THREE MUTANTS THAT WERE NOT DECLARED AT ALL — THEY WERE DELETED.** The
first sweep left `8b83f1b6` (`min(len_dst, len_src)` → `min(len_src, len_dst)`),
`dac6e85c` (`if (n > 0)` → `if (n >= 0)`) and `1db4dd8f` (`if (len_dst > n)` →
`>=`) alive, all three behaviour-preserving, all three declarable. They are not
declared, because unit #32's `findline.cpp` already carries a comment naming
exactly these three at `GetWords`' version of the same expression, together with
the repair: **one loop bounded by the destination**. `char_assign` was rewritten
to that form (P4, byte for byte), the three sites no longer exist, and the one
predicate the loop does have — `i <= len_src` — changes an ANSWER at the
truncation boundary this unit's corpus straddles on both sides. Deleting a site
that admits an undeclarable mutant is unit #7's rule and this campaign's
preference over an argument.

---

## Family 1 — an `INTENT(OUT)` dummy of the callee, or an OPTIONAL that is absent (3)

| id | site | edit | why |
|---|---|---|---|
| `5247dc31` | `:316` | `int32_t FoundLine_l = 0` → `1` | `FoundLine` is `LOGICAL, INTENT(OUT)` in `FindLine`, which assigns `FoundLine = .FALSE.` unconditionally before its search loop (`ROSCO_Helpers.f90:1109`, clean `54dd134`). The initialiser is dead |
| `9c1c4ec7` | `:317` | `int CurLine = 0` → `1` | `LineNum` is `INTEGER(IntKi), INTENT(OUT)` in the same callee and is assigned `LineNum = 0` on the line below it (`:1110`). The initialiser is dead |
| `920580ba` | `:319` | the SECOND `0` of `..., &CurLine, 0, 0)` → `1` | the two trailing arguments are `has_AryLen` and `AryLen`. This edit is on `AryLen`, and the generated bridge reads it only under `IF (has_AryLen /= 0)`, which is FALSE here because the reference calls `FindLine` without its OPTIONAL argument (`ROSCO_Helpers.f90:308`, clean) |

The column is checked rather than assumed: the two zeros on `:319` are at
columns 52 and 55, and `920580ba`'s edit is at column 55.

**THE SIBLING NOT DECLARED** is `af4a0269`, the FIRST zero on `:319` (column 52),
`has_AryLen` itself. Setting it to 1 makes `FindLine` take
`WordInd = AryLen + 1 = 1` and compare word ONE instead of word TWO — a
different search entirely — and the sweep **KILLED** it. The two zeros are three
characters apart and only one of them is dead. This is unit #57's Family 5 with
the same three ids' worth of reasoning and the same control; the arguments are
re-derived against this unit's own line numbers rather than carried over.

## Family 2 — a buffer that is enlarged and never indexed past its original end (1)

| id | site | edit | why |
|---|---|---|---|
| `1a28cd30` | `:331` | `std::vector<char> Words(2 * MaxParamLength)` → `3 *` | the allocation grows from 400 bytes to 600. The only writer is `getwords_c(Line.data(), MaxLineLength, Words.data(), MaxParamLength, 2)`, which is told the element width (200) and the number of words (2) SEPARATELY from the allocation, so it writes exactly 400 bytes; the only readers are `Words1 = Words.data()` and `Words2 = Words.data() + MaxParamLength`, both bounded by 200 in every expression that consumes them. Bytes 401..600 are written by nothing and read by nothing. Strictly SAFER than the original — the mutant allocates more than the program touches — which is the distinction unit #7 drew and unit #32 relied on for `b668005d` |

**THE SIBLING NOT DECLARED** is `3c51517a`, the OTHER `2` → `3`, at `:333`
column 73 — the `NumWords` argument of `getwords_c`. That one asks the callee for
three words out of a two-element buffer, and the sweep **KILLED** it. The two
`2`s are two lines apart and only one of them is a pure allocation.

## Family 3 — a guard whose block is empty (1)

| id | site | edit | why |
|---|---|---|---|
| `ef22930a` | `:369` | `if (DEBUG_PARSING)` → `if (!(DEBUG_PARSING))` | `DEBUG_PARSING` is `constexpr bool = false` and the block it guards is EMPTY, so both spellings execute nothing |

This is the one this campaign would normally answer by DELETING the restatement
rather than declaring it (unit #32's `LEN_TRIM`, unit #55's
`negate_cond 9381bdef`). It is kept for the reason units #56 and #57 kept theirs:
the reference has the block — `IF (DEBUG_PARSING) THEN ... END IF` at
`ROSCO_Helpers.f90:329-331` (clean) — and P7 says the oracle is the original
source. The empty `if` is the transcription of a statement that exists, and all
three siblings in this family carry the same one.

---

## What is NOT here

The six `unreachable` declarations are in
`mutation/ParseInput_Str_Opt.unreachable.json`, with their reasons and evidence
paths, and they are a different claim: `unreachable` says the CORPUS never
reaches the site, `equivalent` says the two PROGRAMS agree. `vit_mutate.py`
refuses a mutant carrying both. Four of the six sit in or feed this unit's
READ-error arm, which is dead in the PROGRAM rather than merely unexercised — a
stronger fact than `unreachable` needs, stated in the reasons because a reader
who only sees the coverage file would take it for a corpus gap that a wider
corpus could close. It cannot.
