# FindLine — unit #32

```fortran
SUBROUTINE FindLine(FileLines, ParamName, FoundLine, Line, LineNum, AryLen)
    CHARACTER(*),   INTENT(IN), DIMENSION(:) :: FileLines
    CHARACTER(*),   INTENT(IN)               :: ParamName
    LOGICAL,        INTENT(OUT)              :: FoundLine
    CHARACTER(MaxLineLength), INTENT(OUT)    :: Line
    INTEGER(IntKi), INTENT(OUT)              :: LineNum
    INTEGER(IntKi), INTENT(IN), OPTIONAL     :: AryLen
```

Search a whole input file, held as an array of lines, for the line whose
`AryLen + 1`-th word (or second word, when `AryLen` is absent) is a given
parameter name. Two callees, `Conv2UC` and `GetWords`, both reached through
their bridges; nothing inlined.

**Disposition: `deferred`.** Every layer that ran is green and every green is
red-tested, and the mutation score is an honest **0.760** against a threshold of
1.000. The six survivors are named below and five of them are one gap.

## The layers

| layer | result | red-tested |
|---|---|---|
| kernel replay, 20 cases (`kernel.verify_fields.csv`) | 20/20, all 60 fields `IDENTICAL` | four stubs: 0/20, 0/20, **20/20**, 5/20 |
| differential harness (`harness/FindLine.json`) | **2370 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | six stubs: 2367 / 47 / 20 / 47 / 0 / 0 |
| mutation (`mutation/FindLine.json`) | **19 of 25 behavioural, 0.760**, 2 declared equivalent, 0 no-compile | the score *is* the red test, 25 times |
| gate, 27 scenarios (`gate/FindLine.json`) | 5,252,000 values / 351 channels, 0 mismatched | the match disabled moves **1,857,893**, revert-verified 0 |
| post-integration (`harness/FindLine.postintegration.json`) | 2370 checked, 0 failed | the wrapper's two extents transposed: **47 of 2370**, revert-verified 0 |

## What the kernel cannot see, measured rather than argued

`vit verify` printed `NON_DISCRIMINATING` and declined to build a red test —
this unit has no by-value floating-point argument. So four stubs were run in its
place (`run_kernel_stub.sh`, `kernel.*-stub.verify_fields.csv`):

```
no-op                      0/20   the comparison is alive
constant, never found      0/20   60 of 60 fields OUT_TOL
FIRST match wins          20/20   IDENTICAL -- the kernel is blind to it
AryLen ignored             5/20
```

`foundline` is `T` in all 20 reference cases, so the not-found path is outside
the window entirely; against the no-op stub 19 of 20 `foundline` comparisons
still come back `IDENTICAL`. **The unit's one invertible detail — the loop does
not exit on a match, so the LAST occurrence wins — is invisible to the kernel
and visible to the harness (47 of 2370).**

## The stated pin, and the reference's own exit status

`harness/ranges.toml` holds `AryLen` at `{ lo = 0, hi = 32 }`. `WordInd =
AryLen + 1` sizes `ALLOCATE(Words(WordInd))`, and at `WordInd <= 0` the
reference does not merely differ — it dies. Thirteen values, one process each,
against the clean Fortran (`arylen_probe.{f90,sh,txt}`):

```
-2147483648   SIGSEGV in __rosco_helpers_MOD_findline
-100 / -2 / -1   SIGABRT, malloc(): corrupted top size
0             returns
1 .. 100000   returns
2147483647    SIGSEGV -- AryLen + 1 overflows
```

The low bound is the boundary itself, not a margin: `AryLen = 0` is defined and
is reachable in the shipped program, since both call sites guard
`IF (AryLen < 1)` immediately *after* the call. The upper bound of 32 is a
narrowing and is stated as one: the reference is defined to at least 100000.

## THE ONE FACT THAT EXPLAINS FIVE OF THE SIX SURVIVORS

**Only 47 of 2370 cases reach the match arm.** Measured, not inferred:
`findline.match-count-probe.cpp` writes `LineNum = -7` inside the arm, so the
harness's failing count *is* the number of cases in which the reference finds
the name at least once (`harness.match-count-probe.json`, 47 of 2370).

In the other 2323 cases `FoundLine` is false, `LineNum` is 0 and `Line` is never
written — so nothing downstream of the comparison can be distinguished. The
corpus draws `FileLines` and `ParamName` independently, and a match then happens
only where two independently drawn strings coincide. That is unit #30's rule
seen from the other side: *a rule aimed at a predicate must supply BOTH sides of
it*, and no rule here supplies both sides of `FileLineUC == ParamNameUC`.

The two stubs that fail NOTHING say the same thing from two directions:

```
AryLen ignored (WordInd pinned at 2)     0 of 2370
Conv2UC on the search key removed        0 of 2370
```

**What would close it** — stated so the next dispatch does not have to rederive
it — is a corpus rule that plants one input inside another: for a unit that
compares a CHARACTER quantity derived from an array input against a scalar
CHARACTER input, generate cases in which the scalar IS the *k*-th word of a
chosen element, for each *k* the signature admits, and in which the case letters
differ. It is not taken here because a new rule shifts every already-scored
unit's draws and that X3 cost has to be measured across the campaign, which is a
dispatch of its own (unit #30 spent one exactly this way).

## The six survivors

| id | mutant | why it survives |
|---|---|---|
| `ca75abea` | `'2048' -> '2049'` (`MaxLineLength`) | **Undefined, not equivalent.** `char_assign(Line, 2049, …)` writes one byte past the caller's 2048-byte buffer. The bytes it writes past the end are not a wrong answer, so no value comparison can see it — and unit #7 settled that a mutant observable only through a write outside its buffer cannot honestly be declared equivalent. Unlike `char_assign`'s old `min`, this site cannot be deleted: VIT emits no `len_Line`, so the width has to be stated in the C++ exactly once, and it is. |
| `6ca91bba` | `'200' -> '201'` (`MaxParamLength`) | Corpus. The constant scales every 200-site together — both locals, the `Words` stride, the width handed to `getwords_c`, and the comparison length — so the mutant is a consistent program that differs only where a *word* or the *key* reaches 200 characters. No case supplies one. |
| `d76903ed` | `'2' -> '3'` (`WordInd = 2`, the `.NOT. PRESENT` arm) | The 47-case gap above. |
| `3bc7aeba` / `b2c89b46` | two `'1' -> '2'` | Identified by building all seven candidate sites and running each (`const_tweak_probes/RESULTS.md`): `WordInd = AryLen + 1` -> `+ 2`, and the search loop starting at line 2. The first is the 47-case gap; the second changes an answer only when a case's *only* match is on line 1, and none of the 47 is. |
| `d126d74e` | `negate_cond` on `if (!has_AryLen)` | The 47-case gap — it swaps `WordInd = 2` for `AryLen + 1`. |

Two more were declared equivalent (`mutation/FindLine.equivalences.json`), and
the undeclared run is committed beside them at 0.7037
(`mutation/FindLine.undeclared.json`) so what survived is on the record before
any of it was excused.

## The canary probe, and the control that took two attempts

Both declared mutants enlarge a fixed-width local by one byte. The claim is that
the extra byte is never read or written. It is provable by inspection — every
index expression is bounded by `MaxParamLength` — and this campaign checks
readings, so it was also measured: `findline.canary-probe.cpp` declares both
arrays one longer, puts `\x7f` in the extra byte of each and sets `LineNum = -7`
if either is disturbed. **0 of 2370.**

A zero from a probe is worth nothing until the probe is shown able to be
non-zero (P10), and the first control **was a no-op**: `conv2uc_c(ParamNameUC,
MaxParamLength + 1)` reads the extra byte and writes it only if it is a
lowercase letter, and `\x7f` is not one. It reported 0 of 2370 — the same number
as the probe, and for one run it looked exactly like a pass. The control now
writes the byte unconditionally and fails **2370 of 2370**
(`harness.canary-control.json`).

## Three tool defects, each fixed where it lives (X2)

This unit is the campaign's first with an **assumed-shape CHARACTER array**
dummy, and it broke three things that had never been asked the question.

1. **`vit check` scoped its Fortran-reading checks to the FILE, not the
   procedure** (vit `c4eb0ad`). `delimiter-set` reported `FindLine` — which
   contains no `SCAN`, `INDEX` or `VERIFY` at all — as missing the backslash
   from `GetPath`'s `INDEX( GivenFil, '\', BACK=.TRUE. )`, 900 lines away.
   Latent for eight units in that one file. Red-tested both ways in
   `vit_check_scope_redtest.{sh,txt}`, including a probe that refuses when its
   own perturbation fails to land.

2. **VIT's two generators disagreed about extent ORDER** (vit `d2de28c`).
   `build_c_params` emits `char* X, int n_X, int len_X`; the test-validate
   bridge emitted `len_X` then `n_X`. C linkage checks nothing, so the
   transposition arrived as a *wrong answer*: 47 of 2358 cases disagreed, with
   the reference's `LineNum` equal to `len_FileLines` in every one — `DO I = 1,
   SIZE(FileLines)` iterating over the element WIDTH. A 4×3 array read as 3×4 is
   well-formed, so **2311 cases still agreed**. The generator now asks
   `build_c_params` and refuses when the two disagree.

3. **The loop's mapper refused the shape with a reason that was false**
   (loop `9eeaf3f`) — "carries its extent in a descriptor `build_c_params` does
   not emit", when it emits it under the ordinary `n_` name. Two more in the
   same commit: `int32_t*` was looked up as a derived-type view struct because
   `int32_t` ends in `_t`, and `CHARACTER(MaxLineLength)` — this unit's
   principal output — was `UNOBSERVABLE`, neither supplied nor compared, because
   the width is a module PARAMETER rather than a literal.

   R12_narrowing_width had the same literal-only blind spot (loop `024982b`) and
   reported "the reference truncates nothing" about a unit that truncates twice.

**The R12 widening on its own killed nothing** — 0.667 before and after, +12
cases — and became three kills only once the translation stopped computing its
truncation through a `std::min` whose mutants write past a buffer instead of
changing an answer. A rule that puts a boundary in the corpus and a translation
with no site at which that boundary changes an answer are two halves of one
measurement.

## A red test that corroborates another unit's

Disabling FindLine's name comparison moves **1,857,893 of 5,252,000 values
across 147 channels** — the same three figures as `gate/GetWords.redtest.json`.
Not a coincidence: GetWords' red test blanks every word, so FindLine then
compares a blank word against a non-blank name and matches nothing. Two
perturbations of two different units reaching one state by different routes,
which is what a same-build control buys, taken here for free.

## Files

```
findline.final.cpp                       the translation as committed
findline_interface.f90, findline_wrapper.f90   what `vit interface` generates
vit_interface.stdout.txt, vit_check.stdout.txt
kernel.verify_fields.csv                 20 cases, 60 fields, all IDENTICAL
kernel.{noop,constant,first-match,arylen-ignored}-stub.verify_fields.csv
findline.{noop,constant,first-match,arylen-ignored}-stub.cpp
run_kernel_stub.sh                       one stub through `vit verify`
run_harness_stub.sh                      one stub through the differential harness
run_wrapper_redtest.sh                   plants a defect in the SHIPPED wrapper
harness.{noop,first-match,arylen-ignored,no-line,no-uppercase,no-truncation}-stub.json
findline.harness-*-stub.cpp
harness.match-count-probe.json           47 of 2370 reach the match arm
harness.canary-probe.json                0 of 2370   the extra byte is untouched
harness.canary-control.json              2370 of 2370   and the probe can fire
harness.postintegration.revert-verified.json
arylen_probe.{f90,sh,txt}                what the reference does per AryLen
const_tweak_probes/                      which `'1' -> '2'` sites are visible
vit_check_scope_redtest.{sh,txt}         the `vit check` fix, both directions
```
