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
red-tested, and the mutation score is an honest **0.960** against a threshold of
1.000. Five of the six survivors this unit's first dispatch left alive were one
measured corpus gap, and R14 (`translation-loop` `552edb1`) closed it. The one
that is left is not a corpus gap and is not equivalent; it is named below and
escalated.

## The layers

| layer | result | red-tested |
|---|---|---|
| kernel replay, 20 cases (`kernel.verify_fields.csv`) | 20/20, all 60 fields `IDENTICAL` | four stubs: 0/20, 0/20, **20/20**, 5/20 |
| differential harness (`harness/FindLine.json`) | **2514 checked, 0 failed, 0 inadmissible** — this unit's primary evidence | seven stubs: 2511 / 47 / 62 / 89 / 30 / 42 / 60 |
| mutation (`mutation/FindLine.json`) | **24 of 25 behavioural, 0.960**, 2 declared equivalent, 0 no-compile | the score *is* the red test, 25 times |
| gate, 27 scenarios (`gate/FindLine.json`) | 5,252,000 values / 351 channels, 0 mismatched | the match disabled moves **1,857,893**, revert-verified 0 |
| post-integration (`harness/FindLine.postintegration.json`) | 2514 checked, 0 failed | the wrapper's two extents transposed: **88 of 2514**, revert-verified 0 |

The corpus was 2370 for this unit's first dispatch and the artifacts of that run
are superseded, not deleted from the record: every number below that reads
`of 2370` is the BEFORE half of R14's measurement and is labelled as such.

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

## THE GAP, MEASURED — AND CLOSED

The section below is the first dispatch's diagnosis, kept verbatim because it is
what the repair was built against. **R14 closed it**, and the same two probes
that measured the gap measure the repair:

```
                                        first dispatch      now
cases reaching the match arm            47 of 2370          89 of 2514
of those, matches on a BLANK key        47                  47
so matches on a NON-BLANK key            0                  42
```

The 42 are the whole of it, and the two stubs that reported the blindness as a
zero now report numbers:

```
AryLen ignored (WordInd pinned at 2)     0 of 2370   ->   30 of 2514
Conv2UC on the search key deleted        0 of 2370   ->   42 of 2514
```

That second number is exactly the non-blank match count, which is what it has to
be: every planted match is a case-INVERTED key against a mixed-case word, so
dropping the fold on either side destroys all 42 and nothing else.

Five mutants died with it, each on a different count — which is the evidence
that they were five distinct behaviours rather than one seen five ways:

| mutant | killed on |
|---|---|
| `'200' -> '201'` `MaxParamLength` | 14 of 2514 (the long-word plant, at R12's width) |
| `'2' -> '3'` `WordInd` on the `.NOT. PRESENT` arm | 48 |
| `'1' -> '2'` `WordInd = AryLen + 1` | 18 |
| `'1' -> '2'` the search loop starting at line 2 | 21 |
| `negate_cond` on `if (!has_AryLen)` | 72 |

R14's own X3 cost is in `x3_check_r14/`, measured in corpus BYTES rather than in
case counts.

### The diagnosis, as it was written

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

**AND ALL 47 ARE THE ALL-BLANK SHAPE — two empty strings comparing equal.** A
second probe settles it (`findline.nonblank-key-probe.cpp`,
`harness.nonblank-key-probe.json`): refusing the match unless the search key
holds a non-blank character changes the answer in **47 of 2370** cases, i.e. in
*every* case that matched. The key is one value per case, so a case whose key is
non-blank would have been left alone and would not have failed; none was.

That is the same thing unit #30 measured for `ChkParseData` — its arm 1 was
reached 52 times in 1284 cases and every one was the all-blank shape — and it
finishes the explanation of the survivors rather than merely bounding it:

* a BLANK word is blank at **every index**, so no blank match can distinguish
  `Words(2)` from `Words(AryLen + 1)`. That is `d76903ed`, `3bc7aeba`,
  `d126d74e` and the `arylen-ignored` stub's zero, all at once.
* `Conv2UC` on an all-blank key is a **no-op**, so deleting it cannot change an
  answer. That is the `no-uppercase` stub's zero.
* a blank key is 200 blanks whether the width is 200 or 201, so `6ca91bba`
  cannot die either.

So the rule the next dispatch needs is sharper than "supply both sides": it must
supply a **non-blank** name that is the *k*-th word of a real file line.

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

> **That paragraph is what R14 is.** It was taken almost verbatim: the plant
> position *k* sweeps 1..3, every free scalar integer is set to *k-1* and *k*
> (because the generator cannot know which integer picks the word, only that one
> usually does), the key is the word case-INVERTED, and the whole construction
> is carried to R12's narrowing width as well. The X3 cost the paragraph was
> waiting on turned out to be a byte-prefix identity on all three units that can
> fire the rule, which is a stronger answer than the case-count comparison the
> previous check settled for.

## The six survivors of the first dispatch, and the one that is left

| id | mutant | first dispatch | now |
|---|---|---|---|
| `6ca91bba` | `'200' -> '201'` (`MaxParamLength`) | corpus gap — no case supplies a *word* or a *key* reaching 200 characters | **KILLED, 14 of 2514.** R14's long-word plant at R12's width |
| `d76903ed` | `'2' -> '3'` (`WordInd = 2`, the `.NOT. PRESENT` arm) | the 47-case gap | **KILLED, 48** |
| `3bc7aeba` | `'1' -> '2'` (`WordInd = AryLen + 1` → `+ 2`) | the 47-case gap | **KILLED, 18 or 21** |
| `b2c89b46` | `'1' -> '2'` (the search loop starting at line 2) | changes an answer only where a case's *only* match is on line 1, and none of the 47 is | **KILLED, 21 or 18** |
| `d126d74e` | `negate_cond` on `if (!has_AryLen)` | the 47-case gap — it exchanges `WordInd = 2` for `AryLen + 1` | **KILLED, 72** |
| `ca75abea` | `'2048' -> '2049'` (`MaxLineLength`) | undefined, not equivalent | **STILL ALIVE.** See below |

The two `'1' -> '2'` kill counts are 18 and 21 and the assignment between the
two sites is deliberately not stated: the mutator records no line, the probe
that used to distinguish them (`const_tweak_probes/`) was run on the 2370-case
corpus, and neither is a survivor any more, so re-deriving the mapping would
cost a run to settle a question with no consequence. Both are dead either way.

Two more were declared equivalent (`mutation/FindLine.equivalences.json`), and
the undeclared run is committed beside them at 0.8889 (24 of 27)
(`mutation/FindLine.undeclared.json`) so what survived is on the record before
any of it was excused.

## `ca75abea`, the one that is left, and why it is (c) and not (a) or (b)

`char_assign(Line, 2049, …)` writes one byte past the caller's 2048-byte buffer.
It is **not equivalent** — the two programs do not agree on every admissible
input, because one of them has no defined behaviour at all, which is the
distinction unit #7 drew. It is **not a corpus gap** either, and that is now
measured against two independent oracles rather than argued:

```
differential harness, 2514 cases          0 failed
gate, 27 scenarios, 5,252,000 values      0 mismatched, 0 channels, 0 scenarios broken
```

**And both zeros have a positive control at the SAME SITE**, chosen against the
program rather than against the probe — the campaign has already had one control
that was a no-op and reported the probe's own number (P10, below). The control
is the same constant moved one byte the OTHER way, so its effect lands INSIDE
the buffer each oracle compares:

```
harness   2048 -> 2047   60 of 2514        findline.linewidth-control.cpp
gate      2048 -> 5      1,583,216 of 4,732,000 across 131 channels, and
                         scenarios 19 and 27 stopped running altogether
```

So the site is live, both oracles can see it move, and the mutant is invisible
to both. **The asymmetry is the finding**: one byte too FEW is a wrong answer;
one byte too MANY is not an answer at all, and no value comparison reads bytes
outside the buffer it is comparing.

The site cannot be deleted, unlike `char_assign`'s old `std::min` — VIT emits no
`len_Line`, so the width has to be stated in the C++ exactly once, and it is.

This is the same class unit #31 recorded as `(c) 1 an out-of-bounds write into
allocator padding`, and `Read_OL_Input` before it. The instrument that would
kill all of them is a sanitiser build (`-fsanitize=address,undefined`), which is
already a **proposed method amendment** in `DECISIONS.md`; it changes what
"killed" means for every unit in the campaign, so it is escalated rather than
adopted inside one unit's dispatch.

**It is, however, now demonstrated rather than argued** (`asan_demo/`). Under
`-fsanitize=address` the shipped translation produces **0 bytes of diagnostic**
and the mutant produces a `heap-buffer-overflow`, `WRITE of size 1`, `0 bytes
after 2048-byte region`, naming `char_assign` and the `std::vector<char> Line_a`
the harness allocates. That answers the two questions three units of argument
could not: the instrument fires on the mutant, and it is silent on the correct
program. What it does not answer is the one the amendment turns on — whether the
other 31 translations are clean under it — and that is a sweep, not a probe. The
score stays 0.960: changing the mutation instrument for one unit is the thing
X3 forbids.

`evidence/FindLine/gate.survivor-ca75abea.json`,
`evidence/FindLine/gate.linewidth-control.json`,
`evidence/FindLine/harness.linewidth-control.json`.

## The canary probe, and the control that took two attempts

Both declared mutants enlarge a fixed-width local by one byte. The claim is that
the extra byte is never read or written. It is provable by inspection — every
index expression is bounded by `MaxParamLength` — and this campaign checks
readings, so it was also measured: `findline.canary-probe.cpp` declares both
arrays one longer, puts `\x7f` in the extra byte of each and sets `LineNum = -7`
if either is disturbed. **0 of 2514.**

A zero from a probe is worth nothing until the probe is shown able to be
non-zero (P10), and the first control **was a no-op**: `conv2uc_c(ParamNameUC,
MaxParamLength + 1)` reads the extra byte and writes it only if it is a
lowercase letter, and `\x7f` is not one. It reported 0 of 2370 on the corpus of the day — the same number
as the probe, and for one run it looked exactly like a pass. The control now
writes the byte unconditionally and fails **2514 of 2514**
(`harness.canary-control.json`).

Both numbers were re-taken on the 2514-case corpus; both mutants are still
survivors of the undeclared run, so the declaration is still a declaration and
not a formality.

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
harness.match-count-probe.json           89 of 2514 reach the match arm
harness.nonblank-key-probe.json          47 of 2514 -- so 42 are NOT blank-vs-blank
harness.canary-probe.json                0 of 2514   the extra byte is untouched
harness.canary-control.json              2514 of 2514   and the probe can fire
harness.postintegration.revert-verified.json
arylen_probe.{f90,sh,txt}                what the reference does per AryLen
const_tweak_probes/                      which `'1' -> '2'` sites were visible
                                         BEFORE R14; annotated, not rewritten
vit_check_scope_redtest.{sh,txt}         the `vit check` fix, both directions

x3_check/                                the FIRST dispatch's X3 check, in case counts
x3_check_r14/                            R14's, in corpus BYTES -- README + run.sh
findline.linewidth-control.cpp           MaxLineLength 2048 -> 2047, the in-bounds twin
harness.linewidth-control.json           60 of 2514   -- the harness CAN see this site
gate.survivor-ca75abea.json              0 of 5,252,000  -- and cannot see 2049
gate.linewidth-control.json              1,583,216 of 4,732,000 for 2048 -> 5,
                                         so neither zero is a dead probe
findline.linewidth-mutant.cpp            ca75abea, written out for a non-vit_mutate
                                         instrument to be pointed at
run_asan_demo.sh, asan_demo/             the proposed amendment's instrument, run:
                                         0 bytes on the correct program, a
                                         heap-buffer-overflow on the mutant
```
