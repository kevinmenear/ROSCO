# Unit #55 — `ParseInAry_Opt` — evidence

`SUBROUTINE ParseInAry_Opt( FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc )`,
`rosco/controller/src/ROSCO_Helpers.f90:741` (clean baseline `54dd134`).

Find the line of an input file whose `AryLen + 1`-th word is `ParamName`,
allocate `Ary(max(AryLen, 1))`, and read `AryLen` **INTEGER(IntKi)** values out
of that line with a Fortran **list-directed READ**. Three callees — `FindLine`
(#32), `GetWords` (#8), `Int2LStr` (#10) — all already translated. Live in all
27 scenarios: **230 calls**.

**Disposition: `deferred`.** Six layers ran. **Five are green and red-tested;
the sixth, mutation, is far below the campaign's threshold at 68 of 156** — for
the same reason as the sibling, and this unit measures the reason from a second
direction that the sibling did not have.

| layer | result | red-tested |
|---|---|---|
| parser conformance (`parser_conformance.txt`, `parser_replay.txt`) | **113 records, 0 mismatched.** Every record is gfortran's own answer for that record, emitted as ICHAR codes beside its iostat and every element; `parser_conformance.cpp` textually includes the SHIPPED translation and replays them through its own `list_read_ints` | **three**, each a mistake this parser could plausibly *have*: the sibling's separator set **3**; the semicolon guard's `sep != Sep::Blank` half **8**; the sibling's zero-store rule **26** |
| differential harness (`harness/ParseInAry_Opt.json`) | **13,674 checked, 0 failed**, 0 inadmissible, against the CLEAN Fortran. `Ary` **not compared on 4,067** cases — the size PREDICTED from this unit's own partition before the entry was written (§4) | **four stubs, every count predicted first and every one exact**: no-op **13,448**; the `.NOT. AllowDefault_` arm deleted **4,811** = 4006 + 805; the READ deleted **103** = 61 + 42; the `IF (AryLen < 1)` arm deleted **16** = 8 + 8, two different cells |
| mutation (`mutation/ParseInAry_Opt.json`) | **157 mutants, 1 nocompile, 156 behavioural, 68 killed, 88 survived, score 0.4359** — BELOW the threshold, and the number is real: green baseline, clean tree, `compared_against: fortran_reference_on_a_clean_tree` | — the score IS the red test (E4.6). 79 of the 88 survivors are in two regions: the list-directed READ, which only **103 cases of 13,674** can distinguish anything about, and the PRINT record, which no layer compares. `mutation_survivors.txt` |
| survivor cross-check (`survivor_replay.json`) | **53 of the 88 survivors are KILLED by the parser replay** — 52 in the READ region, 1 outside | the **negative control is built in**: 16 of the 17 survivors outside the parser come back `unreached`, because the replay enters the translation only through `list_read_ints` (§5) |
| post-integration (`harness/ParseInAry_Opt.postintegration.json`) | 13,674 checked, **0 failed** | this unit's own `vit_copy_scalars_to_errorvariables` deleted from its own wrapper: **10,611 of 13,674**, predicted in the runner's header BEFORE the run; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ParseInAry_Opt.json`) | 5,252,000 values / 351 channels, **0 mismatched** | **TWO**: every parsed value + 1 moves **1,857,893** (35% of the gate), revert-verified at 0; the `Ary = 0` default arm moves **0**, and the artifact carries the argument for the zero (§6) |

**Every artifact of this unit names one instrument revision** — loop `2ff3660`,
`revcheck --unit`. That revision is one commit newer than the sibling's because
this unit had to fix the instrument to run at all (§7).

**No kernel.** The plan allowed "kernel replay **or** direct-call harness", and
the direct-call harness is the layer taken — as for units #45 through #54. It is
also the right one here for a reason of this unit's own: the behaviour under
test is a **parser**, and the corpus a KGen window would capture is 230
invocations of one well-formed input file. The harness supplies 13,674 records
that ROSCO never writes, and the conformance corpus supplies 113 more that
neither writes.

---

## 0. The sibling, and what was and was not carried across

`ParseDbAry_Opt` (#54) and this unit are **the same subroutine with one
declaration changed**. `diff` of the two clean bodies (`ROSCO_Helpers.f90`
:741-876 and :882-1015) is: the routine name, the declaration of `Ary`
(`INTEGER(IntKi)` here, `REAL(DbKi)` there), the declaration ORDER of three
locals, one commented-out `PRINT`, and whitespace. Nothing else.

So the discipline for this unit was decided in advance and is stated here so it
can be checked rather than trusted:

* **Copied, not re-derived (P4):** `ftrim`, `assign_errmsg`, `int2lstr_trimmed`,
  `field`, the descriptor helpers, the whole body's control flow, the two range
  pins and their reasoning, the stub runner, the wrapper red-test runner.
* **Measured here, not adapted:** everything that touches the ITEM TYPE. The
  list-directed READ (§3), the `PRINT` record's field width (§3), the partition
  (§4), and each red test's predicted count (§4, §6b).

The one place where copying would have been wrong is exactly the place the
sibling's own evidence would have led you: **`;` is in `ParseDbAry_Opt`'s
separator set, and putting it in this one's is a defect on three of 113
records.** §3.

And one place where the sibling's *conclusion* does not transfer even though
its reasoning does: **this unit's default arm is dead in a stronger sense.**
`coverage/line_coverage.json` records `ROSCO_Helpers.f90:775` (the `FindLine`
call) **230 times across all 27 scenarios** and :806/:807 (`Ary = 0` and the
`PRINT`) **with no counter at all**. `FoundLine` is TRUE at every one of this
unit's calls. The sibling's same two lines run 66 times across 21 scenarios.

## 1. C1 — what the plan said, and the one thing it had wrong

`plan.json` recorded

> writes to unit UnEc without opening it -- an UNCOMPARED output, not a missing input

The second half is right and the first half is **false**, and it is false for
the same measured reason unit #54 recorded: `ReadControlParameterFileSub` sets
`UnEc = 0` and OPENs it on `<RootName>.RO.echo` when `CntrPar%Echo > 0`
(`ReadSetParameters.f90:358-362`, clean). The unit never writes to an unopened
unit; when the arm is live at all the CALLER has connected it. And the arm is
never live: `:359` is reached 28 times and `:360` zero, and all 22
`Examples/DISCON*.IN` set `Echo` to 0.

Not an uncompared output, but a **dead** one whose only possible C++ counterpart
would write a *different file*. §7.

Ten shipped `CALL ParseAry` sites resolve to this member of the generic
interface, all `INTEGER(IntKi), ALLOCATABLE` fields of `CntrPar`:
`F_GenSpdNotch_Ind`, `F_TwrTopNotch_Ind`, `PerfTableSize`, `Ind_BldPitch`,
`AWC_n`, `AWC_harmonic`, `CC_GroupIndex`, `StC_GroupIndex`,
`Ind_CableControl`, `Ind_StructControl`. They are **index** arrays, which is why
the gate red test in §6 perturbs by 1 rather than by an epsilon.

## 2. C4/C5 — the translation

`vit translate` prompt in `vit_translate.stdout.txt`; the shipped file is
`translations/ROSCO_Helpers/parseinary_opt.cpp` and
`rosco/controller/src/parseinary_opt.cpp`.

Three things had to be exact, and one of them is the unit:

* **`ALLOCATE(Ary(FinalAryLen), STAT=ErrStatLcl)` WITHOUT A PRIOR DEALLOCATE.**
  Allocating an already-allocated object is an *error* in Fortran, so a caller
  who hands in an allocated `Ary` gets a non-zero STAT, keeps the array it
  arrived with — data, extent and bounds — and lands in the branch whose message
  says exactly that. The `ALLOCATED(Ary)` test that picks between the two
  messages is therefore TRUE on the *only reachable* failure. 5,697 of the
  corpus's 13,674 cases take it.
* **The list-directed READ.** §3.
* **`Ary = 0` is a whole-array assignment** over `SIZE(Ary)`, which is
  `FinalAryLen` on a fresh allocation and the caller's extent when the allocate
  failed.

**Three things in the reference are deliberately not translated, and all three
are named here rather than left to be found.** `DEBUG_PARSING` is a `.FALSE.`
PARAMETER, so the block it guards is dead in every build of this tree. The
`STAT=` branch of `ALLOCATE(Words_Ary(AryLen+1))` has no C++ counterpart:
`std::vector` reports failure by throwing rather than by a status, and writing
`ErrStatLcl = 0; if (ErrStatLcl != 0)` would put a comparison in the file that
no input can make either way — the shape units #1, #4 and #43 each measured as
surviving mutants. And the commented-out `PRINT *, "Line: ", TRIM(Line)` at
:777 is a comment, which is the one line of the body the sibling does not carry.
The `GetWords` call ABOVE the debug block **is** translated, and is made
unconditionally as the reference makes it, even though `DEBUG_PARSING` makes its
result unreadable (X1: do not route around a callee).

**One transcription that looks like a defect and is not.** The parse-error
message says

> `The "<name>" array was not assigned valid **REAL** values on line #`

in a routine that parses INTEGERs. That is upstream's own copy-paste from
`ParseDbAry_Opt`. It is transcribed byte for byte (P7): `ErrVar%ErrMsg` is a
compared output, so a "corrected" spelling would be a red case rather than an
improvement.

## 3. C5 — the READ, and the two rules the sibling would have got wrong

`READ (Line,*,IOSTAT=ErrStatLcl) Ary` is list-directed input of `SIZE(Ary)`
INTEGER(4) values from an internal file: **one** record, of `Line`'s declared
width `MaxLineLength = 2048`, blank-padded. Both halves of what it does are
outputs of this unit — the error status decides the message, the array decides
everything else — so the model has to reproduce *which items were assigned*, not
just the verdict.

**Every rule is measured**, five probes, 113 records, none of it read off the
standard and none of it inherited from the sibling's REAL measurements.

### The rule the INTEGER reader does NOT have

Unit #54 found that a REAL field with a decimal point and no digits reaches
`strtod`, whose `0.0` is **transferred before the error is raised** — three
cases of 13,674 turned on it. The INTEGER reader has no such path, and its
absence was **looked for** rather than assumed:

```
'. 5'  '., 5'  './ 5'  '+. 5'  '5. 6'  '5., 6'  'e1 5'  '.e1 5'
    -> iostat 5010, EVERY element still at the sentinel
```

Every one of the 113 records that returns 5010 leaves the failing item
unchanged; items COMPLETED before it keep their values (`'1 2 bad 4'` → 1, 2,
sentinel, sentinel). Transplanting the sibling's rule is red test 3 below: **26
records of 113**.

### The rule that is this unit's own: the semicolon

Unit #54 measured `'1.0;2.0'` as storing 1.0 and then failing, and put `;` in
its `is_separator` set. The INTEGER reader is not that, and it took three probes
to pin:

```
';'  ';1'  '  ;1'  ';1 2'     5010                    not at the first item
'1;2'  '1; 2'                 5010, NOTHING stored    not as a value terminator
'1, ;2'  '1 , ;2'  '1,,;3'    5010                    not after a COMMA
'1<LF>;2'                     5010                    not after an end-of-line
'1 ;2'  '1 ; 2'  '1  ;2'      0,  [1, unchanged]      after a BLANK it is a NULL
'1 ;2 3'                      0,  [1, unchanged, 2]   ... and scanning continues
'1 ;;2 3'  '1 ; ;2'           5010                    ... and it then acts like a comma
```

which is the `Sep` state in the translation. Thirty-one semicolon records,
because one wrong branch here is a wrong `Ary` **and** a wrong `ErrMsg` on the
same case. Copying the sibling's set is red test 1: **3 records of 113**.

### The rest, and the boundary that was measured rather than recalled

```
'1 2' n=4              iostat -1     running out is END, not an error
'1, ,3 Name' n=3       iostat  0     1, unchanged, 3   -- a null consumes an item
'2* 8 Name' n=3        iostat  0     unchanged, unchanged, 8
'1/2 Name' n=3         iostat  0     1, unchanged, unchanged  -- '/' terminates
'0*1 2'                iostat 5010   a zero repeat count is an error
'-2147483648'          iostat  0     the range is exactly INTEGER(4)'s
'2147483648'           iostat 5010
'0000000000000000042'  iostat  0     42 -- the bound is on the VALUE, not the digits
'199999999*7'          iostat  0     7 7
'200000000*7'          iostat  0     7 7      <- MaxRepeat, MEASURED at the boundary
'200000001*7'          iostat 5010
```

The three iostat values are reproduced as gfortran spells them (0, 5010, -1)
even though the reference only ever tests `/= 0`: a model that returned "1" for
both failure kinds would be a model of the TEST rather than of the READ, and the
three sibling units in this family read the value too.

### The PRINT record

Measured for this unit's own statement:

```
 ROSCO Warning: ... Using default value of [           0           0           0 ]
 [           0          -1  2147483647 -2147483647 ]
```

One leading blank starts the record; a CHARACTER item is written raw; each
INTEGER(4) is a self-contained **12-byte** field, right-justified, with no
separator between consecutive integers nor between a CHARACTER and a following
integer; and ONE separator blank appears before a CHARACTER item that FOLLOWS an
integer. The widest INTEGER(4) is `-2147483648` at 11 characters, so the field
never overflows and there is no `'*'` fill to model — which is a difference from
the sibling's 26-wide REAL field, where the fill is reachable.

## 4. C6 — the harness, and an exclusion whose size was predicted first

**13,674 cases, 0 failed, and `Ary` not compared on 4,067.**

The first run, without the exclusion, failed **4,067** — every failure on `Ary`
and no case failing on any other output. `harness_partition.txt`, produced by one
`fprintf` in the generated test and a rebuild without regenerating the corpus
(unit #47's probe), classifies every case by three things, all of them
properties of (corpus × reference):

```
  alloc  arm             cases   Ary-failing
      0  not-allowed      4006     4006      <- 100%
      0  other            3003        0
      0  read-failed        61       61      <- 100%
      1  already-alloc    5697        0
      1  not-allowed       805        0
      1  other              60        0
      1  read-failed        42        0
```

A case fails **if and only if** the reference ALLOCATED `Ary` on this call and
then returned on a path that assigns none of it (`.NOT. AllowDefault_`) or only
a prefix of it (the READ error arm). Those elements are **undefined** in the
reference — the recorded diffs are pointer values, e.g. `b00aab0220e80000` — so
the comparison decides nothing about either implementation. 100% of two cells
and 0% of five: no coincidental agreement anywhere.

**This table is cell-for-cell identical to the sibling's, and it was measured
here.** The two units share a signature and a body, so the corpus rules draw the
same 13,674 cases — but the case FILE differs
(`0c18dba45e22d2df0239c79c7410e11b45b7e28dbf3bcea04f13edf0cce68900` against the
sibling's `64942b97...`, because `Ary`'s elements are 4 bytes here and 8 there),
so nothing about the agreement was guaranteed in advance.

`harness/ranges.toml` then carries the ninth judgement kind, and **its size was
stated before it was written**:

```toml
Ary = { no_oracle_when = ["alloc_Ary == 0", "ErrVar_aviFAIL >= 0"],
        and_reference  = "ErrVar.aviFAIL < 0", reason = "..." }
```

| | predicted | measured |
|---|---|---|
| cases the condition excludes | 4,067 | **4,067** |
| Ary-failing cases outside it | 0 | **0** |
| the one-relation form (dropping the entry-status half) | 4,233 | **4,233** |

The 166 difference is exactly the cases that arrived ALREADY FAILED and never
entered the reference's body (`IF (ErrVar%aviFAIL >= 0)` wraps everything),
where `Ary` is untouched on both sides and the answer is perfectly defined.

**Two checks on it.** The corpus did not move — `parseinary_opt_cases.bin`
hashes `0c18dba4...` before and after, so every case index is the one every
earlier artifact was taken on. And the condition reads the **reference**, never
the translation: `_b` is the Fortran side, and the input half is a snapshot
taken before the calls, so the excluded set is a property of (corpus × reference)
and identical under every mutant and every stub — which is the whole reason a
mutation score over it is a measurement.

**What the green certifies, stated exactly: 13,674 of 13,674 cases agree on
every output that has an oracle, and on 4,067 of them one output — `Ary` — was
not compared because the reference has no answer for it.**

**This is upstream ROSCO's defect, recorded and not repaired (P7)** — the same
one unit #54 recorded, one subroutine over. `ParseInAry_Opt` returns an
ALLOCATED, UNASSIGNED `INTEGER(IntKi) :: Ary(:)` on two live paths. Harmless in
the shipped program (both paths set `aviFAIL = -1` and every caller returns on
that), and it is the reason this unit's primary layer needs the exclusion at all.

### The four red tests, all predicted, all exact

```
the whole unit as a no-op              13,448   = 13,674 - the 226 that arrived failed
the .NOT. AllowDefault_ arm deleted     4,811   = 4006 + 805
the list-directed READ deleted            103   =   61 +  42
the IF (AryLen < 1) arm deleted            16   =    8 +   8
```

The last is this unit's own and its 16 come from **two different cells**, which
is why it is worth more than a larger number: 8 are `alloc=0, AryLen<1, other`,
where `Ary` IS compared and its extent differs (1 against 0); 8 are
`alloc=0, AryLen<1, read-failed`, where `Ary` is excluded but a READ of **zero**
items cannot fail, so `aviFAIL` and `ErrMsg` differ. The 15
`alloc=0, AryLen<1, not-allowed` cases in the same corner must still pass, and
do — that arm's answer does not depend on the extent.

## 5. C6 — mutation: 68 of 156, and 53 of the survivors die on the other instrument

The sweep ran on a clean tree against a green baseline, in 101 s.

```
157 mutant(s), 1 did not compile, 156 behavioural
68 killed, 88 SURVIVED     score 0.4359        <- below the campaign threshold
```

Every mutant is placed by `harness/cppmutate.py`'s own recorded position, not by
a text search (`mutation_survivors.txt`), and the placement checks itself: with
`ParseInAry_Opt` 0 of 88 survivor ids resolve, with `parseinary_opt` all 88 do
(RUNBOOK, unit #40), and the script refuses to print a table if any id fails.

```
survived / mutants   region
     71 / 112        the list-directed READ
      8 /   8        the PRINT record
      4 /  18        the unit's own body
      2 /   4        the constants and the file scope
      1 /   6        assign_errmsg
      1 /   1        int2lstr_trimmed
      1 /   6        the Ary descriptor helpers
      0 /   2        ftrim
```

* **The READ is reached by 103 cases of 13,674 — measured, not estimated.**
  `harness.no-read-stub.json` deletes the whole parser and moves exactly 103.
  0.75% of the corpus. `FindLine` finds the parameter only where R14 planted it,
  and R14 reports reaching its `1:plant` shape alone. 71 survivors in 112
  mutants is what a corpus that exercises a parser 103 times looks like.
* **The PRINT is compared by nothing.** The harness compares out-parameters, the
  gate compares simulation channels, and this record goes to stdout. 8 of 8.

### The survivors put through the other instrument — 53 of 88 die

The RUNBOOK's rule (unit #53) is to ask whether another instrument can reach a
survivor before calling it a corpus gap. Unit #54 asked once per REGION, with
two gate runs. This unit has a second instrument for its largest region that
costs about a second a mutant, so **it was asked for every survivor**
(`run_survivor_replay.sh`, 42 s). Each mutant's source is cppmutate's own
`.source` — the same bytes `vit_mutate.py` compiled — not a text substitution.

```
  region        verdict      count
  READ          KILLED       52
  READ          unreached    19
  other         KILLED        1
  other         unreached    16
```

**The negative control is the half that makes it evidence**, and it holds: the
replay enters the translation only through `list_read_ints`, so the PRINT
record, `assign_errmsg`, `ary_at` and the unit's own body must come back
unreached — and 16 of the 17 do. A run in which everything died would have meant
the replay was measuring something else.

**The one that crosses is the one that should.** `b93bad70`, `MaxRepeat`
`200000000 -> 200000001`, killed by **one record of 113** — the record put in
the corpus for exactly that boundary. The boundary was measured
(`199999999*7` and `200000000*7` both give `7 7`, `200000001*7` gives 5010)
rather than recalled from libgfortran's source, and the kill rests on the
measurement.

**The 19 READ survivors the replay also cannot reach are one thing.** Fifteen of
the nineteen are `p < len` → `p <= len` or `rec[p]` → `rec[p + 1]` at the
record-bound checks: they read one byte past a blank-padded buffer whose next
byte is, on this allocator, another blank, so the answer does not change. That
is the campaign's known sanitiser-build class (unit #31's open amendment); this
unit adds only that its 19 belong to it, measured rather than assumed.

**Nothing is declared equivalent and the score is unchanged at 0.4359.** A kill
by a different instrument on a different corpus is not a kill by this one, and
folding it in would make the number unreadable (a corpus change invalidates
every part already taken). What the run replaces is the ARGUMENT: "the parser's
survivors are a corpus gap" was prose, and it is now 53 measurements.

**The threshold is not met and the disposition stays `deferred`.** What would
move it is what unit #54 already stated: a corpus in which `FindLine` FINDS the
line in thousands of cases rather than a hundred — R14's plant reaching its
`2:plant` and `3:plant` shapes, and the planted line carrying varied numeric
text. That is a generator change and it re-takes every layer that reads the
corpus. The PRINT survivors need a different instrument again.

## 6. C7–C9 — integration and the gate

`vit integrate --apply --reverse-copy`. The flag is required and the wrapper was
**read** rather than assumed (unit #49's practice): the one `INTENT(INOUT)`
derived-type dummy is `TYPE(ErrorVariables)` and the unit writes the SCALAR
`ErrVar%aviFAIL`, so `vit_copy_scalars_to_errorvariables` has to be in the
emitted wrapper. It is.

Gate: **5,252,000 values / 351 channels / 27 scenarios, 0 mismatched.** Two
perturbations, both in this unit's own translation unit, both revert-verified:

```
every parsed value + 1        1,857,893 of 5,252,000 moved   (35%)
the `Ary = 0` default arm             0 of 5,252,000
```

The first is the unit's whole product reaching the controller. The ten arrays it
fills are **index** arrays — `Ind_BldPitch`, `CC_GroupIndex`, `AWC_harmonic`,
`PerfTableSize` and the rest — so `+1` moves what the controller indexes with
rather than a magnitude, which is why it is the right perturbation for an
integer parser and an epsilon would not have been.

**The zero is expected, and the expectation is in the artifact beside the
number** (unit #43's rule) — and it is a *stronger* zero than the sibling's.
There, the arm RUNS (66 executions across 21 scenarios) and moved nothing
because the arrays that take a default are the notch arrays of a scenario with
zero notch filters. Here the arm is **never reached at all**:
`coverage/line_coverage.json` records `:775` 230 times across all 27 scenarios
and `:806`/`:807` with no counter. The layer that covers the arm is the
differential harness — the 3,003 cases of the `alloc = 0, other` cell.

## 6b. E4.5 — the post-integration harness, and a failing set predicted in advance

**13,674 checked, 0 failed.** The red test deletes this unit's own
`vit_copy_scalars_to_errorvariables`, scoped to its wrapper by line range with
the edit count asserted at 1 (the same CALL is generated into a dozen wrappers
in that file).

The runner's header states the failing set **before** the run, from this unit's
own partition: the cases reaching an arm that writes `aviFAIL` are
`not-allowed` (4006 + 805), `read-failed` (61 + 42) and `already-alloc` (5697)
= **10,611**; the `other` cells (3003 + 60) = 3,063 write no scalar and must
still pass.

Measured: **10,611 of 13,674.** Exact. Reverted, rebuilt, green re-taken at 0.

Note what this layer's green over the 4,067 undefined cases means and does not
mean: after integration both sides run the same C++, so both allocate through
the same allocator and both leave the same bytes. That is precisely what this
layer is for — it measures the *wrapper*, not the arithmetic.

## 7. One tool defect, fixed rather than worked around

**`vit_mutate.py` and `vit_harness.py` decoded the generated test's stdout as
strict UTF-8.** A unit whose reference `PRINT`s a CHARACTER argument writes
whatever bytes the CORPUS put in it, and the corpus draws arbitrary bytes.

```
[115/157] const_tweak      '0' -> '1'  SURVIVED
Traceback (most recent call last):
  ...
UnicodeDecodeError: 'utf-8' codec can't decode byte 0xf1 in position 1814
```

The sweep died at mutant 116 of 157, losing 75 seconds and every mutant after
it. The tolerant `_payload()` read two lines below the failing call cannot help:
the process raises before it is reached. **Fixed in the loop repo at `2ff3660`
(X2), not worked around**, and the same edit made to the `make` invocation
beside it — a compiler diagnostic quoting a source byte has the same failure.

**Additive, and checked rather than argued**: the green harness re-taken either
side of the fix differs in `loop_rev` alone, and the corpus hashes identically.
Every artifact of this unit was then re-taken under the new revision, which is
why `revcheck --unit` reports one instrument across all of them.

The `UnEc` echo WRITE is **not translated**, for unit #54's family reason:
`UnEc` is a Fortran UNIT NUMBER and C++ cannot reach the Fortran runtime's unit
table, so the only record this side could write is one to `fort.<UnEc>` — a
different file. Nothing is emitted, not even a guarded no-op (a translated
`if (UnEc > 0) { }` is a mutable comparison no input can kill), and
`harness/ranges.toml` holds `UnEc` at 0 so the reference does not write a record
the translation has no counterpart for. This is the second of the five units
that carry the identical statement.

## 8. What none of the layers can see

* **`Ary`'s undefined elements, on the two paths that produce them.** §4. No
  instrument in this campaign can adjudicate them and none is claimed to; the
  region is NAMED and COUNTED in the artifact (4,067).
* **Whatever the 35 survivors that no instrument reached would have caught.**
  §5. 19 of them are the record-bound reads that need a sanitiser build; 8 are
  the PRINT record; the remaining 8 are in the unit's body, its constants and
  `ary_at`, unreached by the parser replay by construction.
* **The `PRINT` record.** Translated and measured against the reference, but no
  layer *compares* it: the harness compares out-parameters and the gate compares
  simulation channels. It is also **dead in all 27 scenarios** here, which the
  sibling's is not.
* **The `ALLOCATE` failure branch that calls `Int2LStr`.** It needs a genuine
  out-of-memory: with `AryLen` in [0, 32] the request is at most 33 integers,
  and the reachable failure is always "already allocated", which takes the
  *other* branch. Translated (P7), unreachable, ungraded.
* **The `Words_Ary` allocation-failure branch.** Not translated at all, §2.
* **The `UnEc` record.** Not translated, §7.
* **`AryLen` above 32.** A stated narrowing; the reference is defined to at
  least 100,000 (unit #32's probe).

## Files

```
README.md                                this file
done_check.txt                           the done-condition at close
vit_translate.stdout.txt                 the scaffold prompt, as generated
fortran_io_probe.f90  / .txt             40 records through the reference's own READ,
                                         plus the PRINT record layouts
fortran_io_probe2.f90 / .txt             28 more -- separators, repeat counts, and the
                                         search for a zero-store rule that is not there
fortran_io_probe3.f90 / .txt             the semicolon systematically, the repeat-count
                                         ceiling, and what terminates a good value
fortran_io_probe4.f90 / .txt             the semicolon's ADJACENCY
fortran_io_probe5.f90 / .txt             the state the semicolon leaves behind
probe_unec.txt                           the echo record the translation does NOT write
parser_conformance.f90 / .txt            all 113 records, machine-readable: ICHAR codes
                                         beside gfortran's iostat and every element
parser_conformance.cpp                   the replay -- textually includes the SHIPPED
                                         translation so it tests that function, not a copy
run_parser_replay.sh                     the replay plus its three red tests
parser_replay.txt                        0 / 113, and 3, 8, 26 under perturbation
harness_partition.txt                    the 13,674 cases by (alloc_Ary, entry status, arm)
mutation_census.txt                      the sweep's own stdout, every mutant with its verdict
mutation_survivors.txt                   the 88 survivors by source region, and the
                                         cross-instrument result
run_survivor_replay.sh                   every survivor through the parser replay
survivor_replay.txt / .json              53 of 88 killed, and the negative control
run_harness_stub.sh                      one stub through the harness, --no-generate
parseinary_opt.noop-stub.cpp             every statement removed
parseinary_opt.no-allowdefault-arm-stub.cpp   the .NOT. AllowDefault_ arm deleted
parseinary_opt.no-read-stub.cpp          the list-directed READ deleted
parseinary_opt.no-minlen-arm-stub.cpp    the IF (AryLen < 1) arm deleted
harness.*-stub.json                      the four red tests, all at 13,674 cases
run_wrapper_redtest.sh                   perturb the wrapper, prove red, revert, prove green
```
