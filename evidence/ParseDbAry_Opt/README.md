# Unit #54 — `ParseDbAry_Opt` — evidence

`SUBROUTINE ParseDbAry_Opt ( FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc )`,
`rosco/controller/src/ROSCO_Helpers.f90:882` (clean baseline `54dd134`).

Find the line of an input file whose `AryLen + 1`-th word is `ParamName`,
allocate `Ary(max(AryLen, 1))`, and read `AryLen` double-precision values out of
that line with a Fortran **list-directed READ**. Three callees — `FindLine`
(#32), `GetWords` (#8), `Int2LStr` (#10) — all already translated. Live in all
27 scenarios: **1,232 calls**.

**Disposition: `deferred`.** Six layers ran; five are green and red-tested and
the sixth, mutation, is **0.7609 against a threshold of 1.0**. That is this
dispatch's honest number and the shortfall is named rather than argued away.

**AND THE SIXTH LAYER IS NEW AND IT FOUND A DEFECT.** The PRINT record has never
been compared by anything in this campaign; §5b is the instrument that compares
it, the one-record disagreement it found in the shipped translation, and the fix.
Every other layer was then re-taken on the fixed translation.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ParseDbAry_Opt.json`) | **16,512 checked, 0 failed**, 0 inadmissible, against the CLEAN Fortran with all three callee bridges kept. `Ary` **not compared on 5,266** cases (`no_oracle_when`, §4) | **four stubs**, all re-taken on this corpus: no-op **16,252**; the `.NOT. AllowDefault_` arm deleted **6,079**; the READ deleted **394**; the zero-store rule removed **4** |
| mutation (`mutation/ParseDbAry_Opt.json`) | **189 mutants, 4 nocompile, 185 behavioural — 105 KILLED, 17 EQUIVALENT, 30 UNREACHABLE, 33 SURVIVED, score 0.7609.** Sanitised sweep, green baseline, clean tree. `declared_but_killed` and `unreachable_but_killed` both empty | — the score IS the red test (E4.6). §5 |
| PRINT-record conformance (`evidence/ParseDbAry_Opt/print_replay.txt`) | **44 records, 0 mismatched** — every record gfortran's own list-directed output, replayed through the SHIPPED `print_default_warning` by textual include. **Its first run was 1 mismatched and that was a real defect** (§5b) | **the 9 PRINT-region survivors it replayed are ALL killed**, 18 to 43 records each; **negative control 0 of 23** survivors outside the region moved a record |
| post-integration (`harness/ParseDbAry_Opt.postintegration.json`) | 16,512 checked, **0 failed** | this unit's own `vit_copy_scalars_to_errorvariables` deleted from its own wrapper: **13,216 of 16,512**, PREDICTED 13,216 from the re-taken partition before the run; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ParseDbAry_Opt.json`) | 5,252,000 values / 351 channels, **0 mismatched**, re-taken on the fixed translation and again at close | **four**: every parsed value + 0.01 moves **2,236,141** (43%); the `Ary = 0` default arm **0**; the declared-unreachable 48542e3d **0**; the PRINT survivor ae3f319a **0**. §6 |

**Every artifact of this unit names one instrument revision** (loop `b9c8c52`,
`revcheck --unit`). Nothing was left at the second dispatch's `829625b`.

**No kernel**, as at both earlier dispatches: the behaviour under test is a
**parser** and the corpus a KGen window would capture is 1,232 invocations of
one well-formed input file.

---

## 0. What this dispatch was for, and the answer it got

The relaunch (`e2f5139`) cleared this unit to `pending` for one stated reason:
the generator had moved under it. Loop `b33a761` plants **numeric literals** in
the words before R14's key, and its commit message names this unit's survivor
profile as the evidence that the parser region was not being executed. On the
sibling `ParseInAry_Opt` it took R14 from 216 cases to 1,224.

**Re-taken here at `b9c8c52` it changed NOTHING.** The corpus came back
byte-identical — `64942b97…` either side — 13,674 cases, 0 failed, 4,067
excluded. `gen_rev` moved and the corpus did not.

R14's own coverage line said why, and had said it at both earlier dispatches:

```
Reached ['1:plant']
NOT reached ['2:num:dec', … , '2:plant', '3:num:dec', … , '3:plant']
```

Every `k >= 2` shape missing, **including the plain `2:plant` that predates the
numeric lead entirely**. The loop-side fix appended shapes to a `k > 1` branch
this unit's own range pin had already emptied.

**The cause is one keyword** (`evidence/ParseDbAry_Opt/r14_reach_probe.txt`):

```toml
[ParseDbAry_Opt]   UnEc = { lo = 0, hi = 0 }                 # a plant_int
[ParseInAry_Opt]   UnEc = { lo = 0, hi = 0, values = [0] }   # not one
```

R14 sets **every** free scalar integer to `k-1` and `k` in the same case and
drops the whole word index when one refuses (`all(_admits(q, v) for q in
plant_ints)`). A parameter carrying `values` is not a `plant_int`; one narrowed
by `lo`/`hi` is. So `UnEc` pinned to 0 by `lo`/`hi` vetoed `k = 2` and `k = 3`
for the entire unit, while the sibling's identically-meant pin did not. Restated
in the sibling's spelling — **no admissible value of any input changes**.

**NONE OF THAT MECHANISM IS NEW TO THE CAMPAIGN, AND THAT IS THE MORE USEFUL
FINDING.** `DECISIONS.md` records it from unit #55's second dispatch, with the
generalisation attached — "any rule that requires ALL of a class of parameters
to admit a value is silently bounded by the narrowest member of that class" —
and `STATUS.md:38` carries the line
`UnEc = { lo = 0, hi = 0, values = [0] }  R14 1:plant -> 1:/2:/3:plant` in the
sibling's own table. **It was never applied here**, in the unit
`harness/ranges.toml` describes as carrying "THE SAME TWO PINS … FOR THE SAME
REASONS AT THE SAME SITES". The campaign then built on top of the gap: loop
`b33a761` cites both units' survivor profiles, measured itself on the sibling
alone, and is worth zero cases here; and the relaunch cleared this unit for that
generator change alone. §0 of `r14_reach_probe.txt` states it in full.

**The prediction was written first and was WRONG, and its error is the useful
part.** Predicted 1,088 R14 cases, a 14,730-case corpus, and a strict PREFIX of
the old one. Measured **1,224**, **16,512**, and the prefix control **refuses**.
`values` does not only take `UnEc` out of `plant_ints` — it puts it into R2's
flag set (4 flags → 5, `flag_variants` 16 → 18), which grows R14 by 18/16 *and*
grows R2's own block ahead of it, so later case indices move. Two effects, not
separable by reading the entry.

## 1. C1 — what the plan said, and the one thing it had wrong

Unchanged from the second dispatch. `plan.json` recorded "writes to unit UnEc
without opening it"; the second half is right and the first is **false**.
`ReadControlParameterFileSub` sets `UnEc = 0` and OPENs it on
`<RootName>.RO.echo` when `CntrPar%Echo > 0` (`ReadSetParameters.f90:358-362`,
clean), and the arm is never live: `coverage/line_coverage.json` has `:359`
reached **28** times and `:360` **zero**, and all 22 `Examples/DISCON*.IN` set
`Echo` to 0.

## 2. C4/C5 — the translation

Unchanged. `vit translate` prompt in `vit_translate.stdout.txt`; the shipped
file is `translations/ROSCO_Helpers/parsedbary_opt.cpp` and
`rosco/controller/src/parsedbary_opt.cpp`. Four helpers are **copied, not
re-derived** (P4): `ftrim` and `assign_errmsg` from `chkparsedata.cpp`, and
`field` / `nonfinite_text` / `list_directed_real` from
`powercontrolsetpoints.cpp`.

Three things had to be exact: `ALLOCATE(Ary(FinalAryLen), STAT=)` **without a
prior DEALLOCATE** (so a caller who hands in an allocated `Ary` gets a non-zero
STAT and keeps the array it arrived with — 6,767 of the 16,512 cases take it);
the list-directed READ (§3); and `Ary = 0` as a whole-array assignment over
`SIZE(Ary)`.

Two things in the reference are deliberately **not** translated and both are
named here: `DEBUG_PARSING` is a `.FALSE.` PARAMETER, so the block it guards is
dead in every build of this tree; and the `STAT=` branch of
`ALLOCATE(Words_Ary(AryLen+1))` has no C++ counterpart, because `std::vector`
reports failure by throwing and a comparison no input can make either way is a
mutant nothing can kill.

## 3. C5 — the READ, and the defect the harness found in it

Unchanged from the second dispatch, and still the part of this unit with the
most measurement behind it. `READ (Line,*,IOSTAT=ErrStatLcl) Ary` is
list-directed input of `SIZE(Ary)` REAL(8) values from an internal file: one
record, of `Line`'s declared width `MaxLineLength = 2048`, blank-padded.

Every rule is measured rather than read off the standard —
`fortran_io_probe.f90` (31 records) and `fortran_io_probe2.f90` (20 more)
through the reference's own READ with a `-987.654` sentinel:

```
'1.5   NameHere'   n=2  iostat=5010  1.5, SENTINEL   a transferred value STAYS
'1.0 2.0'          n=4  iostat=  -1  1.0, 2.0, S, S  running out is END, not error
'1.0, ,3.0 Name'   n=3  iostat=   0  1.0, S, 3.0     a null value consumes an item
'2* 8.5 Name'      n=3  iostat=   0  S, S, 8.5       r* + separator is r NULLs
'1.0 / 2.0 Name'   n=3  iostat=   0  1.0, S, S       '/' terminates, no error
'1.0;2.0'          n=2  iostat=5010  1.0, S          ';' ends a value, then fails
'1.0.0 2.0'        n=1  iostat=5010  S               a stray char stores nothing
```

**THE DEFECT THE HARNESS FOUND** (first dispatch, kept here because it is the
single strongest thing this layer has done): the first parser had one failure
kind and stored nothing on it. Three cases of 13,674 disagreed
(`harness.parser-no-zero-store.json`), all three the cases where `Ary` arrived
allocated and therefore had an oracle. The discriminator is the **decimal
point**, and it is the point alone:

```
STORED 0.0 then 5010:  '.'  '-.'  '+.'  './'  '-./'  '.,'  '.e1'  '.d0'
NOT STORED,    5010:   '+'  '-'  '+/'  '+,'  'e1'  '..'  '-.e'
```

Removing the rule again is the fourth stub: **4 cases** on this corpus (3 on the
old one).

## 4. C6 — the harness, and the region the reference has no answer for

**16,512 checked, 0 failed, 0 inadmissible.** `harness/ranges.toml` carries
`Ary = { no_oracle_when = ["alloc_Ary == 0", "ErrVar_aviFAIL >= 0"],
and_reference = "ErrVar.aviFAIL < 0", … }` — the ninth judgement kind, added for
this unit at the second dispatch. A case that supplied an unallocated `Ary` and
an admissible entry status, and whose REFERENCE returned `aviFAIL < 0`, is a
case in which the reference ALLOCATEd `Ary` and RETURNed on an arm that assigns
none of it or only a prefix. Those elements are recycled heap.

**On this corpus the exclusion is 5,266 of 16,512** (4,067 of 13,674 before).
The condition reads the REFERENCE side and inputs the calls overwrite are
snapshotted before them, so the excluded set is a property of (corpus ×
reference) and is identical under every mutant and every stub — which is what
makes a mutation score over it a measurement.

The partition is re-taken and now has a **script**
(`run_partition_probe.sh`); the first dispatch's method was "one fprintf and a
`--no-generate` rebuild" and could not be repeated when the corpus moved:

```
  alloc  arm                  cases   aviFAIL-changed
      0  entered-failed         184         0
      0  not-allowed           5207      5207
      0  other                 3036         0
      0  read-failed             59        59
      1  already-alloc         6767      6767
      1  entered-failed          76         0
      1  not-allowed            872       872
      1  read-failed            311       311
                               16512     13216
```

`read-failed` is **370 against the first dispatch's 103** — the READ cell is
what the corpus change bought. `entered-failed` is a new named cell: cases that
arrive with `aviFAIL < 0` and never enter the body, which the first dispatch
folded into `other`.

**THE CLASSIFIER'S FIRST VERSION GUESSED THE MESSAGE STRINGS** and matched none
of them, so 8,302 cases came back `other` and both error cells read as EMPTY.
The substrings are now copied from `ROSCO_Helpers.f90`; the wrong version is
described in the script rather than quietly replaced.

**This is upstream ROSCO's defect, recorded and not repaired (P7).**
`ParseDbAry_Opt` returns an ALLOCATED, UNASSIGNED `REAL(DbKi) :: Ary(:)` on two
live paths. In the shipped program it is harmless — both paths set
`aviFAIL = -1` and every caller returns on that.

## 5. C6 — mutation: 0.7609, and the 33 that are left

```
189 mutants, 4 nocompile, 185 behavioural
105 KILLED   17 EQUIVALENT   30 UNREACHABLE   33 SURVIVED     0.7609
```

**Three things moved it, in order of size, and the kills came before the
declarations.**

```
the corpus (R14 reaching k=2,3 with numeric leads)   77 -> 104 killed
--sanitize                                          104 -> 106 killed
30 unreachable + 17 equivalent declarations       0.5730 -> 0.7681
the print separator fix (the 40-mutant cap)       0.7681 -> 0.7609
```

**THE SANITISER IS ALMOST A NULL RESULT HERE, AND THAT IS A FINDING.** The
dispatch's own instruction is to re-run with `--sanitize` when the survivors are
out-of-bounds reads, and 19 of the 81 were `index_offset` and 16 `compare_op`.
The baseline did **not** report under `-fsanitize=address,undefined`, so the
option's one refusal did not fire and the score is a measurement. It bought
**two** kills (`4d44e6d2`, `f6007814`), nothing regressed, and 10 of the 12
sanitiser kills were mutants the value oracle also killed — the run's own
control. So the `index_offset` survivors here are **not** invisible
out-of-bounds differences: `rec[p + 1]` is inside a 2048-byte blank-padded
record. The unsanitised sweep is kept at `mutation/ParseDbAry_Opt.value-oracle.json`
**and it is a PRE-FIX artifact**: it was taken before §5b's separator repair, so
its 104/185 is against the translation as it stood at `8837a5c2`. It is kept for
the one comparison it exists to support — value oracle against sanitiser on the
same translation — and not as a current number.

**THE 30 UNREACHABLE ARE MEASURED, NOT ARGUED.** `run_line_coverage_probe.sh`
runs gcov over the generated test's own object — the test `#include`s
`parsedbary_opt.hpp`, so the file measured IS the file the sweep mutates and
`vit integrate` ships — at `-O0`, because `-O2` line attribution folds and a
line reported 0 because the optimiser merged it is exactly the false
`unreachable` this exists to avoid.

```
executable lines RUN      224
executable lines NOT RUN   62
CONTROL   the entry line's count is 16,512 = the case count
CONTROL   NON-survivors on never-executed lines: 1, and it is a NOCOMPILE
          (`esign - edig`, char minus std::string), not a kill
```

The second control is what makes the first a measurement: a mutant the corpus
killed on a line it never executed would mean the coverage run measured a
different program.

**THE 17 EQUIVALENCES ARE ARGUED PER MUTANT, WITH THE GREP THAT WOULD REFUTE
EACH** (`mutation/ParseDbAry_Opt.equivalences.md`). The largest group is eight
mutants that change one nonzero IOSTAT into another: `list_read_reals` has one
call site and the value it returns has one read, `if (ErrStatLcl != 0)`. Then
four buffer enlargements (and `char tmp[512]` is argued on the FORMAT's maximum
width, because `sizeof tmp` really is used there); `lower()`'s `'Z'` bound,
whose three call sites compare against `i n f t y a e d q` and never `'z'`; and
`p < len` -> `p <= len` at `parse_real`'s entry, where the one caller has
already established `p < len`.

**The two pairs that are split are the point of doing this per mutant.**
`lower()`'s `'Z'` bound is declared and its `'A'` bound is NOT — `'a'` is the
middle letter of `"nan"`. `std::max`'s argument swap is declared and dropping
the clamp is NOT — the latter holds only while `AryLen = { lo = 0 }` is stated,
which is a fact about the domain rather than about the program.

**THE LAST LINE IS THE CAP, NOT A REGRESSION,** and the arithmetic was predicted
before the sweep. The fix adds one `if`; `negate_cond` is capped at 40; the new
site displaces the last one in file order. `ec43592b`
(`if (ErrVar->aviFAIL < 0)`, the unit's outermost guard) leaves the enumerated
population **and it was killed**, and `b529317d` (`if (n > 0)`) enters it as a
survivor the sweep's oracle cannot kill. One kill out, one survivor in. It is
the 40-mutant cap unit #53 escalated, biting on a one-line repair: this
translation offers 305 mutants and 116 are enumerated by no sweep.

**The 33 that are left are named in `mutation_survivors.txt`**, by region and by
what each region's survival is a fact ABOUT: 10 in the PRINT record, which the
sweep's oracle does not read (§5b compares them anyway); 7 in `match_word`, 5 in
`list_read_reals` and 5 `p <= len` boundaries, which need a different CORPUS;
5 `rec[p + 1]` reads that land on blank padding; and 1 each in `assign_errmsg`,
`lower()` and the unit's own body.

## 5b. A SIXTH LAYER, BUILT THIS DISPATCH, AND THE DEFECT IT FOUND ON ITS FIRST RUN

The PRINT record is the one thing this unit produces that nothing compared. The
instrument is unit #55's `parser_conformance` shape, one region over:

```
print_conformance.f90   44 cases through the reference's OWN list-directed
                        output -- the item list copied from ROSCO_Helpers.f90 --
                        writing each record whole and the same cases as raw
                        IEEE doubles
print_conformance.cpp   textually #includes the SHIPPED translation and replays
                        them through its own print_default_warning
```

**The textual include is the load-bearing choice** (unit #55's finding): the
function lives in an anonymous namespace, a copy in the test would test a copy
and go stale, and `-DVIT_TRANSLATION=` lets a MUTATED copy be replayed without
touching the tree. The three callee bridges are stubbed and abort if reached.

**The values are chosen to sit ON the boundaries the surviving mutants edit**:
`decexp` at −1 and 16 and every value between, `d == 0`, three- and four-digit
exponents, both exponent signs, subnormals, `HUGE`, `TINY`, NaN and both
infinities, an empty array, and names at 0, 1, 60 and 199 characters.

**FIRST RUN: 44 records, 1 MISMATCHED, and it is the translation that is wrong.**

```
case 38, n = 0
  ref | ... Using default value of []|
  got | ... Using default value of [ ]|
```

`print_default_warning` appended the separator blank before `]` unconditionally.
gfortran writes ONE separator blank before a CHARACTER item that follows a REAL
and NONE between two CHARACTER items — the rule this very file's header records
from `fortran_io_probe.txt`. With an empty `Ary` the `[` is followed directly by
the `]`. **No layer could see it**: the differential harness compares
out-parameters, the gate compares simulation channels, and the corpus cannot
even reach it (R5 varies `Ary`'s extent over 3..8).

Fixed at `50b583ab` — `if (n > 0) { rec += ' '; }` — and the replay re-run gives
**44 of 44**. Every other layer was then re-taken on the fixed translation and
none of their numbers moved except the mutation score, for the cap reason in §5.

**THE CONTROL FAILED ON THE FIRST RUN AND THE FAILURE WAS THE BASELINE'S.**
Scored against the REFERENCE, every mutant inherited the baseline's own
one-record deviation and the negative control read *23 of 23 moved a record*. A
kill is *differs from the UNMUTATED TRANSLATION*, not *differs from the
reference* — the same distinction `--sanitize` draws with its baseline refusal.
Corrected, the control is **0 of 23** and all **9 of 9** PRINT-region survivors
die at 18 to 43 records.

**NOTHING IS FOLDED INTO THE SCORE.** A kill by a different instrument on a
different corpus is not a kill by the sweep, and `mutation/ParseDbAry_Opt.json`
is untouched by the replay. What it buys is that nine survivors nothing compared
are now measured — and one real defect.

## 6. C7–C9 — integration and the gate

`vit integrate --apply --reverse-copy`. The flag is required and the wrapper was
**read** rather than assumed (unit #49's practice).

**5,252,000 values / 351 channels / 27 scenarios, 0 mismatched.** Four
perturbations, all revert-verified, all four **predicted before the run**:

```
every parsed value + 0.01     2,236,141 of 5,252,000    predicted 2,236,141
the `Ary = 0` default arm             0 of 5,252,000    predicted 0
survivor 2a9e1695             1,583,216 of 4,732,000    predicted 1,583,216
survivor 48542e3d                     0 of 5,252,000    predicted 0
survivor ae3f319a                     0 of 5,252,000    predicted 0
```

**2a9e1695 IS NO LONGER A SURVIVOR.** At the second dispatch it was the READ
region's representative and its gate run was the argument that the region is a
corpus gap the gate covers. The 16,512-case corpus **kills** it, so the run is
now the control on that argument rather than the argument itself.

Its replacement `48542e3d` (`rec + p` -> `rec - p` in the repeat-count branch)
moves 0, and the prediction was grounded rather than guessed:
`grep -cE '[0-9][*]' Examples/DISCON*.IN` over all 22 files the 27 scenarios
read returns 0.

**So the pair says something different from last dispatch.** Then: READ = a
corpus gap the gate covers, PRINT = an instrument gap nothing covers. Now: the
corpus has killed the mainline parser mutants, and what is left in the READ
region is boundary and rare-form statements the gate's own input files do not
contain either — reachable in principle, unreached by both instruments.

**The zero on the default arm is expected and the expectation is in the artifact
beside the number.** The arm *runs* — `ROSCO_Helpers.f90:945`, 66 times across
21 of 27 scenarios — but only when `AryLen` is 0, and `AryLen` at these call
sites is `F_NumNotchFilts` / `F_GenSpdNotch_N` / `F_TwrTopNotch_N`, so the arrays
that take a default are the notch arrays of a scenario with zero notch filters,
and both consumers sit behind that same count. The differential harness covers
the arm: the 3,036 `alloc = 0, other` cases.

## 6b. E4.5 — the post-integration harness, and a pass set predicted in advance

**16,512 checked, 0 failed.** The red test deletes this unit's own
`vit_copy_scalars_to_errorvariables`, scoped to its wrapper by line range with
the edit count asserted at 1.

The pass set is derived from the re-taken partition **before** the run, not
scaled from the old number: `not-allowed` (5207 + 872), `read-failed` (59 + 311)
and `already-alloc` (6767) = **13,216** must fail; `other` (3036) and
`entered-failed` (184 + 76) = 3,296 write no scalar and must pass. `ErrStat` and
`size_avcMSG` return 0 in all 16,512 and are supplied as 0, so `aviFAIL` alone
decides the set.

**Measured: 13,216 of 16,512. Exact.** Reverted, rebuilt, green re-taken at 0.

*(The revert-verified green needed a second attempt: the first link ran against a
`ROSCO_Helpers.f90.o` the rebuild inside the runner had not finished writing,
and make reported clock skew on the retry. Nothing was wrong with the tree.)*

## 7. Two tool defects and one statement not translated

Unchanged from the second dispatch. VIT's callee-bridge generator was wrong
twice — a CHARACTER dummy whose length is a module PARAMETER, and a
by-reference LOGICAL dummy — and both are fixed in the VIT repo rather than
worked around (X2, `vit@1e30f79`, `vit@3f6e2ce`). The wrong artifact is kept at
`parsedbary_opt_callees.WRONG.f90`.

**The `UnEc` echo WRITE is NOT translated.** `UnEc` is a Fortran UNIT NUMBER and
C++ cannot reach the Fortran runtime's unit table, so the only record this side
could write is one to `fort.<UnEc>` — a different file. Nothing is emitted, not
even a guarded no-op. `harness/ranges.toml` holds `UnEc` at 0.

**AND THAT PIN HAS A SECOND EFFECT NOBODY PRICED**, which is §0: how it is
SPELLED decides whether R14 reaches its second and third word index. The pin
itself is right; two spellings of it are not interchangeable.

## 8. What none of the layers can see

* **`Ary`'s undefined elements**, on the two paths that produce them — 5,266
  cases, named and counted rather than failing.
* **Whatever the 33 surviving mutants would have caught**, and now also the 30
  declared unreachable: an unreachable declaration is a **debt against the
  corpus**, not a credit. §5 and `mutation_survivors.txt`.
* **The `ALLOCATE` failure branch that calls `Int2LStr`** — needs a genuine
  out-of-memory. Translated, unreachable, ungraded.
* **The `Words_Ary` STAT= branch.** Not translated at all, §2.
* **The `UnEc` echo record.** Not translated, §7.
* **The `PRINT` record — NO LONGER TRUE, and §5b is why.** It is now compared,
  44 records against gfortran's own output, and the comparison found a defect.
  What is still true is that the SWEEP's oracle does not read it, so its 10
  mutants survive the score while dying in the replay.
* **`AryLen` above 32.** A stated narrowing; the reference is defined to at
  least 100,000 (unit #32's probe).

## Files

```
README.md                                this file
done_check.txt                           the done-condition at close
r14_reach_probe.txt                      why the generator change was inert here,
                                         the prediction, and the prediction's error
run_partition_probe.sh                   the partition, as a repeatable script
harness_partition.txt                    the 16,512 cases by (alloc_Ary, arm)
run_line_coverage_probe.sh               gcov over the shipped translation
line_coverage.txt                        224 lines run, 62 never, and the two controls
mutation_survivors.txt                   the 33 that are left, by region
print_conformance.f90                    44 records through the reference's own
                                         list-directed OUTPUT
print_conformance.cpp                    the replay, textually including the
                                         SHIPPED translation
run_print_conformance.sh                 both sides, the mutants, and the control
print_replay.txt                         44 / 0 after the fix, 9 of 9 PRINT
                                         survivors killed, control 0 of 23
vit_translate.stdout.txt                 the scaffold prompt, as generated
fortran_io_probe.f90 / .txt              31 records through the reference's own READ
fortran_io_probe2.f90 / .txt             20 more, isolating the zero-store rule
harness.parser-no-zero-store.json        the harness BEFORE the parser fix (first
                                         dispatch; its count is against the old
                                         comparison)
run_harness_stub.sh                      one stub through the harness, --no-generate
parsedbary_opt.noop-stub.cpp             every statement removed
parsedbary_opt.no-read-stub.cpp          the list-directed READ deleted
parsedbary_opt.no-allowdefault-arm-stub.cpp   the .NOT. AllowDefault_ arm deleted
parsedbary_opt.no-zero-store-stub.cpp    the measured zero-store rule removed
harness.*-stub.json                      the four red tests, all at 16,512 cases
mutation.refusal.txt                     189 mutants, and the FIRST dispatch's refusal
                                         to score them against a red baseline
run_wrapper_redtest.sh                   perturb the wrapper, prove red, revert, prove green
parsedbary_opt_callees.WRONG.f90         the bridge VIT generated before the two fixes
probe_unec.txt                           the UnEc arm's coverage
```
