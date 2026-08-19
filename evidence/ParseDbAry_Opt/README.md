# Unit #54 — `ParseDbAry_Opt` — evidence

`SUBROUTINE ParseDbAry_Opt ( FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc )`,
`rosco/controller/src/ROSCO_Helpers.f90:882` (clean baseline `54dd134`).

Find the line of an input file whose `AryLen + 1`-th word is `ParamName`,
allocate `Ary(max(AryLen, 1))`, and read `AryLen` double-precision values out of
that line with a Fortran **list-directed READ**. Three callees — `FindLine`
(#32), `GetWords` (#8), `Int2LStr` (#10) — all already translated. Live in all
27 scenarios: **1,232 calls**.

**Disposition: `deferred`, FOURTH DISPATCH.** Six layers ran; five are green and
red-tested and the sixth, mutation, is **0.8533 against a threshold of 1.0**.
That is this dispatch's honest number and the shortfall is named rather than
argued away.

**THE SIX SURVIVORS THIS DISPATCH WAS SENT FOR ARE ALL KILLED, AND NONE OF THEM
WAS DECLARED ANYTHING.** `b5366e74`, `6b5fc8ce`, `76abf5f3`, `09fbe682`,
`2763d449`, `2f05620e` — every one was case (b), a corpus too narrow to expose
the difference, and every one was closed by fixing the inputs. §5.

**AND 1.0 IS NOT REACHABLE FROM HERE, WHICH IS THE OTHER HALF OF THE ANSWER.**
Ten of the twenty-two survivors are in the PRINT record. They are **not**
equivalent — a second instrument measures every one of them moving between 18
and 44 records — and **not** unreachable — the corpus runs those lines and so do
21 of the 27 gate scenarios. They are mutants with a MEASURED difference that
the scoring instrument cannot see, which is a third case this campaign's
vocabulary has no key for. Escalated in `DECISIONS.md` as a proposed `killed_by`
declaration rather than smuggled into the score. §5.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ParseDbAry_Opt.json`) | **17,520 checked, 0 failed**, 0 inadmissible, against the CLEAN Fortran with all three callee bridges kept. `Ary` **not compared on 1,387** cases (`no_oracle_when`, §4) — was 5,266 of 16,512 | **four stubs**, all re-taken on this corpus: no-op **17,260**; the `.NOT. AllowDefault_` arm deleted **2,209**; the READ deleted **646**; the zero-store rule removed **4** |
| mutation (`mutation/ParseDbAry_Opt.json`) | **189 mutants, 4 nocompile, 185 behavioural — 128 KILLED, 17 EQUIVALENT, 18 UNREACHABLE, 22 SURVIVED, score 0.8533.** Sanitised sweep, green baseline, clean tree. `declared_but_killed` and `unreachable_but_killed` both empty | — the score IS the red test (E4.6), and the **value-oracle control** re-taken beside it says what each step bought: 120 → 128 → 0.8533. §5 |
| PRINT-record conformance (`evidence/ParseDbAry_Opt/print_replay.txt`) | **44 records, 0 mismatched** — every record gfortran's own list-directed output, replayed through the SHIPPED `print_default_warning` by textual include. **Its first run, at the third dispatch, was 1 mismatched and that was a real defect** (§5b) | **the 10 PRINT-region survivors it replayed are ALL killed**, 18 to 44 records each; **negative control 0 of 12** survivors outside the region moved a record |
| post-integration (`harness/ParseDbAry_Opt.postintegration.json`) | 17,520 checked, **0 failed** | this unit's own `vit_copy_scalars_to_errorvariables` deleted from its own wrapper: **9,841 of 17,520**, PREDICTED 9,841 from the re-taken partition before the run; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ParseDbAry_Opt.json`) | 5,252,000 values / 351 channels, **0 mismatched**, re-taken at the new instrument revision and again at close | **six**, and the two non-zero ones reproduce the third dispatch's numbers TO THE VALUE, which is the control on this dispatch's whole claim: parsed value + 0.01 moves **2,236,141** (43%); 2a9e1695 **1,583,216**; the `Ary = 0` default arm **0**; 48542e3d **0**; ae3f319a **0**; and the new a0007207 **0**. §6 |

**Every artifact of this unit names one instrument revision** (loop `59aa876`,
`revcheck --unit`). Nothing was left at the third dispatch's `b9c8c52`.

**No kernel**, as at both earlier dispatches: the behaviour under test is a
**parser** and the corpus a KGen window would capture is 1,232 invocations of
one well-formed input file.

---

## 0. What THIS dispatch was for, and the answer it got

The driver named six survivors and asked, for each, which of three things it is:
(a) genuinely equivalent, (b) unreachable by the harness — *fix the inputs, not
the record* — or (c) a blind spot no rule covers.

**ALL SIX WERE (b), AND ALL SIX ARE DEAD.** None was declared anything. The
table in §5 gives the input that killed each. Two levers did it: the sibling
unit's base-draw pin, which had been sitting in `ranges.toml` one table away
since the second dispatch, and a generator change that builds a lead to the
width of the buffer the reference copies the record into.

**AND ONE THING THAT WAS NOT ON THE LIST TURNED OUT TO BE (c).** Ten of the
remaining twenty-two survivors are in the PRINT record. A second instrument
built at the third dispatch measures every one of them differing from the
unmutated translation, by 18 to 44 records out of 44 — so they are *not*
equivalent, in the strongest sense the campaign can say it. The corpus runs
their lines and 21 of 27 gate scenarios do too — so they are *not* unreachable.
And the sweep's oracle compares out-parameters with `memcmp`, so it cannot see
the stream they write. There is no third key. Escalated in `DECISIONS.md`; the
proposal is a `killed_by` declaration that names the instrument and its artifact,
which is worth 0.8533 → 0.9333 on this unit alone.

**THE ORDER MATTERED AND IT IS THE DISPATCH INSTRUCTION'S OWN POINT.** Five of
the six were `p < len` → `p <= len` or a `>`/`>=` boundary, and the cheapest
thing available was to declare them equivalent — the argument writes itself, the
score goes up, and nothing is learned. The third dispatch had already resisted
that once, declaring `0a3516d1` (the same shape at `parse_real`'s entry, where
the caller has established the precondition) while leaving its five siblings
alive. This dispatch killed three of those five with a record that ends inside a
value. A declaration would have buried them.

**WHAT IT COST, so the next dispatch can price it.** A corpus change re-takes
every layer that reads the corpus; a LOOP-REPO change re-takes everything else
too, because `revcheck` asks whether all of a unit's artifacts name the same
instrument revision. That was 4 harness stubs, a coverage probe, a partition
probe, 4 sanitised mutation parts and their merge, 4 unsanitised parts and
theirs, 3 post-integration runs, a PRINT replay, and 8 gate runs at ~150–290 s
each. The gate runs are the fixed cost and they measure nothing new by
construction — which is exactly why the two non-zero red tests reproducing to
the value is worth having (§6).

## 0b. What the THIRD dispatch was for, and the answer it got

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

**17,520 checked, 0 failed, 0 inadmissible.** `harness/ranges.toml` carries
`Ary = { no_oracle_when = ["alloc_Ary == 0", "ErrVar_aviFAIL >= 0"],
and_reference = "ErrVar.aviFAIL < 0", … }` — the ninth judgement kind, added for
this unit at the second dispatch. A case that supplied an unallocated `Ary` and
an admissible entry status, and whose REFERENCE returned `aviFAIL < 0`, is a
case in which the reference ALLOCATEd `Ary` and RETURNed on an arm that assigns
none of it or only a prefix. Those elements are recycled heap.

**On this corpus the exclusion is 1,387 of 17,520** — 5,266 of 16,512 before,
and 4,067 of 13,674 before that. **It FELL, in both share and count, and that is
the base-draw change**: the excluded region is `alloc = 0` plus a reference that
returned `aviFAIL < 0`, which is `not-allowed` (1,224) + `read-failed` (59) +
the 104 whose message assignment the staging capacity refused. `not-allowed` at
`alloc = 0` went from 5,207 to 1,224 when the two `AllowDefault` flags were
reordered, so four thousand cases that had NO oracle for `Ary` now have one.
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
      0  not-allowed           1224      1224
      0  other                 7523       104      <- 0 of 3036 last dispatch
      0  read-failed             59        59
      1  already-alloc         7094      7094
      1  entered-failed          76         0
      1  not-allowed            881       881
      1  read-failed            479       479
                               17520      9841
```

`read-failed` totals **538, against 370 and the first dispatch's 103** — the
READ cell is what the corpus change keeps buying. The **104** in
`alloc = 0, other` are new and are the point of the base-draw change: the
reference set `aviFAIL = -1` and its message assignment was refused by R13's
staging capacity, on both sides alike, so the case passes and the classifier
sees no arm. Full derivation in `harness_partition.txt`.

**THE CLASSIFIER'S FIRST VERSION GUESSED THE MESSAGE STRINGS** and matched none
of them, so 8,302 cases came back `other` and both error cells read as EMPTY.
The substrings are now copied from `ROSCO_Helpers.f90`; the wrong version is
described in the script rather than quietly replaced.

**This is upstream ROSCO's defect, recorded and not repaired (P7).**
`ParseDbAry_Opt` returns an ALLOCATED, UNASSIGNED `REAL(DbKi) :: Ary(:)` on two
live paths. In the shipped program it is harmless — both paths set
`aviFAIL = -1` and every caller returns on that.

## 5. C6 — mutation: 0.8533, the six that died, and the 22 that are left

```
189 mutants, 4 nocompile, 185 behavioural
128 KILLED   17 EQUIVALENT   18 UNREACHABLE   22 SURVIVED     0.8533
```

**THE SIX THIS DISPATCH WAS SENT FOR ARE ALL KILLED.** Each was case (b) — the
harness could not reach it — and each was closed by fixing the inputs, not the
record. What killed each:

| mutant | edit | the input that kills it |
|---|---|---|
| `b5366e74` | `assign_errmsg`: `s.size() > cap` → `>=` | R13's staging-capacity ladder landing on an arm that assigns a **112-byte** message, so one case sizes the buffer TO it |
| `6b5fc8ce` | `lower()`: `c >= 'A'` → `c > 'A'` | a record spelling **`nAn`** at a value position — `'a'` is the only letter of `inf`/`infinity`/`nan` whose uppercase form the bound can drop |
| `76abf5f3` | `match_word`: `p + LEN(w) > len` → `>=` | a record ending **`,inf`** on column 2048, so the word ends exactly at `len` |
| `09fbe682` | `match_word`: `k < w.size()` → `<=` | any record where the match SUCCEEDS |
| `2763d449` | `parse_real`: `p < len` → `<=`, integer part | a record of **2048 digits**, so the scan reaches `p == len` and the mutant reads `rec[2048]` |
| `2f05620e` | `parse_real`: `p < len` → `<=`, before the `.` | likewise |

**Two levers produced all six, and both are stated where they can be disputed.**

```
harness/ranges.toml   has_AllowDefault = { values = [1, 0] }
                      AllowDefault     = { values = [0, 1] }
```
the sibling `ParseInAry_Opt`'s base-draw fix, applied here at last. R13's
256-case block is taken with every other input at its base draw, and that draw
sat on the one arm that assigns no message at all. §4's table has the 104 cases
this bought, and the run's own stderr — `needs 112 bytes, the staging buffer
holds 109/110/111; the assignment is refused` — is the record of it.

```
loop 59aa876   _NUMERIC_LEADS += -inf, Infinity, nAn
               _RECORD_TAILS   = digits, frac, word, wordUC, built to EXACTLY
                                 the width of a fixed-length CHARACTER buffer
                                 the reference copies the line into
```
Twenty new R14 shapes, all reached. The record-tail idea is the one worth
carrying: every record this campaign has ever generated was a short line
blank-padded out to a buffer's width, so a scan stopped **two thousand
characters short of the last byte** and every `p < len` guard was taken on the
same side in every case. The width comes from `record_widths_from`, which is
R12's detector without its "assigned from a `CHARACTER(*)` dummy"
discriminator — this unit's `CHARACTER(MaxLineLength) :: Line` is filled by a
CALLEE, so R12 reports the unit N/A, correctly, while the record it parses is
2048 bytes wide.

**BOTH RECORDS WERE PRICED AGAINST THE REFERENCE BEFORE THE GENERATOR WAS
TOUCHED**, because the harness compares `Ary` with `memcmp` and a planted record
the two sides disagree about is a red harness that says nothing about the
mutants it was planted for. `ieee_word_probe.txt`: ten IEEE words, **10 of 10
agree bit for bit**, NaN payload included. `record_tail_probe.txt`: the four
tail records, **4 of 4 agree** on IOSTAT and on both elements.

**WHAT MOVED THE NUMBER, KILLS BEFORE DECLARATIONS,** measured against the
value-oracle control re-taken on the same corpus, so each step is a subtraction:

```
                                          killed / 185
third dispatch, value oracle only              104        0.5622
the corpus (base draw, IEEE words, tails)      120        0.6486
--sanitize                                     128        0.6919
17 equivalent + 18 unreachable            128 / 150       0.8533
```

**THE SANITISER'S SHARE QUADRUPLED, 2 KILLS → 8, AND THAT IS A CORRECTION TO THE
THIRD DISPATCH'S CONCLUSION.** That dispatch measured `--sanitize` as nearly a
null result and concluded the `index_offset` and `p <= len` survivors were *not*
out-of-bounds differences — "`rec[p + 1]` is inside a 2048-byte blank-padded
record". True of the corpus it had. A `p <= len` that reads `rec[2048]` is out
of bounds only when a scan REACHES 2048, and until this dispatch no record did.
A sanitiser measures what the corpus executes; the two instruments move together
and neither is evidence about the other on its own. The baseline still does not
report under `-fsanitize=address,undefined`, so the option's one refusal did not
fire and the score is a measurement.

**THE 18 UNREACHABLE ARE MEASURED, AND THEY ARE NOW RE-DERIVED BY A SCRIPT
RATHER THAN CARRIED FORWARD.** `run_line_coverage_probe.sh` runs gcov over the
generated test's own object — the test `#include`s `parsedbary_opt.hpp`, so the
file measured IS the file the sweep mutates and `vit integrate` ships — at
`-O0`, because `-O2` line attribution folds and a line reported 0 because the
optimiser merged it is exactly the false `unreachable` this exists to avoid.

```
executable lines RUN      239        (was 224)
executable lines NOT RUN   42        (was  62)
CONTROL   the entry line's count is 17,520 = the case count
CONTROL   NON-survivors on never-executed lines: 1, and it is a NOCOMPILE
          (`esign - edig`, char minus std::string), not a kill
```

**30 → 18: the corpus retired twelve declarations, which is the correct
direction** — an unreachable declaration is a debt against the corpus, not a
credit. `make_unreachable.py` reads the coverage file, REFUSES if it does not
name this corpus, and re-declares from what gcov measured. It learned two rules
the expensive way: a mutant already declared EQUIVALENT is not re-declared
(`vit_mutate.py` refuses both claims on one mutant, and the equivalence is the
one that survives a corpus later reaching the line — `b76a0c84`, `d764720b`),
and a mutant that does not COMPILE is not declared at all, because it is already
out of both sides and declaring it made the merge's arithmetic double-count it
(186 against 185 behavioural — `120c6e2c`).

The coverage probe's own sentence used to read *16,512-case* while the corpus
had moved to 17,520; it is now READ from the artifact. A file that states the
wrong corpus is the file the next dispatch copies its reasoning from.

**THE 17 EQUIVALENCES ARE UNCHANGED AND ARE ARGUED PER MUTANT, WITH THE GREP
THAT WOULD REFUTE EACH** (`mutation/ParseDbAry_Opt.equivalences.md`), and the
sweep re-checked every one against the wider corpus: `declared_but_killed` is
**empty**. The largest group is eight mutants that change one nonzero IOSTAT
into another: `list_read_reals` has one call site and the value it returns has
one read, `if (ErrStatLcl != 0)`. Then four buffer enlargements; `lower()`'s
`'Z'` bound, whose three call sites compare against `i n f t y a e d q` and
never `'z'`; and `p < len` → `p <= len` at `parse_real`'s ENTRY, where the one
caller has already established `p < len`.

**That last split is now load-bearing rather than fastidious.** Five other
`p < len` → `p <= len` mutants were NOT declared at the third dispatch, and this
dispatch killed three of them with a record that ends inside a value. Had they
been declared equivalent alongside their sibling, the score would have risen and
nothing would have been learned — which is exactly what the dispatch instruction
warns against.

**THE 22 THAT ARE LEFT ARE NAMED IN `mutation_survivors.txt`, EACH WITH THE
EXACT RECORD THAT WOULD KILL IT.** 10 in the PRINT record (below); 4 in the
exponent and 3 in `list_read_reals`, which need a record ending inside an
exponent and a `2*1.5;3.0`; 3 in the IEEE payload buffer, a region that did not
execute at all until this dispatch; 1 in `match_word`; 1 in the unit's own body.

**AND ONE OF THE 22 IS AN INSTRUMENT GAP, NOT A CORPUS GAP, AND IT IS NEW.**
`1beac345` writes the NUL terminator one byte late, so `buf[n]` keeps whatever
the uninitialised `char buf[64]` held. Both indices are INSIDE the array, so
AddressSanitizer is silent by construction; catching it needs MemorySanitizer,
which reports an uninitialised READ. `--sanitize` builds with
`address,undefined`. No record kills this one.

**THE TEN PRINT MUTANTS ARE A THIRD CASE AND THEY ARE ESCALATED, NOT DECLARED.**
They are not equivalent — §5b's replay measures every one of them moving between
18 and 44 records. They are not unreachable — the corpus runs those lines and 21
of the 27 gate scenarios do too. And the sweep's oracle compares out-parameters
with `memcmp`, so it cannot see the stream they write. A mutant with a MEASURED
difference that the scoring instrument cannot see has no key in this campaign's
vocabulary, which has exactly two. Raised in `DECISIONS.md` as a proposed
`killed_by` declaration — naming the instrument and the artifact, the way an
equivalence names an argument and an unreachable names a coverage file. **It is
worth 0.8533 → 0.9333 on its own, and it is the only item on the list worth more
than three mutants.**

## 5b. THE SIXTH LAYER, BUILT AT THE THIRD DISPATCH, RE-TAKEN ON THIS SURVIVOR SET

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

**RE-TAKEN THIS DISPATCH, because the survivor set moved.** Baseline **44
records, 0 mismatched** — the separator fix holds — **10 of 10** PRINT-region
survivors killed at 18 to 44 records, **negative control 0 of 12**. The tenth is
`b529317d` (`if (n > 0)`), the fix's own guard, which did not exist when the
replay first ran and is exactly the statement the replay kills; it now dies on
44 of 44 records.

**NOTHING IS FOLDED INTO THE SCORE.** A kill by a different instrument on a
different corpus is not a kill by the sweep, and `mutation/ParseDbAry_Opt.json`
is untouched by the replay: 0.8533 is the number with these ten counted as
survivors. What it buys is that ten survivors nothing compared are now
measured — and, at the third dispatch, one real defect.

**AND IT IS WHY THE TEN ARE A THIRD CASE RATHER THAN A DECLARATION.** This is
the layer that makes "not equivalent" a measurement instead of an intuition. The
campaign has a key for *the programs agree* and a key for *the corpus never
asks*; it has none for *a second instrument on this same translation kills it,
and here is the artifact*. §5 and `DECISIONS.md`.

## 6. C7–C9 — integration and the gate

`vit integrate --apply --reverse-copy`. The flag is required and the wrapper was
**read** rather than assumed (unit #49's practice).

**5,252,000 values / 351 channels / 27 scenarios, 0 mismatched.** Every gate
artifact was re-taken because the instrument revision moved, and every number
was **predicted before its run**:

```
every parsed value + 0.01     2,236,141 of 5,252,000    predicted 2,236,141
survivor 2a9e1695             1,583,216 of 4,732,000    predicted 1,583,216
the `Ary = 0` default arm             0 of 5,252,000    predicted 0
survivor 48542e3d                     0 of 5,252,000    predicted 0
survivor ae3f319a                     0 of 5,252,000    predicted 0
survivor a0007207                     0 of 5,252,000    predicted 0   <- new
```

**THE TWO NON-ZERO NUMBERS REPRODUCING TO THE VALUE IS THE CONTROL ON THIS
DISPATCH'S WHOLE CLAIM.** The gate never reads the case file, and neither the
translation nor the wrapper moved this dispatch — so a corpus change that stayed
inside the corpus had to give back 2,236,141 and 1,583,216 exactly. It did. A
different number here would have meant something other than the corpus had
changed.

**2a9e1695 IS STILL NOT A SURVIVOR.** At the second dispatch it was the READ
region's representative and its gate run was the argument that the region is a
corpus gap the gate covers. The 16,512-case corpus killed it and the
17,520-case one still does, so the run is the control on that argument rather
than the argument itself.

`48542e3d` (`rec + p` → `rec - p` in the repeat-count branch) moves 0, and the
prediction was grounded rather than guessed:
`grep -cE '[0-9][*]' Examples/DISCON*.IN` over all 22 files the 27 scenarios
read returns 0. It is still declared unreachable on re-measured coverage.

**AND ONE NEW RED TEST, FOR THE REGION THIS DISPATCH DID NOT CLOSE.**
`a0007207` is `p < len` → `p <= len` at the EXPONENT's sign — one of two
boundary comparisons still alive after the corpus was given records that end
inside a value at column 2048. The tails reach that column in the integer part,
in the fraction and at a matched IEEE word; they do not reach it inside an
exponent, which needs a record ending `1E`. It moves 0 of 5,252,000, and the
gate's own 22 `DISCON*.IN` files have no line within two thousand columns of the
boundary — **so both instruments miss the same statement**, which is a fact
about every record ROSCO has been given rather than about the translation.

**So the pair says something sharper than last dispatch.** Then: READ = a corpus
gap the gate covers, PRINT = an instrument gap nothing covers. Now: the corpus
has killed the mainline parser mutants AND three of the six boundary ones, and
what is left in the READ region is rare-form and end-of-record statements the
gate's own input files do not contain either — reachable in principle, unreached
by both instruments, and each one now named with the record that would reach it
(`mutation_survivors.txt`).

**The zero on the default arm is expected and the expectation is in the artifact
beside the number.** The arm *runs* — `ROSCO_Helpers.f90:945`, 66 times across
21 of 27 scenarios — but only when `AryLen` is 0, and `AryLen` at these call
sites is `F_NumNotchFilts` / `F_GenSpdNotch_N` / `F_TwrTopNotch_N`, so the arrays
that take a default are the notch arrays of a scenario with zero notch filters,
and both consumers sit behind that same count. The differential harness covers
the arm: the `alloc = 0, other` cases, 7,523 of them on this corpus.

## 6b. E4.5 — the post-integration harness, and a pass set predicted in advance

**17,520 checked, 0 failed.** The red test deletes this unit's own
`vit_copy_scalars_to_errorvariables`, scoped to its wrapper by line range with
the edit count asserted at 1.

The pass set is derived from the re-taken partition **before** the run, again
rather than scaled from the third dispatch's 13,216: `not-allowed`
(1224 + 881), `read-failed` (59 + 479), `already-alloc` (7094) and the **104**
cases of `alloc = 0, other` in which the reference changed `aviFAIL` while its
message assignment was refused = **9,841** must fail; `entered-failed`
(184 + 76) and the 7,419 of `alloc = 0, other` that write no scalar = 7,679 must
pass. `ErrStat` and `size_avcMSG` return 0 in all 17,520 and are supplied as 0,
so `aviFAIL` alone decides the set.

The 104 are the case the third dispatch's derivation had no cell for, and
putting them on the correct side of the prediction was the one judgement in it:
they **write a scalar** and only their MESSAGE was refused, so they fail.

**Measured: 9,841 of 17,520. Exact.** Reverted, rebuilt, green re-taken at 0.

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

* **`Ary`'s undefined elements**, on the two paths that produce them — **1,387**
  cases, down from 5,266 and 4,067, named and counted rather than failing. The
  base-draw change moved four thousand cases out of the excluded region and into
  the compared one.
* **Whatever the 22 surviving mutants would have caught**, and also the 18
  declared unreachable: an unreachable declaration is a **debt against the
  corpus**, not a credit — 30 → 18 this dispatch, re-derived from re-measured
  coverage rather than carried forward. §5 and `mutation_survivors.txt`, where
  each of the 22 now names the record that would kill it.
* **THE RECORD THAT ENDS INSIDE AN EXPONENT.** Three of the four boundary
  shapes are now in the corpus and this one is not; four survivors sit on it, and
  the gate run `a0007207` shows the gate's own input files do not contain it
  either. It is the sharpest thing on this list because it is the one *both*
  instruments miss and neither one says so on its own.
* **`buf[n] = '\0'` written one byte late** (`1beac345`). Both indices are inside
  a 64-byte local, so AddressSanitizer is silent by construction and no corpus
  can reach it either — it needs MemorySanitizer. An INSTRUMENT gap, and the only
  one on this list that no record closes.
* **The `ALLOCATE` failure branch that calls `Int2LStr`** — needs a genuine
  out-of-memory. Translated, unreachable, ungraded.
* **The `Words_Ary` STAT= branch.** Not translated at all, §2.
* **The `UnEc` echo record.** Not translated, §7.
* **The `PRINT` record's SCORE, though no longer its behaviour.** §5b compares
  it — 44 records against gfortran's own output — and every one of its 10
  surviving mutants dies there. What no layer does is COUNT that: the sweep's
  oracle compares out-parameters, so ten mutants with a measured difference are
  scored as survivors. That is the third case §5 escalates, and it is 0.8533 →
  0.9333 of this unit's shortfall.
* **`AryLen` above 32.** A stated narrowing; the reference is defined to at
  least 100,000 (unit #32's probe).

## Files

```
README.md                                this file
done_check.txt                           the done-condition at close
r14_reach_probe.txt                      why the generator change was inert here,
                                         the prediction, and the prediction's error
run_partition_probe.sh                   the partition, as a repeatable script
harness_partition.txt                    the 17,520 cases by (alloc_Ary, arm), and
                                         the 104 the base draw bought
run_line_coverage_probe.sh               gcov over the shipped translation
line_coverage.txt                        239 lines run, 42 never, and the two controls
make_unreachable.py                      re-derive the unreachable declarations FROM
                                         that file, and refuse a stale one
mutation_survivors.txt                   the 22 that are left, by region, each with
                                         the record that would kill it
ieee_word_probe.f90 / .cpp / .txt        ten IEEE words, gfortran's READ against the
                                         shipped parse_real: 10 of 10 bit-identical
run_ieee_word_probe.sh                   both halves, side by side
record_tail_probe.f90 / .cpp / .txt      the four records that END INSIDE A VALUE:
                                         4 of 4 agree on IOSTAT and both elements
run_record_tail_probe.sh                 both halves, side by side
print_conformance.f90                    44 records through the reference's own
                                         list-directed OUTPUT
print_conformance.cpp                    the replay, textually including the
                                         SHIPPED translation
run_print_conformance.sh                 both sides, the mutants, and the control
print_replay.txt                         44 / 0, 10 of 10 PRINT survivors killed,
                                         control 0 of 12
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
harness.*-stub.json                      the four red tests, all at 17,520 cases
mutation.refusal.txt                     189 mutants, and the FIRST dispatch's refusal
                                         to score them against a red baseline
run_wrapper_redtest.sh                   perturb the wrapper, prove red, revert, prove green
parsedbary_opt_callees.WRONG.f90         the bridge VIT generated before the two fixes
probe_unec.txt                           the UnEc arm's coverage
```
