# Unit #57 — `ParseInput_Int_Opt` — evidence

`subroutine ParseInput_Int_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)`,
`rosco/controller/src/ROSCO_Helpers.f90:115` (clean baseline `54dd134`).

Find the line of an input file whose SECOND word is `VarName`, and read one
**INTEGER(IntKi)** out of that line's FIRST word with a Fortran **list-directed
READ**. Three callees — `FindLine` (#32), `GetWords` (#8), `Int2LStr` (#10) —
all already translated. Live in all 27 scenarios: **1,680 calls in total**.

**Disposition: `integrated`.** Eight layers ran, all eight green and
red-tested, and the mutation layer is at **98 / (143 − 37 − 8) = 1.0000** against
a threshold of 1.0, with **no open survivors**. The two the first dispatch left
open were ONE gap — no corpus this campaign had generated contained a CR or an
LF byte — and it is closed by four R14 records rather than by a declaration. §5
and §7.

## 0. What this unit IS, and where each half came from

`ParseInput_Int_Opt` is `ParseInput_Dbl_Opt` (#56) with one declaration changed.
`diff` of the two clean bodies at `ROSCO_Helpers.f90:115-192` and `:196-274`
gives the routine name, the declaration of `Variable`, the word INTEGER/REAL in
one message, and whitespace. Nothing else — not the control flow, not the callee
calls, not the arms.

So the translation is a splice of two siblings, and the split is the RUNBOOK's
own discriminator (**item TYPE against record GRAMMAR**):

| half | source | why |
|---|---|---|
| the CALLER — constants, `ftrim`, `int2lstr_trimmed`, the subroutine body | `parseinput_dbl_opt.cpp` (#56) | identical control flow |
| the ITEM — the whole list-directed INTEGER reader, and the INTEGER output field | `parseinary_opt.cpp` (#55) | that unit MEASURED both against gfortran for an `INTEGER(4)` item, over 113 records |

**Nothing item-typed comes from the REAL sibling.** Unit #55 broke by copying
unit #54's separator set (`;` terminates a REAL and not an INTEGER, wrong on 3
of 113 records); unit #56 held by reusing measured record GRAMMAR. Six blocks,
**6,656 bytes**, are re-extracted from their source files and byte-compared by
`check_p4_blocks.py` — a script rather than a sentence, because a copy asserted
is a copy nobody can check (P4).

The one thing that is neither sibling's is the PRINT record — a CHARACTER item
followed by ONE INTEGER with nothing after it — and it is DERIVED from unit
#55's measured grammar rules and then priced.

| layer | result | red-tested |
|---|---|---|
| record-form pricing (`record_form_probe.{f90,cpp,txt}`) | **41 of 41** candidate corpus records agree with gfortran's own list-directed READ on (IOSTAT, value), out of the same `CHARACTER(200) :: Words(2)` storage the unit reads from. First run, no defect found | the item is pre-set to `-987654` on both sides, so an UNTOUCHED slot is distinguishable from a stored value — which is the whole content of the INTEGER reader's failure mode |
| PRINT-record pricing (`print_record_probe.{f90,cpp,txt}`) | **6 of 6** records byte-identical to gfortran's, including `-2147483648`, the widest value of the type | the derivation is unit #55's measured grammar with no separator between a CHARACTER item and a following INTEGER; a wrong field width would show on case 4 |
| differential harness (`harness/ParseInput_Int_Opt.json`) | **12,367 checked, 0 failed, 0 inadmissible**, clean tree, three callee bridges kept. R4 compares 6 out-parameters **plus the stdout RECORD on 9,143 cases**. **No `no_oracle` entry** | **four stubs, all four EXACT**: no-op **12,140**; the PRINT **9,143**, equal to `record_nonempty` in the artifact EXACTLY and from a different code path; the `.NOT. AllowDefault_` arm **1,935**; the READ **1,062**. The last two were POINT predictions this time rather than brackets — `A = 92` had measured 92 on three previous corpora — and the identity `A + B = 901` holds to the case. It has now held on FOUR corpora |
| mutation (`mutation/ParseInput_Int_Opt.json`) | **144 mutants, 1 nocompile, 143 behavioural: 98 killed, 37 equivalent, 8 unreachable, 0 open. 98/98 = 1.0000.** Sanitised, green baseline, `--workers 8`, 101 s. `declared_but_killed` and `unreachable_but_killed` both EMPTY | the score IS the red test (E4.6). FOUR corpus rounds, each PREDICTED: 89 → 93 (four named, four killed, nothing else), 93 → 95 (three named, two killed — the third prediction refuted), 95 → 96 on the corrected form, 96 → 98 on the four end-of-line records. Round 4's prediction is `corpus_round4_predictions.txt`, committed before the corpus was regenerated |
| mutation, VALUE ORACLE (`mutation/ParseInput_Int_Opt.value-oracle.json`) | **98/98 = 1.0000 with `--sanitize` off** — the same (empty) survivor SET, so the score does not rest on the sanitiser | the control is the SURVIVOR SET, not the count. `killed_by_sanitizer: 3` are killed by value as well |
| line coverage of the translation (`line_coverage.txt`) | 164 executable lines run, 22 never, at `-O0`. All 8 `unreachable` declarations derive from it, re-derived every run | entry-line count **12,367** = the case count; `make_unreachable.py` REFUSES if the coverage file does not name the current corpus. Re-derived at round 4 to the SAME eight ids |
| gate, 27 scenarios (`gate/ParseInput_Int_Opt.json`) | 5,252,000 values / 351 channels, 0 mismatched | **two, run as a pair**: every parsed value + 1 moves **370,646 of 728,000** — the prediction was 1,857,893 of 5,252,000 and it is FALSIFIED, which is the finding (§5); the default arm moves **0**, predicted exactly |
| post-integration (`harness/ParseInput_Int_Opt.postintegration.json`) | 12,367 checked, 0 failed | the reverse copy deleted from this unit's own wrapper: **2,744**, PREDICTED 2,744 from the partition before the run; reverted, rebuilt, green re-taken at 0 |

## 1. C1 — what the plan said, and the half of it that is false

`plan.json` recorded

> every input and output crosses the signature; writes to unit UnEc without
> opening it -- an UNCOMPARED output, not a missing input

The first clause is right: all seven dummies cross, and none cost a fix. The
second is **false**, for the measured reason units #54, #55 and #56 each
recorded: `ReadControlParameterFileSub` sets `UnEc = 0` and OPENs it on
`<RootName>.RO.echo` when `CntrPar%Echo > 0` (`ReadSetParameters.f90:358-362`,
clean). The unit never writes to an unopened unit.

**Measured on this unit's own lines**: `coverage/line_coverage.json` gives
`ROSCO_Helpers.f90:186` (`IF (PRESENT(UnEc))`) **1,680** hits across the 27
scenarios and `:187` (`IF (UnEc > 0) WRITE`) **1,624** — so `UnEc` is PRESENT at
1,624 of the 1,680 calls and GREATER THAN ZERO at none of them. Not an
uncompared output but a **dead** one whose only possible C++ counterpart would
write a different file. §7.

`bridge_feasible` was `unknown` on the basis that the feature matrix records
five blocking features it cannot attribute to a declaration. Observed: all seven
cross. `Variable` is an `INTEGER(IntKi)` scalar `INTENT(INOUT)` and crosses as
`int*`; `ErrVar` crosses as the `errorvariables_view_t` that already existed;
`AllowDefault` and `UnEc` are OPTIONAL and cross as `has_<name>`/`<name>` pairs.
`--reverse-copy` IS needed: §6.

## 2. C4/C5 — the translation

`vit translate` prompt in `vit_translate.stdout.txt`; the shipped files are
`translations/ROSCO_Helpers/parseinput_int_opt.cpp` and
`rosco/controller/src/parseinput_int_opt.cpp`. `vit check`: 15 checks ran, no
known-shape defects.

**The P4 copy is checked, not asserted.** `check_p4_blocks.py` re-extracts each
block from its SOURCE file and compares bytes:

    ftrim                168  89564d05f913d9f8  parseinput_dbl_opt.cpp
    int2lstr_trimmed     144  6cb662f2ae533715  parseinput_dbl_opt.cpp
    Sep .. parse_int   2,190  ffb3a56566ae326f  parseinary_opt.cpp
    list_read_ints     3,855  22159e5560106f6b  parseinary_opt.cpp
    field                133  c5e8e5ad99ffde8e  parseinary_opt.cpp
    list_directed_int    166  8e0ef600d4b6b948  parseinary_opt.cpp
                       -----
                       6,656 bytes, all IDENTICAL

The ONE place the copy is not byte-identical is named in the script rather than
hidden inside a block: `list_read_ints`'s doc comment says "`Ary`" in unit #55
and "`Variable`" here, because this unit's item is a scalar dummy. It sits
BETWEEN two checked blocks so the check stays a byte comparison.

**Three things in the reference are deliberately not translated**, each named
here rather than left to be found: `DEBUG_PARSING` is a `.FALSE.` PARAMETER;
`! PRINT *, "Line: ", Line` is a comment; and the `IF (PRESENT(UnEc))` echo
WRITE (§7).

**One place where the reference reads a variable it has not defined, and it is
harmless.** `CALL GetWords(Line, Words, 2)` runs even when `FindLine` did not
find a line, and `Line` is `INTENT(OUT)` in `FindLine`, assigned only on a
match. `GetWords` blank-fills all `NumWords` elements before it looks at
anything, so `Words` is DEFINED either way, and every read of `Words` below is
inside `IF (ErrVar%aviFAIL >= 0 .AND. FoundLine)`. The call is reproduced rather
than routed around (X1).

## 3. C5 — the two things that had to be this unit's own

### The reader is the INTEGER one, and the difference is not cosmetic

`READ (Words(1),*,IOSTAT=ErrStatLcl) Variable` is list-directed input of ONE
`INTEGER(4)` from an internal file made from a `CHARACTER(MaxParamLength)`
scalar: one record, 200 bytes, blank-padded.

Two rules the REAL sibling has and this one does not, both measured by unit #55
and inherited with their measurement rather than adapted:

* `;` is in the REAL reader's separator set and is NOT in the INTEGER's
  (`1;2` is 5010 with nothing stored);
* the REAL reader has a store-then-fail path (a point with no digits reaches
  `strtod`, whose 0.0 is TRANSFERRED before the error). **The INTEGER reader has
  none** — every one of the fourteen 5010 rows in `record_form_probe.txt` leaves
  the item at its `-987654` sentinel. That is what makes the `no-read` stub's
  lower bound exact in §4.

### The PRINT record is a CHARACTER item and ONE INTEGER, with nothing after it

```fortran
PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )// &
         " in input file.  Using default value of ", Variable
```

A layout NEITHER sibling has: #55 writes CHARACTER, an array of integers, then a
CHARACTER; #56 writes CHARACTER then one REAL. It is DERIVED from unit #55's
measured grammar — one leading blank starts the record, a CHARACTER item is
written raw, there is NO separator between a CHARACTER and an INTEGER that
follows it, and an INTEGER is a self-contained 12-byte field with no trailing
blanks — and then PRICED against gfortran's own record before the corpus was
generated: **6 of 6 byte-identical**, including `-2147483648`.

It is also COMPARED on every case that reaches the arm, because
`harness/ranges.toml` carries `vit_record = { compare_record = ... }` for this
unit. Unit #55 needed a purpose-built side instrument at its FOURTH dispatch to
see this region and found a real defect on the first run; #56 and this unit take
the comparison at their first.

`Variable = 0` is the statement immediately above the PRINT and is
unconditional, so the only value the statement can ever write is 0.

## 4. C6 — the harness, and the exclusion this unit does NOT need

**No `no_oracle` entry.** `Variable` is a scalar `INTEGER(IntKi), INTENT(INOUT)`,
so it is DEFINED on entry on every path, and the three ways gfortran's
list-directed READ leaves an item untransferred — an END condition, a null
value, a `/` terminator — leave it holding the CALLER's own value rather than
the recycled heap that forced `Ary = { no_oracle_when = ... }` on both
`Parse*Ary_Opt` siblings. Every output of this unit has an oracle on every one
of the 12,367 cases.

### The partition, the brackets, and the identity

`run_partition_probe.sh` classifies every case by the arm the REFERENCE took,
read off the reference's own returned message and `aviFAIL`.

    arm                          cases   scalar-changed   record   Var moved
    default-warned                9143                0     9143         273
    not-allowed                   1843             1843        0           0
    error, msg refused             901              901        0           0
    read-ok                        296                0        0         253
    pre-failed                     184                0        0           0
    TOTAL                       12,367             2744     9143         526

R13's short-capacity block makes the staging buffer refuse the message on 901
cases, and both the `.NOT. AllowDefault_` arm and the READ-error arm land there
with nothing to separate them, so the two arm-scoped stubs need the identity
**A + B = 901** to split the cell:

    noop                  12,140   PREDICTED 12,140 exactly     EXACT
    no-print               9,143   PREDICTED  9,143 exactly     EXACT
    no-allowdefault-arm    1,935   PREDICTED  1,935            EXACT  => A =  92
    no-read                1,062   PREDICTED  1,062            EXACT  => B = 809
    CONTROL               A + B = 901                           EXACT

**THE LAST TWO WERE POINT PREDICTIONS AT ROUND 4 AND BRACKETS BEFORE IT.** `A`
had measured 92 on three previous corpora of three different sizes while `B`
moved 669 → 711 → 753, so `A = 92` stopped being an interval and became a
hypothesis worth stating as a number that could be wrong. It was not: R13's
capacity ladder is a fixed block of cases at every other input's base draw, so
the arm it lands in is fixed, and every record a corpus ADDS lands in the READ
arm because a record is what the READ reads. Full history in
`redtest_predictions.txt`.

That resolves the cell for free and yields the number every survivor is read
against: **1,105 of 12,367 cases (8.9%) reach the READ**.

`no-print` is the sharpest of the four: it deletes the PRINT and nothing else,
`Variable = 0` on the line above is untouched, and it moves exactly the 9,143
cases the record comparison is the only oracle for. Without the `vit_record`
entry the same stub would have moved **zero**.

## 5. C6 — mutation, and what four corpus rounds bought

    89 of 143, 54 survivors   the first sweep, nothing declared, 11,863 cases
    93 of 143, 50 survivors   round 1, loop 343f843,            12,059 cases
    95 of 143, 48 survivors   round 2, loop 581925d,            12,199 cases
    96 of 143, 47 survivors   round 3, loop 8f9d72f,            12,199 cases
    98 of 143, 45 survivors   round 4, loop b3ad414,            12,367 cases
    98 of  98                 after 37 equivalent + 8 unreachable    = 1.0000

**THE SURVIVOR RECORD SEARCH IS WHAT MADE THE CORPUS CHANGE A PREDICTION RATHER
THAN A HOPE.** `survivor_record_search.{cpp,txt}` drives `list_read_ints`
directly over 59,108 records, in THREE PARTS — records **admissible to the
unit** (`GetWords` produces `<word><blanks>`, left-justified, no interior blank,
no comma, no semicolon), records only the **function** can be handed, and — new
at the second dispatch — records carrying an **end-of-line byte**, which are
admissible and which the first two parts' alphabet could not contain. Unit #56
read a `differs` on a function-only record as a corpus lever twice; the tag
exists because of that.

The end-of-line block is APPENDED after the other two rather than merged into
the alphabet (P5), so every row the equivalence families quote is byte-identical
and only a mutant that NOTHING else distinguishes can take a row from it. Four
do. Two are the survivors this dispatch killed; the other two are `972ae933` and
`27b68fda`, both declared EQUIVALENT at `:289` — a site the OLD space could
never execute, because reaching it needs a record whose every byte is a blank or
an end-of-line. Their differences are one non-zero IOSTAT against another, which
is what family 1 claims and what `ErrStatLcl /= 0` cannot see, so the block
turned two arguments into two measurements.

FIVE survivors named an ADMISSIBLE record, and three rounds of R14 entries
closed them; a fourth round closed the two that no record space had contained.
Every round was PREDICTED, every new form was priced against gfortran first, and
**the second prediction was refuted** — which produced the rule that matters
most in this section:

    round 1  four named, four killed, nothing else moved        89 -> 93
    round 2  three named, TWO killed                            93 -> 95
    round 3  the third, on a corrected form                     95 -> 96
    round 4  the two end-of-line survivors, both killed         96 -> 98

The form written for `ab7420d6` in round 2 was a full-width lead carrying a
repeat count of its own, paired with a `'*'` neighbour, and it could not reach
the bound it was aimed at: **a lookahead that scans for a token STOPS AT that
token**, so the star the record carried is exactly what kept the digit scan from
running to the record's last byte. The scan reaches `len` only on a record with
NO star in it, and the mutant that reads one past the end then finds its star in
the NEIGHBOUR. Corrected, priced, and killed. The rule is now in `generate.py`'s
own comment and in the RUNBOOK.

**THE ADDITIVE CONTROL FAILED AND IS REPORTED RATHER THAN CLAIMED.** The old
case file is not a prefix of the new one, and the reason is structural: entries
added to `_NEIGHBOUR_VALUES` are emitted inside the `n:` block R14 appends after
EVERY plant index, so no ordering makes the change a suffix. This is a different
corpus, and every layer that reads it was re-taken on it.

**The 37 equivalences are seven families**, each argued in
`mutation/ParseInput_Int_Opt.equivalences.md` where it can be disputed, and each
with a NAMED SIBLING ON THE SAME SITE THAT IS NOT DECLARED and that the corpus
KILLED — every one of those sibling ids checked against the scored artifact
rather than asserted.

**The 8 `unreachable` declarations are DERIVED**, and `make_unreachable.py`
refuses if the coverage file does not name this corpus. Two controls, both read
after the sweep: the entry line's gcov count is 12,367 = the case count, and
NON-survivors on never-run lines number **0**. The one nocompile mutant,
`9738141b`, was measured by syntax-checking all 144 against the generated callee
header, and the count matches the sweep's own field.

**A defect in an inherited script, recorded with the wrong behaviour (C12).**
`make_unreachable.py`'s coverage parser matched `^  L\s*(\d+)\s` over the WHOLE
file, and the coverage file's own CONTROL line has that shape — so the unit's
ENTRY LINE, the one line that provably runs on every case, was in the
never-executed set. It cost nothing here or on the sibling only because
`cppmutate` offers no mutant on either entry line. Repaired to split on the
`LINES NEVER EXECUTED:` heading, and to refuse if that heading is absent.

**The two survivors the first dispatch left open were ONE gap, and it is
closed.** `is_eol` accepts CR and LF and no corpus this campaign had generated
contained either byte, so the second disjunct of every blank scan had only ever
been evaluated on one side. Four R14 records close it — `eollead` (a CR then a
digit, which separates `is_eol(rec[p])` from `is_eol(rec[p + 1])` in one
character) and `eolfull` (200 bytes of CR with a `/` neighbour, the only shape
that makes the LEADING scan reach the record's last byte, because a record of
all blanks is not a word). Both mutants die; `mutation_survivors.txt` §A.

**The first dispatch's reading of that gap was wrong and the correction is the
finding.** It recorded `is_eol` as live in unit #55's 2048-byte `Line` and
unreachable in this unit's `GetWords` word, and raised **a P4 copy inherits its
source's REACHABILITY as well as its bytes** as a decision for the Driver. There
is no asymmetry: unit #55's `Line` comes from the same `FileLines`, and
`eol_admissibility_probe.txt` bounds both the same way — libgfortran treats a
bare CR as a RECORD TERMINATOR and strips a CR before the LF, so five written
lines come back as seven records and no end-of-line byte can enter `FileLines`
through `ReadControlParameterFileSub`'s READ at all. `is_eol` is in the reader
because gfortran's INTERNAL list-directed READ treats both bytes as separators,
which both units transcribe from the runtime rather than from their record's
origin. §7.

## 6. C7–C9 — integration and the gate

`vit integrate … --apply --reverse-copy`. **`--reverse-copy` was decided by
READING the emitted wrapper** (unit #49's practice), not assumed from the
sibling: this unit writes the SCALAR `ErrVar%aviFAIL` on two arms and
`CALL vit_copy_scalars_to_errorvariables` is the only thing that carries it out
of the view struct. `Variable` needs nothing — a scalar dummy crosses as `int*`.
`vit_errorvariables_view.f90` was regenerated and its `git diff` is EMPTY, which
is the control that the regeneration was a no-op.

**Two gate red tests, run as a pair, and the first prediction is FALSIFIED.**

    parsed-value   *Variable = static_cast<int>(value)  ->  ... + 1
                   PREDICTED 1,857,893 of 5,252,000, 147 of 351 channels
                   MEASURED    370,646 of   728,000,  22 channels,
                               scenarios 19, 20 and 27 only
                   revert-verified 0 of 5,252,000

    default-arm    *Variable = 0  ->  *Variable = 12345
                   PREDICTED EXACTLY 0     MEASURED 0

All three were re-taken at loop `8f9d72f` after the corpus rounds and gave back
the same values to the digit. The gate never reads the case file, so that is the
control saying what changed was the CORPUS and nothing the shipped controller
does.

The RUNBOOK records that this family's gate answer is set by the FAILURE CLASS
and not by the site, on two numbers each measured twice. **An INTEGER control
parameter is a MODE, not a gain**: adding one to every value this unit parses
selects a different controller path and 23 of the 27 scenarios stop producing
comparable output. So the rule survives and this is a THIRD class:

    every parsed value is wrong        1,857,893 / 5,252,000   147 ch  (REAL)
    the parse FAILS, aviFAIL = -1      1,583,216 / 4,732,000   131 ch
    every parsed MODE is wrong           370,646 /   728,000    22 ch  (INTEGER)

The distinction the earlier runs could not make is between "the failure class"
and "the item type", because both perturbed a REAL. This one perturbs an INTEGER
at the same kind of site and lands somewhere else, which settles it. **Writing
the prediction down is what made this visible**; a red test that cannot be wrong
is not evidence about the instrument.

The default-arm ZERO was predicted from coverage rather than hoped for:
`ROSCO_Helpers.f90:154` has 1,680 hits and `:155`–`:162`, the whole not-found
arm, has NO COUNTER AT ALL. Both artifacts carry their prediction in a `--note`.

## 6b. E4.5 — the post-integration harness

    post-integration          12,367 checked, 0 failed
    the reverse copy deleted   2,744 of 12,367   PREDICTED 2,744, exact
    reverted, rebuilt, green  12,367 checked, 0 failed

The prediction is the partition's `scalar-changed` column summed. The
perturbation is ANCHORED TO THIS UNIT'S OWN WRAPPER — that CALL is generated
into every wrapper in `ROSCO_Helpers.f90` that takes an `ErrorVariables`, and a
`str.replace` would have perturbed a dozen units, measured none of them, and
written a red artifact indistinguishable from the right one (unit #26).

## 7. What none of the layers can see

1. **The `IF (UnEc > 0) WRITE (UnEc,*) …` echo record.** Not translated,
   because `UnEc` is a Fortran UNIT NUMBER and C++ cannot reach the Fortran
   runtime's unit table. Unit #54's family decision, in DECISIONS.md, and this
   is the FOURTH of the five units carrying the identical statement. Measured
   dead: `:186` runs 1,680 times and `:187` never fires. **Closing it needs a
   Fortran shim that owns the unit, not a corpus change.**

2. **The mutants the operator cap never enumerated.** `compare_op` and
   `const_tweak` are both capped at 40. Their survival is UNKNOWN, not none
   (P6).

3. **The comma and semicolon branches of the reader**: 22 never-executed lines
   carrying all 8 `unreachable` declarations, at `:300`, `:311` and `:316`.
   `GetWords` splits on `,` and `;`, so no input to this unit can put either
   character in the record — a property of the PROGRAM, not of the corpus, so no
   corpus change empties the group. This was item 3 of THREE at the first
   dispatch; the `is_eol` half of it is no longer a gap.

4. **The distance between what the unit's DUMMY admits and what its shipped
   CALLER produces, which this dispatch measured for the first time.** The four
   round-4 records are admissible to the unit — `FileLines` is
   `CHARACTER(*), INTENT(IN), DIMENSION(:)`, CHAR(13) is a value it holds, and
   CR is not in `GetWords`' separator set — and are NOT producible by
   `ReadControlParameterFileSub`'s `READ (u,'(A)')`, which treats both bytes as
   record terminators. That is the same bar under a 200-digit word and under
   `nan(aaa…)`, neither of which any `DISCON.IN` contains either; it is stated
   here because it had never been stated. `eol_admissibility_probe.txt` also
   names the ONE shipped path it does not bound: the caller ALLOCATEs one more
   element than the file has lines and the last READ fails, so
   `FileLines(NumLines)` is undefined and `FindLine` reads it.

## Files

| file | what it is |
|---|---|
| `README.md` | this |
| `done_check.txt` | the close, captured out of tree by `scripts/capture_done_check.sh` |
| `vit_translate.stdout.txt` | C4, the scaffold and the prompt |
| `check_p4_blocks.py`, `p4_blocks.txt` | the six copied blocks, re-extracted and hash-verified |
| `record_form_probe.{f90,cpp,txt}`, `run_record_form_probe.sh` | 47 candidate corpus records priced against gfortran BEFORE the generator was touched, 47 of 47 agreeing on (IOSTAT, value) |
| `eol_admissibility_probe.{f90,txt}`, `run_eol_admissibility_probe.sh` | which end-of-line bytes the SHIPPED caller's own OPEN/READ can put in `FileLines` — measured: none |
| `print_record_probe.{f90,cpp,txt}` | the PRINT record priced the same way, 6 of 6 |
| `harness_partition.txt`, `run_partition_probe.sh` | the arm partition every prediction is read out of |
| `redtest_predictions.txt` | the four stub predictions on all FOUR corpora, written before the runs, with the bracket identity and the round-4 point predictions |
| `corpus_round4_predictions.txt` | round 4's kills, controls and case-count effects, committed before the corpus was regenerated |
| `parseinput_int_opt.{noop,no-print,no-allowdefault-arm,no-read}-stub.cpp` | the four stubs, each one textual edit from the shipped .cpp |
| `run_harness_stub.sh` | runs one stub, hash-verified inside the container, `--no-generate` |
| `line_coverage.txt`, `run_line_coverage_probe.sh` | gcov of the shipped translation under the corpus |
| `make_unreachable.py` | derives the 8 `unreachable` declarations from that file, and refuses a stale one |
| `nocompile_ids.json` | the one nocompile mutant, measured, with the control |
| `sweep_probe0.json`, `sweep_probe1.json` | the two undeclared sweeps, kept because §5's arithmetic is over them |
| `survivor_record_search.{cpp,txt}`, `run_survivor_record_search.sh` | the 59,108-record search, split into admissible, function-only and end-of-line |
| `mutation_survivors.txt` | one answer per survivor, what the first dispatch got wrong about the last two, and what 1.000 does not cover |
| `gate.redtest_predictions.txt` | the two gate predictions and the falsification |
| `run_wrapper_redtest.sh` | plants a defect in the shipped wrapper, measures, reverts, re-takes the green |
