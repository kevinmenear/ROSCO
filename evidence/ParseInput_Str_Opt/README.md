# Unit #58 — `ParseInput_Str_Opt` — evidence

`subroutine ParseInput_Str_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)`,
`rosco/controller/src/ROSCO_Helpers.f90:278` (clean baseline `54dd134`).

Find the line of an input file whose SECOND word is `VarName`, and copy that
line's FIRST word into a **`CHARACTER(*), INTENT(INOUT)`** dummy with a Fortran
**`'(A)'` formatted READ**. Three callees — `FindLine` (#32), `GetWords` (#8),
`Int2LStr` (#10) — all already translated. Live in all 27 scenarios: **168 calls
in total**, six call sites × 28 reads of the input file.

**Disposition: `integrated`.** Nine layers ran, all nine green and red-tested,
and the mutation layer is at **39 / (50 − 5 − 6) = 1.0000** against a threshold
of 1.0, with **no open survivors**, on the sanitised sweep and on the value
oracle alike. First dispatch.

## 0. What this unit IS, and the one line that changes everything

`ParseInput_Str_Opt` is `ParseInput_Int_Opt` (#57) with one declaration changed.
`diff` of the two clean bodies at `ROSCO_Helpers.f90:115-192` and `:278-354`
gives the routine name, the declaration of `Variable`, the default value and its
comment, `TRIM(Variable)` in the PRINT, the FORM of the READ, and
whitespace/comments. Nothing else — not the control flow, not the callee calls,
not the arms, and not the parse-error message, which still says "INTEGER" for a
CHARACTER item (the reference's own text; P7 makes the source the oracle, not
the sense).

But the one line that differs is the item reader, and it does not SWAP the
number parser — it DELETES it:

    READ (Words(1),*   ,IOSTAT=ErrStatLcl) Variable     <- #55, #56, #57
    READ (Words(1),'(A)',IOSTAT=ErrStatLcl) Variable    <- HERE

The second is not list-directed input at all. An `A` edit descriptor with no
field width takes `w = LEN(item)` (F2018 13.7.4.1), and an internal record
shorter than `w` is blank-padded (`PAD='YES'`, the default an internal file
cannot override here), so the transfer always happens and always fills the whole
item. The statement IS `Variable = Words(1)` under Fortran's own
CHARACTER-assignment rule — **and nothing in it can fail.**

So the split of this translation is:

| half | source | why |
|---|---|---|
| the CALLER — constants, `ftrim`, `assign_errmsg`, `int2lstr_trimmed`, the whole subroutine body | `parseinput_int_opt.cpp` (#57) | identical control flow |
| `char_assign` | `findline.cpp` (#32) | that unit MEASURED that the obvious `min`+`memcpy`+`memset` form leaves three unkillable mutants, and wrote the one-loop form that does not (§5A) |
| the ITEM — the `'(A)'` READ and the PRINT record | **this unit's own** | no unit in this campaign has read a CHARACTER item or written two adjacent CHARACTER items before |

Three blocks, **490 bytes**, are re-extracted from their source files and
byte-compared by `check_p4_blocks.py` — a script rather than a sentence, because
a copy asserted is a copy nobody can check (P4). What is NOT a byte copy is
named in that script rather than hidden.

| layer | result | red-tested |
|---|---|---|
| record-form pricing (`record_form_probe.{f90,cpp,txt}`) | **144 of 144** (form, length) pairs agree with gfortran's own `'(A)'` READ on (IOSTAT, LEN_TRIM, hash, 24-byte prefix), out of the same 400-byte `CHARACTER(200) :: Words(2)` object. Sixteen record forms × nine item lengths, **IOSTAT 0 on every one** | the item is pre-set to `'~'` in every byte on both sides, so an UNTOUCHED item is distinguishable from one the READ filled with blanks. `blankrecN` puts 200 non-blank bytes in `Words(2)` and none of them reach an item of length 1024, which is the form that catches a translation reading the 400-byte object instead of the 200-byte record |
| PRINT-record pricing (`print_record_probe.{f90,cpp,txt}`) | **9 of 9** records byte-identical to gfortran's, at item lengths 0, 1, 5, 6, 7, 256 and 1024 | the layout is TWO ADJACENT CHARACTER ITEMS, which no sibling has, so it turns on the one libgfortran rule none of them exercised. The lengths straddle the truncation of the six-byte default, which is the only input this record has |
| differential harness (`harness/ParseInput_Str_Opt.json`) | **14,116 checked, 0 failed, 0 inadmissible** at the FIRST run, clean tree, three callee bridges kept. R4 compares the return value plus 6 out-parameters **plus the stdout RECORD on 10,595 cases**. **No `no_oracle` entry** | **four stubs, all four EXACT POINT predictions**: no-op **13,866**; the PRINT **10,595**, equal to `record_nonempty` in the artifact EXACTLY and from a different code path; the `.NOT. AllowDefault_` arm **2,214**; the READ **1,057**. And the arithmetic control holds to the case: 10,595 + 2,214 + 1,057 = 13,866 |
| mutation (`mutation/ParseInput_Str_Opt.json`) | **50 mutants, 0 nocompile, 39 killed, 5 equivalent, 6 unreachable, 0 open. 39/39 = 1.0000.** Sanitised, green baseline, `--workers 8`, 77 s. `declared_but_killed` and `unreachable_but_killed` both EMPTY, `capped_operators` EMPTY | the score IS the red test (E4.6). TWO undeclared sweeps are committed: `sweep_probe0.json` (52 mutants, 38 killed, 0.731) and `sweep_probe1.json` (50, 39, 0.780) either side of the `char_assign` repair, so what survived is on the record before any of it was excused |
| mutation, VALUE ORACLE (`mutation/ParseInput_Str_Opt.value-oracle.json`) | **39/39 = 1.0000 with `--sanitize` off** — the same (empty) survivor SET, so the score does not rest on the sanitiser | the control is the SURVIVOR SET, not the count |
| line coverage of the translation (`line_coverage.txt`) | 61 executable lines run, 25 never, at `-O0`. Four of the 6 `unreachable` declarations derive from it, re-derived every run | entry-line count **14,116** = the case count; `make_unreachable.py` REFUSES if the coverage file does not name the current corpus, and now also refuses if one of its four site-keyed declarations stops matching a site |
| boundary probe (`boundary_probe.txt`) | the two width mutants MEASURED AT THEIR SITE: `GetWords` called a second and third time per case, with the line one byte longer and the elements one byte wider — **0 of 13,926 cases differ either way** | the probe REFUTED two sentences written into it before it ran (§5C), and both corrections are in the file |
| gate, 27 scenarios (`gate/ParseInput_Str_Opt.json`) | 5,252,000 values / 351 channels, 0 mismatched | **two, run as a pair**: the parsed value redirected to a valid substitute rotor table moves **1,447,771 of 5,252,000** across 117 channels, revert-verified at 0; the default arm moves **0**, predicted exactly. Half of the first prediction is REFUTED — 22 scenarios of 27, not 27 (§6) |
| post-integration (`harness/ParseInput_Str_Opt.postintegration.json`) | 14,116 checked, 0 failed | the reverse copy deleted from this unit's own wrapper: **2,214**, PREDICTED 2,214 from the partition before the run; reverted, rebuilt, green re-taken at 0 |

## 1. C1 — what the plan said, and the half of it that is false

`plan.json` recorded

> every input and output crosses the signature; writes to unit UnEc without
> opening it -- an UNCOMPARED output, not a missing input

The first clause is right: all seven dummies cross, and none cost a fix. The
second is **false**, for the measured reason units #54–#57 each recorded:
`ReadControlParameterFileSub` sets `UnEc = 0` and OPENs it on
`<RootName>.RO.echo` when `CntrPar%Echo > 0` (`ReadSetParameters.f90:358-362`,
clean). The unit never writes to an unopened unit.

**Measured on this unit's own lines**, and this is the strongest reading of the
five: `coverage/line_coverage.json` gives `ROSCO_Helpers.f90:348`
(`IF (PRESENT(UnEc))`) **168** hits across the 27 scenarios and `:349`
(`IF (UnEc > 0) WRITE`) **168** — so `UnEc` is PRESENT at ALL 168 calls and
GREATER THAN ZERO at none of them. Not an uncompared output but a **dead** one
whose only possible C++ counterpart would write a different file. §7.

`bridge_feasible` was `unknown` on the basis that the feature matrix records five
blocking features it cannot attribute to a declaration. Observed: all seven
cross. `Variable` is a `CHARACTER(*)` `INTENT(INOUT)` scalar and crosses as
`char* Variable, int len_Variable`, with the wrapper staging it in and copying it
back automatically; `ErrVar` crosses as the `errorvariables_view_t` that already
existed; `AllowDefault` and `UnEc` are OPTIONAL and cross as
`has_<name>`/`<name>` pairs. `--reverse-copy` IS needed: §6.

## 2. C4/C5 — the translation

`vit translate` prompt in `vit_translate.stdout.txt`; the shipped files are
`translations/ROSCO_Helpers/parseinput_str_opt.cpp` and
`rosco/controller/src/parseinput_str_opt.cpp`. `vit check`: 15 checks ran, no
known-shape defects.

**Three things in the reference are deliberately not translated**, each named
here rather than left to be found: `DEBUG_PARSING` is a `.FALSE.` PARAMETER;
`! PRINT *, "Line: ", TRIM(Line)` is a comment; and the `IF (PRESENT(UnEc))`
echo WRITE (§7).

**One place where the reference reads a variable it has not defined, and it is
harmless.** `CALL GetWords(Line, Words, 2)` runs even when `FindLine` did not
find a line, and `Line` is `INTENT(OUT)` in `FindLine`, assigned only on a match.
`GetWords` blank-fills all `NumWords` elements before it looks at anything, so
`Words` is DEFINED either way, and every read of `Words` below is inside
`IF (ErrVar%aviFAIL >= 0 .AND. FoundLine)`. The call is reproduced rather than
routed around (X1).

**And one thing a naive copy would get wrong, which is why the 200-byte bound is
carried explicitly.** `Words` is `CHARACTER(MaxParamLength) :: Words(2)` — ONE
contiguous 400-byte object — so byte 201 of that object is `Words(2)`, the
parameter NAME `FindLine` just matched. Every shipped caller passes a `Variable`
of length 1024 or 256, both greater than 200, so a `memcpy` of `len_Variable`
bytes from `Words.data()` would splice the parameter name into the value on
EVERY shipped call. The internal file is `Words(1)` alone. The `blankrecN` form
in `record_form_probe.txt` is the row that catches it.

## 3. C5 — the two things that had to be this unit's own

### The reader has no failure mode, and that is a measurement

`READ (Words(1),'(A)',IOSTAT=ErrStatLcl) Variable` is one `A` edit descriptor
with no field width over a one-record, 200-byte, blank-padded internal file.
`w = LEN(item)`, a short record is blank-padded, so the transfer always happens
and fills the whole item — Fortran's own CHARACTER assignment.

That is a claim about the PROGRAM, so it is measured rather than argued, and
twice: `record_form_probe.txt` runs the reference's own READ over sixteen record
forms × nine item lengths and reports **IOSTAT 0 on all 144**, and
`harness_partition.txt` reports **0 cases in the `read-failed` arm** over 14,116.

**The consequence runs through the whole unit.** `IF (ErrStatLcl /= 0)` and its
whole message block are dead IN THE PROGRAM. They are translated anyway (P7:
mirror the source; X1: `Int2LStr` is called rather than inlined) and the mutants
that land in them — including the only consumer of `Int2LStrLen` — are declared
`unreachable`, §5C. It also removes an ambiguity the siblings had to work for:
because `not-allowed` is the only arm that assigns `ErrVar%ErrMsg`, R13's
short-capacity cell holds exactly one arm, and every stub prediction is a POINT
rather than a bracket. Unit #57 needed the identity `A + B = 901` for the cell
this unit does not have.

### The PRINT record is TWO ADJACENT CHARACTER ITEMS

```fortran
PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )// &
         " in input file.  Using default value of ", TRIM(Variable)
```

A layout NEITHER sibling has: #55 writes CHARACTER, an array of integers, then a
CHARACTER; #56 writes CHARACTER then one REAL; #57 writes CHARACTER then one
INTEGER. So it turns on the one libgfortran rule none of them exercised —

    list_formatted_write_scalar, libgfortran/io/write.c:
        if (first_item) { ...; write_char (dtp, ' '); }
        else if (type != BT_CHARACTER || !char_flag
                 || delim_status != DELIM_NONE)
            write_separator (dtp);

— a CHARACTER item following a CHARACTER item takes NO separator when
`DELIM=NONE`, which is the default. The two items abut. DERIVED from that and
then PRICED against gfortran's own record before the corpus was generated:
**9 of 9 byte-identical**.

`Variable = 'unused'` is the statement immediately above the PRINT and is
unconditional, so the value item is the six-byte literal TRUNCATED to
`LEN(Variable)` — "unused" for every shipped caller, and something shorter only
where the corpus makes the item shorter than six. That is the ONE input this
record has, and the corpus supplies it on **808 of 14,116 cases** (`LEN(Variable)`
of 1 or 2). A corpus with a zero in that column would make the record a constant
and every mutant of the value item equivalent.

## 4. C6 — the harness, the partition, and the exclusion this unit does not need

**No `no_oracle` entry.** `Variable` is a `CHARACTER(*)`, `INTENT(INOUT)` scalar,
so it is DEFINED on entry on every path — and this unit's READ has no
untransferred-item case at all, which is a stronger statement than the siblings
could make. Every output has an oracle on every one of the 14,116 cases.

### The partition, and why every prediction is a point

`run_partition_probe.sh` classifies every case by the arm the REFERENCE took,
read off the reference's own returned message and `aviFAIL`.

    arm                        cases   scalar-changed   record   Variable moved   LEN(Var) < 6
    default-warned             10595                0    10595            10595            672
    not-allowed                 2123             2123        0                0            130
    read-ok                     1117                0        0             1057              6
    pre-failed                   190                0        0                0              0
    not-allowed, msg refused      91               91        0                0              0
    read-failed                    0                -        -                -              -
    TOTAL                      14116             2214    10595            11652            808

    noop                  13866   PREDICTED 13866   EXACT
    no-print              10595   PREDICTED 10595   EXACT   = record_nonempty
    no-allowdefault-arm    2214   PREDICTED  2214   EXACT
    no-read                1057   PREDICTED  1057   EXACT
    CONTROL   10595 + 2214 + 1057 = 13866                   EXACT

The three arm-scoped stubs partition the no-op's failing set exactly, from three
separate runs. `no-read` is the sharpest: it is the `read-ok` row's
`Variable moved` count and NOT its case count — 60 of the 1,117 cases that reach
the READ arrive holding exactly what the READ would store, and the stub leaves
`Variable` alone, so those 60 still agree.

`no-print` moves exactly the 10,595 cases the record comparison is the only
oracle for. Without `vit_record = { compare_record = ... }` in
`harness/ranges.toml` the same stub would have moved **zero**.

## 5. C6 — mutation, and the repair that came out of the first sweep

    52 mutants, 38 killed, 14 survivors   sweep_probe0.json, nothing declared
    50 mutants, 39 killed, 11 survivors   sweep_probe1.json, after the repair
    39 / 39 = 1.0000                      after 5 equivalent + 6 unreachable

Full ledger in `mutation_survivors.txt`; the equivalence arguments are in
`mutation/ParseInput_Str_Opt.equivalences.md` where they can be disputed.

### A. Three mutants were DELETED rather than declared

The first draft of `char_assign` was the obvious `std::min` + `memcpy` +
`memset`, and its three surviving mutants — the swapped `min` arguments,
`n > 0` → `n >= 0`, and `len_dst > n` → `>=` — are all behaviour-preserving.
They could have been three equivalence declarations.

**Unit #32's `findline.cpp` already carries a comment naming exactly these three
at `GetWords`' version of the same expression, together with the repair.** The
one-loop form bounded by the destination has no `min` to swap and no zero-length
guard to widen, and its one surviving predicate `i <= len_src` changes an ANSWER
at the truncation boundary this unit's corpus straddles on both sides. The block
was taken byte for byte (`p4_blocks.txt`), and every layer that reads the
translation was re-taken on it: both record probes (144 of 144, 9 of 9), the
harness, the four stubs, the line coverage and the sweep.

The cost of finding this was one sweep. The cost of not finding it would have
been three declarations that read as reasonable and hid a form unit #32 had
already measured as worse.

### B. Five equivalences, each with a killed sibling

Every one has a NAMED SIBLING on the same site or line that is NOT declared and
that the corpus KILLED — `af4a0269` (the `has_AryLen` zero three characters from
the declared `AryLen` zero) and `3c51517a` (the `NumWords` argument two lines
below the declared allocation) are the two that had something at stake.

### C. Six unreachable, two causes, and a probe that refuted its own prose

Four are the dead READ-error arm and what feeds it (§3). Two are widths:
`MaxLineLength` 2048 → 2049 and `MaxParamLength` 200 → 201.

Unit #32 left `'2048' -> '2049'` OPEN in `findline.cpp` rather than declare it,
because there it writes past the CALLER's buffer — undefined behaviour, which
unit #7's rule says cannot be declared equivalent. HERE the buffer is this
frame's own `std::vector`, so the mutant is in bounds and the question is
genuinely about what `GetWords` returns. **That is measurable**, and
`boundary_probe.txt` measures it: one extra `GetWords` call per case each way,
0 of 13,926 differing.

**The probe refuted two sentences written into it before it ran**, and both
corrections are in the file rather than smoothed away:

* `Line`'s byte 2048 is a blank on only **851** of 13,926 cases, so the mechanism
  is not "the scan stops at a separator". It is TRUNCATION into the 200-byte
  element; only a word ENDING at 2048 and shorter than 200 could tell the widths
  apart.
* the 201st byte of `Words(1)` is non-blank on **28** cases, so the corpus DOES
  carry first words of 201 characters or more — the obvious second reason is
  refuted by its own column, and ONE cause is left.

That cause is the corpus's `LEN(Variable)` ladder, `{1, 2, 6, 7, 11}`. **It is a
claim about the CORPUS and not about the PROGRAM**, and the distance is large:
every shipped caller passes 1024 or 256, so a corpus drawing the length from the
shipped range would kill `739190c8` on records it already has.

**It was attacked before it was declared.** `lenvariable_levers.txt` enumerates
the four levers this campaign owns — `ranges.toml`'s `lo`/`hi`, `same_as` and
`text`, and a `baseline.<Unit>.json` state — each read out of the generator's
source, and shows that none can put `LEN(Variable) >= 201` and a 201-character
first word in the SAME case. The nearest miss is instructive: a baseline state
CAN name the extent (unit #49's second-dispatch correction) and CANNOT name the
string's bytes, and `_baseline_state` says so in its own comment. The only lever
that reaches it is `generate.py`'s length ladder, which re-prices the gate for
every unit already scored. Raised in DECISIONS.md with the number that would
justify it.

## 6. C7–C9 — integration and the gate

`vit integrate … --apply --reverse-copy`. **`--reverse-copy` was decided by
READING the emitted wrapper** (unit #49's practice), not assumed from the
sibling: without it there is no `CALL vit_copy_scalars_to_errorvariables`, and
this unit writes the SCALAR `ErrVar%aviFAIL` on the not-allowed arm.
`Variable` needs nothing extra — VIT's wrapper stages a `CHARACTER(*)`
`INTENT(INOUT)` dummy in and copies it back on its own.
`vit_errorvariables_view.f90` was regenerated and its `git diff` is EMPTY, which
is the control that the regeneration was a no-op.

**The red-test design is this unit's own, and the reason is its outputs.** All
six shipped values are file names or a socket address, and `WE_Mode` is 2 in all
22 `Examples/DISCON*.IN`, so corrupting a parsed value does not give a different
answer — `ReadCpFile` fails to open and the scenario dies. That is unit #24's
`PathIsRelative` shape and it leaves `mismatched` at 0, which `loop/done.py`'s
P12 reads as a blind gate. So the perturbation is a **valid substitute table**:
`Examples/Test_Cases/BAR_10/Cp_Ct_Cq.BAR_10.txt` has the same 36 × 26 × 1 grid
and different numbers in every cell.

    parsed value -> BAR_10's table   1,447,771 of 5,252,000, 117 channels,
                                     22 of 27 scenarios,
                                     perturbation_broke_scenarios EMPTY,
                                     revert-verified 0 of 5,252,000
    default arm  'unused' -> 'XXXXXX'   0 of 5,252,000, PREDICTED EXACTLY 0

**HALF OF THE FIRST PREDICTION IS REFUTED**, and that is the finding: 22
scenarios, not 27. The five that did not move — 10, 13, 14, 17, 24 — are all
alive (`gen_torque` and `gen_speed` vary in every one). Three run open-loop
DISCON files, one patches `WE_Mode: 1`, and the fifth is recorded as NOT pinned
down rather than given a sentence that reads like a proof. The general form:
**this unit's parsed value reaches the gate's 13 channels through ONE runtime
consumer, the WE_Mode = 2 EKF's rotor-performance surface, and only in scenarios
whose outputs are not prescribed.** `gate.redtest_predictions.txt`.

The default-arm ZERO was predicted from coverage rather than hoped for:
`ROSCO_Helpers.f90:317` has 168 hits and `:318`–`:330`, the whole not-found arm,
has NO COUNTER AT ALL. The trap it is aimed at is that `OL_Filename` is literally
the string `"unused"` in 19 of the 22 shipped files — a value that comes from the
FILE, through the READ, not from this constant.

## 6b. E4.5 — the post-integration harness

    post-integration          14,116 checked, 0 failed
    the reverse copy deleted   2,214 of 14,116   PREDICTED 2,214, exact
    reverted, rebuilt, green  14,116 checked, 0 failed

The prediction is the partition's `scalar-changed` column summed. The
perturbation is ANCHORED TO THIS UNIT'S OWN WRAPPER — that CALL is generated
into every wrapper in `ROSCO_Helpers.f90` that takes an `ErrorVariables`, and a
`str.replace` would have perturbed a dozen units, measured none of them, and
written a red artifact indistinguishable from the right one (unit #26).

## 7. What none of the layers can see

1. **The `IF (UnEc > 0) WRITE (UnEc,*) …` echo record.** Not translated, because
   `UnEc` is a Fortran UNIT NUMBER and C++ cannot reach the Fortran runtime's
   unit table. Unit #54's family decision, in DECISIONS.md, and this is the
   FIFTH and LAST of the five units carrying the identical statement. Measured
   dead: `:348` runs 168 times and `:349` is evaluated 168 times and true none of
   them. **Closing it needs a Fortran shim that owns the unit, not a corpus
   change.**

2. **The `LEN(Variable)` ladder.** `{1, 2, 6, 7, 11}` against shipped callers at
   1024 and 256. It carries one `unreachable` declaration (`739190c8`) and it is
   the one this unit makes that a wider corpus would overturn. DECISIONS.md.

3. **Five of the 27 scenarios cannot see this unit by value at all** (§6). The
   gate's green on those five says nothing about this unit, and the red test is
   what makes that visible rather than a hit count.

4. **The three deleted mutants** (§5A) are gone from the population, so this
   unit's 50-mutant denominator is not comparable to `sweep_probe0.json`'s 52.
   Both artifacts are committed and the diff is exactly those three ids.

## Files

| file | what it is |
|---|---|
| `README.md` | this |
| `done_check.txt` | the close, captured out of tree by `scripts/capture_done_check.sh` |
| `vit_translate.stdout.txt` | C4, the scaffold and the prompt |
| `check_p4_blocks.py`, `p4_blocks.txt` | the three copied blocks, re-extracted and hash-verified, with what is NOT a byte copy named |
| `record_form_probe.{f90,cpp,txt}`, `run_record_form_probe.sh` | 16 record forms × 9 item lengths priced against gfortran, 144 of 144 agreeing on (IOSTAT, LEN_TRIM, hash, prefix) |
| `print_record_probe.{f90,cpp,txt}`, `run_print_record_probe.sh` | the PRINT record priced the same way, 9 of 9 |
| `harness_partition.txt`, `run_partition_probe.sh` | the arm partition every prediction is read out of, including the DEAD `read-failed` row |
| `redtest_predictions.txt` | the four stub predictions, written before the runs, with the arithmetic control |
| `parseinput_str_opt.{noop,no-print,no-allowdefault-arm,no-read}-stub.cpp` | the four stubs, each one textual edit from the shipped .cpp |
| `run_harness_stub.sh` | runs one stub, hash-verified inside the container, `--no-generate` |
| `line_coverage.txt`, `run_line_coverage_probe.sh` | gcov of the shipped translation under the corpus |
| `parseinput_str_opt.boundary-probe.cpp`, `run_boundary_probe.sh`, `boundary_probe.txt` | the two width mutants measured at their site, and the two sentences the measurement refuted |
| `lenvariable_levers.txt` | the four levers that could have KILLED `739190c8` instead of declaring it, each read out of the generator's source, and why none reaches it |
| `make_unreachable.py` | derives the 6 `unreachable` declarations, refuses a stale coverage file, and refuses a site key that stopped matching |
| `nocompile_ids.json` | empty, and the control is the sweep's own `nocompile: 0` |
| `sweep_probe0.json`, `sweep_probe1.json` | the two undeclared sweeps, either side of the `char_assign` repair |
| `mutation_survivors.txt` | one answer per survivor, and what 1.0000 does not cover |
| `gate.redtest_predictions.txt` | the two gate predictions, and the half of the first that was refuted |
| `run_wrapper_redtest.sh` | plants a defect in the shipped wrapper, measures, reverts, re-takes the green |
