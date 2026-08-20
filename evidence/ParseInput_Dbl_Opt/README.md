# Unit #56 — `ParseInput_Dbl_Opt` — evidence

`subroutine ParseInput_Dbl_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)`,
`rosco/controller/src/ROSCO_Helpers.f90:196` (clean baseline `54dd134`).

Find the line of an input file whose SECOND word is `VarName`, and read one
**REAL(DbKi)** out of that line's FIRST word with a Fortran **list-directed
READ**. Three callees — `FindLine` (#32), `GetWords` (#8), `Int2LStr` (#10) —
all already translated. Live in all 27 scenarios: **73 calls per scenario**
(146 in scenario 1), 2,044 calls in total.

**Disposition: `deferred`.** Seven layers ran. **Six are green and red-tested;
the seventh, mutation, is below the campaign's threshold of 1.0 at
110 / 133 = 0.8271** — and that shortfall is this unit's disposition.

**FIRST DISPATCH.** Every number below was taken at loop `10afabe`.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ParseInput_Dbl_Opt.json`) | **11,562 checked, 0 failed, 0 inadmissible**, against the CLEAN Fortran with all three callee bridges kept. R4 compares the return value plus 6 out-parameters — `Variable`, `ErrVar_size_avcMSG`, `ErrVar_aviFAIL`, `ErrVar_ErrStat`, `ErrVar_ErrMsg_n`, `ErrVar_ErrMsg` — **plus the stdout RECORD, on 9,148 cases**. 15 parameters varied, 2 held. **No `no_oracle` entry**: §4 | **four stubs, two predicted EXACTLY and two as brackets whose excesses had to sum**: no-op **11,363** ✓ exact; the PRINT **9,148** ✓ exact; the `.NOT. AllowDefault_` arm **1,937** in a predicted [1845, 2056]; the READ **278** in a predicted [159, 370]; and (1937−1845) + (278−159) = **211**, exact |
| mutation (`mutation/ParseInput_Dbl_Opt.json`) | **181 mutants, 4 nocompile, 177 behavioural: 110 KILLED, 25 EQUIVALENT, 19 UNREACHABLE, 23 SURVIVED, 110/133 = 0.8271.** SANITISED, green baseline, clean tree, `--workers 8`, 121 s. `declared_but_killed` and `unreachable_but_killed` both EMPTY | — the score IS the red test (E4.6). All 23 survivors are answered in `mutation_survivors.txt`, **16 of them by a named record** |
| line coverage of the translation (`line_coverage.txt`) | **200 executable lines run, 38 never** over the 11,562 cases, at `-O0`. The measurement all 19 `unreachable` declarations are DERIVED from, by `make_unreachable.py` | **three controls**: the entry line's gcov count is 11,562 = the case count; NON-survivors on never-run lines that are not nocompile number **0**; and the four nocompile ids were measured independently and the count matches the sweep's own `nocompile: 4` |
| survivor record search (`survivor_record_search.txt`) | **16 of 23 open survivors distinguished over 131,312 records — all 16 by a VALUE, 0 by an ADDRESS.** Nothing folded into the score | the **negative control is built into the shape**: the search takes the record length as a literal `200`, so `479d0b11` — the mutant of the constant `MaxParamLength` itself — cannot be reached and must come back `NONE`. It does |
| mutation, VALUE ORACLE (`mutation/ParseInput_Dbl_Opt.value-oracle.json`) | **110 of 177, 0.8271 — the same number**, and the same corpus and declarations. Run as a control on `killed_by_sanitizer: 5`, and it REFUTED the reading of that field | the control is the SURVIVOR SET, not the count: the two runs disagree by exactly one mutant each way, which is why the totals match. §5b |
| post-integration (`harness/ParseInput_Dbl_Opt.postintegration.json`) | 11,562 checked, **0 failed** | this unit's own `vit_copy_scalars_to_errorvariables` deleted from its own wrapper: **2,056 of 11,562**, PREDICTED 2,056 from the partition before the run; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ParseInput_Dbl_Opt.json`) | 5,252,000 values / 351 channels, **0 mismatched**, 28 s | **TWO, both predicted**: every parsed value + 1.0 moves **1,857,893** across 147 channels, revert-verified at 0; the default arm moves **0**, and the artifact carries the argument (§6) |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness", and
the direct-call harness is the layer taken — as for units #45 through #55. It is
also the right one here for this unit's own reason: the behaviour under test is
a **parser**, and the corpus a KGen window would capture is 2,044 invocations of
one well-formed input file in which the READ never fails.

---

## 0. The siblings, and what was and was not carried across

`ParseDbAry_Opt` (#54), `ParseInAry_Opt` (#55) and this unit are three members
of the same generic interface, and #54 is the closest: it reads the same
**REAL(DbKi)** items through the same list-directed rules.

The discipline was decided in advance and is stated here so it can be checked
rather than trusted. **Six blocks are copied byte for byte from
`translations/ROSCO_Helpers/parsedbary_opt.cpp` and hash-verified — 8,771
bytes** (P4):

| block | bytes | sha256 (first 16) |
|---|---|---|
| `ftrim` | 169 | `0f5b3d09482a1e07` |
| `is_separator` … `match_word` | 729 | `113fe22631571206` |
| `parse_real` | 3,940 | `593792b51d652889` |
| `list_read_reals` | 2,377 | `c80b37ac2aba7093` |
| `field` / `nonfinite_text` | 578 | `c04487deffab8b8c` |
| `list_directed_real` | 978 | `1d714643ef041d8e` |

Nothing about the list-directed READ is re-derived here: unit #54 measured all
31 + 20 of its rules against gfortran's own runtime
(`evidence/ParseDbAry_Opt/fortran_io_probe{,2}.{f90,txt}`) and the item TYPE is
identical. `assign_errmsg` and `int2lstr_trimmed` are the same code with this
unit's name in their diagnostics.

**What is this unit's own, and had to be read off the source:**

* **the READ's internal file is `Words(1)`, not `Line`.** One record of
  `MaxParamLength = 200` bytes, not `MaxLineLength = 2048`. Both siblings read
  the line; this one reads the line's first WORD.
* **one item, not `SIZE(Ary)`** — and that has consequences all the way to §4
  and §7.
* **`GetWords` is called UNCONDITIONALLY**, outside the `IF (FoundLine)` block
  both siblings put it in.
* **the PRINT record is two items, a CHARACTER expression and one REAL**, which
  is a layout no array record can reach: §3.
* **there is no `ALLOCATE`**, so there is no already-allocated arm, no
  `FinalAryLen`, and no `AryLen`.

## 1. C1 — what the plan said, and the half of it that is false

`plan.json` recorded

> every input and output crosses the signature; writes to unit UnEc without
> opening it -- an UNCOMPARED output, not a missing input

The first clause is right. The second is **false**, for the same measured
reason units #54 and #55 recorded: `ReadControlParameterFileSub` sets
`UnEc = 0` and OPENs it on `<RootName>.RO.echo` when `CntrPar%Echo > 0`
(`ReadSetParameters.f90:358-362`, clean). The unit never writes to an unopened
unit; when the arm is live at all the CALLER has connected it.

**And this unit measures the arm one notch more sharply than either sibling
does.** `coverage/line_coverage.json` gives `ROSCO_Helpers.f90:268`
(`IF (PRESENT(UnEc))`) **73 hits per scenario** and `:269`
(`IF (UnEc > 0) WRITE`) **71**. So `UnEc` is PRESENT at 71 of the 73 calls and
GREATER THAN ZERO at none of them: the guard is not merely dead, it is
evaluated 71 times per scenario and false every time. All 22
`Examples/DISCON*.IN` set `Echo` to 0.

Not an uncompared output, but a **dead** one whose only possible C++
counterpart would write a *different file*. §7.

`plan.json`'s `bridge_feasible` was `unknown` on the basis that the feature
matrix records five blocking features it cannot attribute to a declaration.
**Observed: all seven dummies cross**, and none of them cost a fix — the two
VIT callee-bridge defects unit #54 found in `FindLine` (a CHARACTER dummy whose
length is a module PARAMETER, and a by-reference LOGICAL dummy) were fixed at
`vit@1e30f79` and `vit@3f6e2ce`, and this unit's `FindLine` bridge is correct
first time. `Variable` is a `REAL(DbKi)` scalar `INTENT(INOUT)` and crosses as
`double*`; `ErrVar` crosses as the `errorvariables_view_t` that already existed;
`AllowDefault` (LOGICAL) and `UnEc` (INTEGER) are OPTIONAL and cross as
`has_<name>`/`<name>` pairs. `--reverse-copy` IS needed: §6.

## 2. C4/C5 — the translation

`vit translate` prompt in `vit_translate.stdout.txt`; the shipped files are
`translations/ROSCO_Helpers/parseinput_dbl_opt.cpp` and
`rosco/controller/src/parseinput_dbl_opt.cpp`. `vit check`: 15 checks ran, no
known-shape defects.

**Three things in the reference are deliberately not translated, and all three
are named here rather than left to be found.**

* `DEBUG_PARSING` is a `.FALSE.` PARAMETER, so the `print` it guards is dead in
  every build of this tree.
* `! PRINT *, "Line: ", Line` at `:233` is a comment.
* the `IF (PRESENT(UnEc))` echo WRITE — §7.

**One transcription that looks like a defect and is not.** `Variable = 0` is
commented `! Default of integer inputs is 0 for now` in a routine that parses a
REAL. That is upstream's own copy-paste from `ParseInput_Int_Opt`; the
statement itself is right for a REAL and is transcribed as `*Variable = 0`.

**One place where the reference reads a variable it has not defined, and it is
harmless — checked rather than assumed.** `CALL GetWords(Line, Words, 2)` runs
even when `FindLine` did not find a line, and `Line` is `INTENT(OUT)` in
`FindLine`, which assigns it only on a match. So on the not-found path the
reference hands `GetWords` an uninitialised local. It cannot matter: `GetWords`
blank-fills all `NumWords` elements before it looks at anything, so `Words` is
DEFINED either way, and every read of `Words` below is inside
`IF (ErrVar%aviFAIL >= 0 .AND. FoundLine)`, where `Line` was assigned. The call
is reproduced rather than routed around (X1). The C++ `Line` is a
`std::vector<char>`, so it is zero-filled where the Fortran local is stack
garbage — a difference on a value neither program reads.

## 3. C5 — the two things that are NOT the sibling's

### The record is `Words(1)`, 200 bytes

`READ (Words(1),*,IOSTAT=ErrStatLcl) Variable` is list-directed input of ONE
`REAL(8)` from an internal file made from a `CHARACTER(MaxParamLength)` scalar:
one record, 200 bytes, blank-padded. Both siblings read `Line`, 2048.

That single declaration is why `--sanitize` is nearly useless on this unit and
was decisive on `ParseInAry_Opt`. `Words` is
`CHARACTER(MaxParamLength) :: Words(2)` — one contiguous 400-byte object — so
**byte 201 of the record IS `Words(2)(1:1)`**, the first character of the
parameter name `FindLine` just matched. A mutant that reads `rec[200]` is not
reading past an allocation; it is reading a letter. The sanitised sweep killed
**5** mutants here against the sibling's **41**, and §5 is where that is turned
back into a measurement.

### The PRINT record is a CHARACTER item and one REAL

```fortran
PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )// &
         " in input file.  Using default value of ", Variable
```

Unit #54's measured list-directed-output rules give the record without a new
probe: one leading blank starts it, a CHARACTER item is written raw, there is
**NO separator between a CHARACTER and a REAL that follows it**, and a REAL is
a self-contained 26-byte field (21 right-justified plus 5 trailing blanks).

**That derivation was checked against gfortran's own record on 9,148 cases and
agreed byte for byte on the first run**, because `harness/ranges.toml` carries
`vit_record = { compare_record = ... }` for this unit. Unit #55 needed a
purpose-built 34-record side instrument at its FOURTH dispatch to see this
region at all, and its first run there found a real defect. Here the comparison
is inside the primary layer from the first run, and the number that makes it
worth something is not 11,562 but **9,148** — the cases that actually write a
record. `emit.py` refuses a run in which that count is zero (P10).

`Variable = 0` is the statement immediately above the PRINT and is
unconditional, so the only value the statement can ever write is `+0.0`. That
is a property of the PROGRAM, and §7 says what it costs.

## 4. C6 — the harness, and the exclusion this unit does NOT need

**Neither sibling could compare its `Ary` on every case.** Both `ALLOCATE` it
and then RETURN on arms that assign none of it or only a prefix, leaving
recycled heap that decides nothing, and both carry a `no_oracle_when` entry
that names 1,283–4,067 cases.

**This unit needs none, and the reason is one declaration.** `Variable` is a
scalar `REAL(DbKi), INTENT(INOUT)`, so it is DEFINED on entry on every path.
The three ways gfortran's list-directed READ leaves an item untransferred — an
END condition, a null value, a `/` terminator — leave it holding the CALLER's
own value. **Every output of this unit has an oracle on every one of the 11,562
cases**, and `harness/ranges.toml`'s `[ParseInput_Dbl_Opt]` block says so.

### The partition, and the four red tests

`run_partition_probe.sh` classifies every case by the arm the REFERENCE took.
It is a script rather than a sentence because the table is what the red tests'
predictions are read out of, and a probe whose method is a sentence expires
with the corpus it was run on.

    arm                          cases   scalar-changed   record   Var moved   Var IN /= 0
    default-warned                9148                0     9148         962           962
    not-allowed                   1845             1845        0           0           166
    not-allowed, msg refused        92               92        0           0             0
    read-failed, msg refused       119              119        0           0             0
    pre-failed                     184                0        0           0            37
    read-ok                        174                0        0         159             0
    TOTAL                       11,562             2056     9148        1121         1165

The probe itself could not split its 211-case `error, msg refused` cell: both
the `.NOT. AllowDefault_` arm and the READ-error arm land there when R13 sizes
the staging buffer below the message, and neither writes a record. **So the two
arm-scoped red tests were predicted as BRACKETS whose excesses must sum to
211**, which is an arithmetic control on the arm attribution rather than two
independent guesses:

    noop                  11,363   PREDICTED 11,363          exact
    no-print               9,148   PREDICTED  9,148          exact
    no-allowdefault-arm    1,937   PREDICTED [1845, 2056]  =>  A =  92
    no-read                  278   PREDICTED [ 159,  370]  =>  B = 119
    CONTROL               A + B = 211                        exact

**`no-print` is the sharpest of the four**: it deletes the PRINT and nothing
else, `Variable = 0` on the line above is untouched, and it moves exactly the
9,148 cases the record comparison is the only oracle for. Without the
`vit_record` entry the same stub would have moved **zero**.

**And the number to read before any survivor is explained: 293 of 11,562 cases
(2.5%) reach the list-directed READ.** `FoundLine` requires `FindLine` to match
the SECOND word of some line against `VarName`, and only R14's planted-word
cases do that. The READ is most of this translation's lines and 2.5% of its
corpus.

## 5. C6 — mutation: 110 of 133, and where the 23 live

    110 of 177 = 0.6215   the sweep, nothing declared
    110 of 133 = 0.8271   + 25 equivalent + 19 unreachable

**The cap is stated rather than left to be found.** `cppmutate` offers 40
mutants per operator and this translation has **273 sites** —
`compare_op` 86, `const_tweak` 81, `negate_cond` 45. So **92 mutants were never
enumerated and their survival is UNKNOWN, not none** (P6);
`scripts/mutation_cap_audit.py` prints the table. The cap is not raised here:
every other unit in this campaign was scored at it, and changing a verification
default mid-run is X3.

**The 19 `unreachable` declarations are DERIVED, not asserted.**
`run_line_coverage_probe.sh` measures gcov over the shipped translation under
the 11,562-case corpus at `-O0`; `make_unreachable.py` turns the never-run
lines into declarations, and refuses if the coverage file does not name this
corpus. Three controls, all read after the sweep rather than asserted:

* the entry line's gcov count is **11,562** = the case count;
* NON-survivors on never-run lines that are not nocompile: **0**;
* the four nocompile ids were measured independently by syntax-checking all 181
  mutants (`nocompile_ids.txt`), and the count matches the sweep's own
  `nocompile: 4`.

**One declaration is hand-added and says so.** `4a9b3707` sits at line 308, a
CONTINUATION line gcov marks `-`, under the statement at 307 that gcov marks
`#####`. Loosening the rule to see that would be a second parser for gcov's
output; naming the one mutant is smaller and checkable.

**The 25 equivalences are five families**, each argued in
`mutation/ParseInput_Dbl_Opt.equivalences.md` where it can be disputed, and
each with a NAMED SIBLING ON THE SAME SITE THAT IS NOT DECLARED — which is the
proof that the family is a family and not a bucket:

| family | n | turns on | the sibling NOT declared |
|---|---|---|---|
| IOSTAT | 8 | the status is read only through `!= 0` | `return 0` at :296 |
| `n == 1` | 4 | the reference transfers ONE item, so the loop body runs once and what it then does to `p` is dead | `for (long k = 0; …)` at :343 |
| `p == 0` | 5 | `GetWords` left-justifies, so `parse_real` is entered at 0 | the `p < len` guards at 213/214/217/227/237 |
| buffer | 4 | a local buffer one byte larger than a bound it never reaches | `62` → `63` at :195, which moves a real bound |
| dead value | 4 | including the two in `list_directed_real` that turn on `Variable` being `+0.0` at the PRINT | `isnan` at :395 |

**All 23 open survivors are answered** in `mutation_survivors.txt`: 16 by a
named record from the 131,312-record search, 6 as bounded-by-the-space NONEs
left undeclared on purpose, and 1 the search's own negative control. 22 of the
23 are behind the single 2.5% gate above.

**AND TWO OF THEM ARE KILLED BY THE OTHER INSTRUMENT.** `9e00d730` and
`b8766137` (`parse_real:247`, the exponent's sign) were run through the
27-scenario gate character for character out of the mutation artifact, as a
PAIR with a negative control at the same operator:

    9e00d730   1,583,216 of 4,732,000, 131 channels, scenarios 19 and 27 BROKE
    8d81269b           0 of 5,252,000                       -- the control

Both predicted before the runs (`gate.survivor_predictions.txt`). Neither is
folded into the score and neither is a declaration; what the pair buys is that
two survivors are mutants this campaign's own gate kills on the shipped input
file, and the differential corpus is the only instrument that misses them.
`Examples/DISCON.IN` gives `VS_ArSatTq` as `4.30935e+04`, and four such scalars
are read by this unit.

## 5b. The value-oracle control, and what it refuted

`mutation/ParseInput_Dbl_Opt.value-oracle.json` is the same sweep without
`--sanitize`. It was run to confirm a number already in the scored artifact —
`killed_by_sanitizer: 5`, read as "five mutants only the sanitiser can kill",
which predicts 105 — **and it refuted that reading**: the value oracle killed
110 too. The field counts the mutants whose kill the sanitiser REPORTED, not
the mutants only it can reach.

**The two runs disagree by exactly one mutant EACH WAY**, which is why the
totals match and why a score-only comparison would have called them identical
(unit #48's rule: compare failing sets, not counts).

| id | site | sanitised | value oracle | why |
|---|---|---|---|---|
| `5a8abaea` | `list_read_reals:337`, `v[i]` → `v[i+1]` | KILLED | survived | the caller passes `&value`, ONE double, so `v[1]` writes one past the object into the caller's stack frame. Nothing a comparison of the OUTPUTS can see — genuinely sanitiser-only |
| `b03a94c5` | `parse_real:198`, `buf[n]` → `buf[n+1]` | survived | KILLED | the edit leaves `buf[n]` UNINITIALISED, so the string handed to `strtod` ends wherever the stack garbage holds a zero. The plain build's garbage differs from the correct string and the ASan build's does not. **Its kill is a property of the BUILD, not of the program**, and neither run is wrong |

**The union is 111 of 133 = 0.8346 and no artifact can carry it.**
`vit_mutate.py` scores one instrument per run and has no union; the RUNBOOK
already records that gap. The scored artifact stays the sanitised one, because
that is what both siblings score and changing a verification default mid-run is
X3. The union is stated here rather than written into a file that would claim a
tool computed it.

## 6. C7–C9 — integration and the gate

`vit integrate … --apply --reverse-copy`. **`--reverse-copy` was decided by
READING the emitted wrapper** (unit #49's practice), not assumed from the
sibling: this unit writes the SCALAR `ErrVar%aviFAIL` on two arms and
`CALL vit_copy_scalars_to_errorvariables` is the only thing that carries it out
of the view struct. It is present. `Variable` needs nothing — a scalar dummy
crosses as `double*` and the C++ writes through the caller's own address.

The integrated `.cpp` is byte-identical to the translation apart from VIT's
10-line header and the `extern "C"` wrapper it appends;
`vit_errorvariables_view.f90` was regenerated and `git diff` on it is EMPTY,
which is the control that the regeneration was a no-op.

**Two gate red tests, run as a pair because the zero is only worth something
next to the non-zero on the same build:**

    parsed-value   *Variable = value  ->  value + 1.0
                   1,857,893 of 5,252,000, 147 of 351 channels
                   PREDICTED > 1,000,000 and > 100 channels; revert-verified 0

    default-arm    *Variable = 0  ->  12345.0
                   0 of 5,252,000
                   PREDICTED EXACTLY 0

The zero was predicted from `coverage/line_coverage.json` and not hoped for:
`ROSCO_Helpers.f90:236` (`IF (.NOT. FoundLine)`) has 73 hits per scenario and
`:239`–`:243`, the whole not-found arm, has NO COUNTER AT ALL. `DISCON.IN`
supplies every parameter this unit asks for, so `FoundLine` is TRUE at all 73
calls and the gate cannot see the default arm. The differential harness reaches
it on 9,148 cases.

**AND THE 1,857,893 IS A CROSS-UNIT CONTROL NOBODY DESIGNED.**
`ParseInAry_Opt`'s parsed-value red test moved **1,857,893 of 5,252,000 across
147 of 351 channels**; this one — a different unit, a different type, a
different site — moved **1,857,893 across 147**. That is not a copied number.
It says the gate's count for *any parsed control parameter is wrong* is set by
WHEN each scenario first diverges and by which scenarios run long enough to be
compared, not by which parameter moved. Two independent runs agreeing to the
value is a control on the gate's own determinism that neither unit could have
produced alone.

## 6b. E4.5 — the post-integration harness

    post-integration          11,562 checked, 0 failed
    the reverse copy deleted   2,056 of 11,562   PREDICTED 2,056, exact
    reverted, rebuilt, green   11,562 checked, 0 failed

The prediction is the partition's `scalar-changed` column summed. The
perturbation is ANCHORED TO THIS UNIT'S OWN WRAPPER — that CALL is generated
into every wrapper in `ROSCO_Helpers.f90` that takes an `ErrorVariables`, and a
`str.replace` would have perturbed a dozen units, measured none of them, and
written a red artifact indistinguishable from the right one (unit #26). The
runner asserts the count inside the subroutine is exactly 1 and that the count
elsewhere is unchanged.

## 7. What none of the layers can see

Four things, and each is named with what would close it.

1. **The `IF (UnEc > 0) WRITE (UnEc,*) …` echo record.** Not translated,
   because `UnEc` is a Fortran UNIT NUMBER and C++ cannot reach the Fortran
   runtime's unit table; the only record this side could write is one to
   `fort.<UnEc>`, a different file from the `<RootName>.RO.echo` the caller
   connected. Unit #54's family decision, in DECISIONS.md, and this is the
   THIRD of the five units carrying the identical statement. Measured dead:
   `:268` runs 71 times per scenario and `:269` never fires, and
   `harness/ranges.toml` holds `UnEc` at 0 so the reference does not write a
   record the translation has no counterpart for. **Closing it needs a Fortran
   shim that owns the unit, not a corpus change.**

2. **The 92 mutants the operator cap never enumerated.** Their survival is
   UNKNOWN. Closing it needs `--offset`/`--limit` slices and a merge whose
   population is asked of `cppmutate` UNCAPPED — `scripts/_mutation_merge.py`
   currently asks it at the default 40, so the merge cannot express the full
   population today.

3. **`list_directed_real`'s E-form and its non-finite words.** Lines 426–436,
   395–398 and 407 are `#####` over the whole corpus, and this is a property of
   the PROGRAM rather than of the corpus: the only value the PRINT can ever
   write is `+0.0`, because `Variable = 0` is the statement above it. Eight
   mutants live there and are declared — five `unreachable` and three
   `equivalent`. **Nothing can close it while the reference writes a constant**,
   and that is the right answer rather than a gap.

4. **The 23 open mutants**, each of which now names the record or the argument
   that would settle it (`mutation_survivors.txt`). 22 of them are behind one
   gate: 2.5% of cases reach the READ.

## Files

| file | what it is |
|---|---|
| `README.md` | this |
| `value_oracle_prediction.txt` | the prediction the value-oracle run refuted, and what it measured |
| `done_check.txt` | the close, captured out of tree by `scripts/capture_done_check.sh` |
| `vit_translate.stdout.txt` | C4, the scaffold and the prompt |
| `harness_partition.txt` | the arm partition every prediction is read out of |
| `run_partition_probe.sh` | how it is re-measured when the corpus moves |
| `redtest_predictions.txt` | the four stub predictions, written before the runs, and what they measured |
| `parseinput_dbl_opt.{noop,no-print,no-allowdefault-arm,no-read}-stub.cpp` | the four stubs, each one textual edit from the shipped .cpp |
| `run_harness_stub.sh` | runs one stub, hash-verified inside the container, `--no-generate` |
| `line_coverage.txt` | gcov of the shipped translation under the corpus |
| `run_line_coverage_probe.sh` | how it is re-measured |
| `make_unreachable.py` | derives the 19 `unreachable` declarations from that file |
| `nocompile_ids.txt` | the four nocompile mutants, measured, with the control |
| `survivor_record_search.{cpp,txt}` | the 131,312-record search and its result |
| `run_survivor_record_search.sh` | builds it against the shipped .cpp and each survivor |
| `mutation_survivors.txt` | one answer per open survivor |
| `run_wrapper_redtest.sh` | plants a defect in the shipped wrapper, measures, reverts, re-takes the green |
