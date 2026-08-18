# Unit #54 — `ParseDbAry_Opt` — evidence

`SUBROUTINE ParseDbAry_Opt ( FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc )`,
`rosco/controller/src/ROSCO_Helpers.f90:882` (clean baseline `54dd134`).

Find the line of an input file whose `AryLen + 1`-th word is `ParamName`,
allocate `Ary(max(AryLen, 1))`, and read `AryLen` double-precision values out of
that line with a Fortran **list-directed READ**. Three callees — `FindLine`
(#32), `GetWords` (#8), `Int2LStr` (#10) — all already translated. Live in all
27 scenarios: **1,232 calls**.

**Disposition: `deferred`.** Four layers ran. Three are green and red-tested;
the primary one is **RED at 4,067 of 13,674**, and the failing set is a
*partition* rather than a defect — every one of them is a case in which the
reference returns an **allocated, unassigned** array. The mutation layer is
**not available**, because `vit_mutate.py` correctly refuses to score against a
non-green baseline. Both facts are measured and neither is worked around.

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ParseDbAry_Opt.json`) | **13,674 checked, 4,067 failed**, 0 inadmissible, against the CLEAN Fortran with all three callee bridges kept. Every failure is on `Ary` and on **no other output**; the 9,607 cases in which every output has an oracle agree on every output | **four stubs**, and each moves EXACTLY the cell in which its behaviour has an oracle: no-op **13,448**; the `.NOT. AllowDefault_` arm deleted **4,872** (+805); the READ deleted **4,109** (+42); the zero-store rule removed **4,070** (+3) |
| mutation | **NOT AVAILABLE — 189 mutants exist and none were scored** (`mutation.refusal.txt`) | — (§5) |
| post-integration (`harness/ParseDbAry_Opt.postintegration.json`) | 13,674 checked, **0 failed** | this unit's own `vit_copy_scalars_to_errorvariables` deleted from its own wrapper: **10,611 of 13,674**, the pass set predicted in the runner's header BEFORE the run and matched to the case; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ParseDbAry_Opt.json`) | 5,252,000 values / 351 channels, **0 mismatched** | **TWO**: every parsed value + 0.01 moves **2,236,141** (43% of the gate), revert-verified at 0; the `Ary = 0` default arm moves **0**, and the artifact carries the argument for the zero (§6) |

**No kernel.** The plan allowed "kernel replay **or** direct-call harness", and
the direct-call harness is the layer taken — as for units #45 through #53. It is
also the right one here for a reason of this unit's own: the behaviour under
test is a **parser**, and the corpus a KGen window would capture is 1,232
invocations of one well-formed input file. The harness supplies 13,674 records
that ROSCO never writes.

---

## 1. C1 — what the plan said, and the one thing it had wrong

`plan.json` recorded

> writes to unit UnEc without opening it -- an UNCOMPARED output, not a missing input

The second half is right and the first half is **false**, measured on this
tree. `ReadControlParameterFileSub` sets `UnEc = 0` and OPENs it on
`<RootName>.RO.echo` when `CntrPar%Echo > 0` (`ReadSetParameters.f90:358-362`,
clean). The unit therefore never writes to an unopened unit; when the arm is
live at all the CALLER has connected it.

And the arm is **never live**: `coverage/line_coverage.json` records
`ReadSetParameters.f90:359` (`IF (CntrPar%Echo > 0)`) reached **28** times and
`:360`, the first statement inside it, **zero**. All 22 `Examples/DISCON*.IN`
set `Echo` to 0. So `UnEc` is 0 at every one of this unit's 1,232 calls and
`IF (UnEc > 0)` is false at all of them.

That changes what the statement is: not an uncompared output, but a **dead** one
whose only possible C++ counterpart would write a *different file*. §7 records
what was done about it.

## 2. C4/C5 — the translation

`vit translate` prompt in `vit_translate.stdout.txt`; the shipped file is
`translations/ROSCO_Helpers/parsedbary_opt.cpp` and
`rosco/controller/src/parsedbary_opt.cpp`.

Four helpers are **copied, not re-derived** (P4): `ftrim` and `assign_errmsg`
from `chkparsedata.cpp` (which copied the second from `checkinputs.cpp`), and
`field` / `nonfinite_text` / `list_directed_real` from
`powercontrolsetpoints.cpp`, where the last is measured against 22,526 records
written by gfortran's own runtime.

Three things had to be exact, and one of them is the unit:

* **`ALLOCATE(Ary(FinalAryLen), STAT=ErrStatLcl)` WITHOUT A PRIOR DEALLOCATE.**
  Allocating an already-allocated object is an *error* in Fortran, so a caller
  who hands in an allocated `Ary` gets a non-zero STAT, keeps the array it
  arrived with — data, extent and bounds — and lands in the branch whose message
  says exactly that. The `ALLOCATED(Ary)` test that picks between the two
  messages is therefore TRUE on the *only reachable* failure. This is not a
  corner: 5,697 of the corpus's 13,674 cases take it.
* **The list-directed READ.** §3.
* **`Ary = 0` is a whole-array assignment** over `SIZE(Ary)`, which is
  `FinalAryLen` on a fresh allocation and the caller's extent when the allocate
  failed.

**Two things in the reference are deliberately not translated, and both are
named here rather than left to be found.** `DEBUG_PARSING` is a `.FALSE.`
PARAMETER, so the block it guards is dead in every build of this tree. And the
`STAT=` branch of `ALLOCATE(Words_Ary(AryLen+1))` has no C++ counterpart:
`std::vector` reports failure by throwing rather than by a status, and writing
`ErrStatLcl = 0; if (ErrStatLcl != 0)` would put a comparison in the file that
no input can make either way — the shape units #1, #4 and #43 each measured as
surviving mutants. The `GetWords` call ABOVE that block **is** translated, and
is made unconditionally as the reference makes it, even though `DEBUG_PARSING`
makes its result unreadable (X1: do not route around a callee).

## 3. C5 — the READ, and the defect the harness found in it

`READ (Line,*,IOSTAT=ErrStatLcl) Ary` is list-directed input of `SIZE(Ary)`
REAL(8) values from an internal file: **one** record, of `Line`'s declared width
`MaxLineLength = 2048`, blank-padded. Both halves of what it does are outputs of
this unit — the error status decides the message, and the array decides
everything else — so the model has to reproduce *which items were assigned*, not
just the verdict.

**Every rule is measured**, not read off the standard.
`fortran_io_probe.f90` runs 31 records through the reference's own `READ` with a
`-987.654` sentinel in every element; `fortran_io_probe2.f90` runs 20 more.
Outputs in `fortran_io_probe.txt` and `fortran_io_probe2.txt`. What they fixed:

```
'1.5   NameHere'   n=2  iostat=5010  1.5, SENTINEL   a transferred value STAYS
'1.0 2.0'          n=4  iostat=  -1  1.0, 2.0, S, S  running out is END, not error
'1.0, ,3.0 Name'   n=3  iostat=   0  1.0, S, 3.0     a null value consumes an item
'2* 8.5 Name'      n=3  iostat=   0  S, S, 8.5       r* + separator is r NULLs
'1.0 / 2.0 Name'   n=3  iostat=   0  1.0, S, S       '/' terminates, no error
'1.0;2.0'          n=2  iostat=5010  1.0, S          ';' ends a value, then fails
'1.0.0 2.0'        n=1  iostat=5010  S               a stray char stores nothing
```

**THE DEFECT, AND THE HARNESS IS WHAT FOUND IT.** The first version of this
parser had one failure kind and stored nothing on it. Three cases of 13,674
disagreed — `evidence/ParseDbAry_Opt/harness.parser-no-zero-store.json` is that
run — and they are exactly the three in which `Ary` arrived **allocated**, so
every element had an oracle and the disagreement was visible at all:

```
c=3979  line ",-./"   ref Ary(2) = 0   got Ary(2) = the value it arrived with
c=3984  line "./\]"   ref Ary(1) = 0   got Ary(1) = ditto
c=4619  line "-./\"   ref Ary(1) = 0   got Ary(1) = ditto
```

Twenty more records (`fortran_io_probe2.f90`) turned that into a rule with no
exceptions:

```
STORED 0.0 then 5010:  '.'  '-.'  '+.'  './'  '-./'  '.,'  '.e1'  '.d0'
NOT STORED,    5010:   '+'  '-'  '+/'  '+,'  'e1'  '..'  '-.e'
```

The discriminator is the **decimal point**, and it is the point alone: libgfortran
does not decide "is this a number" and then convert — it scans, and a decimal
point is enough to reach a state where the field could have ended (`1.` is a
legal real), so it calls `strtod`, which yields 0.0 for a field with no digits,
**and that value is transferred before the error is raised**. Without a point
the scanner rejects before converting. A malformed exponent (`-.e`) and a second
point (`..`, `1.0.0`) also reject before converting.

Removing the rule again is the fourth red test: `harness.no-zero-store-stub.json`
fails **4,070**, which is the baseline's 4,067 **plus exactly those three**.

## 4. C6 — the harness, and why 4,067 is a partition and not a defect

**13,674 cases, 4,067 failing, and every failure is on `Ary`.** No case fails on
`ErrVar_aviFAIL`, `ErrVar_ErrMsg`, `ErrVar_ErrMsg_n`, `ErrVar_ErrStat`,
`ErrVar_size_avcMSG`, `FileLines`, `ParamName` or `FileName`.

`harness_partition.txt`, produced by one `fprintf` in the generated test and a
`--no-generate` rebuild (unit #47's probe), classifies every case by whether
`Ary` arrived allocated and by which arm the reference's own `ErrMsg` says it
took:

```
  alloc  arm             cases   Ary-failing
      0  not-allowed      4006     4006      <- 100%
      0  read-failed        61       61      <- 100%
      0  other            3003        0
      1  already-alloc    5697        0
      1  not-allowed       805        0
      1  read-failed        42        0
      1  other              60        0
```

A case fails **if and only if** the reference ALLOCATED `Ary` on this call and
then returned on a path that assigns none of it (`.NOT. AllowDefault_`) or only
a prefix of it (the READ error arm). Those elements are **undefined** in the
reference — the bytes are recycled heap and the recorded diffs are pointer
values, e.g. `700b6d2e4bf30000` — so the comparison decides nothing about either
implementation. There is no coincidental agreement anywhere: 100% of both cells,
0% of the other five.

**Why this is NOT excused with `no_oracle`.** The RUNBOOK's rule from unit #51
is that `no_oracle` is unavailable to a unit whose excused output *is* its whole
answer, and `Ary` is this unit's whole answer: excusing it would leave the
primary layer green over the four error messages and nothing that the unit
parses. The alternative unit #51 took — a domain pin that removes a region
containing no statement — was **attempted and does not apply**:

* `alloc_Ary = { lo = 1, hi = 1 }` in a copy of `harness/ranges.toml` produced a
  corpus **identical** to the unpinned one, 13,674 cases and 4,067 failures.
  `alloc_Ary` is a synthetic descriptor flag, not a signature parameter, and
  `constrain()` matches by parameter name — so the pin was silently not applied.
  Unit #47's rule read from the other end: the run's own count is what said so.
  The file and its artifact were deleted rather than kept, because an artifact
  named `.allocated.json` reporting the unpinned number is a trap.
* Even had it applied it would have cost the 3,003 `alloc = 0, other` cases,
  which are the only ones where the reference's own `ALLOCATE(Ary(FinalAryLen))`
  sets the extent and `Ary = 0` then defines every element. `FinalAryLen`, and
  with it the `IF (AryLen < 1)` arm, is observable **only** there.

**So the number stands as it is, and what it certifies is stated exactly: 9,607
of 9,607 cases in which every compared output has an oracle agree on every
compared output.** That is not a green artifact and is not claimed as one.

**This is upstream ROSCO's defect, recorded and not repaired (P7).**
`ParseDbAry_Opt` returns an ALLOCATED, UNASSIGNED `REAL(DbKi) :: Ary(:)` on two
live paths. In the shipped program it is harmless — both paths set
`aviFAIL = -1` and every caller returns on that — but it is the sixth upstream
observation this campaign has recorded, and it is the one that decides this
unit's disposition.

Two pins, both in `harness/ranges.toml` with their reasoning:
`AryLen = { lo = 0, hi = 32 }`, inherited from `[FindLine]` because this unit's
first statement passes `AryLen` straight into it and unit #32's thirteen-value
probe is therefore a probe of this domain too; and `UnEc = { lo = 0, hi = 0 }`,
which is §7.

## 5. C6 — mutation: the refusal, run rather than described

```
189 mutant(s) from parsedbary_opt.cpp
baseline is not green (ok); refusing to score
```

`mutation.refusal.txt`. `vit_mutate.py` requires `base["failed"] == 0`, and the
requirement is right: with a red baseline every kill would be attributable to
the harness rather than to the mutant. So there is **no mutation score for this
unit**, 189 mutants are untried, and their survival is UNKNOWN rather than
"none". `mutation/ParseDbAry_Opt.json` does not exist, which is what P12 reads.

The marker did its job: `mutate_guarded.sh` raised
`.loop-run/MUTATE_IN_PROGRESS`, the sweep exited 2, and the marker cleared only
after the translation hashed back to `6abb3328`.

## 6. C7–C9 — integration and the gate

`vit integrate --apply --reverse-copy`. The flag is required and the wrapper was
**read** rather than assumed (unit #49's practice): the one `INTENT(INOUT)`
derived-type dummy is `TYPE(ErrorVariables)` and the unit writes the SCALAR
`ErrVar%aviFAIL`, so `vit_copy_scalars_to_errorvariables` has to be in the
emitted wrapper. It is.

Gate: **5,252,000 values / 351 channels / 27 scenarios, 0 mismatched.** Two
perturbations, both in this unit's own translation unit, both revert-verified:

```
every parsed value + 0.01     2,236,141 of 5,252,000 moved   (43%)
the `Ary = 0` default arm     0 of 5,252,000
```

The first is the unit's whole product reaching the controller: the arrays it
fills are `CntrPar`'s gain schedules, notch frequencies and IPC ramps.

**The zero is expected, and the expectation is in the artifact beside the
number** (unit #43's rule). The default arm *runs* — coverage records
`ROSCO_Helpers.f90:945` 66 times across 21 of the 27 scenarios — but it is
reached only when `AryLen` is 0, and `AryLen` at these call sites is
`F_NumNotchFilts` / `F_GenSpdNotch_N` / `F_TwrTopNotch_N`. So the arrays that
take a default are exactly the notch-filter arrays of a scenario with **zero
notch filters**, and both consumers sit behind that same count:
`prefiltermeasuredsignals.cpp:148` inside a loop bounded by `F_GenSpdNotch_N`,
and `checkinputs.cpp:329` inside `if (CntrPar->F_NumNotchFilts > 0)`. Zero
iterations and a false guard. The differential harness is the layer that covers
the arm: the 3,003 cases of the `alloc = 0, other` cell.

*(The first take of that red test had its `--note` eaten by backtick expansion
inside a double-quoted shell argument — `` `Ary = 0` `` ran `Ary` as a command.
Re-taken with the notes intact; the count was 0 both times.)*

## 6b. E4.5 — the post-integration harness, and a pass set predicted in advance

**13,674 checked, 0 failed.** The red test deletes this unit's own
`vit_copy_scalars_to_errorvariables`, scoped to its wrapper by line range with
the edit count asserted at 1 (the same CALL is generated into a dozen wrappers
in that file).

`run_wrapper_redtest.sh`'s header states the expected pass set **before** the
run, from the partition table: the cases reaching an arm that writes `aviFAIL`
are `not-allowed` (4006 + 805), `read-failed` (61 + 42) and `already-alloc`
(5697) = **10,611**; the `other` cells (3003 + 60) = 3,063 write no scalar and
must still pass.

Measured: **10,611 of 13,674.** Exact. Reverted, rebuilt, green re-taken at 0.

Note what this layer's green over the 4,067 undefined cases means and does not
mean: after integration both sides run the same C++, so both allocate through
the same allocator and both leave the same bytes. That is precisely what this
layer is for — it measures the *wrapper*, not the arithmetic — and it is why the
4,067 appear only in the pre-integration artifact.

## 7. Two tool defects and one statement not translated

**VIT's callee-bridge generator was wrong twice, and both are fixed in the VIT
repo rather than worked around (X2).** `FindLine` is the first bridged procedure
in either campaign with (a) a CHARACTER dummy whose length is a module PARAMETER
and (b) a by-reference LOGICAL dummy. The wrong artifact is kept:
`parsedbary_opt_callees.WRONG.f90`.

```
CHARACTER(maxlinelength) :: local_Line
Error: Symbol 'maxlinelength' at (1) has no IMPLICIT type      vit@1e30f79
CALL FindLine(..., FoundLine, ...)
Error: Type mismatch in argument 'foundline'; passed INTEGER(4) to LOGICAL(4)   vit@3f6e2ce
```

The first is `USE <module>, ONLY : <callee>` doing exactly what `ONLY` is for.
The name is now IMPORTED rather than substituted, so a module that changes its
own constant changes both together. **Additive, checked rather than argued**:
regenerating `ChkParseData`'s callee bridge before and after gives a
byte-identical file. The second is the mirror of a conversion the generator
already did in three other places, and its own comment said so.

**The `UnEc` echo WRITE is NOT translated.**

```fortran
IF ( PRESENT(UnEc))  THEN
    IF ( UnEc > 0 )  WRITE (UnEc,*)  LineNum, Tab, ParamName, Tab, Ary
END IF
```

`UnEc` is a Fortran UNIT NUMBER and the record has to go to whatever file the
caller connected it to. C++ has no access to the Fortran runtime's unit table,
so the only record this side could write is one to `fort.<UnEc>` — a *different
file* from the reference's, whenever the arm is live. Writing to the wrong file
is worse than not writing. Nothing is emitted, not even a guarded no-op, because
a translated `if (UnEc > 0) { }` would be a mutable comparison no input could
kill. `harness/ranges.toml` holds `UnEc` at 0 so the reference does not write a
record the translation has no counterpart for.

The arm is dead in every configuration this campaign tests (§1). **It is not
dead in ROSCO**: a user who sets `Echo = 1` gets an echo file whose array lines
stop appearing. The four sibling `Parse*_Opt` units (#55–#58) carry the
identical statement, so this is a family decision; it is raised in `DECISIONS.md`
as one.

**The `PRINT` IS translated**, because it is live — 66 executions across 21
scenarios — and it goes to unit 6, which C++ can reach. Its record layout is
measured in `fortran_io_probe.f90` rather than recalled: one leading blank, the
character items raw, each REAL a self-contained 26-byte field, and ONE separator
blank before a character item that follows a real (the six blanks before the `]`
against five after the reals in the echo record).

## 8. What none of the layers can see

* **`Ary`'s undefined elements, on the two paths that produce them.** §4. No
  instrument in this campaign can adjudicate them and none is claimed to.
* **Anything the mutation sweep would have graded.** 189 mutants, none scored.
  §5. This is the single largest gap in this unit's evidence.
* **The `ALLOCATE` failure branch that calls `Int2LStr`.** It needs a genuine
  out-of-memory: with `AryLen` in [0, 32] the request is at most 33 doubles, and
  the reachable failure is always "already allocated", which takes the *other*
  branch. Translated (P7), unreachable, ungraded.
* **The `Words_Ary` allocation-failure branch.** Not translated at all, §2.
* **The `UnEc` record.** Not translated, §7.
* **The `PRINT` record.** Translated and measured against the reference, but no
  layer *compares* it: the harness compares out-parameters and the gate compares
  simulation channels.
* **`AryLen` above 32.** A stated narrowing; the reference is defined to at
  least 100,000 (unit #32's probe).

## Files

```
README.md                                this file
done_check.txt                           the done-condition at close
vit_translate.stdout.txt                 the scaffold prompt, as generated
fortran_io_probe.f90 / .txt              31 records through the reference's own READ,
                                         plus the PRINT and echo record layouts
fortran_io_probe2.f90 / .txt             20 more, isolating the zero-store rule
harness_partition.txt                    the 13,674 cases by (alloc_Ary, arm)
harness.parser-no-zero-store.json        the harness BEFORE the parser fix, 4,070
run_harness_stub.sh                      one stub through the harness, --no-generate
parsedbary_opt.noop-stub.cpp             every statement removed
parsedbary_opt.no-read-stub.cpp          the list-directed READ deleted
parsedbary_opt.no-allowdefault-arm-stub.cpp   the .NOT. AllowDefault_ arm deleted
parsedbary_opt.no-zero-store-stub.cpp    the measured zero-store rule removed
harness.*-stub.json                      the four red tests, all at 13,674 cases
mutation.refusal.txt                     189 mutants, and the refusal to score them
run_wrapper_redtest.sh                   perturb the wrapper, prove red, revert, prove green
parsedbary_opt_callees.WRONG.f90         the bridge VIT generated before the two fixes
```
