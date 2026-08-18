# Unit #54 — `ParseDbAry_Opt` — evidence

`SUBROUTINE ParseDbAry_Opt ( FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc )`,
`rosco/controller/src/ROSCO_Helpers.f90:882` (clean baseline `54dd134`).

Find the line of an input file whose `AryLen + 1`-th word is `ParamName`,
allocate `Ary(max(AryLen, 1))`, and read `AryLen` double-precision values out of
that line with a Fortran **list-directed READ**. Three callees — `FindLine`
(#32), `GetWords` (#8), `Int2LStr` (#10) — all already translated. Live in all
27 scenarios: **1,232 calls**.

**Disposition: `deferred`.** Five layers ran. **Four are green and red-tested;
the fifth, mutation, RAN this time and is far below the campaign's threshold at
77 of 185.** The first dispatch left the primary layer RED at 4,067 of 13,674
and the mutation layer unavailable -- `vit_mutate.py` refuses to score against a
non-green baseline, correctly. This dispatch states the 4,067 as what they are:
a region on which the reference has no answer, named by a CONDITION rather than
excused wholesale, so the primary layer is now green and the mutation layer
could run. What it found is this unit's real gap, and it is a fact about the
CORPUS and the INSTRUMENT rather than about the translation (§5).

| layer | result | red-tested |
|---|---|---|
| differential harness (`harness/ParseDbAry_Opt.json`) | **13,674 checked, 0 failed**, 0 inadmissible, against the CLEAN Fortran with all three callee bridges kept. `Ary` **not compared on 4,067** cases -- and 4,067 is exactly the red set the first dispatch measured and partitioned (§4) | **four stubs**, all re-taken on this comparison and **all four PREDICTED IN ADVANCE and exact**: no-op **13,448**; the `.NOT. AllowDefault_` arm deleted **4,811** = 4006 + 805; the READ deleted **103** = 61 + 42; the zero-store rule removed **3** |
| mutation (`mutation/ParseDbAry_Opt.json`) | **189 mutants, 4 nocompile, 185 behavioural, 77 killed, 108 survived, score 0.416** -- BELOW the threshold, and the number is real: green baseline, clean tree, `compared_against: fortran_reference_on_a_clean_tree` | — the score IS the red test (E4.6). 103 of the 108 survivors are in two regions: the list-directed READ, which only **103 cases of 13,674** can distinguish anything about, and the PRINT record, which **no layer compares**. `evidence/ParseDbAry_Opt/mutation_survivors.txt` |
| post-integration (`harness/ParseDbAry_Opt.postintegration.json`) | 13,674 checked, **0 failed** | this unit's own `vit_copy_scalars_to_errorvariables` deleted from its own wrapper: **10,611 of 13,674**, the pass set predicted in the runner's header BEFORE the run; reverted, rebuilt, green re-taken at 0 |
| gate, 27 scenarios (`gate/ParseDbAry_Opt.json`) | 5,252,000 values / 351 channels, **0 mismatched** | **TWO**: every parsed value + 0.01 moves **2,236,141** (43% of the gate), revert-verified at 0; the `Ary = 0` default arm moves **0**, and the artifact carries the argument for the zero (§6) |

**Every artifact of this unit now names one instrument revision** (loop
`829625b`, `revcheck --unit`). The gate and the post-integration layer were
re-taken for that reason alone; neither number moved.

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

## 4. C6 — the harness: 4,067 is a partition, and the second dispatch STATES it

**FIRST DISPATCH: 13,674 cases, 4,067 failing, and every failure is on `Ary`.** No case fails on
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
containing no statement — **does not apply either**, and for a reason the first
dispatch got half right:

* `alloc_Ary = { lo = 1, hi = 1 }` produced a corpus **identical** to the
  unpinned one, 13,674 cases and 4,067 failures. The first dispatch recorded the
  cause as "`alloc_Ary` is a synthetic descriptor flag, not a signature
  parameter, and `constrain()` matches by parameter name". **That is wrong about
  the mechanism.** `alloc_Ary` *is* a `Param` of the derived signature
  (`harness/vitbridge.py`: `out_params.append(Param(name=alloc, kind="int",
  values=(0.0, 1.0)))`), so `constrain()` matched it. It carries `values`, and a
  flag's cases come from `values` — `lo`/`hi` narrow a ladder it does not have.
  The pin that would have applied is `values = [1]`. The conclusion was right
  and the reason was not; recorded because a wrong mechanism in an evidence file
  is what the next unit copies.
* And it still would not be the right pin, for the reason the first dispatch
  gave: it costs the 3,003 `alloc = 0, other` cases, the only ones where the
  reference's own `ALLOCATE(Ary(FinalAryLen))` sets the extent and `Ary = 0`
  then defines every element. `FinalAryLen`, and with it the `IF (AryLen < 1)`
  arm, is observable **only** there.

**SECOND DISPATCH: the region is stated as a CONDITION, and the harness is
green at 13,674 / 0.** No input-domain narrowing can name this region, because
which cases the reference leaves undefined is decided by the reference's own
control flow. `harness/ranges.toml` now carries the ninth judgement kind
(loop `829625b`):

```toml
Ary = { no_oracle_when = ["alloc_Ary == 0", "ErrVar_aviFAIL >= 0"],
        and_reference  = "ErrVar.aviFAIL < 0", reason = "..." }
```

read as: on a case that supplied an UNALLOCATED `Ary` and an admissible entry
status, and whose REFERENCE came back with `aviFAIL < 0`, `Ary` is not compared.
Everything else about that case still is — `aviFAIL`, `ErrStat`, `ErrMsg`, its
length, `size_avcMSG` — and both sides are still called.

**Three checks on it, and they are why this is a statement rather than a
convenience:**

1. **The corpus did not move.** `parsedbary_opt_cases.bin` hashes
   `64942b97cc3a4c2ab6631f8a452141b3e02335f8fd1acfdde613320aa77390c7` before and
   after. The entry is split out before `constrain()`, so it changes the
   COMPARISON and not one case.
2. **The count is the partition's, exactly.** `no_oracle_when_skipped: {"Ary":
   4067}` — the number measured and classified by the first dispatch, predicted
   before this pin was written. The first form of the condition, without the
   `ErrVar_aviFAIL >= 0` half, excluded **4,233**; the extra 166 are the cases
   that arrived ALREADY FAILED and never entered the reference's body at all
   (`IF (ErrVar%aviFAIL >= 0) THEN` wraps everything), where `Ary` is untouched
   on both sides and the answer is perfectly defined. Stating the reference's
   own guard beside the allocation flag brings it to 4,067.
3. **The condition reads the REFERENCE, never the translation.** `_b` is the
   Fortran side. The excluded set is therefore a property of (corpus × reference)
   and is identical under every mutant and every stub — which is the whole
   reason a mutation score over it is a measurement. An input the calls
   overwrite is snapshotted before them (`vit_supplied_<param>`), so the input
   half reads what the case SUPPLIED and not what either side returned.

**What the green certifies, stated exactly: 13,674 of 13,674 cases agree on
every output that has an oracle, and on 4,067 of them one output — `Ary` — was
not compared because the reference has no answer for it.**

**This is upstream ROSCO's defect, recorded and not repaired (P7).**
`ParseDbAry_Opt` returns an ALLOCATED, UNASSIGNED `REAL(DbKi) :: Ary(:)` on two
live paths. In the shipped program it is harmless — both paths set
`aviFAIL = -1` and every caller returns on that — but it is the sixth upstream
observation this campaign has recorded, and it is the one that decides this
unit's disposition.

Three entries now, all in `harness/ranges.toml` with their reasoning — the
`no_oracle_when` above, and two pins:
`AryLen = { lo = 0, hi = 32 }`, inherited from `[FindLine]` because this unit's
first statement passes `AryLen` straight into it and unit #32's thirteen-value
probe is therefore a probe of this domain too; and `UnEc = { lo = 0, hi = 0 }`,
which is §7.

## 5. C6 — mutation: 77 of 185, and where the other 108 are

The first dispatch could not score at all: `vit_mutate.py` requires
`base["failed"] == 0` and the baseline failed 4,067. `mutation.refusal.txt` is
that run and is kept. With the exclusion of §4 the baseline is green, and the
sweep ran on a clean tree in 142 s.

```
189 mutant(s), 4 did not compile, 185 behavioural
77 killed, 108 SURVIVED     score 0.4162        <- below the campaign threshold
```

**The score is a fact about the corpus and the instrument, and the table says
which.** Every mutant is placed by `harness/cppmutate.py`'s own position, not by
a text search (`mutation_survivors.txt` has all 108, per line):

```
survived / mutants   region
     87 / 151        the list-directed READ  (match_word, lower, is_separator,
                     parse_real, list_read_reals)
     16 /  17        the PRINT record  (field, nonfinite_text,
                     list_directed_real, print_default_warning)
      1 /   6        assign_errmsg
      1 /   1        int2lstr_trimmed
      3 /  10        the unit's own body, its constants and ary_at
```

* **The READ is reached by 103 cases of 13,674 — measured, not estimated.**
  `harness.no-read-stub.json` deletes the whole list-directed READ and moves
  **103** cases. Whatever number of cases enter that code, the number that can
  DISTINGUISH anything about it is 103, or 0.75% of the corpus: `FindLine` finds
  the parameter only where R14 planted it, and R14 reached its `1:plant` shape
  alone in both dispatches. 87 survivors in 151 mutants is what a corpus that
  exercises a parser 103 times looks like. **This is the finding of this
  dispatch** and it was invisible while the baseline was red.
* **The PRINT is compared by nothing.** §8, bullet 6, stated at the first
  dispatch and unchanged: the harness compares out-parameters, the gate compares
  simulation channels, and this record goes to stdout — the same stdout the
  harness reads its own JSON out of. 16 of its 17 mutants cannot be killed by
  any layer here; the one that dies, crashes.
* **The unit's own body is not where they are.** 7 of 10 mutants across the
  `aviFAIL` guard, the `AryLen < 1` arm, the already-allocated ALLOCATE branch,
  the `.NOT. AllowDefault_` return, the `Ary = 0` default arm and `ary_at`'s
  stride are killed. The three that survive are a `MaxParamLength` constant no
  output reads and the `std::max(NumWords, 0)` clamp with its argument order,
  which cannot bite while `AryLen = { lo = 0 }` is stated.

**TWO OF THE SURVIVORS WERE RUN THROUGH THE OTHER INSTRUMENT, and the pair
settles which kind of gap each region is.** The RUNBOOK's rule: before calling a
survivor a corpus gap, ask whether the gate can reach it — one gate run replaces
the argument.

```
2a9e1695  index_offset  parse_real:256  rec[p] -> rec[p + 1]   (the fraction-digit loop)
          harness: SURVIVED 13,674 cases
          gate:    1,583,216 of 4,732,000 moved, and scenarios 19 and 27
                   FAILED TO RUN AT ALL under it -- 520,000 values that could
                   not even be compared. Revert-verified: 5,252,000 / 0.
          gate/ParseDbAry_Opt.redtest.survivor-2a9e1695.json

ae3f319a  arith_op      list_directed_real:464  16 - decexp -> 16 + decexp
          harness: SURVIVED 13,674 cases
          gate:    0 of 5,252,000. Revert-verified: 0.
          gate/ParseDbAry_Opt.redtest.survivor-ae3f319a.json
```

**So the 87 READ survivors are a CORPUS GAP and the 16 PRINT survivors are an
INSTRUMENT GAP, and those are different things to fix.** The gate reaches the
parser hard — it is the same 43% of the campaign's compared values that the
`v[i++] = value + 0.01` perturbation moves — so a defect in the READ path would
not leave this campaign undetected even though this unit's own harness cannot
see it. The PRINT is reached by neither layer, and the negative control is what
turns that from an assumption into a measurement.

**Nothing is declared equivalent.** An unreached mutant and a
behaviour-preserving one are different claims; these are unreached or
uncompared, and calling them equivalent would be the exact shape of a green
that established nothing. **The threshold is not met and the disposition stays
`deferred`.**

**What would move it**, stated so the next dispatch does not have to re-derive
it: a corpus in which `FindLine` FINDS the line in thousands of cases rather
than a hundred — R14's plant reaching its 2:plant and 3:plant shapes, and the
planted line carrying varied numeric text rather than the character corpus's
decoys. That is a generator change and it re-takes every layer that reads the
corpus (RUNBOOK), which is why it is not folded into this dispatch. The PRINT
survivors need a different instrument: a harness that compares the two sides'
stdout.

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
  instrument in this campaign can adjudicate them and none is claimed to. The
  difference this dispatch makes is that the region is now NAMED and COUNTED in
  the artifact (4,067) instead of showing up as a red number.
* **Whatever the 108 surviving mutants would have caught.** §5 and
  `mutation_survivors.txt`. 87 of them are in the list-directed READ, which only
  **103 cases of 13,674** can distinguish anything about; 16 are in the PRINT
  record, which no layer compares. This is the single largest gap in this unit's
  evidence, and it is now measured rather than unknown.
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
mutation_survivors.txt                   the 108 survivors by source region, with
                                         the reason each region cannot be reached
harness.parser-no-zero-store.json        the harness BEFORE the parser fix, 4,070.
                                         A FIRST-DISPATCH artifact: its count is
                                         against the old comparison, when the 4,067
                                         were still failing rather than excluded
run_harness_stub.sh                      one stub through the harness, --no-generate
parsedbary_opt.noop-stub.cpp             every statement removed
parsedbary_opt.no-read-stub.cpp          the list-directed READ deleted
parsedbary_opt.no-allowdefault-arm-stub.cpp   the .NOT. AllowDefault_ arm deleted
parsedbary_opt.no-zero-store-stub.cpp    the measured zero-store rule removed
harness.*-stub.json                      the four red tests, all at 13,674 cases
mutation.refusal.txt                     189 mutants, and the FIRST dispatch's refusal
                                         to score them against a red baseline. Kept:
                                         the refusal was right and it is what the
                                         second dispatch had to remove the cause of
run_wrapper_redtest.sh                   perturb the wrapper, prove red, revert, prove green
parsedbary_opt_callees.WRONG.f90         the bridge VIT generated before the two fixes
```
