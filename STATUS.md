# STATUS — rosco-r2

**This is the live state file. Read it first, then `RUNBOOK.md`.**
`DECISIONS.md` is the append-only record of *why*; this file is *where things
stand*. One copy of every count — do not duplicate them anywhere else.

**As of 2026-08-11: unit #8 `GetWords` is `integrated` and CLOSED**, first
dispatch.

**EVERY LAYER IS ALIVE, INCLUDING THE GATE — THE FIRST TIME IN THIS CAMPAIGN.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 62/62 `IDENTICAL` on `words` | no-op → 62/62 `OUT_TOL`; a CONSTANT stub → 1/62; the real translation with `Words(1)` corrupted → 62/62 `OUT_TOL`, so every ELEMENT of the array is compared |
| differential harness vs clean Fortran | 1370 checked, 0 failed | no-op stub → 1343/1370 failed, naming `Words`; green restored |
| mutation score | 57/57 behavioural killed, 1.000, **1 declared equivalent**, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 1370 checked, 0 failed | copy-back stops one element short → 1323/1370 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST PASSED — 1,857,893 of 5,252,000 moved** |

**THE GATE SEES THIS UNIT, AND THE REASON IS THE MIRROR OF UNIT #4's.** `FindLine`
upper-cases `Words(WordInd)` and compares it against the parameter name the
caller asked for — one operand is this unit's output, the other is not. `Conv2UC`
was invisible through 1.3M calls precisely because BOTH its operands went through
it. So the campaign now has a positive case beside its five negative ones, and
one test decides all six: follow the output to its consumers and ask whether
anything the gate reads depends on it ASYMMETRICALLY.

Stated beside the green: **both extents are constant in all 27 scenarios**
(`len_Line` = 2048, `len_Words` = 200, `NumWords` ∈ {2, AryLen+1}). The kernel
inherits all three from the simulation. A defect that only appears at another
width is outside both bit-exact layers; only the differential harness varies
them. Unit #3's constant-argument shape, at EXTENT granularity.

**THE UNIT HAD NO DIFFERENTIAL HARNESS AT ALL, AND ITS ONLY OUTPUT WAS THE
REFUSED ARGUMENT.** `CHARACTER(*), INTENT(OUT) :: Words(NumWords)` is two extents
where `char[]` had one — the gap recorded as known since unit #4, which fell due
here. **`vit interface` handles this shape while the harness generator refused
it**: two generators over the same declaration, disagreeing, and only running
both showed it. Built rather than routed around (loop `6d13949`); rank ≥ 2 is
still refused with its reason.

**THREE MORE CORPUS BLIND SPOTS, ALL THREE NAMED BY A SURVIVING MUTANT.** The
score went 0.889 → 0.905 → 0.921 → 0.983 → 1.000 and no step was found by reading
the generator. Second unit running where that is true.

1. `''` inside a `'...'` literal is ONE APOSTROPHE in Fortran; the miner read the
   unit's own separator set as two literals and lost the character.
2. The seventh separator is `CHAR(9)` — a tab cannot be written as a literal at
   all — and `char_corpus` filtered to printable ASCII, so it was unreachable
   twice over. **The miner's blind spot was exactly the class of character that
   most needs mining.**
3. No string shape put a word anywhere but position 1, so every predicate about
   where a word BEGINS had one answer in every case.

None of the three moves an earlier unit's corpus (22, 26, 26 — recomputed,
unchanged).

**A MUTATION SCORE CAN FLAP, AND THE RUN THAT READS 1.000 CAN BE THE ONE THAT
MEASURED LEAST.** Three IDENTICAL runs scored 0.983, 1.000, 0.983. The mutant
responsible differs from the original only past the end of a buffer — proved
exhaustively, 4,908 disagreements and **zero in bounds**. Two fixes: the
declaration is now a statement about the MUTANT rather than about one run, and a
declared mutant that is killed anyway lands in `declared_but_killed` instead of
being absorbed (loop `5b40e1c`).

**THE REVISION STAMP WAS MIS-ATTRIBUTING ITS OWN OUTPUT, IN THREE PLACES.**
`vit-dev` has no git, so both loop scripts fell through to hand-written,
gitignored pin files. **Unit #7's committed harness and mutation artifacts stamp
`99b57ab-pinned` while the tree was at `0e92a72` — the commit that unit's OWN
corpus fix went in as** — and every artifact since unit #5 says `8c34ceb-pinned`
against a VIT at `87a3847`. `.git/HEAD` is plain text and needs no binary, so
both scripts read it first now and report `-nogit`. And **this campaign's own
`scripts/_harness_stamp.py` was a third site that OVERWROTE the correct read with
the stale pin.** Unit #7's artifacts are not restamped: a verdict that was
correct when it was taken is worth more than a deletion.

---

**Unit #7 `GetRoot` is `integrated` and CLOSED**, first
dispatch.

**THE KERNEL IS A MIRROR AND THE GATE IS BLIND, AND THE INSTRUMENT THAT DOES
CONSTRAIN IT WAS ITSELF BLIND UNTIL THIS UNIT FIXED IT.**

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 62/62 `IDENTICAL` over the full `CHARACTER(8)` | **a NO-OP also scores 62/62 `IDENTICAL`**; a WRONG-constant stub scores `OUT_TOL`, which is what says the comparison is alive |
| differential harness vs clean Fortran | 726 checked, 0 failed | no-op stub → 700/726 failed, naming `RootName`; green restored |
| mutation score | 60/60 behavioural killed, 1.000, **2 declared equivalent**, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 726 checked, 0 failed | `LEN(RootName)` → `LEN(RootName) - 1` in the CALL → 596/726 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

8 of the 60 mutation kills are CRASHES rather than case mismatches, so the
killed-by-comparison count is 52 of 60.

**THE CALL SITE ALIASES ITS TWO ARGUMENTS, AND THAT BREAKS TWO OF THE
CAMPAIGN'S OWN RED TESTS.** `DISCON.F90:67` is `CALL GetRoot(RootName,RootName)`
— one variable as both the INTENT(IN) and the INTENT(OUT) dummy. KGen's captured
input is therefore bit-for-bit its captured reference output (the state file is
`vit_sim1vit_sim1`), so a stub that reads nothing and writes nothing PASSES. And
because every scenario's name contains no `'.'`, `GetRoot` is the IDENTITY on
the exercised domain — so the gate's standard no-op perturbation is not a wrong
implementation either. Both no-op red tests are green for reasons that have
nothing to do with observability. The tests that carry the claim are a
**wrong-constant** kernel stub (62/62 `OUT_TOL`) and a **wrong-output** gate
perturbation (0 of 5,252,000 moved, `replacements: 1`, `revert_verified: true`).
Unit #6's recipe read forwards gives the wrong answer here; RUNBOOK now says so.

**A FIFTH SHAPE OF P9: the result is consumed, by a live line, into a channel
the gate does not read.** `RootName` has six reader sites and all six build a
FILENAME. Five are dead in all 27 scenarios; the sixth runs 24 times and is
`OPEN(unit=UnDb, FILE=TRIM(RootName)//'.RO.dbg')`. `gate.py` compares the `.npz`
arrays `vit_sim.py` collects across the DLL boundary and never opens a `.RO.dbg`.
Perturbing this unit renames a file.

| unit | shape |
|---|---|
| #1 `AddToList` | the line is never executed |
| #3 `ColemanTransformInverse` | an argument is constant in every scenario |
| #4 `Conv2UC` | 1.3M executions, result cancelled by a symmetric consumer |
| #6 `GetPath` | result produced, consumer's guard false wherever it would matter |
| #7 `GetRoot` | result consumed by a live line, into a side effect outside the instrument |

**THE FIRST DIFFERENTIAL HARNESS REPORTED `224 checked, 0 failed` AND HAD NEVER
EXECUTED THE BRANCH THE PROCEDURE EXISTS FOR.** Three independent gaps in the
generator, each found from a SURVIVING MUTANT rather than from the verdict:

1. R6 mined **single-character** literals, so the character SET `'\/'` was
   invisible and the corpus contained **no backslash at all**. Now every
   character of every literal handed to `INDEX`/`SCAN`/`VERIFY` is mined.
2. The corpus is each literal plus its **collating neighbours**, laid down in
   corpus order — and `'/'` is `'.'+1`, so every generated string containing a
   dot had a separator directly after it. The rule that makes the corpus
   relevant is the rule that blinded it. Planted characters and planted PAIRS
   of the reference's own literals now break the adjacency.
3. The length ladder `{1, N, N+5}` had no **2**, the smallest non-degenerate
   string and the first length at which a first-versus-second-character test has
   two answers.

224 cases / mutation 0.648 → 726 / 0.968. Fixed in the loop repo (`0e92a72`).

**TWO KGEN DEFECTS, BOTH FIXED IN KGEN (`4457cd2`).** The generated kernel
compiled, ran, printed `62/62 passed` and compared **nothing** — VIT's own
"kernel compared 0 output variables" is what caught it. `update_state_info` found
an argument's position with `list.index`, and `Fortran2003.Base` compares nodes
by content, so both occurrences of a repeated actual argument resolved to
argument 0. Fixing that exposed a second: the VERIFY name generator embeds the
declaration's selector in a procedure name, so `CHARACTER(LEN=size(avcoutname))`
produced `kv_discon_character_size(avcoutname)_`. `c839e1a` had fixed exactly
that on the GENCORE side and the VERIFICATION side never got it. Red-tested in
both directions; the green control is a `GetPath` re-extraction that comes back
byte-identical with and without the patch.

**REMOVING AN UNOBSERVABLE RESTATEMENT AGAIN — and this time it is the SAME
INTRINSIC unit #4 removed.** Transcribing `LEN_TRIM` literally scored 0.886 with
**six of eight survivors inside the helper**, and three of those six are
out-of-bounds reads, which cannot honestly be declared equivalent. The
translation carries a three-part proof that `LEN(GivenFil)` serves at both sites.
The two mutants that remain are declared equivalent with their proofs committed
at `mutation/GetRoot.equivalences.md`: an out-of-bounds read no value comparison
can see, and `RootName = ''` — **dead code in upstream ROSCO**, unreachable
because the special case at the top of the procedure has already returned on the
only input that could get there.

---

**Unit #6 `GetPath` is `integrated` and CLOSED**, first
dispatch.

**BOTH BIT-EXACT LAYERS ARE VACUOUS FOR THIS UNIT, AND EACH SAYS SO IN AN
ARTIFACT.** The gate is blind and the kernel has one case that a lookup table
passes. What verifies `GetPath` is the differential harness and the mutation
score.

| layer | result | red-tested |
|---|---|---|
| kernel replay, **1 case**, scenario 1 | 1/1 `IDENTICAL` over the full `CHARACTER(1024)` | no-op stub → `OUT_TOL`. **AND A CONSTANT STUB PASSES 1/1** |
| differential harness vs clean Fortran | 236 checked, 0 failed | no-op stub → 234/236 failed, naming `PathName`; green restored |
| mutation score | 25/25 behavioural killed, 1.000, 0 declared equivalent, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 236 checked, 0 failed | `LEN(PathName)` → `LEN(PathName) - 1` in the CALL → 199/236 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

4 of the 25 mutation kills are CRASHES rather than case mismatches, so the
killed-by-comparison count is 21 of 25.

**A FOURTH SHAPE OF P9, AND IT IS A PROPERTY OF THE CONSUMER.** `GetPath` is not
dead: `ReadSetParameters.f90:331` runs 28 times across the 27 scenarios, and so
do both readers of its output. The result is **produced and never consumed**.
`PriPath`'s only two readers are guarded by `PathIsRelative`, and the guard is
false exactly where the answer would matter — `PerfFileName` is one absolute path
in all 14 `Examples/*.IN`; `OL_Filename` is the literal `"unused"` (relative, so
the concatenation happens) in the scenarios where `OL_Mode` is 0 and nothing
opens it, and absolute in the three that do. Six units, four shapes, and
`coverage/line_coverage.json` can express none of them:

| unit | shape |
|---|---|
| #1 `AddToList` | the line is never executed |
| #3 `ColemanTransformInverse` | an argument is constant in every scenario |
| #4 `Conv2UC` | 1.3M executions, result cancelled by a symmetric consumer |
| #6 `GetPath` | result produced, consumer's guard false wherever it would matter |

Two gate red tests are committed and the record says which one carries the
claim: the no-op (decisive) and the `I == 0` branch forced off (deliberately
weak — coverage already showed that branch has zero hits in all 27, so it is
RUNBOOK's "attempt 1" reproduced on purpose).

**THE KERNEL IS STRUCTURALLY ONE CASE.** The call site runs ONCE PER PROCESS, so
no invocation window can widen it, and every scenario builds the argument as
`os.path.join(this_dir, '<one of 14 DISCON*.IN names>')` — one answer in all 27.
Measured, not argued: a stub reading NEITHER input and writing that literal
blank-padded scores **1/1 IDENTICAL**
(`evidence/GetPath/kernel.constant-stub-PASSES.verify_fields.csv`). Unit #2's
all-zero window in a new costume, and unlike #2 there is nothing to widen.

**A KGEN DEFECT WAS FIXED IN KGEN, AND IT IS ON THE CRITICAL PATH FOR MUCH OF
WHAT REMAINS.** The generated kernel would not compile: KGen hoists the enclosing
procedure's dummies into the driver's PROGRAM scope, and
`ReadControlParameterFileSub` declares `CHARACTER(accINFILE_size) ::
accINFILE(accINFILE_size)` — an AUTOMATIC length, legal on a dummy and illegal on
a local. KGen already had the machinery for `CHARACTER(*)` (`c839e1a`) and keyed
on `selector[0] == '*'`. Fixed additively with a narrow new predicate, and
red-tested in both directions: the pre-fix driver plus gfortran's diagnostic is
kept, and a re-extraction of `Conv2UC` — whose enclosing `FindLine` has both a
`CHARACTER(*)` dummy and `CHARACTER(MaxParamLength)` locals, the two shapes the
predicate must treat differently — regenerated **byte-identical to unit #4's
committed kernel apart from its timestamp** and re-verified 62/62 IDENTICAL.
`ReadControlParameterFileSub` encloses the `ParseInput_*`, `ParseAry`, `FindLine`
and `GetWords` call sites, so any unit extracted from there would have hit it.

**Removing an unobservable restatement raised the score from 0.882 to 1.000 —
the fourth time in six units, and the first RE-measurement of unit #1's exact
rule.** Both loops in `char_assign` written 0-based left two `<` → `<=` survivors
that write one byte past a buffer or into a byte the next loop overwrites —
neither a wrong answer, so no value comparison can see either. Written 1-based,
the way the Fortran states the assignment, the same mutant leaves a byte
UNWRITTEN and both die. The mutant count went UP, 17 → 25: the observable form
has more sites, not fewer.

---


**Unit #5 `ExtController` is `integrated` and CLOSED**, on its
third dispatch. Two dispatches closed it `blocked`; both blocking claims are now
refuted by measurement, and the second one cost less than the first.

**A DEAD UNIT WITH A DERIVED-TYPE SIGNATURE CLOSES THE SAME WAY `AddToList` DID.**
The gate is blind to it — 0 of 28 executable lines in all 27 scenarios, and
making the unit a no-op moves 0 of 5,252,000 values — so the evidence is the
differential harness and the mutation score, against an oracle this campaign
had to build:

| layer | result | red-tested |
|---|---|---|
| kernel replay | NOT ATTEMPTED — no live call site, so nothing to capture | n/a; unit #1's rule |
| differential harness vs clean Fortran | 163 checked, 0 failed | no-op stub → 163/163 failed; green restored |
| mutation score | 48/48 behavioural killed, 1.000, 4 declared equivalent, 8 nocompile excluded (13.3%) | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 163 checked, 0 failed | ErrVar reverse copy removed → 163/163 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

**THE DEFECT THIS UNIT FOUND IS BIGGER THAN THE UNIT.** `vit test-validate`'s
decomposed bridge copied every field of a derived-type argument IN and almost
none OUT. A rank-1 ALLOCATABLE field and a SCALAR field of an INOUT argument
were both **input-only**, silently — so `ExtDLL%avrSWAP`, which is the whole of
what `ExtController` produces, and `ErrVar%ErrStat`, which is where ROSCO puts
every error status, were handed to the reference and then discarded. **Every
unit in this campaign taking a derived type was affected**, which is most of
them. Fixed in VIT (`8c34ceb`) as three members — pointer, extent BY REFERENCE
because the callee chooses it, and a capacity — with an over-long result
REFUSED and reported rather than truncated. The wrong artifact is kept at
`evidence/ExtController/vit_defects/extcontroller_bridge.alloc_input_only.f90`.

Four further findings, each measured:

1. **A CHARACTER FIELD did not cross, and 37 of 69 units take one.** `vit
   test-validate` refused a deferred-length `CHARACTER(:)` field outright and
   the loop's `expand_derived` dropped every CHARACTER field with a comment
   saying so. Both closed (VIT `8c34ceb`, loop `7d7c913`). A CHARACTER **array**
   field is still refused, with its reason.
2. **`ExtDLL%avrSWAP(49)` is the constant 2, not the size of the message
   buffer its own comment claims.** `avcMSG` is an ARRAY of `CHARACTER(1)`, and
   `LEN` of a CHARACTER array is the ELEMENT length. Measured before any C++
   was written (`evidence/ExtController/len_probe.txt`) and transcribed, not
   corrected: the original is the oracle. An upstream ROSCO defect.
3. **`avcMSG = TRANSFER(C_NULL_CHAR, avcMSG)` writes ONE byte and leaves the
   rest indeterminate.** gfortran copies min(size) elements from a nonconforming
   right-hand side (`evidence/ExtController/transfer_probe.f90` prints
   `0 90 90 ...`). The translation zero-initialises instead, with the proof
   written in the file — and the oracle fixture deliberately does NOT read those
   bytes, because a stub that folded in indeterminate memory would make the two
   sides disagree for a reason about neither implementation.
4. **`vit integrate --auto-allocate` cannot be used on this codebase.** Its
   copy-call scan matches ANY `CALL x(..., arg%field, ...)`, so it hoisted both
   `LoadDynamicLib` AND the external DLL call into the wrapper; it declines a
   size expression that is a local PARAMETER; and its error handling emits
   `SetErrStat` and `CHARACTER(ErrMsgLen)`, OpenFAST names that exist nowhere
   in ROSCO. Artifact kept; the one ALLOCATE this unit needs is in the committed
   wrapper with the reason beside it. **Open, and it is the campaign's problem
   now, not this unit's.**

**Escalation 3 is CLOSED.** Escalation 2 still stands in part — `loop/done.py`
still has no branch for `integrated_unexercised` — but it no longer blocks
anything: a dead unit whose signature crosses closes as `integrated`, and this
is the second one.

---


**Unit #4 `Conv2UC` is `integrated` and CLOSED.** Every layer green with its own
red test — except the gate, whose red test FAILED and is committed as the
finding.

**THE GATE IS BLIND TO A PROCEDURE CALLED 1,333,146 TIMES.** `Conv2UC` converts
4,558,823 characters across the 27 scenarios; it is among the hottest procedures
in the controller. Two perturbations of the integrated C++ were built and gated
and **each moved 0 of 5,252,000 values** — `ch - 32` → `ch - 31`, and the guard
forced false so no conversion happens at all. The cause is structural: every
value the unit produces is consumed by an equality test against another of its
own outputs (`FindLine` upper-cases both the expected parameter name and the
file's word before comparing them), so a perturbation lands on both operands and
equality survives.

This is the third distinct shape of P9 in four units and the sharpest: #1 was a
line no scenario reached, #3 was an argument constant in every scenario, and
this is an **executed line whose result is cancelled downstream**. A hit count
of 1.3M says nothing about observability, and `coverage/line_coverage.json`
cannot express the difference.

Two further findings:

1. **The differential harness had no string kind, and 20 of 69 units need one.**
   `vit_harness.py` reported the CHARACTER argument comparable-but-held-constant
   and then died on `C parameter 'Str' is not in the mapped signature`. With
   P11 and P12 mandatory, none of those 20 units had a path to close. Built
   rather than worked around: loop `9bee569`. A CHARACTER **array** dummy is
   still refused, with its reason.
2. **Removing a restatement raised the mutation score from 0.696 to 1.000, for
   the second time in the campaign.** Transcribing `LEN_TRIM` literally adds six
   mutable sites computing a quantity nothing downstream can read; five survived
   as out-of-bounds reads no value comparison can see. Unit #1 named a size
   once; this named a loop bound once.

Units #1 `AddToList`, #2 `ColemanTransform` and #3 `ColemanTransformInverse`
remain closed. Escalation 1 is answered; **escalation 2 still stands** — a dead
unit whose signature does NOT cross has no path to harness, mutation or gate,
and `loop/done.py` still has no branch for `integrated_unexercised`.

Next: the lowest-order remaining unit in `plan.json`. Confirm it against
`plan.json` before starting, including its `phase` and `proposed_verification`
fields — those are hypotheses, not facts. Run
`python3.12 scripts/done_check.py <Unit>` before setting any disposition.

## Counts

8 attempted / **8 integrated** / 0 integrated_unexercised / 0 out_of_scope /
0 deferred / 0 blocked.

69 units in `plan.json`; 61 remain.

**5 of the 8 integrated units are invisible to the gate**, for five different
reasons: `AddToList` is never called, `Conv2UC` is called constantly and
cancelled, `ExtController` is never called *and* has no observable effect on any
channel the gate compares even when it is, `GetPath` is called in every scenario
and its answer is never consumed, and `GetRoot`'s answer IS consumed — by a live
line that uses it to name a debug file the gate never opens. Each carries a green
gate artifact committed beside the red test that says it constrains nothing.

**Every unit now has a `harness/` and a `mutation/` artifact.** `ExtController`
was the exception for two dispatches and the absence was recorded as the
finding; it is now unfinished work that got finished.

## Evidenced

- **E1.2** — `rosco/controller/CMakeLists.txt` passes `-ffp-contract=off` to
  gfortran and to g++, and `scripts/assert_fp_contract.sh` asserts both the
  build line and **zero FMA instructions in the linked library**. Red-tested by
  blanking the flag: 0 build lines, 103 FMA, exit 1.
- **E3.1** — `scripts/gate.py` runs 27 scenarios, compares 5,252,000 values
  across 351 channels bit-for-bit against `baseline_arrays`, prints the count
  and persists `gate/__gate__.json`. Exits non-zero on mismatch and on comparing
  nothing.
- **E3.2** — gate observed red: perturbing `LPFilter`'s leading `1.0` moved
  1,604,573 of 5,252,000 values across 138 of 351 channels; reverting restored
  0. `gate/__gate__.redtest.json`.
- **E3.5** — `baseline_arrays/` regenerated from clean pre-integration source
  after E1.2 changed the flags; 20 of 27 scenario files moved. E3.2 re-run
  against the new set, so the red and green evidence describe the same
  baselines.

Both E3.1 and E3.2 were red-tested **as criteria**, not just satisfied:
corrupting the expected values in `phases.toml` turns them `[FAIL]`, restoring
them turns them `[ok]`.

### The campaign's VIT: `d07a716` → `22086e8` → `37f8bdf` (unit #1) → `f8ab74f` (unit #5)

**`f8ab74f` (unit #5)** makes `generate_fortran_wrapper` say, on stderr, which
derived-type arguments it accepted and did not forward. `dropped_derived_args`
already existed and only `test_validate` asked it, so `vit interface` — the
command that SHOWS you the wrapper — and `vit integrate` — the one that SHIPS it
— both emitted a bridge with fewer arguments than the wrapper, in silence. On
`ExtController` that was `LocalVar` and `ErrVar`, and `LocalVar%iStatus` is the
guard on the entire initialisation branch.

**Additive: no generated byte changes**, so no artifact already measured against
an earlier revision is invalidated — units #1–#4 are unaffected. Red-tested in
both directions (fires on `ExtController` with the strategies unset, silent on
`ColemanTransform` and on `ExtController` once they are set). 937 tests pass,
935 before plus the two added with it.

The three earlier moves happened during unit #1. Neither of the first two was an
upgrade; both are fixes that unit produced.

`22086e8` (first dispatch) made `interface_gen` REFUSE an ALLOCATABLE
INTENT(INOUT) dummy instead of emitting a wrapper that compiles and does
nothing, and gave the conformance matrix an `integrates` column measuring the
generators `vit integrate` actually ships.

`37f8bdf` (second dispatch) replaced the refusal with the feature it was
standing in for: the descriptor bridge, in all three generators
(`interface_gen`, `test_validate`, and the scaffold), plus two things found on
the way — `vit_translated.h` did not include `<ISO_Fortran_binding.h>` for a
declaration that names `CFI_cdesc_t`, and **every file `vit integrate`
generated carried `// After verification: <name> kernel PASSED`**, a verdict
the integrator never checked and which was flatly false here. The loop's
harness learned the same convention in `cf885e3`. VIT suite 935 passed; the
loop's 393, with 7 pre-existing `test_tiny_campaign` failures confirmed present
without these changes.

`tests/conformance/matrix.toml`'s `c_alloc_inout` has now been all three
things: `compiles = "no"` (measured on the wrong generator), `integrates =
"refused"`, and now `yes`/`yes`. Its `why` carries the history.

**`AddToList`'s first-dispatch evidence was deliberately measured under
`d07a716`** and stamps it. `evidence/AddToList/vit_interface.stdout.txt`,
`vit_translate.stdout.txt`, `addtolist.scaffold.cpp` and
`bridge_probe/mod_vit.f90` **cannot be regenerated** — that generator no longer
exists in either later form. They are the record of what it did.

`ColemanTransform`'s committed evidence was produced under `d07a716` and is
unaffected in substance. One caveat, recorded rather than left to be
discovered: `rosco/controller/src/colemantransform.cpp` was NOT regenerated, so
the "regenerates byte-identical" property below no longer holds for its three
header comment lines, which `37f8bdf` changed. Everything else about it is
untouched.

### ColemanTransform, second pass — every number re-measured

Every artifact for this unit now comes from ONE instrument pair, VIT `d07a716`
and loop `ebce989`, and each names its own producer in a `vit_rev`/`loop_rev`
field. The first pass used VIT `d85b33b` and loop `3c88913`.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 27 | 63/63, 14,175 fields IDENTICAL | zero-writing stub → 124 `OUT_TOL`; green restored on revert |
| differential harness vs clean Fortran | 199 checked, 0 failed | mutation, below |
| mutation score | 35/35 killed, 1.000 (33 by case mismatch, 2 by failing to compile) | baseline green first, else it refuses to score |
| post-integration harness (wrapper only) | 199 checked, 0 failed | wrapper args swapped → 199/199 failed; green restored on revert |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `2.0/3.0` → `2.000001/3.0` moved 124,353 values across 15 channels; revert → 0 |

`vit integrate` under the new VIT regenerated `Functions.f90`, `CMakeLists.txt`
and `colemantransform.cpp` **byte-identical** to the committed integration.

### ColemanTransformInverse — every layer, and its red test

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 27, `Controllers.f90:561` | 63/63, `pitcomipc_1p` `IDENTICAL` in every case | zero-writing stub → 61/63 `OUT_TOL`; green restored on revert |
| differential harness vs clean Fortran | 257 checked, 0 failed | the mutation score, below |
| mutation score | 24/24 behavioural killed, 1.000, 0 declared equivalent | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 257 checked, 0 failed | `axTIn`/`axYIn` swapped in the wrapper's CALL → 256/257 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `PitComIPC[0]` × 1.000001 moved 389,644 values; revert → 0 |

2 of the 24 mutation kills are CRASHES (`[2] -> [2 + 1]` and `'2' -> '3'`, both
writing past the caller's 3-element array), so the killed-by-case-mismatch count
is 22 of 24. 2 further `compare_op` mutants did not compile and are EXCLUDED
from the score rather than counted — `vit_mutate.py` changed this after unit #2
(loop `46a7f4f`), so that unit's `35/35` and this one's `24/24` are not the same
measurement.

The signature crosses whole: `PitComIPC(3)` `INTENT(OUT)` becomes
`REAL(C_DOUBLE), INTENT(OUT) :: PitComIPC(*)` — by reference, rank preserved, no
extent parameter emitted or needed — and the five scalars go by `VALUE`.
`vit interface` was read attribute by attribute before any C++ was written, per
the habit unit #1 installed; nothing was dropped.

### Conv2UC — every layer, and the one red test that failed

| layer | result | red-tested |
|---|---|---|
| kernel replay, 63 cases, scenario 1, `ROSCO_Helpers.f90:1118` | 63/63 `IDENTICAL` | no-op stub → 31/63 `OUT_TOL`; green restored |
| differential harness vs clean Fortran | 118 checked, 0 failed | no-op stub → 27/118 failed |
| mutation score | 14/14 behavioural killed, 1.000, 0 declared equivalent (4 nocompile EXCLUDED) | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 118 checked, 0 failed | `LEN(Str)` → `LEN(Str) - 1` → 24/118 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED, twice — 0 of 5,252,000 moved** |

2 of the 14 mutation kills are CRASHES rather than case mismatches (both index
before the buffer), so the killed-by-comparison count is 12 of 14. The 4
excluded nocompile mutants are all `harness/cppmutate.py` mangling
`static_cast<unsigned char>` — 22% against the 25% at which `vit_mutate.py`
refuses to score.

The signature crosses whole: `CHARACTER(*), INTENT(INOUT) :: Str` becomes
`CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: Str(*)` plus
`INTEGER(C_INT), VALUE :: len_Str`, with copy-in/copy-out in a wrapper that
declares the dummy exactly as the original does. `vit interface` was read
attribute by attribute before any C++ was written.

**For a CHARACTER output a kernel PASS *is* a bit-identity claim** — KGen's
generated comparison has no tolerance branch at all. Unit #3's caveat is a fact
about REAL fields, not about the verdict line; read the generated comparison for
each new element type.

### ExtController — every layer, and the one red test that failed

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT ATTEMPTED** — 0 of 28 lines in all 27 scenarios, so no state to capture | n/a; unit #1's rule is that this is deadness, not a tool defect |
| differential harness vs clean Fortran | 163 checked, 0 failed | no-op stub → 163/163 failed, naming ExtDLL.avrSWAP, ExtDLL.n_avrSWAP, ErrVar.ErrStat, ErrVar.ErrMsg; green restored |
| mutation score | 48/48 behavioural killed, 1.000, 4 declared equivalent, 8 nocompile EXCLUDED | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 163 checked, 0 failed | ErrVar reverse copy removed → 163/163 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved** |

The gate red test carries `replacements: 1`, `perturbed: true`,
`revert_verified: true`, `residual_dirt: []`, and perturbs toward **absence** —
the unit made a no-op — which is the form that shows the gate is blind to the
*unit* rather than to the perturbation chosen. `gcov` says the same thing
independently: **0 of 28 executable lines, in all 27 scenarios.**

**The four survivors are DECLARED, not smoothed over, and each was applied by
hand and re-run before it was declared** (`mutation/ExtController.equivalences.json`).
Two are the same fact: `LEN_TRIM(ErrVar%ErrMsg)` is 0 in every admissible case
because `LoadDynamicLib` blank-fills the message through a `CHARACTER(*),
INTENT(OUT)` dummy, so the concatenation's right operand is constant and the
memmove moves zero bytes. One is a capacity boundary four orders of magnitude
away. One is a buffer length nothing reads back.

**FOUR OTHER SURVIVORS WERE REMOVED RATHER THAN DECLARED, and one was fixed in
the ORACLE.** `accINFILE`'s bound and record 50 are the same expression, as are
`avcOUTNAME`'s bound and record 51; transcribing each twice left a site
computing a quantity nothing reads. Naming each once took the score from 0.758
to 0.904 — the third time in this campaign that removing a restatement raised a
mutation score. The fifth, `aviFAIL = 0`, was a write-only local: the stub now
reports the incoming value back in record 44 before overwriting it, and the
mutant dies. **Where the blindness is in the instrument rather than in the unit,
the instrument is what to change.**

**The oracle fixture had to grow twice for this**, and both times because a
value was reaching the library and never being read: the bytes of `accINFILE`
and `avcOUTNAME` (record 46) and the incoming `aviFAIL` (record 44). A stub that
answers without reading its inputs is unit #2's all-zero kernel window in
another costume.

### GetPath — every layer, and the two red tests that failed

| layer | result | red-tested |
|---|---|---|
| kernel replay, 1 case, scenario 1, `ReadSetParameters.f90:331` | 1/1 `IDENTICAL` over `CHARACTER(1024)` | no-op stub → `OUT_TOL`; **constant stub → PASSES 1/1** |
| differential harness vs clean Fortran | 236 checked, 0 failed | no-op stub → 234/236 failed, naming `PathName` |
| mutation score | 25/25 behavioural killed, 1.000, 0 declared equivalent, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 236 checked, 0 failed | `LEN(PathName) - 1` in the CALL → 199/236 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST FAILED — 0 of 5,252,000 moved, twice** |

The signature crosses whole and `vit interface` was read attribute by attribute
before any C++ was written: `CHARACTER(*) INTENT(IN)` and `CHARACTER(*)
INTENT(OUT)` each become `CHARACTER(KIND=C_CHAR) :: x(*)` plus
`INTEGER(C_INT), VALUE :: len_x`, with the wrapper declaring both dummies exactly
as the original does. `plan.json`'s `bridge_feasible` said `unknown`, from the
pre-`integrates` conformance matrix; it is now `yes`, from the generator that
ships.

The no-op red test fails **234 of 236**, not all 236, and the two survivors are
the `INTENT(OUT)` copy-in defect below rather than slack in the harness.

`PathSep` is `CHARACTER(1), PARAMETER :: PathSep = '/'` from
`SysFiles/SysGnuLinux.f90` — confirmed from the build log's `Setting system file
as:` line, not read off `CMakeLists.txt`. The forward-slash SEARCH takes the
literal `'/'` and not `PathSep`, because in the reference they are two different
things and only the fallback uses the platform parameter (P7).

## Open

- **The differential harness's CHARACTER-ARRAY refusal is CLOSED, and the way it
  was found is the standing warning.** `vit interface` shipped a working bridge
  for `Words(NumWords)` while the harness generator refused the same declaration
  — so `plan.json` said the signature crossed, the integration worked, and the
  unit's ONLY OUTPUT was invisible to P11 and P12. Two generators over one
  declaration, disagreeing, and only running both showed it. Built in the loop
  repo (`6d13949`). `FileLines(:)` is still refused: assumed-shape, so its
  element count is in a descriptor `build_c_params` does not emit. Rank ≥ 2 also
  still refused. **`FindLine`, `ParseInAry_Opt`, `ParseDbAry_Opt` and the four
  `ParseInput_*_Opt` units all take a `FileLines(:)`, so that is the next
  instrument gap on the critical path** — the refusal that remains is narrower
  than the one that was there, and it is the one that blocks the most units.
- **A revision stamp read from a hand-written pin file was mis-attributing the
  campaign's own evidence, in three places.** CLOSED in the two loop scripts
  (`5b40e1c`, reading `.git/HEAD` before the pin) and in this campaign's
  `scripts/_harness_stamp.py` (which OVERWROTE the correct read with the pin,
  and now fills a missing key only). Kept open as a record of what the stale
  stamps NAME, because the artifacts are not restamped: **unit #7's
  `harness/GetRoot*.json` and `mutation/GetRoot.json` say `loop_rev:
  99b57ab-pinned` and the tree was at `0e92a72`** — the commit carrying that
  unit's own corpus fix — and **every artifact from unit #5 onward says
  `vit_rev: 8c34ceb-pinned` against a VIT checkout at `87a3847`**. The numbers in
  those artifacts stand; the instrument they name is one or more commits behind
  the instrument that produced them.
- **A mutation score can FLAP between runs of the identical command.** Unit #8:
  0.983, 1.000, 0.983 over the same translation and the same 1370 cases, because
  the deciding mutant differs from the original only past the end of a buffer.
  The scoring defect is closed (a declaration is now about the MUTANT, not about
  one run) but the general hazard is not: **a mutation score is not reproducible
  wherever a mutant's only observable difference is undefined behaviour, and the
  run that reads 1.000 is the one that measured least.** Any unit whose score was
  taken once, with survivors of that shape, has a number nobody has repeated.
- **Both of `GetWords`'s extents are constant in all 27 scenarios**, and this is
  P9 at EXTENT granularity — a shape the verification ledger (E5.2) has no column
  for. `len_Line` is `MaxLineLength` = 2048 and `len_Words` is `MaxParamLength` =
  200 at every one of the eleven call sites; `NumWords` is 2 or `AryLen+1`. The
  gate's red test PASSES for this unit and still constrains nothing about what
  happens at another width. Only the differential harness varies them.

- **`vit check -f <file>` MISATTRIBUTES, and at unit #7 it misattributed a
  finding that LOOKED like it belonged to the unit.** FOURTH sighting, at unit
  #8: two findings against a 190-line translation, `narrowing-local` on
  `ExpUCVarName` (clean 1051) and `delimiter-set` on `':/'` (clean 1236, inside
  `GetPath`), against a `GetWords` occupying clean 1150–1216. Third sighting
  below. The two
  findings against `GetRoot` are `narrowing-local` on `ExpUCVarName` (clean 1051,
  inside `FindLine`) and `delimiter-set` on `':/'` (clean 1332, inside
  `PathIsRelative`) — neither in `GetRoot`'s range 1253–1306. The new part is that
  `GetRoot` **does** contain a delimiter set, `'\/'`, so the reported finding is
  a near-miss a reader could plausibly accept. Re-attribute by line range, not by
  plausibility. Fix belongs in VIT: scope the cross-source checks to the
  function, not the file.
- **The differential harness generator has been blind three ways at once, and
  the verdict never said so.** Closed in the loop repo (`0e92a72`) — see the unit
  #7 block above — but kept open here as a standing warning about the SHAPE:
  a harness green is a claim about the cases that were generated, and only the
  mutation survivors say which cases those were. Every one of the three gaps was
  found by asking why a mutant lived, never by reading a `checked N failed 0`.
- **The post-integration harness links PREBUILT Fortran objects.** A red test
  that edits a wrapper and re-runs `harness.sh --post-integration` without
  rebuilding the controller measures the OLD wrapper and reports green — which
  reads exactly like a harness that cannot fail. Measured at unit #7: the same
  perturbation goes 0/726 without a rebuild and 596/726 with one. `harness.sh`
  could rebuild, or refuse to run when a source it links is newer than the object.
- **`vit interface` EMITS A COPY-IN FOR AN `INTENT(OUT)` ARGUMENT.** New at unit
  #6, seen again at unit #7 (26 of 726 no-op cases survive because of it) and at
  unit #8 (27 of 1370, on a CHARACTER ARRAY, where the wrapper reads every one of
  `len_Words * NumWords` undefined bytes before the call). The
  shipped `GetPath` wrapper reads `PathName` — undefined on entry, by
  definition of `INTENT(OUT)` — into the C_CHAR staging array before the call. It
  does not change this unit's answer, because the translation writes every one of
  `len_PathName` bytes. It changes what the instruments can SEE: it is exactly
  why a no-op stub survives 2 of 236 differential cases (a no-op hands back the
  caller's bytes, and twice those already equalled the answer), and it would mask
  a translation that failed to write the tail. Left in the generated wrapper with
  the finding recorded rather than hand-edited — a hand-edit would make the
  shipped bridge disagree with the generator for every unit after it. Fix belongs
  in VIT (X2). **20 of this campaign's 69 units take a CHARACTER dummy**, so this
  will recur.
- **The done-condition CRASHED rather than answering, on a unit whose evidence
  was complete.** `loop/gitrepo.py`'s `file_at` ran `git show` under `text=True`,
  and `_resolves` (K3/P6) calls it only to ask WHETHER an evidence reference
  names a committed artifact — it never reads the bytes. Unit #6 is the first to
  commit a BINARY evidence artifact (the captured KGen state file), so it raised
  `UnicodeDecodeError` out of `subprocess.communicate` and produced no verdict
  and no predicate list. A verifier that cannot report is worse than one that
  reports FAIL: a session cannot tell "the unit is not finished" from "the check
  is broken", and the obvious next move — drop the artifact from the evidence
  list — would have made the evidence WEAKER to satisfy a defect in the thing
  measuring it. Fixed in the loop repo (`74742bc`), verified in both directions.
  CLOSED; kept here because it is the second time an instrument's own failure
  looked like a finding about a unit.
- **The campaign now has FIVE shapes of P9 and the verification ledger (E5.2)
  needs a column for each.** Unit #4 asked for a "hot but cancelled" column;
  unit #6 adds "produced but never consumed"; unit #7 adds "consumed by a live
  line, into a side effect the instrument does not read". All five are consumer-
  or input-space properties that `coverage/line_coverage.json` cannot express,
  because coverage counts entries to a line and every one of these lines is
  entered.
- **A unit can be the IDENTITY on the whole exercised domain, and then the gate's
  standard no-op perturbation is not a wrong implementation.** New at unit #7.
  `GetRoot` strips a file extension and every scenario hands it a name with no
  `'.'`, so all 444,000 calls fall through to `RootName = GivenFil`. A no-op
  returns the caller's own bytes, which through the aliased call site IS the
  right answer. The red test has to perturb the unit to a WRONG value. The same
  property makes the KERNEL a mirror rather than a comparison, so the kernel's
  liveness test also has to be a wrong-constant stub rather than a no-op.
- **A unit's kernel can be STRUCTURALLY one case, and no configuration reaches
  it.** `GetPath`'s call site runs once per process. Unit #2's answer to a
  vacuous window — widen it — does not apply, and `vit.yaml`'s `invocation` is
  irrelevant for any unit called once during initialisation. The check that
  works is the constant stub: a translation that reads NO input and writes the
  captured answer. If it passes, the kernel is a lookup table. This belongs
  beside the existing all-zero-window recipe in RUNBOOK, and it is there.
- **Unit #4's kernel case count is one more than its own window specifies.**
  Re-extracting `Conv2UC` with the identical command yields **62**, both with and
  without unit #6's KGen change; `evidence/Conv2UC/kernel-window-1.statefiles.lst`
  has 63, the extra being `Conv2UC.0.0.21` — outside the configured
  `0:0:1-20`. 20 + 21 + 21 = 62. An extra IDENTICAL case does not weaken that
  unit's 63/63, so nothing about its conclusion changes, but a future run
  comparing case counts across passes should not treat them as stable. Likeliest
  cause is a stale state file from an earlier attempt being swept up.
- **ESCALATION 3 IS CLOSED.** All three items are done and the unit is
  `integrated`. (1) `CHARACTER(:), ALLOCATABLE` crosses a view struct (VIT
  `a2e2c30`) and now a decomposed differential bridge too (`8c34ceb`) — the
  37-of-69 blocker, off the critical path in both generators. (2)
  `TYPE(ExtDLL_Type)` crosses as a PRODUCTION BRIDGE that owns the SAVE
  variable (`rosco/controller/src/vit_extcontroller_dll.f90`) rather than as a
  view: it calls `LoadDynamicLib` (X1) and hands back `ProcAddr(1)` as a C
  function pointer, which is what `C_F_PROCPOINTER` compiles to. (3) The oracle
  exists and changes no verification default.
- **NEW, AND IT IS THE CAMPAIGN'S PROBLEM: `vit integrate --auto-allocate` does
  not work on this codebase.** Three defects, measured, artifact at
  `evidence/ExtController/vit_defects/integrate_auto_allocate.wrapper.f90`:
  its copy-call scan matches ANY `CALL x(..., arg%field, ...)` and hoisted BOTH
  `LoadDynamicLib` and the external DLL call into the wrapper — which would
  call each a second time; it declines a size expression that is a local
  PARAMETER; and its error handling emits `SetErrStat` and
  `CHARACTER(ErrMsgLen)`, OpenFAST names that exist nowhere in ROSCO. This unit
  needs exactly one ALLOCATE, and it is in the committed wrapper with the reason
  beside it. **The next unit that allocates a field of an argument will hit the
  same wall, and hand-editing a generated wrapper does not scale.** Fix belongs
  in VIT (X2).
- **`vit integrate` wrote `verification: simulation` into `vit.yaml`**, having
  run no simulation and read no result. It is FALSE for this unit — the gate is
  blind to it and the committed red test says so. Removed by hand with the
  reason in the file. Same shape as the `// After verification: <name> kernel
  PASSED` comment `37f8bdf` deleted from every generated file: nothing in the
  toolchain checks a claim a generator makes about verification.
- **`vit analyze-types --fix character` must not be used to close this gap.**
  It is what VIT's own error message suggests, and it rewrites ROSCO's type
  definition to a fixed length — changing `LEN(ErrVar%ErrMsg)`, which sizes
  `avcMSG`. That is a behavioural change to the oracle (P7). Recorded here
  because the suggestion is in the tool's output and will be read again.
- **A gitignored file can reconfigure the gate where no clean-tree check can
  see it.** `Examples/DISCON.IN` is ignored at `.gitignore:78`. A run that dies
  on a signal cannot restore it from a `finally`, and `git status` stays clean,
  so `done.py`'s P2 cannot catch it. `gate.py` already snapshots and restores
  these; nothing else did. The general form: **any restore that must survive a
  crash belongs in a parent process, not in the process that crashes.**
- **`coverage/line_coverage.json` cannot distinguish "never ran" from "never
  instrumented".** `scripts/coverage.py` stores only lines with a NON-ZERO hit
  count, so both are the same empty dictionary. Four files read as empty today —
  `ExtControl.f90`, `Constants.f90`, `ROSCO_Types.f90`, `ZeroMQInterface.f90` —
  and only re-running with the denominator printed tells them apart
  (`ExtControl.f90` is genuinely `0/28`, 27 times). The per-scenario log line
  already prints `N/M`; the *artifact* drops M. Storing the denominator would
  close it, and would be an addition.
- **A hot line is not an observable line, and nothing in the campaign's coverage
  data can say which is which.** `Conv2UC` runs 1.3M times and no gate
  perturbation of it moves any output, because its result is only ever compared
  against another of its own results. The verification ledger (E5.2) needs a
  column for this that is distinct from both "unexercised" and "argument held
  constant" — it is a *consumer* property, not a producer one, and neither
  coverage nor the gate's own red test in its usual form can detect it. The
  general instrument found here is to perturb toward ABSENCE (`if (false && …)`)
  rather than toward a different answer; see DECISIONS.md for the candidate
  method amendment this suggests, which is flagged and not made.
- **The differential harness refuses a CHARACTER ARRAY dummy.** PARTLY CLOSED at
  unit #8 and moved to the top of this list. An EXPLICIT-SHAPE one whose element
  count is another dummy — `Words(NumWords)` — now crosses. An ASSUMED-SHAPE one
  — `FileLines(:)`, which is what `FindLine`, `ParseInAry_Opt`, `ParseDbAry_Opt`
  and the four `ParseInput_*_Opt` units take — still does not: its extent lives
  in a descriptor `build_c_params` does not emit.
- **The harness varies a string's length over {1, 3, 8}, not over the 200
  `MaxParamLength` gives it.** `_extent_plan` assigns a lone free extent the
  single value 3; the string stage adds 1 and 8 without touching that plan,
  deliberately, because widening `_extent_plan` would change the case set of
  every unit already scored. A defect that only appears past the eighth
  character is not covered.
- **`vit check -f <file>` attributes findings to the FILE, not the function.**
  It reported `minval-endpoints` and `array-section-row` against a 6-line
  translation containing neither; both came from `interp1d`/`interp2d` elsewhere
  in `Functions.f90`. `--function` only sets the report header. Every finding on
  a multi-procedure source has to be re-attributed by hand, and a checker that
  cries wolf twice per unit is a checker sessions will learn to skip. Fix belongs
  in VIT (X2), not here.
- **One argument of unit #3 is invisible to both bit-exact layers.**
  `aziOffset` is 0 at every `ColemanTransformInverse` call site in every
  scenario. The differential harness covers it and nothing else does. This is P9
  at ARGUMENT granularity — coverage counts a line as exercised while one of its
  inputs is a constant throughout — and the campaign's verification ledger (E5.2)
  should be able to say which arguments each unit's bit-exact layers bound.
- **ESCALATION 1 is ANSWERED: VIT learned the C descriptor.** `37f8bdf` +
  `cf885e3`, reaching all three generators the escalation said it would have to
  — `interface_gen`, `test_validate.generate_fortran_bridge`, and the loop's
  `vitbridge`/`emit`. `AddToList` is integrated on it with a mutation score of
  1.000. **It does not automatically unblock the other three** —
  `Read_OL_Input`, `ParseInAry_Opt`, `ParseDbAry_Opt` — whose `bridge_feasible`
  verdicts came from the same pre-`integrates` matrix and have to be
  re-measured rather than inferred from this one.
- **ESCALATION 2 STANDS, and is sharper.** `AddToList` closed without the gate
  ever seeing it, because the harness and the mutation score do not need it.
  That is the shape of an answer for a dead unit — but only for one whose
  signature crosses. A dead unit that cannot cross has no harness, no mutation
  score, and a vacuous P9, and `loop/done.py` still has no branch for
  `integrated_unexercised`. The vocabulary has the word; the verifier does not.
- **A translation's mutation score can be raised by REMOVING restatements, and
  that is not gaming it.** Two survivors here were a `malloc(isize+1)` and a
  `memcpy(..., isize+1)` restating a quantity the allocation already fixed;
  perturbing either produced a memory error no value comparison can see. Naming
  it once (`nsize`) left one site, and that site decides the extent the caller
  sees. Two others survive genuinely and are DECLARED equivalent with reasons
  (`mutation/AddToList.equivalences.json`). The distinction — removed vs
  declared — is the thing to preserve: declaring the first pair away would have
  recorded a blindness as a property of the mutants.
- **`harness/cppmutate.py` reads a C++ template bracket as a comparison.**
  `static_cast<int*>` mutates to `static_cast<=int*>`, which does not compile. 8
  of 33 mutants on `AddToList` (24%) were that, one point under the 25% at which
  `vit_mutate.py` REFUSES to score. **Seen a second time at unit #4**: 4 of 18
  (22%) on `Conv2UC`'s two `static_cast<unsigned char>`s. Two units, both within
  three points of the refusal threshold, for a reason with nothing to do with
  either translation. This is now a pattern rather than an anecdote and belongs
  in the loop repo's mutator: mask template argument lists the way `cppmutate`
  already masks string and character literals.
- **The differential harness varies this unit's allocation status but not its
  length.** With a single extent parameter `_extent_plan` gives the same value
  (3) in every case, so a defect that only appears at another length is not
  covered by the 43 cases. Recorded in `plan.json`'s `observability`.
- **`bridge_feasible` verdicts in `plan.json` were derived from a column that
  measures the wrong generator.** VIT's conformance matrix fed `bridge` and
  `compiles` from `test_validate.generate_fortran_bridge` — the differential
  harness's Fortran side — while `vit integrate` ships the output of
  `interface_gen`. Fixed in VIT (an `integrates` column, and a refusal), but
  **the plan's 64 `yes` verdicts were computed before that**, and one of them —
  `c_complex_in`'s shape — is now known not to integrate. The plan's
  feasibility column should be re-derived against the new matrix before it is
  trusted again.
- **THREE SCENARIOS EXECUTE NO CONTROLLER CODE AT ALL.** Scenarios 10, 14 and
  24 run 0 lines of Controllers.f90, ControllerBlocks.f90, Filters.f90 and
  Functions.f90; scenario 13 runs 12 lines of Functions.f90 and 0 of
  Controllers.f90. They enter DISCON, read parameters, and stop —
  DISCON.F90 42%, ReadSetParameters 39%, everything downstream 0%. Scenario 14
  reports `PASSED` and 11 of its 13 channels are a single constant. They
  contribute their values to the gate's 5,252,000 while constraining nothing
  about the controller. This is **E3.3's exact failure mode** and E3.3 is still
  `manual`. Measured, not inferred: `coverage/line_coverage.json`.
- **E2.3 is not closed by `coverage/line_coverage.json`.** That file is real
  per-scenario line coverage and it is what C2 now selects call sites from, but
  it comes from a `--coverage -O0` build, not the campaign's Release build. The
  criterion asks for coverage from a clean build committed as phase evidence.
- **The launcher's protect glob has a hole.** `rosco/controller/src/*.f90`
  matches 11 files and NOT `DISCON.F90` — the one file `vit extract` modifies
  (strips CRLF, even under `--dry-run`). Add
  `--protect 'rosco/controller/src/*.F90'`.
- **`vit verify` and `vit integrate` rewrite `vit.yaml` and strip every
  comment.** The file is declared `derive` and its provenance lives in those
  comments; three runs deleted it three times. Currently restored by hand after
  each unit. Either VIT should round-trip comments or the provenance should live
  somewhere VIT does not write.
- **`regress.sh --baseline` does not restore `Examples/DISCON*.IN`.** Only
  `gate.py` snapshots and restores them. A baseline regeneration leaves two
  files modified and the tree dirty, which blocks `done.py`'s clean-tree check.
- **The harness Makefile's LIBS can carry `kgen_utils.f90.o`**, an object that
  exists only because an extraction left it in the build tree. A from-scratch
  build directory would not have it and the link would fail.
- `vit.yaml` is declared `derive` with `red_tested = false`. The planned test is
  to drop `assumed_size_arrays.avrSWAP` and confirm extraction breaks.
- Bootstrap otherwise incomplete: `phases.toml` still declares most criteria
  `manual`, which is NOT_EVALUABLE and never a pass.

## Closed

- **`reset_to_clean.sh` used to leave `CMakeLists.txt` integrated**, so "clean"
  meant the pre-integration SOURCE and not the pre-integration BUILD, and the
  differential harness link died on a second definition of its own translation.
  Closed in `6e614af`: the reset strips translated `.cpp` entries from
  CMakeLists and `restore_integrated.sh` puts them back. `harness.sh`'s LIBS
  workaround is kept — it is what makes the artifacts already committed for
  unit #2 reproducible.

- **`ColemanTransform`'s closure was asserted; now it is measured.**
  `scripts/done_check.py` runs the loop's own `DoneVerifier` with exactly the
  configuration `run_campaign.py` builds. At the start of the second pass it
  returned INCOMPLETE 10/13; it now returns COMPLETE 13/13. The failing
  predicate was P12: `vit_mutate.py` wrote `total`/`equivalent` and P12 reads
  `mutants`/`equivalent_declared`, so a genuine 35/35 was invisible to the check
  that requires one — and it failed with the WRONG REASON, "the operator set
  reached nothing in this unit". Fixed in the loop repo (`13265e7`, both
  spellings emitted) and the artifact regenerated by the tool, not hand-edited.
- **Three defects in this campaign's own scripts, each found by a run that
  should have worked.** All three were invisible while the stale artifacts sat
  on disk looking like results. See DECISIONS.md.
  1. `harness.sh` failed on VIT `d07a716`'s per-unit test directory — the skew
     it existed to reconcile had closed.
  2. Its post-integration LIBS extraction passed the literal words `LIBS` and
     `+=` to the linker, because the new VIT's Makefile has a second `LIBS +=`
     assignment its `^LIBS =` grep could not see. It now asks `make`.
  3. It aborted one line before stamping whenever `./test` exited non-zero — so
     the RED artifact, the one that most needs to say what it measured, was the
     only one that could not.

- **`AddToList` was `blocked`; it is now `integrated`.** The blockage was
  escalation 1, and the answer was to build the feature the refusal was standing
  in for rather than to close the unit as impossible. What made it closable is
  that its verification never needed the gate: a differential harness against
  the clean Fortran (43 cases, both branches red-tested), a mutation score of
  1.000, and a post-integration harness for the wrapper. The gate's green for it
  is still vacuous and is still committed beside the red test that says so.

- **The extraction blocker was a dead call site, not a tool defect.**
  `vit extract` printed `✓ Extraction successful` and `WARNING: No state data
  captured.` for `AddToList` three times. It was aimed at
  `ROSCO_IO.f90:1126`, which coverage now shows has **zero hits in all 27
  scenarios** — as do all five `AddToList` call sites. Pointed at a call site
  that executes, extraction works: `ColemanTransform` at `Controllers.f90:510`
  captured 63 state files, and kernel replay is available as a verification
  route. The tool was reporting success about instrumentation it had genuinely
  installed; the run never reached the pragma.
- **E1.2, previously "declared but unmet".** Closed 2026-08-10 — see Evidenced.
- **Reset-to-clean**, which did not exist: `scripts/reset_to_clean.sh` and
  `scripts/restore_integrated.sh`, red-tested.

## Unit #5, three dispatches — 2026-08-11

**Each dispatch's blocking claim was refuted by the next one, and saying so is
part of the result.**

* **First dispatch** closed `blocked` on *"there is no runnable oracle for
  ExtController anywhere in this campaign"*. The SIGSEGV it measured was real
  and is still true of the campaign's own inputs; the generalisation was not.
  `DLL_FileName` is the literal string `"unused"` in all 14 `Examples/*.IN`, so
  the crash was a property of the INPUT. **Sixty lines of C** made the original
  run to completion.
* **Second dispatch** built that fixture, closed the `CHARACTER(:)` view-struct
  gap and the 132-column bridge-generator gap, and closed `blocked` again on
  three remaining items — one of which it correctly called *a judgement, not a
  feature*.
* **Third dispatch** did the three items and found a fourth that was larger
  than all of them. The judgement (C) is made and written where it can be
  reviewed: `harness/ranges.toml`, four pins, each with the measurement that
  forces it. Every one exists because the REFERENCE crashes on the rest of the
  domain — `ExtController` never checks `ErrVar%ErrStat` after
  `LoadDynamicLib` and calls through a null `ProcAddr` — which is itself a
  finding about upstream ROSCO, and the file says so.

**What the three dispatches cost, and what it bought.** Two `blocked`
dispositions on a unit that closes `integrated`. What made the difference each
time was not new information about the function: it was asking *which
instrument needs this* instead of *is this unit verifiable*. The first dispatch
escalated "an oracle must be constructed" as SPEC §8.4's call because it assumed
an oracle meant a gate scenario; it did not. The third found that the
differential bridge had been discarding outputs for the whole campaign, which
no amount of reasoning about `ExtController` would have surfaced — only
running the harness did.

## Done-condition, unit #5

Run at the end of the third dispatch, after both commits, per RUNBOOK. The
verdict and its transcript are in `evidence/ExtController/done_check.txt`; the
second dispatch's INCOMPLETE 11-of-13 is kept beside it as
`done_check.second_dispatch.txt`, because a verdict that was correct when it was
taken is worth more than a deletion.

Worth carrying, from the second dispatch: the first attempt piped the run into
that file, and **creating the file made the tree dirty**, so the artifact
recorded `P2 FAIL dirty_tree` — describing itself rather than the unit. A
done-condition capture has to be taken before the file that captures it exists,
or written as a transcript.
