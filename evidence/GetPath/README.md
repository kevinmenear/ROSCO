# GetPath — what verified it, and what could not

Unit #6 of rosco-r2. Definition `ROSCO_Helpers.f90:1220` (clean numbering), the
one call site `ReadSetParameters.f90:331`, scenario 1.

```fortran
 I = INDEX( GivenFil, '\', BACK=.TRUE. )
 I = MAX( I, INDEX( GivenFil, '/', BACK=.TRUE. ) )

 IF ( I == 0 ) THEN
    PathName = '.'//PathSep
 ELSE
    PathName = GivenFil(:I)
 END IF
```

| layer | result | red test |
|---|---|---|
| kernel replay, **1 case**, scenario 1 | 1/1 `IDENTICAL` on the full `CHARACTER(1024)` | no-op stub → 1/1 `OUT_TOL`. **AND a CONSTANT stub PASSES 1/1** — see below |
| differential harness, 236 cases vs clean Fortran | 0 failed | no-op stub → 234/236 failed, naming `PathName` |
| mutation | 25/25 behavioural killed, 1.000, 0 declared equivalent, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper marshalling) | 236 cases, 0 failed | `LEN(PathName)` → `LEN(PathName) - 1` in the wrapper's CALL → 199/236 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **FAILED — 0 of 5,252,000 moved.** Two perturbations; see below |

4 of the 25 mutation kills are CRASHES rather than case mismatches, so the
killed-by-comparison count is **21 of 25**.

## THE KERNEL HAS ONE CASE, AND A LOOKUP TABLE PASSES IT

`ReadSetParameters.f90:331` runs **once per process** — 28 hits across the 27
scenarios, one each but two in scenario 1 — and every scenario builds its
argument the same way:

```python
param_filename = os.path.join(this_dir, '<one of 14 DISCON*.IN names>')
```

so `GivenFil` differs only after the last `/` and the ANSWER,
`/workspace/ROSCO-r2/Examples/`, is the same string in all 27. Widening the
invocation window cannot help: there is no second invocation to widen onto.

That is measured, not inferred. `getpath.constant-stub.cpp` reads NEITHER input
and writes that literal blank-padded; it scores
`kernel.constant-stub-PASSES.verify_fields.csv` — **1/1 IDENTICAL**. The kernel's
green for this unit is a green for a lookup table.

`kernel.noop-stub.verify_fields.csv` is the other direction: a stub that writes
nothing scores `OUT_TOL`, so the instrument can move. Both are needed. This is
unit #2's all-zero window in a new costume — there the window was widenable and
the fix was to widen it; here the call site executes once and there is nothing
to widen.

VIT declined to construct its own red test and said why, which is correct — its
automatic red test perturbs a by-value floating-point argument and this unit has
none:

```
Red test: NOT CONSTRUCTED — no by-value floating-point parameter and no
floating-point result ... The kernel's discriminating power is therefore
UNMEASURED for this function
✗ GetPath: NON_DISCRIMINATING — 1 case(s) compared IDENTICAL
```

**For a CHARACTER output a kernel PASS *is* a bit-identity claim**, and the
generated comparison is kept (`kernel-generated-ReadSetParameters.f90`) so that
can be checked rather than believed: `IF (var == kgenref_var)` over
`CHARACTER(LEN=1024)`, no `rmsdiff`, no tolerance, no `IN_TOL` branch. The
comparison covers the blank fill, not just the path.

## THE GATE CANNOT SEE THIS UNIT — AND THE REASON IS A FOURTH ONE

| perturbation | artifact | moved |
|---|---|---|
| the `I == 0` branch forced off | `gate_redtests/GetPath.redtest-deadbranch.json` | 0 |
| `char_assign` made a NO-OP — the unit writes nothing at all | `gate_redtests/GetPath.redtest-noop.json` | 0 |

Both carry `perturbation.replacements: 1`, `perturbed: true`,
`revert_verified: true`, `residual_dirt: []`.

The first is the WEAK one and is committed as such: coverage already showed
clean line 1240 (`PathName = '.'//PathSep`) with **zero hits in all 27
scenarios**, so that perturbation lands on a line nothing runs. It is RUNBOOK's
"attempt 1", reproduced deliberately rather than mistaken for a finding.

The second is decisive. Making the unit a no-op leaves `PriPath` undefined, and
the gate still moves nothing. The cause is a **consumer** property and it is the
fourth distinct shape of P9 in six units:

* #1 `AddToList` — the line is never executed.
* #3 `ColemanTransformInverse` — an argument is constant in every scenario.
* #4 `Conv2UC` — the result is executed 1.3M times and cancelled downstream,
  because both operands of the comparison pass through it.
* #6 `GetPath` — the result is **produced and never consumed**. The line runs
  28 times, the two consumers run 28 times, and neither can carry it into an
  output.

`PriPath` has exactly two readers, both in `ReadSetParameters.f90` (clean 673,
674), both guarded by `PathIsRelative`:

```fortran
IF (PathIsRelative(CntrPar%PerfFileName)) CntrPar%PerfFileName = TRIM(PriPath)//TRIM(CntrPar%PerfFileName)
IF (PathIsRelative(CntrPar%OL_Filename))  CntrPar%OL_Filename  = TRIM(PriPath)//TRIM(CntrPar%OL_Filename)
```

and the guard is false exactly where the result would matter:

* `PerfFileName` is one absolute path in all 14 `Examples/*.IN`
  (`/workspace/ROSCO-r2/Examples/Tune_Cases/../Test_Cases/...`), so
  `PathIsRelative` is FALSE and `PriPath` is not used.
* `OL_Filename` is `"unused"` in most inputs — which IS relative, so the
  concatenation happens — but `OL_Mode` is 0 there and the file is never opened.
  In the three scenarios that do open it (`ol_mode1`, `ol_mode2`, `ol_cc_stc`)
  the path is absolute, so again `PriPath` is not used.

So the one scenario class that consumes `PriPath` discards the result, and the
one class that would use a filename supplies it absolutely. Nothing in
`coverage/line_coverage.json` can express that: line 674 is *hit* 28 times.

## What DOES constrain it

236 differential cases against the clean Fortran, 0 failed. The generator drew
its corpus from the reference's own literals — `\`, `/`, `.` and their collating
neighbours, 26 characters — across both strings, at lengths 1 / 4 / 9 and in the
mixed / word-then-blanks / all-blank shapes. That corpus is why the `I == 0`
branch the gate cannot reach is covered here: a 4-character string drawn from
that alphabet frequently contains no separator at all.

The no-op red test fails **234 of 236**, not 236 of 236, and the two survivors
are worth naming rather than rounding away: the bridge copies `PathName` IN
before the call (see the wrapper defect below), so a no-op returns the incoming
bytes — and in 2 cases the incoming bytes already equal the expected answer.

## The translation departs from literal transcription twice, each with a proof

1. **`char_assign` is a helper, not two transcribed assignments.** A Fortran
   CHARACTER assignment transfers `min(LEN(src), LEN(dst))` bytes and blank-fills
   the rest; both branches of GetPath are one. Writing the fill bound at each
   branch would restate what `len_dst` already fixes.
2. **Both of its loops are 1-BASED.** This one was measured. Written 0-based the
   score was **0.882** with two survivors, both `<` → `<=`
   (`mutation/GetPath.survivors_0based.json`): on the copy loop that writes
   `dst[n]`, which the fill loop immediately overwrites; on the fill loop it
   writes `dst[len_dst]`, one byte past the buffer. Neither is a wrong answer, so
   no value comparison can see either. 1-based, the same mutant LEAVES A BYTE
   UNWRITTEN — `dst` keeps its incoming byte, the comparison sees it, and both
   die. Score 1.000, zero declared equivalent.

   This is unit #1's lesson verbatim ("the literal transcription is the
   observable one") and the first time it has been re-measured on a different
   unit. Note the mutant COUNT rose, 17 → 25: the 1-based form has more mutable
   sites and all of them are observable. Fewer survivors, not fewer sites.

The forward-slash search takes the LITERAL `'/'`, not `PathSep`. On this build
they are the same byte; in the reference they are different things, and
collapsing them would change behaviour on a Windows build. P7.

`PathSep` itself is `CHARACTER(1), PARAMETER :: PathSep = '/'` from
`SysFiles/SysGnuLinux.f90`, which the build selects — confirmed in the build log
(`Setting system file as: src/SysFiles/SysGnuLinux.f90`), not read off
`CMakeLists.txt`.

## The unit found a KGen defect, and it is fixed in KGen

`vit extract` succeeded and the generated kernel **would not compile**:

```
CHARACTER(LEN=accinfile_size), DIMENSION(:), ALLOCATABLE :: accinfile
Error: Variable 'accinfile_size' cannot appear in the expression at (1)
```

kept at `kgen_driver.automatic_char_len.f90`. KGen hoists the call site's
enclosing procedure's dummies into the kernel driver's PROGRAM scope, and
`ReadControlParameterFileSub` declares
`CHARACTER(accINFILE_size) :: accINFILE(accINFILE_size)` — an AUTOMATIC length,
legal on a dummy and illegal on a local. KGen already had the machinery for the
neighbouring case (`CHARACTER(*)`, commit `c839e1a`); it keyed on `selector[0]
== '*'` and an automatic length is not that.

Fixed in KGen, not worked around (X2). See `../../DECISIONS.md`.

## `vit interface` emits a copy-IN for an INTENT(OUT) argument

The shipped wrapper reads `PathName` — an `INTENT(OUT)` dummy, undefined on
entry — into `PathName_c` before the call. Harmless for the answer here, because
the translation writes every one of `len_PathName` bytes. Not harmless for
OBSERVABILITY: it is why the no-op stub survives 2 of 236 differential cases, and
it would mask a translation that failed to write the tail by leaving the
caller's prior bytes there instead of leaving them indeterminate. Recorded in
`STATUS.md` as a VIT defect rather than hand-edited out of the generated wrapper.

## Files

| file | what it is |
|---|---|
| `vit_interface.stdout.txt` | the wrapper VIT emits, read attribute by attribute before any C++ was written |
| `vit_translate.stdout.txt` | the scaffolding prompt |
| `getpath.scaffold.cpp` | the scaffold, unedited |
| `vit_check.stdout.txt` | 2 findings, BOTH belonging to other procedures in the file — see `STATUS.md` |
| `vit_verify.stdout.txt` | the kernel run, including VIT's own `NON_DISCRIMINATING` verdict |
| `kernel.verify_fields.csv` | the one case, `IDENTICAL` |
| `kernel.noop-stub.verify_fields.csv` | the same case against a no-op: `OUT_TOL` |
| `kernel.constant-stub-PASSES.verify_fields.csv` | **the same case against a lookup table: `IDENTICAL`** |
| `getpath.noop-stub.cpp`, `getpath.constant-stub.cpp` | the two stubs |
| `kernel.GetPath.0.0.1.statefile` | the single captured case |
| `kernel-generated-ReadSetParameters.f90` | KGen's generated comparison, for the tolerance claim |
| `kernel-generated-kernel_driver.f90` | the driver AFTER the KGen fix — `CHARACTER(LEN=:)`, length read from the state file |
| `kgen_driver.automatic_char_len.f90` | the driver BEFORE it: the declaration gfortran refused |
| `harness_redtests/GetPath.redtest-noop.json` | 234 of 236 differential cases red under the no-op |
| `gate_redtests/*.json` | the two gate perturbations, each moving 0 values |
