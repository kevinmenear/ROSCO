# Conv2UC — what verified it, and what could not

Unit #4 of rosco-r2. `ROSCO_Helpers.f90:1579` (clean numbering), call site
`ROSCO_Helpers.f90:1118`, scenario 1.

```fortran
DO IC=1,LEN_TRIM( Str )
   IF ( ( Str(IC:IC) >= 'a' ).AND.( Str(IC:IC) <= 'z' ) )  THEN
      Str(IC:IC) = CHAR( ICHAR( Str(IC:IC) ) - 32 )
   END IF
END DO
```

| layer | result | red test |
|---|---|---|
| kernel replay, 63 cases, scenario 1 | 63/63 `IDENTICAL` | no-op stub → 31/63 `OUT_TOL`; green restored on revert |
| differential harness, 118 cases vs clean Fortran | 0 failed | no-op stub → 27/118 failed (`harness_redtests/`) |
| mutation | 14/14 behavioural killed, 1.000, 0 declared equivalent (4 nocompile EXCLUDED) | refuses to score unless the baseline is green |
| post-integration harness (wrapper marshalling) | 118 cases, 0 failed | `LEN(Str)` → `LEN(Str) - 1` in the wrapper's CALL → 24/118 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **FAILED — see below** |

## THE GATE CANNOT SEE THIS UNIT, AND IT IS NOT BECAUSE THE CODE IS DEAD

`Conv2UC` is called **1,333,146 times** across the 27 scenarios and converts
**4,558,823 characters** (`coverage/line_coverage.json`, clean lines 1579/1595).
It is one of the most-executed procedures in the controller.

Two perturbations of the integrated C++ were rebuilt and re-gated, and **each
moved 0 of 5,252,000 values**:

| perturbation | artifact | moved |
|---|---|---|
| `ch - 32` → `ch - 31` (every conversion off by one) | `gate_redtests/Conv2UC.redtest-offbyone.json` | 0 |
| the guard forced false — **no conversion at all** | `gate_redtests/Conv2UC.redtest-noconversion.json` | 0 |

Both artifacts record `replacements: 1`, `perturbed: true`, `reverted: true`,
`revert_verified: true`, so the perturbed library was genuinely built and run.

The reason is structural, and it is a different shape of blindness from
`AddToList`'s. Every value `Conv2UC` produces is consumed by **an equality test
against another `Conv2UC` output**:

```fortran
ParamNameUC = ParamName ;  CALL Conv2UC(ParamNameUC)     ! ROSCO_Helpers.f90:1106
FileLineUC  = Words(WordInd) ;  CALL Conv2UC(FileLineUC) ! ROSCO_Helpers.f90:1118
IF (FileLineUC == ParamNameUC) THEN ...
```

Both operands pass through the same function, so any perturbation of it is
applied to both sides. Equality survives every injective perturbation, and the
`no conversion at all` run shows it survives the non-injective ones the
scenarios reach too. The gate's green for this unit compares 5,252,000 values
and constrains none of them; it is committed next to the red test that says so.

**What this costs the campaign's coverage argument:** `coverage/line_coverage.json`
reports this line as among the hottest in the tree. A line executed 1.3M times
is still a line no output depends on. That is P9 at its sharpest so far — the
previous instances were an unexercised line (`AddToList`) and a constant
argument (`ColemanTransformInverse`); this one is an executed line whose result
is cancelled downstream.

## What DOES constrain it

The differential harness and the mutation score, both of which required a
feature that did not exist: `harness/generate.py` had kinds real / int / real[]
and **no string**. See `../../DECISIONS.md` and loop commit `9bee569`.

* 118 differential cases against the clean Fortran, 0 failed. Strings at
  lengths 1, 3 and 8, in three shapes (mixed / word-then-blanks / all-blank),
  over a 22-character corpus built from the reference's own `'a'` and `'z'`
  plus their collating neighbours `` ` `` and `{`.
* 14 of 14 behavioural mutants killed. 2 of the 14 are CRASHES rather than case
  mismatches (`IC - 1` → `1 - IC`, and `'1' -> '2'` in the same subscript, both
  indexing before the buffer), so the killed-by-comparison count is 12 of 14.
* 4 further mutants did not compile and are EXCLUDED from the score rather than
  counted. All four are `harness/cppmutate.py` reading a C++ template bracket as
  a comparison — `static_cast<unsigned char>` → `static_cast<=unsigned char>`.
  That is 22% against the 25% at which `vit_mutate.py` refuses to score: one
  more cast in this translation and it would have been unscoreable, for a reason
  with nothing to do with the translation. Second sighting; see `STATUS.md`.

## The kernel's window is alive, and the field log CAN say so this time

`kernel-window-1.cases.tsv` decodes all 63 captured state files. **31 of the 63
change the string**; the other 32 are already upper case, numeric, or empty.
That number is confirmed independently by the stub run:
`kernel-window-1.stub-fails.verify_fields.csv` scores exactly 31 `OUT_TOL`.

Unlike unit #3, RUNBOOK's liveness recipe is *runnable* here — VIT logs a
CHARACTER scalar with its computed and reference values, not as an empty array
row — but the recipe counts non-zero *numbers*, so the decode above is what
answers the question.

**And for a CHARACTER field a kernel PASS *is* a bit-identity claim**, which is
the exact opposite of unit #3's finding and for a readable reason. The
comparison KGen generated here, `kv_findline_character_maxparamlength_`, has no
tolerance branch at all:

```fortran
IF (var == kgenref_var) THEN  numIdentical + 1  ELSE  numOutTol + 1
```

There is no `rmsdiff`, no `kgen_tolerance`, no `IN_TOL`. So the caveat unit #3
recorded is **per-TYPE, not per-unit**: read the status column for a REAL
output; for a CHARACTER output the verdict and the status column agree by
construction. Both are recorded because a session that learned only the first
would over-hedge, and one that learned only the second would under-hedge.

`kernel-generated-ROSCO_Helpers.f90` is the generated kernel, kept so that claim
can be checked rather than believed.

## VIT declined to build its own kernel red test, and said so

```
Red test: NOT CONSTRUCTED — no by-value floating-point parameter and no
floating-point result ... The kernel's discriminating power is therefore
UNMEASURED for this function
✗ Conv2UC: NON_DISCRIMINATING — 63 case(s) compared IDENTICAL, and the kernel
did not report a mismatch when the translation was deliberately changed.
```

That is the tool refusing to claim what it did not measure, and it is correct:
its automatic red test perturbs a by-value floating-point argument, and this
unit has none. The manual no-op stub is the answer to it (31/63), and the
`NON_DISCRIMINATING` line is kept in the record rather than argued away.

## The translation departs from literal transcription, once, with a proof

The Fortran's loop bound is `LEN_TRIM(Str)`; the C++ uses `len_Str`. Every
character beyond `LEN_TRIM` is a blank, and a blank is not in `['a','z']`, so
the extra iterations write nothing and the two bounds agree on every input.

The reason to prefer the shorter one was **measured**: with `LEN_TRIM`
transcribed literally the score was **0.696**, and all seven survivors were
inside that computation — five of them out-of-bounds reads (`Str[len_trim + 1]`,
`Str[1 - len_trim]`, `len_trim >= 0` reading `Str[-1]`) that no value comparison
can see. Declaring those equivalent would have recorded a blindness in the
harness as a property of the mutants. Removing the restatement left one loop
bound, which every case can see, and the score is 1.000 with **zero** declared
equivalent. This is unit #1's "name a size once", one restatement further out.

## Files

| file | what it is |
|---|---|
| `vit_interface.stdout.txt` | the wrapper VIT emits, read attribute by attribute before any C++ was written |
| `vit_translate.stdout.txt` | the scaffolding prompt |
| `conv2uc.scaffold.cpp` | the scaffold, unedited |
| `conv2uc.stub.cpp` | the no-op stub both red tests used |
| `kernel-window-1.cases.tsv` | all 63 captured cases, decoded; 31 discriminating |
| `kernel-window-1.verify_fields.csv` | 63/63 `IDENTICAL` |
| `kernel-window-1.stub-fails.verify_fields.csv` | the same 63 against the stub: 31 `OUT_TOL` |
| `kernel-window-1.statefiles.lst` | which state files were replayed |
| `kernel-generated-ROSCO_Helpers.f90` | KGen's generated comparison, for the tolerance claim |
| `harness_redtests/Conv2UC.redtest-noop.json` | 27 of 118 differential cases red under the stub |
| `gate_redtests/*.json` | the two gate perturbations, each moving 0 values |
