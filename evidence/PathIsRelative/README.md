# PathIsRelative — unit #15

`LOGICAL FUNCTION PathIsRelative(GivenFil)` in `rosco/controller/src/ROSCO_Helpers.f90`.
Three `INDEX` tests: a path is absolute if it contains `":/"` or `":\"`, or if
its first character is `"/"` or `"\"`; everything else is relative.

**Five layers ran. Four are alive; the kernel is a lookup table and its own stub
says so.**

| layer | result | red test |
|---|---|---|
| kernel replay | 1/1 case, 197 field rows, ALL `IDENTICAL` | constant-`.FALSE.` stub **PASSES** 197/197 (lookup table); constant-`.TRUE.` stub **FAILS**, moving exactly `perffilename` |
| differential harness (clean Fortran) | 387 checked, 0 failed, 0 inadmissible | the answer INVERTED fails **387 of 387**, naming `vit_result` |
| mutation score | **26 of 26** behavioural killed, **1.000**, **0 declared equivalent**, 0 no-compile | the first form scored 0.938 — see below |
| post-integration harness (the wrapper) | 387 checked, 0 failed | `LEN(GivenFil)` → `LEN(GivenFil) - 1` at the bridge call fails **4 of 387**, and 4 is exactly the number of corpus cases whose answer depends on the last character |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **no value moved in either direction** — and the `.TRUE.` perturbation **stopped 24 of 27 scenarios from running at all** |

## The finding: a perturbation can be maximally visible and still read `went_red: false`

`gate/PathIsRelative.redtest.json`. Forcing the unit to answer `.TRUE.` makes
`ReadControlParameterFileSub` prefix `PriPath` onto an already-absolute
`CntrPar%PerfFileName`, and the run does not finish wrong — it **dies**:

```
At line 879 of file rosco/controller/src/ReadSetParameters.f90 (unit = 10)
Fortran runtime error: Cannot open file
  '/workspace/ROSCO-r2/Examples//workspace/ROSCO-r2/Examples/Tune_Cases/../Test_Cases/NREL-5MW/Cp_Ct_Cq.NREL5MW.txt'
```

(`gate.always-true-scenario1-runtime-error.log`.) 24 of the 27 scenarios abort;
the three survivors are 10, 14 and 24, the scenarios that reach no `.TRUE.`
answer. A scenario that dies produces no values, so `mismatched` stays 0,
`compared` falls from 5,252,000 to 260,000 — and `went_red`, which is
`mismatched > 0`, reads **false** while printing *"either the line is never
executed by these scenarios, or the gate cannot observe it"*. That sentence is
false of this run.

`gate.redtest.as-taken-scenarios-died.json` is the artifact **as first taken**,
kept unmodified (C12). `scripts/gate.py` now also records
`perturbation_broke_scenarios` and prints an accurate message; `went_red` and the
exit code are deliberately unchanged (X3). No earlier artifact in `gate/` has a
non-empty `scenarios_failed`, so nothing already committed is re-read by this.

## What the gate cannot see: the other answer

`gate.always-false-MOVES-NOTHING.json` — forcing the unit to answer `.FALSE.`
moves **0 of 5,252,000** with all 27 scenarios alive and `replacements: 1`,
`revert_verified: true`. The two call sites are one line apart and only one of
them can carry a defect out:

* `CntrPar%PerfFileName` is absolute in all 14 input files, so the answer is
  `.FALSE.` and the branch is skipped — a wrong `.TRUE.` kills the run, a wrong
  `.FALSE.` changes nothing.
* `CntrPar%OL_Filename` is the literal `"unused"` wherever `OL_Mode = 0`, so the
  answer is `.TRUE.` and the branch **does** run — 25 times across the campaign
  — but the value it builds is read only when `OL_Mode > 0`, and every scenario
  with `OL_Mode > 0` is handed an absolute path by `vit_sim.py`. The one live
  `.TRUE.` answer writes a string nothing reads.

`gate.control-getwords-perturbed-MOVES.json` is the same-build control unit #9's
entry requires for a red test that comes back green: the known-red `GetWords`
perturbation reproduces **1,857,893 of 5,252,000** on this build, byte-identical
to `gate/GetWords.redtest.json`. The blindness is a property of the unit, not of
a broken chain.

## The kernel is a lookup table, and the constant stub is the proof

The call site is inside an `IF` CONDITION, so KGen instruments the enclosing
statement and the compared field is `cntrpar` — the unit's result is named
nowhere (unit #12's shape, unit #7's check). Both stubs were run:

* `kernel.constant-false-stub-PASSES.verify_fields.csv` — a stub reading **no
  argument** scores 197 of 197 `IDENTICAL`. `.FALSE.` is the only answer this
  call site ever produces.
* `kernel.constant-true-stub-FAILS.verify_fields.csv` — the wrong constant moves
  exactly **1** row, `perffilename`, which is the branch's only output. The
  comparison is alive; being alive buys one bit.

VIT itself returned `NON_DISCRIMINATING` (no by-value floating-point argument)
and was right to.

## Why the translation writes predicates instead of INDEX

`mutation/PathIsRelative.survivors_index_position.json` is the measurement. A
faithful `index_of(s, len_s, sub, len_sub)` returning the position scores
**0.938**, and both survivors are its loop bound — `len_s - len_sub` →
`len_s + len_sub`, and `+ 1` → `+ 2`. Both run the search **past the end of the
buffer**: they are out-of-bounds reads, not wrong answers, and no value
comparison can see them. The position they return is also a quantity nothing
downstream reads — all three calls are immediately compared against 0 — so every
site computing *which* position matched is unobservable by construction (unit
#4's `LEN_TRIM` lesson, one intrinsic over).

Written as two predicates whose loop bounds carry no arithmetic, the same corpus
kills **26 of 26**. `corpus_answer_distribution.txt` is why that is not luck: the
387 cases answer `.TRUE.` 331 times and `.FALSE.` 56 times (confirmed twice —
by decoding the case file, and by the two constant stubs failing 331 and 56),
and they contain the reference's own two-character sets 8 times at the start of
a string and 4 times at the end, which is exactly what the two boundary mutants
need in order to die.

## Files

| file | what it is |
|---|---|
| `pathisrelative.final.cpp` | the shipped translation |
| `kernel.verify_fields.csv` | the passing kernel run, 197 rows |
| `kernel.constant-false-stub-PASSES.verify_fields.csv` | the kernel as a lookup table |
| `kernel.constant-true-stub-FAILS.verify_fields.csv` | the comparison is alive: 1 row, `perffilename` |
| `pathisrelative.constant-false-stub.cpp`, `...constant-true-stub.cpp`, `...inverted-stub.cpp` | the stubs those runs used |
| `gate.redtest.as-taken-scenarios-died.json` | the red test **as first taken**, before `gate.py` reported broken scenarios |
| `gate.always-true-scenario1-runtime-error.log` | why 24 scenarios died |
| `gate.always-false-MOVES-NOTHING.json` | the other answer, invisible: 0 of 5,252,000, 27 scenarios alive |
| `gate.control-getwords-perturbed-MOVES.json` | same-build control, 1,857,893 |
| `corpus_answer_distribution.txt` | what the 387 cases contain |
| `harness.index_position.json` | the first form's harness green — the same 387 cases, so the score difference is the code, not the corpus |
