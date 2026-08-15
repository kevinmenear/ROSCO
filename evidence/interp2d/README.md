# evidence/interp2d — unit #45

**SECOND DISPATCH, 2026-08-15, loop_rev `eb5028e`.** All four layers re-taken.
Everything below the horizontal rule is the first dispatch's record and is kept
unchanged; the numbers in the table are the current ones.

| layer | result | red-tested |
|---|---|---|
| differential harness vs clean Fortran, `interp1d_c` bridge kept | **1147** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **1147 of 1147** |
| mutation score | **152 of 166 behavioural killed, 0.9744**, 10 declared, 2 no-compile, **4 standing** | the score *is* the red test, 152 times |
| post-integration harness (wrapper only) | **1147** checked, 0 failed | the scoped copy-back deletion fails **161 of 1147** |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | the bilinear result × 1.000001 moves **922,411 of 5,252,000**; revert verified |

**No kernel.** `plan.json` allowed "kernel replay **or** direct-call harness".
That is a stated gap, not a silent one.

## What the second dispatch found

1. **A mistranslation all four instruments had passed.** The reference guards
   *both* corner searches against a NaN query; this translation guarded one. See
   `yq-nan-guard.MISTRANSLATION.md` — written before the fix, with the wrong
   artifact quoted, including the paragraph in `plan.json` that had recorded the
   omission as a property of upstream ROSCO.
2. **`ordered_only` was one shape too wide, and 17 of the 32 survivors died to
   correcting it.** What the reference cannot survive is an *inversion*, not a
   tie. `mutation.census.txt` partitions all 32 by cause; `equivalence_probe.cpp`
   and `errmsg_extremes_probe.cpp` are the two measurements behind the ten
   declarations.
3. **Four survivors are left standing, deliberately.** Each has an input in the
   reference's well-defined domain at which the mutant answers differently.
   `mutation.census.txt` names the input and the rule that does not exist to
   draw it, and `DECISIONS.md` escalates both classes.
4. **Two instrument defects, recorded before their fixes.**
   `generator.shadowed-lengths.txt` (a coverage sentence reporting another
   block's numbers) and `postintegration.retake-failed-once.txt` (a failed
   re-take deleting the artifact it was re-taking, and a commit claiming the
   green anyway).

---

# The first dispatch's record, unchanged

Four layers, **all four alive**, and the dispatch spent most of its clock on two
tool defects that stood between the unit and its primary instrument.

## The plan predicted this signature could not cross. It crosses — and the prediction was still right about something

`bridge_feasible: no`, on the conformance matrix's `c_assumed_shape_2d` cell.
That cell is measured on **one** of VIT's two Fortran-bridge generators.

```
vit test-validate  ->  zData(1:n_zData) against zData(:,:)
                       Error: Rank mismatch in argument 'zdata' (rank-2 and rank-1)   CONFIRMED
vit integrate      ->  BIND(C) interface: zData(*), n_zData_rows, n_zData_cols
                       wrapper passes its own zData(:,:) by sequence association
                       compiles, links, installs; gate 5,252,000 / 0             REFUTED
```

And the cell understates its own case: `interface_gen.build_c_params` has
**always** emitted `n_zData_rows` and `n_zData_cols` for this argument — the C++
scaffold VIT generated for this unit minutes earlier has both — so the harness's
C++ side was passing two extents into a bridge declaring one. The rank error is
what stopped it; had the ranks agreed, that skew would have been a silent misread
of the call frame.

Fixed in VIT (`f0971d8`), not around it. The `RESHAPE` spelling is **copied**
from `interface_gen._build_bridge_call_args`, which has emitted exactly it for
the kernel bridge since before this cell was measured: two generators disagreeing
about one calling convention is the defect being closed, and a second spelling
would be a second chance at it. `INTENT(OUT)`/`INTENT(INOUT)` is refused with its
reason (a `RESHAPE` is an expression and therefore a copy) and rank ≥ 3 for being
unmeasured. Positive control in `bridge.rank2-after.txt`: `interp1d`'s rank-1
bridge regenerated on the same tree with and without the change is byte-identical.

`tests/conformance/matrix.toml` still reads `compiles = "no"`. It is regenerated
by `tests/test_conformance.py --update`, which was **not** run here — that would
rewrite every cell from one dispatch's tool state.

## A generated bridge carried a precondition nothing tested

```
! Kernel stash: pointer to original Fortran type, set by wrapper before calling C++.
TYPE(ErrorVariables), POINTER, SAVE :: vit_original_errorvariables => NULL()
```

The differential harness calls the translation **directly**. There is no wrapper,
so the stash is null and the `interp1d_c` bridge dereferences it: SIGSEGV on case
0, and **no stdout at all**, because the harness prints its JSON after the loop.
This is the campaign's first harness whose callee takes a view type — every
earlier callee took a CHARACTER or a plain array.

Repairing only the pointer is half a fix, and the other half is the interesting
one: both existing converters move a deferred-length CHARACTER field through the
**module staging buffer**, which only a wrapper's own `vit_populate_<t>` fills.
The copy in silently left `ErrMsg` alone; the populate on the way out repointed
`view%ErrMsg` away from the buffer the caller owns.

```
stash alone                 checked 1117  failed 123   all on ErrVar.ErrMsg / n_ErrMsg
+ vit_view_in/out_<t>       checked 1005  failed   0
```

`callee_bridge.null_stash.txt` carries the three instrumented rebuilds that
narrowed it and the `ASSOCIATED` probe that settled it.

## The reference reads out of bounds on a table it has just reported as malformed

`interp2d` checks that both breakpoint vectors are strictly increasing, sets
`aviFAIL = -1`, writes a message — and then interpolates anyway. On a descending
table every interior query satisfies `xq < xData(1)` at the first iteration, so
the corner search exits with `jj = 1`, `j = j - 1 = 0`, and `zData(i, 0)` is one
column before the array. The reference reads what precedes **its** buffer and the
translation reads what precedes **its own**.

```
6 of 1117 cases reach an out-of-bounds corner; 5 of the 6 disagree
every mismatch on vit_result, none on any other output
```

`harness/ranges.toml` gained a fourth judgement kind for it, **`ordered_only`**,
which takes an array out of *both* rules that leave the ascending domain:

```
order ladder excluded only     1117 -> 1029 cases, 5 failures -> 4
+ R10's reversed body          1029 -> 1005 cases, 4 failures -> 0
```

## And that exclusion's cost was stated wrong — the mutation sweep refuted it

`ranges.toml` claimed the ordinary random draws would still reach the two
strictly-increasing checks "one time in six". `harness/generate._fill_array`
draws **strictly increasing bodies by construction**, so once the exclusions are
in, no case in the corpus has a non-increasing table at all.

The refutation is a measurement, not a re-reading: `ErrVar->aviFAIL = -1` inside
the lambda those two checks alone call, mutated to `-2`, **survived** — while
`ErrVar.aviFAIL` is compared on every one of the 1005 cases. One execution would
have killed it. The wrong sentence is kept in `ranges.toml` beside its correction
(C12).

## The 32 survivors, partitioned rather than listed

`mutation.census.txt` maps every one to a source line and a cause:

| group | count | cause |
|---|---|---|
| 1 | 14 | code the corpus never executes — both strictly-increasing checks, dead by `ordered_only` + `_fill_array`'s monotone draws |
| 2 | 2 | the size-mismatch branches, unreachable by the two extent ties |
| 3 | 4 | equivalence **candidates**: `if (x < m) m = x` vs `<=` computes the same extremum for any sequence |
| 4 | 6 | equivalence **candidates**: on an ascending table the bound branch and the on-axis branch return the same thing; and `xq < xData(j)` vs `<=` cannot differ, because `==` is tested on the line above |
| 5 | 6 | the ErrMsg staging helper and the two DO-variable initialisers |

**Nothing is declared equivalent.** Ten have a one-line argument and no probe,
and this campaign's rule is that a declaration rests on a measurement — unit #23
swept 4,294,967,296 values for one of the same MINVAL/MAXVAL mutants. They are
counted as survivors, and `0.8072` is what that costs.

## Two things written to avoid unkillable sites, before the sweep rather than after

Unit #43's rule — *does this spelling offer a mutant no input can kill?* — asked
while writing rather than while reading a survivor list:

- **Six of the reference's ten index assignments are dead and are not
  transcribed.** Every `jj = ...` and `ii = ...` standing immediately before one
  of the five `RETURN`s writes a subroutine local nothing reads afterwards.
  Their liveness is a property of the reference, not a reading of it. The
  `j = ...` and `i = ...` of the same branches **are** live — they subscript the
  slice being passed on — and are kept.
- **`fQ(2,2)` and the one-element `fxy` are not transcribed as arrays.** Four
  scalars and one, because a subscript whose only mutants are out of bounds is a
  site no input can kill. The four *corner reads* keep their subscripts, and
  those are killable: R5 makes array elements distinct.

## Files

| file | what it is |
|---|---|
| `bridge.rank2-before.txt` | the plan's prediction, measured: the rank mismatch and the parameter-list skew |
| `bridge.rank2-after.txt` | the same bridge after the VIT fix, with the rank-1 positive control |
| `callee_bridge.null_stash.txt` | the null-stash segfault, the 123-of-1117 half-fix, and what closed it |
| `size_mismatch_probe.f90` | INPUT to the abort measurement |
| `reference.size-mismatch-aborts.txt` | the abort, measured: LEN 0, 59 characters, exit 2 |
| `harness.out-of-bounds-corner-census.txt` | 6 of 1117, and which rule produced each |
| `interp2d.hstub-noop.cpp` | INPUT to the harness red test |
| `run_harness_redtest.sh` | the red-test runner |
| `run_mutation_part.sh` | one operator-filtered, guarded sweep per invocation |
| `mutation.census.txt` | all 32 survivors, by line, by cause |
| `scope_wrapper_perturb.py` | INPUT to the wrapper red test — scoped to this unit's wrapper |
| `run_wrapper_redtest.sh` | the wrapper red-test runner, with the restore and the `touch` |
| `harness.postintegration.redtest.json` | 46 of 1005 |
| `reset_restore_roundtrip.txt` | E1.4 re-verified with a 41st unit integrated |
| `done_check.txt` | the done condition, run after the commits |
