# The six named `arith_op` survivors, one at a time

**Date:** 2026-08-13
**Status:** completed — all six classified, two of the three categories used, no equivalence declared
**Scope:** unit #29, P12 / E4.6

The dispatch names six survivors of `mutation/CheckInputs.json` (78/173, 0.4509,
loop_rev `2e2295f`) and asks, for each, which of three things it is:

* **(a) genuinely equivalent** — the mutated program agrees with the original on
  every admissible input. Declare it, with a reason that can be disputed.
* **(b) the harness cannot reach it** — an admissible range too narrow to expose
  the difference. Fix the inputs, not the record.
* **(c) a blind spot no rule covers** — record as a known gap and escalate.

**None of the six is (a).** Four are (b) and two are (c). The verdicts are
measured by `corpus_index_shift_reach.py`, which reads the 23,076-case corpus
these mutants were actually scored against and reports, per site, whether the
mutant's own branch answer ever differs from the reference's. Output committed
beside it as `corpus_index_shift_reach.txt`.

## What they all are

All six — and four unnamed siblings of the identical shape — are `arith_op` on
the zero-based index conversion inside a Fortran-indexed loop:

```cpp
for (int Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode)
    if (CntrPar->AWC_freq[Imode - 1] < 0.0)        ->    [Imode + 1]
```

An index shift by two. Nothing about it is equivalent in general: it reads the
wrong element and, at the last two iterations, past the end of the array.

| id | line | array | verdict |
|---|---|---|---|
| `29e57417` | 825 | `Ind_CableControl` | **(c)** dominated by a later check on the same array |
| `bafd410e` | 830 | `Ind_StructControl` | **(c)** same |
| `2ed97e42` | 925 | `AWC_freq` | **(b)** trip count and array never perturbed together |
| `e250cdea` | 932 | `AWC_clockangle` | **(b)** same |
| `91a7adbc` | 933 | `AWC_clockangle` | **(b)** same |
| `e712c281` | 946 | `AWC_clockangle` | **(b)** the block is never executed at all |
| *(sibling)* `a50841f1` | 947 | `AWC_clockangle` | (b) never executed |
| *(sibling)* `1262a174` | 952 | `AWC_harmonic` | (b) never executed |
| *(sibling)* `69a82f24` | 979 | `CC_GroupIndex` | (b) predicate saturated |
| *(sibling)* `f9828789` | 996 | `StC_GroupIndex` | (b) predicate saturated |

## (b) — DISTINCT IS NOT THE SAME PROPERTY AS DISCRIMINATING, AND R11 SUPPLIED THE FIRST

Round one of the triage (`survivors_triaged.md`, bucket C/D) found 49 of 89
survivors were alive because R11's arrays were **constant** — an inert fill of
zeros and a whole-array knob that broadcast, so an index shift landed inside a
run of identical values. The repair landed at `2e2295f`/`7f75ca4`: a
`{"ramp": [start, step]}` form in the baseline state, and a whole-array knob
**placed** at first / interior / last as well as broadcast.

**The repair worked and the mutants still survive.** The ramp is visible in the
measurement — `valdiff` is every single iteration the loop runs:

```
mutant      line array              guard    iters     oob  valdiff  conddiff   cases
2ed97e42     925 AWC_freq           18616       19       0       19         0       0
e250cdea     932 AWC_clockangle       462        6       0        6         0       0
91a7adbc     933 AWC_clockangle       462        6       0        6         0       0
e712c281     946 AWC_clockangle        31        0       0        0         0       0
69a82f24     979 CC_GroupIndex       1000       31       0       31         0       0
f9828789     996 StC_GroupIndex        983      38       0       38         0       0
```

`valdiff` = 19 of 19 and `conddiff` = 0 of 19. The two elements the mutant
confuses are *different numbers* on every iteration, and the predicate gives
them the *same answer* every time. Distinctness kills an index shift only if
some element sits on the far side of the tested literal, and **every element of
an R11 array is admissible by construction** — that is what an admissible
baseline state means. The ramp `AWC_freq = 1.0, 2.0, 3.0, ...` is distinct and
uniformly `>= 0`, so `AWC_freq(Imode) < 0.0` is false at both indices, always.

Three properties are needed to kill an index shift and R11 supplies one:

1. the loop must run — the trip count is a **different** quantity from the array;
2. the array must be distinct — supplied by `ramp`;
3. some element must be on the far side of the predicate — impossible in an
   array that is admissible by construction.

Properties 1 and 3 are each a **second quantity away from baseline**, and R11
moves exactly one, on purpose (`generate.py`: *"a second quantity away from
baseline in the same case re-masks the first"*). It is bucket A's structural
limit reached from a new direction, and the answer is the same one: **a state is
how R11 reaches a conjunction.**

The corpus's other rules do move both, and they still do not discriminate:

```
                 array inadmissible   ... and its loop live
  AWC_freq             21,724                    6
  AWC_clockangle       21,729                    0
  CC_GroupIndex        21,729                   23
  StC_GroupIndex       21,729                   27
```

Twenty-one thousand cases with an inadmissible array, and six of them with the
loop live — and even those six give `conddiff = 0`. Two reasons, both about the
non-R11 fill rather than about the unit:

* `AWC_NumModes` is pinned to `[-1, 3]` in `harness/ranges.toml` — a pin that
  exists because the **reference** reads out of bounds when the count exceeds
  the extent (case 9544 SIGSEGV'd the oracle), so it is not available to widen.
  The loop therefore samples only elements 1..3 while the extent is 3..32, and
  the mutant samples 3..5. In a monotone `_fill_array` ramp across ±1e3 those
  two windows sit on the same side of zero.
* `CC_GroupIndex < 2601` and `StC_GroupIndex < 2801` are saturated the other
  way: every value the ±1e3 fill can produce is below 2601, so the predicate is
  true at both indices.

`e712c281`, `a50841f1` and `1262a174` are the strongest form of (b): their block
is `IF (AWC_Mode == 2)` and the corpus holds **31 such cases, none with
`AWC_NumModes >= 1`** (`-1: 28, 0: 3`). The loop body never executes. And it
cannot be reached by widening a range: the reference itself refuses
`AWC_NumModes` outside `{1, 2}` when `AWC_Mode == 2`, so an admissible
`AWC_Mode = 2` configuration must carry `AWC_NumModes` of 1 or 2 — which is a
**baseline state this unit does not have**. All three baseline states carry
`AWC_Mode` of 0 or 1.

**THE REPAIR IS NAMED AND NOT APPLIED.** Each of these needs a fourth R11
baseline state — `AWC_Mode = 2, AWC_NumModes = 2`, and arrays whose ramp
straddles the tested literal within the first `N+2` positions. That is a change
to `harness/baseline.CheckInputs.json`, which this dispatch prohibits precisely
because it would change the corpus and invalidate the three parts the merged
score rests on. It is escalated, not done. **(b) with the repair stated and
withheld is the honest form of (b) here** — the same shape as `CC_Group_N`'s pin
in round one.

## (c) — A KNOB THAT ARMS THE CHECK IT WAS AIMED AT ALSO ARMS THE ONE BELOW IT

`29e57417` and `bafd410e` are not unreached. They are reached exactly, by a case
built to reach them, and the difference is then overwritten.

```
  cases with OL_Mode > 0                    18612
  cable  loop iterations                   465313
  cable  cases whose SEGMENT differs        18611
  cable  cases whose any_lt DIFFERS             1   <- 29e57417
  struct cases whose any_lt DIFFERS             1   <- bafd410e
```

The mutant computes a different `All_OL_Indices` in 18,611 of 18,612 cases, and
the list's **only** observation is `ANY(All_OL_Indices < 0)`. A shift inside a
list of same-signed values is invisible to a predicate on the sign, so 18,610 of
those differences are absorbed. One is not:

```
  cable  case 22752: any_lt ref=True mut=False
         reference segment [-1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
```

Case 22752 is the R11 `open_loop_flap_fbp` state with exactly one knob applied:
`Ind_CableControl(1) = -1`, everything else admissible (`Ind_Breakpoint = 1`,
`Ext_Interface = 1`, `CC_ActTau = 1.0`, `CC_Mode = 1`). The reference reads
element 1, sees `-1`, and raises *"All open loop control indices must be greater
than zero"*. The mutant starts at element 3, never sees it, and does not raise
it. **That is precisely the case R11 exists to produce, and it works.**

It does not kill the mutant, because twenty-four lines further down the same
subroutine:

```cpp
if (any_gt(CntrPar->Ind_CableControl, CntrPar->n_Ind_CableControl, 0) &&
    CntrPar->CC_Mode != 2) {
    ErrVar->aviFAIL = -1;
    assign_errmsg(ErrVar, "CC_Mode must be 2 if using open loop cable control ...");
}
```

reads **the same array** and fires on the elements the knob did *not* touch —
the `11, 12, 13, ...` of the ramp — with `CC_Mode = 1`. `CheckInputs` has no
early return, so the later message overwrites the earlier one. Both programs end
with `aviFAIL = -1` and the identical string. The difference is real, is
produced, and is unobservable at the interface.

```
  cases with a NEGATIVE cable index           18161
    of which the later check also fires       18103
```

**This is the campaign's masking finding, but it is a new instance of it and the
existing rule does not cover it.** The recorded form (DECISIONS, unit #29) is
statistical: *"every case raises at least two errors, so the last one wins"*,
and its repair is R11 — a case that fails exactly one check. Here R11 did its
job and the case still fails two, because the **two checks read the same
parameter** and the knob that arms the first necessarily arms the second. One
quantity, two checks, in order: one-at-a-time perturbation cannot separate them.
No rule in `harness/` detects that shape, and no widening of any range fixes it.

Worse, the ramp repair *guarantees* it. Round one's constant array (all zeros)
would leave `any_gt(..., 0)` **false**, and the dominating check would not fire.
Making the array admissible-and-distinct is what makes the other elements
positive. **The repair for the first survivor is the mechanism that hides the
second.** That is not a criticism of the repair — it moved the score from 8 to
78 — but it is a real interaction between two corpus rules that nothing in the
apparatus can see.

**Escalated as a new category:** *a survivor whose observable difference is
overwritten by a later check on the same parameter, where the knob that reaches
the first check necessarily satisfies the second.* Killing it requires either an
early-return in the reference (which would change ROSCO), a second output
channel (an artifact carrying the message from *every* check rather than the
last), or a baseline state in which the dominating check is already satisfied —
here `CC_Mode = 2` and `StC_Mode = 2` — so the knob isolates the earlier one.
The third is one line of state per mutant pair and is the cheapest; it is a
corpus decision and is not taken here.

## What was NOT done, and why

* **No equivalence declared.** None of the six agrees with the reference on
  every admissible input; each was shown to differ on a specific input, or shown
  to be unreached rather than agreeing. `equivalent_declared` stays at 0.
* **No range widened, no baseline edited, no corpus regenerated.** Each would
  invalidate the five parts the 0.4509 rests on, which is the whole reason the
  re-take dispatch existed.
* **The score did not move and was not meant to.** 78 of 173 is what this corpus
  can see. What this document adds is the reason, per mutant, and the repair,
  per mutant, for whoever is allowed to change the corpus.
