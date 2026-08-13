# The differential harness's first scaling wall

`bash scripts/harness.sh CheckInputs ...` at ROSCO-r2 `296a8ec`, loop `ff1e6e1`:

```
EXIT=137
```

SIGKILL. No traceback, no artifact, and nothing in the run that named the cause
— the last thing on stdout was the planner's `PREDICATE KNOB` listing.

## The arithmetic

`harness_case_arithmetic.py` runs the loop's OWN planner and generator with one
substitution: `_case_impl`, which materialises a full 467-parameter state, is
replaced by a counter. Nothing else is stubbed, so the counts are the
generator's.

```
varied parameters       148
HELD                    325
predicate knobs          86        <- 6 for the largest unit before this one
full knob cross product  2.7e53    (bound is 4096, so the ALL-PAIRS fallback fires)
CASES PLANNED         287,425      <- ReadAvrSWAP, the previous largest: 27,656
```

287,425 full states at tens of kilobytes each is several gigabytes in a
container with 7.9 GB, and `generate()` holds all of them: `cases` is a list and
the post-generation rules (R3's dead-signal scan, independence) index into every
element.

**The bound that existed was on the wrong quantity.** `_KNOB_CASE_LIMIT` bounds
the FULL PRODUCT and falls back to all PAIRS — but all pairs is `C(k,2)·v_a·v_b`
and R7b, R9 and R10 each cross every combination again, so the fallback is
quadratic in the knob count and unbounded in the case count. With 6 knobs it is
729 combinations and invisible; with 86 it is the whole run.

## What was done

Loop `984a35f` adds `_KNOB_PAIR_LIMIT` inside the fallback, subsampling at an
**even stride** rather than taking a prefix — the pairs come out in `(a, b)`
lexicographic order, so a prefix would keep every combination of the first knobs
and none of the last. Both the count taken and the count dropped go into the
artifact's coverage line.

**The value was set from measured memory, and the first value was still too
big.** 4096 was tried and SIGKILLed as well. The sweep, from the probe:

```
_KNOB_PAIR_LIMIT   4096  ->  46,940 cases   SIGKILLed  (7,869 MB of 7,922 in use)
                   2048  ->  32,604
                   1024  ->  25,436
                    512  ->  21,852         <- chosen, loop 7889dfc
                    256  ->  20,060
                    128  ->  19,164
```

The curve flattens because ~18,000 cases come from rules the bound does not
touch, so pair combinations past a few hundred cost memory for very little
corpus. 512 sits just under the **27,656** that `ReadAvrSWAP` — the same
signature shape, three view types and `avrSWAP` — has actually run here, which
is the only feasibility number in this campaign that was measured rather than
predicted.

```
287,425 -> 21,852 cases
```

## A second defect, found once generation could finish

With the corpus bounded, generation completed and the EMITTER failed:

```
File "harness/emit.py", line 1029, in write_cases
    seq = list(v or [])
TypeError: 'float' object is not iterable
```

No parameter named, no rule named, and four rules downstream of the cause.
`predicate_knobs_from` returns `(name, idx, values)`, and a **reduction**
predicate — `ANY(CntrPar%F_NotchFreqs <= 0.0)`, `MAXVAL(...) > x` — names the
whole ARRAY, so `idx` is `None` on a parameter that is not a scalar. The knob
loop wrote the scalar into the scalar override map.

**7 of CheckInputs' 86 knobs are this shape**: `F_NotchFreqs`,
`F_NotchBetaDen`, `SU_LoadStages`, `SU_LoadRampDuration`, `SU_LoadHoldDuration`,
`Ind_CableControl`, `Ind_StructControl`. No earlier unit had one, and that needs
no survey to establish: the path RAISES in the emitter, so a unit with such a
knob could not have produced an artifact at all.

Fixed in loop `2475425` by broadcasting the knob value over the array body.
Broadcast rather than one element: setting one element makes `ANY` true and
leaves the FALSE side at `_fill_array`'s ramp, which straddles the tested
literal for some extents and not others.

**It costs nothing already measured, and that is checked rather than asserted:**

```
$ grep -o "the full cross product\|ALL PAIRS ONLY" harness/*.json | wc -l
47   -- every one of them "the full cross product"
$ grep -l "ALL PAIRS ONLY" harness/*.json
(no matches)
```

No committed artifact in this campaign enters the fallback, so no scored unit's
corpus can move. This is P5 — closing the gap by addition — with the same shape
unit #24 used for a new mutation operator: measure the blast radius of the
change before making it, and report the debt rather than the intention.

## What it costs THIS unit

4,096 of 42,724 pair combinations, so **38,628 pair combinations are not
covered**. Which pairs of the 86 knobs go untested is decided by the stride and
not by any argument about which matter — that is the honest description, and it
is in the committed artifact's own coverage line rather than only here.

## A third defect, one layer down

With the broadcast in place the emitter refused a different case, this time
naming both the parameter and the numbers:

```
EmitError: CheckInputs: CntrPar_SU_LoadStages has 17 element(s) but its extents say 0
```

`CheckInputs` tests **both** `CntrPar%SU_LoadStages_N < 0` and
`ANY(CntrPar%SU_LoadStages < 0)`, and `SU_LoadStages_N` *is* that array's extent
— so the two are knobs on the same quantity, one on the count and one on the
contents, and a combination that sets the count to 0 met a 17-element body.

The extent wins, and the body is truncated or padded to it (loop `fc1b2e5`).
Dropping the knob instead would have been the other option and it is worse: the
count-and-contents conjunction in one case is exactly what R7 exists to reach.

## And the same `TypeError` again, from the copy

Fixing the whole-array path in R7 left the run failing at the identical line
with the identical message. The block that turns a knob combination into
parameter overrides is **written twice** — once in R7 and once in R7b, which
crosses every two-sided predicate with every knob combination — and only one
copy had been fixed. The second was producing the scalar in **3,078 of 21,852
cases**, and the emitter still named neither the parameter nor the rule.

Found by scanning the generated cases directly for an array parameter holding a
scalar, rather than by reading the traceback again:

```
CntrPar_F_NotchBetaDen        scalar in 3078 case(s), first at case 18726 = -1.0
CntrPar_SU_LoadHoldDuration   scalar in 3078 case(s), first at case 18726 = -1.0
CntrPar_Ind_StructControl     scalar in 3078 case(s), first at case 18726 = -1
CntrPar_Ind_CableControl      scalar in 3078 case(s), first at case 18726 = -1
CntrPar_SU_LoadRampDuration   scalar in 3078 case(s), first at case 18726 = -1.0
CntrPar_SU_LoadStages         scalar in 3078 case(s), first at case 18726 = -1.0
CntrPar_F_NotchFreqs          scalar in 3078 case(s), first at case 18726 = -1.0
```

3,078 is exactly R7b's own reported case count, and 18726 is where its block
begins. Both call sites now go through one `_knob_overrides()` (loop `222647b`).

**Four defects in one unit, all in one rule, none of which any earlier unit
could have found.** R7's fallback is quadratic in the knob count; its
`idx is None` path assumes a scalar; its bodies and its extents can be knobbed
independently; and it exists in two copies, so a fix to one is a fix to half the
corpus. Every one of them needs a unit with enough predicates over enough
shapes, and this is the first with 86 knobs against a previous maximum of 6.

**The emitter's message is the reason this cost four rounds.**
`TypeError: 'float' object is not iterable` names no parameter, no rule and no
case, and it is raised four rules downstream of every one of these causes. The
one error that DID name its parameter and its numbers — `CntrPar_SU_LoadStages
has 17 element(s) but its extents say 0` — was diagnosed and fixed in a single
pass. That difference is the whole cost.
