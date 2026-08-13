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

Loop `984a35f` adds `_KNOB_PAIR_LIMIT = 4096` inside the fallback, subsampling
at an **even stride** rather than taking a prefix — the pairs come out in
`(a, b)` lexicographic order, so a prefix would keep every combination of the
first knobs and none of the last. Both the count taken and the count dropped go
into the artifact's coverage line.

```
287,425 -> 46,940 cases
```

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
