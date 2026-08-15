# The y-direction NaN guard was dropped, and four instruments passed it

**Date:** 2026-08-15
**Status:** recorded BEFORE the fix (C12), against the wrong artifact
**Scope:** rosco-r2 unit #45 `interp2d`

## The defect

The reference guards BOTH directions against a NaN query. The translation
guards only the x-direction.

`rosco/controller/src/Functions.f90` at the clean baseline `54dd134`, the two
lines that begin the two corner searches:

```
231:        IF (xq <= MINVAL(xData) .OR. (ieee_is_nan(xq))) THEN   ! x-direction
257:        IF (yq <= MINVAL(yData) .OR. (ieee_is_nan(yq))) THEN   ! y-direction
```

`translations/Functions/interp2d.cpp` as shipped, at `10eacedf`:

```
230:    if (xq <= xData_min || std::isnan(xq)) {
253:    if (yq <= yData_min) {
```

`grep -c ieee_is_nan` on the reference is **2**. `grep -c isnan` on the
translation is **1**.

## What the two programs do on `yq` NaN with `xq` interior

| | reference | translation as shipped |
|---|---|---|
| `yq <= MINVAL(yData)` | false | false |
| `.OR. ieee_is_nan(yq)` | **true — takes the lower-y branch** | *absent* |
| result | `interp1d(xData, zData(1,:), xq)` | falls through to the `ELSE` loop |
| | | every comparison against NaN is false, so the loop runs to completion |
| | | `i = n_yData + 1`, then `i = i - 1 = n_yData`; `ii` is **read without having been written** |
| | | bilinear interpolation on an uninitialised subscript |

The reference has a defined, ordinary answer on this input. The translation has
none.

## The record said the opposite, and said it as a measured fact

`plan.json`, this unit's `observability` field, at `10eacedf`:

> `yq` NaN HAS NO ORACLE AND NO CASE. The x-direction guards with
> `ieee_is_nan(xq)`; **the y-direction does not.** On `yq` NaN with `xq`
> interior the y-loop completes without EXIT, `ii` is READ WITHOUT HAVING BEEN
> WRITTEN, and the reference has no answer.

The second sentence is false about the reference. It is a true description of
**the translation**, written as though it were a reading of the reference — so
the paragraph then correctly derives the translation's behaviour from it,
declares the input oracle-less, and closes the question. The same sentence is
repeated in the translation's own header comment (lines 206-211), where it
justifies leaving `ii` uninitialised:

> That asymmetry is upstream ROSCO's, and on that input the reference has no
> answer to compare against, the same standing as interp1d's `yData(I-2)`.

The asymmetry is not upstream ROSCO's. It is this translation's.

## Every instrument passed it

| instrument | result on the shipped translation | why it could not see this |
|---|---|---|
| differential harness, 1005 cases | 0 failed | no case draws `yq` NaN. `_fill_array` and the scalar ladders draw finite doubles; nothing in `harness/ranges.toml` asks for NaN. |
| mutation sweep, 166 behavioural | 134 killed | a mutation sweep grades the corpus against **this** translation. A missing statement is not a mutant of it. |
| post-integration harness, 1005 cases | 0 failed | same corpus, and after integration both sides run the same C++. |
| gate, 27 scenarios, 5,252,000 values | 0 mismatched | the scenarios interpolate Cp/Ct/Cq at `BldPitch*R2D` and `Lambda`. Neither goes NaN in any committed baseline. |

The one instrument that would have caught it is the one nobody ran: a
statement-for-statement diff of the two bodies. `vit check`'s registry has no
check that pairs each reference guard with a translated one.

## How it got past the reading, not just the instruments

The x-direction guard IS transcribed, at line 230, one screen above. Having
written `|| std::isnan(xq)` once, the second occurrence read as already handled.
The header comment then wrote the omission down as a property of the reference
and the `plan.json` narrative inherited it, so by the time the sweep ran there
were three artifacts all asserting the same wrong thing and none of them a
reading of `Functions.f90`.

**P7 says the oracle is the original source.** Three of this unit's artifacts
cite each other for a claim about the reference. `git show 54dd134:...` settles
it in one command, and no artifact ran it.

## Blast radius

Nil in the shipped library, and that is a fact about ROSCO's callers rather
than about this translation. `interp2d` is called from
`Functions.f90:365,371,377` (`CPfunction`, `CTfunction`, `CQfunction`) with
`BldPitch*R2D` and `Lambda`. A NaN reaching either would already have been a
fault upstream. The gate is unaffected and no baseline moves.

## The fix

`|| std::isnan(yq)` added at line 253, and the two paragraphs that asserted the
asymmetry corrected in the translation header and in `plan.json`. Recorded
here first, with the wrong artifact quoted, because C12 asks for the defect to
be written down before it is repaired.
