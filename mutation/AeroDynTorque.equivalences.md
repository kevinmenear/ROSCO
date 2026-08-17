# `AeroDynTorque` — the three declared equivalences

Prose companion to `mutation/AeroDynTorque.equivalences.json`, whose KEYS are
what `vit_mutate.py --equivalences` reads. A declared mutant **leaves the
denominator**, so every entry here is a claim somebody has to be able to answer.

**Three are declared and four survivors are NOT.** The score is **0.8750**, not
1.000, and the four are in `evidence/AeroDynTorque/mutation.census.txt` with the
input that would kill each and the measurement that shows the corpus does not
contain it. Read the split before the score.

**All three declarations are EQUIVALENCES over the whole input structure**, not
the weaker *unreachable-with-a-measurement* claim that interp2d used for four of
its ten. Nothing here rests on what the corpus happened to draw.

## `04ed59c3` — `n > 0` → `n >= 0` in `errmsg_trim`

```c++
const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
```

The two conditions differ at exactly one value of `n`, namely `0`, and there the
tight form takes `: 0` and yields `0` while the loose form takes
`static_cast<size_t>(0)` and yields `0`. Same length, same view, same string.
For every other `n` the conditions agree, so no input of any kind distinguishes
them — this is a two-line proof, not a sweep.

Third unit at this site, in the same copied helper. interp1d's `b61327db` was
declared against an exhaustive sweep of all 4,294,967,296 `int` values reporting
0 differences (`evidence/interp1d/equivalence_probe.txt`); interp2d's
`f0949202` states the reason that sweep found nothing; this is the same reason
again.

**Not removed.** A negative length is the view's NOT-ALLOCATED convention, and
the guard is what collapses it to the empty string rather than to a
four-billion-byte view.

## `b600adf5` and `0490bff2` — the two `fmax` argument swaps

```c++
const double WindSpeed = std::fmax(LocalVar->WE_Vw, DBL_EPSILON);   // b600adf5
AeroDynTorque_result = std::fmax(AeroDynTorque_result, 0.0);        // 0490bff2
```

Swapping the arguments of a two-argument selection function asks whether that
function is commutative, and it can fail to be at exactly two kinds of input: a
tie whose operands have distinct bit patterns, which for `double` is only
`±0.0`, and a NaN operand. **Measured on this toolchain rather than read out of
a standard** — unit #24 ran both orders of both and compared bits
(`evidence/saturate/minmax_probe.txt`):

```
fmax(-0.0, +0.0) = 0000000000000000     fmax(+0.0, -0.0) = 0000000000000000
fmax( NaN,  1.0) = 3FF0000000000000     fmax( 1.0,  NaN) = 3FF0000000000000
```

Commutative on every input the type admits. Fourth unit to declare this pair,
after `saturate`, `PIController` and `ratelimit`.

Declared as two entries rather than one because they are two sites and a reader
checking one should not have to infer the other.

**Not removed, and the reason is a different measurement.** gfortran's `MAX`
intrinsic *is* `fmax`: unit #24 ran gfortran's own `MIN(MAX(...))` at this
campaign's flags against all three C++ spellings over 12,167 triples and both
ternary spellings differ from it — 789 and 561 of 12,167, at a signed zero and
at a NaN, in opposite directions. Replacing either `fmax` with a branch to make
these mutants die would be a mistranslation.
