# Declared equivalences — unit #65 `WindSpeedEstimator`

Twenty-nine of the seventy-three survivors. The first thirteen are families
A-D and were declared at the unit's first dispatch; E-I were added at the P12
re-take and each carries a control that FAILED. An equivalence is a claim about the
PROGRAMS — the two agree on every admissible input — and each one below was
EXECUTED before it was believed (unit #48's sixth finding: its first model
proved three mutants behaviour-preserving from a clean inequality and was
false). The runner and its output are
`evidence/WindSpeedEstimator/probe_equivalences.{cpp,txt}`.

## A — `swap_operands` inside `ABS(a - b) > 0` (3): `ef5224a0`, `5abe61d7`, `b31be2ec`

The reference's operational-range ladder is three tests of the same shape:

    IF (ABS(WE_Inp_Pitch  - LocalVar%BlPitchCMeas)   > 0) THEN
    ELSEIF (ABS(WE_Inp_Torque - LocalVar%VS_LastGenTrqF) > 0) THEN
    ELSEIF (ABS(WE_Inp_Speed  - LocalVar%RotSpeedF)      > 0) THEN

`b - a` is the exact negation of `a - b` in IEEE-754 — subtraction rounds
symmetrically about zero — and `fabs` clears the sign bit, so the two spellings
agree on the VALUE and therefore on the predicate. Executed over 4,225 ordered
pairs drawn from 65 values including ±0, ±inf, both NaNs, the subnormal
boundary, `DBL_MIN`, `DBL_MAX` and the two doubles either side of 3.0: **0
disagreements, on the value and on the predicate separately.**

## B — `swap_call_args` on `std::fmax` (5): `bf0aa697`, `e41afb0e`, `34aaf49e`, `22a29bd8`, `20cc8068`

    MAX(LocalVar%HorWindV_F, 3.0_DbKi)        twice, the EKF restart arm
    MAX(WE_Inp_Speed, EPSILON(1.0_DbKi))      the tip-speed ratio
    MAX(LocalVar%WE%xh(1,1), EPSILON(...))    the rotor-speed floor
    MAX(0.0, Cp_op)                           the Cp clamp

`fmax` is symmetric by IEEE-754's `maxNum`: it returns the larger operand, and
where one is NaN it returns the other, in either order. **0 disagreements over
the same 65 values for the 3.0 and `DBL_EPSILON` sites.**

**THE FIFTH ONE WAS GOING TO BE LEFT STANDING AND THE PROBE REFUTED THAT.** The
argument for the other four was "symmetric except possibly at a (+0, −0) pair",
which would have left `MAX(0.0, Cp_op)` killable — `DebugVar%WE_Cp` is compared
bitwise, so a signed zero there is observable. glibc's `fmax` returns `+0` for
both `fmax(0.0,-0.0)` and `fmax(-0.0,0.0)`; the count is 0 and the sign bit is 0
either way. The wrong model is kept in the probe's own source, at the counter
that refuted it, because it is what the next reader would otherwise re-derive.

**THE CONTROL.** That zero-disagreement line cannot fail and so is not evidence
(P10). The line that can is the same comparison machinery over an operator that
is *not* symmetric with `fmax`: `fmax(a, 3.0)` against `fmin(3.0, a)`, **62
disagreements of 65**.

## C — `collapse_stride` where the collapsed index is the same index (2): `c95a7470`, `d58eaa09`

`F[0 * 3 + 0]` → `F[0]`. `0 * 3 + 0` is `0`, in integer arithmetic, exactly.
These are the two `F(1,1)`/`Q(1,1)` sites, the only two of the five
`collapse_stride` mutants whose collapse preserves the address. The other three
— `[1 * 3 + 0]` → `[1]`, `[2 * 3 + 0]` → `[2]`, `[j * 3 + i]` → `[j]` — address
different elements and are NOT declared. **That distinction is the proof
(unit #59's rule: an equivalence is declared per mutant, and the evidence is
that its sibling on the same line is not).**

## D — `drop_factor` and `arith_op` on the literal `0 * 3` (3): `24574509`, `af6d160a`, `95b1fd2b`

`0 * 3` → `0`, and `0 * 3` → `0 / 3`. Both are `0`, in integer arithmetic, at
the column subscript of `F(1,j)`/`Q(1,j)`. Same argument and same control as C:
the neighbouring `1 * 3` → `1` and `2 * 3` → `2` mutants are NOT declared,
because 3 ≠ 1 and 6 ≠ 2.

## E — `const_tweak` / `index_offset` that ENLARGE an array DECLARATION (10)

`0d7f3f68`, `e3b71c74` (`double F[9]` → `F[10]`, `F[9 + 1]`)
`4305aa33`, `d0bb2797` (`double H[3]` → `H[4]`, `H[3 + 1]`)
`73df48ea`, `f4a2fc4f` (`double dxh[3]` → `dxh[4]`, `dxh[3 + 1]`)
`09ec01aa`, `f46204aa` (`double Q[9]` → `Q[10]`, `Q[9 + 1]`)
`4dbb085d`, `e9035603` (`double S[1]` → `S[2]`, `S[1 + 1]`)

Ten of the sixty, and they are the same mutant twice over on five lines: the
mutator reaches a DECLARATION rather than a subscript, and every one of them
gives the array one more element than it had. That is behaviour-preserving iff
nothing ever accesses the element it adds.

**MEASURED, over the whole corpus, and the measurement is COMPLETE rather than
sampled.** The shipped translation was re-run with `operator[]` replaced by a
recording wrapper (`evidence/WindSpeedEstimator/windspeedestimator.subscript-census.cpp`):

```
SUBCENSUS F    extent=9 min=0 max=8 uses=182763
SUBCENSUS H    extent=3 min=0 max=2 uses=72651
SUBCENSUS dxh  extent=3 min=0 max=2 uses=7038
SUBCENSUS Q    extent=9 min=0 max=8 uses=126459
SUBCENSUS S    extent=1 min=0 max=0 uses=4692
```

Why it is complete and not a sample: every subscript site on these five arrays
is inside the `WE_Mode == 2` EKF arm and unconditional within it, and every
index is either a literal or a loop counter bounded by a literal. So one case
that enters the arm exercises every site with its full index range, and 1,173
do. The maxima above are therefore the maxima, not the maxima seen.

The added element is never written either, so it is never read back: there is
no uninitialised-read path through which a stack-layout change could become
observable. **The line that can fail is `max`**: if any of the five had come
back at `extent`, its two mutants would be out-of-bounds accesses and not
equivalences — which is exactly what the three `index_offset` mutants at
`:407`–`:409` are, and they are NOT declared.

## F — `drop_call` on `ABS()` at the TORQUE and SPEED range tests (2): `068d3be3`, `cd428761`

```
IF (ABS(WE_Inp_Torque - LocalVar%VS_LastGenTrqF) > 0) THEN
IF (ABS(WE_Inp_Speed  - LocalVar%RotSpeedF)      > 0) THEN
```

Both saturations are ONE-SIDED — each replaces its input with a strictly larger
value or leaves it alone:

```
IF (VS_LastGenTrqF < 0.0001*VS_RtTq) WE_Inp_Torque = 0.0001*VS_RtTq  ELSE = VS_LastGenTrqF
IF (RotSpeedF < b)                   WE_Inp_Speed  = b + EPSILON     ELSE = RotSpeedF
```

so the difference is `>= 0` or NaN, and `fabs` is the identity on the
predicate: `fabs(d) > 0` and `d > 0` are both false at `d = ±0` and at
`d = NaN`, and both true at `d > 0`. Executed over the campaign's 67-value grid
in both operands (and the gearbox ratio swept over `{1, 97, -1, 0}`):
**0 disagreements of 4,489 and 0 of 17,956, with `d != 0` in 1,975 and 8,601 —
so the confirming line had something to confirm.**

**THE CONTROL, and it is the mutant the sweep KILLED.** The PITCH test one line
above has the same shape and `saturate()` clamps in BOTH directions:
`fabs(d) > 0` vs `d > 0` **disagrees 3,149 times of 13,467**. Same probe, same
grid, same machinery — the argument is about the saturation and not about
`fabs`.

`evidence/WindSpeedEstimator/probe_equivalences2.{cpp,txt}`.

## G — `arith_op` `v_m + v_t` → `v_m - v_t` in the EKF RESTART arm (1): `d3f2e7f0`

The arm assigns, in this order and unconditionally:

```
LocalVar%WE%v_t = 0.0
LocalVar%WE%v_m = max(LocalVar%HorWindV_F, 3.0_DbKi)
LocalVar%WE_Vw  = LocalVar%WE%v_m + LocalVar%WE%v_t
```

so the mutant's operand pair is not (any double, any double): it is
`(max(x, 3.0), +0.0)`. The only double `y` for which `y + 0.0` and `y - 0.0`
differ is `y = -0.0`, and `max(x, 3.0)` is never `-0.0` — it is `>= 3.0`, or
`3.0` when `x` is NaN. **0 disagreements of 67.**

**THE CONTROL.** The same two spellings over an UNCONSTRAINED left operand —
the mutant this would be if the two statements above it were not there —
**disagrees 1 time of 67, at `-0.0`.** One case in sixty-seven is the whole
argument, and a grid without a negative zero in it would have reported this
family as equivalent for the wrong reason.

## H — `compare_op` `n > 0` → `n >= 0` in `errmsg_trim` (1): `a1db7de8`

```
const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
```

The two predicates differ only at `n == 0`, where both arms of the conditional
yield `0`. **0 disagreements over `n` in [-8, 8].**

**THE CONTROL** is the sibling `const_tweak` on the same line, `5f847ebc`
(`n > 0` → `n > 1`), which is **NOT** declared: it differs at `n == 1`, where
the arms yield 1 and 0. **1 disagreement**, and that is the distinction the
declaration rests on (unit #59's rule).

## I — `const_tweak` `1` → `2` at LPFilter's two int flags (2): `71a48358`, `f22014d2`

```
lpfilter_c(..., LocalVar->RestartWSE, LocalVar->restart ? 1 : 0,   // :348  reset
           &objInst->instLPF, 1, LocalVar->WE_Vw);                 // :349  has_InitialValue
```

Both literals are TRUTHS crossing a C boundary, not values — but that is a
claim about the CALLEE, so both configurations this campaign runs were read
rather than assumed (`evidence/WindSpeedEstimator/probe_flag_predicate.{py,txt}`):

```
kernel callee bridge (what the sweep and the harness link)
    IF (has_InitialValue /= 0) THEN
    ... LPFilter(..., (reset /= 0), ...)          x2
integrated translation (what the gate links)
    if (has_InitialValue) InitialValue_ = InitialValue;
    if ((iStatus == 0) || (reset != 0)) {
```

Zero tests against the VALUE `1` in either. **THE CONTROL:** the same detector
over a synthetic source that does compare both flags against 1 finds both — so
a detector that can only ever answer "none found" is not what produced the
zeros above.

## What is NOT declared, and why the list stops here

Sixty survivors remain. None of them is behaviour-preserving as far as this
dispatch could establish; they are corpus questions, and the two things known
about them are written in the census rather than guessed at:

* the whole `WE_Mode == 1` arm has **no gate oracle at all** — `WE_Gamma = 0.0`
  annihilates it, measured at 0 of 5,252,000 in this unit's own red test and
  twice more by unit #48 — so a survivor there has one instrument, not two;
* the EKF UPDATE arm, which is where most of the arithmetic sites live, is
  reached by **1,173 of 63,020** cases (`corpus_arm_census.txt`), so a survivor
  there is a survivor over 1,173 cases and not over 63,020.

Answering them is corpus work of the kind the six `ranges.toml` entries already
did once, and it is priced and handed off rather than guessed.
