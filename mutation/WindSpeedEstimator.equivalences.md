# Declared equivalences — unit #65 `WindSpeedEstimator`

Thirteen of the seventy-three survivors. An equivalence is a claim about the
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
