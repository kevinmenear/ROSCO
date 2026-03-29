# Translation Tips

Lessons learned from Fortran→C++ translation failures. Each tip was discovered
when a translation failed VIT verification and the fix revealed something
generalizable.

`vit translate` includes the contents of this file in the translation prompt
automatically, so new tips improve all future translations.

---

## Fortran `**` operator precedence

Fortran's `**` (exponentiation) has the **highest** arithmetic precedence and
groups each operand independently before multiplication. A naive C++ translation
that uses `*` for squaring can produce different floating-point results:

**Fortran:** `omega**2*DT**2` → `(omega*omega) * (DT*DT)`
**Wrong C++:** `omega * omega * DT * DT` → `((omega*omega) * DT) * DT`

These are mathematically equivalent but differ in intermediate values. With
`-ffp-contract=off`, the different evaluation order can produce different
bit-level results.

**Fix:** Always parenthesize to match Fortran's `**` grouping:
```cpp
double omega2_DT2 = (omega * omega) * (DT * DT);
```

*Discovered in Phase 7A: ResController's Tustin coefficient computation.*

---

## C struct field names use TYPE definition case

C/C++ is case-sensitive. VIT generates C struct field names preserving the exact
case from the Fortran TYPE definition in ROSCO_Types.f90. Fortran code that
*uses* these fields may use different capitalization (Fortran is case-insensitive),
but **the C struct always matches the TYPE definition**.

When accessing struct fields in C++, use the case from the TYPE definition, not
from the Fortran usage code:

**TYPE definition:** `REAL(DbKi) :: IPC_AxisTilt_1P`
**Fortran usage:** `LocalVar%IPC_axisTilt_1P` (different case — valid in Fortran)
**C++ must use:** `LocalVar->IPC_AxisTilt_1P` (matches TYPE definition)

If in doubt, check the generated struct in the scaffold header — the field names
there are authoritative.

*Discovered in Phase 7B: IPC and PitchControl field name mismatches.*

---

## Use Fortran's constant values, not C/C++ standard library values

Fortran projects define their own mathematical constants (PI, D2R, R2D, etc.)
in parameter modules. These may have fewer digits than C's `M_PI`. Use the
**exact value from the Fortran source**, not the C standard library.

**ROSCO Constants.f90:** `PI = 3.14159265359`
**Wrong C++:** `M_PI` (= 3.14159265358979323846, differs by 2.07e-13)
**Correct C++:** `const double PI = 3.14159265359;`

The difference is small but propagates through filter coefficients and trig
computations, producing IN_TOL differences instead of IDENTICAL.

*Discovered investigating CableControl and ActiveWakeControl IN_TOL fields.*

---

## Fortran `x**3.0` (real exponent) calls `pow()`, not `x*x*x`

gfortran compiles `x**N.0` (REAL exponent) differently depending on N:

- **Integer exponent** (any N): always optimized to repeated multiply → use `x*x*...*x`
- **Real exponent 2.0**: special-cased by gfortran, optimized to `x*x` → use `x*x`
- **Real exponent ≥ 3.0**: NOT optimized — gfortran emits a call to libm `pow()` → use `std::pow(x, N.0)`

The results of `pow(x, 3.0)` and `x*x*x` differ by 1 ULP for some input values.
In closed-loop control with feedback, this 1 ULP accumulates over thousands of
timesteps and eventually causes visible divergence.

**Fortran:** `v_m**3.0`
**Wrong C++:** `v_m * v_m * v_m`
**Correct C++:** `std::pow(v_m, 3.0)`

**General rule:** if the Fortran exponent is a REAL literal (has a decimal point)
and the value is 3.0 or higher, use `std::pow()`. If the exponent is an INTEGER
literal (no decimal point), use repeated multiplication.

*Discovered in Phase 8C: WindSpeedEstimator Q(2,2) matrix element.*

---

## Fortran comments can be wrong — the code is authoritative

When translating Fortran to C++, always defer to the actual code (IF conditions,
array indexing, WRITE formats, loop bounds) — not the comments. Comments may
describe intended behavior that was never implemented, or describe an earlier
version of the logic that was updated without changing the comment.

This applies to all aspects of translation: error checks, dimension ordering,
variable names in comments, and "TODO" annotations that may have been resolved
in code but not in comments.

*Discovered in Phase 8C: Fortran interp2d had correct dimension checks
(`SIZE(zData,2)`) but misleading comments that said "zData(:,1)". Reading the
comments instead of the IF conditions led to an incorrect diagnosis.*
