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
