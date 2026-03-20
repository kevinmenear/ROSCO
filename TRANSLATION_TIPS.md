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
