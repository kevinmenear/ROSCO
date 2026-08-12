# identity — evidence

Unit #22. `rosco/controller/src/Functions.f90:329` on the clean tree
(`54dd134`), one call site, `ControllerBlocks.f90:454`. Disposition
**integrated**; the gate sees it as well as it has seen anything in this
campaign, and the kernel's own verdict line cannot fail on it.

```fortran
FUNCTION identity(n) RESULT(A)
    INTEGER, INTENT(IN)         :: n
    REAL(DbKi), DIMENSION(n, n) :: A
```

An automatic array whose extents ARE the argument. That is what makes the unit
interesting: the plan's `bridge_feasible` was `unknown` on five features
including `c_fn_array`, and the answer turned out to be different for VIT's two
generators — one had shipped it since the campaign began, the other could not
compile it.

## The layers, and what each could see

| layer | result | red-tested |
|---|---|---|
| kernel replay | 62 cases, 225 fields, **13,950 of 13,950 IDENTICAL** | **the VERDICT cannot go red** — a no-op stub scores `✓ 62/62 passed`; a determinate WRONG-constant stub scores **0/62** and is what says the comparison is alive |
| differential harness vs clean Fortran | **29 checked, 0 failed** | the unit as a no-op fails **28 of 29** (the survivor is `n = 0`, where neither side writes) |
| mutation score | **20 of 20 behavioural killed, 1.000**, 0 declared equivalent, 0 no-compile | 9 of the 20 are CRASHES, so the killed-by-comparison count is **11 of 20** |
| post-integration harness (the wrapper) | **29 checked, 0 failed** | `CALL identity_c(n, A)` → `identity_c(n - 1, A)`, rebuilt between the edit and the run: **28 of 29**; revert, rebuild, green returns |
| gate, 27 scenarios | 5,252,000 values / 351 channels, **0 mismatched** | the diagonal written `2.0` moves **1,462,798 of 5,252,000** across **115 channels and 22 scenarios**, `revert_verified: true` |

## The kernel passes a stub that writes nothing, and the reason is NaN

`vit verify` reports `✓ VERIFICATION PASSED: 62/62 passed` for
`identity.noop-stub.cpp`, which reads no argument and writes no element of the
result. That is not a window problem and not a tolerance-versus-magnitude
problem. It is the comparison's own arithmetic:

```
IF (ALL(var == kgenref_var)) -> IDENTICAL
ELSE rmsdiff = SQRT(SUM((var-ref)**2)/n)
     IF (rmsdiff > kgen_tolerance) -> OUT_TOL  ELSE -> IN_TOL
```

The no-op leaves the automatic array uninitialised, `LocalVar%WE%P` comes out
NaN, and **`NaN > 1.D-14` is FALSE**, so the field falls to `IN_TOL` and
`IN_TOL` counts as PASSED. The field log says so in the one place the verdict
does not:

```
identity.0.0.1,p,array,IN_TOL,,,n_diff=  8 rms=  NaN
```

`kernel.noop-stub.verify_fields.csv` and `kernel.noop-stub.verify.txt` are that
wrong artifact, committed under C12 before anything was done about it. **Nothing
was done about it here** — writing a NaN guard into KGen's generated comparison
changes the pass/fail basis of every kernel this campaign has run (X3, SPEC
§8.4). Recorded in DECISIONS.md as a candidate for the Driver.

**The magnitude is not the explanation, and that was measured rather than
assumed.** `identity.wrong-constant-stub.cpp` writes a determinate, finite 3x3
identity with `2.0` on the diagonal:

```
kernel.wrong-constant-stub.verify_fields.csv   0 of 62 passed, p OUT_TOL, rms 0.37807928263323742
```

`p` is of order 1. So the comparison can see this unit perfectly well, and what
the no-op demonstrates is a hole in the SCORING, not a blind window. Unit #19's
rule — build the second stub, the one that perturbs only the thing you are
asking about — is what separated the two readings.

## What the kernel cannot constrain: the argument

`identity.constant-stub.cpp` reads no argument and writes a hardcoded 3x3
identity. It scores **13,950 of 13,950 IDENTICAL** — indistinguishable from the
translation on the kernel's entire domain, because the reference's ONE call site
is the literal `identity(3)` and every captured case has `n == 3`. No invocation
window can widen that; `n` is not data. So the kernel's green is a claim about
one value of one input, and the differential harness is the only layer that
varies `n` at all.

## What no layer here constrains, stated rather than left to be found

* **The stride has no mutant.** `harness/cppmutate.py`'s `_OPERAND` is
  `identifier | number`, so a PARENTHESISED operand matches nothing and
  `(j - 1) * n` produces no `arith_op`, no `drop_factor` and no `swap_operands`
  mutant. The parenthesis is not incidental — `exponent-grouping`, one of VIT's
  own checks, requires exactly this spelling. Run by hand instead
  (`stride_probes.sh`, `stride_probes.txt`): `* n` written `* 3` fails 2 of 29
  and `* n` written `/ n` fails 2 of 29, so the corpus WOULD kill them; the
  score does not include them because they were never generated.

* **Column-major versus row-major is UNFALSIFIABLE on this unit.** Transposing
  the index expression moves **0 of 29** cases at every `n`, and no corpus can
  change that: the identity matrix is symmetric, so `A(i,j)` and `A(j,i)` are
  the same value in every element. VIT's `array-section-row`/column-major rule
  is enforced here and cannot be tested here.

* **`n` is varied over {0, 1, 2, 3, 4} and nothing else.** Three values come
  from a predicate knob, `3` from R5's first shape and `4` from its second. A
  defect that first appears at a large `n` is not covered. The domain is bounded
  in the reference too — `A` is an automatic array of `n*n` doubles on the
  stack — but 5 values is a long way inside that bound.

* **The gate is the only post-integration layer that runs the unit in the
  controller.** The post-integration harness measures the WRAPPER's marshalling,
  not the arithmetic: after integration the Fortran body IS the translation.

## The two generators disagreed, and one of them was a defect

`vit interface` has crossed this signature since the campaign began:

```fortran
SUBROUTINE identity_c(n, identity_result) BIND(C, NAME='identity_c')
    INTEGER(C_INT), VALUE :: n
    REAL(C_DOUBLE), INTENT(OUT) :: identity_result(*)
```

`test_validate.generate_fortran_bridge` — the DIFFERENTIAL HARNESS's Fortran
side — declared the result a SCALAR and emitted `vit_result = identity(n)`:
*Incompatible ranks 0 and 2 in assignment*. Fixed in VIT rather than worked
around (X2), `ab75fa0`. Third disagreement between these two generators in this
campaign (units #8, #17, #22) and the second on a FUNCTION RESULT.

The loop's own signature mapper had the matching gap and it was worse than a
build failure: `build_c_params` emits `double* identity_result` with no extent,
`arg_by_name` has no entry for a result, and the parameter fell through to the
scalar branch — so the harness **varied the unit's only output as an input** on
the ±1e3 default and compared eight bytes of a buffer the reference writes n*n
of. Two further defects fell out of fixing it, both recorded in the loop commit
(`20b0dbb`): a predicate knob on an extent desynchronised the case stream, and
R5 emitted ONE SHAPE for any unit with ONE free extent while reporting "1 varied
extent(s) at [3]". The second is load-bearing here — `stride_probes.txt` shows
the hardcoded-stride defect dying on exactly two cases and one of them is the
shape that fix adds.

## Files

| file | what it is |
|---|---|
| `identity.noop-stub.cpp` | writes nothing. The INPUT to the NaN measurement |
| `identity.constant-stub.cpp` | hardcoded 3x3 identity, reads no argument |
| `identity.wrong-constant-stub.cpp` | determinate and finite, `2.0` on the diagonal |
| `kernel.translation.run.txt` | the shipping translation's kernel stdout, taken at the time (`kernel/` does not survive the cycle) |
| `kernel.translation.verify_fields.csv` | 13,950 rows, all IDENTICAL |
| `kernel.noop-stub.verify{.txt,_fields.csv}` | `62/62 passed` with `p` at `rms=NaN` |
| `kernel.constant-stub.verify{.txt,_fields.csv}` | 13,950 of 13,950 IDENTICAL |
| `kernel.wrong-constant-stub.verify{.txt,_fields.csv}` | 0 of 62, `p` OUT_TOL at rms 0.378 |
| `stride_probes.sh` / `stride_probes.txt` | the three defects `cppmutate` cannot generate |
