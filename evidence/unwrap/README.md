# evidence/unwrap — unit #26

```fortran
FUNCTION unwrap(x, ErrVar) result(y)
    REAL(DbKi), DIMENSION(:), Intent(IN)  :: x
    REAL(DbKi), DIMENSION(SIZE(x))        :: y
```

Phase-unwrapping: copy `x`, then walk the array and shift the whole tail by
`±2·PI` until every adjacent difference lies in `(-PI, PI)`.

Disposition: **`integrated`**, first dispatch. **THE GATE IS BLIND TO THIS UNIT
BY MEASUREMENT** — both call sites have zero hits in all 27 scenarios — so the
differential harness, the mutation score and the post-integration harness are
the whole of the evidence, which is unit #1's (`AddToList`) and unit #21's
(`UpdateZeroMQ`) shape.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT AVAILABLE** — no scenario reaches either call site; there is no state to capture | n/a |
| differential harness vs clean Fortran | **403** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **363 of 403**; the `+2·PI` loop deleted **4 of 403**; the `-2·PI` loop deleted **361 of 403** |
| mutation score | see `mutation/unwrap.json` | the undeclared run is committed at **0.925**, and the run against the corpus as it stood BEFORE this unit widened it at **0.875**, so both sets of survivors are on the record before any was excused |
| post-integration harness (wrapper only) | see `harness/unwrap.postintegration.json` | see `harness/unwrap.postintegration.redtest.json` |
| gate, 27 scenarios | see `gate/unwrap.json` | see `gate/unwrap.redtest.json` and the same-build control |

---

## 1. The unit is dead, and the two call sites are dead for two different reasons

`coverage_deadness.py` / `.txt`, read out of the committed
`coverage/line_coverage.json` (generated from the clean baseline `54dd134`, so
every line number below is a line of that revision):

```
Controllers.f90:322  IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_GenTq > 0))   407,976 hits
Controllers.f90:329  IF (CntrPar%OL_Mode == 2)                                        0 hits
Controllers.f90:339  LocalVar%AzBuffer = UNWRAP(LocalVar%AzBuffer, ErrVar)            0 hits

ReadSetParameters.f90:773  IF (CntrPar%OL_Mode == 2)              3 hits  (10, 14, 24)
ReadSetParameters.f90:778  CALL Read_OL_Input(...)                3 hits  (10, 14, 24)
ReadSetParameters.f90:780  RETURN                                 3 hits  (10, 14, 24)
ReadSetParameters.f90:807  CntrPar%OL_Azimuth = Unwrap(...)       0 hits

Functions.f90 518..557 (the whole of unwrap): NO line with any hit, 0 hits total
```

The first site is unit #21's shape: a guard that is never true. `OL_Mode` is `0`
in 11 of the 14 `Examples/DISCON*.IN` and `Ind_GenTq` is `0` in the rest, so the
407,976 evaluations at line 322 never enter the block.

**The second is not, and that is the part worth keeping.** Scenarios 10, 14 and
24 *do* configure `OL_Mode > 0` — scenario 10 sets `OL_Mode = 2` and
`Ind_Azimuth = 6` in `vit_sim.py`, which is exactly the configuration this unit
exists for. They reach line 778 and then `Read_OL_Input` fails, because
`Examples/example_inputs/OL_Mode2_Input.dat` is not in this tree (unit #17
measured the same absence from the other side), and the `RETURN` at 780 takes
them out two statements before the call. Reading the guard alone would say *no
scenario configures it*; reading the `RETURN` says *three scenarios configure it
and a missing file stops them*. Those are different facts and only the second
one is true.

## 2. The reference does not terminate on part of its own domain

`reference_termination_probe.f90` / `reference.does-not-terminate.txt`. The two
`DO WHILE` loops make progress only by `y(i:) = y(i:) ± 2·PI`, so once the
tail's magnitude is large enough that the addition rounds back to the same
double, the guard's value never changes:

```
smallest 2**k with v + 2*PI == v :  7.2057594037927936e16   (2**56)
y(2) = 1e3     TERMINATED after 159 iterations
y(2) = 1e17    still looping after 17,900,000,000 iterations, y(2) unmoved
y(2) = 1e300   still looping after 17,900,000,000 iterations, y(2) unmoved
```

The sixth upstream ROSCO defect this campaign has recorded and the third of the
"the reference has no answer" family (#17 non-termination, #21 indeterminate,
#23 abort). **The translation does not terminate there either, deliberately** —
a translation that broke out of the loop would be a different function on inputs
the reference simply never answers for.

**No `harness/ranges.toml` entry was written**, and that is a decision rather
than an omission. The corpus cannot reach it: `_bounds` defaults an array to
±1e3 and `_real_magnitude_ladder` is gated on `not q.dims`, so no hot rung can
land in `x`. A pin NARROWS the admissible domain, which is the blindness the
generator exists to remove; there is nothing here to narrow.

## 3. `PI` is not π, and the difference is 466 ULP

`pi_literal_probe.{f90,cpp,txt}`. `Constants.f90:24` declares
`REAL(DbKi), PARAMETER :: PI = 3.14159265359` — twelve digits of a decimal
literal, not the double nearest π:

```
                     gfortran            g++
PI      bits         400921FB54442EEA    400921FB54442EEA
2*PI    bits         401921FB54442EEA    401921FB54442EEA
-PI     bits         C00921FB54442EEA    C00921FB54442EEA
M_PI    bits                             400921FB54442D18   <- NOT this constant
PI - M_PI                                2.069456E-13   (466 ULP)
```

`M_PI` would have been a translation that computes a different function on every
input whose unwrapping crosses a threshold. Measured on both compilers rather
than read.

## 4. The differential harness, and the three branches it took three tries to see

403 cases against the clean Fortran, **0 failed, 0 inadmissible**
(`harness/unwrap.json`). Every stub below is the shipped translation with one
thing deleted, run by `run_harness_stub.sh` (which hash-verifies the `.cpp` from
inside the container — unit #23):

| red test | failed |
|---|---|
| the whole unit as a no-op, both loops deleted (`harness.noop-stub.json`) | **363 of 403** |
| the `y(i)-y(i-1) .LE. -PI` guard and its `+2·PI` shift deleted (`harness.no-plus-loop-stub.json`) | **4 of 403** |
| the `y(i)-y(i-1) .GE. PI` guard and its `-2·PI` shift deleted (`harness.no-minus-loop-stub.json`) | **361 of 403** |

**The `+2·PI` branch scored ZERO until the corpus was widened twice, and the
two widenings are two different rules.** `harness_stubs.txt` keeps the sequence:

```
corpus            cases   no-op fails   +2*PI-deleted fails
as inherited        355          355                    0
+ order ladder      366          355                    0
+ range-spanning    377          363                    4
+ constant steps    403          363                    4      <- and the mutants die
```

* **The order predicate is written against a name the corpus cannot set.**
  `order_arrays_from` looks for the same NAME subscripted twice in one
  statement; `unwrap` writes `y = x` and then subscripts `y`. The detector
  returned the empty set, so the ladder never fired at all. Closed by one hop
  through a whole-array copy — `LHS = RHS`, both bare names, RHS a parameter.
* **An ordering at one step size varies the SIGN of a difference and never its
  MAGNITUDE.** The ladder's bodies are `[1.0, 2.0, …]`, so the reversed run
  gives a difference of `-1`, which crosses neither `-PI` nor `+PI`. Running the
  same shapes at steps spanning ±1e3 is what makes the branch reached.
* **And reaching a branch is not reaching its BOUNDARY.** `.LE.` against `.LT.`
  differ on exactly one input, a difference of exactly `-PI`, and the third
  scale — the shapes run again with each constant the reference itself names as
  the adjacent difference — is what supplies it. `-PI` is not a literal anywhere
  in `Functions.f90`; it is a named `PARAMETER` in `Constants.f90`, which is why
  `literals_from` cannot see it and `named_constants_from` can.

## 5. Three instruments disagreed about `DIMENSION(SIZE(x))`

The fourth disagreement between VIT's generators in this campaign (units #8,
#17, #22) and the second on a FUNCTION RESULT. All three carry the FUNCTION's
own dimension text across, and the text is legal in exactly the scope the
reference wrote it in:

| generator | on `DIMENSION(SIZE(x))` |
|---|---|
| `vit interface` (the SHIPPING wrapper) | correct — the wrapper's `x` is the original assumed-shape `x(:)` |
| `test_validate.generate_fortran_bridge` | `REAL(8), DIMENSION(SIZE(x))` over an assumed-size `x(*)`: *"The upper bound in the last dimension must appear in the reference to the assumed size array 'x'"* |
| `harness.vitbridge.map_signature` | REFUSED the result, then `EmitError: C parameter 'unwrap_result' is not in the mapped signature` |

Fixed in both (X2, VIT `_bridge_result_dims` and the loop's `_result_extents`
`size_of`), each resolving `SIZE(<arg>)` to the extent parameter
`build_c_params` already emits rather than evaluating anything. `SIZE(x) + 1`
still falls to the same refusal, deliberately.

One level down, `n_x` sizes **two** arrays — `x` and the result — and it was
written into the case stream twice and declared twice:
`error: redeclaration of 'int32_t n_x_a'`. That one needs no X3 argument: a
duplicate declaration is a compile error, so no already-scored unit can have
been carrying one.

## 6. What the instrument changes cost, measured

* `x3_cost_size_intrinsic.{sh,txt}` — of the 25 units with a committed harness
  artifact, **1** has an array-valued FUNCTION RESULT (`identity`, `n, n`), and
  its dimensions are bare names, which take the path they always did. The new
  branch is reached only where the old code returned a refusal, so it can turn
  a refusal into a mapping and cannot turn a mapping into anything else.
* `x3_cost_order_alias.{sh,txt}` — old `order_arrays_from` against new, over
  every scored unit, **read from the clean baseline source and not the working
  tree**. Four detectors fire (`ColemanTransform`, `NonDecreasing`, `interp1d`,
  `unwrap`) and exactly **one** moves: `unwrap`, `[] → ['x']`. Reading the
  working tree instead would have reported `old=[] new=[]` for 21 of the 25 —
  a wrapper contains no subscript at all — and looked like a proof.
* **The three corpus additions DO grow the corpus of the other three units whose
  order detector fires**, if those units are re-run. That is a corpus addition,
  which can only add cases and only kill more; their committed artifacts stay
  true about what they measured.

## 7. Three survivors, all the `CHARACTER(:), ALLOCATABLE` idiom, none in the arithmetic

`mutation/unwrap.equivalences.json` carries the reasons and keeps the two kinds
apart. `bf2ce388` is EQUIVALENT and proved over all **4,294,967,296** values of
a 32-bit int (`equivalence_probe.{cpp,txt}`); `af5a7c94` and `10e6dfb3` are
UNREACHABLE OVER THIS CORPUS, which is a blind spot and is recorded as one
(`errmsg_extremes_probe.{cpp,txt}`, the shipped translation plus counters):

```
calls to unwrap                        403
ErrVar->n_ErrMsg on entry, min .. max  1 .. 9
calls with n_ErrMsg <= 0               0     <- af5a7c94 needs > 0
ErrVar->n_ErrMsg_cap, min .. max       4097 .. 4105
calls to assign_errmsg                 370
largest message offered                16
assignments with s.size() == cap       0     <- 10e6dfb3 needs > 0
```

The 370 is a cross-check rather than a lone number: the two `negate_cond`
mutants in `assign_errmsg` die on **370 of 403** cases, which is the same count
the probe reaches by a different route.

Like sigma's `7ad82e7d` and unlike the four units before it, the capacity
guard's unreachability here is the WEAKER kind and is written as the weaker one:
`'unwrap:' // TRIM(ErrMsg)` grows with its input, so the guard is unreachable
only because the corpus supplies no `ErrMsg` within seven characters of a 4 KiB
buffer — a claim about the corpus, not about the program.

## Files

| file | what it is |
|---|---|
| `unwrap.final.cpp` | the shipped translation, as integrated |
| `coverage_deadness.{py,txt}` | §1 — both call sites and the whole body, from committed coverage |
| `reference_termination_probe.f90`, `reference.does-not-terminate.txt` | §2 |
| `pi_literal_probe.{f90,cpp,txt}` | §3 |
| `unwrap.noop-stub.cpp`, `unwrap.no-plus-loop-stub.cpp`, `unwrap.no-minus-loop-stub.cpp` | §4 inputs |
| `run_harness_stub.sh`, `harness_stubs.txt` | §4 runner and transcript |
| `harness.{noop,no-plus-loop,no-minus-loop}-stub.json` | §4 artifacts |
| `x3_cost_size_intrinsic.{sh,txt}`, `x3_cost_order_alias.{sh,txt}` | §6 |
| `equivalence_probe.{cpp,txt}` | §7, `bf2ce388` |
| `errmsg_extremes_probe.cpp`, `run_errmsg_probe.sh`, `errmsg_extremes_probe.txt` | §7, `af5a7c94` and `10e6dfb3` |
| `vit_translate.stdout.txt` | the scaffold prompt, as generated (C4) |
