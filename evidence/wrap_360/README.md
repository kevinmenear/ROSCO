# Unit #28 — `wrap_360` — evidence

`REAL(DbKi) FUNCTION wrap_360(x)`, three statements, `rosco/controller/src/Functions.f90`:

```fortran
IF (x .lt. 0.0) THEN
    wrap_360 = x + 360.0
ELSEIF (x .ge. 360.0) THEN
    wrap_360 = x - 360.0
ELSE
    wrap_360 = x
ENDIF
```

Disposition: **`integrated`**. Five layers available, five run, all green, all
red-tested.

**This unit is unit #27's sibling and its counterexample.** `wrap_180` sits one
screen up in the same file with the same shape and the OPPOSITE comparison pair,
and its two arms are dead at all six of its call sites — so its kernel and its
gate watch it run 675,987 times and never once see it wrap. Here **one arm is
live and one is dead**, and every layer says which:

| layer | result | red-tested |
|---|---|---|
| kernel replay | **41 of 41**, 41 of 41 field rows `IDENTICAL` | VIT's own (`x × 1.00001`), plus four stubs — §2 |
| differential harness | **134** checked, 0 failed, 0 inadmissible | no-op **127**; both arms deleted **51**; low arm alone **36**; high arm alone **15**; the sibling's comparison spelling **7** |
| mutation | **11 of 11 killed, score 1.000**, 0 declared equivalent, 0 uncompilable | — (this *is* the red test, eleven times) |
| post-integration harness | **134** checked, 0 failed, against the shipping library | the wrapper hands `-x`: **129 of 134**, revert verified green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | whole unit → `0.0` moves **84,477**; both arms deleted **31,579**; high arm alone **31,579**; low arm alone **0** |

---

## 1. C1/C2 — the two call sites, and why the hot one is also the right one

Two call sites, both in `Controllers.f90`, both inside a `SUBROUTINE` (so unit
#24's KGen `FUNCTION`-scope defect cannot bite):

```
Controllers.f90:515   Y_MErr     = wrap_360(LocalVar%NacHeading + LocalVar%NacVane)      3,999 hits, scenario 2
Controllers.f90:845   StrAzimuth = wrap_360(360*LocalVar%Time*CntrPar%AWC_freq(1))*D2R  15,999 hits, scenario 22
```

`coverage/line_coverage.json`, generated from the clean baseline `54dd134`, over
all 27 scenarios:

```
Functions.f90:466  FUNCTION wrap_360(x)         19,998   scenarios 2, 22
Functions.f90:471  wrap_360 = x + 360.0    <- LOW    0   nowhere
Functions.f90:473  wrap_360 = x - 360.0    <- HIGH  15,199   scenario 22 only
Functions.f90:475  wrap_360 = x            <- ELSE   4,799   scenarios 2, 22
                                           ELSE + HIGH + LOW = 19,998 = the two sites
```

Site 845 is chosen on the arm the unit exists for and not on the hit count —
unit #24's rule. It happens to be the hotter site too, but that is not the
argument: site 515's 3,999 calls are **entirely** the pass-through arm (its
scenario contributes nothing to line 473), so a kernel captured there would be
the wrap_180 situation over again.

**The invocation window's third range was predicted empty before extracting**
(unit #25's arithmetic, which costs one query): the configured window is
`0:0:1-20, 0:0:12000-12020, 0:0:23900-23920` and the site has 15,999 calls, so
`23900-23920` contributes 0 and the capture is 20 + 21 = **41 cases**. It came
back 41, with exactly those indices.

## 2. C6 — the kernel, and what a stub can and cannot move in it

`vit verify` → **41 of 41 passed**, and the claim quoted is the field log's
`status` column and not the verdict line (unit #16): **41 of 41 `IDENTICAL`**,
no `IN_TOL`, no `NaN`. The compared field is the caller's local `strazimuth`.

```
evidence/wrap_360/kernel.Controllers845.translation.verify_fields.csv
```

Four stubs through the same kernel (`run_stub.sh`, which hash-verifies the
`.hpp` from inside the container and deletes the object before rebuilding —
units #23 and #18):

| stub | IDENTICAL | NOT |
|---|---|---|
| a determinate wrong constant `-7.25` | **0** | 41 |
| the campaign's default zero stub | 1 | 40 |
| **both arms deleted** (`return x`) | **20** | **21** |
| the **high** arm alone deleted | 20 | 21 |
| the **low** arm alone deleted | **41** | **0** |

`-7.25` rather than `0.0` is unit #25's rule applied: the call site passes
`360*Time*AWC_freq(1)`, which is `>= 0` at every invocation, and the first
captured case has `x = 0` exactly — so the campaign's default liveness stub
scores a free pass on it and measures the argument list rather than the kernel.

**`captured_domain.py`** turns three of those runs into a statement about the
input domain, which the field log does not carry:

```
passthrough  (both arms deleted)   20 IDENTICAL   21 NOT IDENTICAL
no-low-branch (x<0 arm deleted)    41 IDENTICAL    0 NOT IDENTICAL
no-high-branch(x>=360 deleted)     20 IDENTICAL   21 NOT IDENTICAL
distinct |difference| over the 21 wrapped cases : [6.2831853036]
360 * D2R, D2R = 0.01745329251 from Constants.f90:23   : 6.2831853036
```

A case where the pass-through stub is IDENTICAL is a case the reference did not
wrap, *by definition rather than by inference*; the no-low run then excludes the
low arm by measurement, which is what makes the other 21 unambiguously the high
arm. Each of those 21 differs from the unwrapped answer by exactly the value the
deleted statement subtracts.

Two things that version of the script had to learn:

* An earlier version recovered `x` from a **model** of the call site
  (`x = 0.45·(n−1)` degrees, read off cases 1 and 2). The model puts every case
  in the right arm and does **not** reproduce the captured values bit for bit —
  6 of 41 — because `LocalVar%Time` accumulates by `DT` rather than being
  multiplied out. A measurement was available for the price of a build.
* It compared the per-case difference against `2·π` and reported **DOES NOT
  MATCH on a correct run**. The caller multiplies by ROSCO's own
  `D2R = 0.01745329251` (`Constants.f90:23`), eleven digits, so `360·D2R` is
  `6.2831853036` where `2·π` is `6.2831853072` — a disagreement in the tenth
  digit, above the printed precision of the kernel's own difference line. **P7
  reaches the probe as well as the translation.**

Recovered domain: the 20 pass-through cases span `x ∈ [0.0000, 8.5500]` degrees
and the 21 wrapped cases span `x ∈ [5399.5501, 5408.5501]`. Note what the second
range means — the results are ≈5040 degrees, **not** in `[0, 360)`. The
reference wraps once and leaves the rest, which is exactly why `std::fmod` is a
different function and not a tidier spelling.

## 3. C6 — the differential harness, and the sibling-shaped defect

134 cases against the clean Fortran, 0 failed, 0 inadmissible. R6 fired its
predicate knob at `x ∈ [-1, 0, 1, 2, 359, 360, 361]`, so both thresholds are in
the case stream as values, and its signed-zero rung put `-0.0` there as well.

Five red tests, **all taken at the same 134 cases as the green** — one
generating run, so unit #26's corpus-skew census cannot apply:

| perturbation | failed |
|---|---|
| the whole unit as a no-op | 127 of 134 |
| **both** wrapping arms deleted | **51** |
| the `x < 0` arm alone | 36 |
| the `x >= 360` arm alone | 15 |
| **the two comparisons spelled as `wrap_180` spells them** (`<=` low, `>` high) | **7** |

`36 + 15 = 51` is the internal check: the arms are disjoint, so deleting them
singly must partition what deleting both moves.

The last row is this unit's own hazard. `wrap_180` and `wrap_360` are one screen
apart, three statements each, and their comparison pairs are opposites:
`.le. / .gt.` there and `.lt. / .ge.` here. Reading the sibling across moves
exactly `x = 0.0`, `x = -0.0` and `x = 360.0` — and nothing else in the whole
real line.

## 4. C6 — the mutation score, and where its margin comes from

**11 mutants over 5 operators, 11 killed, 0 survived, 0 declared equivalent, 0
failed to compile.** Score **1.000**, scored against the clean build before
integration. Every kill is behavioural, so none of the 11 is the build catching
what the corpus could not (unit #22's caveat). No `.equivalences.json` and no
`.undeclared.json` exist and the absence is **structural**: the survivor set is
empty, so an undeclared run would be byte-identical and there is nothing to
excuse.

Three of the eleven are killed by a handful of cases, and `boundary_margin.py`
reads the generated case file rather than trusting the count:

```
'<' -> '<='                            differs on 5, the cases holding x in {0.0, -0.0}
'>=' -> '>'                            differs on 2, the cases holding x == 360.0
'360.0' -> '361.0' (the comparison)    differs on 3, the cases in [360.0, 361.0)
multiplicity in the stream: {'0.0': 4, '360.0': 2, '-0.0': 1}
```

Not a copy of `wrap_180`'s probe, because **the low guard's boundary is a zero
and there are two of those**: `-0.0 < 0.0` is false and `-0.0 <= 0.0` is true,
so that mutant differs at both. The margin is two corpus rules — R6's predicate
knob and the signed-zero rung unit #14 added and unit #24 was the first to
need. Remove either and the corresponding mutant survives at any corpus size.

## 5. C7–C9 — integration, and the gate that can see one arm of two

`vit integrate --apply`, **no `--reverse-copy`**: unit #23's grep test returns 0
`INTENT(INOUT)` dummies for this unit, so there is no view type on the signature
and no scalar field for a wrapper to drop. The generated wrapper is two lines.
Links clean — `T wrap_360_c`, `T _Z8wrap_360d`, `T __functions_MOD_wrap_360`, no
undefined `wrap_360` symbol.

The gate was taken **after** the integrate, so unit #23's re-take rule is
satisfied by construction rather than by inspection.

```
gate/wrap_360.json                                     5,252,000 / 0 mismatched
gate/wrap_360.redtest.json          whole unit -> 0.0     84,477 moved, revert 0
gate.both-branches-deleted.json     both arms deleted     31,579 moved, revert 0
gate.high-arm-deleted.json          high arm alone        31,579 moved, revert 0
gate.low-arm-deleted-MOVES-NOTHING.json  low arm alone          0 moved
```

All four perturbations are anchored to `double wrap_360(double x) {`, asserted
to occur exactly once in the perturbed file before cutting (unit #26 — a
generated wrapper line occurs in three units at once and a `str.replace`
measures none of them).

**`31,579` is the whole difference from unit #27.** wrap_180's identical
perturbation moves 0 of 5,252,000. Here the gate carries a five-figure number
for the branch, because 15,199 of 19,998 calls take it.

`0 + 31,579 = 31,579` is the same partition the other two instruments give, and
the three of them disagree about the low arm in a way that is the finding rather
than an inconsistency:

| instrument | low arm alone | high arm alone | both |
|---|---|---|---|
| kernel, 41 cases | 0 | 21 | 21 |
| differential harness, 134 cases | **36** | 15 | 51 |
| gate, 5,252,000 values | 0 | 31,579 | 31,579 |

The low arm is dead in all 27 scenarios at both call sites, so the two bit-exact
layers move nothing when it is deleted — and both of those zeros have their
control on the **same build**, which is what makes them blindness rather than a
broken chain.

## 6. E4.5 — the post-integration harness

134 checked, 0 failed, against the integrated build. The only layer that can see
a defect in the generated bridge. Red-tested by perturbing the argument handoff
— the wrapper hands `-x` to `wrap_360_c` — rebuilt between the edit and the run
**both ways**, **129 of 134 failed**; reverted, rebuilt, re-run green.

The sign is chosen rather than inherited: negating `x` maps the live high arm
onto the dead low arm, so it is a defect this unit's corpus catches 129 times
and the two bit-exact layers, which never see the low arm, would report
differently.

## 7. What none of the five layers can see

* **The low arm on real data.** `x < 0` is reached 0 times in 27 scenarios at
  both call sites. Only the generated corpus tests it (36 of 134 cases), so the
  evidence that `x + 360.0` is right is a generated-input claim and not a
  simulation claim. Unlike unit #27's dead arms, this is not caused by a defect
  in the harness — `360*Time*AWC_freq(1)` is non-negative by construction, and
  `NacHeading + NacVane` simply never goes negative in scenario 2.
* **Arbitrarily large angles.** No rung of the generator puts `x` beyond ±1e3
  except the real-magnitude ladder's extremes, so the wrap is tested near its
  thresholds and at `DBL_MAX`, and not at the multiply-wrapped values a long run
  would produce. The kernel is the layer that does reach them — its 21 wrapped
  cases sit at ≈5400 degrees, where the reference deliberately does not
  normalise — but it reaches them only in one narrow window of one scenario.
* **The comment.** `! Function modifies input angle, x, such that 0<=x<=360`
  is false about its own code in two ways: the result is in `[0, 360)`, and a
  single subtraction does not normalise 5400. Nothing in this campaign compares
  a unit against its documentation, and nothing here should be read as having
  done so. The code is the oracle (P7).

## Files

```
README.md                        this file
captured_domain.{py,txt}         input domain and per-arm counts, from three stub runs
boundary_margin.{py,txt}         what kills the three thin mutants, from the case file
run_stub.sh                      one stub through the kernel, hash-verified
run_harness_stub.sh              one stub through the differential harness
run_wrapper_redtest.sh           perturb the wrapper, prove red, revert, prove green
wrap_360.final.cpp               the shipped translation, as committed
wrap_360.zero-stub.cpp           reads no argument, returns 0.0
wrap_360.wrong-constant-stub.cpp reads no argument, returns -7.25
wrap_360.passthrough-stub.cpp    both arms deleted
wrap_360.no-low-branch-stub.cpp  the x < 0 arm deleted
wrap_360.no-high-branch-stub.cpp the x >= 360 arm deleted
wrap_360.sibling-comparisons-stub.cpp   wrap_180's comparison spelling
kernel.Controllers845.*.run.txt  the five kernel runs
harness.*-stub.json              the four non-canonical harness red tests
harness.postintegration.revert-verified.json
gate.both-branches-deleted.json
gate.high-arm-deleted.json
gate.low-arm-deleted-MOVES-NOTHING.json
vit_translate.stdout.txt         the scaffold prompt, as generated
done_check.txt                   the done-condition at close
```
