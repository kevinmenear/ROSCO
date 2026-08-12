# Unit #24 — `saturate` — evidence

`REAL(DbKi) FUNCTION saturate(inputValue, minValue, maxValue)`, one statement:

```fortran
saturate = REAL(MIN(MAX(inputValue,minValue), maxValue),DbKi)
```

Disposition: **`integrated`**, on the second dispatch. The first closed
`blocked` on P12 and nothing else: `cppmutate` generated **zero** mutants for
this translation, so the score was not low — it was *absent*, which `done.py`
fails by name (`mutation_no_mutants`). The second dispatch added the operator
and paid its campaign-wide price in the same cycle. See §4.

---

## 1. The translation, and why it is `fmin`/`fmax` and not a branch

The `REAL(..., DbKi)` is the identity — its operand is already `REAL(DbKi)` — so
the whole unit is the expansion of two intrinsics, and the only decision it has
is how to spell them. That was **measured, not read**:

| file | what it is |
|---|---|
| `minmax_probe.f90` / `.cpp` / `minmax_probe_cases.txt` | gfortran's `MAX`/`MIN` against three candidate C++ spellings, on nine special inputs, compared through `TRANSFER`/`memcpy` because `+0.0 == -0.0` |
| `saturate_expr_sweep.f90` / `.cpp` / `.sh` / `.txt` | the *whole* reference statement against the same three spellings over **12,167 triples** |

```
shipped   std::fmin(std::fmax(v, lo), hi)                 0 of 12167 differ
branch A  t = (v > lo) ? v : lo; (t < hi) ? t : hi      789 of 12167 differ
branch B  t = (lo > v) ? lo : v; (hi < t) ? hi : t      561 of 12167 differ
```

Both branch spellings are wrong at a signed zero and at a NaN, in **opposite**
directions. gfortran's `MAX`/`MIN` are `fmax`/`fmin`, exactly.

This is **not** unit #14's rule inverted. There the Fortran wrote an explicit
`IF (CornerFreq < 0)` and `fmax` was the *mutant's* answer; here the Fortran
writes the intrinsic. Both times the rule is the same one — transcribe the shape
the reference has. The values were read at runtime so no constant folding can
substitute the front end's answer for the back end's.

## 2. Kernel replay — alive, and blind to one of the two clamps

Shipped kernel: **`Controllers.f90:102`**, scenario 1, 62 cases, **13,950 field
rows, all `IDENTICAL`** (`kernel.Controllers102.translation.{run.txt,verify_fields.csv}`).
VIT's own red test reported the kernel discriminating on `inputValue × 1.00001`.

Four stubs, run through the committed kernel by `run_stub.sh` (which forces the
rebuild — unit #18 — and hash-verifies the `.hpp` from inside the container —
unit #23):

| stub | passed | what it says |
|---|---|---|
| zero (reads no argument) | 21 of 62 | the comparison is **alive**; 41 rows move |
| passthrough (no saturation at all) | 40 of 62 | a clamp fires in **22 of 62** cases |
| lower clamp dropped (`MAX` deleted) | 40 of 62 | it is the **MAX** that fires |
| upper clamp dropped (`MIN` deleted) | **62 of 62**, 14,260 of 14,260 `IDENTICAL` | the **upper clamp is invisible to the kernel** |

**The call site was chosen on this measurement, not on hit count.** The first
site tried, `ControllerBlocks.f90:332` (`WE_Inp_Pitch = saturate(...)`, 407,976
hits across 23 scenarios, non-aliased), is *worse*: there the passthrough stub
passes **62 of 62** — `BlPitchCMeas` is interior to its bounds in every captured
case, so **neither** clamp is visible — while the zero stub still fails 39 of 62.
Both artifacts are kept (`kernel.ControllerBlocks332.*`), because "the kernel is
alive" and "the kernel can see what this unit is *for*" are two claims and only
the second decides a call site.

## 3. Differential harness — the layer that reaches the clamps

451 cases against the clean Fortran, **0 failed** (`harness/saturate.json`).

| red test | result |
|---|---|
| the unit as a no-op (returns 0.0) | **315 of 451** failed (`harness/saturate.redtest.json`) |
| no saturation at all (returns `inputValue`) | **202 of 451** failed (`harness.passthrough-stub.json`) |

202 of 451 against 22 of 62: the corpus reaches the clamp an order of magnitude
more often than the simulation does, and reaches the *upper* clamp, which the
kernel never does.

## 4. Mutation — 1.000, on an operator set that did not exist before this unit

`mutation/saturate.json`: **4 of 4 behavioural mutants killed, 2 declared
equivalent and proved, 0 failed to compile — score 1.000.** Produced by
`vit_mutate.py` against the CLEAN build (`reset_to_clean.sh` → score →
`restore_integrated.sh`; with the integrated build in place `Functions.f90.o`
holds a wrapper calling `saturate_c` and the harness baseline will not link).

```
drop_call      fmin(fmax(v,lo),hi) -> fmax(v,lo)   killed 151/451   the MIN deleted
drop_call      fmax(v,lo)          -> v            killed 100/451   the MAX deleted
swap_call_args fmax(v,lo), hi      -> hi, fmax(..)  SURVIVED        EQUIVALENT, proved
swap_call_args v, lo               -> lo, v         SURVIVED        EQUIVALENT, proved
swap_callee    fmin                -> fmax          killed 418/451
swap_callee    fmax                -> fmin          killed 339/451
```

**Two instruments, one figure, a dispatch apart.** The hand-run measurement the
first dispatch made — before any operator could reach this body — reports
`upper clamp dropped 151` and `lower clamp dropped 100` (§4a). The tool's two
`drop_call` mutants report the same 151 and 100. That is what says the recipe is
sound; a single number from a single instrument would not.

### The operator, and why it was refused until now

This translation's body is a **call expression**, and all nine of the original
operators need an arithmetic operator, a comparison, a subscript or a numeric
literal. A call has none, and neither its argument order nor its callee name was
a site. Three operators were added to `harness/cppmutate.py` (loop `b9fb5ee`),
from a balanced-paren **scanner** rather than a regex, because a call whose
argument is another call is not reachable by a pattern over `_OPERAND`.

Unit #22 had refused exactly this: *a new operator changes the mutant set of
every unit already scored, and unlike a corpus addition — which can only kill
more — it can produce a SURVIVOR in a unit that closed at 1.000.* That is two
questions, and only one is expensive.

| question | answer | how |
|---|---|---|
| does it invalidate existing artifacts? | **no** — `_mid` is content-derived, so an addition cannot move an existing id. **0 lost, 35 gained, 7 of 24 units affected** | old tool vs new over every scored translation, diffed on id sets |
| does it produce survivors in closed units? | **yes — 8, in 3 units.** GetPath **0.500**, GetRoot **0.333**, GetWords **0.667** | `call_operator_retake.{sh,txt}` |

Lost ids would force a re-take. **Zero lost is a debt, not a re-take**, and the
debt was paid in this cycle: the 6 other affected units were re-scored into
`mutation/<U>.call_operators.json`, leaving `mutation/<U>.json` untouched because
it is not wrong — it says what the nine operators found and it still does.
`vit_mutate.py --operator` stamps `operators_filter` so a restricted run cannot
be read as a full one.

All eight survivors are one shape — `std::min(len_src, len_dst)` /
`std::max(la, lb)`, a bounded string-copy length clamp, and its argument swap.
Those corpora never make the two lengths differ in the direction that matters.
This is a **newly visible** defect class, not a newly created one; the three
units' committed scores remain true about what they scored, and reopening them
is the Driver's call (DECISIONS.md, with all eight ids).

**`ReadAvrSWAP` is the one affected unit NOT measured.** Its generated
`readavrswap_test.cpp` predates the signature its translation now has, so the
BASELINE will not compile and `vit_mutate.py` refused before any mutant ran —
a stale generated artifact in another unit's untracked directory, not a property
of the two mutants this sweep would have scored.
`call_operator_retake.ReadAvrSWAP.txt` has the compiler's own words.

### Two restrictions, both forced by a live sweep

Neither was chosen up front and the first version looked right.

**Type-blindness.** `drop_call` replaces a call with its first argument;
`swap_call_args` exchanges two. Unrestricted, the campaign's string-handling
units came back **38%, 43%, 55%, 60%, 73%, 80%, 89%, 91% and 100%
unbuildable** — `std::strtod(c, &stop) -> c` is a `char*` where a `double` is
wanted. `vit_mutate.py` refused all of them, which is `NOCOMPILE_LIMIT` working.
**Raising that limit was the available wrong answer**: it is what catches a
genuinely broken build (a second definition in the link, every mutant failing,
1.000 measured on nothing). Restricting both operators to `_VALUE_PRESERVING` —
callees whose result type is their first argument's type and whose arguments
share one type — took 231 mutants across 13 units to 35 across 7, all compiling.

**Definition heads.** A `{` after the close paren does not identify one: a
constructor's member-initialiser list starts with `:`, and
`FileRecords(std::FILE* f)` and `OneRecord(std::string r)` both came through the
first sweep as call sites. A type-word test cannot see `std::string r` either.
The test that works needs no list of names — **a parameter is two bare
identifiers in a row**, and no C++ expression has that shape.

### The two equivalences are proved, not inherited

The two surviving `swap_call_args` mutants are the same two the first dispatch
hand-measured at 0 of 451 — and *0 of 451* is a statement about the corpus, not
about the mutant. The claim declared in `mutation/saturate.equivalences.json` is
the stronger one, and its proof is four rows of an artifact that already
existed. A two-argument selection function can fail to be commutative at exactly
two kinds of input: a tie with distinct bit patterns, which for `double` is only
`±0.0`, and a NaN operand. `minmax_probe.txt` runs **both orders of both** and
compares bits:

```
case 1  fmin(-0.0, +0.0) = 8000000000000000     fmax(-0.0, +0.0) = 0000000000000000
case 2  fmin(+0.0, -0.0) = 8000000000000000     fmax(+0.0, -0.0) = 0000000000000000
case 5  fmin( NaN,  1.0) = 3FF0000000000000     fmax( NaN,  1.0) = 3FF0000000000000
case 6  fmin( 1.0,  NaN) = 3FF0000000000000     fmax( 1.0,  NaN) = 3FF0000000000000
```

Commutative on every input the type admits, so these are equivalences and not
blind spots. The **undeclared** run is committed beside them
(`mutation/saturate.undeclared.json`, score 0.667) so that what survived is on
the record before anything excused it.

## 4a. The hand measurement (first dispatch), kept as the cross-check

Made when no operator could reach this body, against this unit's own 451-case
corpus (`hand_mutants.sh`, `hand_mutants.txt`):

```
BASELINE (unmutated)                                killed   0 of 451
fmin/fmax transposed                                killed 299 of 451
the two BOUNDS transposed                           killed 271 of 451
upper clamp dropped (the MIN deleted)               killed 151 of 451
lower clamp dropped (the MAX deleted)               killed 100 of 451
both clamps dropped (passthrough)                   killed 202 of 451
clamped against minValue twice                      killed 320 of 451
clamped against maxValue twice                      killed 267 of 451
std::min/std::max instead of fmin/fmax              killed   1 of 451
branch spelling A                                   killed   1 of 451
branch spelling B                                   killed   1 of 451
fmax arguments swapped        (EQUIVALENT)          killed   0 of 451
fmin arguments swapped        (EQUIVALENT)          killed   0 of 451
```

**11 of 11 behavioural killed; 2 equivalent.** It reaches seven defect classes
the operator set still does not, so it is kept rather than superseded — and it
supplies the 151 and 100 that the tool's `drop_call` mutants independently
reproduce.

The sharp line is the last three: the difference between the shipped spelling
and the careless one is visible on **exactly 1 of 451 cases** — the
negative-zero case, which exists only because unit #14 added a signed-zero block
after discovering its own `dict.fromkeys` dedup absorbed `-0.0` into `0.0`.
Without that block, the defect this unit's whole §1 is about would survive
silently.

**The first run of `hand_mutants.sh` was wrong and is worth the warning.** It
reported passthrough at 100 where `scripts/harness.sh` had independently measured
202, and adjacent mutants came back in equal pairs. The generated Makefile's
prerequisites are *correct* (`saturate_test.o: saturate_test.cpp saturate.hpp`),
so unit #18's missing-prerequisite defect is not the hazard — the hazard is the
**bind mount**: a file written on the Mac does not reliably stat() newer from
inside the container than the artifact built from it moments earlier, so `make`
keeps the stale header. The script now deletes the derived files and hash-checks
the input rather than trusting a timestamp.

## 5. Post-integration harness and the gate

| layer | result | red-tested |
|---|---|---|
| post-integration (wrapper only) | 451 checked, 0 failed | the two BOUNDS swapped **at the call** — the `BIND(C)` interface block carries the same token and a perturbation matching both is applied to neither (unit #13) — rebuilt between edit and run: **271 of 451**; revert, rebuild, green returns |
| gate, 27 scenarios | **5,252,000 values / 351 channels, 0 mismatched** | the saturation removed moves **2,255,249 of 5,252,000**, revert verified 0 |

271 is the same number the hand mutant "two BOUNDS transposed" produced against
the same corpus — two instruments, one figure.

2,255,249 matches **no other committed red test** and is the largest in this
campaign (next is GetWords/NonDecreasing at 1,857,893). `saturate` is called
10,203,422 times across 23 of 27 scenarios and clamps the pitch command, the
generator torque and every PI integrator; the gate is not remotely blind to it.

## 6. Two KGen defects, one fixed and one recorded

Found by pointing C3 at `Controllers.f90:1062`
(`PIController = saturate(PTerm + piP%ITerm(inst), ...)`) — the first call site
in this campaign whose **enclosing procedure is a FUNCTION**.

1. **FIXED (X2), KGen `kgen/parser/block_statements.py` + `base/block_statements.py`.**
   `gen_kernel_callsite_file.set_args` sets `node.tosubr = True` to convert the
   hoisted parent block into a SUBROUTINE, and `EndFunction.tokgen` reads
   `tosubr` — but `SubProgramStatement.tokgen` read **`tosurb`**, a transposed
   typo, in both copies of the file. So the kernel came out with a `FUNCTION`
   header, an `END SUBROUTINE`, and a driver that `CALL`s it: 29 compile errors
   (`kgen.parentblock-FUNCTION-emitted-as-FUNCTION-with-END-SUBROUTINE.f90`,
   `kgen.function-parentblock-does-not-compile.txt`).

   Control: for a SUBROUTINE parent block the emitted statement is **byte-identical
   either way** — `clsname` is `'SUBROUTINE'` on both branches, and the two
   `if not tosubr:` blocks read `typedecl` and `result`, neither of which a
   Subroutine statement has. The fix is inert outside the FUNCTION case, and the
   subroutine path was then exercised end to end twice (both kernels here) at
   62/62.

2. **NOT FIXED — recorded.** With the header corrected the kernel *still* does
   not compile: the parent block becomes `SUBROUTINE picontroller`, and the
   callsite statement `PIController = saturate(...)` is then an assignment to the
   subroutine's own name — *"Symbol 'picontroller' has already been host
   associated"* (`kgen.function-parentblock-tosubr-fixed-STILL-collides-with-own-name.txt`,
   `kgen.parentblock-FUNCTION-after-tosubr-fix.f90`). The function RESULT variable
   is neither declared as a local nor captured as state, so `!local output
   variables` is empty and the kernel would compare nothing even if it built.
   That is a design change to KGen's state analysis, not a typo.

   **It did not block this unit** and choosing around it is not X2: 8 of
   `saturate`'s 17 call sites are in SUBROUTINE scope, and C2 selects the call
   site anyway. It *will* block any unit whose only usable call sites sit inside
   a FUNCTION body.

## 7. Files

| file | what |
|---|---|
| `saturate.final.cpp` | the shipped translation |
| `minmax_probe.{f90,cpp}`, `minmax_probe_cases.txt`, `minmax_probe.txt` | §1, the nine special inputs |
| `saturate_expr_sweep.{f90,cpp,sh}`, `saturate_expr_sweep_triples.txt`, `saturate_expr_sweep.txt` | §1, the 12,167-triple sweep |
| `run_stub.sh` | the kernel stub runner (forced rebuild + hash guard) |
| `saturate.{zero,passthrough,no-upper-clamp,no-lower-clamp}-stub.cpp` | §2, the four stubs |
| `kernel.Controllers102.*` | §2, the shipped kernel and its four stub runs |
| `kernel.ControllerBlocks332.*` | §2, the rejected call site: passthrough **passes 62 of 62** |
| `harness.passthrough-stub.json` | §3, 202 of 451 |
| `hand_mutants.{sh,txt}` | §4a, the first dispatch's hand measurement — now the cross-check |
| `call_operator_retake.sh`, `call_operator_retake.txt` | §4, the 6 other affected units re-scored against the new operators |
| `call_operator_retake.ReadAvrSWAP.txt` | §4, the one affected unit that could NOT be scored, and why |
| `../../mutation/saturate.json` | §4, the score: 4 of 4, 1.000 |
| `../../mutation/saturate.undeclared.json` | §4, the same run with nothing declared: 4 of 6, 0.667 |
| `../../mutation/saturate.equivalences.json` | §4, the two declarations and their proofs |
| `../../mutation/*.call_operators.json` | §4, per-unit results of the re-take (`mutation/<U>.json` untouched) |
| `kgen.*` | §6, both defects |
| `done_check.txt` | the done-condition as it stood at the state commit |
