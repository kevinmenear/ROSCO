# Unit #24 — `saturate` — evidence

`REAL(DbKi) FUNCTION saturate(inputValue, minValue, maxValue)`, one statement:

```fortran
saturate = REAL(MIN(MAX(inputValue,minValue), maxValue),DbKi)
```

Disposition: **`blocked`**, on P12 and nothing else. Every other layer ran, and
four of the five are alive. The block is that `cppmutate` generates **zero**
mutants for this translation, so the score is not low — it is *absent*, and
`done.py` fails that by name (`mutation_no_mutants`). See §4.

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

## 4. Mutation — `cppmutate` produced NOTHING, and that is the block

`mutation/saturate.json` is the tool's own artifact and it reads **0 mutants,
score 0.000**. It is committed unaltered.

The cause is structural, not a bug: this translation's body is a **call
expression**, and all nine of `cppmutate`'s operators need an arithmetic
operator, a comparison, a subscript or a numeric literal. A call has none, and
neither its argument order nor its callee name is a site. This is *not* unit
#22's parenthesised-operand gap — that one displaces mutants; this one produces
zero. Recorded in the tool's own `UNMODELLED` table as `call_expression`
(`translation-loop` `harness/cppmutate.py`), which is documentation and changes
no unit's mutant set.

**It is not closed here.** Adding a mutation operator changes the mutant set of
every unit already scored, and unlike a corpus addition — which can only kill
more — a new operator can produce a *survivor* in a unit that closed at 1.000.
That is X3 and SPEC §8.4, the Driver's call. And the artifact `done.py` reads was
**not** hand-written to make the unit close: `min_mutation_score` is 1.0 exactly
so the cheap way out is shut, and authoring both the mutants and the artifact
that grades them is the cheap way out.

What *was* done is unit #22's meanwhile — one unit's worth of the missing
measurement, by hand and committed (`hand_mutants.sh`, `hand_mutants.txt`),
against this unit's own 451-case corpus:

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

**11 of 11 behavioural killed; 2 equivalent.** The two equivalences are not
argued away — IEEE `maxNum`/`minNum` are commutative, including at a signed zero
and at a NaN, which `minmax_probe` measures directly, and both die on 0 of 451
as that predicts.

The sharp line is the last three: the difference between the shipped spelling and
the careless one is visible on **exactly 1 of 451 cases** — the negative-zero
case, which exists only because unit #14 added a signed-zero block after
discovering its own `dict.fromkeys` dedup absorbed `-0.0` into `0.0`. Without
that block, the defect this unit's whole §1 is about would survive silently.

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
| `hand_mutants.{sh,txt}` | §4, the measurement `cppmutate` cannot make |
| `kgen.*` | §6, both defects |
| `done_check.txt` | the done-condition as it stood at the state commit |
