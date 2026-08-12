# evidence/sigma — unit #25

`REAL(DbKi) FUNCTION sigma(x, x0, x1, y0, y1, ErrVar)` — a cubic soft-start
ramp: `y0` below `x0`, `y1` above `x1`, and between them a Hermite cubic whose
four coefficients are built from `x0` and `x1` alone.

Disposition: **`integrated`**, first dispatch. Five layers, all five alive.
Everything below is a file in this directory or in `harness/`, `mutation/`,
`gate/`.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 27, clean `Controllers.f90:526` | **14,136 field rows, all `IDENTICAL`**, 62 of 62 | a constant stub reading no argument passes **0 of 62**; **BOTH CLAMPS PASS 62 of 62** |
| differential harness vs clean Fortran | **1,069** checked, 0 failed, 0 inadmissible | no-op **1069 of 1069**; lower clamp deleted **116**; upper clamp deleted **94**; the ramp deleted **812** |
| mutation score | **42 of 42 behavioural killed, 1.000**, 3 declared equivalent, 0 no-compile | the undeclared run is committed at **0.933** so the survivors are on the record before they were excused |
| post-integration harness (wrapper only) | **1,069** checked, 0 failed | the `--reverse-copy` line deleted from the wrapper fails **1,022 of 1,069** — the same 1,022 a counter in the translation independently reports |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | the cubic replaced by `y0` moves **229,165 of 5,252,000**, 18 channels, **3 scenarios**; revert verified 0 |

---

## 1. The kernel is alive and blind to both clamps, and the call site was chosen knowing it

Shipped kernel: **`Controllers.f90:526`**, `LocalVar%IPC_KP(i) = sigma(LocalVar%WE_Vw, CntrPar%IPC_Vramp(1), CntrPar%IPC_Vramp(2), 0.0_DbKi, CntrPar%IPC_KP(i), ErrVar)`,
inside `SUBROUTINE IPC` — 47,998 hits across 5 scenarios, the highest of this
unit's four call sites, and in SUBROUTINE scope, which unit #24 established is
the scope a KGen call site must be in.

Four stubs, run through the committed kernel by `run_stub.sh` (which forces the
rebuild — unit #18 — and hash-verifies the `.hpp` from inside the container —
unit #23):

| stub | passed | what it says |
|---|---|---|
| constant `-7.25`, reads no argument | **0 of 62**, 62 rows move | the comparison is **alive** |
| the `x < x0` clamp deleted | **62 of 62** | the lower clamp is **invisible** |
| the `x > x1` clamp deleted | **62 of 62** | the upper clamp is **invisible** |
| the cubic replaced by `y0` | 32 of 62, 30 rows move | the **ramp** is visible, in half the cases |

The constant is `-7.25` and not `0.0` **because `y0` is the literal `0.0_DbKi`
at this call site**: a zero stub agrees with every case that takes the lower
clamp, so it would have measured this unit's argument list rather than the
kernel. Unit #22's rule — the liveness stub must be determinate, finite and
*wrong* — with this call site's own arguments deciding what counts as wrong.

**Why both clamps are invisible is a fact about the captured inputs, not a
guess.** `kernel.translation.verify_fields.csv` carries `we_vw` for all 62
cases:

```
we_vw   n = 62   min 9.580446107340883   max 10.799770054153601
below x0 = 9.12: 0        above x1 = 11.4: 0
```

`IPC_Vramp` is `9.120  11.400` in **all 14** `Examples/DISCON*.IN`, so `x0` and
`x1` are constants at this site and every captured wind-speed estimate is
strictly inside them. All 62 cases take the cubic. And the 30-of-62 split in
the last stub is the other argument doing the same thing: the call site runs
`DO i = 1,2`, and where `IPC_KP(i)` is zero, `y1 - y0` is zero and the cubic
returns `y0` whatever it computes.

### The second call site was rejected on a count, not on a stub

Unit #24's rule is to run the branch-deleting stub at each candidate site before
spending the cycle on one. Here the rejection is settled one step earlier, by
the committed coverage data, and the reason is recorded rather than the stub
being skipped silently:

```
ControllerBlocks.f90:612  (Startup, PRC_R_Speed)   800 hits, scenario 9 only
kgen.invocation                        0:0:1-20, 0:0:12000-12020, 0:0:23900-23920
```

**Two of the three configured invocation ranges are past that site's last
call.** It executes 800 times in the whole campaign, so 42 of the window's 62
slots do not exist there and the capture would be the first 20 timesteps of the
load ramp — `x` a hair above `x0`, the least discriminating part of the domain
this unit has. Coverage says the same about the branches: line 502
(`sigma = y0`) has **no scenario-9 hits at all**, so that site cannot reach the
lower clamp either. It is a strictly smaller kernel, and the measurement that
says so cost one query.

## 2. The differential harness is the only instrument that reaches the clamps

1,069 cases against the clean Fortran, **0 failed, 0 inadmissible**
(`harness/sigma.json`).

| red test | failed | the same stub through the kernel |
|---|---|---|
| the whole unit as a no-op (`harness/sigma.redtest.json`) | **1069 of 1069** | — |
| the `x < x0` clamp deleted (`harness.no-lower-clamp-stub.json`) | **116 of 1069** | **0 of 62** |
| the `x > x1` clamp deleted (`harness.no-upper-clamp-stub.json`) | **94 of 1069** | **0 of 62** |
| the cubic replaced by `y0` (`harness.no-ramp-stub.json`) | **812 of 1069** | 30 of 62 |

Two of those rows are the point of having a second instrument. On the two
clamps the kernel's answer is not "small", it is **zero**, and the harness's is
116 and 94 — because the harness varies `x0` and `x1`, which no scenario in
this campaign does.

What reaches them is R6's two-sided-predicate block (unit #20): `x` against
`x0` and `x` against `x1` are both predicates between two VARIED quantities, so
20 cases set one side from the other at equality and at its two neighbouring
representable values. A ladder on `x` alone could not cross either, because
whichever base value is larger is larger in every case.

## 3. `**` with an INTEGER exponent, measured rather than read

`int_pow_probe.{f90,txt}` — the check registry states that gfortran expands an
INTEGER exponent by repeated multiplication and calls `pow` only for a REAL
exponent of 3.0 or more. This unit has `(x0-x1)**3` four times, `x0**2`, `x**3`
and `x**2`, so the rule is load-bearing here and it was run rather than quoted:

```
compared bit patterns:      60022     (TRANSFER to INTEGER(8), bits not values)
v**2 differing from v*v:        0
v**3 differing from v*v*v:      0
```

Both signed zeros are in the special-value list on purpose (unit #14): `(-0.0)**3`
is `8000000000000000` and so is `(-0.0)*(-0.0)*(-0.0)`, which is the one place
an odd power could have parted company with its expansion.

`(x0-x1)**3` is written **once** in the translation. The reference spells it in
all four coefficient statements and it is the same value in all four by
construction, so three of those spellings are restatements — sites a mutant can
move with nothing to compare against (units #1, #4, #21). What is *not*
collapsed is anything the reference varies: `x0**2`, `x**2` and `x**3` stay
where they are written.

## 4. Mutation — 1.000, and the three declared are all one idiom

`mutation/sigma.json`: 45 mutants, **42 of 42 behavioural killed**, 3 declared
equivalent, 0 failed to compile. The undeclared run is committed beside it at
**0.933** (`mutation/sigma.undeclared.json`).

**Not one survivor is in the arithmetic.** The whole cubic, both clamps, all
four coefficients and every literal in them die, including the two `negate_cond`
mutants on the clamps the *kernel* cannot see (`if (x < x0)` negated: 991 of
1069; `if (x > x1)` negated: 914 of 1069). The three declared are all the
`CHARACTER(:), ALLOCATABLE` staging-buffer idiom, which this campaign has now
reached in five units, and **they are not one claim**:

| id | claim | proof |
|---|---|---|
| `532e4d37` `n > 0` → `n >= 0` | **EQUIVALENT** | `equivalence_probe.{cpp,txt}` sweeps all **4,294,967,296** values of a 32-bit `int`; 0 at which the view length differs |
| `2524b715` `: 0` → `: 1` | **UNREACHABLE over this corpus** | `errmsg_extremes_probe`: `n_ErrMsg` on entry is 1..8, `calls with n_ErrMsg <= 0: 0` in 1,069 |
| `7ad82e7d` `> cap` → `>= cap` | **UNREACHABLE over this corpus** | same probe: cap 4097..4104, largest message 14, `s.size() == cap: 0` in 1,022 assignments |

The distinction is unit #23's and it is kept sharp here: the first holds on
every input the C type admits; the other two hold only on the inputs this
generator draws, and they are recorded as blind spots. **One of them is weaker
here than in the four earlier units** and the declaration says so: their
messages are literals of fixed length, so the capacity guard was unreachable by
construction; this unit's message is `'sigma:' // TRIM(ErrMsg)`, whose length
grows with its input, so the guard is unreachable only because no case supplies
an `ErrMsg` within six characters of a 4 KiB buffer.

## 5. The wrapper needed `--reverse-copy`, and the red test says what it bought

`sigma` writes exactly one field of its `INTENT(INOUT)` view-type dummy —
`ErrVar%ErrMsg`, in the `RoutineName` prefix branch. Unit #23's check, run
before integrating rather than after:

```
grep -n 'INTENT(INOUT)' Functions.f90        -> TYPE(ErrorVariables) :: ErrVar
grep -n 'ErrVar%.* *=' <the unit's body>     -> ErrVar%ErrMsg = RoutineName//':'//TRIM(...)
```

so `vit integrate ... --apply --reverse-copy`, and the wrapper carries
`CALL vit_copy_scalars_to_errorvariables`. Deleting that one line, rebuilding
between the edit and the run, fails **1,022 of 1,069**
(`harness/sigma.postintegration.redtest.json`).

**1,022 is the number a second instrument reports independently.**
`errmsg_extremes_probe.txt` counts `assign_errmsg calls  1022` over the same
corpus — the probe was written to answer a mutation question and it happens to
count exactly the cases the wrapper defect can reach. Two instruments, one
number.

Unlike interp1d, this unit's error path appears not to be reachable in the
shipped program at all — and the evidence for that is an absence, so it is
stated as one. In `coverage/line_coverage.json` the guard `IF (ErrVar%aviFAIL <
0)` (clean `Functions.f90:510`) records **257,580 hits across 6 scenarios** and
the assignment on 511 records **nothing**. `scripts/coverage.py` stores only
non-zero counts (`if n:`), and it drops gcov's non-instrumented `-` lines as
well as its never-ran `#####` ones, so an absent line is one or the other; 511
is an ordinary assignment inside a subroutine gcov instrumented, which leaves
only "never ran". That is unit #21's shape — a guard with hundreds of thousands
of hits and its body with none — one inference short of its measurement, and
the inference is written down rather than folded into the claim.

So the gate is very unlikely to have caught this wrapper defect, and the
post-integration harness is again the layer that did the work.

## 6. The gate sees the ramp and cannot see either clamp

`gate/sigma.json`: 5,252,000 values across 351 channels and 27 scenarios, 0
mismatched. `gate/sigma.redtest.json`: the cubic replaced by `y0` moves
**229,165** values across **18 channels in 3 scenarios — 8, 9 and 27** — and
the revert returns 0 of 5,252,000 on the same build. The figure matches no
other committed red test in this campaign.

Three scenarios, not the six that execute the unit. Scenarios 2, 6 and 18 call
`sigma` and their compared channels do not move, which is the same observation
the kernel's 30-of-62 makes from the other side: where the IPC gains are zero
the ramp has nothing to scale.

**And the gate cannot reach either clamp, for a reason that is a property of the
inputs rather than of the instrument.** `IPC_Vramp` is `9.120  11.400` in all 14
`Examples/DISCON*.IN`; the wind-speed estimate crosses those bounds during a
run, so the clamps do execute — but a *defect* confined to a clamp is only
visible where the clamped answer differs from the cubic's, and the two red tests
above measure that the corpus finds 116 and 94 such cases where the kernel finds
none. The gate was not asked separately; the harness is the layer that owns
those two branches and it is red-tested on both.

---

## Files

| file | what it is |
|---|---|
| `sigma.final.cpp` | the shipped translation, as committed |
| `int_pow_probe.{f90,txt}` | gfortran's INTEGER `**` against repeated multiplication, 60,022 bit patterns |
| `equivalence_probe.{cpp,txt}` | `532e4d37` proved over all 2³² ints |
| `errmsg_extremes_probe.{cpp,txt}` | the shipped translation plus counters; the corpus counts behind `2524b715` and `7ad82e7d` |
| `run_stub.sh` | the stub runner: hash-verify, force the rebuild, keep the stdout |
| `sigma.const-stub.cpp` + `kernel.const-stub.run.txt` | kernel liveness |
| `sigma.no-lower-clamp-stub.cpp` + `kernel.…`, `harness.…json` | the lower clamp, through both instruments |
| `sigma.no-upper-clamp-stub.cpp` + `kernel.…`, `harness.…json` | the upper clamp, through both instruments |
| `sigma.no-ramp-stub.cpp` + `kernel.…`, `harness.…json` | the cubic, through both instruments |
| `sigma.noop-stub.cpp` | the harness's whole-unit red test |
| `kernel.translation.run.txt`, `kernel.translation.verify_fields.csv` | the shipped kernel run and its field log |
| `kernel-window.statefiles.lst` | the 62 captured invocations, so the window is on the record |
| `done_check.txt` | the done-condition, run after both commits |
