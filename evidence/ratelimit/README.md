# Unit #46 — `ratelimit` — evidence

```fortran
REAL(DbKi) FUNCTION ratelimit(inputSignal, minRate, maxRate, DT, reset, rlP, inst, ResetValue)
```

Disposition: **`integrated`**. Five layers, all five alive, all five red-tested,
and the mutation score is **1.000** on 17 behavioural mutants with **nothing
declared equivalent**.

It is the campaign's first unit with an **OPTIONAL dummy argument**, and the
first whose kernel is blind to a branch for a reason that is a property of the
*physics* rather than of the instrument.

---

## 1. The call site, chosen on whether the clamp fires

The unit's whole purpose is `rate = saturate(rate, minRate, maxRate)`. At the
three `PitchControl` sites the bounds are `PC_MinRat`/`PC_MaxRat`, which is
0.1745 rad/s in all 14 shipped `Examples/DISCON*.IN`. The last of those sites
writes `LocalVar%PitComAct(K)`, which reaches `avrSWAP(42..44)` unchanged
whenever `PF_Mode == 0` — so **`bld_pitch` in the committed baseline arrays IS
this unit's own output**, per invocation, and `|diff(bld_pitch)| == PC_MaxRat*dt`
is the clamp firing. Same instrument as unit #44's `sign(diff(nac_yaw)/dt)`:
read off an artifact the campaign already owns, not computed from a patch dict.

The census over all 27 committed baselines (`dt = 0.025`, so the clamp value is
`0.1745*0.025 = 0.004362501`):

```
scenario  8   4,611 upper + 4,557 lower of 15,999   <- BOTH clamps, ~57%
scenario 22   2,142 upper + 2,085 lower
scenario 27   1,065 upper + 1,131 lower
scenario  9      43 upper +    36 lower
scenario 20     121 upper +      0 lower            <- one-sided
scenarios 1,3,5,7,11,15,16,18,21,25   1 to 8 upper, ZERO lower
```

**The default scenario would have been nearly blind.** Scenario 1 runs the site
79,998 times and clamps on 4 of them, all in the same direction — a kernel built
there would pass a translation with no lower clamp at all, which is unit #24's
`ControllerBlocks.f90:332` failure one function up.

Window `1-20, 23990-24010, 47900-47920` = **62 cases**. The first range exists
because coverage places the reset arm at exactly one invocation per call site
per run: clean `Functions.f90:75` records **8 hits in every scenario**, which is
1 + 3 + 3 + 1 — the eight `ratelimit` calls of the first DISCON call and no
others.

## 2. Kernel replay — alive, and blind to exactly one thing

`kernel.translation.run.txt`: **62 of 62, 14,818 `IDENTICAL` field rows, 0 NOT
IDENTICAL** (`kernel.translation.verify_fields.csv`). VIT's own red test reported
the kernel discriminating on `inputSignal × 1.00001`.

Eight stubs through `run_stub.sh`, which forces the rebuild (unit #18) and
hash-verifies the `.hpp` from inside the container (unit #23):

| stub | passed | what it says |
|---|---|---|
| zero (reads no argument) | **0 of 62** | the comparison is **alive**; 239 rows move |
| passthrough (no rate limit at all) | 25 of 62 | the limiter moves **37 of 62** |
| lower clamp deleted | 44 of 62 | the lower clamp fires in **18 of 62** |
| upper clamp deleted | 43 of 62 | the upper clamp fires in **19 of 62** |
| `inst = inst + 1` deleted | **0 of 62** | `objinst%instrl` is compared, every case |
| ELSE-arm state write deleted | 3 of 62 | the 3 that pass **are** the reset cases |
| the reset arm unreachable | **62 of 62** | **INVISIBLE** |
| `ResetValue` ignored | **62 of 62** | **INVISIBLE** |

**The last two are one blind spot, and its cause is measured rather than
argued.** `ratelimit.census-probe.cpp` is the shipped translation plus one
`fprintf` to stderr; it changes no compared output, so the run it comes from is
green at 62 of 62 (`kernel.census-probe.run.txt`) and `kernel.census.txt` is a
*reading* of the corpus rather than a perturbation of it — unit #34's
`clamp_census` shape. Over the 62 verified calls:

```
RESET 3   UPPER-clamped 20   LOWER-clamped 19   interior 20
inst on entry (1-based): 6, 7, 8      has_ResetValue: 1, always

invocation 1  inst=6 reset=1 in=+0 last=+0 rv=+0 out=+0
invocation 2  inst=7 reset=1 in=+0 last=+0 rv=+0 out=+0
invocation 3  inst=8 reset=1 in=+0 last=+0 rv=+0 out=+0
```

On the reset invocations **every input is exactly zero**, so the two arms agree:

```
THEN:  ResetValue_ = rv = 0        -> returns 0, stores 0
ELSE:  raw = (0 - 0)/DT = 0        -> returns 0 + saturate(0)*DT = 0
```

That is a property of what a rate limiter is handed at *t* = 0, not of KGen and
not of the window. No re-aiming inside scenario 8 reaches it, because the arm
runs once and that once is at *t* = 0.

**Two instruments agree on the clamp.** The census counts 20 upper / 19 lower
over the same 62 verified calls that the two clamp-deleting stubs move 19 and
18. (The one-case gaps are cases whose clamped and unclamped answers differ by
less than the 1e-14 tolerance.)

## 3. Differential harness — the layer that reaches the reset arm

`harness/ratelimit.json`: **2904 checked, 0 failed, 0 inadmissible**, against the
CLEAN Fortran. Red test (`harness/ratelimit.redtest.json`): the unit as a no-op
fails **2904 of 2904** — the same case count the green certifies (unit #26), and
its mismatch list names both out-parameters plus the return: `rlP.LastSignal`
and `inst`.

The generator recognised `reset` **and** `has_ResetValue` as flags and ran each
of their values under the rest of the corpus, so `PRESENT(ResetValue) == .FALSE.`
— unreachable through the kernel — is ordinary here. `inst` needed no
`ranges.toml` entry: R5 constrained it to the `[1024]` extent on its own, as it
did for `PIController`. **This unit adds no entry to `harness/ranges.toml`**, and
that is a statement about the reference's domain: nothing here crashes or has no
answer on the generated inputs, including `DT = 0`, where the reference computes
`saturate(±inf) * 0 = 0` and returns `LastSignal` unchanged.

## 4. Mutation — 1.000, on 17, with nothing argued away

`mutation/ratelimit.json`: **17 of 17 behavioural killed, score 1.000, 0 declared
equivalent, 2 excluded as no-compile, 7 operators.** Both arms the kernel cannot
reach die here:

```
negate_cond  if (reset != 0)            2335 of 2904
negate_cond  if (has_ResetValue != 0)   1206 of 2904
arith_op     *inst - 1  ->  *inst + 1   2904 of 2904
arith_op     *inst + 1  ->  *inst - 1   2904 of 2904
drop_factor  rate * DT  ->  rate        1357 of 2904
index_offset [i] -> [i+1]  (x4)          678 to 1453
```

**The corpus is the one the green passed, and that is checked rather than
assumed:** `ratelimit_cases.bin` hashes `54f863ee9c90567739c54fa8239d90b6` both
before and after the `--no-generate` re-link that changes the LINK for the clean
tree.

### The two gaps the score does not show, both measured (`hand_mutants.{sh,txt}`)

```
BASELINE (the shipped translation, unmutated)     0 of 2904   the control
index expression swapped: 1 - *inst    (UB)      58 of 2904
rate numerator swapped: last - in               911 of 2904
the rate clamp dropped                          741 of 2904
clamp: value <-> minRate                          0 of 2904   EQUIVALENT
clamp: bounds transposed                        885 of 2904
```

**Gap 1 — the two `swap_operands` mutants did not compile.** cppmutate swaps the
operands of the *text* it matched, and in both cases the text stops one token
short of the expression (`*inst`, `rlP->LastSignal[i]`), so it emits `*1 - inst`
and `inputSignal[i]`. "Excluded" is a statement about the mutant's text, not
about the site. In the shape that compiles, one dies on 911 cases and the other
on 58 — and **58 is a number about an allocator, not about a corpus**:
`1 - *inst` subscripts `LastSignal` at −5..−7, so both the read and the write are
undefined behaviour and what the harness compares afterwards depends on what
lies before the array.

**Gap 2 — the `saturate_c` call has no generated mutant at all.** All three of
cppmutate's call operators are gated on tables of C standard-library callee
names; `saturate_c` is in neither. That is unit #33's finding restated one
function over, and the amendment it proposed is still a campaign-wide re-take
that X3 forbids mid-run. Hand-measured instead: the clamp dropped kills 741, the
bounds transposed 885.

**The one zero is an equivalence with an existing proof, not a blind spot.**
`saturate` is `fmin(fmax(v, lo), hi)`, so exchanging its first two arguments asks
whether `fmax` is commutative — and unit #24 proved it on this toolchain for the
only two inputs at which it could fail, a signed zero and a NaN, both orders of
both, compared as bits (`evidence/saturate/minmax_probe.txt`). Same two mutants
`saturate` itself declared, and `PIController` after it.

## 5. Post-integration — the only layer that reads the wrapper

`harness/ratelimit.postintegration.json`: **2904 checked, 0 failed**, same corpus,
against the INTEGRATED build.

Red test (`run_postint_redtest.sh`, rebuilt between the edit and the run):
`has_ResetValue_flag = 1` set to `0` inside the wrapper's own
`IF (PRESENT(ResetValue))` block, so the C++ side is told the OPTIONAL is absent
on every call — **604 of 2904**. Reverted, rebuilt, and the green re-taken at
0 of 2904 (`harness/ratelimit.postintegration.revert-verified.json`).

604 is the count of cases whose reset arm is reached with a `ResetValue` that
differs from `inputSignal`. It is the wrapper's half of the branch the kernel
cannot see at all.

## 6. Gate — 27 scenarios, and it sees the kernel's blind spot

`gate/ratelimit.json`: **5,252,000 values / 351 channels / 27 scenarios, 0
mismatched.** Two red tests, both revert-verified at 0 on the same build:

| perturbation | moved |
|---|---|
| the rate limit deleted (`saturate_c` replaced by its first argument) | **861,012** of 5,252,000 |
| the OPTIONAL ignored (`ResetValue_ = ResetValue` deleted) | **370,796** of 5,252,000 |

The second is the point. The kernel's `ignores-resetvalue` stub passes 62 of 62
on the identical perturbation. **The unit's one kernel blind spot is closed by a
different layer, on a number, rather than left open with an argument** — and the
two layers that close it (gate 370,796, post-integration 604) reach it for
different reasons: the gate because 26 other scenarios and 4 other call sites
have a non-zero `BlPitch` at *t* = 0, the harness because it draws the flag
freely.

## 7. What no bit-exact layer here can see

1. **`ControllerBlocks.f90:813`, the TRA call site**, is reached by scenario 9
   alone and by no kernel of this unit. It is the only site that passes **no**
   `ResetValue` — so `PRESENT(ResetValue) == .FALSE.` occurs in the shipped
   program at exactly two sites (`:319` and `:813`) and in the kernel at none.
   The differential harness draws the flag freely and is the instrument that
   covers it; the gate moves 370,796 values when the branch behind it is
   perturbed.
2. **`inst` outside 6..8.** The kernel captures three subscripts (better than
   `PIController`'s single 3, and it is why this site was chosen over
   `Controllers.f90:103`, where `inst` is always 2). The harness varies it over
   the whole `[1024]` extent and all four `index_offset` mutants die.
3. **A shape this unit does not have**, recorded because the campaign keeps
   meeting it: there is no `CHARACTER` anywhere in the signature, so R12/R13/R14
   are N/A and no truncation, capacity or word-planting question arises. The
   generator's own rule report says so rather than leaving four rules missing.

## 8. One tool defect, fixed rather than routed around (X2)

`scripts/harness.sh --no-generate` returned three steps early — before step 2e,
the step that decides per tree whether `<callee>_c` comes from the generated
bridge or from an integrated `<callee>.cpp.o`. Step 1 has just rewritten
`<stem>_callees.f90` with every bridge restored, so the early return left a
freshly generated bridge set against whatever LIBS is on disk:

```
ld: saturate.cpp.o: multiple definition of `saturate_c';
    ratelimit_callees.o: first defined here
vit_mutate.py: baseline is not green (nocompile); refusing to score
```

Loud rather than silent, and **only because `vit_mutate.py` refuses a non-green
baseline**: a sweep that had linked would have scored the C++ `saturate` against
itself. The repair 2e already contains is its `unlinked` branch; the fix is to
move the return below 2e.

---

## Files

| file | what it is |
|---|---|
| `ratelimit.final.cpp` | the shipped translation, as committed |
| `kernel-window.statefiles.lst` | the 62 captured invocation indices |
| `kernel-generated-Functions.f90` | the wrapper VIT substituted into the kernel |
| `kernel.translation.{run.txt,verify_fields.csv}` | the green, and its per-field log |
| `ratelimit.*-stub.cpp` / `kernel.*-stub.run.txt` | the eight stubs and their runs |
| `ratelimit.census-probe.cpp` / `kernel.census-probe.run.txt` / `kernel.census.txt` | the census, and the green it was taken under |
| `run_stub.sh` | the stub runner (rebuild forced, `.hpp` hash-verified in-container) |
| `run_harness_redtest.sh` / `ratelimit.noop-harness-stub.cpp` | the harness red test |
| `run_postint_redtest.sh` | the post-integration red test (rebuilds between edit and run) |
| `hand_mutants.{sh,txt}` | the six measurements cppmutate cannot make |
| `done_check.txt` | the done-condition verdict |
