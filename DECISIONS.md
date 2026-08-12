# DECISIONS — rosco-r2

Append-only record of *why*. Never read end to end.

## 2026-08-11 — Unit #12 `NonDecreasing`: the corpus had one ordering in it

**Every array this generator has ever produced was sorted ascending, and the
unit whose whole body is an order predicate is what made that visible.**
`harness/generate.py`'s `_fill_array` returns `lo + span*(k+1) + jitter` — a
strictly increasing ramp, deliberately, because the shape it was written for is
a table of interpolation breakpoints and an unsorted one puts the reference
outside its own admissible domain. The consequence nobody had measured: all 25
generated cases for `NonDecreasing` were ascending, the reference answered
`.TRUE.` in every one, and a translation that reads no argument and returns
`.TRUE.` passes that corpus.

That is the SAME green the kernel gives, for the same reason. `NonDecreasing =
.FALSE.` has zero hits in all 27 scenarios, so a constant-`.TRUE.` stub reading
no argument scores 200 of 200 `IDENTICAL`. **Two independent instruments, one
blindness** — and the differential harness exists precisely to not share the
simulation's blind spots.

Fixed by addition, and the addition is narrow on purpose. An ORDER LADDER —
reversed, one adjacent inversion at first / interior / last, one adjacent EQUAL
pair at interior and last, the constant array, and the degenerate lengths 1 and
2 — appended last, fired only for an array the REFERENCE ITSELF subscripts twice
in one statement (`Array(I_DIFF + 1) - Array(I_DIFF)`). Read out of the Fortran,
not the C++, for unit #11's reason: a red-test stub contains no subscript, so a
corpus keyed off the translation collapses on exactly the run that proves the
harness can fail. Not "any array": `interp1d` compares `xData(I)` against a
scalar, one subscript per statement, and would not fire.

**The measurement that says it bought something**, rather than the claim: both
`compare_op` mutants (`<= 0.0` → `< 0.0`) are killed by 4 of 36 cases, and
`evidence/NonDecreasing/order_ladder_kills_the_le_mutant.txt` enumerates those 4
— all of them order-ladder cases, none of them among the 25 base draws. Without
the repair the score is 14/16 = 0.875, below the campaign's threshold. The same
4 cases are the only ones the post-integration `SIZE(Array) - 1` red test can
reach, so the ladder is also what made the marshalling red test possible.

**A green that is one-sided does not announce itself.** 25 checked, 0 failed
read exactly like 36 checked, 0 failed. What exposed it was decoding the case
file and counting distinct answers — three lines, and it belongs before
believing any harness green on a unit whose output is a predicate.

## 2026-08-11 — Unit #12: two generators that had never met a numeric array's extent

Both recorded with their wrong artifact before either was fixed (C12), under
`evidence/NonDecreasing/loop_defects/`.

1. **`harness/emit.py` emitted a use before its declaration.** A numeric array's
   extent is an ordinary C parameter and `vit interface` puts it AFTER the
   buffer (`nondecreasing_c(Array, n_Array)`), while the generated `main`
   declares in C-parameter order — so `std::vector<double> Array_a(n_Array_a);`
   came out a line before `n_Array_a` existed. The CHARACTER path had solved
   this: it reads the length WITH the buffer and passes it at its own position
   through a `predeclared` placeholder. The numeric path never got the
   mechanism. **A fix applied at one of two sites that share a code shape is a
   fix the other site escapes** — the campaign's fifth instance.

   The fix reuses the existing placeholder machinery and is gated on
   `names.index(d) > i`, which is *precisely* the condition under which the old
   emitter produced the compile error. So no unit that ever produced a compiling
   harness can enter the new branch, and no already-measured corpus moves. A
   second defect went with it: at its own position the extent was typed from
   `elem_ctype`, which defaults to `double`, so a `double` was being read from an
   int slot and narrowed back at the call.

2. **`scripts/_integration_shim.py` emitted `int32_t` with no `<cstdint>`.** The
   shim compiles standalone and includes nothing the translation includes. It
   already has two conditional includes for exactly this reason
   (`ISO_Fortran_binding.h`, `vit_types.h`); this is the third, keyed the same
   way — on the types actually emitted, so no unit whose shim already compiles
   gains a line. A LOGICAL RESULT is the first fixed-width return this campaign
   has produced.

## 2026-08-11 — Unit #12: the gate's red is a refusal to start, not a computation

`gate/NonDecreasing.redtest.json` moved **1,857,893 of 5,252,000**, which is
byte-identical to `gate/GetWords.redtest.json`. Read as a second gate-visible
unit that would be an overstatement. All three live call sites are
`.NOT. NonDecreasing(...)` → `ErrVar%aviFAIL = -1`, so answering `.FALSE.` makes
the controller reject its own input file — the same end state as breaking
GetWords' word parser, and a rejected input file has one output signature.

Two things follow and both are kept. The red test IS its own same-build control
(it reproduces the campaign's designated control figure exactly, which is what
that control exists to check). And what the gate constrains is a single boolean:
nothing about the array, and nothing about the answer `.FALSE.`, which no
scenario ever produces.

## 2026-08-10 — Phase 1: the gate got an artifact, and extraction got a verdict

**The gate counts values, not channels.** `regress.sh` compared whole channels
with `np.array_equal` and reported 351. The same comparison at element level is
5,252,000. `loop/done.py` P9 only asserts `compared > 0`, so a channel-counted
artifact would have passed while claiming four orders of magnitude less than it
appeared to — and, worse, would not have been evidence about the instrument
E3.2 observed failing, because the red test in RUNBOOK.md is recorded in values
(198,892 of 624,000). One criterion's artifact describing a different instrument
than the next criterion's evidence is not a bookkeeping problem; it means
neither number constrains the other.

**`scripts/gate.py` exits non-zero when it compares nothing.** Not part of E3.1
as written. E3.1 asks for the count to be printed; SPEC §7 explains that the
count is what catches the vacuous case, and a gate that compares zero values and
exits 0 is the failure this project exists to remove. Recorded here as a local
decision so nobody later reads it as the criterion.

**The gate restores the input files it dirties.** Found by running it: two
`Examples/DISCON*.IN` files came back modified (`IPC_ControlMode` 1→2,
`F_NumNotchFilts` 0→1), because `vit_sim.py`'s `write_discon()` rewrites them
per scenario in place. The loop verifies with `require_clean_tree=True`, so this
would have blocked every unit from closing — and the obvious fix a session would
reach for, `git add -A`, would have committed a silently reconfigured gate for
every unit that followed. An instrument that edits its own subject is not
measuring it. `gate.py` snapshots and restores, names what it restored in
`inputs_restored`, and reports anything still dirty in `residual_dirt` rather
than trusting its own list to be complete.

**`vit.yaml` is derived from bf25e35, not from the first replication's HEAD.**
At HEAD that file carries 54 `status: integrated` entries naming the .cpp files
a translator is meant to produce, two extra `types` entries, a `shared_files`
list and a widened kgen `invocation` — all learned *during* that run. Copying it
would have pre-declared 54 of this campaign's 69 units done and handed over an
answer key. bf25e35 is the first replication's *setup* commit, which knew none of
it, and is the revision `campaigns/rosco-r2.toml` already pins as `src_rev`. A
semantic diff of the two YAMLs shows exactly four keys differing — the mechanical
`/workspace/ROSCO-replication` → `/workspace/ROSCO-r2` path substitution — and
nothing else. Declared `derive`, and `red_tested = false` until the derivation
has actually been observed failing.

The judgement this replaced is worth recording: asked whether `types`,
`allocatable` and `shared_files` were setup or answer key, the honest answer was
not available by reasoning about them. It was available by reading the pinned
revision, which shows `types` with three entries and `allocatable` present at
setup, and `shared_files` absent. Deriving from the revision the manifest
already names beat deciding on their behalf.

**Extraction is recorded as NOT WORKING, against its own success message.**
`vit extract` prints `✓ Extraction successful` and then `WARNING: No state data
captured.` Three runs, zero files matching the campaign's state pattern, while
the first replication has `kernel/AddToList.0.0.1` for the same function. The
instrumented library, the run command and the invocation window were each ruled
out by measurement rather than argument (see RUNBOOK.md). The cause is not yet
known. Recording it as TODO-with-evidence rather than as a working step is the
whole point: a green that was never observed going red is what this campaign is
built to refuse, and here the green is coming from our own front-end.

Consequence for planning: kernel replay is not currently an available
verification route, so units must close on the generated differential harness.
Six of them are `respecify`, for which P13 makes a mutation score mandatory
anyway.

## 2026-08-10 — bridge_feasible was a reading of a parse failure

`AddToList` was unit #1, `contract: mirror`, `bridge_feasible: yes`. Its whole
contract is `allocate(clist(isize+1))` → copy → `move_alloc(clist, list)`: it
changes the extent and the data pointer of the caller's allocatable array. The
bridge VIT generates for it passes `list(*)` as a bare pointer and `n_list` BY
VALUE, and the wrapper's dummy drops `allocatable` altogether. The function's
only effect has no representation in its own signature.

The plan already said so, two entries away. `ParseInAry_Opt` and
`ParseDbAry_Opt` were `bridge_feasible: no` with the basis `c_alloc_inout does
not cross (INTENT(INOUT) ALLOCATABLE needs an ALLOCA)` — the identical feature.

**The mechanism was not the parameter name.** `harness/contract.py`'s `_DECL`
matched `^(CHARACTER|INTEGER|...)\b[^:]*::`, and `[^:]*` forbids a colon
anywhere in the attribute list — so every declaration carrying `DIMENSION(:)`
or `dimension(:,:)` failed to match and the argument silently got no entry in
`arg_decls`. Measured on this tree: 15 of 418 dummy arguments lost, in 12 of 83
procedures. `AddToList`'s `list` was one of them, so `bridge_blockers` iterated
a one-argument list, found no ALLOCATABLE, and returned (). The evidence of `no`
had been parsed away and the absence was rendered as a positive.

All 67 `yes` verdicts carried the same basis string — "no signature feature is
recorded as failing to cross" — which states in words that nothing was
recorded. `Tri` requires a basis on a settled value precisely so a yes cannot be
told from a default; a constant met that requirement syntactically and defeated
it semantically. A constant is not a provenance.

Fixed in the loop repo: the regex now admits colons inside the attribute list;
`_feasibility` returns UNKNOWN if any argument's declaration did not parse,
rather than a verdict about the arguments that happened to be readable; and a
`yes` basis now names the arguments it read. Both fixes are red-tested.

Re-derived, unchanged sources, same entry point and exclusions:

    bridge_feasible   67 yes / 2 no   ->   64 yes / 5 no
    changed:  AddToList     yes -> no   (list: c_alloc_inout)
              Read_OL_Input yes -> no   (channels: c_alloc_inout)
              interp2d      yes -> no   (zdata: c_assumed_shape_2d)

`interp2d` fails on a different cell — a rank-2 assumed-shape dummy — a blocker
class the old plan could not see at all. Nothing else in the plan moved: unit
set, order, contracts, absorption, exclusions and notes are byte-identical, so
this is a contained correction and not a re-planning.

**Consequence not yet addressed.** `loop/driver.py:82`'s `next_unit` selects on
`disposition` alone and ignores `bridge_feasible`, so the Driver would still
dispatch `AddToList` first — now against a plan that states its signature cannot
cross. Whether an infeasible unit should be skipped, escalated, or given a
disposition up front is a policy question, recorded here rather than decided in
passing.

## 2026-08-10 — what "the smoke test passed" means, written before launching

Named in advance, because afterwards any outcome can be narrated into either
column. The smoke test is `--only ColemanTransform --max-units 1`, and what it
tests is the WIRING, not the translation.

  PASS   ColemanTransform closes all 13 predicates.

  PASS   It fails at P11 or P12 because the harness or the mutation score could
         not produce evidence, HAVING REACHED that point through
         translate -> verify -> integrate -> gate. The path working is the
         result. Record it as a finding about the evidence path, not as a
         failure of the loop.

  FAIL   Anything earlier: dispatch, Sentinel, budget accounting, prompt
         render, done-condition evaluation, or the state commit.

**`--mutation-glob` will not be unset to make the unit close.** `done.py:344`
returns `self._mutation(...)` unconditionally when the glob is set — the
red-test fallback exists only when it is UNSET — and `min_mutation_score` is
1.0 and is never overridden by `run_campaign.py`. So mutation is mandatory for
all 69 units at a 100% kill rate, mirror and respecify alike. Dropping the flag
would convert a hard requirement into a red test and make the first unit's
evidence permanently weaker than every later unit's, silently. If mutation
genuinely cannot run here, that is a Phase 1 blocker to fix or escalate, not a
flag to drop.

## 2026-08-10 — the loop repo runs from a pinned checkout inside the mount

`vit_harness.py` and `vit_mutate.py` need three things at once: ROSCO-r2's
Fortran objects, this repo's `harness/` package, and a Linux toolchain. No
environment had all three. `vit-dev` mounts the campaign but not the loop repo;
`vit-harness` mounts the loop repo but not the campaign; the host has both plus
gfortran, but it is macOS and the objects are Linux, so they cannot link.

Both scripts resolve their imports repo-relatively
(`sys.path.insert(0, parents[1])`), so they need the repo PRESENT at any path --
not a new mount and not re-plumbing. So a checkout lives at

    ~/Artifacts/vit_translation/translation-loop   ->  /workspace/translation-loop

cloned from the LOCAL repo, not origin: origin does not have the fixes. No
container was recreated. Compilation now happens in the same container that
built the objects and ran the gate -- identical, not merely equivalent, which
matters because equivalence is a measurement and identity is a property. (The
toolchains do happen to be identical: both images carry gcc/gfortran/g++ 13.3.0
on Ubuntu 24.04 with glibc 2.39, despite being different images.)

**Pinned to `d58a418`, recorded here, and reversible with `rm -rf`.** Two
checkouts drift, and a campaign quietly running month-old harness code is the
silent-wrong-answer shape this project exists to remove. So both scripts now
stamp `loop_rev` into their JSON beside `{against, checked, failed}` and
`{total, killed, equivalent, score}`: the evidence already said what it measured,
this says which instrument measured it.

`vit-dev` has NO git, so the stamp returned "unknown" there -- silently, in the
direction that looks like it worked. It now falls back to a `.loop_rev` pin file
and reports `d58a418-pinned`, never dressed up as a verified read: it states
what the pin claims and cannot see whether the tree was edited after. Live git
still wins where it exists.

Verified against ROSCO-r2 rather than assumed: `vit_harness.py` generated 257
cases for ColemanTransform with the rules applied, and `vit_mutate.py` reached
the point of reading the translation and failed with a clean FileNotFoundError
because no translation exists yet -- which is the smoke test's job to produce.

Separately: `vit-dev`'s image was UNTAGGED (`b83f3bc715d0`). A `docker image
prune` would have deleted the campaign's execution environment, which holds two
weeks of state and is not rebuildable from a Dockerfile we have. Now tagged
`vit-dev-rosco:latest`.

## 2026-08-10 — unit #2: the extraction blocker was a dead call site

`vit extract` was recorded here as NOT WORKING, against its own success message:
three runs on `AddToList`, each printing `✓ Extraction successful` and
`WARNING: No state data captured.` The instrumented library, the run command and
the invocation window had each been ruled out by measurement, and the conclusion
drawn — reasonably, on that evidence — was that the front-end was broken and
kernel replay unavailable.

It was aimed at a line that never runs. `coverage/line_coverage.json` now
measures per-scenario line hits across all 27 scenarios, and
`ROSCO_IO.f90:1126` has **zero** in every one of them. So do the other four
`AddToList` call sites. There was no state to capture; KGen instrumented a
region the process never entered, and reported — correctly — that it had
instrumented it.

Pointed at `Controllers.f90:510`, which runs 23,999 times in scenario 27,
extraction captured 63 state files on the first attempt. Kernel replay is
available.

**What went wrong was the order of the checks, not any of them.** Each thing
ruled out was ruled out properly. What was never measured was the premise every
one of those checks rested on: that the call site executes. The previous entry
even records the suspicion — "the call site is unreachable in the default
scenario. True but not the whole story" — and then moved past it after
re-running against scenario 3, where it is equally dead. A hypothesis discarded
because a *different* scenario also failed is not a hypothesis tested.

## 2026-08-10 — a green kernel that a zero-writing stub also passes

`vit verify ColemanTransform` returned `62/62 passed`, `13,950 field entries`,
every one IDENTICAL. Then the same command, against a translation that reads
none of its arguments and writes `0.0` to both outputs, returned `21/21 passed`,
`4725/4725 IDENTICAL`.

The setup invocation window `0:0:1-20` captures the first 21 calls at the
extracted call site, and at `Controllers.f90:510` all 21 land at simulation
start, where `Azimuth` and `rootMOOPF` are 0. Every case fed the function zero
and expected zero back. The comparison was real, 4,725 values were compared,
and the measurement could not distinguish a correct translation from a constant.

**Widening the window did not fix it, and that is the more useful half.** With
`0:0:2000-2020,0:0:3900-3920` against scenario 6 — which
`coverage/line_coverage.json` shows executing the call site 3,999 times and the
function body 7,998 times — all 62 cases were *still* all-zero. Scenario 6 is a
1-DOF simulation that never drives azimuth or blade root moments. The line
executed 7,998 times on zeros.

That is P9 as a measurement rather than a maxim. Coverage says a line ran. It
does not say the line ran on data that can tell two translations apart, and the
gap between those two is invisible in every number either instrument reports.
The fix was a different SCENARIO, not a wider window: scenario 27 injects
synthetic azimuth and per-blade rootMOOP, and against it the same stub fails
62 of 63 cases.

The vacuous artifact is committed under `evidence/ColemanTransform/` rather than
deleted. A campaign that keeps only its greens has no record of what its
instruments looked like while they were lying.

## 2026-08-10 — E1.2 was not deferrable, and unit #2 is where it stopped being

`ColemanTransform` is five lines of arithmetic. Its C++ disagreed with the
Fortran by ~1 ULP on 105 of 200 random inputs.

The cause was not the translation. At Release (`-O3`) gfortran contracts
`cos(a)*r1 + cos(b)*r2 + cos(c)*r3` into 4 `vfmadd` instructions — confirmed by
objdump on the shipped `__functions_MOD_colemantransform`, 18 in
`Functions.f90.o`, 103 across `libdiscon.so`. Isolated by compiling the same
subroutine five ways: `-O3` without the flag differs on 105 of 200, `-O3` with
it on 0 of 200, `-O2` on 0 of 200 with or without.

`vit.yaml` had declared `-ffp-contract=off` since setup. `CMakeLists.txt` never
passed it. STATUS.md recorded the gap honestly and deferred it because closing
it means recapturing the baseline — which was true, and which is exactly why it
could not be deferred past the first unit whose evidence depends on it.

Closing it moved 489,734 of 5,252,000 gate values across 90 of 351 channels.
`baseline_arrays/` was regenerated from clean pre-integration source (20 of 27
files changed) and E3.2 re-run against the new set, so the campaign's red
evidence and green evidence describe the same baselines rather than two
different ones.

**`scripts/assert_fp_contract.sh` asserts two things, because either alone
passes while the requirement fails.** The flag is on the build line CMake emits,
AND the linked library holds zero FMA instructions. The first can be satisfied
by editing a file; the second cannot be satisfied by declaring anything. It is
whole-library rather than per-file on purpose: a per-file check goes green the
moment the file under translation is clean and says nothing about the next unit.

The C++ compiler needed it too, and only after integration — `vit integrate`
adds `enable_language(CXX)` and a `.cpp` to SOURCES and sets no CXX flags, so
the translation would have compiled at bare `-O3` and could have reintroduced
the same contraction from the other side, after the Fortran fix had been
verified. "Every compiler" is the criterion.

## 2026-08-10 — the same argument, the same mistake, two independent generators

`rootMOOP` is `REAL(DbKi), INTENT(IN) :: rootMOOP(3)`. Three of this campaign's
generators had to render it:

    interface_gen      kernel bridge + integration wrapper   `double*`         correct
    vit test-validate  the harness's Fortran side            `REAL(C_DOUBLE), VALUE`
    loop vitbridge     the harness's C side                  one double, read three deep

Both wrong ones dropped the rank of an EXPLICIT-SHAPE array — the shape that
carries its extent in its own declaration and therefore needs no synthesised
size parameter. Each generator handled assumed-shape (extent arrives as `n_x`)
and assumed-size (extent declared in vit.yaml) and fell through to the scalar
branch for the third case.

**Neither was found by review.** The first was caught by gfortran, and only
because the bridge `USE`s the module and so is type-checked against the real
interface. The second produced 256 failures out of 257 — loud, but by luck: both
sides read three doubles out of a one-double object at two different stack
addresses, so they disagreed about elements nobody had given them. The identical
defect on an INTENT(OUT) array would have compared one element of three and
reported a clean pass on a translation whose other two were never looked at.

Both fixed at the source with red-tested regression tests (X2), not worked
around here. The loop fix is `translation-loop@8641fbc`; the campaign's pinned
checkout moved `d58a418 -> 3c88913` and `.loop_rev` with it, so `loop_rev` in
every artifact still names the instrument that produced it.

**VIT was NOT upgraded**, and that is the change deliberately not made. This
workspace's VIT is at `d85b33b`; the pinned loop repo expects a newer one (it
cites VIT dev note `202608092223` for a per-unit harness directory this VIT does
not use). Swapping VIT mid-unit would mean this unit's extract/verify/integrate
evidence came from two different tools. `scripts/harness.sh` reconciles the two
directory layouts instead, in a committed script rather than in steps somebody
remembers to repeat.

## 2026-08-10 — what a post-integration harness can actually measure

E4.5 asks for the generated differential harness to be re-run against the
integrated build. Run literally, it does not link: the integrated
`colemantransform.cpp.o` and the harness's own copy of the translation are two
definitions of one symbol.

Removing the collision exposes the real limit. After integration the Fortran
`ColemanTransform` **is** the C++ — the body is a wrapper calling
`colemantransform_c`. There is no independent Fortran reference left in the
build, by construction, so no harness against the integrated build can compare
the arithmetic. One that appears to is comparing the translation with itself.

What remains, and is covered nowhere else, is the INTEGRATION WRAPPER: the
marshalling between the Fortran caller and the C++ callee. That is a generated
bridge, and two other generated bridges in this same campaign were found
dropping this unit's array rank. So the post-integration run routes the
reference side through the real wrapper and the got side straight to the
translation — identical arithmetic on purpose, so that the only thing a failure
can mean is that the wrapper corrupted an argument.

Red-tested: swapping the wrapper's two INTENT(OUT) arguments failed 217 of 217
cases; reverting restored 217/0. The artifact carries a `measures` field saying
this in full, because `checked 217 failed 0` is otherwise indistinguishable from
the pre-integration run that compared against real Fortran, and it is not that.

## 2026-08-10 — three of the gate's 27 scenarios execute no controller code

Not a unit finding, and recorded here because it bears on every number this
campaign reports. Scenarios 10, 14 and 24 run **zero** lines of Controllers.f90,
ControllerBlocks.f90, Filters.f90 and Functions.f90. Scenario 13 runs 12 lines
of Functions.f90 and zero of Controllers.f90. All four enter DISCON and read
parameters — DISCON.F90 42%, ReadSetParameters 39% — and stop.

Scenario 14 prints `Scenario 14: PASSED`, and 11 of its 13 output channels are
one repeated constant. Its 104,000 values are counted in the gate's 5,252,000
and constrain nothing about the controller.

This is E3.3's stated failure mode — "no rejected runs, no all-constant channel
sets" — and E3.3 is still `manual`. It is now measured rather than suspected, so
it belongs to whoever closes E3.3 rather than to this unit. The gate's compared
count is not wrong; it is 5,252,000 values of which some number are inert, and
nothing currently says which.

## 2026-08-10 — the invariant layer asks to be edited, and the hash forbids it

**For the Driver, as a proposed amendment to the method rather than to this
campaign.**

`RUNBOOK.md`'s invariant layer says: "`Status here` starts at `inherited,
unexercised` for every rule. Update it to `exercised at unit #N` the first time
the rule actually bears on the work. Rules still unexercised after N units are
the P4 report."

It also says, four paragraphs above, that editing that layer changes
`invariant_hash` and the Driver reads the change as `method_amendment_proposed`.

Those cannot both be followed. Verified rather than assumed:
`loop/bootstrap.py:143` hashes everything between the front matter and
`\n---\n\n# Runbook — target layer`, which is exactly the block containing the
`Status here:` lines. Writing `exercised at unit #2` on P9 would change the
hash and escalate a routine bookkeeping update as a proposed change to the
spec. The unit's instructions said not to modify the invariant layer, so it was
not modified — verified unchanged at `e4e6b553a5588907` by re-running that same
function.

Two ways out, both for the Driver to choose between: exclude the `Status here:`
lines from the hashed body, or move the status ledger out of the invariant layer
into a target-layer table. The second is cheaper and keeps the hash meaning
exactly one thing.

Meanwhile, so the P4 report is reconstructible without editing anything, the
rules unit #2 actually bore on:

    P3   the kernel's first green named 4,725 compared values and could not
         go red; the stub artifact is committed
    P4   vit.yaml comments restored from the pinned revision, not retyped
    P7   the oracle is the original Fortran -- and after integration it no
         longer exists in the build, which is what limits E4.5
    P8   E1.2 became a disposition (close it) rather than a blocker
    P9   coverage said the line ran 7,998 times; it ran on zeros
    C1   plan.json's bridge_feasible prediction confirmed against the built
         wrapper, not against the declaration it was derived from
    C2   coverage data did not exist; scripts/coverage.py now produces it
    C3   extraction works; the recorded blocker was a dead call site
    C12  the vacuous kernel recorded with its passing artifact before the fix
    X2   two generator bugs fixed at the source, not worked around
    X3   VIT deliberately NOT upgraded mid-unit
    X4   every green red-tested on first use: kernel, gate, post-integration
         harness, and the E1.2 assertion itself
    E1.2 closed
    E3.5 baselines regenerated
    E4.5 the post-integration harness, and what it cannot measure
    E4.6 mutation 35/35

    NOT exercised, and worth naming: P5 (no gap was closed by addition),
    P6 (nothing absent had to be rendered), E1.3, E1.5, E1.6, E2.x, E3.3,
    E3.4, E3.6, E5.x.

## 2026-08-10 — --timeout-s raised to 7200, with the basis measured not guessed

The ColemanTransform session hit the 3600 s default. Its own commit timestamps
split the hour, so the number is derivable rather than chosen:

    17:45:33  dispatched
    18:15:23  Phase 1/3 -- closed E1.2, regenerated the baselines it invalidated
    18:40:50  ColemanTransform translated, verified four ways, integrated
    18:44:07  state commit
    18:45:33  killed by the timeout, ~90 s after its last commit

So roughly **30 minutes of one-time infrastructure** and **~28 minutes of unit
work**. Telemetry agrees in shape: 100 events, 29.6 minutes of bracketed command
time, dominated by extract (28), test-validate (20) and gate (18).

The infrastructure half does not recur, so it says almost nothing about a
typical unit -- which is exactly why 3600 cannot be defended by "it nearly
fit". What it does say is that a unit's own cycle here is ~28 minutes for a
SMALL mirror unit with five scalar/array arguments, and the campaign this method
comes from recorded a 4.8x upward cost trend across its run.

**7200 s.** Roughly 4x the observed unit cycle, which absorbs that trend without
being unbounded. Not a default change: `run_campaign.py` still defaults to 3600,
because one campaign's single measurement should not silently retune every other
campaign. It is passed explicitly at launch and recorded here.

A timeout is no longer a run-ender in any case: `Driver.run_unit` now catches at
the dispatch, records the unit with its cost as UNKNOWN rather than $0.00, and
escalates `session_failed`. The timeout is now a bound, not a cliff.

## 2026-08-10 — the campaign's VIT was 89 commits stale; reconciled at unit 1

`vit-dev` sets `PYTHONPATH=/workspace/vit`, which is `~/Artifacts/vit_translation/vit`
— NOT the canonical checkout the handoff named. So every `vit extract /
translate / verify / integrate` in this campaign ran from a tree 89 commits
behind canonical, and ColemanTransform's evidence already did. Merge-base
`10f7cd4`; 1 campaign-only commit (`d85b33b`) plus 37 uncommitted lines.

Nobody checked which VIT the container imports. The answer was one `git log`
away throughout.

Those 89 commits are the post-replication work whose purpose was removing
fails-green defects from VIT — the redtest `candidates()` fix, the check
registry with its deferral list emptied, reachability's `compile_references`,
exponent grouping, the locale fixes. Running a campaign about silent
verification failure on an instrument known to contain silent verification
failures is self-defeating. Reconciling at unit 1 costs a rebuild and a
re-verify; at unit 40 it would be unaffordable and the instrument would have to
be defended rather than fixed.

Merged to `d07a716`. The one conflict is the argument in miniature: both sides
had independently fixed the SAME explicit-shape-array bug, and canonical's was
strictly better — ours hardcoded `REAL(C_DOUBLE)` for every REAL array, which IS
the kind defect canonical fixed with `_iso_c_for()`, so keeping ours would have
traded a caught rank bug for an uncaught precision one on every `REAL(ReKi)`.
The Makefile link fix was superseded the same way and removed rather than kept
alongside.

Re-verified after the merge rather than assumed: `vit status` and `vit parse`
work (`rootMOOP: REAL(8) INTENT(IN)(3)`, rank preserved), the integrated build
gates 5,252,000 / 0, and `vit_harness` generates 217 cases.

`vit_rev` is now stamped into harness and mutation artifacts beside `loop_rev`,
via a `.vit_rev` pin file because `vit-dev` has no git. Every instrument that
produces evidence should say which version of itself produced it — the loop
stamp caught the loop's own drift on its first live run, and this one would have
caught the 89 before the campaign started.

## 2026-08-10 — ColemanTransform's disposition cleared for re-dispatch

The session recorded `integrated`. The done-condition, evaluated against exactly
what it committed, returns INCOMPLETE at 12 of 13: P12 fails because
`mutation/ColemanTransform.json` carries the pre-fix schema (`total`/
`equivalent`) that P12 cannot read. So the plan asserted a closure the loop had
never verified — the session set the disposition, and the loop, having crashed
on the timeout, never checked it.

`next_unit` selects on `disposition`, so the field had to be cleared for the
unit to be walked again. Evidence is kept: those artifacts exist and are real.
What is NOT kept is the claim that they add up to a closed unit.

Not waived. DECISIONS.md already records that mutation is mandatory for all 69
at a 1.0 threshold; closing unit #1 at 12/13 would make its evidence permanently
weaker than every unit after it, which is the specific thing that commitment
forbids. The artifact must be regenerated by the tool — reset_to_clean ->
generate -> mutate -> restore — not hand-edited to the names P12 wants. A number
I believe is genuine is still fabricated if I type it in rather than measure it.

## 2026-08-10 — unit #2, second pass: the closure measured, and three scripts
   that were wrong the whole time

The unit was re-dispatched to regenerate one artifact. It ended up re-measuring
every layer, because the first thing built was the thing that had been missing:
a way to RUN the done-condition instead of guessing at it.

### `scripts/done_check.py` — run the condition, do not predict it

The previous session set `disposition: integrated` and the loop, crashing on its
timeout, never checked. The claim stood in `plan.json` for four commits. Nothing
in a unit session could have caught it: the done-condition lives in the loop
repo, is evaluated by the Driver, and a session has no way to ask it.

So it is asked now. `done_check.py` imports `loop.done.DoneVerifier` and builds
`DoneConfig` with **exactly** the four globs and four state files
`scripts/run_campaign.py` builds — copied from that call site, not paraphrased
from the docstrings, because a checker that approximates the checker is a second
opinion, not a check. It reads and never writes.

At the start of this pass: INCOMPLETE, 10 of 13. P12 FAIL, P2 dirty (my own
untracked file), P3 NOT_EVALUABLE (the cleared disposition). That is the real
starting position and it was not knowable before.

The 12-of-13 recorded in the previous entry and this 10-of-13 are the same
verdict counted at different moments; the extra two are artifacts of the
re-dispatch itself, not new defects.

### P12: a real 35/35 that the check could not read

`vit_mutate.py` wrote `{total, equivalent}`; `loop/done.py` P12 reads
`{mutants, equivalent_declared}`. `int(d.get("mutants", 0) or 0)` is 0, so P12
reported `mutation_no_mutants: the operator set reached nothing in this unit`
about an artifact recording 35 mutants, all killed.

**Failing with the wrong reason is worse than failing.** It failed safe, which
is the direction that matters least here: a session reading that message goes
looking for missing operators in `cppmutate.py` and finds nothing wrong, because
nothing is wrong there. Fixed at the source in the loop repo (`13265e7`), which
now emits both spellings — the old pair for a human reading the file, the
canonical pair for the loop — rather than dropping names that campaigns already
have on disk. Regenerated here by the tool: 35 mutants, 35 killed, score 1.000,
baseline verified green first.

**33 of the 35 kills are behavioural.** Two `compare_op` mutants are scored
`killed (no compile)`. A translation with no comparison operator in it has
nothing for that operator to reach, and counting a build failure as a kill is
defensible — but it is not the same evidence as a case mismatch, and 35/35 reads
as though it were. Recorded in `plan.json`'s `observability`, since a mutation
score is only as good as what the reader thinks it says.

### Three defects in this campaign's own scripts

All three had one shape: **a stale artifact on disk made a broken run look like
a working one.** `harness/ColemanTransform.json` from the first pass sat there
saying `199 checked, 0 failed`. Three runs today failed to overwrite it, and the
first two were read as successes — including one where the link had died and I
read the previous session's numbers back to myself as if they were new. The tell
was `loop_rev: 3c88913-pinned` in a file a `ebce989` instrument had supposedly
just written. The version stamp caught this, exactly as the entry above hoped it
would; without it there was nothing to notice.

The general rule, which belongs to the method and not to this campaign: **an
artifact that a run failed to write is indistinguishable from one it wrote,
unless something in it identifies the run.** Timestamps do not survive git.
Every evidence-producing instrument should stamp its own version, and a reader
should check the stamp before believing the numbers. `_harness_stamp.py` now
stamps `vit_rev` beside `loop_rev` — the post-integration artifact measures the
integration WRAPPER, the one thing in this pipeline VIT generates, and it was
the only artifact not recording which VIT that was.

1. **The skew `harness.sh` existed to reconcile had closed.** VIT `d07a716`
   brings in dev note `202608092223`'s per-unit test directory, so
   `vit test-validate` now writes the Makefile where `vit_harness.py` already
   looked. The script exited 1 with "vit test-validate did not write Makefile"
   while the Makefile sat one level down, timestamped four seconds earlier by
   that same command. Both layouts are now handled and which one ran is
   PRINTED. The old branch is kept rather than deleted: the artifacts already
   committed for this unit came from the old layout, and a script that cannot
   reproduce them stops being able to say where they came from.

2. **The post-integration LIBS extraction passed the words `LIBS` and `+=` to
   the linker.** It grepped `^LIBS =` through to the first blank line and
   stripped the prefix per line. The new VIT's Makefile ends the block with a
   second assignment, `LIBS += -lgfortran -lm`, which `^LIBS =` does not match,
   so two makefile tokens arrived at `ld` as filenames. Replaced with asking
   `make` to evaluate its own variable through a two-line generated include.
   A hand-rolled parser that knows one assignment operator and not the other
   will break again on the next generator change; `make` already knows.

3. **A red run could not be stamped.** `./test` exits non-zero when cases
   mismatch, and under `set -e` the script aborted one line before
   `_harness_stamp.py`. The red-test artifact was therefore written with
   `against: "translation"`, no `measures`, and no revisions — the run whose
   artifact most needs to say what it measured was the only one that could not.
   The stamp now runs whenever a parseable artifact exists, a link failure is
   still a hard stop, and `--red-test "<perturbation>"` records the
   perturbation and INVERTS the verdict, so a red test that stays green is
   itself a failure. The previous pass's red-test artifact carries a `red_test`
   block that was added by hand; this one was written by the tool.

### The pre-integration harness could not link, and that is `reset_to_clean`'s
    boundary showing

`reset_to_clean.sh` restores the Fortran body and deliberately leaves
`CMakeLists.txt` alone. After a unit has been integrated once, the build target
still compiles `rosco/controller/src/<stem>.cpp`, so the tree it produces is a
hybrid: pre-integration Fortran, post-integration build. The differential
harness compiles its own copy of the translation and links the build's objects,
so `<stem>.cpp.o` is a second definition of the same function and the link dies.

This could not have happened on the first pass — nothing had been integrated
yet, so the object did not exist. **It will happen on every unit from here.**

`harness.sh` now drops this unit's own object from the generated LIBS, and only
its own: other units' `.cpp.o` files are reached through their `_c` bridges and
are part of the reference build. The edit goes into the Makefile rather than
onto one `make` command line, because `vit_mutate.py` runs `make` itself — a fix
that lived only in the shell script would leave every mutant failing to link,
scored `killed (no compile)`, i.e. a 1.000 that measured nothing. `vit_mutate.py`
refuses to score when the baseline will not build, so it would have failed
loudly rather than lied. The point is that it must not have to.

The better fix is to make `reset_to_clean.sh` revert `CMakeLists.txt` too, so
"clean" means the pre-integration BUILD and not merely the pre-integration
SOURCE. Not made here. That script is red-tested against ten assertions and the
second pass of unit #2 is not where to re-open it; recorded in STATUS.md as an
open decision for before unit #3.

### What the re-measurement found: nothing wrong with the translation

Every layer was re-run under VIT `d07a716` / loop `ebce989`, and every one
agrees with the first pass. `vit integrate` regenerated `Functions.f90`,
`CMakeLists.txt` and `colemantransform.cpp` byte-identical to what `d85b33b`
produced — `git diff` over the three is empty. Kernel 63/63 on live inputs, the
zero-writing stub still fails 124 field entries, harness 199/0 against clean
Fortran, mutation 35/35, post-integration 199/0 with the wrapper swap turning it
199/199 red, gate 5,252,000/0 with the perturbation moving 124,353.

Worth stating plainly, because the re-run was expensive and found no arithmetic
defect: **that is the result, not a wasted pass.** The 89-commit VIT reconcile
was justified on the grounds that a campaign about silent verification failure
must not run on an instrument known to contain silent verification failures. The
evidence that the reconcile did not disturb this unit is the thing that lets the
first pass's conclusions stand — and it turned up three defects in the
campaign's own scripts that the next unit would have hit anyway.

### The bridge prediction

CONFIRMED, and now confirmed twice by two generators. All five arguments cross
with no decomposition; details in `plan.json`'s `bridge_feasible.observed`.

## 2026-08-10 — for the Driver: the invariant layer's `Status here` cannot be
   maintained by a unit session

The invariant layer of `RUNBOOK.md` says of every rule:

> `Status here` starts at `inherited, unexercised` for every rule. Update it to
> `exercised at unit #N` the first time the rule actually bears on the work.
> Rules still unexercised after N units are the P4 report: inherited, never
> tested against this target.

That instruction lives INSIDE the invariant layer and can only be carried out by
editing the invariant layer — which changes `invariant_hash`, which the Driver
reads as a proposed amendment to the method and escalates. A unit session is
told not to edit that layer. So the field is unmaintainable by the only actor
positioned to know the answer, and after two closed units all 44 rules still
read `inherited, unexercised` while at least a dozen demonstrably bore on the
work. This pass alone exercised C1-C11, P3, P7, P9, X2 and X4.

The P4 report the field exists to produce is therefore empty-by-construction
rather than empty-by-measurement, which is the failure this method is about.

Not fixed here, because fixing it IS the escalation. Raised as a proposed
amendment. Three shapes worth considering, in preference order:

1. Move `Status here` out of `RUNBOOK.md` into a generated sidecar
   (`runbook_status.json`) that a unit session may write and the invariant text
   references. The rule list stays hashed; the observation about it does not
   need to be.
2. Exclude `Status here:` lines from the `invariant_hash` computation, so
   updating them is not an amendment.
3. Have the Driver derive it, from the predicates each unit's `done_check`
   evaluated and from what the session's commits touched — no session writes it
   at all. Weakest of the three: it can see which contracts were checked, not
   which prohibitions were felt.

Until one of them exists, the field should be read as "nobody could write this",
not as "these rules were never exercised".

## 2026-08-10 — ceiling $3000, per-unit cap $25, both settled before unit 2

Both parameters are recorded in `run.json` and the Driver REFUSES to resume a
run whose limits changed, because carrying spend across a changed ceiling
misreports both the spend and the headroom. So they had to be decided together
and decided now: settling the total today and the per-unit cap at unit 10 costs
the same restart twice. One unit is closed, so the restart is free; at unit 40
it would not be.

**Total $3000, not $1000 and not $2000.** The total only functions as a guard
BELOW `68 x per_unit_cap`. At the old $35 cap that product is $2,380, so $2000
sits inside the nominal range and could bind from ordinary spending — the exact
failure we are paying a restart to avoid. Above it, the total stops being a
truncation risk and becomes what a total is for: catching retry storms. A retry
is a second dispatch, so the worst case is 68 x $50 = $3,400 at the new cap, and
$3000 halts that.

**Per-unit cap $25, down from $35.** A generous cap SILENCES the per-unit
signal: `unit_overran` almost never fires, so costs get watched by hand instead
of reported. ColemanTransform closed at $11.885, so $25 is ~2x the observation —
generous enough not to false-fire, tight enough to catch an outlier. Exceeding
it escalates AFTER the fact rather than killing work, so a tight cap buys signal
at no cost. 68 x $25 = $1,700 nominal, comfortably inside $3000.

**$11.885 is a FLOOR, not an estimate**, and the projections above are built on
it knowingly. That dispatch found its translation and integration already
committed; the cold run that did that work timed out and its spend was never
recorded, so nothing in the ledger reflects a full cold unit. The accounting
restarts here: `run.json.pre-ceiling-change` preserves the old record and
`units.jsonl` keeps ColemanTransform's row, so the campaign's true spend is
$11.885 plus whatever this run reports.

Unit 2's cold cost will be reported when it lands. Not as a gate — it is
ColemanTransform's sibling and among the simplest units in the plan, so it is
another floor rather than a representative sample — but because the gap between
warm and cold is the number that makes every later projection honest.

## 2026-08-10 — unit #1: the verdict was right, the reason was about a
   different generator, and the matrix could not have known

`plan.json` predicted `AddToList` cannot cross the C bridge, basis
`list: c_alloc_inout does not cross (INTENT(INOUT) ALLOCATABLE needs an
ALLOCA...)`. **The verdict is confirmed. The basis is refuted.** Both halves
were measured; neither was argued.

### Confirmed: the generated bridge compiles and does nothing

`vit interface` emits `INTEGER(C_INT), INTENT(INOUT) :: list(*)` and
`INTEGER(C_INT), VALUE :: n_list`, under a wrapper whose dummy reads
`INTEGER(4), INTENT(INOUT) :: list(:)` — the ALLOCATABLE attribute gone.
`fortran_parser` had recorded `is_allocatable=True`; `generate_fortran_wrapper`
attaches `, ALLOCATABLE` only for `is_alloc_return` dummies and dropped it
without a word.

`AddToList`'s whole contract is `allocate(clist(isize+1))` → copy →
`move_alloc(clist, list)`: it replaces the caller's data pointer AND its extent.
A bare pointer and a by-value length carry neither. Three implementations were
built against one driver on the live ROSCO path (both call sites `ALLOCATE`
first, then append):

    orig  A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17
    vit   A1 size=4         A2 size=4         A3 size=4
    cfi   A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17

The array never grows. The appended values are written one past the end of the
caller's heap block. gfortran said nothing, g++ said nothing, the run did not
crash. **A wrapper that compiles and does nothing is worse than one that does
not compile** — the second is caught by the build.

### Refuted: the limit is VIT's, not Fortran's

Fortran 2018 permits an `ALLOCATABLE` dummy in a `BIND(C)` interface. The C side
receives the caller's own `CFI_cdesc_t *`, and `CFI_deallocate`/`CFI_allocate`
through it do exactly what `move_alloc` does. A hand-written CFI bridge
reproduced the oracle **byte for byte on both branches**, including
`move_alloc`'s reset of the lower bound, under gfortran 13.3 / g++ 13.3.
Red-tested: one perturbed token in the CFI body turns the probe red.

So `c_alloc_inout does not cross` was a true sentence about the tool wearing the
grammar of a claim about the language. The distinction is not academic — it is
the difference between "close this unit as impossible" and "VIT is four
`plan.json` entries short of a feature it can have."

### Why the matrix could not have known: it measures a different generator

`tests/test_conformance.py:probe()` fed `bridge` and `compiles` from
`test_validate.generate_fortran_bridge` — the **differential harness's** Fortran
side, linked into a test binary and never into the library. On `AddToList` it
fails with `Actual argument for 'list' must be ALLOCATABLE`, which is where
`[c_alloc_inout] compiles = "no"` came from and therefore where the plan's basis
came from. The matrix never called `generate_fortran_interface_block` or
`generate_fortran_wrapper`, the two generators whose output `vit integrate`
writes into the shipped source, and those compile clean.

**The prediction was right by luck of a different generator failing.** Any cell
whose harness bridge happened to compile would have read `yes` and the campaign
would have integrated a no-op. That is the campaign's own failure shape, sitting
in the file built to prevent it: a column that means less than its name.

### Fixed at the source, both halves, red-tested (X2, P5)

1. `interface_gen.assert_integration_bridgeable` raises `UnbridgeableSignature`
   for an ALLOCATABLE INTENT(INOUT) dummy. Called from the interface-block
   generator, the wrapper generator, and the C++ scaffold — `vit translate` is
   the earliest point at which a wasted body is avoidable. `vit translate`,
   `vit interface` and `vit integrate` on `AddToList` now exit 1 and write
   nothing. **Only INTENT(INOUT) is refused**, because only INTENT(INOUT) was
   measured; INTENT(IN) ALLOCATABLE has the same dropped attribute and no
   campaign has hit one, and inventing a verdict for an unmeasured cell is the
   habit this refusal exists to break.
2. `test_conformance.py` gained an `integrates` column that runs the integration
   generators and compiles their output. Regenerating the matrix moved
   `c_alloc_inout` to `integrates = "refused"` and turned up **`c_complex_in`**,
   a cell every previous column called supported, whose integration wrapper
   emits `COMPLEX(z, C_DOUBLE_COMPLEX)` for an already-COMPLEX dummy and does
   not compile. Five of 39 cells now disagree between the two columns.

Both red-tested by disabling them and confirming the suite fails
(`test_cell_matches_the_record[c_alloc_inout]`, three refusal tests), then
restoring. VIT suite: 930 passed.

**Two probe defects were found first, and both were the same mistake in
miniature.** The `integrates` column initially reported `no` for seven cells:
three because the synthetic module did not `USE vit_conformance_types`, three
because the generated view-populator modules did not exist, one because
`_compiles` matched only `Error:` and not `Fatal Error:` and so reported
`compilation terminated.` — a diagnostic naming no cause. Every one of those was
measuring the probe. They are fixed and the reasons are in the code.

### The gate cannot see this unit, measured twice

`coverage/line_coverage.json` says the body and all five call sites have zero
hits in all 27 scenarios. Confirmed independently of gcov by perturbing BOTH
branches (`= element` → `= element + 1000`, 2 replacements), rebuilding, and
re-gating: **0 of 5,252,000 values moved.**

So `gate/AddToList.json` — 5,252,000 compared, 0 mismatched — is a green that
constrains nothing about `AddToList`. It is committed next to the red test that
says why, because the green alone would be a lie by omission.

### A reading hazard the dead red test exposed

On a `--perturb-*` run, `gate.py` left `verdict` at the COMPARISON's verdict. So
a red test that correctly went red wrote `verdict: FAIL`, and one that failed to
go red wrote `verdict: PASS`. `AddToList` produced the dangerous half: a gate
blind to the unit, filed under `PASS`.

`verdict` on a perturbing run is now `RED_TEST_PASS` / `RED_TEST_FAIL`, and the
comparison's own verdict is kept under `comparison_verdict`. **The new spelling
is deliberately not `PASS`/`FAIL`**: `gate/ColemanTransform.redtest.json` was
written under the old convention, and a reader can now tell which convention a
file uses from the value itself rather than from its date. Verified in both
directions: the `LPFilter` perturbation over scenarios 1/3/6 moved 265,890 of
780,000 and wrote `RED_TEST_PASS` (rc 0); `AddToList` wrote `RED_TEST_FAIL`
(rc 1).

### Disposition: `blocked`, and it escalates

SPEC §8.4 requires escalation when a disposition is `blocked` and the blockage
is in the substrate. It is: VIT does not generate CFI descriptors. Two questions
for the Driver, neither decidable inside a unit session:

1. **Should VIT learn the CFI bridge?** It is demonstrated working here, and it
   is what `AddToList`, `Read_OL_Input`, `ParseInAry_Opt` and `ParseDbAry_Opt`
   are waiting on — 4 of the 5 `bridge_feasible: no` entries in this plan. It
   also has to reach `test_validate`'s bridge and the loop's `vitbridge` before
   a blocked unit can produce a mutation score, so it is three generators, not
   one.
2. **Can a unit no scenario reaches ever close?** P9 can only be satisfied
   vacuously here, P12's mutation score cannot be produced, and P12's red-test
   fallback is impossible by construction. `integrated_unexercised` exists in
   the vocabulary for exactly this case and the done-condition has no branch for
   it. Until it does, every dead unit in this plan will end `blocked` for a
   reason that has nothing to do with its translation.

### What was NOT done, and why

No translation was written and none was integrated. The correct CFI translation
exists in `evidence/AddToList/bridge_probe/addtolist_cfi.cpp` and was **not**
promoted into `translations/`: integrating it would need the interface block and
wrapper hand-written into `ROSCO_Helpers.f90`, which is the generator's job
(X1/X2), and `reset_to_clean.sh` and `restore_integrated.sh` select integrated
files by looking for a `<name>_c(` bridge that would not exist. Shipping it by
hand would buy one function and leave the next three at the same wall with the
tooling still unable to see it.

### The done-condition, run rather than predicted

`python3.12 scripts/done_check.py AddToList --baseline 2ef6d0d` →
**INCOMPLETE, 11 of 13.**

    P1..P8   ok
    P9       ok   -- on the vacuous gate green, which is the point
    P12    FAIL   mutation_missing: no mutation/AddToList.json
    P11    FAIL   harness_not_rerun: no harness/AddToList.postintegration.json
    P13,P10  ok

The two failures are the two artifacts that cannot exist: both the differential
harness and the mutation score are generated from the bridge that does not
cross. There is nothing to fix that is inside this unit — which is escalation 2,
now measured rather than argued.

`--baseline` is required: `done_check.py` infers the window from the first
commit touching `plan.json`'s `translation`, and a blocked unit has none.

Recorded here rather than left to be re-derived, because the alternative is a
later session reading `blocked` beside a green STATUS and assuming the condition
was never run — which is exactly what happened to unit #2.

## 2026-08-10 — unit #1, second dispatch: the prediction is refuted, because
   the tool learned the thing it was refusing

The unit was re-dispatched with exactly two unmet conditions: `P12
mutation_missing` and `P11 harness_not_rerun`. Both are generated from the
bridge, so "address exactly these" had one honest reading — build the bridge.
That is escalation 1, answered by doing it rather than by deciding it.

### What crossed, and how

```fortran
    INTERFACE
        SUBROUTINE addtolist_c(list, element) BIND(C, NAME='addtolist_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), DIMENSION(:), ALLOCATABLE, INTENT(INOUT) :: list
            INTEGER(C_INT), VALUE :: element
        END SUBROUTINE addtolist_c
    END INTERFACE
```

The dummy is declared exactly as the original declares it, **no extent
parameter is emitted** — the descriptor carries the extent, and a separate
`SIZE()` is stale the moment the callee reallocates — and the C side receives
`CFI_cdesc_t*`, which IS the caller's descriptor.
`CFI_deallocate`/`CFI_allocate` through it do what `move_alloc` does.

Three generators, as the escalation predicted it would take: `interface_gen`
(what ships), `test_validate.generate_fortran_bridge` (the differential
harness's Fortran side), and the loop's `vitbridge`/`emit` (the C++ side).
Missing any one of them leaves a unit that integrates and still cannot produce
a mutation score — which is precisely how this unit was blocked.

**Nothing about the integration is hand-written.** The first dispatch declined
to promote the working CFI body into `translations/` because the interface
block and wrapper would have had to be hand-written into `ROSCO_Helpers.f90`
(X1/X2). They are generated now, which is what made promoting it legitimate.

### Refused, still, and why the list is the point

`assert_integration_bridgeable` now refuses a SCALAR allocatable, rank >= 2, a
CHARACTER or derived-type element, and any type/kind the interface cannot
declare as the actual's own (LOGICAL, COMPLEX). **Every one of those is refused
for being unmeasured, not for being impossible**, and each exception message
says so. Rank >= 2 is the sharp case: the descriptor carries any rank, and the
C++ body's indexing convention is exactly where a silent error would live, so
the refusal is about what nobody has run against an oracle.

### The mutation score went 0.739 -> 0.920 -> 1.000, and only the middle step
    was about the harness

First run: 17 of 23, six survivors. Two of them were real and neither was a
weakness of the input set:

* `malloc(isize + 1)` -> `isize - 1`: heap under-allocation, then a write past
  the end. No value comparison can see it.
* `memcpy(..., isize + 1)` -> `isize + 2`: a copy past the buffer, same.

Both existed because ONE quantity — `SIZE(clist)` — was written three times: the
allocation, the new upper bound, and the copy length. Two of the three
restatements were therefore unobservable by construction. Naming it once
(`nsize`) leaves a single site, and that site decides the extent the caller
sees; both mutants are then killed by the extent comparison.

A third came from writing Fortran's `do i=1,isize` as a 0-based C loop:
`i < isize` perturbed to `i <= isize` writes a slot the next statement
overwrites, and is invisible. Written 1-based, as the Fortran has it, the
perturbation goes the other way and leaves an element uncopied — which the
comparison sees. **The more literal transcription is the more observable one.**

The two survivors that remain are DECLARED equivalent, with the reason in
`mutation/AddToList.equivalences.json`: `CFI_allocate`'s `elem_len` argument is
ignored unless the type is `CFI_type_char`, `CFI_type_ucs4_char` or
`CFI_type_struct` (F2018 18.5.5.5). No input can distinguish them — which is
what "equivalent" has to mean, and is not what was true of the two above. The
file records the removed pair too, so the distinction survives the commit.

### The unallocated actual is an input the generator had no rule for

`.NOT. ALLOCATED(list)` is not extent 0, and it is the whole `else` branch.
`_extent_plan` ranges over 1, 3, 4, ... and would never have produced it, so
the branch would have been unreachable by every case — 25 of the 43 cases here.
Modelled as a two-valued flag it becomes R2's business, and R2 already
guarantees every declared value appears. No new rule, an existing rule given
something to act on.

Measured: perturbing the allocated branch turns 18 of 43 red, the unallocated
branch 25 of 43, and 18 + 25 = 43. **No case is inert.**

### Two defects found in generators on the way, both fixed at the source (X2)

1. `vit_translated.h` is included FIRST by every generated `.cpp`, ahead of the
   translation's own includes, so a declaration naming `CFI_cdesc_t` made the
   header uncompilable. The interop include is now added to it on demand.
2. **Every file `vit integrate` generated carried `// After verification:
   <name> kernel PASSED`.** Integration does not run a kernel, does not read a
   kernel result, and cannot know whether one exists. On this unit the sentence
   was flatly false: there is no kernel and there cannot be one, because no
   scenario reaches the function. A generator asserting a verdict it never
   checked, shipping inside the translated source, is the exact failure this
   campaign exists to find — and it was found by reading the file it had just
   written, not by any check.

   `rosco/controller/src/colemantransform.cpp` was NOT regenerated, so unit
   #2's "regenerates byte-identical" property no longer holds for those comment
   lines. Recorded in STATUS rather than silently repaired.

### A hand-copied ABI in a shell script

`scripts/harness.sh --post-integration` wrote its shim from a heredoc
containing `void ${UNIT}(double*, double, int, double*, double*)` — ColemanTransform's
parameter list, as a literal, for every unit. This unit would have failed to
compile, which is the good outcome; a unit whose arity happened to match would
have compiled and forwarded the wrong arguments. `scripts/_integration_shim.py`
asks `build_c_params`, so there is one spelling of the ABI rather than two.

### What this unit is verified BY, and what it is not

| layer | result | red test |
|---|---|---|
| differential harness, 43 cases vs clean Fortran | 0 failed | 18/43 and 25/43 red, one per branch |
| mutation | 23/23 behavioural killed, 1.000 | baseline must be green or it refuses to score |
| post-integration harness (wrapper marshalling) | 43 cases, 0 failed | wrapper corrupts an argument -> 43/43 red; green restored |
| gate, 27 scenarios | 5,252,000 values, 0 mismatched | perturbing the integrated C++ moved **0** — `RED_TEST_FAIL` |

The last row has not changed and is the point. **This unit closed without the
gate ever being able to see it.** Escalation 2 is therefore narrower than it
was: a dead unit whose signature crosses now has a path — harness plus mutation
— and a dead unit whose signature does not cross still has none, with
`integrated_unexercised` still unimplemented in `loop/done.py`.

### Instrument stamps: the artifacts were regenerated, not relabelled

The first pass's artifacts were produced by code that was still uncommitted, so
they stamped `22086e8-pinned` / `46a7f4f-pinned` — revisions that did not
produce them. VIT and the loop were committed (`37f8bdf`, `cf885e3`), the pin
files updated, and **every artifact re-run**: the pre-integration harness and
the mutation score from a `reset_to_clean` tree (they measure the clean Fortran
and must not be taken against the integrated build), the post-integration pair
after `restore_integrated`, and the gate last. The RUNBOOK's warning about
stale artifacts reading like fresh passes is the reason this was not skipped.

## 2026-08-10 — for the Driver: one candidate method amendment, and two that
   are not

The invariant layer was not edited, so `invariant_hash` is unchanged. Flagged
here instead, as the dispatch asks.

**CANDIDATE (method).** *A generator may not state a verification verdict it did
not obtain.* `vit integrate` wrote `// After verification: <name> kernel PASSED`
into every file it produced, having run no kernel and read no result. It is the
same defect P3 and P6 describe — a claim with nothing behind it, and absence
rendered as a value — but neither principle reaches a GENERATOR's output, only
a checker's. Nothing in the toolchain checks a comment, and this one shipped in
source for as long as the generator existed. If the method has a rule that
artifacts must be able to name what they measured, its scope should include
what tools emit, not only what verifiers report.

**NOT method, target.** The descriptor bridge itself, and the ordering
constraint that a pre-integration harness and a mutation score must be taken
against a clean tree: both are in the RUNBOOK's target layer, where they
belong.

**NOT method, but worth a second campaign's data.** "Raise a mutation score by
removing restatements of one quantity, and DECLARE only what no input can
distinguish." It reads like a rule and it has been measured exactly once. Two
survivors here were unobservable memory errors caused by writing `SIZE(clist)`
three times; two others are equivalent by the Fortran standard. The distinction
held up on this unit. One unit is not a rule.

## 2026-08-11 — Unit #3, ColemanTransformInverse: three instruments read for what
   they actually say

**The RUNBOOK's liveness recipe cannot be run on a unit whose outputs are
arrays, and nothing said so.** The recipe reads `reference` out of
`kernel/<Unit>/verify_fields.csv` and counts non-zeros; VIT logs a scalar field
with its computed and reference values, and an ARRAY field as a single row with
BOTH value columns EMPTY and `diff=size=3`. For ColemanTransformInverse — one
output, `PitComIPC(3)` — the recipe returns nothing at all. It does not fail; it
produces an empty answer that a session in a hurry would read as "no zeros
found".

The stub run is therefore not a second opinion here, it is the only opinion.
Against a translation reading no argument and writing `0.0`, 61 of 63 cases go
`OUT_TOL`. The 2 that do not are invocations 1 and 2 — simulation start, where
the axis inputs are still zero. That is unit #2's `1-20` window failure mode
surviving as two cases out of 63 instead of all 21, and it is visible only
because the stub was run.

The RUNBOOK's target layer now says this; the recipe is kept beside it, because
it is still the right check for a scalar-output unit.

**A kernel PASS is not a bit-identity claim, and this was read out of the
generated code rather than assumed.** `kv_ipc_real__dbki_dim1`, the comparison
KGen generated for this unit's array output:

    IF (ALL(var == kgenref_var)) -> IDENTICAL
    ELSE rmsdiff = SQRT(SUM((var-ref)**2)/n)
         IF (rmsdiff > kgen_tolerance) -> OUT_TOL  ELSE -> IN_TOL

`kgen_tolerance` is `1.D-14` and `rmsdiff` is ABSOLUTE, not relative. This
unit's outputs are of order 1e-3 rad, so a translation wrong at ~1e-11 relative
would score `IN_TOL`, `numOutTol` would stay 0, and the run would print
`✓ VERIFICATION PASSED: 63/63 passed`. Nothing in that line distinguishes it
from bit-identity.

What makes this unit's kernel evidence bit-exact is not the verdict. It is that
all 63 fields landed in the `ALL(var == kgenref_var)` branch — `IDENTICAL` in
the field log's status column. **Read the status column, not the verdict line.**
This is P3 applied to somebody else's instrument: a green must be able to name
what it compared, and `63/63 passed` names a tolerance, not an equality.

**One argument of this unit is invisible to both bit-exact instruments.**
`aziOffset` is 0 at all five call sites in all 27 scenarios: `IPC_aziOffset` is
`0.000 0.000` in all 14 `Examples/DISCON*.IN` and no scenario patches it,
`AWC_phaseoffset` is `0.000000000000` in all of them and the single scenario
that patches it (15) patches it to `'0.0'`, and `Controllers.f90:699` passes the
literal `0.0_DbKi`. A translation that ignored `aziOffset` entirely would pass
the kernel 63/63 and the gate 5,252,000 of 5,252,000.

The differential harness is what covers it — `R6_reference_literals` varies it,
and the three `aziAngle + aziOffset` → `-` mutants die in 81 of 257 cases each.
Those 81 cases are the entirety of what constrains that argument in this
campaign. This is P9 at argument granularity rather than line granularity: the
line is executed 100,000+ times per scenario and one of its inputs is a
constant throughout. Coverage cannot express that, and the gate's red test
cannot either — perturbing the output moved 389,644 values, which says the unit
is seen, not that every argument is.

**Not escalated, because the harness already answers it.** Recorded so that the
next unit with a parameter constant across all scenarios is not surprised, and
so the campaign's ledger can say which arguments its bit-exact layers bound.

**`vit check -f <file>` scopes its cross-source checks to the FILE, not the
function.** It reported `minval-endpoints` and `array-section-row` against this
translation; both come from `interp1d`/`interp2d` elsewhere in `Functions.f90`,
lines 132-271, and neither intrinsic appears anywhere in
`ColemanTransformInverse`. `--function` only sets the report header — its own
help text says so. On a 900-line multi-procedure source every finding has to be
re-attributed by hand before it means anything, and a session that trusts the
count will either chase two ghosts or start ignoring the checker. Not fixed
here: it is a VIT change, and X2 says fix it rather than work around it, but it
belongs to a VIT session and not to this unit's cycle. Left as an open item in
STATUS.md.

**Extraction dirties one more file than the RUNBOOK listed.** The scenario's
`Examples/DISCON*.IN` comes back with its `File written using ROSCO version ...
on MM/DD/YY` header line rewritten to today's date, because extraction runs
`vit_sim.py`, whose `write_discon()` regenerates it. Content-free, and it still
blocks `done.py`'s clean-tree predicate. Restored by hand; noted in the RUNBOOK
beside the other extraction leftovers rather than added to
`reset_to_clean.sh`, because that script is about the SOURCE tree and this file
is the gate's input — `gate.py` already owns restoring these.

### What this unit is verified BY

| layer | result | red test |
|---|---|---|
| kernel replay, 63 cases, scenario 27 | 63/63, all `IDENTICAL` (the equality branch, not the tolerance branch) | zero-writing stub → 61/63 `OUT_TOL`; green restored on revert |
| differential harness, 257 cases vs clean Fortran | 0 failed | the mutation score, below |
| mutation | 24/24 behavioural killed, 1.000, 0 declared equivalent (2 nocompile EXCLUDED) | refuses to score unless the baseline is green |
| post-integration harness (wrapper marshalling) | 257 cases, 0 failed | `axTIn`/`axYIn` swapped in the wrapper's CALL → 256/257 red; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | `PitComIPC[0]` scaled by 1.000001 → 389,644 values moved; revert → 0 |

Two of the 24 mutation kills are CRASHES, not case mismatches: `[2] -> [2 + 1]`
and `'2' -> '3'` both write one past the end of the caller's 3-element array.
Honest kills — the harness died — but a value comparison alone would not have
seen them, so the behavioural-by-comparison count is 22 of 24. The same
distinction unit #1 drew about `killed (no compile)`, one layer in.

`vit_mutate.py` now EXCLUDES nocompile mutants from the score rather than
counting them (loop `46a7f4f`), so unit #2's `35/35` and this unit's `24/24` are
not the same measurement. Unit #2's included 2 nocompile kills; this one's does
not.

## 2026-08-11 — Unit #4, Conv2UC: a hot line the gate cannot see, and the
   instrument that had to be built before the unit could close

**The differential harness had no string kind, and 20 of 69 units need one.**
`vit_harness.py` printed `UNOBSERVABLE [character-arg] Str: crosses as char* +
length and IS comparable; the generator has no string kind, so this argument is
held constant` — a diagnostic somebody had already written, whose own text calls
the gap "a gap with a fix, not a property of the bridge" — and then died:
`EmitError: Conv2UC: C parameter 'Str' is not in the mapped signature`. So the
report and the behaviour disagreed. It said *held*; it meant *unrunnable*.

The scale is what made this a feature rather than a note. Parsing the plan's 69
units against their Fortran declarations, **20 take a CHARACTER dummy argument**
— `GetPath`, `GetRoot`, `GetWords`, `FindLine`, the five `ParseInput_*_Opt`, and
`DISCON` itself. P11 (differential harness) and P12 (mutation score at 1.000)
are mandatory for every unit and `done.py` has no branch that waives them, so
none of those 20 had any path to close.

Fixed rather than worked around, which is X2 and is also what unit #1 did with
VIT's descriptor bridge. Loop `9bee569`: a `char[]` kind in which a CHARACTER
dummy's length is an EXTENT — the thing the rules already know how to vary — and
its bytes are an array of one-byte elements, which the case stream already knows
how to carry. 8 tests, none of which need VIT; 414 collected; the pre-existing
failures in this environment are unchanged, verified by diffing the failure set
against a `git stash`ed baseline rather than by counting.

Four things in it are worth carrying:

1. **`len_<x>` had to be RECOGNISED as an extent.** `_EXTENT_1D` matches `n_<x>`
   only, so the length fell through to the scalar branch and was VARIED OVER
   ±1e3 — two implementations reading that many bytes out of a buffer sized by
   a different number. It is now matched, guarded on the C type of `<x>` rather
   than on the name.
2. **The string and its length are ONE call argument.** `_order` has to put the
   length in the stream before the bytes, because the buffer cannot be declared
   until its size is read; that is only expressible if one object owns both.
   Adjacency in the C parameter list is asserted, not assumed — a generator that
   separated them would transpose the call rather than fail to compile.
3. **R6, one type over.** The reference's own CHARACTER literals and their two
   collating neighbours are the discrete analogue of a literal and its ULP
   neighbours, and they are what makes `>= 'a'` distinguishable from `> 'a'`.
   Scoped to the UNIT's body, unlike `literals_from`, which reads the whole
   file: a stray numeric literal is mostly discarded by the range filter, but
   every character is inside every string's domain, so `ROSCO_Helpers.f90`'s
   hundred stray literals would have buried the two `Conv2UC` branches on.
4. **Three SHAPES, not just three lengths** — mixed, word-then-blanks,
   all-blank. `LEN_TRIM` and `LEN` are the same number on a string with no
   trailing blank in it.

REFUSED, for being unmeasured rather than impossible, each with its own reason
in the report: a CHARACTER **array** dummy (element width and element count are
two extents where `char[]` has one), and a `char*` with no `len_` beside it.
`FindLine` and the `Parse*Ary_Opt` family all take a CHARACTER array, so that is
the next thing this will need — recorded now rather than discovered then.

**THE GATE IS BLIND TO A PROCEDURE CALLED 1.3 MILLION TIMES, AND THE REASON IS
NOT DEADNESS.** `Conv2UC` converts 4,558,823 characters across the 27 scenarios.
Two perturbations of the integrated C++ were built and gated and each moved
**0 of 5,252,000 values**: `ch - 32` → `ch - 31`, and the guard forced false so
that no conversion happens at all. Both artifacts carry `replacements: 1`,
`perturbed: true`, `reverted: true`, `revert_verified: true`.

The cause is that every value this unit produces is consumed by an equality test
against another of its own outputs. `FindLine` upper-cases the expected
parameter name at `ROSCO_Helpers.f90:1106` and the file's word at `:1118` with
the same function and compares them; a perturbation lands on both operands, and
equality survives. The no-conversion run shows it survives non-injective damage
too, on the inputs these scenarios carry.

This is the third distinct shape of P9 in four units, and it is the one that
most directly attacks the coverage argument. #1 was a line no scenario reached.
#3 was an argument that is a constant in every scenario. **This is an executed
line — one of the hottest in the tree — whose result is cancelled downstream.**
A hit count says a line ran. It says nothing about whether anything downstream
can tell two implementations apart, and `coverage/line_coverage.json` cannot
express the difference. The `--perturb-to 'if (false && ...)'` form is the
general instrument for asking, because it makes the unit a no-op rather than
testing one guess about what a defect would look like; it is in the RUNBOOK.

**Removing a restatement raised the mutation score from 0.696 to 1.000, and it
is the second measurement of that rule rather than the first.** The Fortran's
loop bound is `LEN_TRIM(Str)`. Transcribed literally that is
`while (len_trim > 0 && Str[len_trim - 1] == ' ') --len_trim;` — six mutable
sites computing a quantity nothing downstream can read, because every character
past `LEN_TRIM` is a blank and a blank is never converted. All six survived, and
FIVE survived as out-of-bounds reads: `Str[len_trim + 1]`, `Str[1 - len_trim]`,
and `len_trim >= 0` reading `Str[-1]`.

Declaring those equivalent would have recorded a blindness in the harness as a
property of the mutants — the exact distinction unit #1 drew. Removing the
restatement left one loop bound, which every case can see, and the score is
1.000 with ZERO declared equivalent.

The generalisation, now with two measurements behind it: **before declaring a
survivor equivalent, ask whether any input can make the quantity it perturbs
change an output.** If not, the site is not equivalent, it is unobservable, and
the fix is to delete the restatement. Unit #1 named a SIZE once; this named a
LOOP BOUND once. The cost is a departure from literal transcription, which is
only admissible with a proof written into the translation — this one has it.

**A kernel PASS *is* bit-identity for a CHARACTER output, and that is not a
contradiction of unit #3.** KGen's generated comparison for `CHARACTER(len)`,
`kv_findline_character_maxparamlength_`, is `IF (var == kgenref_var)` →
IDENTICAL, ELSE → OUT_TOL. No `rmsdiff`, no `kgen_tolerance`, no `IN_TOL`
branch. Unit #3's caveat — an ABSOLUTE RMS against 1e-14 that lets a 1e-11
relative error print PASSED — is a fact about REAL fields. The rule is
therefore: **read the generated comparison for each new element type**, and do
not carry a hedge across types where the instrument is exact.

**VIT declined to build its own kernel red test and said so.**
`NON_DISCRIMINATING — 63 case(s) compared IDENTICAL, and the kernel did not
report a mismatch when the translation was deliberately changed.` Its automatic
red test perturbs a by-value floating-point argument and this unit has none, so
it refused to claim discriminating power it had not measured. That is the tool
doing the right thing, and the line is kept in the evidence rather than argued
away; the manual no-op stub is what answers it (31 of 63 cases OUT_TOL, matching
the 31 of 63 captured strings that actually change).

**Two defects in this campaign's own scripts, both found by a run that should
have worked, both fixed by addition:**

1. `reset_to_clean.sh` restored the SOURCE and left the OBJECT instrumented.
   `vit extract` puts its own pragma-carrying file back when it finishes, so
   `ROSCO_Helpers.f90` returned textually clean, step 1 skipped it (no `_c(`
   wrapper), its mtime never moved, and make had no reason to recompile. Every
   step reported success and the script then exited 1 on its own assertion:
   `kgen symbols in libdiscon.so: 1`. It could detect the contamination and not
   repair it. Step 3c now removes any build object that DEFINES kgen symbols —
   a rule about contents, so it covers whatever the next extraction
   instruments. Red-tested in both directions on the same run: 2 removed and
   the assertion 1 → 0, then 0 removed and still 0.

2. `harness.sh --red-test` was **accepted and silently dropped in pre mode**.
   The pre path returns before the stamping block, so a pre-integration red
   test wrote `failed: 27` with nothing about what was perturbed — the script's
   own header says a red artifact that cannot say what it measured is not
   evidence about the instrument. `_harness_stamp.py` now takes `--pre`.

**NOT method, target.** The string kind, the `--perturb-to 'if (false && ...)'`
habit, and both script fixes are in the RUNBOOK's target layer.

**CANDIDATE METHOD AMENDMENT, flagged and NOT made.** The gate red test as SPEC
frames it asks whether the gate can see a *perturbation*. Three units in, that
has twice given a false negative for a reason the perturbation choice caused
rather than the unit: RUNBOOK's own attempt-1 (a line the scenarios never run)
and, less obviously, unit #4's off-by-one (a perturbation an equality test
cancels). The stronger question is whether the gate can see the unit's
*absence*. Making a unit a no-op is not always as easy as forcing one guard
false, so this is a candidate rather than a rule — but "perturb toward absence,
not toward a different answer" is a sharper statement of what the red test is
for, and it would have got the right answer on unit #4 in one attempt instead of
two. The Driver may raise it; it is not made here.

---

## Unit #5 — ExtController, 2026-08-11: `blocked`, and the first unit with no oracle

### The decision

`ExtController` closes `blocked`, with `blocked_substrate` escalated. It is not
translated. This is the first unit in the campaign that ends without a
translation, and the reasoning is worth stating plainly because "I could not
make the tool work" and "no instrument in this campaign can verify this" look
alike in a report and are not the same claim.

Four units closed before this one, three of them with a gate that could not see
them. Every one still closed, because *something* could see them: `AddToList`
was dead and the differential harness ran the clean Fortran as an oracle;
`Conv2UC` was cancelled downstream and the kernel plus the harness carried it.
The route in both cases was **P7 — the oracle is the original source.**

Here the original source **cannot be run**. That is the whole decision.

### What was measured, in the order it was measured

1. **`ExtControl.f90` is 0 of 28 executable lines in all 27 scenarios.**
   `Ext_Mode` is `0` in all 14 `Examples/*.IN` — one distinct value across every
   input file — `vit_sim.py` never patches it, and `DISCON.F90:90` sits under
   `IF (CntrPar%Ext_Mode > 0 .AND. ...)`.

   The denominator is not decoration. `scripts/coverage.py` stores only lines
   with a **non-zero** hit count, so a file that never ran and a file that was
   never instrumented are **the same empty dictionary**, and four files read as
   empty in the committed artifact. Re-running with the denominator printed is
   what turns "no entry" into `0/28`, twenty-seven times. This is the same
   failure the script's own docstring records about `gcov`'s notes file, one
   level up: there the numerator was wrong, here the numerator is right and
   unfalsifiable without its denominator.

2. **The gate is blind to it, measured against the Release build, twice.**
   Perturbing Record 49 moved 0 of 5,252,000; making the unit a **no-op**
   (first statement → `RETURN`) moved 0 of 5,252,000. Both artifacts record
   `replacements: 1` and `revert_verified: true`. The second is the form the
   RUNBOOK now prescribes — perturb toward absence — and it is what makes the
   claim about the *unit* rather than about the perturbation.

3. **Forcing the oracle to run makes it segfault.** `Ext_Mode = 1` through
   `vit_sim.py`'s own `write_discon(patches=...)`: **exit status −11, signal
   11**. `DLL_FileName` is the literal string `"unused"` in all 14 inputs, no
   external Bladed-style library is shipped anywhere in the tree, and
   `ExtController` never checks `ErrVar%ErrStat` after `LoadDynamicLib` — so
   `dlopen` fails, `ProcAddr(1)` stays `C_NULL_FUNPTR`, the code prints
   *"Library loaded successfully"*, and the next two statements are
   `C_F_PROCPOINTER` on that null and a CALL through it.

   This was measured rather than reasoned about, and the order matters: the
   source was read first and the probe written to test what the source
   predicted. Reasoning alone would have got the same answer here and has been
   wrong before in this campaign.

4. **VIT refuses to generate anything, deterministically and correctly.**
   `TYPE(ErrorVariables)` has four fields and exactly one blocks it:
   `CHARACTER(:), ALLOCATABLE :: ErrMsg`. Not a defect — a refusal with a
   correct message.

### Why the substrate gap was NOT closed here, when unit #1 and unit #4 closed theirs

Unit #1 built the descriptor bridge; unit #4 built the harness's string kind.
Both were the right call and the same argument applies to `CHARACTER(:),
ALLOCATABLE`: **37 of the 69 units take a `TYPE(ErrorVariables)` dummy**, so it
is on the critical path and will have to be built.

The difference is that building it **would not close this unit.** Even with a
bridge there would be nothing to compare the translation against. Spending the
unit's budget on a feature that leaves the unit exactly as unverifiable would
have produced a translation with no evidence — which is the failure mode this
whole method exists to prevent, arrived at by working hard rather than by
cutting corners. The gap is escalated with its blast radius measured, which is
what the Driver needs in order to schedule it against the other 36 units.

`vit analyze-types --fix character` — the remedy VIT's own error message
suggests — is **refused**. It rewrites ROSCO's type definition to a fixed
length, changing `LEN(ErrVar%ErrMsg)`, which sizes `avcMSG`, which is Record 49.
Changing the oracle to make the translation checkable inverts P7.

### Two defects, each recorded before it was fixed (C12)

1. **VIT emitted a five-argument wrapper over a three-argument bridge, in
   silence.** With no view strategy configured, `vit interface` produced
   `CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(ExtDLL_view))` —
   `LocalVar` and `ErrVar` declared in the wrapper's dummy list and absent from
   the call, with no diagnostic anywhere in the output. `LocalVar%iStatus` is
   the guard on the entire initialisation branch, so that bridge could never
   have loaded the library it exists to call. It compiles and it links.

   Same shape as unit #1's dropped ALLOCATABLE attribute, one level up: there an
   attribute went missing, here two whole arguments. The wrong artifact is kept.

   Fixed in VIT (`f8ab74f`), not worked around (X2). `dropped_derived_args`
   already existed and only `test_validate` asked it, so the warning now lives
   in `generate_fortran_wrapper` — every path that ships a wrapper goes through
   it, and a fourth caller added later cannot forget to ask. Additive: no
   generated byte changes, so no artifact already measured against an earlier
   revision is invalidated. Red-tested both directions; 937 tests pass (935 + 2).

   **The habit did not save this; reading the wrapper did.** RUNBOOK's unit #1
   entry says to read the emitted wrapper attribute by attribute whatever the
   tool's current answer is. That is what caught it. A habit is weaker than a
   tool, which is why the tool changed.

2. **A crashed run can reconfigure the gate where no clean-tree check can see
   it.** This probe's first draft restored `Examples/DISCON.IN` in a `finally`,
   and **a `finally` does not run through SIGSEGV**. The run left `Ext_Mode = 1`
   in the gate's own input — and `Examples/DISCON.IN` is **gitignored**
   (`.gitignore:78`), so `git status` stayed clean and `done.py`'s P2 could not
   have caught it. Every gate run afterwards would have measured a silently
   reconfigured controller.

   The general form, and it is not specific to this probe: **a restore that must
   survive a crash belongs in a parent process, not in the process that
   crashes.** `gate.py` snapshots and restores these files and is the reason the
   campaign has not been bitten before; nothing else did.

### On the disposition, and on `done_check.py`'s verdict

`done_check.py` will report `ExtController` INCOMPLETE: P9 (gate) passes on a
vacuous green, and **P11 and P12 FAIL** because `harness/ExtController.json` and
`mutation/ExtController.json` do not exist. **That verdict is correct and is
kept.** The RUNBOOK says a unit that is not COMPLETE is not finished — and this
unit is not finished; it is *blocked*, which is a different claim, and P8 exists
so that the distinction can be recorded instead of halting the run. Manufacturing
a harness artifact for a unit with no bridge and no oracle would have made the
condition green and the campaign's evidence weaker, silently — the same trade
the RUNBOOK refuses when it forbids unsetting `--mutation-glob`.

The absence of those two files is therefore load-bearing evidence, and it is
named as such in `plan.json`'s `observability` rather than left to be noticed.

### NOT method, target

The coverage-denominator gap, the gitignored-input hazard, the `CHARACTER(:)`
refusal and the `ExtDLL_Type` bridge are all in the RUNBOOK's target layer.

**CANDIDATE METHOD AMENDMENT, flagged and NOT made.** SPEC §P8 lists five
dispositions and says each must state the evidence it rests on. Four units of
evidence here suggest a missing distinction *within* `blocked`: this unit is
blocked because **no oracle exists**, which is a permanent property of the
campaign's fixtures until someone adds one, and is categorically different from
blocked-on-a-tool-gap (unit #1, which was answered in two dispatches by building
the feature). A Driver reading `blocked` alone cannot tell which it has, and the
two schedule completely differently — one is engineering work with a known
endpoint, the other is a decision about what the campaign's inputs are for. The
`evidence` field carries the difference today, in prose. Whether the vocabulary
should carry it is the Driver's call; it is not made here.

**A SECOND CANDIDATE, weaker, also flagged.** SPEC §10.3 says "a function no
instrument can turn red is unverified regardless of its recorded disposition."
That is exactly this unit, and the spec has no name for it. `integrated_
unexercised` is close and wrong — it claims translation and verification, and
`AddToList` shows the campaign already uses plain `integrated` for a verified-
but-unexercised unit. The gap is a name for *verified by nothing, and here is
why*. Raised, not resolved.

## Unit #5 — ExtController, second dispatch, 2026-08-11: the blocking claim was wrong, and two substrate gaps are closed

### The decision

The first dispatch closed this unit `blocked` on an argument with two legs.
**One leg is refuted by measurement. The other has been removed by building the
thing it said had to be built.** The disposition stays `blocked` — P11 and P12
still cannot be produced — but for a *third* reason, which is smaller, named,
and ordinary engineering rather than an impossibility.

The first dispatch's own words, from `plan.json`:

> THERE IS NO RUNNABLE ORACLE. […] So P7 — the oracle is the original source —
> has nothing to point at. […] a unit whose reference implementation cannot be
> executed to completion on any input the campaign possesses.

The last clause is the one that is true, and the one that was over-read. It
*possessed* no such input. It does now.

### 1. The oracle runs

`ExtController` crashed because `dlopen` failed, not because the function is
unrunnable. `DLL_FileName` is the literal string `"unused"` in all 14
`Examples/*.IN`; no external Bladed-style library was shipped anywhere in the
tree; `ExtController` does not check `ErrVar%ErrStat` after `LoadDynamicLib`, so
`ProcAddr(1)` stays `C_NULL_FUNPTR` and the CALL two statements later is the
signal. Every step of that chain is a fact about the INPUT.

`fixtures/bladed_stub/discon_stub.c` is 60 lines of C exporting `DISCON` with the
signature `ExtControl.f90`'s own `ABSTRACT INTERFACE` declares.
`probe_ext_mode_1_with_oracle.json`:

```
exit_status: 0    returned_normally: true    discon_in_restored_by_parent: true
```

Three properties are load-bearing and are argued in the fixture's own header:
deterministic (both sides of a differential comparison call it), dependent on its
inputs (a stub writing constants would let a translation that never fills
`ExtDLL%avrSWAP` pass — unit #2's all-zero-window vacuity again), and stateless.

**It is an addition, not a verification-default change, and getting that wrong is
what made this a Driver question.** The first dispatch escalated "construct an
oracle" under SPEC §8.4 on the grounds that it "adds a gate scenario and so
changes the gate's compared count and its baseline set". That is true of a gate
scenario and false of this: the **differential harness does not run scenarios**,
it calls the unit directly. The 27 scenarios keep `Ext_Mode = 0`, no baseline
moves, and `gate/__gate__.json`'s pinned `compared = 5252000` is untouched. P5
permits it outright. The general form is in the RUNBOOK: before escalating "an
oracle must be constructed", ask *which instrument* needs it.

The probe is a new file beside the old one rather than an edit of it. The SIGSEGV
result is still true of the campaign's own inputs and both artifacts stand.

### 2. `CHARACTER(:), ALLOCATABLE` crosses — VIT `a2e2c30`

The first dispatch declined to close this gap with an argument worth quoting,
because it is the argument that has to be rejected:

> The difference is that building it **would not close this unit.** Even with a
> bridge there would be nothing to compare the translation against.

The premise was §1, and §1 is false. With an oracle, the bridge is exactly what
is needed. The gap was already known to block **37 of 69 units**; declining it
cost two dispatches.

A deferred-length CHARACTER has no C type because its LENGTH is part of its
allocation. It crosses as three members — a staging buffer the populator module
owns, the current length, and the writable **capacity** — with the reallocating
assignment done by the reverse copy. The capacity is what makes the field
*writable* rather than merely readable, and without it
`ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)` could not be translated
at all. An over-long assignment is **refused and reported, never truncated**: a
shortened error message is a plausible wrong answer, which is the one kind of
wrong answer a bit-for-bit comparison cannot catch.

`vit analyze-types --fix character` is still refused for the same P7 reason and
is now also unnecessary.

**A defect inside the fix, found by RUNNING the generated code, not reading it.**
The populator's first draft filled the buffer with `buf = src%ErrMsg` — a
whole-variable assignment to a deferred-length ALLOCATABLE, so Fortran 2003
automatic reallocation applies and the buffer comes straight back out at
`LEN(src%ErrMsg)`, discarding the headroom allocated one line above and making
the capacity published to C++ **a lie in the dangerous direction**. Every shape
assertion passed. The round-trip fixture reported `cap=5` where 4101 had been
allocated. That is the whole argument for
`tests/test_deferred_char_roundtrip.py`, which compiles and runs the generated
bridge; red-tested in both directions.

### 3. A generated bridge no compiler could read — VIT `83d25f9` (C12)

Found on the way and **larger than this unit**. `vit test-validate` emitted the
differential bridge's dummy list on one line. Decomposing `ControlParameters`
(214 fields) and `LocalVariables` (168) produced a `SUBROUTINE` statement
**11,747 characters long**; free-form Fortran stops at 132 columns, so gfortran
truncated it and reported **1,153 diagnostics, none of which name the cause**.

**Every unit in this campaign taking either type — most of ROSCO's controllers —
was outside `vit test-validate` entirely, and nothing said so.** P11 and P12 are
mandatory for all of them. The first four units took scalars, arrays and strings
and never touched a derived type, which is the only reason this went unmeasured.

Fixed as a rule about line *contents* rather than a list of emission sites,
because wrapping the dummy list alone left the array copy-in statements at
133–153 columns. Two defects in the fix itself, both caught by testing rather
than reading: an offset-tracking loop that never terminated and **hung the test
suite**, and a test that asserted the wrong property and failed against a correct
implementation. `statemachine_bridge.f90` goes from 1,153 errors to 0.

### 4. What still blocks it, and why the disposition is still `blocked`

`harness/ExtController.postintegration.json` and `mutation/ExtController.json`
still do not exist, so P11 and P12 still FAIL and `done_check.py` is still right.
Manufacturing either remains the trade this campaign refuses. What changed is the
*reason*, and there are three, all named in `plan.json`'s escalation:

* **A.** `vit test-validate` must cross a deferred-length CHARACTER **field** of
  a decomposed type. Same three-member shape as §2, one generator over; same 37
  of 69 units.
* **B.** `harness/vitbridge.py`'s `expand_derived` drops every CHARACTER field —
  its own comment says so — so VIT's bridge would declare parameters the case
  generator does not produce. Unit #4 built the `char[]` kind for CHARACTER
  *arguments*; this is fields.
* **C.** *A judgement, not a feature.* Every generated case must supply a
  **loadable** `DLL_FileName` or the reference crashes, and the SAVE `DLL_Ext`
  means case ORDER matters too. That is pinning a field to a fixture path and
  constraining ordering, in a generator whose stated premise is that narrowing
  the input domain is the blindness it exists to remove. It is defensible — the
  admissible domain really is that narrow, because the original crashes on the
  rest of it, which is itself a finding about upstream ROSCO — but it is a
  decision about what the harness may hold constant and the artifact must report
  it. **Raised, not made.**

Also unmeasured and cheaper: the `TYPE(ExtDLL_Type)` production bridge for
`LoadDynamicLib` and its SAVE state (X1 forbids inlining), and whether
`ALLOCATE(ExtDLL%avrSWAP(max_avr_entries))` is auto-extracted by
`vit/allocate_extract.py`.

### 5. On the first dispatch's candidate method amendment

It proposed splitting `blocked` into blocked-on-a-tool-gap and blocked-because-
no-oracle-exists, on the grounds that the second is "a permanent property of the
campaign's fixtures until someone adds one".

**This dispatch is evidence against that amendment**, and it is worth saying so
because the amendment was raised from this unit. The distinction it draws did
not survive contact: what looked like a permanent property of the fixtures was
60 lines of C. A vocabulary that had let this unit be filed as categorically
different would have made the wrong answer easier to keep. The existing
`evidence` field carried the difference in prose, and prose was reviewable.

The second candidate — a name for *verified by nothing, and here is why* — is not
affected and still stands.

### A defect in this session's own gitignore hygiene

`.gitignore:65` is `*build*`, which silently matched
`fixtures/bladed_stub/build.sh`. The first commit of the fixture added the `.c`
and left the script untracked, so a committed evidence file named a library
nothing committed could produce (K3) — and `git status` was clean throughout.
Same shape as the gitignored `Examples/DISCON.IN` the first dispatch found. After
committing a new fixture, `git check-ignore -v` each file rather than trusting a
clean status.

### NOT method, target

All of it. The oracle-vs-instrument distinction, the 132-column limit, the
deferred-CHARACTER shape and the gitignore hazard are RUNBOOK target-layer
entries. No amendment to the invariant layer is proposed; the one the first
dispatch proposed is argued against above.

## Unit #5, third dispatch — 2026-08-11: `ExtController` is `integrated`

### 1. The blocking claim was wrong for the second time, and for a smaller reason

The second dispatch closed `blocked` on three remaining items and called them
"ordinary engineering with a known endpoint". They were, and they are done. What
that dispatch could not have known is that doing them would surface a defect
larger than the unit — see §3.

The pattern across three dispatches is worth naming because it is not about this
function. Each dispatch's blocker dissolved when someone asked **which instrument
needs this**, rather than **is this unit verifiable**:

* #1 escalated "an oracle must be constructed" to the Driver under SPEC §8.4,
  on the assumption that an oracle meant a gate scenario. The differential
  harness does not run scenarios. It was P5, and 60 lines of C.
* #2 declined to close the `CHARACTER(:)` gap on the grounds that it "would not
  close this unit". The premise was #1's, and #1 was false.
* #3 found that the differential bridge had been discarding outputs for the
  whole campaign. No amount of reasoning about `ExtController` would have shown
  that; only running the harness did.

### 2. Gap C is a JUDGEMENT and it is now made, in a file that can be reviewed

`harness/ranges.toml` did not exist in this campaign. It does now, with four
pins for this unit and a header saying what the file is for: **the only
judgement in the pipeline**, and every entry NARROWS the generated domain, which
is the blindness the generator exists to remove.

The four are `CntrPar_DLL_FileName` (the fixture path), `CntrPar_DLL_ProcName`
(`DISCON`), `LocalVar_iStatus` (0) and `ExtDLL_avrSWAP_n` (−1, meaning
unallocated). **Every one exists because the REFERENCE crashes on the rest of
the domain, not because the translation would**: `ExtController` never checks
`ErrVar%ErrStat` after `LoadDynamicLib` and then calls through
`DLL_Ext%ProcAddr(1)`, which is `C_NULL_FUNPTR` whenever the load failed. That
is a fact about upstream ROSCO and the file records it as one.

What the pins COST is stated in `plan.json`'s `observability` rather than left
to be discovered: the `iStatus /= 0` path is never exercised, two of the three
DLL strings are held at one value each, and `LEN_TRIM(ErrVar%ErrMsg)` is 0 in
every admissible case. The third of those is where two of the four
declared-equivalent mutants come from.

`statevary.constrain` gained `text = "..."` so a pinned string is written as the
string it is. A judgement nobody can read is not one anybody can review.

### 3. C12 — the decomposed bridge was discarding outputs, for the whole campaign

Recorded before it was fixed; the wrong artifact is
`evidence/ExtController/vit_defects/extcontroller_bridge.alloc_input_only.f90`.

`vit test-validate` decomposes a derived-type argument into one bridge parameter
per field. It copied every field IN. It copied back only fixed-size arrays,
nested types and fixed-width CHARACTERs. A **rank-1 ALLOCATABLE** field and a
**SCALAR** field of an INOUT argument were input-only:

* the reference wrote into a Fortran allocation the bridge then deallocated,
  while the translation wrote into the caller's buffer — so the two were never
  compared on that field at all;
* a scalar crossed by VALUE, so the harness compared its own INPUT against the
  translation's answer. On `ExtController` that reported the translation wrong
  on all 163 cases, which is the loud version; a unit whose scalar output
  happens to equal its input would have reported a full green.

`ExtDLL%avrSWAP` is everything `ExtController` produces and `ErrVar%ErrStat` is
where ROSCO puts every error status. Both were invisible.

Fixed in VIT `8c34ceb`. The extent goes BY REFERENCE, because the callee chooses
it — `ExtController` receives the field unallocated and returns 2000 elements —
and a CAPACITY says how much room the caller has, with three states kept
distinguishable because one of them is absence (P6): `n < 0` unallocated,
`n == 0` allocated-empty, `n > 0` allocated. A result past the capacity is
REFUSED and reported, never truncated, which is the same rule the view struct's
reverse copy already used.

**The proof that it matters is the red test**: with the translation replaced by
a no-op the harness fails 163 of 163 and names `ExtDLL.avrSWAP`,
`ExtDLL.n_avrSWAP`, `ErrVar.ErrStat` and `ErrVar.ErrMsg` — the four outputs that
no harness in this campaign could see before this commit.

### 4. Where the blindness is in the INSTRUMENT, change the instrument

The mutation score went 0.758 → 0.904 → 1.000, and each step is a different kind
of move:

* **0.758 → 0.904, by REMOVING restatements.** `accINFILE`'s bound and record 50
  are the same expression, and so are `avcOUTNAME`'s bound and record 51.
  Transcribing each twice leaves a second site computing a quantity nothing
  reads back. Naming each once leaves only the observable site. That is the
  third time in this campaign — unit #1 named a SIZE once, unit #4 a LOOP BOUND,
  this a BUFFER LENGTH — and the test is still the one the RUNBOOK states: *can
  any input make this quantity change an output?*
* **A survivor fixed in the ORACLE, not in the translation and not by
  declaration.** `int aviFAIL = 0;` was write-only: the local is handed to the
  external library and never read again. The stub now reports the INCOMING value
  back in record 44 before overwriting it, and the mutant dies on all 163 cases.
  The fixture had already been extended once for the same reason — the bytes of
  `accINFILE` and `avcOUTNAME` were reaching the library and never being read.
  **A stub that answers without reading its inputs is unit #2's all-zero kernel
  window in another costume.**
* **0.904 → 1.000, by DECLARING four, each applied by hand and re-run first.**
  Two are one fact: `LoadDynamicLib` blank-fills `ErrMsg` through a
  `CHARACTER(*), INTENT(OUT)` dummy, so `LEN_TRIM` is 0 in every admissible case
  and the concatenation's right operand is constant. One is a capacity boundary
  four orders of magnitude out of reach. One is a buffer length nothing reads.
  The distinction this campaign preserves — REMOVED vs DECLARED — is why the
  file has four entries and not eight.

### 5. Two things about the ORIGINAL that had to be measured, not read

Both were measured before any C++ was written, and both are transcribed rather
than corrected, because the original is the oracle (P7).

* **`ExtDLL%avrSWAP(49) = LEN(avcMSG) + 1` is the constant 2.** `avcMSG` is an
  ARRAY of `CHARACTER(KIND=C_CHAR)`, whose ELEMENT length is 1; `LEN` of a
  CHARACTER array is the element length, not the array size. The record's own
  comment calls it "Maximum number of characters in the MESSAGE argument". It is
  not. `evidence/ExtController/len_probe.txt`.
* **`avcMSG = TRANSFER(C_NULL_CHAR, avcMSG)` writes ONE byte.** `TRANSFER` with
  an array MOLD returns an array just large enough for SOURCE, and gfortran
  assigns a nonconforming right-hand side by copying min(size) elements. Bytes
  2..N of an automatic array are left INDETERMINATE.
  `evidence/ExtController/transfer_probe.f90` prints `0 90 90 ...`. The
  translation zero-initialises instead — a stated departure with its proof in
  the file — and the oracle fixture deliberately does NOT fold those bytes into
  its answer, because a stub reading indeterminate memory would make the two
  sides of a differential comparison disagree for a reason about neither
  implementation.

### 6. `--auto-allocate` is unusable here, and that is now the campaign's problem

RAISED, not worked around in silence. Artifact:
`evidence/ExtController/vit_defects/integrate_auto_allocate.wrapper.f90`.

Three defects, each measured on this unit:

1. Its copy-call scan matches **any** `CALL x(..., arg%field, ...)`. Its own
   docstring says "Registry Copy calls"; the implementation says any call. It
   hoisted BOTH `LoadDynamicLib` and the external DLL call into the wrapper,
   which would have called each a second time.
2. `ALLOCATE(ExtDLL%avrSWAP(max_avr_entries))` is classified `local` because the
   size is a local PARAMETER, so it is declined even after (1).
3. Its error handling emits `SetErrStat(...)` and `CHARACTER(ErrMsgLen)` —
   OpenFAST names that exist nowhere in ROSCO. The generated wrapper cannot
   compile on this codebase at all, so this path has never been exercised
   outside OpenFAST.

This unit needs exactly one ALLOCATE and it is in the committed wrapper, one
statement, transcribed verbatim, with all of the above beside it. **That does
not scale**: the next unit that allocates a field of an argument hits the same
wall, and a hand-edited generated wrapper is a thing that drifts. Fix belongs in
VIT (X2).

### 7. Four smaller defects in the loop's own harness, all found by one run

Each was a wrong answer in the safe direction, which is why none of them had
been seen.

1. `parse_struct_members` split on `;` **before** stripping comments, so a
   semicolon inside a `//` comment ended a member and the next real member was
   glued to the comment's tail. VIT's own generated header did exactly that and
   `n_ErrMsg_cap` was reported ABSENT FROM THE STRUCT.
2. A string's `values` are its BYTES, and two stages read them as an
   enumeration. `R2_flag_values` multiplied the case set by a 57-byte stated
   path — 218 cases became 9,635 and 211 MB of case data — and `_random_over`
   replaced the whole parameter with ONE code point drawn from its own contents,
   so the pinned library path became 1024 semicolons and the reference
   `dlopen`ed a file named `";"`.
3. `_struct_fields` decided "the C struct has no member for this extent" from
   whether the extent was FIXED, which is the same answer for the common cases
   and the wrong one as soon as an ALLOCATABLE field's extent is pinned.
4. `json.loads(run.stdout)` assumed the generated program was the only thing
   printing. `ExtController` has five `PRINT *` statements, so a clean run of
   163 cases reported "harness produced no JSON" — and `vit_mutate.py` had the
   same line, where it reported the BASELINE as a crash. Had its baseline check
   not refused to score, every mutant would have scored `killed (crash)`: a
   1.000 that measured nothing.

Two campaign scripts needed the same lesson: `_integration_shim.py` did not
include `vit_types.h` (first unit with a view-struct signature), and
`harness.sh`'s post-integration LIBS went stale the moment integration added the
four view-populator sources — repaired by CONTENTS (every object CMake built for
this target) rather than by a list, the same shape as `reset_to_clean.sh`'s
"remove any object that DEFINES kgen symbols".

### 8. NOT method, target

All of it. The oracle-vs-instrument distinction, the input-only decomposed
bridge, the `ranges.toml` judgement, the two measured facts about the original,
and the `--auto-allocate` refusal are RUNBOOK target-layer entries. No amendment
to the invariant layer is proposed.

One candidate is worth naming for the Driver and is NOT proposed as an
amendment, because the existing rules already reach it: **P3 already says a
green must be able to name what it compared and be able to go red, and this unit
is the case where the RED TEST is what proved the instrument had been discarding
outputs.** The no-op stub did not merely turn the harness red; the LIST OF
FIELDS it named is what showed four of them had never been compared before. A
red test that reports WHICH outputs moved is strictly more informative than one
that reports that some did, and this campaign's harness already does it. Nothing
to amend — worth knowing.

## Unit #6 — GetPath, 2026-08-11: `integrated`, and the gate is blind for a NEW reason

`GetPath` parses the directory prefix off a filename. Two `CHARACTER(*)` dummies,
no callees, one call site. It closes `integrated` on the differential harness and
the mutation score, with a green gate committed beside a FAILED gate red test —
the third unit in six to close that way, and the reason is different from the
other two.

### 1. The gate's blindness here is a CONSUMER property, and it is a fourth shape

Making the unit a NO-OP moved **0 of 5,252,000** values
(`gate/GetPath.redtest-noop.json`, `perturbation.replacements: 1`,
`revert_verified: true`). The line is not dead — `ReadSetParameters.f90:331`
runs 28 times across the 27 scenarios, and so do both of the readers of its
output. The result is *produced and never consumed*:

```fortran
IF (PathIsRelative(CntrPar%PerfFileName)) CntrPar%PerfFileName = TRIM(PriPath)//...
IF (PathIsRelative(CntrPar%OL_Filename))  CntrPar%OL_Filename  = TRIM(PriPath)//...
```

`PerfFileName` is one ABSOLUTE path in all 14 `Examples/*.IN`, so the guard is
false and `PriPath` is unused. `OL_Filename` is the literal `"unused"` in most of
them — relative, so the concatenation DOES happen — but `OL_Mode` is 0 there and
nothing opens the result; in the three scenarios that do open it the path is
absolute again. The one class of scenario that reads `PriPath` throws the answer
away, and the one that would need a filename supplies it whole.

So the campaign now has four distinct shapes of P9, and coverage can express none
of them: unexercised line (#1), argument constant in every scenario (#3),
result cancelled downstream by a symmetric consumer (#4), and result never
consumed because its consumer's guard is false wherever it would matter (#6).
The verification ledger (E5.2) needs the fourth column too; #4 already asked for
the third.

A SECOND gate red test is committed and it is the WEAK one, deliberately: forcing
the `I == 0` branch off also moved 0, and coverage already said clean line 1240
has zero hits in all 27 scenarios. That is RUNBOOK's "attempt 1" reproduced on
purpose, so the record shows which of the two perturbations carries the claim.

### 2. THE KERNEL HAS ONE CASE AND A LOOKUP TABLE PASSES IT

`vit verify` reports `1/1 IDENTICAL`. A stub reading NEITHER input and writing
the literal `/workspace/ROSCO-r2/Examples/` blank-padded ALSO reports
`1/1 IDENTICAL` (`evidence/GetPath/kernel.constant-stub-PASSES.verify_fields.csv`).

This is unit #2's all-zero window in a new costume, and the difference matters
for what to do about it. There the window was widenable and widening it was the
fix. Here `ReadSetParameters.f90:331` executes **once per process**, so there is
no second invocation to widen onto, and every scenario builds the argument as
`os.path.join(this_dir, '<one of 14 DISCON*.IN names>')` — the answer is the same
string in all 27. The kernel is not badly configured; it is structurally
one-case for this unit, and no configuration change reaches it.

Recorded rather than smoothed over: `plan.json`'s `verification` hypothesis was
"kernel replay or direct-call harness". Kernel replay EXISTS and RAN and is worth
almost nothing. The harness is what verifies this unit.

### 3. A KGEN DEFECT, FIXED IN KGEN (X2)

Extraction succeeded and the generated kernel would not compile:

```
CHARACTER(LEN=accinfile_size), DIMENSION(:), ALLOCATABLE :: accinfile
Error: Variable 'accinfile_size' cannot appear in the expression at (1)
```

KGen hoists the call site's enclosing procedure's dummies into the kernel
driver's PROGRAM scope. `ReadControlParameterFileSub` declares
`CHARACTER(accINFILE_size) :: accINFILE(accINFILE_size)` — an AUTOMATIC length:
legal on a dummy, illegal on a local. KGen already had the whole mechanism for
the neighbouring case, `CHARACTER(*)` (commit `c839e1a`, unit #4's era): make the
driver local `CHARACTER(LEN=:), ALLOCATABLE` and carry the element length in the
state file. It keyed on `selector[0] == '*'`, and an automatic length is not
that.

Fixed in KGen, additively: a new `is_automatic_length_char` beside the existing
predicate, and `hoists_as_deferred_length` for the generators that need both.
Three things about the fix are worth carrying:

* **It is deliberately narrow.** It fires only when the length expression names
  an entity THIS SCOPE declares (`typedecl is not None`) as a non-PARAMETER
  variable. `CHARACTER(MaxParamLength)`, which `ROSCO_Helpers` uses everywhere
  and which resolves through a `USE`, is a constant in the driver too and is left
  alone. `get_variable()` is not used for that test: it CREATES the variable when
  the name is absent, which would report every intrinsic in the expression as a
  variable.
* **The read and write sides must decide it identically or the state file
  desynchronises.** The length record is written first and read first; one side
  emitting it without the other shifts every field after it. Only the CALLER
  knows whether it is hoisting an ARGUMENT (becomes a PROGRAM local — defer) or a
  LOCAL (stays in a subroutine — an automatic length is legal, leave it), so the
  flag is passed in rather than recomputed inside `create_read_subr` /
  `create_write_subr`. The `argintype` test qualifies only the automatic half:
  `CHARACTER(*)` is not legal on a local at all, so that half is an argument by
  construction.
* **This was not a usage error.** The pragma was at a call site, the parentblock
  is a subroutine, and the declaration KGen copied is ordinary legal Fortran.
  CLAUDE.md's prior is that a KGen failure is almost never a KGen bug; this is
  the exception it allows for, and the fix went into the KGen repo.

Unlike the `CHARACTER(*)` case this one fails LOUDLY, at the kernel build, so it
cost a diagnosis rather than a wrong answer. **It is on the critical path for
much of what remains**: `ReadControlParameterFileSub` is the enclosing procedure
for the `ParseInput_*`, `ParseAry`, `FindLine` and `GetWords` call sites, so any
unit extracted from there would have hit the same wall.

### 4. THE FIX WAS RED-TESTED IN BOTH DIRECTIONS, AND THE CONTROL FOUND SOMETHING ELSE

Red direction: the pre-fix driver is kept at
`evidence/GetPath/kgen_driver.automatic_char_len.f90` with gfortran's diagnostic.

Green direction: re-extracted `Conv2UC` (`ROSCO_Helpers.f90:1118`, scenario 1),
whose enclosing `FindLine` has `CHARACTER(*)` dummies AND `CHARACTER(MaxParamLength)`
locals — exactly the two shapes the predicate must treat differently. The
generated kernel came back **byte-identical to unit #4's committed copy apart from
its timestamp comment**, and re-verified 62/62 IDENTICAL against the committed
`conv2uc.cpp`.

**62, where unit #4 recorded 63** — and that is NOT this change. Stashing the KGen
patch and re-running the identical command also yields 62. The configured window
is `0:0:1-20,0:0:12000-12020,0:0:23900-23920`, which is 20 + 21 + 21 = **62**;
unit #4's committed statefile list carries one extra, `Conv2UC.0.0.21`, outside
the first range. An extra IDENTICAL case does not weaken that unit's 63/63, so
nothing about #4's conclusion changes — but its count is one more than its own
window specifies, and the likeliest cause is a stale state file from an earlier
run being swept up. Worth knowing before any future run compares case counts
across passes as if they were stable.

### 5. Removing an unobservable restatement raised the score from 0.882 to 1.000 — the FOURTH time

Two survivors, both `<` → `<=`, both in the 0-based form of `char_assign`
(`mutation/GetPath.survivors_0based.json`). On the copy loop the mutant writes
`dst[n]`, which the fill loop immediately overwrites; on the fill loop it writes
`dst[len_dst]`, one byte past the buffer. Neither is a wrong ANSWER, so no value
comparison can see either.

Rewriting both loops 1-BASED — the way the Fortran states the assignment — turns
the same mutant into "leaves a byte unwritten", which the comparison sees. 25 of
25 killed, 0 declared equivalent.

This is unit #1's rule verbatim ("the literal transcription is the observable
one") and the first time it has been RE-MEASURED on a different unit, so it is
now a rule with two independent measurements rather than one. One refinement
worth recording: the mutant COUNT went UP, 17 → 25. The 1-based form has more
mutable sites, and all of them are observable. The thing to optimise is not the
number of sites but whether a site can change an output.

### 6. Two tool defects observed and NOT worked around

**`vit interface` emits a copy-IN for an `INTENT(OUT)` argument.** The shipped
wrapper reads `PathName` — undefined on entry, by definition of INTENT(OUT) —
into the C_CHAR staging array before the call. It does not change this unit's
answer, because the translation writes every one of `len_PathName` bytes. It does
change what the instruments can see: it is exactly why the no-op stub survives 2
of 236 differential cases (a no-op returns the incoming bytes, and twice those
already equalled the answer), and it would mask a translation that failed to
write the tail. Left in the generated wrapper with the finding recorded, rather
than hand-edited: a hand-edit would make the shipped bridge disagree with the
generator for every future unit.

**`vit check -f <file>` attributed both of its findings to other procedures.**
`narrowing-local` on `ExpUCVarName` (clean line 1051, inside `FindLine`) and
`delimiter-set` on `'\/'` (clean line 1284, inside `GetRoot`), reported against a
GetPath translation containing neither. Second sighting; already open in
`STATUS.md`. Re-attributed by hand, as that entry says to.

**`vit integrate` did NOT write `verification: simulation` into `vit.yaml` this
time** — the only keys it added under `GetPath:` were `status` and `cpp_file`.
Recorded because unit #5 found it doing so; whatever the trigger is, it is not
every integration.

### 7. A hazard in this campaign's own scripts, paid for once

`restore_integrated.sh` restores the integrated sources **from HEAD**, and
GetPath's integration was not yet committed — so running the reset/restore pair
after the post-integration measurements silently reverted the wrapper, leaving an
orphaned `getpath.cpp` and a library with no GetPath in it. The script prints
this warning; it was run anyway. Every post-integration artifact taken before
that point was DELETED and re-taken after re-integrating, which is what the
warning tells you to do. The RUNBOOK's existing rule — commit before exercising
the reset/restore cycle — is the rule that covers it, and it is a target-layer
entry, not a method gap.

### 8. NOT method, target

All of it. The consumer-shaped P9, the structurally-one-case kernel, the KGen
predicate, the 1-based loop measurement and the two VIT defects are RUNBOOK
target-layer entries. No amendment to the invariant layer is proposed.

One observation for the Driver, again NOT an amendment: **C6 ("verify against
captured state") was performed, passed, and is worth almost nothing here, and
nothing in the cycle says so.** The unit is not exempt from C6 and should not be
— running it is what produced the constant-stub measurement, which is the
evidence that the kernel is vacuous. The existing rules reach this: X4 says never
take a green at face value on first use, and X4 is what turned C6's green into a
finding. Worth knowing that a mandatory step's whole value can be the red test
that discredits its own green.

### 9. Unit #6 postscript — the done-condition crashed, and the fix went into the loop repo

`scripts/done_check.py GetPath` raised `UnicodeDecodeError` instead of printing a
verdict. `loop/gitrepo.py`'s `file_at` ran `git show` under `text=True`, and
`done.py::_resolves` — the K3/P6 predicate that checks every evidence reference
names a committed artifact — calls it purely for its exit status. It never reads
the bytes. Unit #6 is the first in this campaign to commit a BINARY evidence
artifact (`evidence/GetPath/kernel.GetPath.0.0.1.statefile`, the captured KGen
state), and that was enough to abort the whole verifier.

Two things make this worth a decision entry rather than a one-line fix note.

**The failure mode is the campaign's own subject.** A verifier that CANNOT
REPORT is worse than one that reports FAIL, because a session cannot distinguish
"the unit is not finished" from "the check is broken". Unit #2 was re-dispatched
for setting a disposition nothing had asked the done-condition about; a
done-condition that throws puts a session back in exactly that position while
looking like the unit's fault.

**The obvious workaround would have weakened the evidence.** The quickest way to
a green here is to drop the statefile from `plan.json`'s evidence list — which
removes the one artifact that lets anyone check the constant-stub claim, in
order to satisfy a defect in the instrument reading the list. Fixed in the loop
repo (`74742bc`) instead, additively: `errors="replace"` only changes bytes that
would previously have thrown, and every caller that actually reads the text is
UTF-8 by construction. X2.

Red-tested in both directions on this unit: before, a traceback and no verdict;
after, COMPLETE 13/13 on the same evidence list. The transcript is at
`evidence/GetPath/done_check.txt` and says so.

## Unit #7 — GetRoot, 2026-08-11: `integrated`, and every red test the campaign owned had to be replaced

`GetRoot` parses the root name off a filename. Two `CHARACTER(*)` dummies, no
callees, ONE call site — structurally the twin of unit #6's `GetPath`. It closed
`integrated` on the first dispatch, on the differential harness and the mutation
score, with a green gate committed beside a FAILED gate red test. That much is
unit #6 repeated. Nothing else was.

### 1. THE CALL SITE ALIASES ITS TWO ARGUMENTS, AND THAT INVALIDATES TWO RED TESTS

```fortran
RootName = TRANSFER(avcOUTNAME, RootName)
CALL GetRoot(RootName,RootName)          ! DISCON.F90:67 -- the ONLY call site
```

One variable, passed as both the `INTENT(IN)` and the `INTENT(OUT)` dummy.

**The kernel becomes a mirror.** KGen captures the pre-call value as the input
and the post-call value as the expected output; they are the same storage, so
`strings kernel/GetRoot/GetRoot.0.0.1` prints `vit_sim1vit_sim1`. A stub that
reads nothing and writes nothing scores **62/62 IDENTICAL**
(`evidence/GetRoot/kernel.noop-stub.verify_fields.csv`).

Unit #6 wrote the rule "the no-op says the kernel is alive, the constant stub
says whether being alive buys anything". Read forwards here it says the kernel is
DEAD, which is not what a passing no-op means and would have been the wrong
conclusion. Four stubs, all committed, and the fourth is the one that resolves it:

| stub | verdict |
|---|---|
| the translation | 62/62 `IDENTICAL` |
| no-op | 62/62 `IDENTICAL` |
| right constant | 62/62 `IDENTICAL` |
| **wrong constant** | **62/62 `OUT_TOL`** |

The wrong-constant stub is the liveness test. RUNBOOK now says so, as an
exception scoped to aliased call sites rather than as a replacement of unit #6's
entry.

**The gate's no-op perturbation is degenerate too**, for a second and independent
reason: every scenario's `avcOUTNAME` is `vit_sim<N>`, containing no `'.'`, so
`GetRoot` is the IDENTITY on the whole exercised domain and a no-op is the
CORRECT implementation there. The perturbation that carries the claim makes the
unit return `XXXXXXXX`; it moved **0 of 5,252,000** with `replacements: 1` and
`revert_verified: true`. The no-op run is committed beside it, labelled
degenerate — the same discipline unit #6 used for its deliberately weak
`I == 0` perturbation.

### 2. A FIFTH SHAPE OF P9: consumed, by a live line, outside the instrument

`RootName` has six reader sites and every one builds a FILENAME. Measured:

| reader (clean) | hits, all 27 |
|---|---|
| `ROSCO_IO.f90:1102` `OPEN(unit=UnDb, FILE=TRIM(RootName)//'.RO.dbg')` | **24**, 23 scenarios |
| `ROSCO_IO.f90:1110`, `:1142` (`.RO.dbg2`, `.RO.dbg3`) | 0 |
| `ROSCO_IO.f90:30`, `:373` (`.RO.chkp`) | 0 |
| `ReadSetParameters.f90:360` (`.RO.echo`) | 0 |

`gate.py` compares `baseline_arrays/scenario_N.npz`, which `vit_sim.py` builds
from the arrays crossing the DLL boundary. It never opens a `.RO.dbg`. So the one
live consumer takes the value out of the process through a channel the instrument
does not read: perturbing the unit renames a file.

That is not any of the four shapes already recorded. #1 was an unexecuted line,
#3 a constant argument, #4 a result cancelled by a symmetric consumer, #6 a
result whose consumers' guards are false. Here the consumer runs and the value
matters — to something the gate is not looking at. `coverage/line_coverage.json`
can express none of the five, and this one least of all: line 1102 is hit in
nearly every scenario.

### 3. THE DIFFERENTIAL HARNESS REPORTED GREEN WITHOUT EVER RUNNING THE UNIT'S POINT

First run: `224 checked, 0 failed`. It had not once executed
`RootName = GivenFil(:I-1)` under `INDEX('\/', ...) == 0` — stripping an
extension, which is what the procedure is FOR. Nothing in the verdict said so.
What said so was the mutation score: 0.648, 25 survivors, six of them clustered
in that branch.

Three independent gaps, each fixed in the loop repo (`0e92a72`) and each
generalising past this unit:

1. **The literal miner read SINGLE-character literals only.** A Fortran character
   SET is one multi-character literal, so `'\/'` was invisible and the corpus
   held **no backslash at all** — the generator could not construct a case that
   told the two halves of the set apart. Now every character of every literal
   passed to `INDEX`/`SCAN`/`VERIFY` is mined. Those three intrinsics take a set
   by definition, which is what keeps error messages and format strings out; the
   single-character rule exists to avoid exactly that flooding and the narrow
   exception preserves it. Conv2UC's corpus is unchanged (`a`, `z`), GetWords'
   gains its five delimiters.
2. **The corpus is each literal plus its COLLATING NEIGHBOURS, laid down in
   corpus order.** `'/'` is `'.'+1`. So every mixed string containing a dot had a
   separator immediately after it, and the branch that requires a NON-separator
   there was unreachable by any seed. *The rule that makes the corpus relevant is
   the rule that blinded it.* Fixed by planting a corpus character at an interior
   position of an ordinary string, by planting PAIRS of the reference's own
   literals adjacently, and by a blank-tail variant that makes `LEN_TRIM` land on
   the plant.
3. **The length ladder `{1, N, N+5}` had no 2.** Length 1 is degenerate — the
   first character is also the last — and at 4 both sides of an `I == 1` test are
   false together. `IF ( I == 1 )` → `IF ( I == 2 )` survived 726 cases on
   `{1,4,9}` and dies on `{1,2,4,9}`.

224 / 0.648 → 726 / 0.968. All three are additions appended after the existing
draws, so the rng sequence the other shapes consume is unchanged.

**The general lesson, and it is about how to read a harness.** A green
`checked N failed 0` is a claim about the cases that were generated. Only the
mutation survivors say which cases those were. Three times in this unit the
verdict was green and wrong to trust, and three times the survivor list was what
pointed at the gap.

### 4. THE SAME INTRINSIC UNIT #4 REMOVED, REMOVED AGAIN — with a harder proof

Transcribed literally, `LEN_TRIM` scored 0.886 with **six of eight survivors
inside the `len_trim` helper** (`mutation/GetRoot.survivors_len_trim.json`).
That is unit #4's `Conv2UC` measurement reproduced on a different unit and the
same intrinsic, so the rule now has two independent measurements.

The proof here is longer than #4's and is written into the translation, because
the quantity is used three ways:

1. `TRIM(X) == "."` and `X == "."` are the SAME PREDICATE in Fortran — comparison
   blank-pads the shorter operand, so TRIM on the left of a comparison against a
   blank-free literal is a no-op by the language's own rule.
2. The scan looks for `'.'`; the positions between `LEN_TRIM` and `LEN` are
   blanks, and a blank is not a `'.'`, so the wider bound finds the same index.
3. `I < LEN_TRIM` and `I < LEN` differ only at `I == LEN_TRIM < LEN`, where the
   character at `I+1` is a blank, `INDEX('\/', blank)` is 0, and both paths write
   `GivenFil(:I-1)`.

**What decided it was not the count but the KIND of survivor.** Three of the six
(`s[i+1]`, `s[1-i]`, `s[i-2]`) are OUT-OF-BOUNDS READS. A mutant whose behaviour
is undefined cannot honestly be *declared* equivalent — "equivalent" is a claim
about behaviour and it has none. Deleting the site is the only move that is both
true and closes the gap. That is a refinement of the existing rule worth keeping:
when survivors are undefined-behaviour mutants, DECLARE is not available and
REMOVE is the only honest option.

### 5. TWO MUTANTS DECLARED EQUIVALENT, AND THE REASONS ARE COMMITTED

`vit_mutate.py --equivalences` takes a bare list of ids, so
`mutation/GetRoot.equivalences.md` carries the proofs beside
`mutation/GetRoot.equivalences.json` — the same shape as `harness/ranges.toml`
at unit #5. A judgement stated without its basis is one nobody can check.

* `6751970e` `I < len_GivenFil` → `I <= len_GivenFil`: equivalent everywhere it
  is DEFINED; at `I == len_GivenFil` it reads one byte past the argument. Stated
  plainly as a limit of value-comparison mutation testing, with the instrument
  that would kill it named (`-fsanitize=address`).
* `51209c13` the `0` in `RootName = ''`: **dead code in upstream ROSCO**.
  Reaching it needs `GivenFil` to be the single character `'.'`, and the
  procedure's own first statement has already returned on that. Transcribed
  anyway under P7; no input executes the line, so no mutant of it can be seen.

Score 1.000, 60 of 60 behavioural killed, 8 of them by CRASH — so the
killed-by-comparison count is 52 of 60, reported beside the score as unit #2's
entry requires.

### 6. TWO KGEN DEFECTS, BOTH FIXED IN KGEN (X2)

`vit extract` succeeded, the kernel BUILT, it RAN, it printed `62/62 passed` —
and it compared nothing:

```
✗ VERIFICATION FAILED: 62/62 passed
  FAILED: kernel compared 0 output variables — nothing was verified
```

VIT's guard is what caught it. `evidence/GetRoot/kgen_callsite.aliased_args_no_verify.F90`
is the generated file, with an empty `!local verify variables` section.

`kganalyze.update_state_info` promotes a call-site variable to STATE_OUT by
finding its position in the actual-argument list and reading the matching dummy's
INTENT. It used `arglist.items.index(argobj)`, and `Fortran2003.Base.__cmp__`
compares nodes by CONTENT — so two occurrences of one name compare equal and
`list.index` returns 0 for both. Measured in the container rather than read off
the source:

```
Actual_Arg_Spec_List('RootName, RootName')  ->  index(items[1]) == 0
Actual_Arg_Spec_List('A, B')                ->  index(items[1]) == 1
```

Argument 0 is `GivenFil`, INTENT(IN). Fixed by resolving the position by
IDENTITY — `argobj` comes out of the traversal's own lineage, so it IS the node
in the tree — with the value lookup kept as a fallback, so the change can only be
more precise than what it replaces.

**The dangerous version of this defect is not the one that happened.** Here the
kernel compared nothing and something noticed. At a call site with a SECOND
out-argument the kernel would have compared that one, dropped this one, and
reported a clean pass.

Fixing it exposed a second defect one file over. `get_typedecl_subpname` composes
a procedure name out of the declaration's selector, and `RootName` is declared
`CHARACTER(LEN=size(avcoutname))`, so the verify subroutine came out as
`kv_discon_character_size(avcoutname)_` — parentheses in an identifier.
`c839e1a` fixed exactly this on the GENCORE side during unit #4's era; the
VERIFICATION side has the same two lines and never got it, because no unit had
generated a verify subroutine for such a type before. **A fix applied at one of
two sites that share a code shape is a fix the other site escapes** — the same
lesson unit #5 recorded about the 132-column bridge generator, now measured a
second time. The sanitiser moved to `kgutils` so both generators share one
definition.

Red-tested in both directions (KGen `4457cd2`). The GREEN direction found
something too: unit #6's committed `evidence/GetPath/kernel-generated-ReadSetParameters.f90`
**cannot be used as the control**, because it is a POST-`vit verify` file —
`vit verify` reports "Modified 10 files" and rewrites the generated callsite file
(USE-ONLY lists, `verboseLevel` 100 → 1, a NaN branch, `[VIT_FIELD]` prints).
The control that works is a same-stage comparison: stash the patch, re-extract
`GetPath`, and diff the two fresh kernels. They are byte-identical.

### 7. A RED TEST THAT STAYED GREEN BECAUSE THE PERTURBATION NEVER REACHED THE BINARY

The post-integration harness links the campaign's PREBUILT Fortran objects; it
compiles the C++ test and nothing else. Editing the wrapper and re-running
`harness.sh --post-integration` reported `checked 726  failed 0` — which is
indistinguishable from a harness that cannot fail. Rebuilding the controller
between the edit and the run turns the same perturbation red, 596 of 726.

Worth keeping because it is the third instance in this campaign of *an instrument
reporting about itself and being read as reporting about the unit*: the stale
harness artifact at unit #2, the crashed done-condition at unit #6, this.

### 8. A VIT CHECK FIXED, AND A THIRD SIGHTING OF ITS FILE-SCOPING DEFECT

`vit check` reported `delimiter-set` against `'\/'` — this unit's own set, and a
FALSE POSITIVE: the check gathers only string literals from the translation, and
this translation spells the set as `static const char Separators[] = { '\\', '/' }`,
which is the form the campaign's own "name a size once" rule asks for (an array
states its length via `sizeof`; a string literal needs the length written beside
it as a second mutable site, and carries a NUL the Fortran set does not). So the
check fired on a translation for following the campaign's rule. Fixed in VIT
(`87a3847`) by reading braced character arrays as sets; registry self-tests 67
passed.

The two findings that remain are the file-scoping defect's third sighting, and
this time one of them is a NEAR-MISS: `delimiter-set` on `':/'` from
`PathIsRelative`, reported against a unit that genuinely has a delimiter set.
Misattribution is most dangerous when the neighbouring procedure's shape
resembles the unit's. Re-attribute by LINE RANGE, not by plausibility.

### 9. NOT method, target

All of it. The aliased-argument red-test inversion, the fifth P9 shape, the three
harness-corpus gaps, the LEN_TRIM removal, the two KGen defects, the VIT check
fix and the prebuilt-objects trap are all RUNBOOK target-layer entries. No
amendment to the invariant layer is proposed.

Two observations for the Driver, neither an amendment:

**X4 did the work again, and it did it three levels deep.** "Never take a green
result at face value on first use" caught the kernel (a no-op passes), the gate
(a wrong answer moves nothing) and the harness (224 green cases that missed the
unit's purpose). The third is the interesting one: the harness green was not
merely uninformative, it was *actively misleading*, and the thing that exposed it
was P12's mutation score, not any red test. **A red test asks whether the
instrument CAN move; the mutation score asks whether it moves for the right
reasons.** The campaign has now had one unit where those two questions had
different answers.

**Three of this unit's fixes were to instruments, not to the translation**, and
the translation was written in an hour. That ratio is worth watching: at unit #4
it was one, at #5 it was four, here three. It is not obviously a problem — the
fixes are permanent and each closed a blindness that would have silently applied
to later units — but a campaign whose per-unit cost is dominated by instrument
repair is a campaign whose bootstrap phase is not finished, and E1/E2/E3 all
report closed.

---

## Unit #8 — GetWords, 2026-08-11: `integrated`, and the first unit the gate can see

`GetWords(Line, Words, NumWords)` splits a line of text into words on a
seven-character separator set. Its one hot call site is inside `FindLine`, which
is how every `ParseInput_*` and `ParseAry` in `ROSCO_Helpers` reads a parameter
out of a `DISCON*.IN` file — 1,333,130 calls across the 27 scenarios.

**Disposition: `integrated`.** Every layer green, every layer red-tested, and
this is the first unit where none of the five red tests failed.

| layer | result | red-tested |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 62/62 `IDENTICAL` on `words` | no-op → 62/62 `OUT_TOL`; constant → 1/62; `Words(1)` corrupted → 62/62 `OUT_TOL` |
| differential harness vs clean Fortran | 1370 checked, 0 failed | no-op stub → 1343/1370 failed, naming `Words` |
| mutation score | 57/57 behavioural killed, 1.000, 1 declared equivalent | refuses to score unless the baseline is green |
| post-integration harness (wrapper only) | 1370 checked, 0 failed | copy-back stops one element short → 1323/1370 failed |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **RED TEST PASSED — 1,857,893 of 5,252,000 moved** |

### 1. The gate is not blind here, and the reason is the mirror of unit #4's

Five of the seven units before this one were invisible to the gate. This one is
not, and the difference is a property of the CONSUMER, exactly as the blindness
always was. `FindLine` upper-cases `Words(WordInd)` and compares it against the
parameter name the caller asked for. **That comparison is asymmetric in this
unit's output** — one operand is the word `GetWords` produced, the other comes
from the caller — which is precisely what `Conv2UC` lacked: there both operands
went through the same function, so any perturbation landed on both sides and
equality survived 1.3 million times.

So the campaign now has a positive case to set beside its five negative ones, and
it is the same test that decides all six: **follow the output to its consumers
and ask whether anything the gate reads depends on it asymmetrically.**

Stated beside the green, because a passing red test is not a claim about every
argument: **both extents are constant in all 27 scenarios.** `len_Line` is
`MaxLineLength = 2048` and `len_Words` is `MaxParamLength = 200` at every call
site; `NumWords` takes two values. The kernel inherits all three from the
simulation. A defect that only appears at another width — truncating a word wider
than an element, say — is outside both bit-exact layers, and only the
differential harness varies them. That is unit #3's constant-argument shape at
EXTENT granularity.

### 2. The unit had no differential harness at all, and P11/P12 are mandatory

`Words` is `CHARACTER(*), INTENT(OUT) :: Words(NumWords)` — an array of
fixed-width strings, **two extents where the generator's `char[]` had one** — and
it was refused. The refusal was recorded as a known gap at unit #4 and it fell
due here: the refused argument is this unit's ONLY output, so the harness would
have supplied and compared nothing at all, and `NumWords` — the extent of a
buffer both sides WRITE THROUGH — would have been varied over ±1e3 as a free
integer.

Built, not routed around (loop `6d13949`). Four files, all additive; rank ≥ 2
stays refused with the reason the refusal always had. Two things worth carrying:

1. **The element count is an ORDINARY dummy of the original.** Nothing in the
   emitted C signature says it sizes anything — it has to be read off the Fortran
   declaration, and VIT's parser upper-cases the dimension text while
   `build_c_params` keeps the argument's declared spelling, so a literal match
   found nothing and looked exactly like a declaration naming no parameter.
2. **`vit interface` handled this shape while the harness refused it.** Two
   generators over the same declaration, disagreeing, and only running both
   showed it. That is the same shape as unit #1's `bridge_feasible` lesson and
   unit #5's 132-column bridge: a capability measured on one generator says
   nothing about the other.

### 3. Three corpus blind spots, all three named by a surviving mutant

The score went 0.889 → 0.905 → 0.921 → 0.983 → 1.000, and **every step was forced
by a specific survivor.** None was found by reading the generator. This is the
second unit in a row where that is true, so it is a pattern rather than an
anecdote: *a harness green is a claim about the cases that were generated, and
the survivors are the only thing that says which cases those were.*

1. **The apostrophe.** The unit's set is `' ,!;''"'//Tab`. In Fortran `''` inside
   a `'...'` literal is ONE apostrophe; the miner's `'([^']*)'` read it as two
   literals and lost the character. A corpus for a word splitter that contained
   no apostrophe.
2. **The tab.** The seventh separator is `CHAR(9)`, because a tab cannot be
   written as a source literal at all — so the miner's blind spot was exactly the
   class of character that most needs mining. And `char_corpus` filtered
   `32 <= v <= 126`, which would have discarded it even if mined. The rule that
   filter was reaching for is *do not INVENT unprintable characters*, not *the
   reference cannot have meant one*: a character the reference NAMES is now
   admitted, and only its neighbours must be printable. NUL stays out at both
   ends.
3. **The leading blank.** The string shapes were mixed / word-then-blanks /
   all-blank. `padded` made the END of a string interesting and left the START
   pinned to a word, so **every predicate about where a word BEGINS had one
   answer in every case** — and `Ch = Ch + 1` → `Ch + 2` only matters when the
   character after a separator begins a word. `leading` is `padded`'s mirror, and
   ROSCO's own input files are full of it.

None of the three moves an earlier unit: `Conv2UC`, `GetPath` and `GetRoot` name
only printable characters, none of their set literals contains a doubled quote,
and `leading` is appended after the existing draws. Verified by recomputing their
corpora — 22, 26, 26, unchanged.

### 4. A mutation score that flaps on memory the program does not own

The last survivor was `len_Line - Ch` → `len_Line + Ch`, and **three IDENTICAL
runs over the identical translation and the identical 1370 cases scored 0.983,
1.000, 0.983.** The mutant asks `scan_first` to search a superset of the correct
region whose extra bytes lie entirely past the end of the caller's buffer, so
whether it "dies" depends on the heap.

Two consequences, and the second is the campaign's, not this unit's.

**The declaration is proved exhaustively rather than argued.** A probe enumerates
every `Line` over {blank, non-separator, separator} at every length 1..7 and
every reachable `Ch`, with the bytes after the buffer set to separators so the
out-of-bounds region is maximally able to disagree: `same 19695,
differ-only-past-the-end 4908, differ-IN-BOUNDS 0`. That is a measurement, and it
is what the equivalence rests on.

**A declaration was a statement about ONE RUN, and now it is a statement about
the mutant.** `vit_mutate.py` drew its equivalence set from the mutants that
SURVIVED, so a declared mutant that happened to die was silently counted an
ordinary kill and the declaration did nothing. On this unit the run that reads
1.000 without the declaration is the run that measured LEAST. Fixed in the loop
repo (`5b40e1c`): the declaration applies either way, and a declared mutant that
was nonetheless killed is printed and recorded in `declared_but_killed` — because
a false equivalence is how a real defect gets excused and it must not hide.

### 5. Four out-of-bounds-only survivors were deleted rather than declared

Unit #7's rule: *a mutant whose behaviour is undefined cannot honestly be
declared equivalent; it can only be deleted along with the site that admits it.*
Written as `for (int i = 1; i <= len; ++i) if (s[i - 1] != ' ')`, the blank-line
guard admitted four survivors — `s[i+1]`, `s[1-i]`, `s[i-2]` and the bound. Three
are observable only through a read outside the buffer. `std::all_of` states the
same predicate with no index arithmetic to admit them.

The fourth is genuinely equivalent and the proof is a property of the REFERENCE:
`i < len` differs from `i <= len` only on a `Line` whose sole non-blank is the
LAST character, and on such a line the parse below emits nothing anyway — a word
is written only when a separator FOLLOWS it. Both answers give an all-blank
`Words`. **The guard is an optimisation of a path the loop already computes
correctly, so only a wrongly-TRUE guard can change the output at all.** That is
why its mutants are hard to observe, and it is worth writing down because it says
which of them could ever be.

The `LEN_TRIM` removal itself is now the FIFTH time this campaign has removed a
restatement of a quantity nothing downstream reads (#1 a size, #4 a loop bound,
#6 and #7 this same intrinsic).

### 6. The revision stamp mis-attributed its own output

`vit-dev` has no git, so both loop scripts fell through to `.loop_rev` — a
hand-written, GITIGNORED pin. **Unit #7's committed harness and mutation
artifacts stamp `99b57ab-pinned` while the tree was at `0e92a72`, the commit that
unit's OWN corpus fix went in as.** The artifact names the generator that could
not yet reach `GetRoot`'s principal branch. `.vit_rev` was stale too: every
artifact since unit #5 says `8c34ceb-pinned` against a checkout at `87a3847`.

`.git/HEAD` and the ref file it names are plain text and need no binary, so this
was fixable rather than workaround-able (X2). Both scripts read it before the
pin now and report `-nogit`, which cannot see a dirty tree and does not claim to.

**And the campaign's own `scripts/_harness_stamp.py` was a third site with the
same logic that OVERWROTE the correct read with the stale pin** — the green
pre-integration run stamped `57c6fe3-nogit` and the red-test run, passing through
that script, came back `6d13949-pinned`. A measured value clobbered by a claimed
one. It now fills a MISSING key only, and runs git first (it runs on the Mac,
where git exists). Same lesson as unit #5's 132-column bridge and unit #7's
KGen sanitiser: **a fix applied at one of N sites sharing a code shape is a fix
the other N-1 escape.**

Unit #7's artifacts are NOT restamped. A verdict that was correct when it was
taken is worth more than a deletion; what it actually names is recorded in
STATUS.md.

### 7. `vit check`'s file-scoping defect, fourth sighting

Two findings against a 190-line translation, both from other procedures:
`narrowing-local` on `ExpUCVarName` (clean line 1051, inside `ParseInput_*`) and
`delimiter-set` on `':/'` (clean line 1236, inside `GetPath`). `GetWords` is
clean 1150–1216. Re-attribute by line range. Fix belongs in VIT.

### 8. NOT method, target

All of it. The CHARACTER-array bridge, the three corpus gaps, the declaration
semantics, the stamp precedence and the gate-visibility test are RUNBOOK
target-layer entries. No amendment to the invariant layer is proposed.

Two observations for the Driver, neither an amendment:

**The instrument-repair ratio did not fall.** Unit #4 spent one instrument fix,
#5 four, #7 three, and this one five — the harness's CHARACTER-array kind, three
corpus gaps and the declaration semantics — plus a campaign-script fix. The
previous entry flagged this as worth watching, and eight units in it has not
converged. What HAS changed is the character of the repairs: #4 and #5 were
building capability that did not exist, while #7 and #8 are closing blindnesses
in capability that already shipped and reported green. Those are different
problems, and only the second kind casts doubt on artifacts already committed.

**P12 is now doing work no red test does, three units running.** The mutation
score found all three corpus gaps here and all three at unit #7, and in both
cases the harness verdict was a confident green. The RUNBOOK says a green must be
able to go red; these units say something narrower and sharper — *a green must be
able to go red FOR THE RIGHT REASON*, and the survivors are the only instrument
the campaign has that can tell.

## 2026-08-11 — unit #9 `HPFilter`: a blind gate, proved blind by a control

**A red test that reads zero and an instrument that is broken are the same
artifact, and this unit is where that stopped being tolerable.** `HPFilter`'s
gate red test moved 0 of 5,252,000 — twice, once with the unit made a complete
no-op and once with only its return value zeroed. Five earlier units recorded the
same shape and none of them ran a control. Here one was run: GetWords' committed
perturbation, re-applied to the SAME integrated build in the same session, moved
1,857,893 of 5,252,000, which is bit-for-bit the number `gate/GetWords.redtest.json`
already carried. Only with that beside it does "0 moved" mean *this unit is
invisible* rather than *the chain from build to install to simulation to compare
is broken today*.

The cost is one extra 3-minute gate run per blind unit. Cheap next to what it
buys, and it is proposed as a habit for every future red test that comes back
green — recorded in RUNBOOK's target layer.

**The blindness was traced, not asserted, and it has three causes.** Coverage
says the unit runs 892,000 times; all four of its readers run too. Unit #6's
"follow the OUTPUT, not the call" and unit #7's "does the live reader put it
somewhere the gate READS" both answer *yes* here and both are wrong. What is
actually true is that each site is multiplied by something zero:

- `Fl_Kp` is `0.0000` in all 14 `Examples/*.IN` and is never patched by
  `vit_sim.py`, so `Kp_Float` is 0 and `FloatingFeedback` is 0 whatever the two
  hottest call sites return.
- `FA_KI` is `0.00000` in all 14, and the proportional gain at that call site is
  the literal `0.0_DbKi`, so `FA_PitCom` is 0.
- The third site runs only in scenario 4, whose `flp_angle_*` channels are
  identically 0.0 for all 4,000 timesteps — `vit_sim.py`'s own scenario-26
  docstring records why: every flap scenario but 26 has zero rootMOOP.

So the question to ask of a unit the gate cannot see is **what SCALES the value
between the reader and a compared channel**, and the answer is usually a gain in
the input files rather than anything in the source. `grep -h '<gain>' Examples/*.IN
| sort -u` is the whole check.

**The first extraction window was vacuous and was discarded rather than
explained.** On scenario 27 the real translation and a zero stub both scored
14,508/14,508 `IDENTICAL`. Re-extracted on scenario 1, where the zero stub moves
183 rows. Both CSVs are committed. A window that a zero-writing stub passes is
not a weak measurement, it is not a measurement, and keeping the discarded one is
what lets the kept one be read as a claim.

**The VIT defect was fixed in VIT, and recorded before it was fixed (C12).**
`test-validate`'s bridge dropped the OPTIONAL argument's `PRESENT()` flag and a
procedure-scope `USE`, emitting a 9-argument Fortran bridge against a 10-argument
C++ caller and passing `InitialValue` unconditionally to an OPTIONAL dummy. Fixed
at `300da9c`; the two wrong artifacts are kept under
`evidence/HPFilter/vit_defects/`. Without the fix the `PRESENT` branch would have
been outside every instrument — which matters more than usual here, because
`has_InitialValue` is 0 at all four call sites in all 27 scenarios, so the
differential harness is the ONLY thing that ever reaches that branch.

**`C_LOC` onto a flat struct was checked before it was trusted.**
`TYPE(FilterParameters)` crosses as a raw pointer rather than through a view
populator. That is sound only because the type is 46 fixed-size
`REAL(DbKi), DIMENSION(1024)` fields — no ALLOCATABLE, no padding possible — and
the field order was verified name-by-name against `ROSCO_Types.f90` before any
C++ was believed. A reordered struct compiles, links, and silently reads the
wrong array; no layer in this campaign would have caught it, because every layer
uses the same header.

**Two red tests were kept where one would have done, because the weaker one is
informative.** Swapping `DT` and `CornerFreq` in the wrapper fails only 85 of 829
cases — a swap of two INTENT(IN)s is a no-op wherever the corpus draws them
equal. Scaling the marshalled return value fails 527 of 829, the remaining 302
having a reference result of exactly 0.0. Neither is "fails every case", and
saying so is better than picking the perturbation that flatters the harness.

### NOT method, target

All of it. The control-for-a-blind-gate habit, the scales-not-consumes test, the
`C_LOC` field-order check and the vacuous-window discard are RUNBOOK target-layer
entries. No amendment to the invariant layer is proposed.

One observation for the Driver, not an amendment. **The instrument-repair ratio
finally fell to zero.** Units #4/#5/#7/#8 spent one, four, three and five
instrument fixes; this unit spent one, and it was in VIT rather than in the
harness generator, and the mutation score reached 1.000 with no survivor at any
point and no equivalence declared. The generator repairs of units #7 and #8 —
mined character sets, collating neighbours, string shapes, the length ladder —
were paid for once and this unit inherited them working. That is the first
evidence in this campaign that the instrument is converging rather than being
patched per unit.

**And the first dispatch of this unit died of sleeping, not of difficulty.** It
backgrounded a build and then wrote 11 polling loops against a task-output file,
several running to full exhaustion; roughly 100 of its 120 minutes were spent
asleep and the timeout killed it with everything uncommitted. The work it had
already done — kernel, pre-integration harness, both red tests — was still on
disk and this dispatch re-ran none of it. P1 is what saved it: state on disk, not
in context. The RUNBOOK now carries the prohibition in its target layer.

---

## Unit #10 — `Int2LStr` — 2026-08-11

`FUNCTION Int2LStr(Num)` returning `CHARACTER(11)`. Two statements: write the
integer right-justified into an 11-character field, then left-justify it.
`integrated`, first dispatch, done-condition COMPLETE.

### The decision that mattered: ask BOTH generators, before writing any C++

Unit #8 wrote that rule after a CHARACTER ARRAY dummy shipped through
`vit interface` and was refused by the differential harness. It cost this unit
about four minutes to apply and it changed the whole shape of the work.

`vit interface` handles a `CHARACTER(11)` function result cleanly: VIT's
`result_is_out_param` turns it into a trailing `char* Int2LStr_result`, and the
wrapper copies 11 bytes back with the width **compiled into both sides**.
`map_signature`, asked the same question about the same declaration, returned

    Unobservable(name='Int2LStr_result', kind='character-arg',
      detail='crosses as char* but build_c_params emitted no len_Int2LStr_result,
              so nothing states how many bytes either side may read;
              neither supplied nor compared')

and that argument is the unit's ONLY output. Had the harness simply been run
against a translation, it would have failed at emit time with a message about a
case stream — which is what happened later, once, for a different reason — or
worse, generated something. Asking first is what made this a known gap rather
than a symptom.

### Three defects, and the ordering of the fixes is the finding

They are the same defect seen from three sides: **a CHARACTER function RESULT is
a `char*` whose width is not a parameter**, and three different places assumed a
width always is one.

1. `harness/vitbridge.py` — refused it for want of a `len_` that cannot exist.
   Fixed by mapping it to a `char[]` with a SYNTHETIC, PINNED extent, which is
   the shape `_constant_extent` already established for `rootMOOP(3)`: `lo == hi`,
   `generate` holds it, `build_call` never passes it.
2. `harness/emit.py` — ASSERTED that a `char[]`'s length is the NEXT C parameter.
   That assertion is correct and load-bearing for a CHARACTER dummy (it is what
   stops a transposed call compiling), so it was not weakened. A new
   `string_fixed` category was added beside `string`: one CArg standing for ONE C
   parameter, width in the stream, never in the call.
3. `scripts/vit_harness.py` — set `result_ctype` whenever `sig.is_function`.

The third is the one worth carrying. It asked **`is_function`** where the
question is **does the result come back through the return value**. VIT already
answers that, in one predicate, `result_is_out_param`, and the harness had a
second, worse answer of its own. Had the other two fixes landed without it, the
generated test would have emitted `char ret_a = int2lstr(...)` against a C++
wrapper returning `void`, and passed a `&ret_b` the Fortran bridge has no dummy
for — through a C linkage that checks neither. *A reader of two agreeing
generators can disagree with both.*

### `intent="out"` was wrong, and the module already said so

The first mapping gave the result `intent="out"`, which reads correctly and is
not this module's vocabulary: `Signature.inputs` excludes an `out`, so nothing
FILLED the buffer and `write_cases` put 0 elements in a stream whose extents said
11. `_arg_intent` had drawn the line already — *"an OUT struct is still supplied
(allocated) and compared"* — and `inout` is also the stronger choice: the bytes a
translation FAILS to write are the ones the harness put there, and they are
compared. The red test shows it working, the no-op returning the harness's own
fill (`!!!!!`, `/////`, `00000`) rather than anything the unit produced.

### A scalar INTEGER reached no value this generator ever chose

Nine mutants survived a 144-case green — a 0.654 — and the survivors are what
said why, exactly as at units #7 and #8. `Num` spanned **-933 to 891**. Not
because anything declared that range: `_ladder` and `_literal_values` are both
driven off `reals`, so the only thing that has ever set a scalar int in this
apparatus is `rng.randint` over `_bounds`' DEFAULT `±1e3`.

A Fortran default INTEGER is the whole 32-bit domain. `±1e3` is 0.00005% of it,
all of it in the middle — and **every branch this unit has is a branch about the
WIDTH of the number.** At `|Num| < 1000` the width is 1 to 4 in every case ever
generated and the blank run is never shorter than six, so the entire structure
of an `I11` field and of `ADJUSTL` was outside the case set.

The addition is an integer DECADE ladder: `9/10`, `99/100`, … `999999999/10⁹`,
both signs, plus `INT_MAX` and `INT_MIN`. Those are the only places a
width-dependent predicate can change its answer, and no interior value can stand
in for them. `-2147483648` is listed rather than derived — it is the one value
whose magnitude has no positive counterpart, and it is exactly 11 characters
wide, which is the field this unit writes into.

**It is appended last and fires only for a DEFAULTED scalar int**, for the reason
the character ladder states in its own comment: it can only ADD cases, so the
draws every earlier unit was scored on are byte-for-byte unchanged. A parameter
whose range is DECLARED — `harness/ranges.toml`, `types.toml` — is left alone,
because that range is a judgement about the admissible domain and widening past
it would generate exactly the cases the declaration exists to exclude.

**This is not an X3 change.** X3 forbids changing a verification default mid-run,
and the reflex to check was right. But nothing here loosens a default to let
something pass: the default range is untouched, no existing case is altered, and
the effect is strictly more input, strictly more mutants killed. It is the same
move units #7 and #8 made when a mined character set and a missing string shape
turned out to be corpus blind spots, and it is recorded here for the same reason
those were — because *a green is a claim about the cases that were generated*.

### The translation was fixed before the survivors were declared away

0.731 → 0.737 by deleting restatements, which is now the third sighting of unit
#1's rule. The first draft blank-filled the whole buffer, wrote the digits, then
performed `ADJUSTL` as a literal scan-and-shift — **four traversals of one
11-byte buffer, restating its bound three times.** 26 mutable sites.

The count of leading blanks is not an independent quantity: by the `I11` contract
the field is blank at exactly `0 .. first-1`, and `first` is already computed. So
the scan is a restatement, and the initial blank fill is the same restatement one
step earlier — the only positions the shift leaves undefined are the trailing
ones. 26 mutants became 19.

The departure from literal transcription carries its proof in the file, which is
what the RUNBOOK rule requires. Likewise the `'*'`-overflow branch an `I11` field
has in Fortran and this translation does not: `Num` crosses as a 4-byte `int`
whose widest representation is `-2147483648`, exactly 11 characters, so **no input
can overflow the field.** Writing that branch would have added mutable sites no
input can reach, which is the shape that survived at units #1 and #4.

### Five declared equivalent, proved over 4,038,021 inputs

All five are buffer-bound: two on a ternary whose arms agree at `Num == 0`, three
writing or reading index 11 of an 11-byte result. Unit #8's rule is that this is
exactly where reasoning must not be trusted — three identical runs there scored
0.983, 1.000, 0.983, because whether such a mutant dies depends on the heap, and
**the run that reads 1.000 is the one that measured least.**

So `evidence/Int2LStr/mutant_equivalence_probe.cpp` sweeps rather than argues:
every value with `|Num| ≤ 2,000,000`, every value within 1000 of every decade
boundary to 10⁹ both signs, every value within 1000 of `INT_MAX` and `INT_MIN`,
against a guard byte poisoned to a value no correct output contains, counting
disagreements in bytes 0..10 only. `differ-IN-BOUNDS 0` for all five. The domain
is exhaustive over the only structure the function has: its behaviour is a
function of the decimal STRING of `Num`, so it can change only where the digit
count changes, and every such transition is swept whole rather than sampled.

### C6 was run on a kernel that turned out to be a lookup table, and running it is what proved that

One live call site in all 27 scenarios, hit once, in scenario 24. `vit extract`
captures 1 case whatever the invocation window says — unit #6's `GetPath` shape,
and the recipe for it was already written. What the recipe did not anticipate:
**the kernel compares the CALLER's variable.** KGen instruments the assignment
`OL_String = TRIM(OL_String)//' Cable'//TRIM(Int2LStr(I))//' '`, so the compared
field is `ol_string`, and the generated `!local verify variables` names the unit's
own result nowhere at all — which is precisely the check unit #7 said to run
before trusting a kernel.

Four stubs, one more than any unit before. The fourth, `'X'` padding, is kept
even though it FAILS, and it is kept as a warning: it looks like proof the kernel
sees all 11 bytes and it is not, because it survives `TRIM` only by being
non-blank. A translation that padded with a different blank would be invisible.
**A stub that fails does not tell you why it failed.**

### NOT method, target

Almost all of it. The four-stub ladder, the `PRINT`-only-reader grep, and the
integer decade ladder are RUNBOOK target-layer entries. No amendment to the
invariant layer is proposed.

**One observation for the Driver, and it is the counterweight to unit #9's.**
Unit #9 reported the instrument-repair ratio falling to zero and read it as
convergence. This unit spent three repairs, all in the differential path, and the
reason is not regression: it is **the first unit of a new SHAPE**. Every unit
before it was a SUBROUTINE, or a function whose result was a scalar returned by
value. The generator repairs of units #7 and #8 were paid for once and this unit
inherited them working — the corpus, the string shapes, the collating neighbours
all just ran. What was unbuilt was the function-result path, and nothing about
units #1–#9 could have exercised it. So the ratio is not a measure of
convergence per unit; it is a measure of **how many new declaration shapes remain
unmet**, and `plan.json` can be read for that in advance. 20 of the 69 units take
a CHARACTER dummy; the count that would have predicted this unit is *how many
are FUNCTIONs with a non-scalar result*, and nobody has computed it.

### The artifacts stamp `5b40e1c-nogit`, and the code that produced them is `d79f39e`

Read this before believing the revision on any of unit #10's harness or mutation
artifacts. The loop repo's HEAD was `5b40e1c` throughout the measurements, with
the three fixes above sitting uncommitted in the working tree; they were
committed as `d79f39e` only after the suite was shown at parity. The `-nogit`
suffix is the stamp saying exactly this — unit #8 built it to read `.git/HEAD`
directly and to admit that it **cannot see a dirty tree and does not claim to**.

So `5b40e1c-nogit` is honest and it is not the answer to "what code ran". The
answer is `5b40e1c` plus the diff that became `d79f39e`, and nothing but this
paragraph records it. That is the residue of unit #8's finding rather than a
regression of it: a stamp that cannot see a dirty tree is strictly better than
one that reports a stale pin as fact, and it still leaves the campaign relying on
prose for the one case that matters — measurements taken while fixing the
instrument, which is every measurement that finds an instrument defect.

## 2026-08-11 — Unit #11 `LPFilter`: five layers, all five alive

**The gate red test went RED, and that is the whole difference from the six
units before it.** 1,592,059 of 5,252,000 values moved when the returned value
was scaled by 1.000001; 0 moved after the revert. Nothing about the method
changed — the same `gate.py`, the same 27 scenarios, the same bit comparison. The
unit is in a different place in the controller: 18 live call sites, 3,527,912
calls across 23 scenarios, and the busiest one produces `LocalVar%GenSpeedF`,
which the torque and pitch controllers subtract from their reference speeds. No
gain multiplies it away.

So **no same-build control was taken, deliberately**. Unit #9 established that a
gate red test coming back GREEN needs a control, because "the gate cannot see
this unit" and "the chain is broken today" produce the identical artifact. A red
test that goes RED has just exercised that chain end to end. Adding a control run
here would cost three minutes and measure something already measured.

### Two instrument findings, and the order they were found in matters

**1. The differential harness's INPUT DOMAIN was a function of the code under
test.** `map_signature` promotes a scalar to `role="index"` by finding
`FP->lpf1_a1[i]` in the **C++**. The no-op red test has no subscript in it, so
`inst` stayed an ordinary scalar integer; unit #10's integer decade ladder then
drew `INT_MAX` for it and handed that to the REFERENCE, which subscripts a
`DIMENSION(1024)` array with it. Signal 11. `harness.sh` printed *"harness
produced no JSON"* — the same sentence a reference that PRINTs produces (unit
#5) — and wrote no artifact at all.

The consequence is worse than a missing red test. P11's green is only a claim
about a corpus; the red test is what says the harness can move. Here the red test
was the ONE run the instrument could not complete, so the failure mode was
invisible in exactly the case built to expose it.

Fixed by ADDITION in the loop repo (`54d792f`), not worked around (X2). **Every
artifact of this unit stamps `d79f39e-nogit`, which is the commit BEFORE the fix**
— the same residue unit #10 recorded: the `-nogit` stamp reads `.git/HEAD` and
cannot see a dirty tree, and the fix sat uncommitted in the working tree while
the measurements that justify it were taken. `d79f39e` plus the diff that became
`54d792f` is what ran, and nothing but this paragraph records it.
`infer_indexes_fortran`
asks the same question of the REFERENCE — `arg%field(expr)` in the unit's own
Fortran body — and runs AFTER the C++ pass, so it can only fill a gap the
translation left and no already-measured unit's corpus moves. The number that
says it worked is the case count: 1157 before (an `inst` nobody constrained),
**996 after — the green run's own figure** — with 996 of 996 failed.

A second, smaller thing fell out of it and is worth carrying: the existing
body-locating search in `char_literals_from` anchors `FUNCTION` at the start of a
line, so a TYPED function — `REAL(DbKi) FUNCTION LPFilter(...)`, which is how
every filter in ROSCO is declared — never matched it. There the miss is silent
and benign (the character corpus falls back to its base set). The new
`unit_body()` allows the type prefix, because here a silent miss would have
returned the unfixed behaviour while looking fixed.

**2. `vit_mutate.py` mutates the translation IN PLACE, so a killed run leaves the
mutant in the tree.** A run was stopped with `pkill` to correct a comment before
re-running it. It restores the file when it completes; killed, it does not. What
it left was `*inst = *inst + 2;` in `translations/Filters/lpfilter.cpp` — the
file `vit integrate` reads next — in a file whose own header still said
`Status: unverified`, at a path `git status` had been listing as untracked all
along.

It was caught by accident: the harness was re-run for an unrelated reason and
reported `checked 996  failed 996` where the identical command had passed ten
minutes earlier. **The instrument that caught it is the one whose red test the
first finding had just repaired.**

NOT fixed in the loop repo, and the reason is stated rather than assumed: the
fix is not obviously "restore on SIGTERM" — a mutation run that dies of anything
(OOM, a killed container, a full disk) has the same problem, and a handler that
restores on the signals it thought of is the same shape of partial fix this
campaign has recorded twice (the 132-column bridge, the two `get_typedecl_subpname`
sites). The RUNBOOK entry requires the check that works regardless of how the run
died: save a copy before, diff after. Recorded in `TOOL_GAPS` terms here rather
than patched under time pressure.

### A red test that stayed green, kept because it should have

The campaign's standard post-integration perturbation — swap two INTENT(IN)
arguments in the wrapper — failed **0 of 996** here. That is correct, not broken:
`DT` and `CornerFreq` are read only inside the initialisation branch and only as
the product `CornerFreq*DT`, which is symmetric. The artifact is kept as
`evidence/LPFilter/harness.postintegration.redtest.args-swapped-EQUIVALENT.json`
beside the real red test (the marshalled result scaled by 1.000001, 656 of 996).

The transferable part: a red test that stays green is either a broken instrument
or an equivalent perturbation, and the two are distinguished by READING THE UNIT,
not by re-running. HPFilter's identical swap failed 85 of 829 because `K = 2.0/DT`
reads `DT` alone.

### NOT method, target

The index-role fix, the mutation-run rule and the equivalent-perturbation note
are all RUNBOOK target-layer entries. No amendment to the invariant layer is
proposed.

**One observation for the Driver.** Unit #10 reported that the instrument-repair
count tracks *new declaration shapes*, not convergence, and predicted it from
`plan.json`. This unit is evidence for that reading: its declaration shape is
`HPFilter`'s exactly — same file, same nine parameters, same derived type, same
OPTIONAL — and it inherited that path working, needing no signature-level repair
at all. Both of its findings came from somewhere else entirely: one from a RED
TEST rather than from the unit, and one from an interrupted tool. Neither is
predictable from a declaration, and neither would have appeared in a run that
skipped a red test on the grounds that the green looked convincing.

## 2026-08-11 — Unit #11 `LPFilter`, second dispatch: a claim without its artifact

The first dispatch of this unit ran all five layers, wrote everything above, and
then ended without closing: `done_check.py` reported `P2:dirty_tree`,
`P7:no_state_commit`, and — the interesting one —
`P5:unresolved_evidence:evidence/LPFilter/kernel.hardcoded-cornerfreq-stub.verify_fields.csv`.

**`plan.json` and the evidence README both described that measurement in prose
and the file did not exist.** What did exist, committed, was
`lpfilter.hardcoded-cornerfreq-stub.cpp` — the stub itself. That is the trap and
it is worth naming: *a stub committed as evidence is the INPUT to a measurement,
not the measurement.* The directory looked complete because the hard-to-write
part of the artifact was there. This is precisely the shape K3 exists for, and
K3 caught it — the first time in this campaign that a done-condition predicate,
rather than a person, found a verification claim resting on nothing.

**Run rather than retract**, because the claim was true and the number is worth
having. The stub scores **14,508 of 14,508 `IDENTICAL`** — the kernel cannot
constrain `CornerFreq` at all.

And running it made the finding SHARPER than the prose had it. The prose said
what unit #3 and unit #9 said: the argument has one distinct value across all 14
`Examples/*.IN`, so the kernel cannot tell a translation that reads it from one
that writes the literal. True, and not the whole of it. `CornerFreq` is read
**only inside the initialisation branch**, and exactly **1 of the 62 captured
cases has `istatus == 0`**. So 61 of the 62 cases do not read the argument at
all — the pass is 61 parts blindness and 1 part coincidence, where a
constant-argument blindness is 62 parts coincidence. Two different weaknesses
with the same green, and only the CSV distinguishes them. That distinction is
now in `plan.json`'s `verification` block instead of a paragraph asserting a
sameness that was not there.

**Method or target?** Target — the campaign already has the rule (K3), and the
rule worked. What belongs in the RUNBOOK's target layer is the recipe for
re-running a kernel verify AFTER integration without resetting the source tree,
which is what made "run it" cheap enough to prefer over "delete the claim", plus
the `vit.yaml` hazard that comes with it: `vit verify` rewrote `vit.yaml`,
stripping every provenance comment (already recorded) AND adding a
`translations.LPFilter` record — `cases_passed: 62`, `red_test: demonstrated` —
describing **the stub's run**. Committing that would have left a machine-readable
claim, in the config the next unit reads, that the shipped translation was
verified by a run of something else. Restored with `git checkout -- vit.yaml`.

No amendment to the invariant layer is proposed.

## 2026-08-11 — Unit #13 `NotchFilter`: `integrated`, five live layers, and a
## survivor that was a corpus gap rather than an equivalent mutant

`REAL(DbKi) FUNCTION NotchFilter(InputSignal, DT, omega, betaNum, betaDen, FP,
iStatus, reset, inst, InitialValue)`, `Filters.f90`. Same shape as unit #11 —
`TYPE(FilterParameters)` crossing as `C_LOC(FP)`, an `inst` index the caller
sees advance, an OPTIONAL `InitialValue` — so the bridge was not the interesting
part. Five layers, all five alive; the second unit here of which that is true.

    kernel     62/62, 14,508 field rows all IDENTICAL, scenario 7, clean
               Filters.f90:356. Window exactly the configured 62 indices, no
               strays. `genspeedf` has 62 DISTINCT non-zero references, so the
               window is not vacuous. Zero stub 0/62 (375 rows move, all this
               unit's outputs). Hardcoded-arguments stub PASSES 14,508/14,508.
    harness    2652 checked, 0 failed, 0 inadmissible, vs the clean Fortran.
               No-op red test 2652 of 2652.
    mutation   126 of 126 behavioural, 1.000, 0 declared equivalent.
    post-int   2652 checked, 0 failed; betaNum/betaDen swapped at the bridge
               call fails 980 of 2652.
    gate       5,252,000 / 351 channels, 0 mismatched. RED 551,278, 0 after
               revert — its own figure, matching no other committed redtest.

### The decision this unit actually turned on

The mutation score came in at **0.968** with four survivors, all
`assoc_reorder`: `2.0 * (x*x)` regrouped as `(2.0*x) * x`. `min_mutation_score`
is 1.0 and the RUNBOOK forbids unsetting the flag, so there were exactly two
honest routes — kill them, or declare them equivalent with a reason.

**Declaring would have been false, and proving that took three probes** (unit
#8's rule: prove a survivor exhaustively rather than reasoning about it).

1. In isolation the two groupings differ on 130,696 of 20,000,000 full-range
   draws and **0 of 20,000,000 inside ±1e3**. Multiplying by two is exact and
   rounding is scale-invariant under powers of two — in the NORMAL range. Once
   `x*x` is subnormal the grid is absolute and the invariance fails.
2. Through the whole coefficient, over the **reachable** inputs. The first pass
   of this probe drew `K` directly and found witnesses at `K = 7.4e-313` —
   fiction, because `K = 2.0/DT` and `|K|` below `2/DBL_MAX` is unreachable from
   any finite timestep. Redrawing `DT` and deriving `K` gives a real witness at
   `DT = 2.44e204, omega = 1.34e-154`. **A local's range is not an input
   domain**, and getting that backwards would have inverted the disposition.
3. Which magnitudes: at exponent −535, 126 of 256 mantissas differ, end to end.
   Round decades `1e-155`, `1e-161` and `sqrt(DBL_MIN)` do not; `1e-156`,
   `1e-158`, `1e-160` do.

So the mutants were killable and the corpus could not reach them — the same
blind spot unit #10 found for scalar integers, one type over: `harness/generate.py`
had **never drawn a real outside ±1e3**, because `_ladder` and `_literal_values`
are both driven off `_bounds`, whose default is ±1e3, and a REAL(8) spans 1e±308.

### Two additions to `harness/generate.py`, and the second is the finding

`_real_magnitude_ladder` — the thresholds where a product first goes subnormal,
first flushes to zero and first overflows, plus a decade inside each region,
across defaulted scalar reals. The obvious fix. **It bought nothing**: 1380 →
2172 cases, score still 0.968, same four survivors.

The reason is worth more than the fix. A rung puts `omega` at 1e-158 and leaves
`K` at the ±1e3 default, and the coefficient subtracts the two — so a 2⁻¹⁰⁷⁴
difference in the first term is annihilated before it can reach an output. **A
ladder that varies one parameter at a time is blind to any defect the other
parameters can dominate.** The second addition runs each rung again with every
other defaulted scalar real pinned to an isolating `0.0` or `1e300` (so a
reciprocal like `2.0/DT` underflows): 2652 cases, 126 of 126, **1.000**.

Both are appended last, both draw no random numbers when they do not fire, and
both are gated on a DEFAULTED scalar real with no role — so a declared range in
`harness/ranges.toml` still means what it says, and `role is None` keeps them off
anything the reference indexes with, which is the shape that segfaulted the
reference when unit #10's integer ladder first fired. Loop repo test suite: 422
passed, 2 skipped, before and after.

**Not an X3 change.** No default is loosened, no existing case is altered, and
the effect is strictly more input and strictly more mutants killed — the same
argument unit #10 recorded. What DOES change for any earlier unit with a
defaulted scalar real is its case COUNT, and the `R_random` tail after the R6
block now draws from a further-advanced RNG. That was already true of the
integer ladder; it is stated here because no entry had said it.

### For the Driver

Two candidates, neither acted on here.

1. **`vit.yaml`'s `translations:` block is now one unit behind the tree.** Unit
   #12 correctly reverted `vit.yaml` wholesale after a STUB verify wrote a false
   record into it, and `NonDecreasing` went with it. Twelve entries, thirteen
   integrated units. Cosmetic — nothing reads it as a disposition — but it is a
   machine-readable file that disagrees with `plan.json`, and the campaign has
   been bitten by exactly that shape before.
2. **The harness prints at most 8 mismatch lines.** Unit #5's rule is that a
   no-op red test must fail every case AND the mismatch list must name every
   output. For a unit with more outputs than fit in 8 lines the list cannot be
   read off the log — here it named 6 of 11, and the five coefficient arrays
   only differ in cases where the init branch fires. The `R4` line does
   enumerate what is compared, and the mutation run is what actually constrains
   them, but the rule as written cannot be satisfied by reading the artifact.

## 2026-08-11 — Unit #14 `NotchFilterSlopes`: `integrated`, and a corpus that
## could not contain a negative zero

`REAL(DbKi) FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP,
iStatus, reset, inst, Moving, InitialValue)`, `Filters.f90`. Unit #13's shape
with two additions — a second OPTIONAL (`Moving`), and a `CornerFreq < 0`
saturation branch — so the bridge was again not the interesting part. Five
layers, all five alive; the third unit here of which that is true.

    kernel     62/62, 14,508 field rows all IDENTICAL, scenario 27, clean
               Filters.f90:405 — the unit's ONLY call site in the controller.
               Window exactly the configured 62 indices, no strays. `rotspeedf`
               has 23 DISTINCT non-zero references and the site passes
               `Moving = .TRUE.`, so all 62 cases enter the coefficient block —
               the weakness #13 recorded (61 of 62 reading none of four
               arguments) is absent here. Zero stub 0/62, 661 rows moving,
               naming all 11 of this unit's outputs. Constant-arguments stub
               PASSES 14,508/14,508.
    harness    4508 checked, 0 failed, 0 inadmissible, vs the clean Fortran.
               No-op red test 4508 of 4508.
    mutation   84 of 84 behavioural, 1.000, 0 declared equivalent.
    post-int   4508 checked, 0 failed; CornerFreq/Damp swapped at the bridge
               CALL (not the interface block — #13's rule) fails 2192 of 4508.
    gate       5,252,000 / 351 channels, 0 mismatched. RED 128,918, 0 after
               revert — its own figure, matching no other committed redtest.

### The decision this unit actually turned on

The mutation score came in at **0.988** with one survivor:

    38bbc289  compare_op  'if (CornerFreq < 0)' -> 'if (CornerFreq <= 0)'

on the saturation guard. The two predicates disagree on exactly one input,
`CornerFreq == 0`, and at `+0.0` both branches produce identical bits — so no
amount of magnitude coverage can separate them. **The witness is a negative
zero, and the corpus could not hold one.**

`harness/generate.py`'s `_real_magnitude_ladder` and `_ladder` both end in
`list(dict.fromkeys(out))`, and `0.0 == -0.0` in Python as in IEEE, so a `-0.0`
written into either list is silently absorbed by the `0.0` already there. Every
rung this generator has gained since unit #7 could be added where it belonged.
This one could not — **the dedup that keeps the ladders honest is what makes the
value unrepresentable** — which is why it went in as a block appended last
rather than as an entry, gated on the same defaulted-scalar-real test the
magnitude ladder uses. Loop repo `5d83048`.

Proved killable rather than declared equivalent, per unit #8's rule
(`evidence/NotchFilterSlopes/negative_zero_survivor_probe.*`): at `+0.0`, 0 of
36 draws differ; at `-0.0`, 36 of 36, because the reference carries the sign
into `2.0*DT*CornerFreq_` and the mutant substitutes `+0.0`, giving
`FP%nfs_b2 = 8000000000000000` against `0000000000000000`. 4468 cases / 0.988
→ **4508 cases / 1.000**, and the mutant dies on exactly **2 of 4508** — the two
new cases that put `CornerFreq` itself at `-0.0` and enter the coefficient
branch. Nothing in the other 4468 reaches it.

### The transferable half: the RETURN VALUE agrees in all 36

This is the part worth more than the fix. `(+0.0) + (-0.0)` is `+0.0` under
round-to-nearest, so the sign difference in `nfs_b2` and `nfs_b0` cancels inside
the filter expression and the value the function RETURNS is bit-identical in
every differing draw. **A differential harness comparing only the unit's result
could not have killed this mutant at any input, at any magnitude, ever.** It is
reachable only because `R4_compare_all_outputs` compares the coefficient
out-parameters as well — a rule that has until now read like thoroughness and
here is the whole of the discrimination.

The same argument applies to the translation itself. Writing the saturation as
the Fortran's branch rather than as `std::fmax(CornerFreq, 0.0)` is what made
the reference and the mutant distinguishable at all: `fmax(-0.0, 0.0)` returns
`+0.0`, which is precisely the mutant's answer. "Transcribe the shape, not the
algebra" bought an observable defect here, not just a matching one.

### An observability shape the coverage data alone does not show

The gate red test moves channels in scenarios 8, 26 and 27 and in none of the
other three that run the unit. `Examples/vit_sim.py` injects a synthetic
per-blade `rootMOOP` in exactly those three; scenarios 6, 16 and 18 execute the
call site **108,000 times between them on a zero input**, where the filter's
output is zero and scaling it by 1.000001 is the identity. The two columns are
put side by side in `evidence/NotchFilterSlopes/scenario_arguments.txt` and they
agree exactly. This is unit #2's finding — a line that runs is not a line that
runs on data — at the level of the GATE rather than of the kernel window, and
the check is one grep of the scenario driver rather than a decoding of captured
state.

### Housekeeping closed here

`vit.yaml`'s `translations:` block listed twelve entries against thirteen
integrated units — `NonDecreasing`'s record was lost when unit #12 correctly
reverted the file wholesale after a stub verify wrote a false `cases_passed`
into it. Unit #13 recorded it as a candidate for the Driver. Added by hand here,
with a comment saying so, alongside this unit's own entry: fourteen and
fourteen, and `plan.json` and `vit.yaml` now name the same set.

No amendment to the invariant layer is proposed.

## 2026-08-12 — Unit #15 `PathIsRelative`: `integrated`, and a red test that was
## seen in a way its own verdict cannot express

`LOGICAL FUNCTION PathIsRelative(GivenFil)` — three `INDEX` tests deciding
whether a file name is absolute. Four live layers, a kernel that is a lookup
table, mutation 1.000, gate green over 5,252,000 values, and two findings that
are not about this unit.

### The gate saw the perturbation by KILLING 24 of 27 scenarios, and reported
### `went_red: false`

Recorded first, fixed second (C12). `gate/PathIsRelative.redtest.json` as first
taken is kept unmodified at
`evidence/PathIsRelative/gate.redtest.as-taken-scenarios-died.json`.

Forcing the unit to answer `.TRUE.` makes `ReadControlParameterFileSub` prefix
`PriPath` onto a `CntrPar%PerfFileName` that is already absolute in all 14
input files. The scenarios do not finish with wrong numbers — they **abort**:

```
At line 879 of file rosco/controller/src/ReadSetParameters.f90 (unit = 10)
Fortran runtime error: Cannot open file
  '/workspace/ROSCO-r2/Examples//workspace/ROSCO-r2/Examples/.../Cp_Ct_Cq.NREL5MW.txt'
```

24 of 27 died, `compared` fell from 5,252,000 to 260,000, `mismatched` stayed 0.
`went_red` is `mismatched > 0 and compared > 0`, so it read **false**, and the
message printed under it — *"either the line is never executed by these
scenarios, or the gate cannot observe it"* — is false of this run. A perturbation
that stops the simulation is the most visible failure available, and the
instrument classified it as invisibility.

`verdict_of` already treats `scenarios_failed` as broken **for an ordinary gate
run**; only the red-test path ignores it. So this is one branch's omission, not
a missing concept.

**Fixed by addition (P5), not by redefinition (X3).** `scripts/gate.py` now also
writes `revert_scenarios_failed` and `perturbation_broke_scenarios` (the
scenarios that failed under the perturbation and ran on the revert), and prints
an accurate message when that list is non-empty. `went_red` and the exit code
are deliberately UNCHANGED: fourteen committed red-test artifacts mean what they
say today, and re-defining a verdict mid-campaign is exactly what X3 forbids. A
reader who needs the distinction reads the new field; nobody's old number moves.

**Nothing already committed is re-read by this**, and that was checked rather
than assumed: every `gate/*.json` in the tree was scanned and
`PathIsRelative.redtest.json` is the first with a non-empty `scenarios_failed`.

**This may belong to the method rather than to this campaign.** The Driver is
the one to decide. E3.2 asks for "gate observed red under a deliberate
perturbation" and P3 for a green that "must be able to go red"; neither says how
a red test should classify a perturbation that prevents the run from producing
output at all. The rule that would generalise is: *a red test must distinguish
"nothing moved" from "the run did not happen", and the second is not evidence of
blindness.* No edit to the invariant layer is made here; this paragraph is the
signal.

### Both directions of wrongness are invisible to the gate, for opposite reasons

The two call sites are one line apart, and coverage of the clean source shows
they take different branches: `PathIsRelative = .TRUE.` has 25 hits against the
function's 56, so one call answers each way in 24 of the 27 scenarios.

* `PerfFileName` is absolute everywhere, so the answer is `.FALSE.`, the branch
  is skipped, and a wrong `.FALSE.` changes nothing. A wrong `.TRUE.` kills the
  run — above.
* `OL_Filename` is the literal `"unused"` wherever `OL_Mode = 0`, so the answer
  is `.TRUE.` and the branch **does** run. The name it builds is read only when
  `OL_Mode > 0`, and every scenario with `OL_Mode > 0` is handed an absolute path
  by `vit_sim.py`. The live `.TRUE.` answer writes a string nothing opens.

Forcing `.FALSE.` moves 0 of 5,252,000 with all 27 scenarios alive
(`replacements: 1`, `revert_verified: true`). Unit #9's rule applied because a
red test came back green: the same-build GetWords control reproduces 1,857,893
of 5,252,000 byte-identically, so the chain is alive and the quiet is the unit's.

This is unit #6's `GetPath` finding measured from the other side. `PriPath` is
invisible *because* this guard is false where it would matter; `PathIsRelative`
is invisible *because* it is only true where nothing reads the result.

### Mutation 1.000 was reached by DELETING sites, not by adding cases

The first form transcribed the reference's `INDEX` literally, as
`index_of(s, len_s, sub, len_sub)` returning the position. It scores **0.938**,
and both survivors are its loop bound `len_s - len_sub + 1`:
`len_s - len_sub` → `len_s + len_sub`, and `+ 1` → `+ 2`. Neither is a wrong
answer. Both run the search **past the end of the buffer** — undefined
behaviour, which cannot honestly be DECLARED equivalent (unit #8's rule about
out-of-bounds survivors), and which no value comparison can see.

The position is also a quantity nothing downstream reads: all three `INDEX`
calls are immediately compared against `0`, so any site computing *which*
position matched is unobservable by construction. That is unit #4's `LEN_TRIM`
finding one intrinsic over, and its remedy is the same — delete the restatement
and write the proof in the file, rather than declare the blindness away.

Rewritten as two predicates — `contains_pair`, indexed on the position of the
SECOND character so the bound is a bare `i <= len_s`, and `contains_char` — the
**same 387 cases** kill **26 of 26**. Both harness runs report 387 cases
(`evidence/PathIsRelative/harness.index_position.json` is the first form's), so
the score difference is the code and not the corpus. This is the first unit since
#4 whose score gap was a transcription choice rather than a generator gap.

The corpus was checked before that conclusion, not after: decoding
`pathisrelative_cases.bin` gives 331 `.TRUE.` against 56 `.FALSE.`, the two
constant stubs fail 331 and 56 respectively, and the reference's own
two-character sets appear 8 times at the start of a string and 4 times at the end
— which is exactly what the two boundary mutants of the predicate form need in
order to die (`evidence/PathIsRelative/corpus_answer_distribution.txt`).

### Re-observed, already recorded

`vit check -f ROSCO_Helpers.f90 --function PathIsRelative` reported two findings
(`narrowing-local` on `ExpUCVarName`, `delimiter-set` on `'!'`), neither of which
exists in this unit's six-line body or its Fortran — both come from other
functions in the same file. Unit #4 recorded that `-f` scopes cross-source checks
to the FILE and `--function` only sets the report header. No action; the entry
already exists in the target layer.

## 2026-08-12 — Unit #16 `ReadAvrSWAP`: `integrated`, and an input surface no
## scenario drives

**The unit is the controller's whole front door — 444,000 calls across all 27
scenarios, every one of them at the single call site `DISCON.F90:81` — and
`vit verify` and `gate.py` between them can constrain fewer than half of what it
writes.** 62 of 62 cases `IDENTICAL` across 14,135 field rows, 5,252,000 of
5,252,000 gate values agreeing, and a stub with **23 of the unit's 43 output
fields simply deleted** passes both. The differential harness fails that same
stub 26,198 of 26,198 and names the fields, which is what makes this a statement
about the simulation rather than about the method.

### What the two bit-exact layers cannot see, and the three reasons

`evidence/ReadAvrSWAP/readavrswap.zero-fed-outputs-deleted-stub.cpp` is the
stub. It drops the entire `IF (CntrPar%Ext_Interface > 0)` block — 18 `Ptfm*`
assignments — plus `VS_MechGenPwr`, `FA_Acc_TT`, `SS_Acc_TT`, `NacIMU_FA_RAcc`
and `FA_Acc_Nac`. Three distinct mechanisms put those 23 out of reach, and they
are not the same finding:

1. **No data at all.** `avrSWAP(14)`, `avrSWAP(54)` and `avrSWAP(1001..1018)`
   are never written by anything in this tree. `rosco/toolbox/control_interface.py`
   allocates `np.zeros(3000)` and writes 24 indices, none of them these; the
   scenarios add six more, none of them these. The extended Bladed interface is
   a real input surface — `Ext_Interface = 1` in all 14 `Examples/DISCON*.IN`,
   and coverage puts the block at 443,972 executions — that a Python driver
   emulating ServoDyn simply does not supply.

2. **Data written and then overwritten.** Scenario 27 injects
   `controller_int.avrSWAP[52]` (tower-top acceleration, for its `TD_Mode = 1`)
   and `[82]` (nacelle IMU, for its `Fl_Mode = 2`) and then calls
   `call_controller`, whose first act is a block of `self.avrSWAP[...] =`
   assignments that includes `except KeyError: self.avrSWAP[82] = 0`. The
   `turbine_state` dict it is handed carries neither key. **Both injections are
   zeroed before the DLL is called**, and the two modes the scenario exists to
   exercise run on zeros. Scenarios 7 and 26 inject the same two indices and
   lose them the same way.

3. **No reader the gate reads.** Unit #7's question, asked of each output. The
   18 `Ptfm*` fields have no consumer in the controller *at all*: outside this
   one assignment they appear only in `ROSCO_IO`'s debug `WRITE`, its
   restart-file `READ`, and the `LocalVarOutData` table — and `gate.py` compares
   `baseline_arrays/*.npz`, built from the arrays crossing the DLL boundary.
   `VS_MechGenPwr` is the same plus `ZeroMQInterface`, and `ZMQ_Mode` is `0` in
   all 14 inputs. So for those 19 fields, fixing (1) would not help.

`FA_Acc_Nac` is the one to keep in view: its readers are live and unconditional
(`Filters.f90:372` and `:395`, a `SecLPFilter` and an `HPFilter` every
timestep). It is invisible **only** because both of its inputs are zero — which
is mechanism (2), not (1) or (3), and is therefore the one a driver fix would
actually repair.

**Not fixed here, deliberately.** Repairing (2) changes what the 27 scenarios
feed the controller; that moves `baseline_arrays` and the compared count, which
is X3 and SPEC §8.4 — the Driver's call, not a unit's. Recorded as a candidate.
Its cost is written into `plan.json`'s `observability` rather than left to be
discovered.

### The kernel said FAILED; `vit verify` said 62/62; both were right

`kernel.exe`'s own summary reads `Total number of verification cases : 62 /
Number of verification-passed cases : 61 / kernel: ReadAvrSWAP: FAILED
verification`, and `verify_fields.csv` — the artifact this campaign commits as
the green — carries one `OUT_TOL` row: case 2, `alreadyinitialized`, computed 1
against reference 0. `vit verify` printed `✓ VERIFICATION PASSED: 62/62 passed`.
Both artifacts are committed, the wrong one first (C12):
`evidence/ReadAvrSWAP/vit_defects/kernel.run-says-FAILED-61-of-62.txt` beside
`evidence/ReadAvrSWAP/kernel.green.vit_verify.stdout.txt`.

The override is VIT's design and it is correct. `run_kernel_verify` runs the
ORIGINAL FORTRAN through the same kernel first and compares field logs; when
they match, the kernel's per-case failures are state-capture artifacts rather
than translation defects. **Measured rather than taken on trust**: the original
`ReadSetParameters.f90` rebuilt into this kernel produces `alreadyinitialized`
= 1 against reference 0 on case 2 and reports 61 of 62 FAILED —
`evidence/ReadAvrSWAP/kernel.original-fortran-replay-FAILS-case2.txt`.

The cause is this campaign's own `vit.yaml`. `kgen.dll_persistence.resets`
inserts `LocalVar%AlreadyInitialized = 0` before every line matching
`CALL ReadAvrSWAP` **in the instrumented source** — that is, between KGen's
input capture and the call — and the generated kernel does not carry the
inserted line. The reference output was therefore produced by a statement the
replay does not execute. It is observable on exactly one case: case 1's captured
input is already 0, and from case 3 on the previous call's reset left it 0. A
`dll_persistence` reset naming a variable the unit itself READS is this shape,
and this unit is the only one in the campaign whose signature reaches the
variable that config names.

**What is missing in VIT is a print, not a verdict.** `run_kernel_verify`
already computes `Note: N state file artifacts detected (Fortran and C++ match
each other)` and a field-coverage line into `result.output`, and `cmd_verify`
never prints `result.output`. So the screen says `62/62` where the instrument
says 61 of 62 plus an override, and nothing on it says an override happened.
Not repaired inside this unit: the fix is print-only and cannot change a
verdict, but it changes what every future run's transcript says, and this unit's
own evidence was taken with the current build. Left as a one-line addition for
the next unit, with the two artifacts above as its red test.

### Transcription notes worth keeping

`REAL(ReKi)` is `C_FLOAT` (`Constants.f90:18`), so `avrSWAP` crosses as
`float*` and **every read in this unit is a float widened to a double** — not a
double read. `R2D` and `D2R` are `REAL(DbKi)` initialised from literals with no
kind suffix, so under `-fdefault-real-8` the literals are themselves kind 8 and
nothing narrows. `REAL(LocalVar%NumBl)` likewise names no kind and is kind 8, so
`(1 / REAL(NumBl))` is a double reciprocal formed BEFORE the multiply — written
`(1.0 / (double)NumBl) * ((BlPitch[0] + BlPitch[1]) + BlPitch[2])`, left-to-right
inside the parentheses, because that is the shape the reference has. `NINT` is
`std::lround`, twice.

`ErrVar%ErrMsg = '...'` on a `CHARACTER(:), ALLOCATABLE` field is a
REALLOCATING assignment: the new length is the literal's 58, not the old one's.
The view carries `n_ErrMsg_cap`; an assignment that does not fit is refused and
reported, never truncated. Coverage of the clean source puts
`IF (LocalVar%AlreadyInitialized == 0)` at **28 hits**, its TRUE branch at 28
and **its ELSE branch — the two statements that write `aviFAIL` and `ErrMsg` —
at zero, in all 27 scenarios**. No scenario loads the library twice, so that
branch is transcribed and unverifiable by any layer here except the harness.

## 2026-08-12 — Unit #16 `ReadAvrSWAP`, second dispatch: a ladder aimed at a
## NAME, and two ladders that never crossed

The first dispatch closed with the mutation score unrun and the tree dirty. The
score, run here, came back **0.874 — 14 of 111 behavioural mutants alive**, the
lowest first-pass score in this campaign. Every one of the fourteen is a corpus
gap, and the corpus had two distinct holes in it, only the first of which is a
variation on something already recorded.

### 1. The corpus CONTAINED the value and the branch was still unreachable

`ReadAvrSWAP` branches four times on `LocalVar%iStatus == 0`, and
`LocalVar_iStatus` **is** a scalar integer parameter of the mapped signature —
so unit #10's integer decade ladder gave it `0`, along with every other decade
boundary and both 32-bit extremes. Every one of those cases was dead. The unit's
first statement is

```fortran
LocalVar%iStatus = NINT(avrSWAP(1))
```

so the value the branch reads is not the parameter the ladder moved; it is
element 1 of an array. `harness/generate.py`'s `_fill_array` returns
`lo + span*(k+1) + jitter` — one ascending ramp across ±1e3 for 3000 elements —
so `avrSWAP(1)` is about **-999 in every case this generator has ever
produced**, and `iStatus == 0` was FALSE in the whole 26,198-case corpus. The
same holds for `NINT(avrSWAP(61))` → `NumBl`, the trip count of the unit's only
loop (about -959), and for `avrSWAP(2)` → `Time`.

This is the tenth corpus blind spot recorded here and the first of this shape.
Unit #10's was a range never reached, #12's an ordering never generated, #13's a
magnitude drowned by its neighbours, #14's a value the dedup absorbed. Here the
generator reached the value, at the right type, in the right parameter — and the
reference had already overwritten the name before the first branch read it.
**A ladder over a name is not a ladder over the quantity when the reference
reassigns that name from somewhere else.**

### 2. Two ladders that never cross cannot reach a branch that needs both

Separate, and it cost as much. Every stage in `generate()` sets ONE parameter
and leaves the rest at base; `flag_variants` crosses only DECLARED flags, and a
scalar integer the reference compares against a literal is not one — the only
flag in this unit's signature is `LocalVar_restart`, which the reference
overwrites unconditionally, so R2's whole "2 values across 1 flag" is dead too.

The seven mutants on the pitch-fault loop need

```
CntrPar%PF_Mode == 1   AND   NINT(avrSWAP(61)) >= 2   AND   NINT(avrSWAP(1)) /= 0
```

**in the same case**. `PF_Mode == 1` was individually reachable (the integer
ladder produces it); the conjunction was not. R2's own header already says the
rule is "additive across flags — the sum of their arities, not the product",
which is a correct statement of what it claims and a precise statement of what
it cannot reach.

### The fix, by addition, and what it is bounded by

`scripts/vit_harness.py` gains `predicate_knobs_from`, which reads the
REFERENCE (P7 — a red-test stub contains no predicate, unit #11's rule) for
every quantity a predicate tests, follows `LHS = ... arr(lit) ...` back to the
element the quantity ENTERS by, and returns `(parameter, element index or None,
values)`. The values are `{0, 1, 2}` — the trip counts at which a `DO K = 1, N`
body runs never, once and twice, and three values at which `== 0`, `== 1` and
`> 0` each take both answers — plus every literal the reference compares the
quantity against, and that literal plus one.

`harness/generate.py` gains an R7 block, appended LAST and silent when no knob
is found, that emits the CROSS PRODUCT of the knobs. Six knobs here
(`Ext_Interface`, `PF_Mode`, `AlreadyInitialized`, and elements 1, 2 and 61 of
`avrSWAP`), 729 combinations, **26,198 → 27,656 cases**. The product is
exponential in the number of knobs, so it is bounded at 4096 combinations and
the bound is REPORTED rather than silent: past it the block covers all PAIRS
instead and its own coverage line says so. No silent caps.

Five earlier units would now fire the detector — `GetWords` (`NumWords`),
`HPFilter`, `LPFilter`, `NotchFilter` (`iStatus`) and `NotchFilterSlopes`
(`iStatus`, `CornerFreq`), one or two knobs each. The block appends, so **every
case those units were scored on is byte-for-byte unchanged** and no committed
artifact moves; a re-run would add cases, not renumber any.

### One survivor is left and it is declared, with the same reason as unit #5's

`e82a5f90`, `compare_op '<=' -> '<'` at
`if (ALREADY_LOADED_LEN <= (int)ErrVar->n_ErrMsg_cap)`. The two forms differ on
exactly one input, `n_ErrMsg_cap == 58`, and the capacity cannot be 58 on either
side of the boundary: the harness sets it to `LEN(ErrMsg) + ALLOC_HEADROOM`
(`harness/emit.py`, 4096) and the integrated build to
`LEN(src%ErrMsg) + VIT_DEFERRED_CHAR_HEADROOM` (`vit_errorvariables_view.f90:63`,
also 4096). **Same site, same reason, same declaration as `ExtController`'s
`11d2c9cc`** — the two units share the `CHARACTER(:), ALLOCATABLE` assignment
idiom and this is the second unit to reach it. Not removed: the guard is what
makes an over-long assignment a refusal rather than a truncation, and a
truncated error message is the one wrong answer a bit comparison cannot catch.
The OTHER mutant on that same line — `negate_cond`, which inverts the whole
guard — is killed by the new corpus, which is the difference between an
unobservable site and an unreached one.

`mutation/ReadAvrSWAP.survivors_before_predicate_knobs.json` is the 0.874 run,
kept: it is the measurement that makes the generator change a finding rather
than a preference.

## 2026-08-12 — Unit #17 Read_OL_Input: the prediction is refuted, and the disposition is `blocked`

### The prediction, and which generator it measured

`plan.json` predicted this unit's signature CANNOT cross, basis
`channels: c_assumed_shape_2d does not cross`. **REFUTED.** `vit interface`
crosses it by allocate-on-return: `double** Channels, int* n_Channels_rows,
int* n_Channels_cols` plus a `vit_free` interface, with `C_NULL_PTR` carrying
NOT-ALLOCATED so P6 survives; `TYPE(ErrorVariables)` crosses as
`C_LOC(ErrVar_view)`. Integrated, rebuilt, gate 5,252,000 / 0 mismatched.

The disagreement is not a contradiction. The conformance matrix's
`bridge`/`compiles` columns measure `test_validate.generate_fortran_bridge` --
the differential harness's Fortran side -- and `vit integrate` ships
`interface_gen`. The RUNBOOK's unit #1 entry already says `bridge_feasible` is
not the answer to "does the signature cross"; this is the first unit where the
two answers actually differ, and the harness side is still `no`. Both columns
are right about the thing each measured. `plan.json`'s `bridge_feasible` for
this unit is updated to `yes` with that basis written in.

### C12 — a green that should have been red, recorded with the wrong artifact

`ErrVar%ErrMsg` is `CHARACTER(:), ALLOCATABLE` and arrives UNALLOCATED at this
unit's only reachable call site. `vit_populate_errorvariables` published
`C_NULL_PTR / n = 0 / cap = 0` for that case. That carries the P6 distinction
correctly and leaves the C++ **with no buffer**, so the reference's own
`ErrVar%ErrMsg = TRIM(OL_InputFileName)//' does not exist'` -- a reallocating
assignment, which ALLOCATES -- had no representation on the C side.

The translation refused the write and said so on stderr six times. The kernel
reported **1/1 passed, 200 of 200 IDENTICAL**, because KGen guards the field
comparison on `IF (ALLOCATED(var%errmsg))` where `var` is the KERNEL's own
value: **an output the translation fails to allocate deletes its own
comparison.** The as-taken artifact is committed at
`evidence/Read_OL_Input/kernel.errmsg-never-written-still-PASSES.verify_fields.csv`
beside the run log naming the refusals.

Fixed in VIT (X2 — a tool we control), by ADDITION: the staging buffer is now
supplied whether or not the field arrived allocated, and NOT-ALLOCATED moves
onto the length using the convention this codebase already had for an
ALLOCATABLE array extent (`n < 0` unallocated, `n == 0` allocated-empty,
`n > 0` allocated). `vit_copy_scalars_to_*` already guarded `n >= 0`, so a
field the C++ does not touch stays unallocated exactly as before. **Not an X3
change:** no default is loosened and no existing comparison is altered; the
only behaviour that moves is a write that was previously REFUSED, and every
such refusal disagreed with the reference. Measured: gate 5,252,000 / 0 after
the change, with ReadAvrSWAP and ExtController in the same library.

### CANDIDATE, NOT MADE — KGen does not round-trip a deferred-length CHARACTER

With the buffer supplied, `errmsg` appears in the field log and reads
`OUT_TOL` against an **empty** reference. The generated reader is
`READ (UNIT = kgen_unit) var%errmsg` into a field it has just DEALLOCATED, with
no length record written anywhere. Measured rather than argued: the ORIGINAL
FORTRAN, rebuilt into this same kernel, fails the same case with the same
message
(`evidence/Read_OL_Input/kernel.original-fortran-replay-FAILS-errmsg-empty-reference.txt`).

X2 says fix it in KGen. It is NOT fixed here because writing a length record
changes the STATE FILE FORMAT for every type carrying such a field --
`ErrorVariables` is one, and `ReadAvrSWAP`'s and `ExtController`'s committed
kernel artifacts both rest on it. That re-takes committed evidence for closed
units, which is X3 and SPEC 8.4: the Driver's call, not this session's.

### The disposition: `blocked`, on three reasons, and the third is the one

Measured, not predicted —

```
bash scripts/harness.sh Read_OL_Input ROSCO_Helpers read_ol_input <file>.f90 --against translation
EmitError: Read_OL_Input: C parameter 'OL_InputFileName' is not in the mapped signature
```

1. `CHARACTER(1024), INTENT(IN)` is a width COMPILED INTO BOTH SIDES, so
   `build_c_params` emits `char*` with no `len_` and `map_signature` refuses
   what it cannot size. Unit #10's CHARACTER-function-result shape, one
   declaration over, and closable the same way.
2. `Channels` maps as an INPUT `real[]` on the `+/-1e3` default where the
   shipping bridge makes it allocate-on-return. The harness's own report says
   so: `UNCONSTRAINED: 1 varied parameter(s) ... Channels`.
3. **The unit's principal input is not in its signature.** Its behaviour is a
   function of a FILE ON DISK; the signature carries the file's NAME. No
   generator in this campaign has a notion of a file-valued input, so with (1)
   and (2) both fixed the corpus would vary a name and never the bytes the unit
   reads. A meaningful P11 here needs a FIXTURE CORPUS OF FILES and a way to
   pin the name to each — a new kind of input, not a wider ladder.

`ExtController`'s precedent governs: manufacturing a harness artifact to make
`done_check.py` green would make the campaign's evidence weaker, silently. The
absence of `harness/Read_OL_Input.json` and `mutation/Read_OL_Input.json` is the
load-bearing evidence and is named in `plan.json`'s `observability`.

**This sharpens the candidate amendment unit #5 raised.** That entry asked for a
distinction *within* `blocked` between no-oracle and tool-gap. This unit is a
THIRD thing: the oracle runs, the bridge ships, the gate passes — and the
instrument's input space has the wrong SHAPE for the unit. Still the Driver's
call; recorded, not decided.

### P13 fires for the first time in this campaign

`done_check.py` reports **INCOMPLETE, 9 of 13**, failing P11, P12 and P13. The
first two are the two artifacts that do not exist. **P13 is new here**: sixteen
`mirror` units never reached it, and its message is the whole reason this unit
cannot be closed on what it does have --

```
P13  respecify_unscored: ... a reimplementation closes on a mutation score,
     not on the gate
```

A `respecify` unit is not held to a bit-identical transcription, so a passing
gate is not evidence about it in the way it is for a `mirror` unit. The
condition already knew that and said so on the first `respecify` unit it saw.
The verdict is correct and is kept.

### NOT method, target

The allocate-on-return bridge, the view-populator fix, the KGen deferred-length
gap, the three-scenario footprint and the two harness mappings are all in the
RUNBOOK's target layer. The invariant layer is untouched.
