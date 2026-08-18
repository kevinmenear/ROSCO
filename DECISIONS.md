# DECISIONS — rosco-r2

Append-only record of *why*. Never read end to end.

## Unit #54 — ParseDbAry_Opt — second dispatch — 2026-08-18

### The proposed amendment of the first dispatch was BUILT: `no_oracle_when`, the ninth judgement kind

The first dispatch (below) recorded that neither of the campaign's two keys fits
a reference that is undefined on SOME of its own paths, and proposed one. It
exists now, in the loop repo at `bf5a91b` and `829625b`, with tests:

```toml
Ary = { no_oracle_when = ["alloc_Ary == 0", "ErrVar_aviFAIL >= 0"],
        and_reference  = "ErrVar.aviFAIL < 0", reason = "..." }
```

The output named by the key is not compared on a case where every stated
condition holds — a conjunction of relations on the inputs AS THE CASE SUPPLIED
THEM, and one relation on a field the REFERENCE RETURNED. Everything else about
such a case is still compared, and both sides are still called.

**Four design points, each of which is a thing that would otherwise have gone
wrong, and three of them did before they were fixed.**

1. **The reference side, never the translation's.** `_b` is the Fortran. Reading
   `_a` would let a MUTANT choose which cases grade it — the excluded set has to
   be a property of (corpus × reference), identical under every mutant and every
   stub, or a mutation score over it is not a measurement.
2. **The input half reads a SNAPSHOT taken before the calls.** `ErrVar_aviFAIL`
   is an INOUT: it is read from the stream into `ErrVar_a.aviFAIL`, which both
   calls then write. Evaluated at comparison time it would be an OUTPUT wearing
   an input's name.
3. **The input half is a LIST.** One relation could not separate "the reference
   allocated here and failed" from "the reference never ran". Measured: the
   one-relation form excluded **4,233** where the partition predicts **4,067**,
   and 4,233 − 4,067 = 166 is exactly the set that arrived with `aviFAIL < 0`
   and never entered the body, where `Ary` is untouched on both sides.
4. **It refuses to excuse nothing.** Only the RUN knows how many cases the
   condition caught, because half of it reads the reference's return, so the
   generated program counts them, the count lands in the artifact beside
   `checked`, R4's coverage line says the output carries a region, and
   `vit_harness.py` returns 2 without writing an artifact if the count is zero —
   the same refusal `implied_by` makes about an implication that rewrote no case.

**The check that makes it a statement rather than a convenience: the corpus did
not move.** `parsedbary_opt_cases.bin` hashes
`64942b97cc3a4c2ab6631f8a452141b3e02335f8fd1acfdde613320aa77390c7` before and
after. The entry is split out before `constrain()`, so every case index is the
one every earlier artifact was taken on, and the four red-test stubs are
comparable case-for-case with the green (unit #26's rule).

**METHOD, not campaign — the Driver should consider it for the invariant layer.**
P6 says absence must not render as a value, and this is the same argument one
level down: a reference that has no answer on part of its domain must be able to
SAY SO on that part, or the layer is red for a reason that is not a defect and
every layer built on it goes with it (here: the mutation layer, entirely).

### A red PRIMARY layer takes the MUTATION layer with it — and the converse, which is the part worth keeping

The first dispatch recorded the first half. The second half is that **removing
the redness for the right reason cost nothing and bought the strongest evidence
this unit has**: 189 mutants scored, 77 killed, 108 survivors — and the survivors
say something no other layer of this unit could.

```
survived / mutants   region
     87 / 151        the list-directed READ
     16 /  17        the PRINT record
      3 /  10        the unit's own body, its constants and ary_at
```

**A mutation score localises a corpus's blind spot in a way a green cannot.**
The differential harness reports 13,674 cases and 0 failures; that number is the
same whether the corpus exercises the parser 13,674 times or 103. The mutation
sweep is what says 103 — and the no-read stub is what CONFIRMS it, moving
exactly 103 cases when the whole READ is deleted. Two instruments, one number,
neither of them the one that looked strongest.

### A survivor is a corpus gap or an instrument gap, and ONE GATE RUN EACH is what tells them apart

The RUNBOOK already says to ask whether the other instrument can reach a
survivor before calling it a corpus gap. Unit #54 has 108 of them in two
regions, so the question was asked ONCE PER REGION, with the answer predicted
first:

```
2a9e1695  parse_real:256  rec[p] -> rec[p + 1]     harness SURVIVED
          gate 1,583,216 of 4,732,000 moved, scenarios 19 and 27 dead
ae3f319a  list_directed_real:464  16 - decexp      harness SURVIVED
          gate 0 of 5,252,000
```

**The negative control is the half that makes it evidence.** A single red run
would have shown the gate can kill A survivor; the pair shows the gate can kill
the READ survivors and CANNOT kill the PRINT ones — so the two regions need
different repairs (a corpus that finds the line more than 103 times; an
instrument that compares stdout) and the campaign is not exposed on the first.

Cost: two gate runs, about ten minutes. Compare with the argument it replaces,
which is unfalsifiable prose about what a simulation probably exercises.

### A `no_oracle_when` condition can be RIGHT and still over-exclude, and the number that says so is one the campaign already had

The first form of the pin was defensible on its face — "the array arrived
unallocated and the reference came back failed" — and it excluded 166 cases too
many. What caught it was not review: it was that the first dispatch had already
MEASURED and PARTITIONED the region, so the pin had a number to be checked
against before anyone believed it.

**A judgement whose size is predicted in advance is checkable; one whose size is
only reported is not.** Same shape as this unit's four red-test stubs, all four
of which were predicted from the partition and all four of which came back
exact. The general rule: when stating an exclusion, state its expected SIZE
first and let the run agree or disagree.

### An evidence file's wrong MECHANISM is what the next unit copies

The first dispatch recorded that `alloc_Ary = { lo = 1, hi = 1 }` did nothing
because "`alloc_Ary` is a synthetic descriptor flag, not a signature parameter,
and `constrain()` matches by parameter name". It IS a signature parameter —
`harness/vitbridge.py` builds it as `Param(name=alloc, kind="int",
values=(0.0, 1.0))` — so `constrain()` matched it and applied the pin. The pin
did nothing because a parameter carrying `values` draws its cases from `values`,
and `lo`/`hi` narrow a ladder it does not have. The applicable form is
`values = [1]`.

The CONCLUSION was right (it is the wrong lever: it costs the 3,003
`alloc = 0, other` cases, the only ones where `FinalAryLen` is observable), and
that is exactly why the wrong mechanism was dangerous — nothing about the
outcome would have contradicted it, and the next unit to reach for a flag pin
would have read "flags cannot be pinned" and stopped.

## Unit #54 — ParseDbAry_Opt — 2026-08-18

### A Fortran unit that returns an ALLOCATED, UNASSIGNED array cannot be adjudicated, and this campaign has no key for it (a proposed tooling amendment)

`ParseDbAry_Opt` fails 4,067 of 13,674 differential cases, and the failing set is
a partition:

```
  alloc  arm             cases   Ary-failing
      0  not-allowed      4006     4006      <- 100%
      0  read-failed        61       61      <- 100%
      0  other            3003        0
      1  already-alloc    5697        0
      1  not-allowed       805        0
      1  read-failed        42        0
      1  other              60        0
```

A case fails **iff** the reference ALLOCATED `Ary` on this call and then returned
on a path assigning none of it or only a prefix. The elements are undefined; the
diffs are pointer values.

**The two keys this campaign has do not fit, and both were tried.**

* `no_oracle` is barred by the RUNBOOK's own unit #51 rule — the excused output
  IS this unit's whole answer.
* The domain pin unit #51 took instead was written and **did not apply**:
  `alloc_Ary = { lo = 1, hi = 1 }` produced a byte-for-byte identical corpus,
  because `alloc_Ary` is a synthetic descriptor flag rather than a signature
  parameter and `statevary.constrain` matches by parameter name. This is unit
  #47's rule met from the other end: there, `--dump-plan` caught a pin that
  silently did nothing; here the pinned run's own case count did.

What the shape actually needs is a judgement neither key expresses: **this
OUTPUT has no oracle ON THE CASES WHERE THE REFERENCE DID NOT WRITE IT.** The
generator cannot infer that condition, and the harness's existing
`inadmissible` counter — which is declared and printed but never incremented on
this unit's emitted test — is the vocabulary it belongs in. Proposed rather than
implemented: it is a generator change with real design risk, and this dispatch
had a translation to finish. Until then the honest close is a red primary layer
with the partition committed beside it, which is what this unit does.

**And note which side of it the campaign's own instruments landed on.** The
three cases where `Ary` arrived ALLOCATED are the ones that exposed a REAL
parser defect (§ the zero-store rule); the 4,067 where it did not are the ones
nothing can adjudicate. The same fact produced both.

### The echo `WRITE (UnEc,*)` cannot be translated at all, and four more units carry it

```fortran
IF ( PRESENT(UnEc))  THEN
    IF ( UnEc > 0 )  WRITE (UnEc,*)  LineNum, Tab, ParamName, Tab, Ary
END IF
```

`UnEc` is a Fortran UNIT NUMBER, and the record must go to whatever file the
CALLER connected it to — `<RootName>.RO.echo`, OPENed by
`ReadControlParameterFileSub` when `CntrPar%Echo > 0`. C++ has no access to the
Fortran runtime's unit table. The only record the translation could write is one
to `fort.<UnEc>`, a **different file**, whenever the arm is live at all.

So nothing is emitted — not even a guarded no-op, which would be a mutable
comparison no input could kill — and `harness/ranges.toml` holds `UnEc` at 0 so
the reference does not write a record the translation has no counterpart for.

**The arm is dead in all 27 scenarios and it is NOT dead in ROSCO.**
`coverage/line_coverage.json` has `ReadSetParameters.f90:359` reached 28 times
and `:360` zero, and all 22 `Examples/DISCON*.IN` set `Echo` to 0. A user who
sets `Echo = 1` gets an echo file whose array lines stop appearing.

**This is a family decision, not a local one.** The identical statement is at
`ROSCO_Helpers.f90:187`, `:269`, `:349`, `:852` and `:991` — units #55
`ParseInAry_Opt`, #56 `ParseInput_Dbl_Opt`, #57 `ParseInput_Int_Opt` and #58
`ParseInput_Str_Opt` all carry it. Three ways to close it, in increasing cost:
accept the gap in all five (what this unit does, named in §7 of its evidence);
add a permanent Fortran bridge `vit_write_unec(unit, ...)` that the translations
call, which is the only option that preserves the behaviour; or promote `UnEc`
out of the translated boundary. Raised for the Driver rather than decided here,
because the answer should be the same for all five and #54 is the first.

### The gate's `--note` is a shell argument, and backticks inside double quotes run

The first take of the default-arm gate red test recorded a note that had been
partly eaten: `` `Ary = 0` `` inside a double-quoted argument ran `Ary` as a
command (`/bin/bash: Ary: command not found`) and the artifact lost the text
that explained a zero. Re-taken with single quotes; the count was 0 both times.
A red test recording 0 is exactly the artifact whose note is load-bearing (unit
#43), so the mangling is worth one line: **single-quote every `--note`.**

### The dispatch opened on a live orphaned sweep, a live mutant, and a de-integrated tree

`ParseDbAry_Opt`'s dispatch began at 23:48 local. The tree it inherited from unit
#53 carried, simultaneously:

```
.loop-run/TREE_IS_DE_INTEGRATED   raised 03:35:25Z -- 9 src files reverted to 54dd134
.loop-run/MUTATE_IN_PROGRESS      raised 03:38:03Z -- host pid 78240, GONE
translations/Controllers/ipc.cpp  CntrPar->IPC_KI[i - 1] -> [i - 2]   <- A LIVE MUTANT
container pid 1117614             vit_mutate.py IPC --operator const_tweak, STILL RUNNING
```

The host side of that `docker exec` was killed with the dispatch. **The
container side was not**, and ten minutes later it was still sweeping — which is
exactly the outliving-orphan failure the prompt's step 2 describes, observed
here rather than inherited. It had written no artifact yet (`vit_mutate.py`
writes its JSON at the end), so nothing was lost by ending it.

Recovery, in the order the marker itself prescribes:

```
docker exec vit-dev kill 1117614          # the orphan, confirmed gone
git checkout HEAD -- translations/Controllers/ipc.cpp
git hash-object ...  811d6842dccf68f59ba7a7a7c44901bb5d436245   == intended
rm .loop-run/MUTATE_IN_PROGRESS           # only after the hash agreed
bash scripts/restore_integrated.sh        # 83 files, CMakeLists, rebuild+install
nm -D rosco/lib/libdiscon.so              # 3 of 3 _c bridges present, 0 kgen symbols
```

**The guard worked.** `MUTATE_IN_PROGRESS` named the file, the intended hash and
the exact repair, and the pre-commit hook refused every commit in the window —
so the mutant could not be committed and, because `TREE_IS_DE_INTEGRATED` was
also up, neither could the de-integrated tree. Two markers, two refusals, and
the recovery was mechanical rather than a diagnosis.

**What the guard does not do is reach into the container.** `mutate_guarded.sh`
records a *host* pid, and the process that must be killed is a *container* one;
nothing in the campaign kills it, and nothing would have noticed had it run to
completion and written `mutation/IPC.clean.const_tweak.json` into a tree that had
since been restored. Raised as a proposed amendment: the marker should record the
container-side command's own pid (or a container-visible marker file the sweep
removes), so that a later session can ask *is it still running* rather than
inferring it from a dead host pid. Until then, the check is one command and it
belongs beside the hash check:

```
docker exec vit-dev bash -lc "ps aux | grep '[v]it_mutate'"
```

## Unit #53 — IPC — 2026-08-17

### An operator's mutant population can exceed one foreground call, and nothing in the toolchain can split it (a proposed tooling amendment)

`IPC`'s differential corpus is **63,888 cases and 656 MB** — 8.4x
`ForeAftDamping`'s and 3.8x `CheckInputs`'. The per-mutant cost that follows is
25.5 s, and five operator-filtered parts agree on it to within 2%:

```
index_offset 18 in 476 s   arith_op 17 in 453 s   swap_operands 15 in 400 s
compare_op   13 in 358 s   negate_cond 9 in 264 s
```

`const_tweak` produces 40 mutants. 40 x 25.5 + 20 = **1040 s against the Bash
tool's 600 s ceiling**, and it cannot be split into two parts:

* `vit_mutate.py --limit` is `ms = ms[:args.limit]` applied AFTER the operator
  filter. There is no offset, so mutants 21..40 are not addressable at all.
* `scripts/_mutation_merge.py` refuses a part that scored a subset of its own
  operator's sites (*"each part's mutant total must equal the number of mutants
  its operators produce"*) and refuses a union missing an operator. Both
  refusals are correct — they are what stops a partial sweep reading as a whole
  one — and together they make the split impossible rather than merely awkward.

So `IPC` closes `deferred` on P12 with 78 of 118 mutants scored, and the
refusal is committed (`evidence/IPC/mutation.merge_refusal.txt`) rather than
described.

**THE OBVIOUS WAY OUT WAS TRIED AND PRICED AT ZERO, WHICH IS WHY IT WAS NOT
TAKEN.** `LocalVar_NumBl = { values = [3, 0] }` shrinks the corpus to 42,694 and
the per-mutant cost to 19.4 s — `const_tweak` would still be 796 s. `values =
[3]` would not have been enough either: 40 compiles alone are about 180 s, so
the corpus would have to fall to roughly 25,000 cases, and there is no narrowing
of that size with an admissibility argument behind it. The two-value take was
kept anyway, because it answers unit #52's question a second time:
`evidence/IPC/mutation.index.numbl_2value.json` scores the identical 18
`index_offset` mutants at 14 of 18 with the SAME four survivor ids as the
63,888-case take. The narrower list costs nothing measurable here — which is a
second data point on #52's lever, in the opposite direction from #52's own.

**What would fix it, in order of how little it changes.** (a) an `--offset N` on
`vit_mutate.py` plus a merge that sums per-operator counts across parts instead
of checking them per part; (b) putting the generated case file on
container-local storage rather than the bind mount, since 24 s to read 656 MB is
27 MB/s and the run is I/O bound rather than compute bound. Neither was done
here: both change the instrument, and changing it mid-unit would split
`loop_rev` across this unit's own artifacts, which is exactly what
`scripts/revcheck.py` exists to catch. Raised rather than taken.

### `{ lo = N, hi = N }` on an ALLOCATABLE extent silently deletes that array from the comparison (a proposed method amendment)

This unit reads four `CntrPar` ALLOCATABLE arrays and subscripts every one with
a literal. ROSCO's own reader allocates each at exactly 2
(`ReadSetParameters.f90:389,392-394`), and an extent below 2 is an
out-of-bounds read in the REFERENCE — so `{ lo = 2, hi = 2 }` is the pin the
admissibility argument asks for. It is also the wrong pin, and the run says so
in a place nobody had been reading:

```
UNOBSERVABLE CntrPar.IPC_Vramp: the bridge and the C struct disagree about its
  shape (no member not in the struct); supplied to Fortran as a zeroed buffer
  and NOT compared
```

`Param.fixed_extent` is DEFINED as `lo == hi` (`harness/signature.py:83`) and
means "an extent the TYPE fixes, not one the caller chooses" — the mechanism
`vitbridge.py` uses to model `REAL(DbKi) :: rootMOOP(3)`. So `emit.py` laid all
four arrays out as fixed C struct members while the real view struct carries a
pointer and a count, and four of this unit's inputs were zeroed and dropped from
the comparison by a pin written to protect them. **The harness still passed.**

`{ lo = 2, hi = 3 }` is the same narrowing without the collision. But the
general shape is a method-level hazard rather than this unit's: a bounds pin
whose ends are EQUAL is a different kind of statement from one whose ends
differ, and `harness/ranges.toml`'s header does not say so. Two candidate fixes,
neither taken here: have `constrain()` raise when a stated `lo == hi` lands on
an ALLOCATABLE field's extent, or have the run promote its own UNOBSERVABLE
lines from a note to a refusal when the parameter is one the unit READS.

Unit #47's rule — read `--dump-plan` before believing a pin — does not catch
this one. The plan reports `bounds_source: "stated:CntrPar_IPC_Vramp_n"`, which
is true; the loss happens one layer down, in the C layout.

### A mutant that survives one instrument and is killed by another is a corpus gap, and one gate run settles it

Eight of this unit's twelve mutation survivors sit at the two `std::fmin`
saturation statements: `drop_call` on both arms, `swap_call_args` on both,
`arith_op` on both, `swap_operands` and `compare_op` on one each. Every one of
them looked like a candidate for an equivalence declaration, and an argument for
one was available (`fmin` is commutative; the saturation only reaches an output
on one arm; and so on).

The surviving `drop_call` mutant was instead run through the GATE, character for
character, as a `--perturb-from/--perturb-to` pair:

```
mutation/IPC.clean.calls.json   SURVIVOR b36f5d50   63,888 cases, survived
gate.redtest.fmin_dropcall.json                     159,758 of 5,252,000, KILLED
```

**The same edit survives 63,888 differential cases and is killed by the gate.**
So it is not equivalent, and the eight survivors are a property of the harness
corpus rather than of the program. No declaration was made.

This is the campaign's usual complementarity inverted. The gate is normally the
coarser instrument — unit #52's gate constrained nothing its unit computed, and
five units in six have found an exact zero annihilating a gate perturbation. Here
the gate is the ONLY instrument that discriminates at this site, because the
harness reaches the arm (one of the two `swap_operands` mutants dies there) and
never supplies a case in which `fmin`'s second argument wins.

**The practice worth generalising:** before declaring a survivor equivalent, ask
whether the OTHER instrument can reach it, and if it can, spend the run. One
289 s gate red test replaced eight equivalence arguments with a measurement, and
it replaced them with the opposite conclusion.

### An arm can be inside the gate's window, exercised, and still compute an exact zero — and the additive/multiplicative pair is what says so

Unit #52's pair, run deliberately on the same statement:

```
IPC_PitComF[K-1] = PitComIPCF[K-1] + 0.01    334,388 of 5,252,000   scenarios 2, 6, 8, 18, 27
IPC_PitComF[K-1] = PitComIPCF[K-1] * 2.0     201,604 of 5,252,000   scenarios       8,     27
```

Scenarios 2, 6 and 18 run this unit and its answer is EXACTLY 0.0 in all three.
The baselines confirm it independently — all three `bld_pitch` channels are
identical in exactly those three scenarios and differ in the other two, which is
unit #46's "a baseline array can be a unit's own output" used as a control
rather than as a selector.

Scenario 18 is the one that matters: it is the campaign's ONLY scenario with
`IPC_ControlMode == 2`, so it is the only one that reaches the 2P arm, and it
runs non-zero gains (0.1/0.05, 0.01/0.005). It still computes zero, because its
1-DOF sim has zero blade root moments and the Coleman axes are therefore zero.
**The gate can tell that the 2P arm RAN and cannot tell what it COMPUTED.** That
is unit #46's finding — presence in a window is not visibility — met at the
level of a whole arm rather than of an invocation, and it is the reason a
scenario census by hit count would have been misleading here.

## Unit #52 — ForeAftDamping — 2026-08-17

### A stated range is applied and yet the corpus it produces may discriminate nothing, and that is P9 one level down (a proposed method amendment)

`harness/ranges.toml` entries are written to keep a corpus inside the domain the
reference has an answer on. This campaign already has one check that a pin was
*applied* — `vit_harness.py --no-build --dump-plan` prints `bounds_source`, and
unit #47 lost a corpus to a TOML block whose keys silently landed in the wrong
table. That check passed here on the first try:

```
{"name": "LocalVar_NumBl", "kind": "int", "lo": null, "hi": null,
 "values": [3, 0, 1, 2], "bounds_source": "stated:LocalVar_NumBl"}
```

**Applied is not the same as discriminating.** The pin as first written was
`{ lo = 0, hi = 3 }` — the spelling three earlier units carry — and it was
applied then too. A census of the corpus it produced
(`evidence/ForeAftDamping/numbl_census.txt`) says:

```
2539 cases     NumBl   0: 14    1: 2505    2: 12    3: 8
```

98.7% at one value. This unit's only index arithmetic is `FA_PitCom[K - 1]`,
and at `NumBl == 1` the loop runs once with `K == 1`, where `K - 1` and `1 - K`
are both 0. So the `swap_operands` mutant on that subscript was killed on **20
cases of 2539** — exactly the 12 + 8 with `NumBl >= 2` — and the configuration
ROSCO itself runs, `NumBl == 3` in all three gate scenarios, had 8 cases. The
sweep still scored 8 of 9; nothing in the artifact said the margin was 0.8% of
the corpus.

Replacing the entry with `{ values = [3, 0, 1, 2] }` — the same interval, every
member enumerated, base draw at the shipped configuration — makes NumBl a second
FLAG, so R2 and R6 re-run their ladders under every declared value: 7567 cases,
`{0: 1890, 1: 1894, 2: 1893, 3: 1890}`, the subscript observable on 3,783.

**The proposed amendment.** P9 says coverage is not visibility. Unit #46 raised
the capture-window form of it: presence of an arm in a capture window is not
visibility of that arm, and only a stub settles it. This is the *stated-range*
form: a pin that is applied, in-domain and correct still fixes a base draw, and
the base draw is a claim about what the corpus can discriminate that no artifact
in this campaign currently reports. What would close it is not a new rule but a
number the generator already knows — the per-parameter value histogram of the
corpus it just wrote, printed beside `bounds_source` in `--dump-plan`. Today it
costs an `fprintf` and a `--no-generate` rebuild per unit, which is why three
earlier units carry the same pin and nobody had looked.

**What is NOT claimed.** This is one parameter in one unit. It is not evidence
that `[PreFilterMeasuredSignals]`, `[ActiveWakeControl]` or `[FlapControl]`
have concentrated corpora — none of the three has been censused — and it is not
a claim that a `values` list is generally better than a `lo`/`hi` pair. The
`values` form is only cheap here because the admissible interval is `[0, 3]` and
can be written out; on a real-valued or wide integer parameter it would be a
much larger cost than the one it removes.

### The gate can be green on 5,252,000 values while every scenario drives the unit at an exact zero, and the separating probe should be run on purpose

Unit #48 discovered the additive/multiplicative distinction by accident, chasing
a perturbation that moved nothing. Run deliberately it costs one gate run and it
answers a question the green cannot:

```
the stored value + 0.01    311,723 of 5,252,000
the stored value * 2.0             0                <- the SAME statement
```

All 21 `Examples/DISCON*.IN` set `FA_KI = 0.00000` and `FA_IntSat = 0.00000`,
and `PIController` ends `saturate(PTerm + ITerm, minValue, maxValue)` with both
limits at zero, so this unit's answer is exactly `0.0` on all 63,997 calls the
27 scenarios make. Any translation returning zero passes this gate.

That belongs in E5.2's verification ledger as this unit's "what it could not
see", and it is stated in `plan.json`'s `observability` rather than left to be
inferred from a green. Whether the ledger should carry a *column* for it —
"gate perturbation class that moves the unit's own channel: additive only" — is
the campaign question this raises; it is the fifth instance in six units and the
first in which the zero covers every scenario rather than one.

## Unit #44 — YawRateControl — 2026-08-15

### The reference cannot pass its own kernel, and only a positive control could say so (C12, and a proposed method amendment)

`vit verify` printed

```
✓ VERIFICATION PASSED: 104/104 passed
  Field log: kernel/YawRateControl/verify_fields.csv (47944 entries)
```

and the field log carried **two `OUT_TOL` rows**, both `debugvar%yawratecom`,
both at a state-machine edge:

```
YawRateControl.0.0.188   computed  0.49847328176309996  reference 0.0
YawRateControl.0.0.1189  computed -0.49847328176309996  reference 0.0
```

Running `kernel.exe` directly ends with `Number of verification-passed cases :
102` and `kernel: YawRateControl: FAILED verification`. So VIT reported a green
over a red kernel. **That is recorded here first, with the wrong artifact, and
before anything was done about it** — `evidence/YawRateControl/kernel.verify_fields.csv`
is committed with the two OUT_TOL rows in it.

Two readings of those two rows were possible and they call for opposite actions:
a translation defect at exactly the two invocations where the state machine
changes state — which is where a wrong translation *would* show first — or an
instrument that cannot replay this unit at all. **A positive control
distinguishes them and nothing else does.** `Controllers.f90.kgen` is KGen's own
output before VIT substituted the C++ bridge into it, so it holds the original
`YawRateControl` body, the four `SAVE` declarations included; swapping that ONE
file back and rebuilding the same kernel gives

```
the C++ kernel VIT built              102 of 104, failing 188 and 1189
the same kernel, reference body       102 of 104, failing 188 and 1189
full verbose output, line for line    97,568 lines compared, 0 differing
```

The cause is in `kernel_driver.f90`: it calls the procedure **three times per
case** — evalstage, warmupstage, mainstage — re-reading the captured in-state
before each and verifying the THIRD. The in-state carries `avrSWAP` and the five
derived types. It does not carry `INTEGER, SAVE :: YawState`, which is a
subroutine local. On 102 of 104 cases that costs nothing because the unit is
idempotent there; the two exceptions are the two START edges, where the first
call makes the transition and the second and third then find the machine already
yawing.

`evidence/YawRateControl/run_fortran_control_kernel.sh` is the control, and it
is a committed script rather than steps somebody remembers to repeat.

**PROPOSED METHOD AMENDMENT.** X4 says never take a green at face value on first
use. This is its mirror and it is not stated anywhere: *before attributing a RED
result to the translation, put the reference's own body through the same
instrument.* It costs one rebuild here — 71 seconds — and it is the difference
between "the translation is wrong at the two most interesting invocations" and
"this instrument cannot replay a procedure with SAVE state". Both readings were
available from the same two rows, and only the control chooses between them.

### The kernel is blind to this unit's only real output, and the harness is blind to both its stop arms

Unit #43 measured that `avrswap` is not a compared field. This unit is where
that costs the most: `avrSWAP(48)` is the commanded yaw rate in rad/s, the one
value the calling program reads from this procedure, and everything else it
writes is a debug store or an intermediate. Deleting **both** writes to it —
the main one and the open-loop override — leaves the kernel at **102 of 104**,
its own green, unmoved. VIT's own comment in the generated `DISCON.F90` says
why: `skipped avrSWAP verification (assumed-size incompatible with KGen
comparator)`.

The differential harness is not blind there — but it is blind somewhere the
kernel is not, and the two blind spots do not overlap:

```
                                        kernel 104 cases    harness 8093 cases
  avrSWAP(48), both writes deleted      102 of 104 PASS     (compared)
  the two STOP arms                     51 of 104 (stub 3)  0 entries, ALL 8
                                                            undeclared survivors
```

Entering a state arm at all needs `iStatus /= 0` — `iStatus == 0` resets
`YawState` two statements earlier — and at `iStatus /= 0` LPFilter takes its
steady-state arm against a `LocalVar%FP` the harness zero-initialises on every
case and does not vary:

```
1.0 / FP->lpf1_a1[i] * (-(a0*last) + b1*x + b0*lastIn)
     = 1.0/0 * (0 + 0 + 0)  =  inf * 0  =  NaN
```

Every comparison against NaN is false, so `error <= 0` and `error >= 0` are
false on all 725 entries. `evidence/YawRateControl/arm_census.txt` measures it:
`from 1: stop 0 persist 6`, `from -1: stop 0 persist 719`.

**The eight survivors there are NOT declared equivalent.** A defect in either
stop arm is a real defect; what is missing is a corpus that reaches it, and the
kernel has one. `yawratecontrol.kstub-no-right-stop.cpp` deletes the single
statement `YawState = 0` from the right stop and the kernel goes **51 of 104**,
with the first failing case **1045** — the invocation the extraction window was
aimed at, computed before any of this was known.

### The capture window was read off a committed baseline rather than estimated

`Examples/vit_sim.py::run_scenario_7` integrates the commanded yaw rate itself,
`nac_yaw[i] = nac_yaw[i-1] + nac_yawrate[i]*dt`, and `nac_yaw` is one of the 13
channels in `baseline_arrays/scenario_7.npz`. So

```python
sign(diff(nac_yaw)/dt)   # IS YawState as written, per invocation
```

read off the reference run this campaign already owns. The four edges fall at
invocations 187, 1044, 1188 and 2044 and repeat every 2,000 (NacVane is 20 deg
at a 50 s period, 50/0.025 = 2000). CROSS-CHECKED rather than trusted: the
census of that same sign array is 10,261 / 10,215 / 3,522, and clean-source
coverage carries 10,261 hits at `:440-:441` and 10,215 at `:451-:452` — exact,
both. Five ranges, 104 cases.

**AND KGen's CASE INDEX IS ONE HIGHER THAN THE PYTHON LOOP INDEX.** `ROSCO_ci`'s
constructor calls the controller once with `iStatus = 0`, which the loop
(`for i, ti in enumerate(t): if i == 0: continue`, `iStatus = 1` thereafter)
never does — and coverage agrees, recording exactly **1** hit on the
`iStatus == 0` arm. That off-by-one is what makes case 188 the WRITE invocation
whose reference `YawRateCom` is 0.0 rather than the first persist invocation
whose reference is `Y_Rate`. Getting it wrong would have made the SAVE-state
finding unreadable.

### Three baseline states, and the mechanism is inter-case state chaining

The first sweep scored **0.6988** and the arm census said why in one line:
`ENTRY YawState: 1 -> 0` in 8036 cases. The third arm's two `IF`s are
independent, so a NEGATIVE `deadband` fires both and the second write wins:

```
  from 0: start-right-only 0   start-left-only 48   BOTH ifs fired 192
```

R6 draws `Y_ErrThresh` on the +/-1e3 default, so `deadband` was negative in 192
of the 242 third-arm cases. Nothing left the arm at +1, so the entire
`YawState == 1` block was dead.

No case can set `YawState`: it is ambient state, not a parameter. What reaches
it is that **the harness advances the state by calling the unit**, and an R11
block runs consecutive cases from one baseline. A state that makes
`deadband` small and non-negative and the error positive leaves `YawState = 1`,
and the next case in the block — one where R11 has moved `iStatus` off 0, so
there is no reset — enters the arm. Measured: `ENTRY YawState: 1 -> 6`.

Reaching a *controlled* `NacHeadingError` at all is unit #41's route met for the
third time: `LPFilter`'s initialisation arm returns its input BIT-EXACTLY at
`CornerFreq = 0`, so `F_YawErr = 0` with `iStatus = 0` turns the two filtered
components back into `cos` and `sin` of the input and `atan2` inverts them.
State C takes every wind-direction term to 0, where the whole chain is exact and
`NacHeadingError` is exactly `0.0` against a `Y_ErrThresh(1)` of exactly `0.0` —
which is what kills `>` -> `>=` and `<` -> `<=`, two strictness mutants no
ladder over a computed quantity can reach.

### Two subscripts re-spelled before the second sweep, not after it

Unit #43's rule — *ask before writing the line whether the spelling offers a
mutant no input can kill* — applied prospectively for the first time.
`CntrPar->Y_ErrThresh[2 - 1]` keeps the reference's `(2)` visible and offers

```
swap_operands  '2 - 1' -> '1 - 2'    index -1, a read BEFORE the array
arith_op       '2 - 1' -> '2 + 1'    index 3, in bounds only at extent >= 4
const_tweak    '1 - 1' -> '1 - 2'    index -1 again
```

`[0]` and `[1]` offer one `const_tweak` and one `index_offset` each, all four
landing on a real element of an array R5 never draws shorter than three. Both
`index_offset` mutants were killed on the second sweep, 17 and 351 cases. The
cost is a fifth mutation part: re-spelling gave `index_offset` a site the unit
did not have before, and `_mutation_merge.py` refused the union until it was
covered — correctly, and that refusal is the check doing its job.

### A zeroed nested member is not a conservative default, it is an inadmissible input -- and it made eight mutants unkillable (second dispatch, proposed method amendment)

`harness/emit.py` gives every nested derived-type member the same value on
every case: zero. It says so, in the `UNOBSERVABLE` line it prints, and unit
#44's first dispatch read that line and drew the right conclusion about the
survivors -- both stop arms unreachable, `from 1: stop 0`, `from -1: stop 0` on
all 725 entries.

What the first dispatch did NOT do is ask whether zero is an admissible value.
It is not. `LocalVar%FP` holds `LPFilter`'s coefficients, and the shipped
program initialises them on the first call and reads them on every call after,
so an all-zero `FP` at `iStatus /= 0` is a state no reachable program produces.
Handed that state the reference evaluates

    1.0/a1 * (-(a0*out) + b1*x + b0*in)  =  1.0/0 * 0  =  inf * 0  =  NaN

which is not weak coverage of the arms downstream -- it is a division by zero
inside a CALLEE, poisoning every comparison after it. The difference matters
because the two call for opposite actions: under-coverage is a corpus gap to
widen, an inadmissible input is a defect in the instrument to fix.

Fixed by addition (P5), in the loop repo (X2), off by default:
`vit_harness.py --persist-nested ARG.MEMBER` keeps the member per side in .bss
and copies it back in at the top of the next case. **That is the property this
harness already relies on for a `SAVE` local**, which persists across cases
because it is a function-local static nothing resets -- so the generator was
already modelling a call SEQUENCE for state it could not see, and resetting the
state it could see. Control: the loop test suite's failure set is identical with
and without the change, and with an empty set the emitted source is
byte-identical to before.

**The method-level rule, which is why this is marked a proposed amendment.**
`UNOBSERVABLE <arg>.<member>: nested type, zero-initialised` is printed as a
coverage note and read as one. For a member that holds a callee's ACCUMULATED
STATE it is a correctness note about the input domain, and the question to ask
of every such line is the one this campaign asks of a scenario: *can the shipped
program ever be in this state?* Where the answer is no, the survivors underneath
it are not evidence about the translation at all. It belongs beside P9 --
coverage is not visibility -- as its input-side twin: **a value the instrument
supplies is not automatically a value the program can hold.**

### This unit's corpus can no longer be generated in the container, and the host was killed too

Measured, twice, on the second dispatch, and it is a fact about the campaign's
own instrument rather than about this unit's translation:

```
vit-dev (7.7 GiB)   python3 vit_harness.py YawRateControl ... --no-build
                    RC=137 (Killed) after 33 s, immediately after the
                    ADMISSIBLE BASELINE line -- i.e. inside generate(),
                    long before anything this dispatch changed is reached
                    free -g inside the container afterwards: 7 total, 7 free
host (36 GiB)       the same command, RSS climbing past 6.6 GiB
                    Killed: 9 after 1178 s on the first attempt
```

The 8093-case corpus in the tree was generated by the first dispatch at 08:15
on the same day, from the same inputs, in the same container. A corpus that can
be produced once and not again is not reproducible evidence, and every red test
this unit has rests on the claim that the corpus a red run uses is the corpus
the green used.

It also decides what a second dispatch can and cannot do here. Widening the
input domain -- which is the correct response to a survivor the harness cannot
reach -- requires regenerating the corpus, and the environment refuses. The
generation was moved to the host (`evidence/YawRateControl/run_generate_corpus.sh`,
`--no-build`, so only the generation moves and the build and the run stay in
the container) because that is the only place it has any chance of completing.

**A standing candidate for the Driver.** 507 varied parameters, several of them
arrays of 1024 and 3000 elements, held as Python objects for every case at once
is what costs the memory; `generate()` materialises the whole corpus before
`write_cases` sees any of it. Streaming it -- one case built, written and
dropped -- is a generator change with no effect on the bytes, and it is the
difference between this unit's evidence being reproducible and not.

### Standing, unchanged: `vit verify` and `vit integrate` rewrite vit.yaml with `yaml.dump`

735 of 747 lines of comment lost per run. The unit's own block was re-merged by
hand into the committed file both times, which is what every unit since #2 has
had to do. Recorded again only because the count is now large enough that a
reader might mistake the rewrite for a deliberate simplification.

## Unit #43 — StructuralControl — 2026-08-15

### The kernel is blind to one of this unit's two writing statements, and a stub says so

`StructuralControl` writes in exactly two places: the three-constant step into
`LocalVar%StC_Input` under `StC_Mode == 1 .AND. Time > 500`, and the loop that
copies `StC_Input` into `avrSWAP`. The extracted kernel compares ~250 field
names and `avrswap` is not one of them, so the second statement is invisible to
it. Deleting the whole loop **passes 62 of 62**.

That was read off `verify_fields.csv` and then MEASURED, which is the order that
matters: the field list is a claim about what the kernel compares, and a stub is
the only thing that turns it into a claim about what the kernel can catch. The
gate is not blind — `gate/StructuralControl.redtest.step_constant.json` moves
7,998 values through that loop — so this is a statement about one instrument and
not about the unit.

### A no-op passes 61 of 62 because the state it writes is state it was handed

Both `avrSWAP` and `LocalVar%StC_Input` are INOUT and persist across calls in
the driver. From invocation 20,003 onward the values this unit writes are
already present on entry, so a unit that does nothing reproduces them exactly.
The whole discriminating power of the kernel against the emptiest possible
defect is **one case**, invocation 20,002 — the single call in a 23,999-call run
at which `StC_Input` changes value.

That case is in the corpus because the middle window was aimed at
`23,999 - 3,998 + 1 = 20,002`, arithmetic over a coverage count, and written
into `vit.yaml` with the cost stated BEFORE the extraction ran. A start/middle/
end window would have reported the same 62/62 green over a corpus a no-op passes
completely. This is unit #41's and #42's finding met a third time, and the shape
is now stable enough to state generally: **when a unit's outputs are INOUT state
that persists between calls, the kernel's power lives entirely at the
transitions, and a window not aimed at one measures nothing a no-op fails.**

### A dead arm whose scenario exists: reading the guard would have said the wrong thing

The `StC_Mode == 2` open-loop arm is at 0 hits in all 27 scenarios and the gate
red test on it moves 0 of 5,252,000. The obvious sentence — "no scenario
configures StC_Mode = 2" — is **false**. `Examples/vit_sim.py` scenario 24 sets
`StC_Mode = 2`, `StC_Group_N = 1`, `StC_GroupIndex 2801`, `Ind_StructControl 8`
and `OL_Mode = 1`; it is the one scenario in this campaign built for this arm.

Coverage puts 0 hits on `DISCON.F90:136` under scenario 24 while `DISCON.F90:141`
carries 8,000, so all 8,000 of its calls fall through the main block. Units #23
and #26 had already measured the cause — `Read_OL_Input` takes its `RETURN` on
the absent `Examples/example_inputs/OL_Mode2_Input.dat` — and STATUS.md has
carried scenario 24 executing no controller code as an E3.3 failure since phase
3. The difference matters because it decides whether widening the scenarios
could ever help: here it could, and the blocker is one missing file rather than
a configuration nobody wrote.

Both statements are recorded INTO `gate/StructuralControl.redtest.openloop_arm.json`
as `notes`, not into prose elsewhere, because a reader who opens that file finds
a zero and needs the expectation beside it.

### A defensive guard the reference does not have is a site no input can kill (proposed method amendment)

The first sweep scored 0.810 with eleven survivors. Four of them were at one
site I wrote:

```cpp
std::vector<double> row(cols > 0 ? static_cast<size_t>(cols) : 0);
for (int32_t j = 0; j < cols; ++j) row[j] = OL_StructControl[j * rows + (I_GROUP - 1)];
... interp1d_c(..., row.data(), cols, ...)
```

`cols` is `n_OL_StructControl_cols`, which R5 never draws below 3 and the view
populator never makes negative. So `cols > 0`, `cols >= 0`, `cols > 1` and
`: 0` versus `: 1` all spell the same number, and **no input can distinguish
them** — three mutants that are equivalent for a reason that is not about the
reference at all. The fourth is worse: passing `cols` rather than the buffer's
own length as `SIZE(yData)` means a loop bound widened to `j <= cols` writes
past the vector and the extra element is never read, so the mutant corrupts the
heap in silence and scores as a survivor.

Rewriting the site removed all four: no guard (`cols` is a size), `push_back`
(so a wrong bound changes the buffer), and `row.size()` as the length (which is
what the reference's `SIZE(yData)` actually denotes). **The rule this
generalises is unit #37's, seen from the other side.** #37 recorded that
`std::max(a, b)` is a site `swap_call_args` can never kill and declared it. The
same question asked before writing the line is cheaper: *does this spelling
offer a mutant no input can kill?* A defensive test on a quantity that cannot
take the value it guards against always does, and it is not transcription — the
reference has no such test.

**Proposed as a method amendment.** The existing prohibition is about inlining
and about working around tool bugs; this is a third thing — a translation may
not add a guard the reference does not have, and the mutation score is what
detects it. It belongs beside X1/X2 rather than in this campaign's target layer,
because it is about how a translation is written and not about ROSCO.

### R11 reaches an arm R6 crosses but cannot enter

R6 crossed this unit's four predicate quantities at the full cross product, so
`StC_Mode = 2` with `StC_Group_N = 1` IS in the corpus. What those cases lack is
`Ind_StructControl(1) > 0` — and that is not a knob quantity, because its
subscript is the loop counter. The generator says so itself:

```
note: CntrPar_Ind_StructControl[I_GROUP - 1]: subscript not traceable to a
      parameter -- its index is NOT exercised by R5
```

So the arm is *crossed into and not entered*, which no widening of R6 can fix.
`harness/baseline.StructuralControl.json`'s first state ramps
`Ind_StructControl` from exactly **1** — the value at which the surviving
`> 0` -> `> 1` mutant changes an answer — and kills all three of the remaining
open-loop survivors at once.

### R13 and the arm it exists to reach are disjoint, on a corpus 10x smaller

Unit #41 found that `R13_staging_capacity`'s 256 cases all take their other
inputs at the BASE DRAW, where `aviFAIL` is not negative and the ErrMsg refusal
guard is never entered. This unit reproduces it exactly: 120 of 1263 cases reach
the helpers, and `n_ErrMsg_cap` is **4104 in every one of them** — the base
headroom — against a longest message of 35. Two units, two corpora an order of
magnitude apart, one number. That is what makes it a property of the RULE rather
than of either unit, and it is the third of this unit's four declarations.

`evidence/StructuralControl/errmsg_census.txt` is the measurement; the probe run
is GREEN (1263/0), so it is a reading of the corpus and not a perturbation of it.

### A cleanup that restores from version control discards uncommitted work of the same kind (proposed method amendment)

`evidence/<Unit>/run_harness_redtest.sh` — the shape every unit in this campaign
copies — ends with `trap 'git checkout -- "$CPP"' EXIT`. That restores the
**committed** translation, so an uncommitted edit to it is silently gone when
the red test finishes. The gather rewrite above had to be made twice, and the
first version's disappearance was invisible: the red test itself reported
correctly, and only `grep` on the file afterwards showed the old code back.

This is `restore_integrated.sh`'s stated warning one directory over, and the two
have the same fix: **commit before running anything whose cleanup is a
checkout.**

**Why this is a method amendment rather than a target-layer note.** The general
form is not about these two scripts. Any tool that guarantees "the tree is put
back" by restoring from version control puts back the COMMITTED state, so it
silently discards uncommitted work of the same kind — and the discard is
invisible precisely because the tool reports success and its own measurement is
correct. The campaign already has the rule for the *reset* window (commit before
you reset) and for *artifacts* (commit each as it exists); what is missing is
the general one: **a working file that any cleanup path will `checkout` must be
committed before that path can run, and the file under test is such a file.**
Three of this campaign's stub runners, `restore_integrated.sh` and
`mutate_guarded.sh` all have this shape; only the last one detects it, by
recording a hash first.

### Standing, unchanged: the two ErrMsg helpers are copied into every unit that takes an ErrVar

This is the seventh unit to declare `'>' -> '>='` on the staging-capacity refusal
and the seventh to carry a character-for-character copy of `assign_errmsg` and
`errmsg_trim`. Four of this unit's four declarations are at those two helpers.
A shared, once-mutated, once-declared helper would make one declaration instead
of N. Unit #37 proposed it to the Driver as a phase-bracketed change (K4); it is
still open, and every future unit that takes an `ErrVar` will pay the same four.

## Unit #42 — Startup — 2026-08-15

### A survivor list is not a list of sites: six of fourteen were one fact about `sigma` (proposed method amendment)

The first mutation sweep scored 92 of 106 and the fourteen survivors looked like
fourteen problems. Six of them were one:

    sigma(x, x0, x1, y0, y1)   returns y0 outright when x < x0

and in every stage-2 case the generator produced, `LocalVar%Time` was below
`LocalVar%SU_LoadStageStartTime`. So the call returned `y0` and nothing past it
reached an answer — not the ramp end `LSST + SU_LoadRampDuration(SU_Stage-1)`,
not its subscript, not `y1`. Every mutation of those was invisible, and the
survivor list reads as five independent index/arithmetic blind spots plus one
constant:

    arith_op       'LSST + SU_LoadRampDuration' -> 'LSST - ...'    x1
    arith_op       'SU_Stage - 1' -> 'SU_Stage + 1'                x1
    swap_operands  'SU_Stage - 1' -> '1 - SU_Stage'                x1
    const_tweak    the two `- 1`s of `SU_Stage - 1 - 1`            x2
    const_tweak    '1.0' -> '2.0'  (sigma's y1)                    x1

**The confirmation was already in the same sweep, from the other side.**
`drop_factor` on `y0` — `SU_RotorSpeedThresh * WE_GearboxRatio` — was KILLED on
624 cases. A corpus that kills the argument a function returns and misses every
argument it does not is not a corpus with six holes in it; it is a corpus that
reaches exactly one arm of one callee. Reading the survivor list as a set of
sites would have produced five separate baseline states and one of them would
still have been wrong.

Two states, not one, because `x < x0` and `x > x1` are two different arms of
`sigma` and a single `Time` cannot be in both.

**Why this is a method amendment rather than a target-layer note.** The
generator, the mutator and the score are all method machinery; the step between
*here are N survivors* and *here is what to add* is not, and it is where this
campaign has spent the most time. The rule that worked here: partition the
survivors by SOURCE POSITION, then ask what single fact about control or data
flow puts a whole partition out of reach -- and look for a KILLED mutant at the
same call to say which arm the corpus did reach. Reading the list site by site
produces one baseline state per site, and here that would have been five states
where two were correct.


### A threshold whose left side the unit COMPUTES, reached the same way unit #41 reached its two

`LocalVar%SU_RotSpeedF < 0.95 * CntrPar%SU_RotorSpeedThresh` compares a value
this unit computes against one it is given. R6's relational-pair rule sets one
side FROM the other and needs BOTH to be inputs, so it cannot put this predicate
at equality, and `'<' -> '<='` and `'0.95' -> '1.95'` both survived 7103 cases.

`LPFilter`'s initialisation arm returns its input BIT-EXACTLY at
`CornerFreq = 0`: the coefficients become a1 = 2, a0 = -2, b1 = b0 = 0 and the
expression collapses to `(1/2)*(2x)`, both operations exact. Setting
`SU_RotorSpeedCornerFreq = 0` with `iStatus = 0` turns the computed side back
into an input.

**The part that had to be checked rather than assumed is that `iStatus = 0` does
not cost the `SU_Stage == 1` conjunct.** It forces `SU_Stage = -1` two
statements earlier — which looks like it kills the arm — and the statement
between them promotes -1 to 1 whenever `Time > SU_StartTime`. So the state
arrives at the freewheel test in stage 1 by the reference's own route, and this
is a case where the ARM and the ANSWER are reached by the same setting rather
than traded off against each other. Unit #39's rule read from the other end.

### A capture window computed from the scenario's patch dict, confirmed to the case index

`Startup` is a state machine whose whole life is over by invocation 1,801 of
11,999. A start/middle/end window — the shape every earlier unit in this
campaign used — would have spent two of its three ranges on `SU_Stage == 0`,
where the body writes nothing.

The three transitions were computed BEFORE the extraction, from
`Examples/vit_sim.py::run_scenario_9`'s own patch dict (dt = 0.025,
SU_StartTime = 0, SU_FW_MinDuration = 5, SU_LoadRampDuration = 10 10,
SU_LoadHoldDuration = 10 10), and each was cross-checked against a coverage hit
count before the window was written:

    invocation  201  t = 5    stage 1 -> 2     :594 has 1 hit; :608 has 199
    invocation 1001  t = 25   stage 2 -> 3     :602 has 2 hits; :610 has 800
    invocation 1801  t = 45   stage 3 -> 4 -> 0    :619 has 1 hit

Four ranges rather than three. The capture agrees to the invocation: case 200
leaves `SU_Stage` at 1 and case 201 at 2, case 1000 at 2 and 1001 at 3, case
1800 at 3 and 1801 at 0.

**And the hand stub is what turns that from a description into a measurement.**
Deleting the `SU_Stage = 0` reset fails EXACTLY ONE of 83 cases, and the case is
`Startup.0.0.1801`. A midpoint window would have reported the same 83/83 green
over a corpus in which that arm is dead — which is the failure mode unit #2's
all-zero ColemanTransform kernel is kept in this repository to illustrate.

### Three instruments agree that one statement is outside every simulation this campaign runs

`ControllerBlocks.f90:587` — `SU_LoadStageStartTime = Time` under
`(SU_Stage == 1 .AND. SU_RotSpeedF < 0.95 * SU_RotorSpeedThresh)` — is measured
three ways and they agree:

    coverage/line_coverage.json          0 hits in all 27 scenarios
    gate/Startup.redtest.freewheel_arm   0 of 5,252,000 values moved, revert-verified
    kernel stub, the write deleted       83 of 83 cases PASS

and the differential harness is the one instrument that is not blind: it kills
`negate_cond` on that condition on 5618 of 7151 cases. The reason is arithmetic
rather than incidental — scenario 9 starts the rotor at 4 rpm = 0.419 rad/s
against a threshold of 0.95 * 0.3 = 0.285 — so no window into scenario 9 could
have reached it and no other scenario calls this unit at all.

This is the second unit running whose gate red test correctly FAILED, and the
distinction unit #38 drew still holds: a red test that moves nothing is a
measurement of the instrument, not a defect in the translation, and it is only
worth anything when a second instrument reaches what the first cannot.

### Four equivalences, all four argued from source rather than from case counts

Two are decided by `translations/Filters/lpfilter.cpp`'s own two lines — the
callee tests `(reset != 0)`, so `restart ? 1 : 0` and `restart ? 2 : 0` are the
same predicate; and it reads `InitialValue` only under
`if (has_InitialValue)`, which is 0 at this call site. Two are unreachable
through the PRC_R_Speed ELSEIF chain, whose second arm is `SU_Stage == 2` and
whose third therefore cannot be entered with 2 — which is exactly the value at
which `>= 2` differs from `> 2` and from `>= 3`.

**The positive control for the first pair is in the same sweep.** The companion
mutant on `has_InitialValue` itself (`'0' -> '1'`) is NOT declared and was killed
on 6033 of 7103 cases: the flag is observable and the value it guards is not.
And the same `>= 2` text at the OTHER site — line 308, the torque chain, whose
preceding arm is `== 1 .OR. == -1` and therefore does not exclude 2 — is not
declared and was killed. A declaration that would also cover a killable twin is
a declaration that is too wide.

### A perturbation that does not COMPILE is not a perturbation the gate cannot SEE (proposed method amendment)

The second gate red test failed to BUILD on its first attempt -- `\&\&` inside
single quotes reaches the compiler as backslashes -- and `scripts/gate.py`
printed `PERTURBED BUILD FAILED -- no red test was performed` rather than a red
test that moved nothing. Those two outcomes have almost the
same shape — both end with the tree reverted and no movement recorded — and this
unit's second red test is one where **moving nothing is the expected result**.
Had the tool reported the build failure as a zero, the artifact would have read
as a corroboration of the coverage data and would have been a fabrication.

Worth keeping as a property of the instrument rather than as a shell lesson: a
perturbation that does not compile and a perturbation the gate cannot see are
distinguishable only if the tool distinguishes them.

**Why this is a method amendment rather than a target-layer note.** It is not
about `&&` and not about this campaign's gate. Any red test whose expected
result is *the instrument does not see this* reports the same number as a red
test that never ran, and P3 -- *a green result must be able to name what it
compared, and be able to go red* -- says nothing about the case where RED is the
expected answer. A red test that cannot silently fail to RUN is a second,
unstated requirement on every instrument the method asks for.


## Unit #41 — Shutdown — 2026-08-15

### A STATED RANGE DID NOT NARROW A PREDICATE KNOB, and the corpus put the reference one element before the start of an array (proposed method amendment)

`harness/ranges.toml` is described in its own header as *the only judgement in
the pipeline*. It narrowed R6's ladders, the random fill and the all-pairs draw,
and it did not narrow the PREDICATE KNOB — because `predicate_knobs_from` reads
its values out of the REFERENCE's own predicates and never consults the
signature. So a range could be stated, recorded, reviewed, and have no effect on
the one generator stage that most directly decides which arms run.

Measured here. `LocalVar_SD_Stage = { lo = 0, hi = 3 }` was stated because the
reference subscripts three allocatables with `SD_Stage` under a guard that reads

    ELSEIF (LocalVar%SD_Stage .LE. CntrPar%SD_Stage_N) THEN

and NOT `1 .LE. SD_Stage .AND. ...`. The reference's own comment on the arm
below it says *"Stage > 0"*; the condition does not say so. The knob supplied
-1 anyway, in 31 of 14,621 cases, and the harness reported:

    HARNESS FAIL: checked 14621  failed 31  inadmissible 0
      case 9475 LocalVar.SD_MaxPitchRate: ref 176b14841ba98540 != got 0000000000000000
      case 9476 LocalVar.SD_MaxPitchRate: ref 3000000000000000 != got 0000000000000000

An instrumented run named the inputs in one 10-second pass —
`SD_Method=2 SD_Stage=-1 SD_Stage_N=0` — and the two "references" are
uninitialised memory: the Fortran read one element before its allocation while
the C++ read its own zero padding. **A red result that is really an undefined
reference is worse than no result**: it cannot be told from a translation
defect, and `vit_mutate.py` refuses to score against a red baseline, so the unit
could not have closed either way.

Fixed where it lives (`translation-loop f92fb9f`), not routed around: knob
values are now filtered through the parameter's stated bounds, and **only a
`stated:` bounds_source narrows** — filtering on the +/-1e3 default would
silently delete knob values for every unit already scored, which is the positive
control in the new test. A knob narrowed to NOTHING keeps its values and says
so, because deleting a predicate silently is the failure the change exists to
stop. Four tests; 159 pass in the loop's suite.

**The method question this raises** is not about knobs: it is that a file whose
entire purpose is to record a human judgement had no check that the judgement
reached every stage that could act on it. `unconstrained()` reports parameters
with NO stated range. Nothing reported a stated range that was ignored.

### THE FIRST CORPUS KILLED THE REFERENCE, AND THE C++ SIDE HAD CORRUPTED IT BEFORE THE REFERENCE RAN

    In file 'shutdown_bridge.f90', around line 1704:
    Error allocating 11703869192 bytes: Cannot allocate memory
    harness produced no JSON

11.7 GB is 1,462,983,649 REAL(8) elements, and `harness produced no JSON` is
also what a PRINTING reference produces, so the message named neither the case
nor the input. Two `fprintf(stderr)` lines in the generated test and one
`WRITE(0,*)` at the top of the generated bridge — unit #38's instrument — named
both in one 9-second run:

    VITTEST  case 21  n_PF_TimeStuck=3           n_PF_Offsets=3
     VITBRIDGE        n_PF_TimeStuck=1462983649  n_PF_Offsets=-1746481605

The two sides disagree about a value neither of them assigned. Between the read
and the call the generated test does exactly one thing: it runs the C++
translation. `LPFilter` indexes six `DIMENSION(1024)` fields of `FP` with
`objInst%instLPF`, `FP` is a NESTED STRUCT inside `localvariables_view_t`, and an
`instLPF` outside 1..1024 writes past `LocalVar_a` into whatever the compiler
laid down next — here two extents the bridge then ALLOCATEs with.

Fifth instance of one class (`[PIDController]`, `[PreFilterMeasuredSignals]`,
`[ResController]`, `[SetpointSmoother]`), same pin, and one detail worth keeping:
**it surfaced as an ALLOCATE and not a SEGFAULT only because the corrupted bytes
landed on two INTEGER extents rather than on a pointer.** The same defect one
field over is `exit 139, 0 bytes of stdout`.

### R6's ALL-PAIRS FALLBACK NEEDED THIRTEEN BASELINE STATES HERE, and the last two are a technique rather than a state

Unit #37's finding met a second time. Every statement that writes this unit's two
principal outputs needs a TRIPLE, and all-pairs holds every uncrossed knob at its
ladder's first value — `SD_Method`'s is 0, neither method. The cost was measured
before it was closed: 29 of 109 mutants survived and every non-equivalent
survivor was a subscript of one of the four SD_ allocatables.

Two of the states are worth carrying forward as a technique.
`SD_GenSpeedF > SD_MaxGenSpd` and `ABS(SD_NacVaneF) > SD_MaxYawError` compare a
value the unit COMPUTES against one it is given, so R6's relational-pair rule —
which sets one side FROM the other and needs both to be inputs — cannot reach
their boundary. **A CALLEE'S IDENTITY CASE TURNS A COMPUTED SIDE BACK INTO AN
INPUT.** `LPFilter`'s initialisation arm computes

    1/(2+c) * ( -(c-2)x + cx + cx )         c = CornerFreq*DT

which is `x` in real arithmetic for every `c` and is `x` BIT-EXACTLY at `c = 0`,
where the coefficients are 2, -2, 0, 0 and the expression collapses to
`(1/2)*(2x)` — both operations exact powers of two. Setting the corner frequency
to zero with `iStatus = 0` makes the filter the identity, and both mutants are
KILLED rather than declared. This generalises to every unit that thresholds a
filtered signal, which in ROSCO is most of them.

### A MUTATION SWEEP WAS KILLED BY THE 600-SECOND TOOL CEILING, LEFT A MUTANT LIVE, AND ORPHANED IN THE CONTAINER (proposed method amendment)

`scripts/run_if_time_remains.sh` was asked for 450 seconds, 8,400 remained, and
it started a 69-mutant part that needed ~800. **It guards the DISPATCH deadline
and knows nothing about the per-command ceiling**, which is a different number
with the same consequence — and the consequence here was the exact failure the
guard exists to prevent:

    mutate_guarded: REFUSING TO CLEAR THE MARKER.
      is       eff8060a8d49496e6e92876df53712f28f9108d8
      intended 07c2967fde7fa8318b3bebd7f1ea2eb361402e7c

plus a `vit_mutate.py` still running in the container after the tool had
returned, which had to be `pkill`ed. `mutate_guarded.sh` did its whole job: the
mutant was named, commits stayed refused, and nothing was built or gated against
it. What no tool did was refuse to START.

The remedy taken here is arithmetic and a split — four parts, the longest 460
seconds — and it is a habit rather than a mechanism. **The mechanism would be for
`run_if_time_remains.sh` to take the smaller of the dispatch deadline and a
stated per-command ceiling**, which is a change to the method's own guard and so
is raised here rather than made.

### THE POST-INTEGRATION RE-TAKE FAILED TO LINK, AND ONE COMMIT MESSAGE ASSERTS A RUN THAT DID NOT HAPPEN

Recorded under C12, with the wrong artifact named rather than quietly fixed.
Commit `3c65dbc7` says *"post-integration harness 14708/0, re-taken after the
wrapper red test"*. The re-take had FAILED to link; `harness.sh` deletes its
output before running, so that commit records the DELETION of the artifact from
the run that did pass. `09ef82b4` restores the green and
`evidence/Shutdown/postintegration_shim_race.txt` is the diagnosis. The message
is left as written: rewriting it would remove the only trace that a commit
message in this campaign has asserted a number nobody took.

The cause is worth keeping. The link failed with
`undefined reference to shutdown_c`, which reads exactly like a broken
integration, and the shim that defines it was on the link line with a plausible
952-byte object:

    $ nm vit_integration_shim.o        # .bss .comment .data .text and NO shutdown_c
    $ wc -c vit_integration_shim.cpp   # 451
    $ g++ ... -c vit_integration_shim.cpp -o (the same .o); nm ... | grep shutdown
    0000000000000000 T shutdown_c      # from the same bytes

`harness.sh` writes that file by redirecting a `docker exec`'s stdout to a HOST
path and compiles it in a SECOND `docker exec`. The two are ordered and the bind
mount still did not make the host's write visible to the container process that
followed. This tree's RUNBOOK records the same hazard twice in other forms — a
`git checkout` whose mtime make would not believe, and a host unlink racing a
container create — and this is the third: **a host WRITE racing a container
READ.** `nm` on the object settles it in seconds, because only the object was
wrong.

### Twelve declared equivalences, and none of them is at a statement this unit's Fortran contains

Stated because declaring twelve is the thing to be suspicious of. Eight are
ARGUMENTS of the four `LPFilter` calls and are decided by the callee's own
source, not by the corpus: `reset` is tested as `(reset != 0)` so `1` and `2` are
one program, and `InitialValue` is read only `if (has_InitialValue)` which is the
literal `0` at all four sites. The `has_InitialValue` flag itself is NOT declared
— its four `0 -> 1` mutants are killed. The other four are inside the two ErrMsg
helpers copied verbatim from `pitchsaturation.cpp`, and they carry a measurement
rather than an argument: a green counter probe over all 14,708 cases reports
3,565 calls, `n_ErrMsg` equal to 7 in every one, `n_ErrMsg_cap` 4103 in every
one, and zero occurrences of each of the three boundaries.

**That census also names a gap in the instrument.** `R13_staging_capacity` exists
precisely to sweep the ErrMsg capacity to its boundary, and it IS applied to this
unit — 256 cases from 0 to 255 — and it still cannot reach the refusal guard,
because every one of those cases takes its other inputs at the BASE DRAW, where
`ErrVar%aviFAIL` is not negative and the guard is never entered. The rule and the
arm it aims at are disjoint. Sixth unit to declare that site; first to be able to
say why R13 did not close it.

## Unit #40 — SetpointSmoother — 2026-08-15

### The generator PRINTED a predicate knob and put four cases of 5599 on the branch it named (proposed method amendment)

Every statement this unit computes is on the `SS_Mode == 1` arm; the other arm
writes a constant. `vit_harness.py` printed

    PREDICATE KNOB: CntrPar_SS_Mode at [0.0, 1.0, 2.0]

and the corpus it then generated reached that branch **four times in 5599**.
The two are not connected: `generate.py`'s `flags` list is
`p.values and p.kind != "char[]"`, which is the RANGES-FILE enumeration, not the
knob. `flag_variants` crosses every later stage against `flags`, so R6's 1456
ladder values were all drawn at whatever `SS_Mode` happened to be. The knob
reached the branch only through R6's own integer ladder and its own 3
combinations.

**This is P9 one level up.** P9 says coverage is not visibility — a line can
execute on data that cannot tell two translations apart. Here the *instrument*
made the same error about itself: it named a branch as a knob and reported
nothing about how often it turned it. Neither `rule_coverage` nor the artifact
disagrees; `R2_flag_values` correctly says "2 value(s) across 1 flag(s)" and
that flag is `restart`.

Measured, not suspected. Two probes deleting one arm each, one corpus, and the
partition closes to the value on both:

```
                        5599 cases      13550 cases
    IF arm deleted           4             4516
    ELSE arm deleted      5351             8628
                        ------           ------
    the no-op             5355            13144
```

`objInst%instLPF` is what makes the 4 exact: LPFilter increments it on every
call, so an IF-arm case cannot fail to differ when the arm is deleted.

Closed by ADDITION (P5): `CntrPar_SS_Mode = { values = [0, 1, 2] }`, and it is
`harness/ranges.toml`'s first entry that WIDENS — every previous one narrows an
inadmissible domain. The cost is stated in the file: `SS_Mode` leaves R6's
integer ladder, and since the reference has exactly one comparison and no
arithmetic on that parameter, every value it loses is behaviourally identical
to the 2 it keeps.

**PROPOSED METHOD AMENDMENT.** A generator that can NAME a branch should not be
able to report a corpus that barely enters it. Two forms, cheapest first:
(a) any parameter `predicate_knobs_from` identifies joins `flags`, so
`flag_variants` crosses every later stage against it — which is what this unit's
ranges entry does by hand, for one unit; or (b) the emitted `rule_coverage`
carries, per knob, the number of generated cases at each of its values, so a
four-of-5599 is visible in the artifact without an arm census. (b) is the
smaller change and it makes the failure *reportable* rather than only fixable.
Neither is a unit's decision: both change what every unit's corpus is.

### Widening the corpus killed the REFERENCE, and "no JSON" is what that looks like

Moving a third of the cases onto the arm that calls LPFilter reached case 8114 —
`SS_Mode=1` with `instLPF=-999` — and LPFilter subscripts six `DIMENSION(1024)`
fields of `FP` with `inst`. The reference read and wrote 999 elements before the
start of each array: SIGSEGV, exit 139.

`vit_harness.py` reports that as **"harness produced no JSON"**, which is
indistinguishable from a broken harness. It was localised rather than guessed:
the ALREADY-BUILT binary was instrumented twice (`CASE %d`, then the two
parameters that decide the call) and reverted, giving
`CASE 8114 SS_Mode=1 instLPF=-999` and its three surviving neighbours.
`objInst_instLPF = { lo = 1, hi = 1000 }` is [PIDController]'s,
[PreFilterMeasuredSignals]'s and [ResController]'s bound unchanged; this is the
fourth unit to meet it, and the first to meet it through this callee.

### One defect shape is seen by exactly ONE of the four instruments

Rewriting `(num/VS_RtPwr)*SS_PCGain` as `num*SS_PCGain/VS_RtPwr` is equal in
real arithmetic and a different rounding in IEEE. It is the shape this
campaign's own guidance names.

```
kernel replay, 62 cases          62 of 62 PASS   -- blind
mutation sweep, 18 mutants       silent          -- assoc_reorder is in
                                                    operators_offered and
                                                    produced no mutant here
differential harness, 13550      35 failed       -- 1 to 2 ULP
```

The harness had to be asked on a tree where the reference is real Fortran: in
PRE mode after integration both sides run the harness's own copy (unit #29), so
this unit's body alone was restored by `git checkout` rather than by the
reset/restore pair, WITH the shipped translation as a control in the same window
at 13550/0. A window that reports a stub green is indistinguishable from a
window that reports everything green.

**What this says about the operator set:** a unit can have no `assoc_reorder`
mutant and still have the defect the operator exists for. The score does not
know that, and 1.000 reads as though it did.

### The stated basis for the extraction scenario conflated a PARAMETER with a local of the same name (C12)

Commit `05ef7cae` and `vit.yaml` both said scenario 25 was "the only one whose
PRC_R_* product is not 1.0". The `R_Total` gate red test moved **two**
scenarios:

```
scenario_9   33,686 values, 6 channels     <- and it moves MORE than 25
scenario_25  22,442 values, 6 channels
```

The claim read `CntrPar%PRC_R_Speed`. The unit multiplies
`LocalVar%PRC_R_Speed`, and `ControllerBlocks.f90:555-581` (`StartupControl`)
drives it and `PRC_R_Torque` off 1.0 for the whole startup ramp whenever
`SU_Mode` is on. A scenario census that greps the patch dicts for `PRC_` cannot
see that. The CHOICE stands — stub 3 finally has its verdict, `R_Total -> 1.0`
fails 0 of 62 — but the basis recorded for it was wrong, and a red test found it
where a re-reading had not.

### A killed in-place editor left a mutant across a dispatch boundary, and the marker does not cover it (proposed method amendment)

`mutate_guarded.sh` exists because `vit_mutate.py` edits the translation in
place and restores only on completion. **`evidence/*/run_*stubs*.sh` do exactly
the same thing and no marker covers them.** The previous dispatch of this unit
was killed inside stub 3 of `run_kernel_stubs.sh`, between the edit and the
restore, and `R_Total = 1.0;` was still in
`translations/ControllerBlocks/setpointsmoother.cpp` when the next dispatch
opened the tree. `git diff` found it. Nothing was watching.

Every stub runner written this dispatch restores from `git checkout` on an EXIT
trap, and that held: the four-stub sweep was killed at the 600-second tool
ceiling and the file hashed back to the shipped translation. **The trap is not
sufficient on its own** — the killed sweep also ORPHANED a `vit verify` inside
the container, which had to be `pkill`ed, and which would have gone on writing
into a tree that had already been restored. The one-stub-per-command runner that
replaced it kills stragglers in its own trap.

**PROPOSED METHOD AMENDMENT:** the marker should belong to the ACT of editing a
tracked artifact in place, not to one tool that does it. `mutate_guarded.sh`
generalised to `guarded_edit.sh <file> <command...>` would cover both, and the
hash check it already performs is the whole mechanism.

### A trap that restores with `git checkout -- <path>` restores from the INDEX, which the probe had just poisoned

`git checkout <commit> -- <path>` writes the index as well as the working tree.
The association probe staged the pre-integration `ControllerBlocks.f90` that
way, and its EXIT trap said `git checkout -- "$F"` — restoring the working tree
FROM THAT STAGED COPY. It printed `restored`, and
`grep -c setpointsmoother_c ControllerBlocks.f90` returned **0**: the integrated
wrapper gone, staged, one rebuild away from a `libdiscon.so` with no C++ in it
and a gate that would have passed anyway, because clean and integrated are
bit-identical by design.

The only thing that said so was the `M ` in the first column of
`git status --short` — the staged-not-modified marker, which reads almost
exactly like the ` M` that a normal edit produces. Every restore trap in this
unit now names `HEAD` explicitly. This is `restore_integrated.sh`'s documented
hazard reached from a direction its warning does not cover: that script checks
for MODIFIED files under `src/`, and this file was STAGED and clean.

### Two smaller ones, recorded because both read as something worse than they are

**The post-integration harness failed once with an undefined `setpointsmoother_c`
and passed on an immediate re-run with nothing changed.** `vit_integration_shim.o`
on disk defined no symbols at all — `nm` returned zero from a 952-byte object —
though its `.cpp` was correct and recompiling by hand produced
`T setpointsmoother_c` at once. `harness.sh` writes that `.cpp` from the HOST (a
stdout redirect) and compiles it INSIDE the container one command later.
Host-to-container write propagation is the only moving part between them.

**A commit message passed as a shell argument had its backticks EXECUTED.** Four
phrases were deleted from the recorded message and `vit: command not found`
appeared on stdout. This campaign's messages quote artefact names in backticks
constantly. Messages are now written to a file and passed with `-F`.


## Unit #39 — ResController — 2026-08-14

### A reference can have no answer for its own RETURN VALUE, and `no_oracle` could only name a field

`ResController` assigns `ResController` in the ELSE arm only. On `IF (reset)` it
writes four `resP` elements and returns whatever the result slot holds, and
Controllers.f90:815 assigns that to `AWC_TiltYaw(Imode)` — the inverse Coleman
transform and every blade's pitch command. Upstream ROSCO's fifth recorded
defect, after the two `Flp_Mode==2` indexing bugs, `ExtController`'s unchecked
`ErrStat` and `UpdateZeroMQ`'s 256-into-256 record.

The harness measured it before anyone read the source that way: `failed 1763 of
3532`, every kept mismatch naming `vit_result` and nothing else, the reference's
bytes changing from case to case.

**The remedy that was available was the wrong one.** `reset = { values = [0] }`
pins the input that reaches the arm, and the arm is four writes that **no other
instrument in this campaign can reach** — the kernel has `restart` F in 62 of 62
captured cases and the gate moves 0 of 5,252,000 when the arm is perturbed, both
measured. Pinning would have bought a green by deleting the only coverage of the
only thing the harness uniquely sees.

So `no_oracle` was extended to name the function result (`translation-loop
04975cf`), which excludes the ANSWER rather than the ARM. Three properties were
kept from the existing key rather than reinvented: the exclusion is reported in
`Emitted.notes` and in the artifact's `no_oracle_outputs`, a stated name that
matches nothing is an ERROR, and `result_ctype is None` (a SUBROUTINE) is
refused with its own message rather than silently accepted. One thing was added:
R4's coverage line stops saying `return value + N out-parameter(s)` when the
return is not compared, because a true statement and a false one in one artifact
is the shape this campaign exists to remove.

**The cost is stated and it is small, and the smallness is a fact about THIS
unit rather than about the key.** The ELSE arm stores the returned value into
`res_OutputSignalLast1(inst)`, which is compared on every case, so the
arithmetic is not excluded — only the copy of it that leaves through the return.
A unit whose result is NOT mirrored into a compared out-parameter would pay far
more for the same key, and the entry in `harness/ranges.toml` says so rather
than leaving the next reader to assume the cost is always this cheap.

**FOR THE DRIVER — a possible method amendment, not taken here.** SPEC's P6 says
absence must not render as a value. This unit is the case where the ABSENCE is
in the reference and the instrument had no vocabulary for it at one of its two
output kinds. The general statement — *every compared output must be nameable as
having no oracle, not only those the tool happens to reach through a struct* —
looks like it belongs to the method rather than to rosco-r2, and it is written
here rather than in the invariant layer for the Driver to raise.

### The kill counts are the census, and they cost nothing to read

The first sweep scored 0.9275. Its per-mutant kill counts are bimodal:

```
killed on    12 of 3532 cases   21 mutants   <- the ELSE arm's arithmetic
killed on  1763 of 3532 cases    8 mutants   <- the reset arm's four writes
killed on  3532 of 3532 cases    7 mutants   <- structural
```

1766 cases run the ELSE arm and about twelve of them let its arithmetic reach a
compared output. Unit #34 needed a purpose-built census probe
(`clamp_census.csv`) to learn the same thing about `PIDController`; here the
sweep's own stdout said it, and the only reason it nearly went unread is that
the first run's output was not captured to a file. **Redirect the sweep's stdout
even on a run you expect to discard** — it is the cheapest census this campaign
has, and it is already being produced.

The cause is `[PIDController]`'s cause 2 exactly: `saturate(x, lo, hi)` returns
`hi` whenever `lo >= hi`, R6 draws both bounds from one ±1e3 default, and its
isolating stage pins every other real to 0.0 or 1e300 — which sets the bounds
equal. The pins are the call site's own shape widened, and they differ from
`PIDController`'s in one place: `minValue = { lo = -1e9, hi = 0 }` rather than
`hi = -1e-3`, because all 14 `Examples/DISCON*.IN` carry `PC_MinPit = 0.000` and
a pin that excludes the value the whole program uses is a narrowing nobody
needed.

### A mutant can be non-equivalent, unreachable by every ladder, and killable by one case

`2.0*(omega*omega)` → `(2.0*omega)*omega` survived two corpora. It is NOT an
equivalence: multiplying by two is exact, so in the normal range `2·RN(u)` and
`RN(2u)` agree for every input, and in the SUBNORMAL range — where the quantum
is absolute rather than relative — they do not. `u = 0.4 D` gives `0.0` and `D`.

What made it look equivalent is that reaching a compared output from that corner
needs a SECOND quantity at its own extreme, and every ladder in the generator
moves ONE. The one-quantum difference has to survive `-8 +`, so `DT*DT` must be
within a factor of two of DBL_MAX before it clears half an ulp of 8.

Closed by addition (P5), one state in `harness/baseline.ResController.json`, and
the state is arithmetic rather than a guess: `freq = 1.77e-163` puts
`omega*omega` at 0.4 D, `DT = 1e154` puts `DT*DT` at 1e308, `ki = 0` keeps
`2*DT*ki` out of a0 and a2 where it would swamp the b1 term, and every other
term is zero so the sum is the b1 term alone — 2.0 against 1.9999999999999998.
0.986, and the last survivor is the declared one.

**The rule this is an instance of:** *before declaring a floating-point
reassociation equivalent, ask where the rounding grid stops being relative.*
Powers of two are exact and the two forms agree everywhere the result is normal;
subnormals and overflow are the whole of the difference, and both are reachable
inputs rather than theoretical ones.

### The no-op red test needs the callee bridge the no-op has no reason to call

`harness.sh` decides which callee bridges to generate by reading the translation
for calls, and drops the callee's own `.cpp.o` from LIBS when it keeps one. A
no-op stub calls nothing, so no bridge is kept — and every OTHER integrated unit
in the build tree that calls `saturate_c` fails to link:

```
rescontroller_test.cpp:(.text+0x388): undefined reference to `saturate_c'
picontroller.cpp:(.text+0x74):        undefined reference to `saturate_c'
collect2: error: ld returned 1 exit status
```

The red-test stub keeps one `(void)saturate_c(0.0, minValue, maxValue);` whose
result is discarded and whose arguments are constants, so it cannot make the
stub agree with the reference on any output. Recorded because the failure looks
like a broken harness and is a property of the STUB: the first unit whose no-op
red test needed a call in it, and there will be more as the callee graph fills.

## Unit #37 — PowerControlSetpoints — 2026-08-14

### The differential harness ran eleven of twenty statements zero times, and the green did not say so

`probes/arm_census.cpp` is the shipped translation with one counter per arm and
an `atexit` that writes them out. It changes no behaviour — it fails 0 of the
corpus — and it answers the one question a green cannot: which statements did
the cases actually reach.

```
                                BEFORE   AFTER
calls                             3596    3648
PRC_Mode == 2                       17      63
  PRC_Comm == Constant               1      21
  PRC_Comm == OpenLoop               1      21
    Ind_R_Speed  > 0                 0      17
    WRITE(401,*) executed            0      17
  PRC_Comm == ZMQ                    1       3
  PRC_Comm matched NOTHING          14      18
the ELSE arm                      3579    3585
```

**The cause is arithmetic, not luck.** R6 crosses the quantities the reference's
own predicates test. Six knobs here — `Ind_R_Pitch`, `Ind_R_Speed`,
`Ind_R_Torque` (4 values each), `PRC_Comm` (5), `PRC_Mode` (4), `aviFAIL` (4) —
and 4·4·4·5·4·4 = **5120**, past `generate.py::_KNOB_CASE_LIMIT` of **4096**. Past
the bound the block covers all PAIRS, and in the pair fallback every knob not
being crossed sits at the FIRST value of its ladder — for `PRC_Mode` that is 0,
the ELSE arm. Reaching the `WRITE` needs `PRC_Mode==2` **and** `PRC_Comm==1`
**and** `Ind_R_Speed>0` in one case: a TRIPLE, which all-pairs cannot express by
construction. **1024 combinations over the bound cost this unit its entire
OpenLoop arm.**

This is the first unit in the campaign to reach that fallback. Every earlier
artifact's coverage line reads `the full cross product`, which is why the
fallback's cost had never been paid.

**Closed by addition (P5).** `harness/baseline.PowerControlSetpoints.json` states
two admissible states and R11 walks each knob off them one quantity at a time —
which is exactly the shape a triple needs. 52 cases added, the other 3596
unchanged, so the corpus is a strict extension and the green before it is the
green inside it. The baseline is measured to be load-bearing rather than argued:
the OpenLoop-no-interp1d stub fails **19 of 3648** with it and **0 of 3596**
without it.

### A mutation score and the green it is scored against must name the same case count

C12, recorded with the wrong artifact before the repair:
`evidence/PowerControlSetpoints/mutation.WRONG-CORPUS-3596.json`.

`vit_mutate.py` rebuilds and re-runs the differential harness per mutant against
**whatever case file is on disk**. The last thing to write that file before the
first sweep was the P10 control — the OpenLoop stub with the R11 baseline moved
aside — which regenerates the corpus at 3596. So a complete, internally
consistent, entirely wrong sweep was produced: 77 mutants, 21 killed, score
0.2763, and nothing in it announcing which corpus it read.

**It was caught because the survivor list reproduced the census's zeros.** The
six survivors on `Ind_R_Speed > 0`, `Ind_R_Torque > 0` and `Ind_R_Pitch > 0` are
exactly the predicates the R11 baseline was added to reach.

This is unit #26's rule one instrument over — a red result and the green it
certifies must name the same case count — and **nothing in the pipeline checks
it**: `mutation/<U>.json` records `compared_against` but no case count at all.
Candidate for the Driver: `vit_mutate.py` should stamp the case count it scored
against, and `done.py` P12 should compare it with P11's.

### Three oracles, because the unit has three kinds of output and one instrument sees one kind

`WRITE(401,*) LocalVar%PRC_R_Speed` is an output nothing in this campaign
compares. The kernel cannot — the arm is dead in all 27 scenarios. The gate
cannot — it compares `avrSWAP` channels. The differential harness cannot — it
compares the mapped signature, and no signature carries a file. plan.json said
so in advance: *"writes to unit 401 without opening it — an UNCOMPARED output,
not a missing input"*.

It is reproduced anyway, because the mirror contract is about the reference's
behaviour and not about the part of it some instrument happens to watch. What
makes that affordable is that the format was **measured** rather than read off a
standard: `list_directed_corpus.f90` writes 22,526 doubles twice, once as
list-directed records and once as raw bits, and the model reproduces all 22,526
byte for byte.

```
every record        exactly 26 characters
non-finite          "NaN" / "Infinity" / "-Infinity", right-justified in 26
0.1 <= |v| < 1e17   F-form, 17 significant digits, right-justified in 21,
or v == 0           then FIVE trailing blanks
otherwise           E-form, 1 digit + 16 decimals + E<sign><3 digits>, in the 26
```

The one place C and Fortran disagree is `d == 0`: `%.0f` drops the decimal point
and `F21.0` keeps it — **488 of the 22,526 records**, and the whole difference
between a model that mismatched and one that does not.

### Three oracles, 68 of 76 mutants killed, and one field that now overstates what it means

The unit has three kinds of output and each instrument sees one kind, so the
mutation score had to be taken three times:

```
                                                     killed   of
differential harness, 3648 cases                         29   76
list-directed format oracle, 22,526 gfortran records     36   45 formatter mutants
unit-401 file oracle, fort.401 vs fort.401.cpp            3    3 write_401 mutants
                                                     -------
killed by SOME instrument                                68   76
declared, unkilled by any                                 8
```

**None of the eight is in a statement transcribed from the Fortran.** Every
mutant in the unit's twenty reference statements is killed by the harness: 0
body survivors. The eight are all in the `WRITE(401,*)` formatter, six of them
unreachable *by the type* rather than by the corpus — a double's decimal
exponent is at most three digits, so the `Ee` overflow arm and everything in it
is dead for any input C admits.

**`vit_mutate.py` has one bucket for "not killed by me" and this unit needs
two.** Writing "killed by another named oracle" into `--equivalences` is the
only way to record it, and it makes `equivalent_declared: 47` an overstatement
of its own field name: 39 of those 47 are kills, with mismatch counts, in
`mutation/PowerControlSetpoints.equivalences.json`. `mutation/
PowerControlSetpoints.harness-only.json` is the same sweep with nothing
declared — 29/76 = **0.3816**, the harness's number alone.

Candidate for the Driver: a `killed_by` field naming the instrument, so a union
across oracles is expressible. `scripts/dbgmutate.py` solved the same problem
for unit #31 by replacing the oracle wholesale; this unit needs the union of
three, which nothing in the pipeline expresses.

### A probe that goes red must survive its own `set -e`

`run_ld_probe.sh` measured correctly and *reported* nothing. A red probe exits
1; under `set -e` the script died on the measuring block and never reached its
own `cat "$OUT"`. The artifact on disk was always right; only the echo was lost.
So a caller reading stdout saw no counts at all and could not tell a kill from a
build failure — **37 of 45 formatter mutants were first graded `nocompile`, and
every one of them was a kill.**

Worse than a wrong answer: the file and the caller disagreed, and the file was
the one nobody read.

Beside it, the same script found the bind mount again: two mutants came back
`nocompile` on the first pass and both compiled on the second, one to 3,649
mismatches. Units #23 and #30 already paid for that; the retry is now in the
scorer, and a `nocompile` is counted as neither a kill nor a survivor there —
`vit_mutate.py` is right to count one, because there the compiler rejected the
mutant; here a non-building slice is far more likely to be the mount.

## Unit #35 — PIIController — 2026-08-14

Closed `integrated` on the first dispatch. All five layers exist, all five ran,
all five are red-tested, and the mutation score is **1.000** on 30 behavioural
mutants with **nothing declared equivalent**. The interesting result is not the
score — it is that the layer this campaign normally trusts most on a live unit,
the kernel, is here a **20-of-20 pass on all-zero data**, and it took nine stubs
and one probe to say so with a number.

**A KERNEL THAT PASSES 20 OF 20 AND CANNOT SEE NINE OF THE TEN THINGS THE UNIT
DOES.** The stubs are one edit each, generated from the shipped translation by
`make_stubs.py` so they cannot drift:

```
the whole unit as a no-op returning 0.0     fails objInst%instPI ONLY
the ITerm / ITerm2 / output clamp deleted   PASSES 20/20, each
piP%ITermLast(inst) = ... deleted           PASSES 20/20
`error` forced to 0.0                       PASSES 20/20
`error2` forced to 0.0                      PASSES 20/20
a wrong answer if EITHER input is non-zero  PASSES 20/20   <- the one that closes it
```

Both error inputs are identically zero in every captured case. So `PTerm` is
zero, both accumulations are zero, all three clamps are no-ops on zero, and the
reference returns 0.0 twenty times out of twenty. Unit #2's ColemanTransform
finding at a different call site, and P9 stated once more: coverage says a line
executed, not that it executed on data that can tell two translations apart.

**AND VIT'S OWN RED TEST WENT RED ON THE INPUT THE STUB SHOWS IS DEAD, WHICH IS
NOT A CONTRADICTION AND IS WORTH THE SENTENCE.** `vit verify` printed *"the
kernel reported a mismatch with input `error` offset by 1e-05"*. That is true:
offsetting a zero makes it non-zero, so the kernel CAN go red on `error`. It is
a claim about the INSTRUMENT's sensitivity, and the stub is a claim about the
CORPUS. Unit #33 had to measure a refusal's stated reason because a refusal can
be wrong; this is the mirror — **a demonstration's stated reason can be right
and still not be the claim a reader will take from it.** The two sentences to
keep apart are "the kernel can see `error`" and "the kernel's cases vary
`error`", and only the second decides whether the 20/20 means anything.

**THE ARITHMETIC THAT SAYS THE WINDOW CANNOT BE WIDENED INTO A FIX WAS DONE
BEFORE EXTRACTING, WHICH IS THE ONLY REASON IT COST NOTHING.** Unit #25's rule:
the window is `0:0:1-20,0:0:12000-12020,0:0:23900-23920` and the site is called
11,994 times, so ranges two and three are past its last call and the capture is
invocations 1..20 — the first ~7 timesteps, before `rootMOOPF` and `Flp_Angle`
have moved off zero. It came back exactly 20. The other call site is worse: 4
hits, and it is the reset arm.

**A GATE RED TEST WHOSE COUNT IS THE COVERAGE'S CALL COUNT, TO THE BLADE.**
Offsetting the raw sum by +1000.0 moves **11,997 of 5,252,000** across three
channels — `scenario_4:flp_angle_1/_2/_3`, each 3999 of 4000. `coverage/
line_coverage.json` independently records 11,997 calls, all in scenario 4,
decomposing as 1 reset-arm call + 3998 else-arm calls per blade. gcov's
statement counter and a bit-exact comparison of simulation output have nothing
in common and agree on the total *and* on the decomposition. **When a
perturbation is confined to one channel family, check its count against the
coverage before calling the red test weak** — containment here is the shape of
the loop (the output is clamped to ±`Flp_MaxPit` and fed back as the next
call's `error2`), not a small footprint.

**P10 CAUGHT A COLLAPSED COMPARISON BEFORE IT PRINTED A NUMBER, AND THE COST OF
THE CONTROL WAS ONE RUN.** Three counting probes were generated and then, before
any of them was run, the no-op was re-run through the same path as a positive
control. On the INTEGRATED tree it fails **0 of 4607**:

```
the no-op, this unit's Fortran body intact     4528 of 4528
the no-op, this unit INTEGRATED                   0 of 4607
```

Two independent things moved. `harness.sh` links `vit_integration_shim.o` so the
integrated wrapper's `piicontroller_c` resolves — and after integration that
wrapper IS the reference, so both sides run the harness's own copy of the probe
and every difference cancels. That is unit #29's finding reproduced at a unit
whose corpus is otherwise healthy, and **the tell is far cheaper here: a no-op
scoring zero, rather than 169 survivors on a passing corpus.** Separately the
corpus changed SIZE, 4528 → 4607, because the generator mines the reference's
own literals and a marshalling wrapper has none — unit #32 already recorded that
about R12's widths. Either alone invalidates a probe number.

**THE TREE A PROBE NEEDS IS NOT ALWAYS THE CLEAN ONE, AND THE THIRD STATE IS
THREE FILES WIDE.** The corpus a committed green was taken over is reproducible
only on the tree it was taken on, and here that is neither of the two states the
reset/restore pair offers:

```
fully integrated     the reference is a wrapper: 4607 cases, no-op scores 0
this unit reverted   the reference is the 45-line body: 4528 cases    <- the one
fully clean          every OTHER unit's body is restored too, so the C++ side's
                     callees are different objects again
```

`git checkout <the pre-integration commit> --` on the three integration-carrying
paths, with an EXIT trap restoring from HEAD, lands on the middle state exactly
and does not trip the reset marker. **Ask which tree the number has to be
commensurable with, not which tree is cleanest.**

**THE PARTITION CLOSES, WHICH IS WHAT MAKES THE TWO COUNTS READABLE.** ELSE arm
2261, RESET arm 2267, corpus 4528, and 2261 + 2267 = 4528. It also explains the
mutation sweep's own numbers without a further run: the `index_offset` mutants
killing 2261 or 2267 are the reads in one arm or the other.

**AN UPSTREAM ASYMMETRY WAS TRANSCRIBED, AND THE PROBE THAT PROVES IT COULD
HAVE BEEN CAUGHT IS WHAT MAKES THAT A DECISION RATHER THAN AN OVERSIGHT.** The
reference writes `piP%ITermLast(inst)` in the ELSE arm and does **not** write
`piP%ITermLast2(inst)` there, although the RESET arm initialises both. P7 says
transcribe it. The no-op red test does not name `ITermLast2` among the five
outputs it moves — and that absence, on its own, is indistinguishable from a
channel the corpus cannot reach. `probes/itermlast2-repaired.cpp` adds the
mirror write and is rejected on **2261 of 4528**, every ELSE-arm case. **When a
red test's output list is missing a field the unit's type declares, ask whether
a probe can make that field an output before recording the absence.**

**THE THREE `saturate_c` SITES STILL GET NO MUTANT, AND ONE OF THE HAND-RUN
SHAPES DOES NOT EXIST AT ANY UNIT.** Unit #33 measured this gap at two sites;
this unit has three, and the two integrator clamps are the campaign's first
SIBLING PAIR. `drop_call` on either alone kills 2250 and 2254, on both 2255 —
three counts within five of one another, so **the corpus does not separate the
two channels through their clamps.** What separates them is exchanging the two
clamps' destinations, which moves **14**, and `swap_call_args` cannot produce
that shape because it exchanges arguments *within* one call. The three
`swap_call_args` zeros are the equivalences already proved at
`evidence/saturate/minmax_probe.txt`, so the standing amendment's effect on this
unit is +3 kills, +3 equivalences and a score unchanged at 1.000 — the third
consecutive unit for which the amendment is free, which is the cheap case for
the Driver.

**NO ENTRY WAS ADDED TO `harness/ranges.toml`, AND THE ABSENCE IS A DECISION.**
This unit has PIDController's structural gap — `minValue` and `maxValue` drawn
independently from one default, so the pair that must be an INTERVAL often is
not — and it cost nothing measurable: 1.000 with no pins, `kp*error -> kp`
killed on 167 and `DT*ki -> DT` on 15, so the clamps are inactive often enough.
A pin narrows a domain and every entry in that file carries the measurement that
forces it; there is none to carry here. `R15_bracketing_bounds` remains the
generator-level remedy and remains unbuilt.

### C12 — two instrument faults, each recorded with its failing artifact first

**`scripts/harness.sh` COULD NOT RUN ITS OWN DOCUMENTED USAGE LINE.** In PRE
mode with no optional arguments, `${ARGS[*]}` on an empty array is an unbound
variable under `set -u` in bash 3.2 — this machine's shell. Latent for
thirty-four units because every previous pre-mode invocation passed `--out`,
which appends to that array. The failure is in the SAFE direction (it aborts
before generating a corpus, so no number was ever produced from it), and that is
why it is worth writing down rather than only fixing: the shape to fear is the
mirror image, an unset variable expanding to nothing and a command running with
one argument missing. `evidence/PIIController/harness_sh_unbound_args.txt`,
fixed additively as `${ARGS[*]-}`.

**A TRANSIENT BUILD FAILURE DELETED A COMMITTED GREEN, AND THE NEXT COMMIT
RECORDED THE DELETION UNDER A MESSAGE SAYING THE GREEN HAD BEEN RE-TAKEN.**
`harness.sh` writes its artifact by redirecting into `--out`, so a build that
dies does not merely fail to write — it destroys what was already there. The
identical command a minute later passed. That is unit #23's and unit #30's
bind-mount hazard at a new site; what is new is the blast radius. `git add -A`
then staged the deletion and the message was written from the red tests' output
one line above it. **`b090ab2` is left in the history unamended**, with
`evidence/PIIController/postintegration_transient_build_failure.txt` beside it.
Making `harness.sh` write to a temporary and rename is the obvious repair and is
NOT made here: it changes how every artifact in this campaign is written, which
is X3's question and not a unit's. Candidate for the Driver.



## Unit #34 — PIDController, SECOND DISPATCH — 2026-08-14

The first dispatch closed `deferred` at **0.759** with seven survivors and
declared none of them. This one takes it to **1.000**. Six of the seven died
because the INPUTS changed; the seventh is declared, and it is the only one that
was ever a candidate.

**A SURVIVING MUTANT ASKS WHICH OF THREE THINGS IT IS, AND THE ANSWER HERE WAS
THE SAME FOR SIX OF THEM: THE HARNESS COULD NOT REACH IT.** Not equivalence —
declaring them would have moved 0.759 to 1.000 in one edit and verified nothing.

**A CENSUS BEATS A COUNTING PROBE WHEN THE QUESTION IS NOT A BIT, AND IT COSTS
ONE RUN INSTEAD OF ONE PER QUESTION.** The first dispatch answered five
questions with five probes, each writing a sentinel into `piP%ITerm2` and each
costing a harness run to learn one number. The question the REMEDY needs is not
a bit: it is *what does the corpus actually put in the two bounds and the three
terms, case by case*, and a remedy chosen without seeing that distribution is a
guess. So `evidence/PIDController/probes/clamp-census.cpp` — generated from the
shipped translation by `make_census.py`, the same anti-drift rule the counting
probes use — appends one CSV row per call and **changes no compared output**.

The run that produces it is therefore GREEN, and that is the check that the
census did not perturb what it measures: `0 of 4610 failed`. Read by
`census_report.py`, over the 2307 ELSE-arm cases:

```
the return value == minValue or maxValue    2307 of 2307
minValue < maxValue at all                   207 of 2307
EFilt finite                                   1 of 2307
the raw sum PTerm+ITerm+DTerm finite           1 of 2307
the outer clamp INACTIVE                       0 of 2307
```

The middle two rows are the ones no counting probe had asked for, and they are
the bigger half of the answer.

**A GREEN WHOSE COMPARISON IS NaN AGAINST NaN IS AN ABSENCE RENDERED AS
AGREEMENT.** `LPFilter` ends in

```fortran
LPFilter = 1.0/FP%lpf1_a1(inst) * (-FP%lpf1_a0(inst)*... )     Filters.f90:65
```

and `FP%lpf1_a1(inst)` is written ONLY inside `IF ((iStatus == 0) .OR. reset)`.
The harness supplies `LocalVar%FP` **zeroed** on every case — it is a NESTED
type the generator does not vary — so on every case with `iStatus /= 0` and
`reset` false the reference computes `1.0/0.0 * 0.0` = NaN, the translation
computes the same NaN, the comparison passes, and the case constrains nothing.
`iStatus` was an ordinary defaulted integer on R6's decade ladder, which reaches
0 exactly **once in 2307 cases**. This is P6 from the other side: the apparatus
already refuses to let an absence render as a value, and here an absence
rendered as *agreement*.

**AND THE SECOND CAUSE IS R15's, MEASURED AGAIN AND WORSE THAN STATED.**
`minValue` and `maxValue` are drawn independently from one default and came out
`(0, -300)` in **1512 of 2307** cases — `saturate(x, lo, hi)` is
`MIN(MAX(x,lo),hi)`, which returns `hi` for every x once `lo >= hi`. R6's
isolating stage then pins every OTHER defaulted real to one value, which sets
the two bounds EQUAL: the same empty interval by another route.

**THE REMEDY IS THREE ENTRIES IN `harness/ranges.toml`, WHICH IS A PER-UNIT
STATEMENT AND NOT A GENERATOR CHANGE.** `R15_bracketing_bounds`, specified in
this file at the first dispatch, remains NOT BUILT: it is a change to a shared
generator, X3 forbids that mid-run without an ablation, and it would move every
already-scored unit's corpus. What this dispatch establishes is that the
campaign's existing judgement mechanism can state the same thing for ONE unit,
with no revision to any instrument — `loop_rev` is unchanged at `e7d5583` across
all nine of this unit's result artifacts, and `revcheck --unit PIDController` is
clean.

```
LocalVar_iStatus = { values = [0, 1, -1] }     the reference's OWN documented
                                               enumeration (Filters.f90:41),
                                               0 first so it is values[0]
minValue = { lo = -1e9, hi = -1e-3 }           the ONLY call site's own shape,
maxValue = { lo = 1e-3, hi = 1e9 }             widened: Controllers.f90:346
                                               passes -VS_MaxTq*2, +VS_MaxTq*2
```

The magnitude is taken from the clamped expression rather than picked — |raw|
over the finite cases has median 2994 and p90 1.86e6, and 1e9 is above both.
That is R15's own "lo and hi from the SCALE" clause, satisfied by a constant
because a per-unit range cannot compute one per case; it is enough here and it
is not the general rule.

```
                                     BEFORE            AFTER
the outer clamp INACTIVE        0 of 2307        1316 of 4884
the ITerm clamp INACTIVE        8 of 2307        4285 of 4884
minValue < maxValue           207 of 2307        4884 of 4884
EFilt finite                    1 of 2307        1507 of 4884
```

and the six survivors die at ordinary counts: `kp*error -> kp` 1288 of 9758,
`-> kp/error` 1349, `EFilt-ELast -> EFilt+ELast` 1367, `PTerm+ITerm ->
PTerm-ITerm` 1325, and the two `[i] -> [i+1]` reads the first dispatch
identified by building twelve variants — lines 112 and 121 — at 1274 and 1323.
**28 of 29, 0.966, WITH NOTHING DECLARED** (`mutation/PIDController.undeclared.json`).

**THEN, AND ONLY THEN, ONE DECLARATION.** `const_tweak '0.0' -> '1.0'` on the
last argument of `lpfilter_c(..., 0, 0.0)`. The preceding literal `0` is
`has_InitialValue`; the generated bridge does not pass the argument at all in
that arm (`pidcontroller_callees.f90:40-44`) and the shipped C++ `LPFilter`
reads it only under `if (has_InitialValue)`. **The flag's own mutant, one
position left on the same line, is killed in 1145 of 9758** — so the corpus
distinguishes the gate and fails to distinguish only the value the gate makes
unreadable. That pairing is what makes the declaration checkable rather than
convenient, and it is the shape to look for before declaring anything:
*is the thing that would make this argument live itself observable?*
`mutation/PIDController.equivalences.md`.

**AN ID IS NOT A PLACE, AND THE ARTIFACT ONLY CARRIES A SHAPE.** `'0.0' ->
'1.0'` matches two sites. `harness.cppmutate.mutants()` carries a LINE, so
re-running it over the shipped translation turns every id into a place
(`evidence/PIDController/mutant_sites.txt`) — and independently, the OTHER
`'0.0'` mutant is killed in exactly **4874 of 9758**, which is exactly the
reset-arm case count the census reports. Both identifications agree. The first
dispatch built twelve variants to answer the same question for `index_offset`;
this is the cheaper instrument for it, and it was in the repository the whole
time.

**WHAT IS STILL NOT SEEN, AND IT IS A GENERATOR GAP RATHER THAN A RANGE.**
`LocalVar%FP` is a nested type: zero-initialised, never varied. So `LPFilter`'s
RECURSIVE arm is unreachable from this harness — 3377 of 4884 else-arm cases
still answer NaN — and what the corpus exercises is the INIT arm plus NaN
propagation. No entry in `ranges.toml` can state a non-zero filter state; that
needs the generator to vary a nested type's fields, which is the same
frontier `observability` has been naming since unit #11 and is not this unit's
to cross. The pins also remove the INVERTED bracket (`lo > hi`) from the domain,
and the defect that would need it lives in `saturate` — unit #24, CALLED not
inlined, so both sides of this comparison always reached one implementation of
it and this harness never constrained it.

**FOR THE METHOD, NOT FOR THIS CAMPAIGN — two candidates, stated here rather
than edited into the invariant layer.**

1. **A census probe belongs beside the counting probe in the method.** The
   counting probe answers "how many cases reach this arm" and needs one run per
   question; the census answers "what is in every case" in one run, and it is
   admissible for the same reason — it changes no compared output, so its own
   run is a green that certifies it did not perturb what it measured. The rule
   is the discipline, not the file: *a probe that changes a compared output is
   a red test; a probe that changes none is an instrument, and it must be run
   green before its numbers are quoted.*
2. **P6 has a second face: an absence must not render as AGREEMENT.** A
   comparison of NaN against NaN, of a bound against the same bound, or of a
   held parameter against itself is a case that passes and constrains nothing.
   The apparatus counts `checked` and `failed`; nothing counts *cases whose
   comparison could not have failed*. On this unit that number was 2307 of 2307
   and the artifact said `4610 checked, 0 failed`.

## Unit #34 — PIDController — 2026-08-14

**THE SCENARIO BUILT FOR THIS UNIT IS ONE OF THE THREE A MISSING FILE STOPS, AND
THAT MAKES TWO OF THE FIVE LAYERS UNAVAILABLE BEFORE ANY WORK STARTS.**
`vit_sim.py`'s scenario 10 exists for exactly this function — its docstring is
"OL_Mode=2 azimuth tracking to exercise PIDController" — and it sets `OL_Mode=2`,
`Ind_GenTq=5`, `Ind_Azimuth=6` and `RP_Gains = 1000 100 500 0.1`. It is also one
of the three scenarios (10, 14, 24) that reach `ReadSetParameters.f90:778`, fail
in `Read_OL_Input` on the absent `Examples/example_inputs/OL_Mode2_Input.dat`
(unit #17), and are taken out by the `RETURN` two statements later before any
controller code runs. So the one call site at `Controllers.f90:346` has **0 hits
in all 27 scenarios**, there is no runtime state to capture and no kernel to
build, and the gate compares 5,252,000 values while constraining none of them.
`evidence/PIDController/coverage_deadness.{py,txt}`, exit 0, with the positive
control P10 asks for: PIController's body, read out of the same dict by the same
expression, 12 lines and 12,339,878 hits.

This is unit #26's `unwrap` shape for the fourth time (#1 `AddToList`, #21
`UpdateZeroMQ`, #26 `unwrap`) and the first time the dead unit is one the
campaign's own scenario set *tried* to reach.

**TWO INSTRUMENT DEFECTS, BOTH RECORDED WITH THEIR FAILING ARTIFACT BEFORE BEING
FIXED (C12), AND BOTH IN THE SAME FAMILY: A HELPER WHOSE NOTION OF A NAME IS A
BARE IDENTIFIER.**

1. `vit 3ce00e8` — `generate_callee_declarations_header` enumerated *two* types
   its declarations can name and does not define (`int32_t`, `CFI_cdesc_t`) and
   there is a third: `build_c_params` renders a derived-type dummy as
   `<typename>_t*`, and those structs are in `vit_types.h`, which sits beside
   this header in both directories that consume it. `LPFilter` takes a
   `TYPE(FilterParameters)`, so this is the first unit whose CALLEE has the
   shape. `pidcontroller_callees.h:14: error: 'filterparameters_t' has not been
   declared`, and g++ recovers by reading the unknown type as `int` — so the
   diagnostic points at the CALLER's argument rather than at the header, and a C
   consumer would take the implicit int and link.
   `evidence/PIDController/vit_defects/`.

2. `loop e7d5583` — **an index can itself be a derived-type field, and neither
   inferrer could see one.** `piP%ITerm(objInst%instPI)`: the ARRAY being a field
   was handled, the INDEX being one was not, because splitting `objInst%instPI`
   on non-word characters yields `objInst` and `instPI`, neither of which is a
   parameter name (the parameter is `objInst_instPI`). The index stayed
   unresolved, the generator drew it from R6's decade ladder, and the REFERENCE
   indexed a `DIMENSION(1024)` array with it:

   ```
   instPI outside 1..1024   4741 of 4771 cases   before
                               0 of 4692 cases   after
   ```

   That is unit #11's failure exactly — a no-op red test segfaulting the
   reference and the harness reporting "harness produced no JSON", which is also
   what a printing reference produces — one qualification level in. Fixed
   ADDITIVELY, applied only where the bare-identifier rule found nothing, under
   the same discipline the Fortran inferrer was added with at unit #11: it can
   promote an index that was `unresolved` and cannot change one already
   resolved, so no already-measured unit's corpus moves.

**ONE `ranges.toml` PIN, AND THE PARAMETER BESIDE IT IS WHY IT IS ONLY ONE.**
This unit has two 1-based indices into `DIMENSION(1024)` arrays and they need
different answers. `objInst%instPI` is subscripted in this unit's own body, so
the fix above resolves it and R5 sweeps it at 1 / interior / n — an inferred
role, which is stronger than a range. `objInst%instLPF` is subscripted inside
the CALLEE, against a field of `LocalVar%FP`, a NESTED type the generator does
not expand into parameters — there is no compared out-parameter to attach a role
to, and no inference can reach it. So it gets the campaign's only judgement
mechanism, backed by the reference's own exit status
(`evidence/PIDController/instlpf_probe.{f90,sh,txt}`, fourteen values, one
process each): SIGSEGV in `__filters_MOD_lpfilter` at both 32-bit extremes and
at -100000. The returning rows are the reason the bound is `1..1024` rather than
"whatever does not crash" — every returning value outside it reads and writes
six doubles past the declared extent, and the two sides of this comparison do
not have that memory in the same place.

**A NO-OP RED TEST CHANGES THE GENERATED CALLEE SET, AND THEREFORE THE LINK.**
`vit test-validate` derives the callee bridges from the TEXT OF THE C++
TRANSLATION (`generate_callee_bridges(Path(args.cpp_file).read_text(), ...)`) and
passes `callee_bridge=bool(callee_src)` to the Makefile generator. A stub with no
`_c(` call produces a Makefile that does not link `pidcontroller_callees.o`, and
the link then dies on a symbol belonging to a DIFFERENT unit —
`picontroller.cpp.o: undefined reference to 'saturate_c'` — because
`reset_to_clean.sh` leaves every earlier unit's `.cpp.o` in the build tree and
`vit test-validate` globs them into LIBS. A bare no-op stub therefore does not
measure a weaker instrument, it measures a DIFFERENT one, and unit #26's rule is
that a red result and the green it certifies must name the same instrument. The
stub names both callees inside `if (false)`, which keeps the bridge set, the
Makefile and the link identical and cannot contribute to any output.
`evidence/PIDController/pidcontroller.noop-stub.cpp`.

**THE MUTATION SCORE IS 0.759 AND SIX OF THE SEVEN SURVIVORS ARE ONE NUMBER:
ZERO.** Five counting probes, generated from the shipped translation by
`evidence/PIDController/make_probes.py`, each writing a sentinel into
`piP%ITerm2` — a field this unit never touches and R4 compares as one of 174
out-parameters, so no real answer can collide with it and the failing count IS
the number of cases reaching the arm:

```
the ELSE arm runs                      2307 of 4610   <- the control (P10), alive
minValue < maxValue AT ALL              207 of 4610
the ITerm clamp is INACTIVE               8 of 4610
the OUTER clamp is INACTIVE               0 of 4610
both inactive at once                     0 of 4610
```

**The return value is saturated in every case in the corpus.** `PTerm`, `DTerm`
and the sum are therefore invisible in it, and six of the seven survivors follow
from that one zero: `kp * error -> kp`, `kp * error -> kp / error`,
`EFilt - ELast -> EFilt + ELast`, `PTerm + ITerm -> PTerm - ITerm`, and both
`[i] -> [i + 1]`.

The last two were named by the artifact only as `(index_offset, '[i]',
'[i + 1]')`, which matches TWELVE sites, so they were identified BY BUILDING
EACH CANDIDATE (unit #32's rule) rather than by reasoning about which they were
— twelve variants, one per occurrence, same corpus
(`evidence/PIDController/index_sites/RESULTS.md`). The counts reproduce the
sweep's own per-mutant counts in order, which is what says the variants ARE
those mutants and not merely the same shape:

```
site08  kd * (EFilt - piP->ELast[i]) / DT                     0 of 4610
site09  saturate_c(PTerm + piP->ITerm[i] + DTerm, min, max)   0 of 4610
site05  the ITerm read that feeds the integrator update       8 of 4610
        ... and the ITerm clamp is INACTIVE in exactly        8 of 4610
```

Both survivors are READ sites whose only consumer is the saturated return, and
site05's count agreeing with the clamp probe's TO THE CASE is what makes the
explanation a measurement rather than a plausible story.

It is unit #32's finding with a different predicate: **the corpus draws the two
sides of a comparison independently, so the interesting arm is reached only
where two independent draws happen to coincide.**

**PROPOSED CORPUS RULE — R15_bracketing_bounds. A PAIR OF PARAMETERS THAT FORMS
AN INTERVAL MUST SOMETIMES BRACKET THE VALUE CLAMPED INTO IT, AND R6's ISOLATING
PIN IS WHAT CURRENTLY GUARANTEES IT DOES NOT.** Specification, with the details
that are load-bearing — a simpler rule gets each of them wrong:

```
identify the pair          from the CALL, not from the names: the 2nd and 3rd
                           arguments of a saturate(x, lo, hi)-shaped callee, and
                           the same two in the reference's own MIN(MAX(x,lo),hi).
                           `minValue`/`maxValue` is a ROSCO spelling, not a rule.
lo < hi, always            207 of 2307 ELSE-arm cases have it today. R6's
                           isolating pin sets every OTHER defaulted real to the
                           SAME value (0.0 or 1e300), which collapses exactly the
                           pair that has to be an interval. An isolating pin must
                           skip a parameter it would make equal to its partner.
lo and hi from the SCALE   a bracket must be wider than the value: draw
of the clamped expression  M = 4 * max(|each term the expression multiplies|) and
                           emit (lo, hi) = (-M, +M). Fixed bounds do not work --
                           the terms here are products of two +/-1e3 draws, so a
                           +/-1e3 bracket clamps them anyway. This is the half
                           that turns 0 of 4610 into a non-zero count.
BOTH sides of the boundary the point of a clamp is that it fires sometimes. Emit
                           cases at (-M, +M), at (-M/2, +M/2) and at
                           (raw - eps, raw + eps), so `<` vs `<=` at the bound
                           has a case and the ACTIVE arm keeps the coverage it
                           already has.
per CLAMP SITE, not per    this unit clamps twice with the same two bounds and
unit                       the sites saturate independently: the ITerm clamp is
                           inactive in 8 cases and the outer in 0, so a rule that
                           only brackets "the output" leaves the other site where
                           it was.
```

Expected effect here: the four arithmetic survivors and both surviving
`index_offset` mutants become reachable, taking 0.759 to at least 0.966 (the
remaining survivor being the equivalence below). NOT IMPLEMENTED IN THIS
DISPATCH: it is a change to a shared generator, which is what X3 forbids mid-run
without the ablation unit #32 paid for, and this dispatch's remaining clock was
committed to the cycle. Unit #32's own history is the precedent — its first
dispatch measured the gap and wrote the specification at 0.760, its second built
the paragraph almost verbatim and five of six survivors died in one sweep.

**THE SEVENTH SURVIVOR IS AN EQUIVALENCE AND IS NOT DECLARED HERE.**
`const_tweak '0.0' -> '1.0'` on the last argument of
`lpfilter_c(..., 0, 0.0)`: the preceding `0` is `has_InitialValue`, and both the
generated bridge and the shipped translation of `LPFilter` read `InitialValue`
only under `IF (has_InitialValue /= 0)`. The mutant cannot differ on any input.
It is left UNDECLARED so the undeclared score is on the record first (unit
#26's discipline), and because declaring it moves 0.759 to 0.786 — which changes
no disposition and would make the artifact harder to compare with the re-take
that R15 is for.

## Unit #33 — PIController — 2026-08-14

**A TOOL'S REFUSAL TO CLAIM IS A MEASUREMENT REQUEST, AND ANSWERING IT COST ONE
STUB.** `vit verify` printed `62/62 passed` and, in the same breath, that its
own red test's two INPUT perturbations on `error` had been ABSORBED — "typically
a saturated output". Unit #32's rule is that a refusal's stated *reason* is a
claim and can be false; this is the other half of the same rule, where the
refusal is *right* and quoting it would still have been wrong. Quoted, it is a
hedge. Measured, it is a number: the reference return value equals `minValue` in
**62 of 62** captured cases, and a stub that forces `error` to zero **passes 45
of them**. The difference matters because "the kernel may be weak here" and "the
kernel cannot see this argument in 73% of its cases" get written into
`observability` as very different sentences, and only one of them tells the next
reader which oracle to trust.

The general form, and it is a candidate for the target layer rather than for the
method: **when a verification tool declines to certify its own red test, the
decline names a specific input — perturb THAT input to zero and count.** It is
cheaper than the run that produced the hedge.

**READ THE STATUS COLUMN, NOT THE VALUE COLUMNS — `verify_fields.csv` PRINTS
NEITHER SIDE OF THE COMPARISON FOR SOME FIELDS.** `instpi` reads `3,3` in all 62
rows of this unit's field log while the stub that deletes `inst = inst + 1`
moves that exact field in all 62 cases. The CSV carries 14,508 rows where the
kernel's own stdout carries 14,818 — five fields per case are dropped. The
verdicts agree everywhere, so this is not a wrong answer; it is a column that
reads like evidence and is not. Recorded rather than fixed, because fixing VIT's
CSV writer mid-unit is not what this dispatch was for, and because the campaign
already has the rule it needs (unit #4: quote the `status` column, never the
verdict line) — this extends it one column to the left.

**PROPOSED AMENDMENT — `cppmutate`'s CALL OPERATORS SHOULD REACH THIS
CAMPAIGN'S OWN TRANSLATED CALLEES, AND THE COST IS MEASURED.**

All three call operators (`drop_call`, `swap_call_args`, `swap_callee`) are
gated on tables of callee NAMES, and every entry in `_VALUE_PRESERVING` and
`_SIBLINGS` is a C standard-library name. `saturate_c` is not one, so the two
calls in this unit's body have **no generated mutant at all** — 22 mutants, none
of them at either call site. That is not a defect in the tables: their comment
records that letting `drop_call` fire on every call made four of this campaign's
units 38–73% unbuildable, and the restriction is what stops a 1.000 measured on
the compiler.

What it is, is a table that has stopped covering the program as the campaign
integrates its own units. `saturate_c` satisfies exactly the property the table
exists to guarantee — result type equals first argument's type, all parameters
`double` — and so will every `mirror` unit whose signature is scalar-in,
scalar-out. There are more of them ahead: `ratelimit`, `PIIController`,
`PIDController`.

NOT DONE HERE (X3): amending the mutation instrument mid-run makes this unit's
score incommensurable with 32 others'. Instead the missing measurement was made
by hand, as unit #24 made it before the call operators existed at all
(`evidence/PIController/hand_mutants.{sh,txt}`, baseline 0 of 3532):

```
drop_call    ITerm clamp dropped                  1761 of 3532
             output clamp dropped                  383 of 3532
swap_args    ITerm clamp,  value <-> minValue         0 of 3532
             output clamp, value <-> minValue         0 of 3532
transposed   ITerm clamp,  bounds swapped           63 of 3532
             output clamp, bounds swapped          135 of 3532
```

The two zeros are equivalences, not gaps — `fmax(a,b)` and `fmax(b,a)` agree at
a signed zero and at a NaN on this toolchain, already proved at
`evidence/saturate/minmax_probe.txt`. So the amendment's effect on THIS unit is
+2 kills, +2 equivalences, score unchanged at 1.000. **A proposed amendment
whose effect on the unit that proposes it is nil is the cheap case to decide**,
and the Driver has the number rather than an argument.

The last row is the one the amendment does NOT cover and is worth naming
separately: transposing the two BOUNDS (arguments 2 and 3) is a shape
`swap_call_args` never produces — it swaps arguments 1 and 2 only — and it kills
63 and 135. It is also the exact defect unit #24's own post-integration red test
was built from. A three-argument selection function has a transposition the
operator cannot reach.

**ONE PREDICATE, THREE INSTRUMENTS, ONE NUMBER.** The reset branch was perturbed
from the C++ (`negate_cond`, 3112 of 3532), from the Fortran wrapper
(`MERGE(1_C_INT, 0_C_INT, reset)` inverted, 3112 of 3532) and from the whole
program (the gate, 2,128,633 of 5,252,000). Two instruments agreeing to the case
is what the campaign already treats as corroboration (unit #24's `drop_call`
figures, unit #9's control). Recording it here because the second of the three
is a shape worth reusing: **a wrapper-side perturbation of a marshalling
construct is a mutant of the SAME predicate seen from the other side of the
boundary**, and it costs one rebuild.

**A COVERAGE ZERO THAT WAS NOT A DEAD CALL SITE.**
`coverage/line_coverage.json` reports 0 hits at the line of this unit's
CableControl call site — and at three of its other 17 sites. None is dead: gcov
attributes a continued statement's hits to its LAST continuation line, and the
line three below carries 127,994. This campaign has a standing rule that a call
site with no hits is not a tool failure (unit #1, `AddToList`); the converse
needed saying too, because the report is the same zero. **Check the continuation
lines before recording a dead call site**, and the check is `range(l, l+5)` over
the same dict.

**`vit.yaml` STRIPPED OF ITS COMMENTS FOR THE THIRTEENTH UNIT IN A ROW**, by
`vit verify` and again by `vit integrate` — 267 lines on this run's diff. The
remedy is unchanged and is still manual: restore from HEAD, re-add the entry by
hand. It remains a candidate for the Driver and nothing about this dispatch
changes the argument.

## Unit #32 — FindLine — second dispatch — 2026-08-14

**THE RULE THE LAST DISPATCH SPECIFIED WAS RIGHT, AND BUILDING IT COST LESS THAN
THE DIAGNOSIS DID.** The first dispatch measured the gap (47 of 2370 cases
reach the name comparison; a second probe says all 47 are two blank strings),
named the remedy in a paragraph, and declined to build it because the X3 cost
was unmeasured. R14_planted_word (`translation-loop` `552edb1`) is that
paragraph almost verbatim, and five of six survivors died: 0.760 → 0.960.

Worth recording that the *specification* was the expensive half and it
transferred intact. The paragraph named four things a simpler version gets
wrong, and all four turned out to be load-bearing:

* the key is the word **case-inverted** (`aA` in the line, `Aa` as the key), so
  both sides fold to the same string and differ before the fold. One shape kills
  a dropped uppercasing on *either* side; a same-case plant leaves whichever
  fold is redundant untested, which is exactly the `no-uppercase` stub's old 0.
* the plant position *k* sweeps 1..3 **and every free scalar integer is set to
  *k-1* and to *k***. The generator cannot identify which integer picks the
  word — only that one usually does, under the `+1` or the `+0` convention.
* the element is FIRST and LAST **and nothing else**, so a search that skips the
  first line and one that stops at the first are both distinguishable.
* it is carried to R12's **narrowing width**, because the width of the local a
  *word* is read into is invisible for the same reason the word index is. That
  is the one that killed `'200' -> '201'`, on 14 cases.

Five kills on five different counts — 14, 48, 18, 21, 72 — which is the evidence
that they were five behaviours and not one seen five ways.

**AN X3 CLAIM ABOUT A CORPUS SHOULD BE MEASURED IN BYTES, NOT IN CASE COUNTS.**
The previous x3 check compared case counts. Two corpora of 1552 cases can differ
in every one of them, so a matching count is compatible with the thing the check
exists to exclude. `evidence/FindLine/x3_check_r14/` compares the case files:
for all three units R14 can fire on, the corpus WITH the rule is a byte-prefix
extension of the corpus WITHOUT it, and the R14-off column reproduces the
committed counts exactly. That is the whole claim, checkable, in one table.

**FOR THE DRIVER — a method-level observation. "APPENDED AFTER EVERY OTHER RULE"
IS A PROPERTY OF ONE RULE AT A TIME, AND ADDING A RULE AFTER IT SILENTLY VOIDS
THE CLAIM.** R12's own test asserted that turning R12 on moves no existing case
index. R14 fires on the same signature shape and is appended after R12, so with
R14 live, turning R12 on inserts twelve cases *in front of* R14's and the prefix
moves. Nothing was wrong with R12; what changed is that it stopped being last.

The test was not made to pass — it now ablates R14 on both sides and says why,
and the property that actually protects an already-scored corpus moved to
`test_R14_is_a_strict_extension_of_the_corpus_without_it`. But the general shape
belongs to the method rather than to this campaign: **in any generator whose
rules append, the strict-extension guarantee is held by exactly one rule, and
the guarantee each earlier rule's test asserts is conditional on every later
rule being ablated.** A campaign that adds rules over time will keep rediscovering
this one test at a time. Raising it as a candidate; no runbook edit made.

**FOR THE DRIVER — a second method-level observation, sharpening P10. A POSITIVE
CONTROL MUST SHARE THE SURVIVOR'S SITE, NOT MERELY ITS INSTRUMENT.** Unit #31
established that a control must be chosen against the program rather than
against the probe. This unit found the next step in. The surviving mutant
`'2048' -> '2049'` reports 0 against the differential harness *and* 0 against
the gate, and the gate already had a red test at 1,857,893 — in the same file,
under the same instrument, on the same unit. That red test proves the gate sees
`FindLine`; it says nothing about whether the gate can see the constant the
survivor moves. The controls that do are the same constant at the same site
moved one byte the OTHER way:

```
                                  the mutant                the control, same site
differential harness, 2514 cases  0 failed                  2048 -> 2047:  60 failed
gate, 5,252,000 values            0 moved, 0 channels,      2048 -> 5:     1,583,216 moved
                                  0 scenarios broken        across 131 channels
```

**And the asymmetry is the finding rather than the bookkeeping**: one byte too
FEW is a wrong answer, one byte too MANY is not an answer at all. No value
oracle of any kind reads bytes outside the buffer it compares.

### PROPOSED AMENDMENT, RESTATED WITH A THIRD INSTANCE — the sanitiser build

`ca75abea` is the third out-of-bounds survivor this campaign has met, after
`Read_OL_Input`'s one-byte read and `Debug`'s `c3a5bb71`. It is the first with a
same-site positive control on **two independent oracles**, which is what makes
the classification a measurement instead of a reading: it is not (a), because
the two programs do not agree on every admissible input — one of them has no
defined behaviour at all (unit #7); and it is not (b), because no widening of
any corpus reaches a difference that is not in any compared value.

The instrument is unchanged from unit #31's proposal — `-fsanitize=address,
undefined` over the same scenarios, a mutant killed when the sanitiser reports.
Two things are new. First, evidence that no cheaper instrument exists: the
second oracle was tried, at the cost of two gate runs, and reports the same
zero. Second, **the instrument itself was run** (`evidence/FindLine/asan_demo/`,
`run_asan_demo.sh`), which three units of proposal had not done:

```
original          ./test exit 0,     0 byte(s) on stderr
mutant-ca75abea   ./test exit 1,  4475 byte(s): heap-buffer-overflow,
                  WRITE of size 1, 0 bytes after 2048-byte region,
                  in char_assign, on the std::vector<char> the harness allocates
```

It fires on the mutant and it is **silent on the correct program**, which is the
half that decides whether this is one CMake option or a project — a sanitiser
that reported on the unmutated translation would mark every mutant killed and
make the score meaningless. Leak detection has to be off; libgfortran's I/O
buffers are reachable at exit and with `detect_leaks=1` the correct program
reports too.

What the demonstration does NOT do is answer the question the amendment turns
on: whether the other 31 translations are clean under it. That is a sweep and a
dispatch of its own, and it is the thing the Driver is being asked to schedule.
`mutation/FindLine.json` is untouched at 0.960 — adopting a different mutation
instrument for one unit is precisely what X3 forbids, and the demonstration is
filed as evidence rather than as a score.

### A closed unit's corpus moved, and it is a finding rather than a repair

R14 adds 72 cases to `ChkParseData` (1552 → 1624). **All 72 pass**, so this is
not a defect in that translation; what it is, is 72 inputs its committed 1.000
mutation score was never taken over. `GetWords`' older 1370-vs-1373 drift is
still open beside it. Neither is repaired here, for the reason unit #32's first
dispatch already gave: re-taking a closed unit's evidence inside another unit's
dispatch puts a number in its artifact that no commit of its own explains.

The general form is worth naming because it will recur every time a corpus rule
is added: **a shared generator improvement makes every already-scored unit's
score a statement about a corpus that no longer exists**, and the campaign needs
one deliberate re-scoring pass rather than 32 opportunistic ones. That is E5.6's
business, and this is the second dispatch to say so.

## Unit #32 — FindLine — 2026-08-14

**A REFUSAL'S STATED REASON CAN BE FALSE, AND IT COSTS MORE THAN A REFUSAL WITH
NO REASON WOULD.** `map_signature` declined `CHARACTER(*) :: FileLines(:)`
saying it "carries its extent in a descriptor `build_c_params` does not emit".
`build_c_params` emits it — `char* FileLines, int n_FileLines, int
len_FileLines` — and one run of `vit interface` shows the generated wrapper
passing `SIZE(FileLines)` and `LEN(FileLines)` as exactly those two arguments.
The count had been sitting in `extent_of` under the ordinary `n_` name the whole
time; only the CHARACTER branch never looked at `dims_of`. A refusal that names
a mechanism is read as a fact about the boundary, and this one was read that way
for as long as no unit had the shape. **Ask the generator; do not read the
refusal.** (loop `9eeaf3f`)

**TWO GENERATORS, ONE ABI, AND C LINKAGE CHECKS NEITHER.** `build_c_params`
emits `n_X` before `len_X` for an assumed-shape CHARACTER array;
`test_validate.generate_fortran_bridge` — a second, hand-restated spelling of
the same call — emitted `len_X` before `n_X`. The differential harness therefore
read a 4×3 array as 3×4, which is a well-formed array of the wrong shape, so
nothing crashed and **2311 of 2358 cases still agreed**. The 47 that did not
each had the reference's `LineNum` equal to `len_FileLines`: `DO I = 1,
SIZE(FileLines)` iterating over the element WIDTH.

The fix is not the reorder. It is that the bridge now **asks** `build_c_params`
for its extent order and refuses when the two disagree, naming both lists. A
restatement that cannot be checked is a defect waiting for the first input shape
that distinguishes it — this one waited 31 units. (vit `d2de28c`)

**A CHECK'S SCOPE IS PART OF ITS PRECISION.** `vit check` handed every
Fortran-reading check the WHOLE FILE, so a sibling procedure could supply the
Fortran half of a finding. `FindLine` contains no `SCAN`, `INDEX` or `VERIFY`
and was reported as missing `GetPath`'s backslash from 900 lines away. It is the
expensive kind of false positive: the finding names a real literal and a real
absence, and only reading the reference shows they belong to different
procedures. Latent for eight units in that file because `GetPath` and `GetRoot`
happened to carry the same separators. When the name cannot be found the CLI
falls back to the whole file **and prints that it did** — a slice that silently
missed would turn ten checks off without a word. (vit `c4eb0ad`)

**A CORPUS RULE AND A TRANSLATION SITE ARE TWO HALVES OF ONE MEASUREMENT.**
R12_narrowing_width was extended to resolve a width written as a module
PARAMETER rather than a literal, which put the 200-character truncation boundary
into this unit's corpus: +12 cases, **0.667 before and 0.667 after**. It became
three kills only when `char_assign` stopped computing its truncation through a
`std::min` whose mutants write PAST the destination instead of changing an
answer. Neither half is worth anything alone, and a rule that fires and kills
nothing is not evidence that the rule is wrong.

**THE COUNTING PROBE IS THE CHEAPEST INSTRUMENT HERE, AGAIN, AND IT REPLACED AN
ARGUMENT.** "Most cases probably don't find the name" was an inference; making
"a match happened" an OUTPUT turned it into **47 of 2370** — one stub, one run,
`LineNum = -7` inside the arm. Five of six mutation survivors follow from that
number directly. Unit #30 recorded this rule; this is its second use and it paid
for itself in one command.

**A POSITIVE CONTROL CAN BE A NO-OP FOR A REASON SPECIFIC TO THE PROGRAM UNDER
TEST.** The canary probe backing this unit's two declared equivalences reported
0 of 2370 disturbed. Its first control — `conv2uc_c(ParamNameUC,
MaxParamLength + 1)` — reported **the same 0**, because Conv2UC writes a byte
only when it is a lowercase letter and the sentinel was `\x7f`. For one run a
dead control and a passing probe were the same number. P10 says a pass built
from an empty set must prove the set could have been non-empty; the corollary
this adds is that **the control must be chosen against the program, not against
the probe** — an unconditional write, not a call that might write.

**PROPOSED, NOT TAKEN: a corpus rule that plants one input inside another --
and a second probe is what makes it specifiable rather than hopeful.** This
unit's remaining gap is that `FileLines` and `ParamName` are drawn
independently, so the predicate `FileLineUC == ParamNameUC` is true in 47 of
2370 cases and everything downstream of it is unobservable in the other 2323.

The second probe asked what those 47 ARE. Refusing the match unless the search
key holds a non-blank character fails **47 of 2370** -- all of them -- so EVERY
MATCH IN THIS CORPUS IS TWO EMPTY STRINGS COMPARING EQUAL, the same shape unit
#30 measured for `ChkParseData`. That matters because a rule which merely made
matches MORE FREQUENT would move none of the five survivors: a blank word is
blank at every index, `Conv2UC` on a blank key is a no-op, and a blank key is
200 blanks whether the width is 200 or 201. The rule must supply a NON-BLANK
name, and it needs a near-miss sibling so the comparison is not satisfied by
every candidate at once.

The rule shape: for a unit comparing a CHARACTER quantity derived from an ARRAY
input against a scalar CHARACTER input, generate cases in which the scalar IS
the *k*-th word of a chosen element, for each *k* the signature admits, and in
which the case letters differ so the `Conv2UC` on the key is load-bearing. It is
a change to a SHARED generator, so its X3 cost has to be measured across every
scored unit before it is taken.

**What this dispatch DID establish is that the measurement is cheap** --
`evidence/FindLine/x3_check/` re-takes the two units that could move in about
two minutes, and the same procedure would answer it for this rule. So the
reason it is still not taken is narrower than "that is a dispatch of its own",
and it is worth stating exactly: the rule would fire on `ChkParseData`, which
is CLOSED at 1.000, so taking it inside another unit's dispatch risks
converting a closed unit into a non-reproducing one -- which is precisely the
condition this dispatch found `GetWords` already in, and creating one
deliberately is not the same as finding one.

**THE X3 ARGUMENT WAS CHECKED AND ONE HALF OF IT WAS ALREADY FALSE FOR ANOTHER
UNIT.** All four generator changes here carry a source-level argument that no
scored unit's corpus can move. Two units could plausibly move and both were
re-taken (`evidence/FindLine/x3_check/`): `ChkParseData` is unchanged at 1552
cases, and `GetWords` produces **1373** where its committed artifact records
**1370**. One more run attributed it — at loop `12dbaa0`, the revision this
dispatch opened on, GetWords already produced 1373 — so the drift is not this
dispatch's and the X3 claim holds for every change made here.

It is a finding about `GetWords` and it is left as one. `harness/GetWords.json`
(1370 cases) and `mutation/GetWords.json` (57 of 58, 1.000, scored against that
corpus) do not reproduce on today's loop. Repairing a closed unit's evidence
inside another unit's dispatch would put a number in its artifact that no commit
of its own explains. **Raised for the Driver: a closed unit's corpus can go
stale under a later unit's generator work, and nothing currently notices.**
`revcheck` asks whether one unit's artifacts agree with EACH OTHER; it does not
ask whether they still agree with the instrument. A campaign-wide re-take is
E5.6's business and this is an argument for scheduling it.

**FOR THE DRIVER — a method-level observation, not a target one.** Three of this
unit's five layers were unavailable or wrong until a tool was fixed, and in each
case the tool *reported* rather than crashed: a false refusal reason, a silent
argument transposition, and a check answering about the wrong procedure. The
campaign's existing rule is "never work around a tool bug you control" (X2). The
gap this unit found is one step earlier: **a generator that REFUSES should be
required to state its reason in terms a reader can check against the other
generator's output**, because "UNMEASURED here" and "impossible here" read alike
and only one of them is worth a dispatch. Raising it as a candidate; no runbook
edit made.

## Unit #30 — ChkParseData — second dispatch — 2026-08-14

**THE FOUR SURVIVING MUTANTS WERE TWO STATEMENTS ABOUT THE CORPUS, AND BOTH ARE
NOW RULES.** The first dispatch closed at 0.852 with four undeclared survivors
and a proposed generator rule it deliberately did not implement, on the grounds
that a change to a shared generator is a Driver decision and that the X3 cost
had not been measured. This dispatch measured it and took it. The score is
1.000, **no line of the translation changed, and no fifth equivalence was
declared** — which is the only way a mutation score is worth reading.

The four sorted into two causes, and the prompt's own taxonomy is what sorted
them: none was (a) genuinely equivalent, all four were (b) the harness cannot
reach it, and the instruction for (b) is to fix the inputs rather than the
record.

**R12 — THE TRUNCATION BOUNDARY OF THE REFERENCE'S OWN CHARACTER LOCAL.** Every
length this generator can produce comes from `_extent_plan`: extents of 3 to 6,
laddered by R6 to `{1, 2, n, n+5}`. The reference assigns `CHARACTER(*)` dummies
into `CHARACTER(20)` locals and every real caller hands it `CHARACTER(200)`, so
the truncation happens in every shipped call and in no generated case. The width
is now mined from the reference's BODY — the same raw material R6 already mines
for numbers and characters — with `vit/checks.py::_narrowing_local` transcribed
rather than re-derived (P4).

The half that is not obvious is the second one, and it is the half a length rung
alone would have missed: **a truncation is observable only where it changes an
ANSWER**, and the answer here is a comparison between two strings. Two
independently drawn strings are equal only by accident — this corpus took its
equality arm 52 times in 1284 cases and every one was the all-blank shape — so
R12 sets every string parameter of the case from ONE body with one
distinguishing mark per parameter at index W. They agree to character 20 and
differ at 21; the reference truncates them to the same thing and a translation
that does not, does not.

**R13 — THE STAGING CAPACITY IS AN ARGUMENT OF THE CONTRACT, AND IT WAS A
CONSTANT.** This is the more general finding and it was not in the first
dispatch's proposal. A `CHARACTER(:), ALLOCATABLE` output has no C
representation, so the view carries a POINTER and a CAPACITY and both
implementations branch on it: the translation refuses an assignment that does
not fit, and the generated Fortran bridge refuses the same one, each writing a
diagnostic and leaving the field alone. `emit.ALLOC_HEADROOM` added a constant
4096 to whatever the case supplied — **in every case of every corpus this
generator has ever produced, for every unit** — so that arm was unreachable
everywhere and its boundary could not be told from either side of it. The
capacity now travels in the case stream, defaulting to `ALLOC_HEADROOM` where no
case names it, and R13 sweeps it 0..255 above what the case supplies.

**AND THE FIRST VERSION OF R13 WAS ALIVE AND BLIND, WHICH IS THIS CAMPAIGN'S OWN
RECURRING SHAPE.** It called `_case` once per rung. `_case` draws its strings
from `rng`, so 256 rungs meant 256 different inputs and therefore 256 different
message lengths, and a sweep of the capacity past a moving target meets it only
by luck. It missed — and the block was demonstrably live while it missed:
negating the refusal went from a mismatch to a SEGFAULT, a 136-byte message
written into a 6-byte buffer. **A rule that fires, changes the program's
behaviour, and still kills nothing is not an inert rule; it is a rule aimed one
degree of freedom off.** One base case copied 256 times fixed it, and the
mutant it exists for is now killed by exactly 1 of 1552 — the number the design
predicts, which is what makes the instrument readable rather than lucky.

**THE KILL SETS WERE MEASURED, NOT ARGUED.** `harness/independence.py` states
the standard — a rule's kill set is what dies with it and survives without it —
and both new rules were run back out of the corpus one at a time:

```
R12 ablated   1540 cases   '20'->'21' SURVIVED   '11'->'12' SURVIVED
                           min->len_src SURVIVED   '>'->'>=' killed (1)
R13 ablated   1296 cases   '>'->'>=' SURVIVED      the other three killed
```

Disjoint, three and one, neither covering the other's. This also settled the one
attribution that was otherwise an argument: `'11' -> '12'` moves a READ bound
onto the byte past the 11 the `Int2LStr` bridge writes, and the first dispatch
declined to call it equivalent precisely because whether that byte reads as a
blank is a property of the stack rather than of the input. It is killed by 9,
and 9 is also the number of R12 cases that reach the message-writing arm — but
an equal count is not an equal set (unit #27), so the ablation is the
measurement and the coincidence is not. The mechanism is still an indeterminate
read; the buffer is still not zero-initialised, because that would define bytes
the correct program never reads purely to move a number (unit #4, `Conv2UC`).

**A RED TEST WHOSE REVERT IS A NO-OP CANNOT CERTIFY ANYTHING, AND IT REPORTS
ITS OWN FAILURE COUNT AS THE GREEN.** `run_wrapper_redtest.sh`'s
revert-verification came back at `1552 checked, 9 failed` -- the red test's own
9 -- with the source provably reverted and the object stamped a second BEFORE
the source it was built from. The bind mount had not propagated the revert when
`make` stat'd the file. The wrong artifact is committed at `d3a8253` before the
fix (C12), and the fix is the rule `run_harness_stub.sh` already applies one
file over, plus a `touch` from inside the container: a hash says the content
arrived and says nothing about the mtime `make` compares against. **The hazard
grows as the build gets warmer** -- the same script passed twice earlier in the
same session, when the cycle was slower than the mount.

**AN ARTIFACT CAN NAME A COMMIT WHOSE CODE DID NOT PRODUCE IT, AND ONLY THE
STAMP WILL SAY SO.** The sweep that first scored 1.000 ran with R13 in the
working tree and stamped `3c19267` — the R12 commit. `revcheck` reported the
base-sha split it is for, and every artifact was re-taken at `1386430` with both
rules committed. The counts reproduced exactly, which is also the check that
neither new rule reads entropy.

**METHOD, NOT TARGET — for the Driver.** Two of the things learned here are
statements about the METHOD rather than about ROSCO, and neither is in the
invariant layer:

1. *A quantity the HARNESS chooses is an input like any other, and pinning it is
   a narrowing of the domain that no rule reports.* `ALLOC_HEADROOM` was not a
   rule, not a range in `ranges.toml`, and not in any coverage line; it was a
   constant in the emitter, and it made a branch of the C contract unreachable
   for every unit in the campaign. The generalisation: **anything the harness
   supplies that the unit BRANCHES on belongs to a rule and belongs in the
   coverage report.**
2. *A rule can fire, be alive, and kill nothing because it varies the right
   quantity against a moving baseline.* The remedy is to hold everything else
   fixed across a sweep — which is R11's "one at a time" principle applied to a
   quantity R11 does not reach.

## Unit #30 — ChkParseData — 2026-08-14

**A UNIT WITH NO KERNEL AND A BLIND GATE CLOSES ON THE GENERATED CORPUS OR NOT
AT ALL, AND THIS ONE DOES NOT — AT 0.852.** Five call sites, zero hits in 27
scenarios, so C3 has nowhere to put a pragma and the gate's 5,252,000 compared
values constrain nothing. The differential harness is the whole of the evidence,
and its threshold is the campaign's, not this unit's: `min_mutation_score = 1.0`.
Recorded `deferred`, like unit #29 and for a smaller reason.

**THE FOUR SURVIVING MUTANTS ARE THREE STATEMENTS ABOUT ONE CORPUS RULE, AND
THAT IS WHY THE NUMBER IS WORTH KEEPING RATHER THAN CHASING.** R6's string
lengths are `sorted({1, 2, ex0, ex0 + 5})` — here `[1, 2, 6, 11]`. The reference
assigns `CHARACTER(*)` dummies into `CHARACTER(20)` locals, so its truncation
boundary is 20 and **no case in this corpus reaches a 21st character**. `'20' ->
'21'` survives; so does `min(len_src, len_dst) -> len_src`, which differs only
when the source is longer than the destination; and the red test that widens the
locals to the arguments' own lengths fails **0 of 1284**, which is the same fact
from the other side and is the only one of eight red tests that stays green.

**PROPOSED GENERATOR RULE, NOT IMPLEMENTED HERE.** R6 already mines the
reference for its own literals and named constants; its LENGTH ladder is the one
input that is not derived from the reference at all. The rule that would close
this gap is of exactly the same kind:

> The declared width of every fixed-width CHARACTER LOCAL in the reference
> belongs in the string-length ladder, and so does width + 1 — because an
> assignment into it TRUNCATES at exactly that boundary, and a corpus that never
> crosses the boundary cannot see the truncation.

It is a change to a shared generator that widens the corpus of every unit with
such a local, so it is a Driver decision and not a unit's. The X3 cost has not
been measured; measuring it is the first step, not the second. Note that VIT's
`narrowing-local` check already catches this defect STATICALLY and names this
unit as its canonical instance — so the campaign is not blind to it, only its
differential harness is, and the two instruments are complementary rather than
redundant.

**A RULE THAT IS CORRECT ON AN INTEGRATED TREE CAN BE SILENTLY WRONG ON A CLEAN
ONE, AND THE FAILURE IS A NUMBER RATHER THAN AN ERROR.** Unit #29 settled
`harness.sh`'s bridge-vs-object question by asking LIBS: if `<callee>.cpp.o` is
in the link, drop the bridge, because there the Fortran callee IS a wrapper
around that object and the two would call each other. On a clean tree the same
condition holds for the opposite reason — the object is a stale artifact of an
earlier integrated build that `vit test-validate` globs in, and the Fortran
callee is the real body. Dropping the bridge there links cleanly, runs, and
reports 1284 of 1284 while running **two different `Conv2UC` implementations**,
one per side. VIT's stated one-implementation property, which is what makes a
mismatch attributable to the unit under test, was simply gone.

The repaired rule asks the TREE — does any Fortran source CALL `<callee>_c(` —
which is `reset_to_clean.sh`'s own test and maintains itself as units are
integrated. It then has to run in BOTH modes and compute the link set per mode,
because post mode adds every `.o` in the build tree on its own `make` command
line; the first post-integration run after the repair died on
`multiple definition of int2lstr_c`. **A conditional written from one tree state
is a conditional that has been tested in one tree state.** Ask which state a
rule was learned in before reusing it in the other.

**A `cp` ACROSS A BIND MOUNT CAN BREAK EVERY BUILD IN A SWEEP, AND THE ARTIFACT
IT WRITES IS A SCORE.** 31 of 31 mutants reported `no compile`; the artifact
recorded `score 0.000`; the identical command 90 seconds later compiled 31 of 31
and scored 0.742. Unit #23 recorded this hazard for ONE file and guarded it with
a hash check in `run_harness_stub.sh`; at sweep scale nothing was checking, and
the only reason the tree did not carry a mutant into the integrate is that
`mutate_guarded.sh` refused to clear its marker. **The guard the campaign built
for a hard kill caught a race instead** — worth saying, because the argument for
building it was about timeouts and this was not one.

**AND THE COST OF EXPLAINING A COUNT, PAID TWICE IN ONE CAMPAIGN.** Unit #27
wrote down "do not explain two equal counts — compute the two sets", and commit
`57b2f37` of this unit explains a count from an argument about the generator:
the wrapper red test moves 6 of 1284 because R6 tiles one string body across
every element of a CHARACTER array, so `Words(1) == Words(2)` almost always. A
one-line probe — bump `ErrVar%ErrStat`, which R4 compares and the reference
never writes — says **990 of 1284** cases have distinct elements. The wrong
claim is left standing in the git log beside the artifact that refutes it (C12).

What is true costs one source-level fact and one more probe, and it closes
exactly: arm 3 is invisible to a transposition BY CONSTRUCTION (its condition is
symmetric in the two words and its message names `TRIM(ExpVarName)`), which
removes 1230 of 1284 before the corpus is consulted; of the remaining 54,
`Words(2)` is ALSO the expected name in 48, leaving `4 + 2 = 6`. Four measured
counts, no argument. **A probe that writes to an output the reference never
touches is the cheapest counting instrument this campaign has**, and it needed no
new tooling — it is a stub through `run_harness_stub.sh` like any other.

**`vit check`'s `array-section-row` FALSE POSITIVE HAS A DIFFERENT CAUSE FROM
THE ONE ALREADY RECORDED.** Unit #28 recorded that `vit check -f <file>` scopes
its cross-source checks to the FILE and not the function, and decided not to act
(a VIT change, X3-adjacent for every unit that has run it). That decision stands
and is followed here. What is new is the MECHANISM in this instance: the match
is `Channels(:,:)`, a deferred-shape DECLARATION, not a row section in any
function at all — the detector's `\b(\w+)\s*\(\s*[^,()]+\s*,\s*:\s*\)`
accepts a bare `:` as the first subscript. So "re-attribute the finding to the
unit's own line range", which unit #28's entry prescribes, would have worked
here only by accident: the line is in a different unit. A unit-scoped run of this
detector would still misfire on any procedure declaring a rank-2 assumed-shape
array.

## Unit #29 — CheckInputs — 2026-08-13

**THE CAMPAIGN'S LARGEST UNIT IS ALSO ITS FIRST TOOLING WALL, AND THE TWO ARE
THE SAME FACT.** `CheckInputs` is 857 lines and about 180 validity checks — an
order of magnitude past anything before it — and every one of the six defects it
found is a rule that had never been asked a question this size. R7's all-pairs
fallback is quadratic in the knob count (86 here, 6 for the previous largest);
its `idx is None` path assumes a scalar and seven of those knobs name a whole
array through `ANY`; its bodies and its extents can be knobbed independently;
and it exists in two copies, so fixing one fixes half the corpus. None of these
is a defect that a smaller unit could have exposed, and none was found by
reading — each was found by a run failing.

**AN ERROR THAT NAMES ITS PARAMETER COSTS ONE PASS; ONE THAT DOES NOT COSTS
THREE.** The same emitter raised both. `CntrPar_SU_LoadStages has 17 element(s)
but its extents say 0` was diagnosed and fixed in a single cycle.
`TypeError: 'float' object is not iterable`, raised four rules downstream of
each of its three distinct causes and naming neither the parameter nor the rule,
took three. That is the whole difference, measured on one unit in one afternoon.

**THREE GENERATORS NAMED A SYMBOL AND DECLARED NONE OF THEM.**
`vit_kernel_callees.h` emitted `void addtolist_c(CFI_cdesc_t*, int)` with no
`<ISO_Fortran_binding.h>` and `int32_t nondecreasing_c(...)` with no
`<stdint.h>`; `vit test-validate` wrote Fortran bridges DEFINING both and no C
declaration at all. Both were already right in the INTEGRATION path — the
generator that has had a callee since unit #1. `CheckInputs` is the first unit
whose *kernel* and whose *differential harness* have a callee, and the two
paths that had never been exercised were the two that were wrong. **A generator
pair with one exercised half is a generator pair with one untested half**, and
nothing in this campaign was asking.

**"OTHER UNITS' OBJECTS STAY, THEY ARE PART OF THE REFERENCE BUILD" WAS TRUE FOR
28 UNITS BECAUSE NONE OF THEM HAD A CALLEE.** `harness.sh` said so in a comment
and it was right until this unit. `checkinputs_callees.f90` defines
`addtolist_c`; so does the integrated `addtolist.cpp.o`. The first repair —
drop the object, keep the bridge — fixed the duplicate symbol and created a
loop, because on an integrated tree the Fortran `AddToList` IS a wrapper around
`addtolist_c`: bridge → wrapper → bridge, SIGSEGV on case 0 with no message.
**The right resolution inverts it**: drop the bridge, keep the object. VIT's
one-callee-implementation property then holds trivially — the Fortran callee is
a wrapper around the same C++, so both sides of the comparison reach it.

**A DEFERRED-LENGTH STRING HAS NO BYTES PAST ITS LENGTH, AND WHAT THE HARNESS
RENDERS THERE IS DECIDED BY WHOEVER ALLOCATED THE BUFFER.** `ErrVar%ErrMsg` is
`CHARACTER(:), ALLOCATABLE`; an assignment reallocates it to exactly `LEN`. The
staging buffer the view crosses on is wider, and the region past `n_ErrMsg` is
not part of any value the reference has. Three renderings, all measured:

```
leave the previous message's tail   16,729 of 16,769 FAILED
blank-fill it                       16,769 of 16,769 FAILED
clear it to NUL                              0 FAILED
```

Neither red run tells you which way to go — they differ by 40 cases and both
say "ErrMsg". What settled it was the first differing BYTE, `a=0x20 b=0x00` at
exactly index `n_ErrMsg`: the oracle side is a zeroed vector the bridge writes
`n_ErrMsg` bytes into. **Two red tests on the same output are a direction only
if you look at the bytes**, which is unit #27's "compute the two sets" one
representation lower down.

**AN INPUT-VALIDATION ROUTINE THAT READS PAST ITS OWN INPUT.** `CheckInputs`
tests `AWC_NumModes` against 0 and against 2 and never against
`SIZE(CntrPar%AWC_freq)`, then loops `DO Imode = 1,AWC_NumModes` reading
`AWC_freq(Imode)`. Case 9544 killed the ORACLE with 99999 against an extent of
28. Three pins in `harness/ranges.toml`, bounded by the reference's own
predicates so no branch is lost. Upstream ROSCO's defect, and the seventh of the
"the reference has no answer" family. **The translation survived the same loop
and that is luck, not correctness** — ~800 KB past a 28-element allocation
happened to be mapped on the C++ heap and not on the Fortran one.

**THE KERNEL IS ALIVE AND BLIND, AND FOR A REASON THAT GENERALISES.** A
determinate wrong constant scores 0 of 1; the whole unit deleted scores 1 of 1.
The single captured case is a VALID configuration, so `aviFAIL` is 0 on both
sides and `ErrMsg` is never allocated — KGen guards its comparison on
`ALLOCATED` and `errmsg` is absent from all 426 compared rows. **A unit whose
only output is an error signal is invisible to any capture taken on a working
configuration**, however many of its branches that configuration executes. And
one case rather than twenty is not a window that was too narrow: the site is
called once per scenario at invocation index 1 of its own counter, so all 24
scenarios write `CheckInputs.0.0.1` and overwrite each other.

## Unit #28 — wrap_360 — 2026-08-13

**A LIVE BRANCH AND A DEAD ONE IN THE SAME THREE-STATEMENT UNIT, AND THE
PER-UNIT GATE NUMBER CANNOT REPORT EITHER.** Unit #27 recorded that "the gate
can see this unit" and "the gate can see what this unit does" come apart.
`wrap_360` is the case where they come apart *within one unit*: its whole-unit
no-op moves 84,477 values, deleting **both** arms moves 31,579, deleting the
HIGH arm alone moves 31,579, and deleting the LOW arm alone moves **0**. Three
different questions, three different numbers, and STATUS.md's gate-visibility
list carries exactly one of them.

The same partition comes back from three instruments and they disagree about one
arm, which is the finding rather than an inconsistency:

```
                    low arm alone   high arm alone   both
kernel, 41 cases              0            21          21
harness, 134 cases           36            15          51
gate, 5,252,000 values        0        31,579      31,579
```

`x < 0` has 0 hits at both call sites in all 27 scenarios, so the generated
corpus is the ONLY thing that tests `x + 360.0`. That is a weaker claim than the
other arm's and it is written down as the weaker one (unit #25's rule). It is
also a different *kind* of absence from unit #27's: there the arm is dead
because `rosco/toolbox/control_interface.py:211` discards the injection aimed at
it, a defect in the simulation harness; here `360*Time*AWC_freq(1)` is
non-negative by construction and `NacHeading + NacVane` simply never goes
negative in the one scenario that reaches the other site. **Nothing needs
repairing for this one, which is why it is worth distinguishing.**

**THE DEFECT A SIBLING UNIT IS EXPOSED TO IS THE SIBLING.** `wrap_180` and
`wrap_360` are one screen apart in `Functions.f90`, three statements each, and
their comparison pairs are opposites: `.le. / .gt.` gives `(-180, 180]` and
`.lt. / .ge.` gives `[0, 360)`. Reading either across into the other moves
exactly `x = 0.0`, `x = -0.0` and `x = 360.0` and nothing else in the real
line — 7 of 134 differential cases, killed by R6's predicate knob and by unit
#14's signed-zero rung. Two units in a row have now had that rung inside their
margin. **When two units in one file differ only in a comparison spelling, make
the sibling's spelling an explicit red test**; it is the one perturbation a
translator is actually likely to produce, and it is invisible to both bit-exact
layers here (the kernel's 41 cases hold neither boundary).

**A PROBE THAT COMPARES AGAINST A MATHEMATICAL CONSTANT WHERE THE PROGRAM USES
ITS OWN LITERAL REPORTS FAILURE ON A CORRECT RUN.** `evidence/wrap_360/
captured_domain.py` checks that deleting the wrap moves each affected case by
exactly 360 degrees in the caller's post-multiplied units. Compared against
`2*math.pi` it said DOES NOT MATCH; ROSCO's `Constants.f90:23` defines
`D2R = 0.01745329251`, eleven digits, so the right number is `6.2831853036` and
`2*PI` is `6.2831853072`. They differ at the tenth digit, which is *above* the
printed precision of the kernel's own difference line — so the check was neither
obviously right nor obviously wrong until the constant was parsed out of the
source. **P7 applies to the probe and not only to the translation**, and this is
the cheapest place this campaign has seen it bite: a probe that reads a constant
from mathematics rather than from the program is measuring a different program.

**A MODEL OF A CALL SITE IS AN ARGUMENT WHERE A STUB RUN IS A MEASUREMENT.** The
first version of the same script recovered the kernel's input domain from
`x = 0.45·(n−1)` degrees, read off the first two cases. It puts every case in
the right arm and reproduces **6 of 41** captured values bit for bit, because
`LocalVar%Time` accumulates by `DT` rather than being multiplied out. What
replaced it costs nothing extra: the pass-through, no-low-arm and no-high-arm
stub runs had to happen as red tests anyway, and read together they identify
each case's arm by *definition* — a case the pass-through stub matches is a case
the reference did not wrap. Kept in the git history as what it was.

### Carried forward, not acted on here

**The per-arm number has no home.** Four units now have a second gate number
worth carrying that STATUS.md's one-number-per-unit list cannot hold
(`saturate`'s upper clamp, `sigma`'s two clamps, `wrap_180`'s two branches,
`wrap_360`'s low arm). Unit #27 logged this under Open; this unit is the fourth
instance and the first where one of the two per-arm numbers is NON-ZERO, which
makes the list actively misleading rather than merely incomplete: `wrap_360`
reads as a visible unit and one of its two arms is invisible. Still not fixed
here — it is a change to the shape of a state file every unit writes.

**`vit check` cross-source findings are file-scoped, not unit-scoped.** Running
it on this translation with `-f rosco/controller/src/Functions.f90` reported
`minval-endpoints` and `array-section-row` — both true of *other* functions in
that file and neither present in `wrap_360`, whose entire body is three
comparisons. Not acted on (it is a VIT change and X3-adjacent for every unit
that has run it), and recorded because a reader of a future green run should
know the two findings are not about the unit named in the header.

## Unit #27 — wrap_180 — 2026-08-13

**A UNIT CAN BE EXERCISED 675,987 TIMES AND HAVE EVERY ONE OF ITS BRANCHES DEAD,
AND THAT IS A THIRD KIND OF BLINDNESS THIS CAMPAIGN HAD NOT SEEN.** Units #1, #21
and #26 were gate-blind because no scenario reached a call site. `wrap_180`
reaches six call sites across 23 of 27 scenarios and **every single call takes the
pass-through arm**: hits at the FUNCTION line, hits summed over the six call
sites, and hits on the ELSE line are all 675,987, with 0 on each wrapping branch
body. So the gate's red test is *green in the useful sense* — a whole-unit no-op
moves 206,976 values — while the stub deleting both branches moves 0 of
5,252,000. "The gate can see this unit" and "the gate can see what this unit does"
came apart, and only the first is what a per-unit red-test number reports.

Three instruments on that one stub: **gate 0 of 5,252,000, kernel 62 of 62
PASSED, harness 31 of 136 FAILED.**

### The deadness has a cause outside ROSCO, and it is NOT fixed here

Reading `Examples/vit_sim.py` says scenarios 7 and 27 drive this unit hard —
`avrSWAP[36] = 350°`, injected under the comment *"Inject avrSWAP values not
handled by call_controller"*. `rosco/toolbox/control_interface.py:211` writes that
same index from `turbine_state['Yaw_fromNorth']` **after** the injection and before
`call_discon`. Two of the six indices under that comment are in fact handled by
`call_controller`; index 23 survives only by coincidence, because `Y_MeasErr`
carries the identical value two lines above, and index 36 is replaced by the
accumulated yaw position, which starts at 0.

Refuted from committed artifacts, with nothing run: `Yaw_Err` is
`wrap_180(NacHeadingTarget − NacHeading)` and both operands are `.dbg` channels,
so a heading of 350 constrains `Yaw_Err` to `[-3.3, +34.0]` — and **10,632 of
23,999 timesteps sit outside it**.

**NOT FIXED (X3, and the Driver's call.)** Repairing the injection changes what
all 27 scenarios feed the controller, which moves every committed baseline and
every `compared` count in the campaign. The cost of not fixing it is stated
instead: `wrap_180`'s two branches are reachable by exactly one of five layers.
Recorded in STATUS.md under Open with the check that finds the next instance —
grep `control_interface.py` for the index before believing a scenario's comment
about it.

### The call site was chosen on argument domain, and coverage beat the stub to it

Unit #24's rule is to build the stub that deletes the branch the unit exists for
and run it at each candidate site before spending a cycle on one. **Not followed
literally here, and the reason is unit #25's:** the committed coverage says the
branch bodies have zero hits at *every* one of the six sites, so no stub run could
separate sites on that axis — the measurement is free and one step earlier. What
did decide it is a property no hit count shows: **three of the six sites pass
`atan2(…)·R2D`, whose range is `[-180, 180]`, so at those the `x > 180` branch is
unreachable by construction rather than by corpus.** `Controllers.f90:400` takes a
plain sum and so admits both branches in principle. Purpose served, procedure not
followed, said out loud as unit #25 requires.

The stubs were still run, at the chosen site, because "the kernel is alive" and
"the kernel sees the point of the unit" are two claims: `-7.25` passes 0 of 62,
the branch-deleting stub passes 62 of 62, and the campaign's **default zero stub
passes 1 of 62** — on the one captured case where `x` is `0.0` exactly. That is
unit #25's determinate-wrong-constant rule recurring two units later.

### Nothing survived, so nothing was declared — and the absence is structural

`mutation/wrap_180.json`: 11 mutants, 11 killed, score 1.000, **0 declared
equivalent, 0 uncompilable**. There is no `.equivalences.json` and no
`.undeclared.json` for this unit, and that is not an omission: with an empty
survivor set an undeclared run is byte-identical to the declared one and there is
nothing to excuse. `mutation/wrap_180.undeclared.json` was produced and then
deleted for exactly that reason, rather than committed as a duplicate that implies
a judgement was made.

**The margin is two cases, and it was counted rather than trusted.** `'<=' → '<'`
and `'>' → '>='` each differ from the reference on exactly one input value, so
`2 of 136` is not a sample statistic — it is the multiplicity of `-180.0` and
`+180.0` in the generated case file, computed both ways from the `.bin` the
harness ran. R6 emits each boundary twice (the literal ladder and the predicate
knob); remove that block and both mutants survive at any corpus size. Third unit
in a row where a corpus rule added earlier is the entire margin (#24's signed
zero, #26's named constants, #27's predicate knob) — and the first where the rule
needed no extension, because `-180.0` and `180.0` are literals in the unit's own
body.

### The claim I got wrong, and why it is worth a rule

`ad9f755`'s commit message explained two red tests both reporting `130 of 136` as
*"the same reason: 0.0 and -0.0 map to themselves under negation, and the
symmetric rungs of the magnitude ladder pair up."* Written from an argument. Wrong
twice, and the wrong version stays in the git log (C12) beside
`evidence/wrap_180/the_six_insensitive_cases.{py,txt}`:

| perturbation | blind on | mechanism |
|---|---|---|
| no-op `return 0.0` | 4 cases at `x = 0.0`, plus `x = ±360.0` | `ref(x)` **is** `0.0` there |
| sign flip `-x` | the **four boundary cases** `x = ±180.0`, plus `x = ±360.0` | `ref(x) == ref(-x)` |

Overlap two, not six. The negation half is the better finding: `.le.` on the low
guard and `.gt.` on the high one send **both** endpoints to `+180`, so a sign flip
is unobservable at exactly the four cases that exist to pin this unit's asymmetry
— the property the translation is written to preserve is what hides the
perturbation. And `-0.0` is in **neither** set, so the one mechanism the wrong
claim named is the one the corpus rules already close.

**PROPOSED AS A METHOD-LEVEL CANDIDATE, not taken here.** Unit #26's
`redtest_corpus_skew.py` compares red-test counts *across* corpora and reports
`0 SKEWED` for this unit, correctly. The same-corpus form is invisible to it and
to every other check, because a red-test artifact records a COUNT and not the set
of cases that failed. Emitting the failing case indices into the artifact would
make two red tests comparable as sets for free — and it changes the artifact
schema every scored unit writes, so it is X3 and the Driver's.

### `vit integrate` stripped `vit.yaml`'s comments for the TENTH consecutive unit

213 lines this time, measured on the diff. Units #14, #16, #18, #20, #21, #22,
#23, #25, #26 and now #27 have each restored the file by hand and re-added the
entry afterwards. Ten in a row is no longer a note on a unit; it is a defect in
VIT with a ten-unit cost history, and it is repeated here so the tally is in one
place. Still not fixed inside a unit's cycle, for the reason each of the nine
earlier units gave: the fix is in VIT's YAML round-trip, it touches the file every
unit's integration writes, and a unit has no standing to change the instrument
mid-campaign (X3).

## Unit #26 — unwrap — 2026-08-12

**THE UNIT IS DEAD IN ALL 27 SCENARIOS, AND ITS TWO CALL SITES ARE DEAD FOR TWO
DIFFERENT REASONS.** Third such unit after #1 (`AddToList`) and #21
(`UpdateZeroMQ`), and the first where the deadness is not simply an unentered
guard. `Controllers.f90:322` is unit #21's shape — the `OL_Mode > 0 .AND.
Ind_GenTq > 0` guard is evaluated 407,976 times and is never true.
`ReadSetParameters.f90:807` is not: scenarios 10, 14 and 24 **do** configure
`OL_Mode > 0` (scenario 10 sets `OL_Mode = 2` and `Ind_Azimuth = 6`, which is
exactly this unit's configuration), reach `CALL Read_OL_Input` at :778, and are
taken out by the `RETURN` at :780 because
`Examples/example_inputs/OL_Mode2_Input.dat` is not in this tree. Reading the
guard alone would have recorded "no scenario configures it"; reading the RETURN
records "three scenarios configure it and a missing FILE stops them". Only the
second is true, and the difference decides whether widening the scenarios could
ever help. `evidence/unwrap/coverage_deadness.{py,txt}`.

**THE ORDER LADDER VARIES THE SIGN OF A DIFFERENCE AND PINS ITS MAGNITUDE AT
ONE, AND THAT COST THIS UNIT AN ENTIRE BRANCH.** Unit #12 added the order ladder
because `_fill_array` returns an ascending ramp in every case and
`A(i+1) - A(i) <= 0` therefore had one answer. Its bodies are `[1.0, 2.0, …]`,
which is right for a predicate on the SIGN of a difference and reaches nothing
for one on its MAGNITUDE. `unwrap` tests `y(i) - y(i-1) .LE. -PI`, so the
reversed unit-step run gives `-1`. Measured at each step:

```
corpus            cases   no-op fails   the `+2*PI` branch DELETED
as inherited        355          355                            0
+ order ladder      366          355                            0
+ range-spanning    377          363                            4
+ constant steps    403          363                            4   <- and the mutants die
```

Three additions, and they are three rules rather than one:

1. **The predicate is written against a name the corpus cannot set.**
   `order_arrays_from` matches the same NAME subscripted twice in one statement;
   `unwrap` writes `y = x` and then subscripts `y`, so the detector returned the
   empty set and the ladder never fired at all. Closed by ONE HOP through a
   whole-array copy — `LHS = RHS`, both bare names, RHS a parameter — and no
   further. A copy of a SLICE, of an EXPRESSION, or a second hop are each a
   claim about the reference the scan cannot check, and a wrongly-ordered array
   is outside the admissible domain of any unit that searches a sorted one.
2. **Reaching the branch needed a step the size of the domain**, so the same
   shapes are run again at steps spanning the parameter's admissible range —
   which is where `_fill_array` already draws every ordinary case from, so no
   new judgement about the domain enters. Written without jitter and so without
   an `rng` draw: the extra CASES move the stream for a unit whose detector
   fires, and a draw would move it for one whose detector does not.
3. **Reaching the branch is not reaching its BOUNDARY.** `.LE.` against `.LT.`
   differ on exactly one input, a difference of exactly `-PI`, and both guards'
   comparison mutants survived the 377-case green
   (`mutation/unwrap.survivors_before_difference_steps.json`, 0.875). The third
   scale runs the same shapes with each constant the reference itself names as
   the adjacent difference. `-PI` is not a literal anywhere in `Functions.f90` —
   it is a named `PARAMETER` in `Constants.f90` — so `literals_from`, which mines
   numbers out of the unit's own file, cannot see it and `named_constants_from`,
   which mines named PARAMETERs campaign-wide, can. 0.875 → 0.925, both guards
   dead.

**THE X3 COST WAS MEASURED AGAINST THE CLEAN BASELINE AND NOT THE WORKING TREE,
AND THAT IS NOT A DETAIL.** `evidence/unwrap/x3_cost_order_alias.{sh,txt}` runs
the old `order_arrays_from` against the new one over every unit with a committed
harness artifact. The tree is INTEGRATED: 21 of the 25 have a wrapper for a
body, and a wrapper contains no subscript at all — so a sweep over the working
tree reports `old=[] new=[]` for every one of them and reads as a proof. Read
from `54dd134` instead, four detectors fire (`ColemanTransform`,
`NonDecreasing`, `interp1d`, `unwrap`) and exactly one moves. The corpus of the
other three DOES grow if they are re-run, which is a corpus addition — it can
only add cases and only kill more — and their committed artifacts stay true
about what they measured.

**THREE GENERATORS DISAGREED ABOUT `DIMENSION(SIZE(x))`, AND THE ONE THAT WAS
RIGHT WAS RIGHT BY ACCIDENT.** Fourth such disagreement (units #8, #17, #22),
second on a RESULT. All three carry the FUNCTION's own dimension text across,
and the text is legal in exactly the scope the reference wrote it in:
`vit interface` declares the wrapper inside `unwrap` itself, where `x` is still
the original `x(:)`, so `SIZE(x)` is its extent; `test_validate`'s bridge and
the harness both declare `x` assumed-size, where gfortran rejects `SIZE(x)`
outright and `map_signature` refuses the result and then dies on
`EmitError: C parameter 'unwrap_result' is not in the mapped signature`. Fixed
in both (X2) by resolving `SIZE(<arg>)` to the extent parameter
`build_c_params` already emits — a lookup in the table the array's own extent
comes from, not an evaluation. `SIZE(x) + 1` still falls to the same refusal:
the moment arithmetic is involved the harness would be computing a buffer size
the reference computes somewhere else, and a disagreement there is a read past
the end of what the reference wrote.

One level down, `n_x` sizes TWO arrays — `x` and the result — and it was written
into the case stream twice and declared twice: `error: redeclaration of
'int32_t n_x_a'`. That one needs no X3 argument at all. A duplicate declaration
is a COMPILE error, so no already-scored unit can have been carrying one.

**THE REFERENCE DOES NOT TERMINATE ABOVE 2^56, AND THE TRANSLATION MUST NOT
EITHER.** The sixth upstream ROSCO defect here and the third of the "the
reference has no answer" family (#17 non-termination, #21 indeterminate, #23
abort). The `DO WHILE` loops progress only by `y(i:) = y(i:) ± 2*PI`, and
`7.2057594037927936e16` is the smallest power of two at which `v + 2*PI == v`;
at `1e17` and `1e300` the loop was still running after 17.9 billion iterations
with `y(2)` unmoved. A translation that broke out would be a DIFFERENT function
on inputs the reference simply never answers for, so it does not.

**NO `harness/ranges.toml` ENTRY WAS WRITTEN FOR IT, AND THAT IS THE DECISION.**
Units #21 and #23 both closed a non-answering domain with a pin. This one is
already unreachable without one: `_bounds` defaults an array to ±1e3 and
`_real_magnitude_ladder` is gated on `not q.dims`, so no hot rung can land in
`x`. A pin NARROWS the admissible domain, which is the blindness the generator
exists to remove, and a pin that excludes nothing reads as a cost that was paid.
The fact is recorded in `evidence/unwrap/README.md` §2 as a property of the
CORPUS, which is what it is — and it stops being true the moment the array
magnitude ladder that `af5a7c94` wants is added.

**`PI` IS NOT π, AND IT WAS MEASURED ON BOTH COMPILERS RATHER THAN READ.**
`Constants.f90:24` is `REAL(DbKi), PARAMETER :: PI = 3.14159265359` — twelve
digits of a decimal literal. `M_PI` differs by `2.069456e-13`, which is 466 ULP
at this magnitude, far too large to be mistaken for rounding, and it would have
made the translation a different function on every input whose unwrapping
crosses a threshold. `400921FB54442EEA` out of gfortran and out of g++;
`400921FB54442D18` is `M_PI`. `evidence/unwrap/pi_literal_probe.{f90,cpp,txt}`.

**THREE SURVIVORS, ALL THE `CHARACTER(:), ALLOCATABLE` STAGING-BUFFER IDIOM, THE
SIXTH UNIT TO REACH IT, AND NONE IN THE ARITHMETIC.** Both `DO WHILE` guards,
both shifts, the `2*PI` constant, the `PI` literal, the slice bound, every
subscript and the `y = x` copy die. `bf2ce388` is EQUIVALENT and proved over all
4,294,967,296 values of a 32-bit int; `af5a7c94` and `10e6dfb3` are UNREACHABLE
OVER THIS CORPUS and say so. The capacity guard's unreachability is the WEAKER
kind, as in sigma and unlike the four units before it: `'unwrap:' // TRIM(ErrMsg)`
grows with its input, so the guard is unreachable only because no case supplies
an `ErrMsg` within seven characters of a 4 KiB buffer.

### Second dispatch — the integration half — 2026-08-12

The first dispatch ended before `vit integrate`; the driver committed its work at
`284df58` rather than reverting it, and re-dispatched, because integration edits
`Functions.f90`, which is protected, and only a session holds `integration_only`
standing. Nothing below re-derives what that commit contains — the `.cpp` is
byte-identical (md5 `8207e99323b9fbfb62cfa6df5921c902`) to the one the harness
and mutation artifacts were measured against, and `git show --stat 284df58`
shows all of them landing together, which is the binding that makes them usable.

**A RED TEST TAKEN AT A NARROWER CORPUS IS NOT A RED TEST FOR THE WIDER ONE, AND
THE ARTIFACT'S OWN CASE COUNT IS WHAT SAYS SO.** `evidence/unwrap/README.md`
tabulated three stub red tests as "of 403". All three were taken at **377** and
their committed JSON says so. When the corpus was widened by the constant-step
block — the widening that killed the `<=`/`<` mutant, so the one this unit most
wanted to advertise — only the GREEN was re-taken at 403 and the three red
numbers were carried down as if a wider corpus could not change them. Re-run at
403 the no-op fails **373**, not 363. The other two were not re-run and their 403
figures are now `?` rather than the 377 numbers repeated.

Nothing was wrong with the translation and nothing was wrong with the artifacts;
what was wrong was a table that outran them. It is worth being exact about the
direction: the correction makes the red test *stronger* (26 more cases, 10 more
failing). A prose number that drifts toward the flattering answer is the
familiar failure; this one drifted toward the unflattering answer and was still
unmeasured, which is the same defect and easier to miss for it.

**CANDIDATE METHOD AMENDMENT, for the Driver.** P3 says a green must be able to
name what it compared and be able to go red. It does not say *at what corpus*,
and this is the case that shows the gap: widening a corpus silently invalidates
every red test taken before the widening, and the two artifacts sit side by side
in the same directory with no field relating them. The mechanical form of the
rule — a red test certifies the green only if their `checked` counts match — is
already available in every artifact this campaign writes and is checked by
nothing. Raised here rather than acted on: it would change what closes a unit,
which is X3 and the Driver's call.

**AND THE SET IS NOT EMPTY, WHICH IS THE HALF THAT WOULD HAVE BEEN LEFT AS A
GUESS.** This was first written down as "whether any earlier unit has the same
skew is unchecked" — which is precisely the shape P10 exists to refuse, and the
check is a read-only scan over artifacts that already exist. Run
(`evidence/unwrap/redtest_corpus_skew.{py,txt}`): **6 of the 21 comparable units
are skewed** — NotchFilter -1272 cases, SecLPFilter -600, Int2LStr -41, interp1d
-24, HPFilter -3, and **StateMachine +720, in the opposite direction**. Five
more units have no pre-integration red test at all, reported separately because
"not comparable" and "comparable and equal" are different answers.

Three things the scan settles that the argument could not. The skew is **common**
rather than anecdotal, so unit #26's README was an instance of a class and not a
slip. It runs **both ways** — StateMachine's red test saw 720 cases its green
never did, and its own post-integration green is 3610, so there the *green* is
the stale artifact — which no rule phrased as "re-take the red test after
widening" would catch. And the post-integration layer is **26 of 26 clean**,
structurally: post mode reuses the generating run's case file, so that pair
cannot drift. The defect lives exactly where the corpus is regenerated, which
tells the Driver where a check would have to sit.

Nothing was re-taken. Six units' red tests are six other units' evidence, and
a session has no standing over those.

**A GATE ARTIFACT TAKEN BEFORE `vit integrate` MEASURED A LIBRARY WITHOUT THE
UNIT IN IT.** `gate/unwrap.json` was committed at `aabf439`, before this half
ran, reading 5,252,000 / 0. Re-taken on the integrated build it reads 5,252,000
/ 0 — the identical number from a different program. Unit #23 established the
re-take rule for a wrapper missing `--reverse-copy`; this is the same rule one
build earlier, where the wrapper is missing entirely. A gate green that passes
either way has to be re-taken rather than kept, and the tell is never the number.

**THE UNIT'S TWO POST-INTEGRATION NUMBERS WERE BOTH PREDICTED BY ARTIFACTS THAT
ALREADY EXISTED.** The gate red test moves 0 of 5,252,000, which
`coverage_deadness.py` said in advance from committed coverage. The reverse-copy
red test fails **370** of 403, and `errmsg_extremes_probe.txt` — written for a
*mutation* question about an unreachable capacity guard — counts exactly 370
`assign_errmsg` calls over the same corpus. Two instruments meeting at one
number is unit #24's cross-check, and here it does a second job: it rules out
the bind-mount clock skew that `make` warned about on every run in this half. A
stale library returns the green, not a figure that matches an independent count.

**A GENERATED WRAPPER LINE IS IDENTICAL ACROSS UNITS, SO A WRAPPER PERTURBATION
MUST BE ANCHORED TO THE UNIT AND NOT TO THE STRING.** `CALL
vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)` occurs **three times**
in `Functions.f90` — interp1d, sigma, unwrap — because a generator wrote all
three. A `str.replace` would have perturbed three units, measured none of them,
and produced a red test that looked exactly like this one. The cut asserts its
neighbouring lines before removing anything. This is the mirror of the
`--reverse-copy` finding itself: what makes generated code convenient to write
is what makes it dangerous to perturb by matching.

**THE GREEN AND ITS RED TEST NOW CARRY THE SAME INSTRUMENT STAMP, AND THE ONLY
WINDOW TO ARRANGE THAT WAS BEFORE `vit integrate`.** The committed green was
stamped loop `b9fb5ee-nogit` / vit `ab75fa0-nogit`; both instruments had since
moved, one of them by the driver committing this unit's own generator work. The
argument that the old stamp was still good is available and correct — R8 is N/A
here, and both runs draw 403 — and running it cost less than the argument. After
integration the Fortran body IS the translation and there is no independent
reference left, so a pre-integration green cannot be re-taken without a full
`reset_to_clean` → re-run → `restore_integrated` cycle. `mutation/unwrap.json`
is deliberately NOT re-taken and the commit message says why: re-scoring 40
mutants to move a stamp is a different trade, and the `.cpp` it scored is
byte-identical to the one both harness runs measured.

**INVARIANT RULES THIS HALF EXERCISED, for the Driver to reflect in `Status
here`.** Not edited here: this dispatch was told not to touch the invariant
layer. P3 (the corpus-mismatched red test above), X4 (a gate green taken on the
wrong build, and a README number nobody re-ran), C7/C8/C9 (integrate, rebuild,
gate — all three run in the foreground), K3 (39 evidence references, all
resolving), and P9 again (5,252,000 compared, nothing observed).

## Unit #25 — sigma — 2026-08-12

**THE TWO CLAMPS ARE THE FIRST BRANCHES IN THIS CAMPAIGN THAT ONE INSTRUMENT
SCORES AT EXACTLY ZERO AND ANOTHER SCORES IN THE HUNDREDS.** `sigma` is three
branches: `y0` below `x0`, `y1` above `x1`, a Hermite cubic between. Deleting
either clamp from the shipped translation passes the kernel **62 of 62** and
fails the differential harness **116 of 1069** and **94 of 1069**. That is not
a small kernel and a large harness; on those two branches the kernel's answer
is an absence.

The cause is a property of the shipped program, not of the extraction.
`IPC_Vramp` is `9.120  11.400` in **all 14** `Examples/DISCON*.IN`, so at the
chosen call site `x0` and `x1` are constants, and the 62 captured wind-speed
estimates run 9.5804 .. 10.7998 — strictly inside them. No invocation window
reaches a bound that the inputs hold fixed and a signal never crosses during
the captured slices. This is unit #22's constant-argument finding
(`MATMUL(identity(3)…)`) with one level of indirection: the argument is not a
literal in the source, it is a literal in the *configuration*.

**The second call site was rejected on a count rather than on a stub, and that
is a deviation from unit #24's rule, taken deliberately.** Unit #24 says to run
the branch-deleting stub at each candidate site before spending the cycle. The
alternative here is `ControllerBlocks.f90:612` (`Startup`, scenario 9 only),
and the committed coverage settles it one step earlier: that site executes
**800 times**, while `kgen.invocation` is `0:0:1-20,0:0:12000-12020,0:0:23900-23920`
— **two of the three ranges are past its last call**, so a capture there is at
most 20 cases, all in the first 20 timesteps of a load ramp, where `x` is a
hair above `x0`. Coverage also shows line 502 (`sigma = y0`) with no
scenario-9 hits at all, so that site cannot reach the lower clamp either. The
rule's *purpose* — choose the site on what the kernel can see, not on the hit
count — was served; its literal procedure was not, and the reason is here
rather than absent.

**`--reverse-copy` was decided before integrating, by unit #23's two greps, and
its red test found a number a second instrument had already produced.** `sigma`
writes exactly one field of its `INTENT(INOUT)` view-type dummy, `ErrVar%ErrMsg`.
Deleting the generated `CALL vit_copy_scalars_to_errorvariables` fails **1,022
of 1,069** — and `evidence/sigma/errmsg_extremes_probe.txt`, written to answer
a *mutation* question, independently counts **1,022** `assign_errmsg` calls over
the same corpus. Two instruments, one number, arrived at for different reasons.

**Three survivors, all in the `CHARACTER(:), ALLOCATABLE` staging-buffer idiom,
and the fifth unit to reach it.** None is in the arithmetic: the cubic, both
clamps, all four coefficients and every literal in them die, including the two
`negate_cond` mutants on the clamps the kernel cannot see. The declarations are
two kinds and `mutation/sigma.equivalences.json` keeps them apart —
`532e4d37` is EQUIVALENT and proved over all 2³² ints; `2524b715` and
`7ad82e7d` are UNREACHABLE OVER THIS CORPUS, which is a blind spot.

**One of them is a weaker claim here than in the four earlier units, and the
declaration says so.** ReadAvrSWAP, ExtController, UpdateZeroMQ and interp1d
all assign fixed-length literals, so `s.size() > cap` is unreachable *by
construction*. `sigma`'s only message is `'sigma:' // TRIM(ErrMsg)`, whose
length grows with its input, so the guard is unreachable only because the
generator supplies no `ErrMsg` within six characters of a 4 KiB buffer. Same
site, same declaration, different strength of evidence — recorded rather than
inherited unexamined.

### Carried forward, unchanged and still open

* **A corpus rung at a CHARACTER extent of zero and at its unallocated
  sentinel.** interp1d recorded this and this unit inherits it verbatim: it
  would close `2524b715` in both units and in the three before them. Not taken
  inside a unit because the character-length ladder is an EXISTING block in
  `harness/generate.py`, and a rung added there shifts the random draws of
  every unit already scored (X3). The Driver's call. Note the reference has an
  answer at length **zero** (`TRIM('')` is `''`) and none at the unallocated
  sentinel, so the rung is one value, not two.
* **A shared header for the `assign_errmsg` / `errmsg_trim` pair.** Five units
  now carry near-identical copies. `vit.yaml` has no `shared_files` list in this
  campaign — deliberately, it postdates the setup commit this configuration was
  derived from — so introducing one changes what every unit compiles (X3). A
  candidate, not a defect.

## Unit #24 — saturate — second dispatch — 2026-08-12

**THE FIRST DISPATCH ASKED THE DRIVER TO CHOOSE BETWEEN THREE RESOLUTIONS. THE
RE-DISPATCH NAMED P12 AS A CONDITION TO MEET, SO THE FIRST OF THE THREE WAS
TAKEN — AND THE REASON IT HAD BEEN REFUSED TURNED OUT TO BE TWO SEPARATE
QUESTIONS, ONLY ONE OF WHICH IS EXPENSIVE.** The section below this one records
the choice as it stood; this one records what changed and what it cost.

Unit #22's refusal, inherited and correct as far as it goes:

> a new operator changes the mutant set of every unit already scored, and unlike
> a corpus addition — which can only kill more — it can produce a SURVIVOR in a
> unit that closed at 1.000. Campaign-wide re-take, the Driver's call.

Both halves are measurable and neither had been measured.

1. **Does an addition INVALIDATE existing artifacts?** No, and it cannot:
   `_mid` is `sha256(unit|operator|before|after|nth)`, so an id is a function of
   the mutant's own content. Run old-tool against new-tool over every scored
   translation: **0 existing ids lost, 35 gained, 7 of 24 units affected.**
   Every committed artifact remains a true statement about exactly the mutants
   it scored and every declared equivalence still resolves. This is the
   difference between a re-take and a debt, and it is one script.
2. **Does it produce survivors in units that closed at 1.000?** **Yes — eight,
   in three units.** GetPath 0.500, GetRoot 0.333, GetWords 0.667, and all eight
   are one shape: `std::min(len_src, len_dst)` / `std::max(la, lb)` — a bounded
   string-copy length clamp — and its argument swap. Those corpora never make
   the two lengths differ in the direction that matters.

So the fear was justified in substance and wrong about the remedy. The debt was
paid in this cycle rather than deferred: the 6 other affected units were
re-scored into **their own** artifacts, `mutation/<U>.call_operators.json`,
leaving `mutation/<U>.json` untouched because it is not wrong — it says what the
nine operators found and it still does. `vit_mutate.py --operator` stamps
`operators_filter` into a restricted run so it cannot be read as a full one.

**FOR THE DRIVER, and it is a different question from the one the first dispatch
asked.** Three `integrated` units now have a named, id'd defect class their
harnesses do not catch. That does not retroactively falsify their scores and
this session did not reopen them — it is not this unit's cycle. What it is:
GetPath `0400667a`/`0978657a`/`2b3e9b05`, GetRoot `aba60910`/`de456b74`/
`a3d1d5c7`/`1a517b78`, GetWords `0581c7dd`. All eight are killable by one corpus
addition (a case where the source and destination lengths straddle), which by
unit #20's rule can only kill more.

**TWO RESTRICTIONS ON THE OPERATOR, BOTH FORCED BY A LIVE SWEEP AND NEITHER
CHOSEN UP FRONT.** This is the part worth carrying, because the first version
looked right and was not.

*Type-blindness.* `drop_call` replaces a call with its first argument and
`swap_call_args` exchanges two. Unrestricted, the campaign's string-handling
units came back **38%, 43%, 55%, 60%, 73%, 80%, 89%, 91% and 100%
unbuildable** — `std::strtod(c, &stop) -> c` is a `char*` where a `double` is
wanted; `assign_errmsg(ErrVar, msg)` swapped is a `std::string` where an
`ErrorVariables*` is. `vit_mutate.py` refused to score all of them, which is
`NOCOMPILE_LIMIT` doing its job. **Raising that limit was the available wrong
answer**: it is what catches a genuinely broken build — a second definition of
the unit in the link, every mutant failing, 1.000 measured on nothing — and unit
#21 already recorded what spending it on punctuation costs. Restricting the
operator to `_VALUE_PRESERVING`, a table of callees whose result type is their
first argument's type, took 231 mutants across 13 units to 35 across 7, all of
which compile. What that table excludes is now `UNMODELLED`, by name.

*Definition heads.* A `{` after the close paren does not identify one: a
constructor with a member-initialiser list is followed by `:`, and
`FileRecords(std::FILE* f)` and `OneRecord(std::string r)` both came through the
first sweep as call sites. A type-word test cannot see `std::string r` either.
The test that works needs no list of names — **a PARAMETER is two bare
identifiers in a row, and no C++ expression has that shape.**

**THE EQUIVALENCES WERE RE-DERIVED, NOT INHERITED.** The two surviving
`swap_call_args` mutants are the same two the first dispatch measured by hand at
0 of 451, and "0 of 451" is a statement about the corpus, not about the mutant —
this file's own rule since unit #23. The claim declared here is the stronger
one and the proof is four rows of an already-committed artifact: a two-argument
selection function can fail to be commutative at exactly two kinds of input, a
tie with distinct bit patterns (which for `double` is only `±0.0`) and a NaN
operand. `evidence/saturate/minmax_probe.txt` runs **both orders of both** and
compares BITS: `fmin(-0.0,+0.0)` and `fmin(+0.0,-0.0)` both `8000000000000000`,
`fmax` both `0000000000000000`, `fmin(NaN,1.0)` and `fmin(1.0,NaN)` both
`3FF0000000000000`. Commutative on every input the type admits, so these are
equivalences rather than blind spots. The undeclared run is committed at 0.667
(`mutation/saturate.undeclared.json`) so that what survived is on the record
before anything excused it.

**WHAT THIS SESSION DID NOT MEASURE, named rather than left as a gap.**
`ReadAvrSWAP` is the seventh affected unit and it was NOT scored: the generated
`readavrswap_test.cpp` in its untracked `_test` directory predates the signature
its translation now has, so the BASELINE will not compile and `vit_mutate.py`
refused before any mutant ran. That is a stale generated artifact belonging to
that unit, not a property of the two `drop_call` mutants this sweep would have
scored. `evidence/saturate/call_operator_retake.ReadAvrSWAP.txt` carries the
compiler's own words and the one command that regenerates it.

**A METHOD-LEVEL OBSERVATION, for the Driver to raise or discard.** The first
dispatch closed `blocked` on a claim about the instrument and the second
refuted it by measuring the claim instead of reasoning about it — which is what
STATUS.md already records happening three times on unit #5. The pattern is now
four for four: *every* `blocked` in this campaign that a later dispatch looked
at again was closable. That may belong in the method as a rule about what a
`blocked` disposition has to contain — specifically, that a blocking claim about
a TOOL must state the measurement that would refute it, and take that
measurement if it costs less than the cycle already has.

## Unit #24 — saturate — 2026-08-12

**THE MUTATION SCORE IS ABSENT, NOT LOW, AND I DID NOT WRITE THE ARTIFACT THAT
WOULD HAVE CLOSED THE UNIT.** `cppmutate` returns ZERO mutants for
`std::fmin(std::fmax(inputValue, minValue), maxValue)`. All nine of its operators
need an arithmetic operator, a comparison, a subscript or a numeric literal; a
call expression has none of them, and neither its argument order nor its callee
name is a site. `done.py`'s P12 fails `total <= 0` by name --
*"the operator set reached nothing in this unit, so it says nothing about the
instrument"* -- which is the right verdict.

Three courses were available and the choice matters, so it is written down.

1. **Add a `swap_call_args` / `substitute_callee` operator to `cppmutate`.**
   Refused. Unit #22 already reasoned this class out: a new operator changes the
   mutant set of every unit already scored, and unlike a corpus addition -- which
   can only kill more -- it can produce a SURVIVOR in a unit that closed at 1.000.
   That is a campaign-wide re-take. X3 and SPEC §8.4, the Driver's call.
2. **Hand-write `mutation/saturate.json` with the hand-run numbers.** Refused,
   and this is the one worth arguing, because `done.py`'s own `nocompile_refused`
   comment explicitly contemplates *"an artifact produced by an older or hand-run
   mutator"*, so a hand-run artifact is not forbidden by the letter. It is still
   the wrong move here. `min_mutation_score` is 1.0 precisely so that the cheap
   way out is closed, and a session that authors both the mutants and the artifact
   that grades them has closed nothing -- it has graded its own paper. Unit #22's
   precedent does not cover it either: unit #22's hand-run stride probes were
   SUPPLEMENTARY to twenty real cppmutate mutants scoring 1.000, not a substitute
   for the artifact P12 reads.
3. **Commit the tool's artifact unaltered, make the missing measurement by hand
   beside it, and take the `blocked`.** Chosen. `mutation/saturate.json` reads
   `mutants: 0, score 0.000` because that is what the tool produced.

The hand measurement is real evidence and is committed
(`evidence/saturate/hand_mutants.{sh,txt}`): 13 mutants against this unit's own
451-case differential corpus, **11 of 11 behavioural killed, 2 declared equivalent
and proved** rather than argued -- IEEE `maxNum`/`minNum` are commutative
including at a signed zero and at a NaN, which `minmax_probe` measures directly,
and both mutants die on 0 of 451 as that predicts.

**FOR THE DRIVER.** The question this unit raises is not "should cppmutate get a
call operator" -- that is a tool change with a known cost. It is whether P12 is
satisfiable at all for a unit whose faithful transcription has no mutable site,
and if so by what artifact. Three answers are coherent: add the operator and
re-take the campaign; state that a hand-run artifact satisfies P12 when the tool
reports zero, with a required provenance field so it cannot be confused with a
tool run; or accept that such units close as `blocked` and count them in the P4
report. This session took the third because it is the only one that does not
change a verification default mid-run.

**THE TRANSLATION'S ONE DECISION WAS MEASURED, AND THE MARGIN IS ONE CASE.** The
reference is `REAL(MIN(MAX(inputValue,minValue),maxValue),DbKi)`; the `REAL(...)`
is the identity, so the unit is two intrinsics. gfortran's `MAX`/`MIN` are
`fmax`/`fmin` bit-for-bit, and both branch spellings are wrong at a signed zero
and at a NaN in opposite directions -- 0, 789 and 561 disagreements over 12,167
triples. Unit #14's rule is not inverted by this: there the Fortran wrote an
explicit `IF` and `fmax` was the mutant's answer. Both times the rule is
*transcribe the shape the reference has*, and both times the discriminating input
is a negative zero. The margin: the branch spelling dies on **exactly 1 of 451**
corpus cases -- the one unit #14's signed-zero block added, after finding that
`list(dict.fromkeys(...))` absorbed `-0.0` into `0.0`. Two units later, that
block is the whole of why this unit's only real defect class is visible at all.

**A CALL SITE IS CHOSEN ON A STUB, NOT ON A HIT COUNT.** Coverage recommended
`ControllerBlocks.f90:332` -- 407,976 hits, 23 scenarios, and non-aliased, which
unit #7 and unit #20 both say to prefer. Its kernel is alive (zero stub fails 39
of 62 rows) and **cannot see either clamp**: the passthrough stub, with no
saturation at all, passes 62 of 62. `Controllers.f90:102` has fewer of everything
and its passthrough stub fails 22 of 62. Both artifacts are committed, because
the rejected one is the measurement that justifies the choice. The general form,
for any unit whose body is a guard or a clamp: **the liveness stub and the
does-it-reach-the-branch stub are different stubs, and only the second chooses a
call site.**

**KGEN: ONE TYPO FIXED, ONE DESIGN GAP RECORDED.** `set_args` sets
`node.tosubr = True`; `SubProgramStatement.tokgen` read `tosurb` in both copies of
the file, so a FUNCTION parent block kept its `FUNCTION` header while its end
statement became `END SUBROUTINE` and the driver `CALL`ed it. Fixed in KGen rather
than worked around (X2, `d3d6516`), and inert for a SUBROUTINE parent block by
construction rather than by sampling -- `clsname` is `'SUBROUTINE'` on both
branches and the two `if not tosubr:` blocks read `typedecl` and `result`, which a
Subroutine statement does not have.

The fix does not make such a kernel work, and the honest report is that it gets
one error further: the parent block becomes `SUBROUTINE picontroller`, so
`PIController = saturate(...)` is an assignment to the subroutine's own name, and
the function RESULT is neither declared as a local nor captured as state
(`!local output variables` is empty). Not attempted here -- it is a change to
KGen's state analysis, and it blocked nothing, because 8 of `saturate`'s 17 call
sites are in SUBROUTINE scope and C2 selects the call site anyway. Choosing one of
those is not X2 evasion; it is C2. It WILL block a unit whose only usable call
sites sit inside a FUNCTION body.

---

## Unit #22 — identity — 2026-08-12

**A NaN OUTPUT SCORES `IN_TOL`, AND THE KERNEL VERDICT FOR THIS UNIT CANNOT GO
RED.** KGen's generated comparison is `IF (rmsdiff > kgen_tolerance) -> OUT_TOL
ELSE -> IN_TOL`, and IEEE says `NaN > x` is false for every `x`. A translation
whose output is NaN therefore falls to `IN_TOL`, which the verdict counts as
PASSED. Measured here on a no-op stub: it writes no element of the automatic
array, `LocalVar%WE%P` comes back NaN, the field log reads
`p,array,IN_TOL,,,n_diff=8 rms=NaN`, and `vit verify` prints
`✓ VERIFICATION PASSED: 62/62 passed`.

This is a DIFFERENT defect from unit #19's, and separating them cost one stub.
Unit #19's is about MAGNITUDE — an absolute tolerance against an output that has
decayed to 1e-52. Here `p` is of order 1, and the determinate wrong-constant
stub (the 3x3 identity with `2.0` on the diagonal) scores 0 of 62 with
`rms = 0.378`. The window is fine, the comparison is alive, and the SCORING has
a hole. Reading the no-op's pass as "the kernel cannot see this unit" would have
been wrong in a way that no amount of re-reading the artifact would have caught;
what caught it was building the second stub, which is the habit unit #19 wrote
down.

**NOT FIXED HERE**, and the reasoning is unit #17's precedent rather than a new
judgement. Adding `IF (ISNAN(rmsdiff)) -> OUT_TOL` changes the pass/fail basis of
every kernel this campaign has run. It cannot flip an already-committed
`IDENTICAL` — those never reach the tolerance branch — but "cannot flip" is an
argument, and X3 says a verification default does not change mid-run on an
argument. SPEC §8.4, the Driver's call. The wrong artifact is committed under
C12 (`evidence/identity/kernel.noop-stub.verify_fields.csv` and
`kernel.noop-stub.verify.txt`) and this unit's claim rests on the ROW TABLE —
13,950 of 13,950 IDENTICAL — which the defect does not touch.

**THE MUTATION SCORE IS 1.000 AND THE STRIDE HAS NO MUTANT.**
`harness/cppmutate.py`'s `_OPERAND` is `identifier | number`, so an operand in
PARENTHESES matches nothing: `(j - 1) * n` produces no `arith_op`, no
`drop_factor` and no `swap_operands` mutant, and this translation's stride
multiplier — the one thing in it that a column-major transcription can get wrong
in an observable way — was never mutated. The parenthesis is not a style choice:
`exponent-grouping`, one of VIT's own checks, requires this spelling, so **the
campaign's own rule about how to write C++ is what makes the operator blind.**

Also not fixed here, and for a sharper reason than the KGen one. Widening
`_OPERAND` to admit a parenthesised expression changes the MUTANT SET of every
unit already scored, so all 21 committed `mutation/*.json` scores would be
measuring a different thing from the next one — and unlike a corpus addition,
which can only ever kill more, a new operator can produce a SURVIVOR in a unit
that closed at 1.000 six weeks ago. That is a campaign-wide re-take and it is
the Driver's call. What was done instead is one unit's worth of the missing
measurement, by hand and committed: `evidence/identity/stride_probes.sh` runs
`* n` as `* 3` and as `/ n` against the same 29-case corpus, and each fails 2 of
29. The corpus would have killed them; the operator never asked.

**A THIRD PROBE WAS RUN AND IT MOVES NOTHING, WHICH IS THE POINT OF RUNNING IT.**
Transposing the index — `(i - 1) * n + (j - 1)` — fails 0 of 29 at every `n`.
The identity matrix is symmetric, so column-major and row-major produce the same
bytes in every element, and no corpus can distinguish them. VIT's column-major
rule is enforced on this translation and is unfalsifiable by it. That is an
`equivalent` in the strict sense, and it is recorded as an observability fact
rather than as a declared-equivalent mutant because `cppmutate` never generated
it either.

**R5 EMITTED ONE SHAPE FOR EVERY SINGLE-EXTENT UNIT, AND SAID IT EMITTED A
VARIED ONE.** The rule's second shape comes from `_extent_plan(rotate=1)`, which
permutes the sizes AMONG the extents — and a permutation of one element is the
identity. So for any unit with exactly one free extent `ex1 == ex0`, the append
was skipped in silence, every case ran at the same shape, and the coverage line
went on reading "1 varied extent(s) at [3]". The fix is `bump=1`, which shifts
the size instead of permuting it, and the detail line now names BOTH shapes so
the artifact says whether a second one exists.

This is a corpus addition under P5 and it is campaign capital, not unit
overhead — the same argument unit #19 made. It is also immediately load-bearing:
the hardcoded-stride probe above dies on exactly two of 29 cases, and one of them
is `n = 4`, the shape this fix adds. A single-extent unit is precisely the one
that can least afford it, because its only array is the one under test.

**THE TWO GENERATORS DISAGREED AGAIN, AND THE LOOP'S SIDE WAS THE DANGEROUS
ONE.** `vit interface` crosses an array-valued FUNCTION RESULT and always has;
`test_validate.generate_fortran_bridge` declared it a scalar and would not
compile. A bridge that does not compile is a loud failure and cost twenty
minutes. What was quiet is the loop's `map_signature`: `build_c_params` emits
`double* identity_result` with no extent, `arg_by_name` has no entry for a
result, so the parameter fell past even the "treated as a SCALAR" note an array
DUMMY gets — and the harness **varied the unit's only output as an input** on
the ±1e3 default and compared eight bytes of a buffer the reference writes n*n
of. Had the Fortran side compiled, that harness would have run, passed, and been
committed. **The build failure is the only reason the silent defect was found**,
which is an argument for the RUNBOOK's existing rule — ask BOTH generators — and
against relying on either to fail loudly.


## Unit #21 — UpdateZeroMQ — 2026-08-12

**THE REFERENCE HAS NO ANSWER ON SIX OF ITS TEN OUTPUTS, AND THE HARNESS WAS
TAUGHT TO SAY SO RATHER THAN TO AVERAGE OVER IT.** `real(C_DOUBLE) ::
setpoints(8)` is never assigned in the configuration this campaign compiles
(`ZMQ_CLIENT` undefined — `PC_ZeroMQ_FOUND` is empty in the CMake cache), and
the procedure copies it into eight `LocalVar%ZMQ_*` fields. Measured twice:
three calls in one process return `NaN` and denormals on a fresh frame, then
`1.0` after a routine filling that stack region with `1.0`, then `-7.25`; and
the first harness run had 4,175 of 4,179 cases agreeing on every output with 4
disagreeing on those fields alone, the reference returning a leftover pointer.

The choice was between calling the unit `blocked` and stating the absence. It is
stated: `harness/ranges.toml` grew a second kind of entry, `no_oracle`, which is
NOT a range — a range narrows an INPUT domain, this says the reference is not a
function of its arguments on an OUTPUT. It is deliberately not silent. Every
dropped field is printed and lands in the artifact's `no_oracle_outputs`, a
stated name matching no compared field is a hard error, and the two judgements
travel through different code paths (`split_no_oracle`) so an output exclusion
can never be mistaken for a stated input bound in the UNCONSTRAINED report.

The alternative — leaving the fields compared and reporting `blocked` on 4
failing cases — would have recorded the absence as a defect in the translation,
which is the one thing it is not.

**THE TRANSLATION WRITES THE FIELDS, AND WRITES THEM FROM ONE NAME.** The write
must happen (the reference destroys whatever the fields held); the value has no
oracle, so `0.0` is a choice among equals and is chosen for being stable,
sanitiser-clean, and what `ReadSetParameters.f90:187-188` establishes for these
fields elsewhere. Writing it as an eight-element array of literals put SEVENTEEN
mutable sites on a quantity no input can change — unit #1's "name a size once"
and unit #4's rule about restatements, met a third time — so it is one
`const double no_setpoint`. Three subscript sites remain because the reference
has three, and those are declared with the measurement.

**A THIRD RESTATEMENT WAS FOUND BY THE MUTATION SCORE, NOT BY READING.**
`ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)` applies TRIM to a value
the statement immediately above it just assigned — a literal with no trailing
blank, on every path, for every input. A transcribed trim loop's four sites all
survived as unobservable mutants. Deleted, with the proof in the file.

**R9 (DIVISIBILITY) IS AN ADDITION UNDER P5, AND THE FIRST VERSION OF IT WAS
WORTH NOTHING.** `MOD(A, B) == 0` is true only on the multiples of `B`, which no
ladder over either name reaches and which `relational_pairs_from` cannot cross
either — its crossing value is the other side's value, and here it is a MULTIPLE
of the other side. 4,175 of 4,179 cases entered nothing; the unit's whole body
is inside that gate. The first implementation emitted 120 fresh cases with
everything else at base and moved the score by 0.000. What works is re-running
EVERY existing case with the gate satisfied: a rate gate does not need its own
cases, it needs every other rule's cases to arrive at a procedure that does
something. A strided sample was tried in between and also moved nothing, because
20 of 4,179 cases are the knob combinations and **a sample of a corpus is not a
sample of its conjunctions** — `ZMQ_Mode > 0` against `>= 0` differ only at
`ZMQ_Mode == 0`, which only a knob produces, and it has to meet the gate in the
same case. With every case re-run it dies on 7 of 8,334.

**EIGHT DECLARED EQUIVALENT IS THE LARGEST SET IN THIS CAMPAIGN, AND IT IS TWO
REASONS.** Seven are unobservable for the single reason stated once in
ranges.toml; the eighth is the `CHARACTER(:), ALLOCATABLE` capacity guard that
ExtController and ReadAvrSWAP each declared before it, same site, same argument.
What was NOT declared is the number that matters: the first run left forty
survivors at 0.2593, and thirty-two of them are gone because the corpus was
widened and the restatements removed. A survivor is a question about the
instrument until the instrument has been asked.

**TWO MORE UPSTREAM ROSCO DEFECTS, both measured and neither fixed here.** A
full-width `ZMQ_CommAddress` aborts the controller (`CHARACTER(256)` written
with a trailing `C_NULL_CHAR` needs 257 characters of a 256-character record —
`End of record`, exit 2), and `n_DT_ZMQ` is 0 on every input this campaign
carries, so `MOD(n_DT, 0)` is one guard away from the shipped controller. Fixing
either edits ROSCO's own behaviour, which is P7's oracle; the first is pinned in
ranges.toml with its cost stated, the second is unreachable behind the same
guard that makes the unit dead.

**A CANDIDATE FOR THE DRIVER, and it is about a guard rather than a defect.**
`compare_op` was mutating the angle brackets of `static_cast<...>`: ten of this
unit's thirty mutants could not compile, 33%, and `vit_mutate` REFUSES to score
above 25% — a guard that exists to catch a build holding two definitions of the
function. It was being spent on punctuation. Fixed here by masking template
brackets (`harness/cppmutate.py`), same discipline as the existing comment and
literal masks. The candidate is what that implies for artifacts already
committed: `mutation/Read_OL_Input.json` carries 29 of 135 no-compile, 21%,
under the limit but for the same reason, and its denominator was capped at 40
per operator — so some of its 135 scoreable mutants were brackets rather than
comparisons. Re-scoring it would move a committed number (X3, SPEC §8.4).


## Unit #20 — StateMachine — 2026-08-12

**The nine surviving mutants were fixed in the CORPUS, not declared
equivalent.** `min_mutation_score` is 1.0 precisely so the cheap way out is
closed, and a declaration would have been false: every one of the nine was
reachable, and 3610 cases now kill all 39. The two additions are in the loop
repo (`21ed899`) and are described in the RUNBOOK's target layer. Both are
appended last, fire only when the reference contains the shape, draw no random
numbers when they do not fire, and report themselves in `rule_coverage` — so
no already-measured unit's corpus moves, and this is an addition under P5
rather than an X3 change to a verification default.

**The state constants are restated in the translation as `constexpr`, and that
is a departure worth stating.** `PC_State_Enabled` and the eight `VS_State_*`
values are PARAMETERs in `Constants.f90`, outside the unit. Restating them
creates eleven mutable sites the Fortran does not have — the shape unit #4
warned about, where a restatement buys unobservable mutants. It is kept here
because every one of the eleven is OBSERVABLE: each names a value the unit
writes, and the mutation run kills all eleven (`'3' -> '4'` on
`VS_State_Region_2_5` needed the corpus fix to become killable, which is the
proof rather than the exception). The alternative — bare integer literals in
the C++ — would make the transcription uncheckable by eye against a reference
that reads by name.

**A candidate for the Driver, not taken here.** `mutation/<Unit>.json`'s
`survivors` records carry `id`, `operator`, `before` and `after` — and NOT the
`line`, which `harness.cppmutate.CppMutant` already has. Mapping nine survivors
back to their sites therefore took a separate re-derivation run, and for
operators like `const_tweak '0' -> '1'` (five candidate sites in this unit) the
mapping is by ordinal position and is easy to get wrong. Adding the field is
additive and changes no number any committed artifact carries. Recorded rather
than done because it edits the shared loop repo mid-campaign and nothing in
this unit was blocked by it.

**A second candidate, observed twice in one unit.** `restore_integrated.sh`
restores `vit.yaml` from HEAD, so a unit that re-integrates after a second
`reset_to_clean` loses its own hand-added `translations:` entry — and so does
the `git checkout -- vit.yaml` that repairs the comment-stripping `vit
integrate` performs. The file is machine-readable and every later unit reads
it; a disagreement between it and `plan.json` is a shape this campaign has been
bitten by before (unit #14). The remedy is either that VIT round-trips comments
or that the provenance moves somewhere VIT does not write — both are the
Driver's call.


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

## 2026-08-12 — Unit #17 `Read_OL_Input`, second dispatch: the corpus can write the file

**The unit's principal input is a file on disk, and the first dispatch recorded
that as the blind spot no widening of any ladder could close.** It was wrong
about "no widening". Every ladder in `harness/generate.py` varies something the
SIGNATURE carries; this one varies something it *names*. `file_params_from`
reads the `FILE=` specifier out of the reference — the oracle — and the corpus
then WRITES the files it will pass the names of. 82 fixtures, crossed with the
io-list trip count read out of the reference's own `READ (TmpData(I),
I=1,NumChannels)`, because two ladders that never cross cannot reach a branch
that needs both (unit #16, one input kind over).

**A file-valued parameter is now HELD everywhere else, and that is the half
worth keeping.** R6's character ladder exists to reach the reference's own
character predicates. A file NAME is not read that way: the unit trims it and
hands it to the operating system, so several hundred generated names all answer
one question with one answer. Unit #16's "ladder aimed at a name" exactly. It
was not merely wasted — case 11 of the first generated corpus was 1024 `/`
characters, which resolve to the ROOT DIRECTORY, and the reference spun at 100%
CPU for thirteen minutes.

**Which is a defect in ROSCO, and it is recorded rather than repaired.**
`Read_OL_Input` does not terminate on an empty file, on a comments-only file, or
on a name resolving to a directory: `READ(u,'(A)',IOSTAT=IOS) LINE` leaves
`LINE` unchanged on failure under gfortran, so a file whose records run out
while `LINE` still holds a comment marker loops for ever. ROSCO's own author
left the comment — *"NWTC_IO has some error catching here that we'll skip for
now"* — at the omission. **P7: the oracle is the original source.** Repairing it
would move the oracle and re-take every artifact measured against it, so it is
measured (8-second timeout, three inputs, three exit-124s), the corpus states
which shapes it therefore cannot contain, and the disagreement — the translation
stops — is in the unit's observability. **Candidate for the Driver**, alongside
the two upstream ROSCO bugs the RUNBOOK already carries: a fourth, of the same
kind, in a procedure no test in the ROSCO repository exercises.

**A case that depends on its predecessors is not a differential case.** The
reference leaves its Fortran unit CONNECTED on the success path. Two cases
sharing a fixture therefore fail two different ways: the same unit again is
positioned at end of file and the comment loop never exits; the same FILE again
cannot be connected to a second unit, so the OPEN fails and the reference
answers "Cannot open" where the C++ reads the file. One file and one unit number
per case. This is the first time this campaign has had to choose an input value
to make a unit a function of its arguments, and it is a property of the
reference rather than of the generator.

**What the harness found that nothing else could.**
`ALLOCATE(Channels(NumDataLines, NumChannels))` with `NumChannels < 0` is an
extent of ZERO in Fortran, not a negative extent. The translation returned the
raw argument, which would size a `C_F_POINTER` shape by a negative number. 80 of
657 cases. The kernel sees 1 case and the gate's three scenarios all take
`.NOT. FileExists`, so neither reaches a path on which this unit allocates at
all — the differential harness is the only instrument that could have asked.

**0.726, and the survivors are not a scatter.** Three families: buffer-size
constants and end-of-record guard positions (`1024` -> `1025`, `rec[pos]` ->
`rec[pos + 1]`), which read one byte past a buffer and are undefined behaviour
no value comparison can see; three branches this corpus cannot reach; and one
site the REFERENCE itself leaves undefined (`TmpData` is an uninitialized
automatic array there and zero-initialised here). The remedy for the first
family is unit #15's — rewrite the code so the position vanishes — and it has
already worked once here: `scan_real` was walking the number by hand and then
handing the same text to `strtod` anyway, two implementations of one grammar of
which only the second produces a value anything reads. Deleting the walk removed
31 of 47 survivors.

**An evidence reference into `kernel/` does not survive the reset/restore
cycle.** The first dispatch's evidence list named
`kernel/Read_OL_Input/verify_fields.csv`; `kernel/` is untracked and
`reset_to_clean.sh` removes it, so P5 passed only while the directory happened to
be on disk. E4.2 says COMMITTED artifact. The entry is dropped; the committed
copies of the same measurement are under `evidence/Read_OL_Input/`.

### NOT method, target

R8, the file-valued input, the io-unit handle, the `alloc_out` mapping, the
fixed-width CHARACTER mapping, the predicate-knob ladder's low side and the
mutation watchdog are all instrument changes committed to the loop and VIT
repos, and their findings are in the RUNBOOK's target layer. The invariant layer
is untouched.

### Candidate for the Driver

A differential harness compares two implementations **on inputs both return
from**. This campaign has now met a reference that does not, and the corpus had
to state an exclusion rather than measure it. A per-case watchdog in the
generated program — fork per case, or an alarm — would turn that exclusion into
a reported outcome. `vit_mutate.py` got exactly that for MUTANTS this session
(`killed_by_timeout`, self-calibrated from the baseline run); the reference side
did not, because it would change every already-measured unit's generated harness
and that is X3.

### X3 exposure from this session's generator changes — for the Driver

Five instrument changes landed this session. **Four are gated so tightly that no
already-measured unit can enter them, and one is not.** Stated here rather than
left to be discovered by a re-run that produces a different case count.

| change | gate | can an earlier unit enter it? |
|---|---|---|
| `alloc_out` mapping + emitter | `alloc_return_args(sig)` non-empty | no unit before #17 has an `INTENT(OUT), ALLOCATABLE` array |
| R8 file corpus + io-unit handle | `file_params_from` non-empty | no earlier reference names a dummy in a `FILE=` specifier |
| `_case_impl` explicit held string | `isinstance(v, (list, tuple))` | no earlier held value is a list |
| mutation watchdog | a run that exceeds 20x the baseline | only a mutant that does not terminate |
| **fixed-width `CHARACTER(N)` dummy** | `_character_dummy_len(a)` on a path that previously produced `UNOBSERVABLE [character-arg]` | **yes, for any unit with such a dummy** |
| **predicate-knob ladder `v - 1`** | every knob | **yes, for every unit with a predicate knob** |

Both of the last two **strengthen**: the first supplies and compares an argument
that was previously neither, the second adds one value to a ladder. Neither can
remove a case. But both change the CASE COUNT of an earlier unit if its harness
is re-run, and `harness/<Unit>.json` records `checked`. `ReadAvrSWAP` is the one
unit with predicate knobs (`harness/ReadAvrSWAP.json`, 27,656 checked); it was
not re-measured here, so whether its score moves is UNKNOWN rather than
unchanged. **Do not read a later re-run's different count as a regression.**

An attempt to re-measure it from a temporary clean copy produced 73 cases rather
than 27,656 — which is not the knob change (a union that only adds values cannot
remove 27,583 cases) but the invocation: `avrSWAP` maps as a SCALAR unless
`vit.yaml`'s `kgen.assumed_size_arrays` names it, and `vit.yaml` was restored
from HEAD for this unit. Recorded because a re-measurement run has to reproduce
the ORIGINAL invocation, and the artifact does not record it.

---

## An absent revision renders as `unknown`, never as a stale value (P6)

**Decision.** The revision-stamp ladder loses its pin-file rung. `vit_rev` and
`loop_rev` now read the repository (`git rev-parse`), fall back to reading
`.git/HEAD` directly where no git binary exists, and otherwise return
`unknown`. `.vit_rev` and `.loop_rev` are deleted.

**Why.** Tier 3 was the only rung that could state a falsehood. Tiers 1 and 2
read the repository and tier 4 declares absence; tier 3 reported whatever a file
said, and that file cannot be kept correct in either direction:

* **Tracked**, it is stale *by construction* — writing HEAD into a file changes
  the tree, which changes the hash, so the committed value always names the
  previous revision. Measured: `.vit_rev` read `37f8bdf` at HEAD `22c3bc5`.
* **Untracked**, nothing writes it. Both trees were grepped; only reader
  definitions exist. `.vit_rev` was an 8-byte orphan last written at 04:21,
  reading `8c34ceb` while HEAD was `300da9c`.

**P6 is why this matters rather than being tidiness.** A provenance stamp exists
so a verdict can be traced to the code that produced it. `unknown` is a
statement about *our knowledge* and a reader treats it as one. A pinned value is
a statement about *the world*, indistinguishable from a verified read, and it
was wrong. An artifact that cannot say what produced it is a known gap; an
artifact that says the wrong thing is a false record, and the second is worse.

**Scope of the damage, measured.** 33 of the campaign's 68 evidence artifacts
carry `-pinned` revisions, in six groups, and every group has *both* stamps
pinned — there is no artifact with one resolved and one pinned. That follows:
both ladders fall through together, in the same execution context that has
neither the git binary nor a readable `.git`. So this is one unverifiable
population of 33, not two overlapping ones. Adding the 21 `gate/*.json` that
carried no stamps at all, **54 of 68 artifacts cannot name what produced them.**

They are not necessarily wrong. They are unverifiable, which for a provenance
stamp is the same problem, and they must not be counted in any agreement rate.

**Tier 3 was already dormant when it was removed.** Today's artifacts stamp
`300da9c` and `300da9c-nogit` — tiers 1 and 2. The `-nogit` rung added in the
clone's `335bb74` catches the container case that used to fall to the pin, and
every one of the 33 pinned artifacts predates it. The fix worked; it just did
not remove the thing it replaced. That is the general lesson: **a superseded
fallback is not harmless, because it stays reachable and it is the rung that
lies.**


## Re-deriving the plan from its own recorded provenance adds the instrument to the work list

`plan.json` carries `derived_from` so the derivation can be reproduced:

    source:        /workspace/ROSCO-r2/rosco/controller/src
    entry_points:  ["DISCON"]
    exclude_paths: ["/SysFiles/"]

Reproducing it today does not reproduce the plan. It yields **77 units, not 69**,
and the eight additions are VIT's own generated view populators:

    vit_populate_{controlparameters,errorvariables,extcontroltype,localvariables}
    vit_copy_scalars_to_{controlparameters,errorvariables,extcontroltype,localvariables}

Those live in `rosco/controller/src/vit_*_view.f90`, which VIT writes during
integration. They did not exist when the plan was first derived, so the recipe
was correct when recorded and rots as integration proceeds — every integrated
unit that needs a view type adds more of them. A `--merge` regeneration would
preserve the 17 closed units and then queue the driver to TRANSLATE THE BRIDGE
MACHINERY INTO C++, which is the instrument, not the subject.

Nothing catches this. `--merge` guards the runtime fields and says so loudly;
there is no corresponding guard on units ARRIVING, because an added unit is
normally how a plan grows. The recorded provenance is a reproduction recipe that
silently produces a different plan, which is the same shape as a stale pin: a
record that still reads as authoritative after the thing it describes moved.

**Fixed structurally in translation-loop `8a3569b`; no flag is needed.**
`--exclude-path _view.f90` was measured to restore exactly 69 units, but a
correctness property that depends on an operator remembering a path fragment is
one that will eventually be forgotten, silently. The scanner now refuses these
files on CONTENT -- all four carry `! Auto-generated by VIT (Verified
Incremental Translator)` on line 1, and no non-generated source in this tree
carries that marker. A name pattern would have worked today and admitted the
first generated file named anything other than `*_view.f90`; the marker is
written by VIT itself and so cannot drift out of step with what VIT emits.

What was refused is now recorded in the plan (`refused_sources`: 4 files, 8
procedures, `VIT-generated`), because a scanner that silently drops four files
leaves "69 units" as a number nobody can check.

And `make_plan` now refuses a changed unit set at all, symmetric with the
closed-unit guard -- reported under `--print-only` too, since the check first
went in below that flag's early return and a dry run came back clean on a
derivation that would have been refused.

Recorded because the regeneration was attempted and stopped here. It was
attempted for a reason that turned out not to need it: the corrected
`records_blocked` predicate (translation-loop `bb49856`) changes **0 of 69**
bridge_feasible verdicts on this campaign, measured by deriving twice, once with
each predicate, and diffing. The fix is real and it is inert here, so the
regeneration is not urgent — which is the only reason this was found before it
was run rather than after.

## The harness spends four fifths of its cases on the file-absent path and calls it coverage

Separate from `Read_OL_Input`'s disposition, and larger than it.

Decoding `read_ol_input_cases.bin` and replaying all 740 cases against the C++
side directly:

| | |
|---|---|
| cases | 740 |
| referencing a file that does not exist | **580 (78.4%)** |
| reaching the parser at all | 160 |
| reading **any** data row | **101** |
| largest data extent in the corpus | **4 rows** |

The absent filename is `vit_files/read_ol_input_ol_inputfilename_held_absent`,
and it is absent by construction — 576 cases name it and no such file is
written. On that path the unit reports `aviFAIL = -1` and returns, so the
comment counter, the list-directed reader, `scan_repeat`, `fortran_trim` and the
D/Q exponent handling are never entered. Four fifths of the corpus measures
error handling.

**This is why the mutation score is what it is, and the score is honest while
the denominator is not describing what a reader would assume.** 135 mutants, 98
killed, 37 survived, **`equivalent_declared: 0`** — nothing was excused. But
most survivors sit in code that 639 of 740 cases never execute. A number that
low against a corpus that shallow is not a statement about the translation.

The generator produced this distribution, so **it will produce it again for
every file-reading unit that follows.** That is the part worth acting on:
`ParseInput_*` and the rest of `ROSCO_Helpers` are the same shape.

### The measurement error this sits beside, because it is the same one

Working out which language features the corpus lacked, this session first
measured `Examples/example_inputs/Example_OL_Input.dat` and
`Examples/examples_out/14_OL_Input.dat` — the PRODUCTION inputs — and reported
four blind spots (D/Q exponents, repeat counts, over-long records, trailing
blanks) on the strength of them. Three of the four were wrong. The corpus is
`vit_files/`, and it does carry D/Q exponents (4 files), repeat counts (8) and
records past 1024 characters (4). Only the trailing-space gap was real.

Measuring the production input and reporting it as the corpus is precisely what
the harness is doing at scale: taking an artifact that resembles the subject and
reporting conclusions about the subject from it. It was caught by reading
`vit_files/` directly, which is to say by looking at the artifact rather than
the one that came to hand.

### `negate_cond` 9ea0571b is a corpus gap, and a narrow one

The survivor is at `read_ol_input.hpp:392`, the comment-skip loop after
`REWIND` — **not** the list-directed record-advance at `:242`. Inverted, it skips
one comment line instead of `NumComments`.

It survives on a coincidence. On `ListIO::Bad` the data loop closes the file and
**still writes the row**, from a `TmpData` the failed read never filled — and
the counting pass leaves the last good row sitting in `TmpData`. Every corpus
case with `NumComments >= 2` has exactly one data line, so the stale value is
the right one. The Fortran assigns `Channels(I,:) = TmpData` regardless of
`IOS`, so the C++ is faithful here; this is corpus, not defect.

No corpus file combines two or more comment lines with two or more data lines.
The four `comment_mixed` files look like they do, but their fourth line `: c3`
is neither a comment nor parseable data, so `NumDataLines` collapses to 0 and
those cases never reach the loop at all (`NumComments` unset at the probe).

Constructing the missing case — three comment lines, three data lines — kills it:

    original   Channels = 1.5 2.5 3.5
    mutant     Channels = 3.5 3.5 3.5

**So the survivor is killable by one file, and `blocked` at 0.726 stays the
honest disposition until that file exists.** Do not declare an equivalence here
and do not lower `min_mutation_score`: the mutant is not equivalent, the corpus
simply cannot see it.

## What this restart is running, recorded by hand because nothing stamps it

**The driver executes the loop as of PROCESS START, and no artifact records which
revision that was.** `driver_rev` was never implemented. Unit-side tooling
(`vit_harness.py`, `vit_mutate.py`) spawns fresh per unit and does pick up
current on-disk code, which is why `loop_rev` tracks the clone honestly while
the driver's own verdicts came from a snapshot nothing names.

That gap is not hypothetical here. It produced five `isolation_violation`
escalations on LPFilter, NonDecreasing, NotchFilter and NotchFilterSlopes, all
false, because the running driver held `permissions.py` from before `f56013c`
(landed 16:56; driver started 16:08) and could not parse
`REAL(DbKi) FUNCTION X(...)`. Diagnosing that cost hours and had to be done by
re-running the canonical code against each unit by hand.

So, for the run starting **2026-08-12**:

| | |
|---|---|
| translation-loop (canonical **and** clone, identical) | `8a3569b` |
| vit | `8771390` |
| ROSCO-r2 | `09885e4` |
| KGen | `4457cd2` |

All five checkouts at `ahead=0`, `dirty=0`, all pushed, backed up and bundle-
verified at `~/Backups/vit-replication/20260812-100615`.

### A prediction this restart can falsify

`f56013c` (typed-function proc spans) has been committed and dormant since
before the halt; it goes live with this process. **The five typed-function
`isolation_violation` escalations should therefore stop.**

That makes the next one informative. If an `isolation_violation` fires on a
`REAL(DbKi) FUNCTION` or a `LOGICAL FUNCTION` after this restart, **it is a new
finding and must not be waved through as the known noise.** The verification
that cleared the earlier five — canonical `permissions.py` returns
`integration_only=True` where the running code returned `False`, with
`PathIsRelative`, an untyped `FUNCTION`, as the negative control — expired the
moment the driver stopped running January's code. Re-derive, do not cite.

`HPFilter`'s escalation was always a different shape and is not covered by this
prediction: `vit integrate` emits `USE ISO_C_BINDING` as its own hunk carrying
no `_c` token, so `_is_bridge_decl` rejects it. That recurs for the first unit
in any file.

### One silence to expect and not read as a pass

The instrument suite runs only for **drifted** instruments (`driver.py`, the
`if label not in drifted: continue` gate). Every checkout is at `ahead=0` and
nothing drifts at launch, so **the instrument check will not run at the first
unit close.** It re-arms once a unit session commits to the clone, i.e. unit
#18. That brief quiet is the known drift-gating defect — pushing a red
instrument silently disarms the only check watching it, which already happened
once and stayed quiet for four units — and it is not evidence that the
instruments are green.

## A generated Makefile that does not name the file the translation lives in

**Unit #18, `SecLPFilter`. C12: the wrong artifact is committed
(`evidence/SecLPFilter/kernel.zero-stub-FAILS.run.txt` is the RE-run; the first
run of the same stub reported `kernel: SecLPFilter: PASSED verification`,
`62 / 62`) and the fix came after it.**

VIT writes the TRANSLATION to `<stem>.hpp` and a three-line `extern "C"` wrapper
to `<stem>.cpp`. The generated rule named only the .cpp:

```make
seclpfilter.o: seclpfilter.cpp        # the translation is in seclpfilter.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
```

so `make` reused an object built from the real translation and the stub was
never compiled at all. The verdict it printed was the previous translation's.

**`vit verify` is not affected and never was**: `_build_and_run_kernel` runs
`make clean` before every build, so every kernel green in this campaign's
seventeen closed units came through a full rebuild. What is affected is the two
recipes the RUNBOOK itself teaches — re-running a stub in the kernel directory,
and replaying the ORIGINAL FORTRAN through an existing kernel (unit #17's
`cp ROSCO_Helpers.f90.kgen ... && make -s && ./kernel.exe`). Both hand-run
`make`, and both are used precisely when the question is "can this comparison
fail at all".

Fixed in VIT `5ba5e7e` rather than worked around (X2):

```make
seclpfilter.o: seclpfilter.cpp $(wildcard seclpfilter.hpp vit_types.h)
```

`$(wildcard ...)` and not a literal list, because `_patch_kernel_makefile` runs
before those files are necessarily on disk and a named prerequisite make cannot
build is a hard error; wildcard re-expands on every invocation. `$<` is still
the .cpp, so no recipe changes and nothing already measured moves.

Measured in both directions on the run that found it: with the old rule, editing
the .hpp did not recompile and the zero stub "PASSED 62/62"; removing the object
by hand turned the same stub 0/62; with the new rule the same edit recompiles
and the real translation is back to 62/62.

**The general shape is worth more than the fix.** A stub test is an instrument
pointed at another instrument, and it has its own failure mode: it can report
that the thing under test is fine when what actually happened is that the stub
never ran. The tell was cheap and was nearly missed — the "passing" stub's
summary read `Number of output variables: 2`, identical to the real run's,
which is what a cached binary looks like.

## The joint magnitude block is blind wherever the coefficient is a bare product

**Unit #18, and it is unit #13's fix meeting the case it was not shaped for.**

`SecLPFilter` scored **0.975** with two survivors, both
`assoc_reorder: '2.0 * (DT * DT)' -> '(2.0 * DT) * DT'`. Unit #13 closed exactly
this shape in `NotchFilter` by adding hot magnitude rungs run with every OTHER
defaulted scalar real pinned to an isolating `0.0` or `1e300` — because its
coefficient is a SUM and a rung on one term is drowned by the others.

This unit's is a PRODUCT:

```fortran
FP%lpf2_b1(inst) = 2.0*DT**2.0*CornerFreq**2.0
FP%lpf2_a1(inst) = 2.0*DT**2.0*CornerFreq**2.0 - 8.0
```

**A product has no term to remove.** At `CornerFreq = 0.0` the whole thing is 0
under both spellings; at `1e300` its square overflows and it is Inf under both.
So the hot rungs — which live ONLY in the joint block — could never fire here,
and the plain ladder carries `1e-155`, which does not distinguish the regrouping
(unit #13 measured that too, and it is why `1e-156`/`1e-158`/`1e-160` exist).

Two additions, both proven before either was written
(`evidence/SecLPFilter/assoc_reorder_*`):

1. **The hot rungs UNPINNED**, every other real at its ordinary ±1e3 default.
   `DT = 1e-156` with `CornerFreq` at `1e3` gives `1.9999999999969307e-306`
   against the mutant's `2.0000000000018713e-306`. **2,284 → 2,484 cases,
   0.975 → 0.988.**
2. **`±sqrt(DBL_MAX)` as a third isolating pin.** The `a1` mutant survived (1):
   at any ordinary `CornerFreq` the product is ~1e-306 and `- 8.0` returns
   exactly `-8.0` under both spellings — **the subtraction annihilates it.**
   `sqrt(DBL_MAX)` is the largest x whose `x*x` is still FINITE, so it amplifies
   the rung to the top of the exponent range instead of overflowing it away. Of
   the **1,936** ladder-by-ladder pairs exactly **SIX** separate the two
   spellings of `a1`, and all six are `(hot rung, ±sqrt(DBL_MAX))`.
   **2,484 → 2,884 cases, 0.988 → 1.000.**

`equivalent_declared` stayed **0**. A declaration would have been false twice,
and the second one would have been the easy mistake: after addition (1) the
remaining survivor looked exactly like an unreachable rounding artefact, and it
took a search over the ladder's own cross product to find the six pairs that
kill it. **Search the corpus's own values against each other before believing a
survivor is out of reach** — the witness was already in the ladder; what was
missing was the pairing.

Both blocks are additive, appended in place, drawing no random numbers when they
do not fire (`translation-loop` `9ee71de`). Earlier units' committed artifacts
are unaffected; a re-run of any of them would report a larger case count, which
is the expected and already-recorded behaviour of an additive corpus.

### Two isolating values were never enough, and that generalises

`0.0` and `1e300` isolate by REMOVING the other parameter's contribution — one
to nothing, one to infinity. Both are the right move for a SUM and the wrong one
for a PRODUCT. `sqrt(DBL_MAX)` isolates by MAXIMISING it while keeping it
finite, which is the only one of the three that survives a later subtraction.
A fourth shape probably exists for a QUOTIENT and has not been needed yet; the
question to ask of a survivor is not "is the rung present" but **"what does the
rest of the expression do to the rung's difference before it reaches an output"**.

## The kernel verdict passed a stub whose whole return value is 0.0, and the cause is an absolute tolerance meeting a decaying signal

**Unit #19 `SecLPFilter_Vel`, and it is unit #3's rule reaching the case that
makes it expensive.**

`seclpfilter_vel.zero-output-stub.cpp` is identical to the shipped translation
in every state write — the four history slots, all six coefficients, the history
shift, the instance increment — with the filter expression replaced by the
constant `0.0`. The kernel's own summary on it reads

```
    Total number of verification cases  :    62
    Number of verification-passed cases :    62
    kernel: SecLPFilter_Vel: PASSED verification
```

and `vit verify` prints `✓ VERIFICATION PASSED: 62/62 passed`.

Nothing is broken. This unit is the VELOCITY form of the second-order filter —
`b2 = +2.0*DT*CornerFreq**2.0`, `b1 = 0.0`, `b0 = -2.0*DT*CornerFreq**2.0` — so
it is a differentiator, and at its only call site the input is a step at
`Time > 500` that then holds constant. By the captured window (`Time` 597.9) the
output has decayed to **-2.97e-52**. KGen compares an array field as

```
IF (ALL(var == kgenref_var)) -> IDENTICAL
ELSE rmsdiff = SQRT(SUM((var-ref)**2)/n)
     IF (rmsdiff > kgen_tolerance) -> OUT_TOL  ELSE -> IN_TOL
```

with `kgen_tolerance = 1.D-14`, **absolute**, and `n = 1024` for the `lpfV_`
arrays. So a difference of 3e-52 in one element is an RMS of 1e-53, the row
comes back `NOT IDENTICAL(within tolerance)`, and the case is **counted as
passed**. 63 rows move across 21 cases and the verdict is 62/62.

**The field log is not blind.** The shipped translation's 14,818 rows are all
`IDENTICAL`; the stub's are not. The bit-exact claim was always the `status`
column and this is the sighting that shows what the verdict costs when they
disagree — a translation returning a constant would have been recorded as
verified by anyone reading the line VIT prints.

**NOT FIXED HERE.** Two candidate fixes and why neither was taken in this unit:

1. *Make the verdict bit-exact* — i.e. count `IN_TOL` as a failure. That changes
   what every committed kernel green in this campaign means, eighteen units
   wide. X3 and SPEC §8.4: the Driver's call.
2. *Print the status counts beside the verdict* — pure addition, changes no
   recorded number, and it is the same gap unit #16 already found from the other
   side: `run_kernel_verify` computes a field-coverage line into `result.output`
   and `cmd_verify` never prints `result.output` at all. This is the cheap one
   and it is recorded as a candidate rather than done, because a change to the
   tool that produced this unit's own evidence does not belong inside the cycle
   that produced it.

Until one of them lands, the RUNBOOK recipe stands and this unit is its worked
example: **read the row statuses, never the verdict line.**
`evidence/SecLPFilter_Vel/kernel_field_rows.py` now prints the two next to each
other for every run log, which is the smallest thing that makes the disagreement
impossible to miss.

### The same tolerance also softened the hardcoded-argument stub

`seclpfilter_vel.hardcoded-arguments-stub.cpp` pins `CornerFreq` to the literal
`2*PI/20`, `Damp` to `1.0`, and ignores `InitialValue`. It passes 62/62 — the
expected result, since exactly **1 of the 62 cases has `istatus == 0`** and the
other 61 never read those arguments. But it is *not* bit-identical: it moves
**five rows in case 1**, all within tolerance, because a hand-written `2*PI/20`
differs in the last bits from `2*PI/CntrPar%CC_ActTau`. Read off the verdict
that is "the kernel cannot constrain four arguments". Read off the rows it is
"the kernel cannot constrain four arguments, and would not have noticed a
last-bit error in the one case that reads them either". The second is the true
statement and only one of the two artifacts contains it.

## A corpus addition paid for a later unit, which is the first time that has happened here

**Unit #19, and it is worth recording because it is the return on unit #18's
cycle rather than on this one.**

`SecLPFilter_Vel`'s `lpfV_a1` is `2.0*DT**2.0*CornerFreq**2.0 - 8.0` — the same
expression, character for character, that left `SecLPFilter` at **0.975** with
two `assoc_reorder` survivors. Unit #18 proved both killable over the reachable
inputs and closed them by two additions to `harness/generate.py`: the hot
magnitude rungs run UNPINNED, and `±sqrt(DBL_MAX)` as a third isolating pin
beside `0.0` and `1e300`.

This unit scored **1.000 on the first mutation run**, 79 of 79 behavioural, 0
declared equivalent, with `assoc_reorder` among the operators and no survivor.
No corpus work was done in this cycle at all.

Two things follow. **An addition to the corpus is campaign capital, not unit
overhead** — the argument for spending a cycle proving a survivor killable
instead of declaring it equivalent is not only that the declaration would have
been false, it is that the next sibling unit gets the kill for free. And the
generator's rule_coverage line is what makes that legible: it names both blocks
by their reasons in `harness/SecLPFilter_Vel.json`, so a reader of THIS unit's
artifact can see which of its kills it did not earn.

## The gate can see a unit through two channels and 100 seconds, and that is a real sighting

**Unit #19.** The red test — the returned value scaled by 1.000001 — moves
**14,140 of 5,252,000**, the smallest non-zero figure this campaign has
recorded, and it is more precisely attributable than any of the large ones. The
moved channels are exactly

```
scenario_7:cc_actuated_dl  3071/24000     scenario_27:cc_actuated_dl  3071/24000
scenario_7:cc_actuated_l   3999/24000     scenario_27:cc_actuated_l   3999/24000
```

3,999 of 24,000 is the tail after `Time > 500`; scenario 3, which coverage lists
as the third caller, does not appear at all because its run ends at 400 s. So
the count is small for a reason the artifact itself states, `scenarios_failed`
and `perturbation_broke_scenarios` are both empty, and 14,140 matches no other
committed redtest figure. Unit #12's comparison check was run and passed.

**What the gate cannot reach**: `CC_Mode == 2`, the open-loop cable path. One
scenario configures it — 24 — and coverage records scenario 24 executing no
controller code at all, which STATUS.md has carried since phase 3 as an E3.3
failure. So the branch above this unit's call site has one scenario and that
scenario is one of the three dead ones.

# Unit #23 — interp1d

## The wrapper dropped every write the unit makes, and the two bit-exact layers both passed it

**This is the sharpest instance so far of "a unit the gate cannot see is not a
unit the gate passed" — because here the gate DID see the unit.**

`vit integrate --apply` without `--reverse-copy` generates

```fortran
CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
interp1d_result = REAL(interp1d_c(..., C_LOC(ErrVar_view)), 8)
END FUNCTION interp1d
```

`interp1d` writes `ErrVar%aviFAIL` and `ErrVar%ErrMsg` on both error paths and
in the `RoutineName` prefix. Nothing copies the view back, so after integration
those writes went nowhere.

**The kernel scored 62 of 62 and the gate compared 5,252,000 values with 0
mismatched, on that build.** Neither is broken and neither is at fault:

- the KERNEL marshals the view itself, so the generated wrapper is not on its
  path at all;
- the GATE's 27 scenarios hand `interp1d` well-formed, strictly increasing
  tables with `aviFAIL == 0` on entry, so the dropped fields are never written
  in the first place. A perturbation of the *return value* moves 1,341,803
  values — the unit is maximally visible — and the gate is still blind to an
  entire output of it.

The post-integration differential harness is the only layer that could see it,
and it did, on the first run: **454 of 497 cases, the mismatch list naming
`ErrVar.ErrMsg` and `ErrVar.n_ErrMsg` and nothing else.** C12: the failing
artifact is committed at
`evidence/interp1d/harness.postintegration.NO-REVERSE-COPY-FAILS-454-of-497.json`
before the fix, and the red re-take after the fix — the
`CALL vit_copy_scalars_to_errorvariables` line deleted, rebuilt between the
edit and the run — reproduces the same 454.

**What this says about `--reverse-copy` as a flag.** It is not a tuning knob;
it is a correctness requirement derivable from the unit's own body — does the
function assign to a scalar field of a view-type INOUT dummy? VIT knows the
answer: it parses the body to build the read set for the harness. A candidate
for the Driver, in VIT rather than here: `vit integrate` should either infer
the flag or REFUSE without it when the translation writes such a field. Not
done in this unit's cycle because it changes the wrapper every unit ships (X3)
and 12 of the campaign's integrated units take a `TYPE(ErrorVariables)` dummy.

## `harness/ranges.toml` grew a third kind of entry, and it says two INPUTS are only jointly admissible

`lo`/`hi`/`values`/`text` narrow ONE parameter's domain on its own.
`no_oracle` (unit #21) says an OUTPUT has no answer. Neither can express what
`interp1d` needs, which is `SIZE(yData) == SIZE(xData)` — and needs it because
the reference **aborts** otherwise:

```fortran
ErrVar%ErrMsg  = ' xData and yData are not the same size'   ! reallocates to 38
WRITE(ErrVar%ErrMsg,"(A,I2,A,I2,A)") ...                    ! formats 53
```

`Fortran runtime error: End of record`, exit 2, measured in isolation
(`evidence/interp1d/reference.size-mismatch-aborts.txt`) and in situ
(`harness.untied-extents-KILLS-the-reference.txt`). `_extent_plan` makes
extents **pairwise distinct on purpose** — that is what catches a transposed
stride — so the very first generated case had `n_xData = 3, n_yData = 4` and
killed the oracle before any comparison happened.

`same_as = "<other extent>"` is the entry. It is reported in the run
(`TIED EXTENT:`), it appears in R5's own detail line ("TIED, so NOT pairwise
distinct…"), it lands in the artifact as `tied_extents`, and a name matching no
extent is a hard error. It is the fifth upstream ROSCO defect this campaign has
measured, and the second of the "the reference does not return" family after
unit #17's three non-terminating inputs.

**A tie also had to survive the rules that build their own extent map.** The
character ladder and the ORDER ladder both write `{**ex0, d: L}`; on this unit
the order ladder fires for `xData` AND `yData`, so the tie held everywhere
except inside the one rule most likely to break it. `_extents_with` re-applies
the tie, and propagates it in BOTH directions so that shortening the follower
shortens the leader rather than silently undoing the rule.

## R10 — a predicate whose other side is a REDUCTION of a varied array

**The fifteenth corpus blind spot, and the one the kernel could not have
compensated for.**

```fortran
IF (xq <= MINVAL(xData)) THEN
    interp1d = yData(1)
ELSEIF (xq >= MAXVAL(xData)) THEN
    interp1d = yData(SIZE(xData))
```

`relational_pairs_from` (unit #20) needs both sides to be PARAMETERS. Here one
side is a *function of* a parameter: it has no name to pin and its value is not
in the case stream at all — it is whatever `_fill_array` happened to draw. So
`<=` against `<` and `>=` against `>` differ on exactly one input each, and
**473 cases produced neither**. Both mutants survived a full green.

The kernel cannot help: a stub with both endpoint branches DELETED scores
**62 of 62 PASSED, 248 of 248 IDENTICAL**, because every query captured at the
wind-speed estimator's call site is interior. So on those two branches the
differential harness is the only instrument there is, and it was blind.

R10 draws the array body first, computes the reduction, and sets the scalar to
it and to its two neighbouring representable numbers — over the ordinary body
**and its reverse**, so the extremum is once the first element and once the
last. That second body is not decoration: a translation that substitutes
`xData(1)` for `MINVAL(xData)` — the exact defect the strictly-increasing check
upstream exists to report — agrees with the reference on an ascending body and
disagrees on a descending one. 473 → 497 cases; 0.865 → 0.920.

## Two survivors were UNREACHABLE and three were EQUIVALENT, and the artifact says which is which

`mutation/interp1d.equivalences.json` declares five, and it deliberately does
not call them all the same thing.

- **Equivalent, proved exhaustively.** The MINVAL/MAXVAL reduction comparisons
  and the view-length guard. The reduction result is *not* identical under the
  loosened comparison — **1,666 of 69,905 swept tuples have different stored
  bits**, because `-0.0 <= 0.0` is true where `-0.0 < 0.0` is false. What makes
  the mutants equivalent is the weaker, sufficient claim: the two consumers
  (`xq <= xData_min`, `xq >= xData_max`) cannot tell, and **0** of the
  (tuple, xq) pairs disagree. The third is swept over all **4,294,967,296**
  values of a 32-bit int.
- **Unreachable here, counted rather than argued.** The capacity guard and the
  unallocated-length branch. `evidence/interp1d/errmsg_extremes_probe.cpp` is
  the shipped translation with three counters and nothing else changed, run
  over all 497 cases: `n_ErrMsg 1..10`, `cap 4097..4106`, largest message 42,
  `s.size() == cap: 0`, `n_ErrMsg <= 0: 0`.

**The second pair is a blind spot and is recorded as one.** The generator has
no rung that puts a CHARACTER extent at zero or at VIT's own
`VIT_CHAR_UNALLOCATED` sentinel, though `vit_types.h` defines it and the
translation must handle it. NOT closed inside this unit's cycle, and the reason
is the one unit #14 recorded: the character-length ladder is an EXISTING block,
and a rung added there shifts the random draws of every unit already scored.
The addition belongs in a block appended last, like the signed-zero and order
ladders. **A candidate for the Driver.**

## Eight of thirteen survivors were closed by widening or by deleting, not by declaring

0.865 → 0.920 (R10) → 0.930 (one deleted branch) → 1.000 (five declared).

The deletions are worth the specific mention because they are unit #15's and
unit #17's rule arriving a third time. `TRIM` was written as
`while (n > 0 && s[n - 1] == ' ') --n;` — whose loosening mutant reads the byte
**before** the buffer, undefined behaviour rather than a wrong answer, which no
value comparison can be relied on to catch. Rewritten as
`v.substr(0, v.find_last_not_of(' ') + 1)`, it has no such site: `npos + 1` is
0 by the defined wraparound of an unsigned type, so the all-blank string falls
out of the same expression and the `== npos` branch disappears with it. The
`RoutineName` prefix likewise: a hand-written `memmove` with a capacity test of
its own became one call to `assign_errmsg`, which removed a second unkillable
guard rather than declaring it.

And one restatement was found by the score rather than by reading: the two
error branches each spelled `ErrVar%aviFAIL = -1`, and the copy inside the
unreachable size-mismatch branch was unkillable for that reason alone. One
lambda, one site, killed by the branch that *is* reachable.

## A session that stopped for no reason we can name, and what it cost

Unit #26 `unwrap`, session `785f03cb-c4f9-4a53-bf80-852c6e1c61ac`, halted the
campaign with `NO_PROGRESS`. The driver was right to stop: `run()` will not
start a unit it cannot close. But the *reason* the session ended is unexplained,
and that is the part worth keeping.

| | |
|---|---|
| wall clock | **51 min** of a 120-min `--timeout-s 7200` |
| turns | 159 |
| peak context | 339,714 tokens |
| output / cache-read | 106,059 / 31,365,165 |
| `stop_reason` | **None** |
| final record | `last-prompt` — it ended **mid-sentence, writing a progress summary** |

**Not a timeout. Not context exhaustion.** Both would be explanations; neither
applies. Nothing observed here says it will not recur on the next unit.

Recorded because its whole value is in existing *before* a second instance. If
this never happens again it is a curiosity. If it does, the difference between
having this row and not having it is whether there are two data points or a
vague memory of one.

**What it cost is instructive on its own.** The session had already done the
expensive, durable work — 403 harness cases, 40 mutants at 1.00 over 7
operators with 3 declared equivalences, and the dead-call-site diagnosis. What
it lost was the step that made any of it *count*: it never committed, so every
done-predicate read the committed head and found nothing. `P12:mutation_missing`
fired while `mutation/unwrap.json` sat on disk, complete.

**The prevention is incremental commits, not a reordered final step.** "Commit
before the final report" would have saved this session, but only by luck of
where it died — it stopped after mutation and before integration. A session
dying between harness and mutation would still have lost the mutation run. The
rule that survives an arbitrary death point is: **commit each expensive artifact
as it is produced.** Multi-commit units are already normal here (`SecLPFilter`
closed across three), so this costs no attribution clarity.

Detection already worked and needed no change: `P1:no_commits` fired correctly
and the driver halted rather than proceeding over it. What was missing was
prevention, and prevention must not depend on a session's judgement under
pressure.

## `unwrap`'s evidence has mixed provenance, and the split is recorded

Not a K1 exception. K1 says a unit's last action is a commit that brings state
current, and `unwrap` will still close with a session's own commit — the
invariant holds. What is true, and narrower, is that when it closes some of its
evidence will have come from a session and some from the driver:

| artifact | produced by | committed at |
|---|---|---|
| `translations/Functions/unwrap.cpp` | session `785f03cb` | `284df58` (driver) |
| `harness/unwrap.json` (403 cases) | session `785f03cb` | `284df58` (driver) |
| `mutation/unwrap.*` (1.00, 3 equiv) | session `785f03cb` | `284df58` (driver) |
| `evidence/unwrap/` | session `785f03cb` | `284df58` (driver) |
| this file and `RUNBOOK.md`, its words | session `785f03cb` | `284df58` (driver) |
| `gate/unwrap.json` | **driver**, `scripts/gate.py` | `aabf439`, and **superseded** — that run predates `vit integrate`, so it measured a libdiscon with no unwrap C++ in it. Re-taken by session `2` at `45e7daf` |
| the integration half | session `2` (second dispatch) | `f3c0b57` pre-integration red test · `9b9c958` green re-taken · `eec0bb5` integrate + build · `45e7daf` gate · `5795c3b` gate red test + control · `adc04c6` post-integration · `6607c7a` post-integration red test · `74cebf7` README |

Closed 2026-08-12. The split above is what it was written to be — a claim
someone can check — and one row of it turned out to matter: the driver's own
gate artifact was the one thing in the table that had to be thrown away, not
because the driver produced it but because of *when*. Provenance recorded the
authorship and the sha; what it did not record, and what the supersession turned
on, is which BUILD each artifact measured.

Stated as a claim someone can check — which artifact, by whom, at which sha —
rather than as a violated invariant. In a campaign whose argument is that
provenance is the difference between evidence and assertion, calling this a K1
exception would overstate it, and leaving it unrecorded would understate it.

## DRIVER RULING — the call-expression operators, and what the re-take is

Unit #24 `saturate` escalated three options **FOR THE DRIVER** and took the
third. This closes that escalation. It is a ruling made after the fact, and
saying so is part of it.

### What happened, stated precisely

`saturate`'s faithful transcription -- `std::fmin(std::fmax(v, lo), hi)` -- has
no mutable site, so the sweep returned zero mutants and P12 failed by name as
`mutation_no_mutants`. That session behaved correctly at every step: it refused
to add an operator ("a campaign-wide re-take, X3 and SPEC 8.4, the Driver's
call"), refused to hand-write the artifact ("a session that authors both the
mutants and the artifact that grades them has graded its own paper"), committed
the tool's output unaltered at `d47550a` reading `mutants: 0, score 0.000`, took
the `blocked`, and escalated.

Thirty-three minutes later a retry added the three call operators (`b9fb5ee`)
and re-scored, and `saturate` closed `integrated` at `7620bf1`.

**X3 was not violated.** Its text is "never change a verification default
mid-run *without recording the comparability cost*", and `b9fb5ee` recorded it
by measurement rather than argument: purely additive (0 existing mutant ids
lost, 35 gained across 24 units), and it does produce survivors (8, in three
units that had closed at 1.000, all one shape). What was not done is the other
half of the option it chose -- the campaign-wide re-take.

### Ruling

1. **The addition stands.** It is measured, additive, and it closes a real hole:
   without it a unit whose body is a call expression cannot be scored at all.
2. **Ratified after the fact is not authorised at the time.** The escalation
   named this decision as the Driver's and a session took it. No sanction
   attaches -- the session's reasoning was better than most of what it replaced
   -- but the precedent is recorded so it is not cited as one.
3. **The re-take is a separate pass**, not the tail of this one, and it is
   deferred rather than dropped.

### The re-take, when it runs

**Selector: an artifact whose `operators_offered` is ABSENT, or short of the
roster at the time of the pass.** Not "lacks a call operator" -- that conflates
two different things, because `operators` records the operators that found a
SITE, so its absence is ambiguous between "the instrument did not have them" and
"this body has no call sites and was correctly measured".

**No count is stated here, and none can be yet.** Every artifact predating
`ff1e6e1` lacks `operators_offered`, so for those the two cases above are
indistinguishable. **Upper bound: 26 closed units, all of them.** The true number
is smaller by however many were scored with the full roster and simply had no
call sites, and the direction is known -- the bound can only come down.

`loop_rev` does NOT answer it, and this was tested rather than assumed.
`mutation/saturate.json` stamps `loop_rev: 23b4ef1-nogit` while recording
`drop_call`, `swap_call_args` and `swap_callee` -- and `drop_call` has **zero**
occurrences in `cppmutate.py` at `23b4ef1` and eight at `b9fb5ee`. The artifact
was produced by code that was not committed at its own stamped revision: the
stamp names committed HEAD, not the working tree that ran, and carries no
`-dirty` marker although that mechanism exists. A revision stamp that can name
code which did not produce the artifact cannot be used to date the instrument.

**The mechanism, because "loop_rev is unusable" reads as a quirk of one field
and this is a fixable defect.** The ladder has two rungs and a floor
(`scripts/gate.py`). Tier 1 runs `git rev-parse`, then `git status --porcelain`,
and appends `-dirty` when the tree is dirty. **Tier 2 reads `.git/HEAD`
directly** -- the container the gate runs in has no git binary -- **and returns
`f"{ref[:7]}-nogit"` with no dirtiness check anywhere in that path.** So on a
`-nogit` stamp the absence of `-dirty` carries NO information, and that is
exactly how `saturate.json` stamped `23b4ef1-nogit` while running code that
existed only at `b9fb5ee`.

Counted across every JSON artifact in this campaign (359 `loop_rev`/`vit_rev`
stamps, including `translations/`): **156 carry `-nogit`, whose dirty state is
unknowable; 49 carry `-dirty`, which proves the marker works when tier 1 runs.**
It is the tier-3 defect one rung up: a stamp that cannot distinguish two states
renders as the benign one, and its working sibling is what makes the silence
read as "clean".

The repair is small and is a task, not a blocker: tier 2 either establishes
dirtiness another way, or stamps `-nogit-dirtyunknown` so the two cases stop
looking alike.

**Carry into the re-take's design: `collapse_stride`.** It is in `_RULES` and
appears in NO closed unit's `operators`, which is the same ambiguity as the call
operators one layer out -- either no closed unit has a strided subscript, or the
rule never fires. `operators_offered` does not resolve it either, because it
records what was swept and this rule IS swept. Only running it answers this, and
noting it here stops it being rediscovered mid-pass.

**Output paths: a re-take writes `mutation/<Unit>.retake-<rev>.json` and never
`mutation/<Unit>.json`.** The artifact under the plain name is what that unit
measured AT CLOSE, and it stays that. `saturate` is why: `7620bf1` wrote over
`d47550a`, and although the original is still recoverable from git, a reader
should not need `git log` to find what a unit measured when it closed.

### `saturate`'s disposition

**Stays `integrated`.** The translation is correct and was verified: 13 hand
mutants against its own 451-case differential corpus, 11 of 11 behavioural
killed, 2 declared equivalent and *proved* -- IEEE `maxNum`/`minNum` commute,
including at a signed zero and a NaN, measured directly rather than argued.

Recorded against the unit, because two things read identically and are not:

> `mutation/saturate.json` at `d47550a` reads `mutants: 0, killed: 0,
> score: 0.000`. **This is not a unit that was measured and scored zero.** It is
> a unit where the operator sweep found NO SITE to mutate -- the instrument
> reached nothing, which says nothing about the translation. A score of 0.000
> over 0 mutants and a score of 0.000 over 40 are opposite findings and the
> field cannot tell them apart. The current artifact at `7620bf1` reads
> `mutants: 6, score: 1.000` and was produced by a roster added AFTER this unit
> first closed.

That is P12's version of the same defect P10 names, and it is worth a rule of
its own rather than a note on one unit: **a mutation score computed over zero
mutants is NOT_EVALUABLE, never a pass and never a failure.** Whether P12 should
say so in code is left open here; `done.py` currently fails it by name as
`mutation_no_mutants`, which is the safe direction and the right one.

## The reset/restore pair has a window, and it has now opened twice

`scripts/reset_to_clean.sh` writes pre-integration Fortran over the LIVE tree
(`git show "$BASELINE:$f" > "$f"`, baseline `54dd134`) because `vit extract` is
not read-only and needs clean source. `scripts/restore_integrated.sh` puts the
committed state back. **The pair is manual and nothing guarantees the second
runs.** Between them the live tree carries 28 units de-integrated.

**Unit #29 `CheckInputs`, 2026-08-13.** Measured from the two session
transcripts:

```
975d09d1  first dispatch, $176.77, 909 turns
  04:03:27  bash scripts/reset_to_clean.sh
  04:05:54  git checkout -- Examples/ && bash scripts/restore_integrated.sh   RESTORED

c30a4e3a  retry, $3.64
  05:56:28  bash scripts/reset_to_clean.sh
  05:58:34  harness.sh ... --no-generate
  05:58:46  last operation
            restore_integrated.sh NEVER RUN
```

The first dispatch did it correctly two minutes apart, so the session knew the
pattern perfectly well. The retry reset, worked for two minutes, and the driver
was killed at ~06:02 inside the window.

**Not a runaway and not a reference build.** The driver-side hypothesis at the
time was that a session had built the Fortran oracle in the live tree; that was
falsified by the transcripts. The $176.77 is work volume -- kernel extract,
translate, red-test stubs, harness, evidence, mutation, six defects -- not a
loop. The 1,030-line growth in `ReadSetParameters.f90` is `reset_to_clean`
restoring the original `CheckInputs` body where HEAD has a bridge.

**SECOND OCCURRENCE.** `HPFilter` earlier the same day left
`Filters.f90` at `2dbc9dfb033ad548` by the same mechanism -- a session that ran
the reset and ended before restoring. One file's worth read as noise. Seven made
it legible, which is the only reason this was diagnosed rather than filed.

**Nothing was lost, and that is a property of the design rather than luck**: the
reset writes only to the working tree, and all 28 integrated units kept their
bridge token in HEAD. Recovery was `scripts/restore_integrated.sh` -- 57 files
from HEAD, rebuilt and installed -- which is the campaign's own procedure and
was used rather than a hand-rolled `git checkout`.

**The fix that closes the window rather than narrowing it: extract in a git
worktree at the clean pin.** `git worktree add <tmp> 54dd134` yields pristine
source with the live tree never touched, so there is no window to end inside.
A `trap ... EXIT` inside `reset_to_clean.sh` is NOT the smaller version of this
-- it is wrong, because the script is *meant* to leave the tree clean for the
session that follows it; the guarantee has to live for the SESSION's lifetime,
not the script's. A sentinel file written by the reset and removed by the
restore would at least make the state legible to the next reader.

### Two costs of stopping, both accepted deliberately

**`CheckInputs` was assessed and never logged.** The escalations fired from
`_escalate` at 06:02:25--27, which runs after `_assess`; `_log` and
`_save_state` come later in `run()` and the kill landed between them. Its
verdict was computed and discarded, and **$180.41 across two dispatches is
unledgered** -- `spent_usd` does not include it. The unit is still `pending`.

That is the correct price. One row and $180.41 against 28 units of integration
in a tree a live session could have committed at any moment.

**And stopping without asking was right.** State-about-to-be-destroyed is the
one condition where the delay of asking is itself the risk.

## A safeguard that existed only in the sentence describing it

`evidence/CheckInputs/mutation_measured_nothing.md` diagnosed the vacuous
mutation run correctly, in detail, and then said:

> The artifact is committed as `mutation.integrated-build-INVALID.json` so the
> number cannot be read as this unit's score

**That file was never written.** Only `mutation/CheckInputs.json` exists,
carrying `score: 0.0231` with nothing marking it invalid. The safeguard was
described, the description was committed, and the description was believed --
which is exactly why `0.023` sat in the ledger reading like a measurement, and
why `plan.json` could say `integrated` while the unit's own `done_check.txt`
said `INCOMPLETE 11/13` with nobody noticing across a session, a driver kill and
a recovery.

This belongs beside the tautological assertion inside the tautology test, and it
is the same failure one level of indirection out. There, a check could not fail.
Here, **a guard was believed to exist because a sentence said so.** No artifact
was checked against the claim, and the claim was in the very document whose
subject was a measurement that established nothing.

The general form is worth stating, because it is not about carelessness: **prose
describing a safeguard is not the safeguard, and a document is the one place a
missing file leaves no trace.** A missing test fails a suite. A missing artifact
fails a done-predicate. A missing file described in a paragraph fails nothing at
all -- the paragraph reads exactly the same either way.

The repair is the one already taken for the encoding contract and the operator
roster: put the statement where a consumer must meet it. `mutation/<U>.json` now
carries `not_evaluable` and `compared_against` as FIELDS, so a reader of the
number sees the disclaimer in the same file rather than in a neighbouring
document they may not open. `vit_mutate` refuses the run outright, so the
invalid artifact is not produced in the first place.

## Unit #29 — CheckInputs — the mutation re-take — 2026-08-13

**THE INVALID RUN AND THE VALID ONE ARE NEXT TO EACH OTHER, AND THE ARGUMENT
FOR RE-TAKING IT PREDICTED THE OPPOSITE.** This is the finding, and it is a
correction to something this campaign wrote down as settled.

```
integrated tree, mutant vs itself   4 killed of 173    0.0231   INVALID
clean tree, mutant vs Fortran       8 killed of 173    0.0462   valid
```

`RUNBOOK.md`'s target-layer entry and `scripts/_mutation_stamp.py`'s docstring
both say that *"the same mutants scored on a CLEAN tree give a number at the
other end of the range"*. They do not. The score doubles and stays at the
bottom. The reasoning that DETECTED the invalid run — "169 survivors on a corpus
that passes 16,769 of 16,769 clean is the tell" — reached a true conclusion from
a false premise: 165 survivors is what this unit's corpus produces when the
instrument is working perfectly.

The consequence is procedural. The RUNBOOK proposes a shape-check on the number
(compare `mutation/<U>.json`'s kills against `harness/<U>.json`'s pass count).
On this unit that check fires identically for the valid run and would have been
read as "still invalid". What actually settles it is one `nm` on the reference
object, which is what `_mutation_stamp.py` does and why its field, not its
prose, is the part worth keeping:

```
ReadSetParameters.f90.o  defines __readsetparameters_MOD_checkinputs
                         references no checkinputs_c    -> the Fortran body
```

**WHY 165 SURVIVE, MEASURED RATHER THAN ARGUED.** Three one-line perturbations
of `assign_errmsg`, the single sink every message in the translation goes
through, on the clean tree and the same 16,769-case corpus:

```
aviFAIL = -1 -> -7 everywhere    16,769 of 16,769 FAILED   (control: alive)
no ErrMsg ever written           16,769 of 16,769 FAILED
first-writer-wins                16,769 of 16,769 FAILED
unperturbed                               0 FAILED
```

The second says every case raises at least one error, so `aviFAIL` is `-1` in
all 16,769 and distinguishes nothing. The third says the FIRST failing check's
message differs from the LAST in every case — so every case raises at least
TWO, and `CheckInputs` has no early return, so the last one wins. **The single
discriminating output of a 180-check validator is the message of whichever check
happens to be last.** A mutant anywhere above it changes nothing observable.

That is not weak coverage and it is not a bad corpus. It is a shape: the
generator varies many parameters at once and this unit answers with one string.
Reaching the survivors needs cases that fail exactly ONE check — a near-valid
configuration perturbed one check at a time — and no rule in `harness/` makes
one. Writing that rule is a corpus feature, not a repair to this unit, and it is
NOT attempted here (X3: it would move every unit's corpus).

**DISPOSITION `deferred`, not `integrated`.** The translation is written,
integrated, and shipping: 16,769 differential cases green against clean Fortran,
16,769 more against the integrated build, the gate at 5,252,000 values / 0
mismatched over 27 scenarios. What is missing is P12, at 0.046 against a
threshold of 1.0. A previous dispatch recorded `integrated` while its own
`done_check.txt` said INCOMPLETE, and the Driver reverted it; the number has not
improved, so neither has the claim. `integrated_unexercised` was considered and
rejected — this campaign has reserved that word for units NO SCENARIO REACHES
(DECISIONS §"Can a unit no scenario reaches ever close?"), and `CheckInputs` is
called 25 times across 24 of 27 scenarios.

**The survivors are NOT declared equivalent, and that is the whole point.**
`negate_cond` on `(LoggingLevel < 0) || (LoggingLevel > 3)` inverts a real check.
It survives because the instrument cannot see it, not because it does nothing.
Declaring 165 equivalences would produce a 1.000 and would be exactly the false
equivalence `min_mutation_score: 1.0` exists to shut.

**A 32-MINUTE SWEEP DOES NOT FIT IN A 600-SECOND FOREGROUND COMMAND, AND THE
OBVIOUS WAY ROUND IT IS THE ONE THIS CAMPAIGN HAS ALREADY PAID FOR TWICE.** 192
mutants at ~9.5 s each. Backgrounding it orphans the sweep in the container,
where it writes into the tree after the session that launched it is gone —
`CheckInputs`' own previous dispatch did this and ran 25 minutes as an orphan.
Run instead as five `--operator` invocations, each blocking, unioned by
`scripts/_mutation_merge.py`.

The union is a coverage claim, so it is built to fail. The operator population
is asked of `harness.cppmutate` directly and never derived from the parts: a
filtered run's own `operators` field is computed AFTER the filter, so a union
checked against it would be a tautology — which is how the first version of the
merge was written and what its refusal caught. Being a per-operator COUNT it
also answers the stronger question, that each part scored every mutant its own
operators produce (39+33+40+40+40 = 192).

**The window was opened twice and closed twice**, with a commit before each
opening and immediately after each closing. The second opening was for the
`assign_errmsg` probes, which are meaningless on an integrated tree for the same
reason the mutation run was. `evidence/CheckInputs/errmsg_masking.sh` now opens
and closes that window itself on a `trap ... EXIT` — correct THERE and still
wrong in `reset_to_clean.sh`, because a probe's window lives for one measurement
and the reset's is meant to outlive the script.

### Three of the above may belong to the METHOD, not to rosco-r2

Named here for the Driver to raise as amendments; the invariant layer was not
touched.

1. **A mutation artifact must record the reference side it was compared against,
   read from the build rather than inferred.** `compared_against` and
   `_mutation_stamp.py`'s `nm` read exist in this campaign because unit #29
   needed them. Nothing in the spec requires a mutation result to say what was
   on the other side of the comparison, and a score without that field is
   unreadable — 0.0231 and 0.0462 came from the same 192 mutants.
2. **A score cannot be validated by its shape.** This campaign's own rule was to
   check the kill count against the harness's pass count; measured here, that
   check does not discriminate a valid run from an invalid one. Whatever the
   spec says about recognising a measurement that measured nothing, "the number
   looks wrong" is not it.
3. **A verification step longer than one foreground command needs a defined way
   to be split, or it will be backgrounded.** Three units of this campaign lost
   work to backgrounding or to polling. The prohibition is being enforced by
   prose in the dispatch; the durable form is a tool that splits and a union
   that refuses to over-claim (`scripts/_mutation_merge.py` is this campaign's
   version, and its coverage check had to be derived from the mutator rather
   than from the parts to be non-vacuous).

## 2026-08-13 07:15 — the kill counts that went DOWN: REPLACEMENT, not extension

`c0f8356`'s closing paragraph is superseded. It says the re-takes "score LOWER
than the components they replace, so any union of all five overstates." That
framing is wrong, and appending the correction rather than amending it is
deliberate: the message is evidence of what was believed at 07:05, and a
campaign that rewrites its own history to look consistent removes the record
this project exists to keep.

**What was believed.** That the corpus grew (22,824 -> 23,076), that the five
components therefore measured the same mutants against a superset of cases, and
that the re-takes drifting downward meant the union was an over-estimate — a
number to be corrected downward rather than discarded.

**What was measured.**

- The mutated source never changed. `translations/ReadSetParameters/checkinputs.cpp`
  was last touched at 01:11 (`911affc`); the merged run is 06:14 and the re-takes
  06:37–06:53. Per-operator mutant and nocompile counts are identical across the
  two revisions (29/10, 33/0, 32/8).
- **The three admissible baseline states were EDITED between the runs**, by
  `7f75ca4` at 06:29 — 9 value lines removed against 176 added.
  `CntrPar_VS_ControlMode: 1` deleted, six constant `fill` arrays replaced by
  `{"ramp": [...]}`, the convention string rewritten.
- The "strict extension" claim is true as scoped and false as read. It is prose
  inside `rule_coverage`, and it says R11 is *appended after* R3's scan and the
  random fill "so every case index the other rules produced is unchanged" — a
  statement about rule ORDERING within one generation, not about stability
  across a regeneration. Diffing the two harness artifacts, only R11's line
  differs; the other seven rules are byte-identical. R11 went 1,095 -> 1,347,
  and that +252 is the entire corpus delta. R11 is the last case-generating rule
  (R1 and R4 produce none).

So the corpus across the boundary is:

      0 … 21,728    21,729 cases   unchanged
 21,729 … 22,823     1,095 cases   SAME INDEX, DIFFERENT CONTENT
 22,824 … 23,075       252 cases   new

**4.8% of prior indices were regenerated, and they are precisely the
admissible-state probes that exist to defeat masking.** That is a corpus
REPLACEMENT, not an extension — which is why kills moved in both directions, and
which means no union of the five is valid at all. Not an over-estimate to be
adjusted: a comparison between incommensurable measurements. Nothing needs to be
posited about nondeterminism.

**The strongest evidence is a prediction made before the measurement.** Mutant
`1b723864` (`ZMQ_UpdatePeriod - DT` -> `+ DT`) flipped survived->killed, and
`7f75ca4`'s own message says it set `ZMQ_UpdatePeriod=0.05, n_DT_ZMQ=5, DT=0.01`
specifically so that `a - b` and `a + b` stop agreeing. The state edit did what
it claimed. It did it by replacing cases, not by adding them.

**The apparatus was stricter than the summary.** `0.4624` was arithmetic in a
supervising summary, carrying an accurate caveat ("mixing revisions"). It could
never have entered an artifact: `_mutation_merge.py:189` refuses parts with
differing `loop_rev`, verified by running it on the five real parts — exit 2,
`the parts were produced by different instruments`, nothing written. A number
with a caveat attached outlives its caveat; the refusal does not.

**Consequence for the unit.** `const_tweak` (22/40) and `negate_cond` (36/39)
are not merely un-re-taken, they are INVALID for the current corpus, measured
against states that no longer exist. `84/173 = 0.4855` is a valid score for a
corpus that no longer exists. Re-taking the two is 80 mutants at ~9.5 s ≈ 13
minutes of compute, after which the merge accepts all five at `2e2295f` and this
unit has, for the first time, one commensurable number.

### A fourth finding for the METHOD

4. **An artifact must record the INPUT that makes its claim recomputable, not
   only the claim.** Two symptoms, one defect, and the second was found by
   noticing the first could not be checked:
   - `rule_coverage` asserts "a strict extension of the one before it" and
     carries nothing by which a reader could test it. A hash of the generating
     states would have made the 06:29 edit visible as a corpus change at the
     moment the next sweep was stamped.
   - The mutation artifacts record only SURVIVORS, so "the same population was
     scored at both revisions" could only be inferred here from matching mutant
     and nocompile counts. A mutant-ID list would have settled it directly.

   These are the same fix applied twice, and the same shape as finding 1: the
   artifact carries the conclusion and withholds the input. Finding 1 was about
   the reference side, this is about the corpus and the population — three
   instances now, which is what makes it a method question rather than a
   rosco-r2 one.

### Refinement, same day: the corpus changed on BOTH sides at 06:29

The entry above attributes the replacement to `7f75ca4`'s state edit. That is
half of it, and the half visible from inside this campaign. The loop-side commit
`2e2295f` — *"R11: an admissible array must be DISTINCT, and a whole-array knob
is placed"* — landed at 06:29:27, sixteen seconds before `7f75ca4` at 06:29:43.
They are one change made in two repositories, and `harness/generate.py` moved 37
lines:

- R11 now accepts `{"ramp": [start, step]}` in a baseline state at all. The state
  edit could not have been written before the generator understood it.
- **A whole-array knob is now PLACED at one element — first, interior, last —
  as well as broadcast.** Previously only broadcast.

That second item is where the +252 comes from: a whole-array knob now yields
four cases where it yielded one. And it independently explains why prior indices
moved even where a state's values were untouched — the knob's placement changed,
so the case at a given index is a different case.

The loop-side message states the defect in the terms this campaign found it:
49 of 89 survivors were one shape, because R11's inert fill is zeros and its
whole-array knob broadcast, so every array in its block was constant — and an
index shift, a loop-bound shift and a `<` on a loop bound all land inside a run
of identical values. *"R5 already requires 'array elements distinct' of the
corpus; the rule written to remove masking did not have that property itself."*

**So `loop_rev` is not a version label, it is the corpus generator's identity,
and `_mutation_merge.py:189` refuses across it for exactly the right reason.**
The guard would have been correct here even if the states had never been edited.

One residual, named because it is not resolvable from the artifacts: `-nogit`
means the rev was read from `.git/HEAD` directly, because `vit-dev` has no git
binary, and that path CANNOT SEE A DIRTY TREE. `2e2295f-nogit` therefore asserts
HEAD and is silent on cleanliness. The loop checkout is clean now and still at
`2e2295f` with no commits since 06:29:27, so the pending re-take is
commensurable; whether the three existing parts were taken against a clean loop
tree is not recoverable. This is finding 4 once more — the artifact records the
conclusion (`2e2295f`) and withholds the input that would make it checkable.

## 2026-08-13 09:30 — CheckInputs: one score, of one corpus, of one reference side

The two stale operators were re-taken, the merge accepted all five, and this
unit has for the first time a number that is a score of something nameable.

**What the number is.**

```
mutation/CheckInputs.json   192 mutants   19 nocompile   173 behavioural
                            78 killed     95 survived    score 0.4509
```

and it is a score OF, in the three axes that have each been wrong here once:

- **corpus** — `harness/CheckInputs.json` at **23,076 cases**, loop_rev
  `2e2295f-nogit`. Not regenerated, not widened, not pinned: untouched by this
  dispatch. The generator's identity is the same `2e2295f` for all five parts,
  which is the thing `_mutation_merge.py:189` refuses across.
- **reference side** — `fortran`, stamped by `_mutation_stamp.py` BEFORE the
  restore, from `nm` on the build tree as it stood: `ReadSetParameters.f90.o`
  defines `_MOD_checkinputs` and references no `checkinputs_c`, so the reference
  side ran the Fortran body and nothing routed back into the mutant.
- **population** — 192 mutants over 9 operators, the population asked of
  `harness.cppmutate` over `translations/ReadSetParameters/checkinputs.cpp` and
  never derived from the parts.

**Per operator, across the corpus boundary.** Only the last two rows were
re-taken; the other three were already at `2e2295f` and are byte-unchanged.

```
operator      behavioural   813e7a2      2e2295f
calls_arith        29         --          8      (already re-taken 06:37-06:53)
compare_op         32         --         11
index_offset       33         --          3
const_tweak        40         22         23      <- re-taken here
negate_cond        39         36         33      <- re-taken here
                  ---                   ---
                  173         84*         78
```

`*` the committed `84/173 = 0.4855` was the 06:14 merge with all five at
`813e7a2`. It is superseded, not corrected: the two artifacts describe different
corpora and neither number is a refinement of the other. **The re-takes moved in
opposite directions** — `const_tweak` up one, `negate_cond` down three — which is
the corpus REPLACEMENT of the 07:15 entry showing up one more time. An extension
could only move kills upward.

**And a third number, which is the one to keep.** `plan.json` and `STATUS.md`
were both still carrying `0.0462` (8 of 173), from the single clean-tree sweep
taken before R11 existed. That sweep's corpus was **16,769 cases**; the five-part
sweeps ran against 22,824 and then 23,076. So the sequence 0.0462 → 0.4855 →
0.4509 is not an instrument settling down, it is three corpora:

```
16,769 cases   8 of 173    the masking argument at its strongest
22,824 cases  84 of 173    R11's admissible-state probes reach 76 more mutants
23,076 cases  78 of 173    R11's whole-array knob PLACED as well as broadcast
```

The masking finding is therefore **narrower than it was written, and not
withdrawn**. `CheckInputs` still has no early return, still writes one message,
and 95 mutants above the last failing check still change nothing observable. But
"165 of 173 are invisible" was a fact about a 16,769-case corpus and reads as a
fact about the unit; at 23,076 it is 95. A blindness claim carries its corpus or
it decays into a claim about the code.

**Disposition `deferred`, restored.** `c1311bc` cleared it to null because the
number `deferred` rested on had been withdrawn. That reason is now resolved and
nothing else about the unit moved: the translation is byte-identical
(`bcdf486d`, proved twice by `mutate_guarded.sh`), the gate artifact after the
restore differs from its predecessor **in no field at all** — not even
`loop_rev`, since the predecessor was already at `2e2295f` — and 27 scenarios /
351 channels / 5,252,000 values / 0 mismatched. `P12` fails at 0.4509 against a
threshold of 1.0. **That is the honest outcome of this dispatch and not a
shortfall of it**: the alternative was declaring 95 equivalences, and the
`negate_cond` survivors include `if (a == nullptr)` inversions that plainly are
not equivalent.

**The signature prediction is CONFIRMED, and it was confirmed by evidence that
already existed rather than by anything run here.** Three view types
(`CntrPar`, `LocalVar`, `ErrVar`), an assumed-size `REAL(ReKi)` array and a
scalar `INTEGER`; none of the five blocking features the conformance matrix
could not attribute (`c_alloc_out`, `c_fn_alloc_character`, `c_fn_array`,
`c_unit_read`, `c_unit_write`) appears in it. The wrapper `vit integrate --apply
--reverse-copy` emitted is in the shipping library right now — this dispatch
de-integrated the tree and put it back, and the gate is bit-identical across
that round trip, which is a stronger statement about the bridge than the
original integration was. `--reverse-copy` is required and remains so: both
outputs live inside a view. No argument failed and no feature blocked.

### Two artifacts observed stale, NEITHER touched

Named for the supervisor because acting on either is a corpus decision:

1. `harness/CheckInputs.postintegration.json` is at loop_rev **`813e7a2`** with
   **22,824** cases — one corpus revision behind the pre-integration artifact it
   is paired with (23,076 at `2e2295f`). This is unit #26's red-test-corpus-skew
   census in its post-integration form, which that census reported as
   `26 of 26 EQUAL` precisely because post mode reuses the generating run's case
   file. Regenerating the corpus broke the structural guarantee. `P11` passes on
   it, and what it measures — the wrapper's marshalling — is not corpus-sensitive
   in the way the arithmetic is. Re-taking it is one harness run and no corpus
   change; it was not in this dispatch's scope.
2. `harness/CheckInputs.json` carries `"against": "integrated"` while occupying
   the pre-integration slot, written by `c0f8356`'s recovery. The mutation
   numbers above do not rest on that field — `vit_mutate.py` performs its own
   comparison on the clean tree and `_mutation_stamp.py` read the reference side
   out of `nm` rather than out of any artifact — but a configuration field that
   disagrees with its slot is exactly the defect `compared_against` was added to
   close, one file over. It is finding 1 again, in the harness rather than the
   mutation artifact.

## 2026-08-13 09:53 — the six named survivors: four unreached, two reached and overwritten

The re-take dispatch's eight steps were already complete at `48b9ae3` — five
parts at `2e2295f`, a merged artifact the merge agreed to write, a stamp naming
`fortran` as the reference side, a gate proving the restore, `done_check` at 12
of 13. What remained open is P12's own list: six `arith_op` survivors, to be
sorted into equivalent / unreachable / uncovered-gap. **None of the six is
equivalent. Four are (b) and two are (c)**, and the classification is measured
rather than argued — `evidence/CheckInputs/corpus_index_shift_reach.py` reads
the 23,076-case corpus these mutants were scored against and reports, per site,
whether the mutant's branch answer ever differs from the reference's. Full
triage in `evidence/CheckInputs/survivors_arith_op.md`.

All six, and four unnamed siblings, are one shape: `[i - 1]` -> `[i + 1]`, the
zero-based index conversion inside a Fortran-indexed loop.

**DISTINCT IS NOT DISCRIMINATING, AND THE `ramp` REPAIR SUPPLIED THE FIRST.**
Round one found 49 of 89 survivors alive because R11's arrays were constant, and
`2e2295f` fixed that with `{"ramp": [start, step]}` and a placed whole-array
knob. The fix worked and these mutants still survive:

```
mutant      line array              guard    iters  valdiff  conddiff   cases
2ed97e42     925 AWC_freq           18616       19       19         0       0
e250cdea     932 AWC_clockangle       462        6        6         0       0
91a7adbc     933 AWC_clockangle       462        6        6         0       0
e712c281     946 AWC_clockangle        31        0        0         0       0
```

`valdiff` is every iteration and `conddiff` is none of them. The two elements
the mutant confuses are different numbers each time and the predicate gives them
the same answer each time, because **every element of an R11 array is admissible
by construction** — `AWC_freq` ramps 1.0, 2.0, 3.0 and `< 0.0` is false at both
indices. Killing an index shift needs three properties: the loop must run, the
array must be distinct, and some element must sit on the far side of the tested
literal. R11 supplies the second. The other two are each a SECOND quantity away
from baseline, and R11 moves one on purpose. Bucket A's structural limit
arriving from a new direction, with the same answer: a state is how R11 reaches
a conjunction.

`e712c281` is the strongest form: its block is `AWC_Mode == 2`, the corpus holds
31 such cases and **none has `AWC_NumModes >= 1`**, so the body never executes.
Nor can a range reach it — the reference refuses `AWC_NumModes` outside `{1,2}`
under `AWC_Mode == 2`, so an admissible configuration must set both, and all
three baseline states carry `AWC_Mode` of 0 or 1.

**A KNOB THAT ARMS THE CHECK IT WAS AIMED AT ALSO ARMS THE ONE BELOW IT.** This
is the (c), it is new, and it is the finding of this dispatch. `29e57417` and
`bafd410e` ARE reached — by a case built to reach them:

```
  cable  cases whose SEGMENT differs        18611
  cable  cases whose any_lt DIFFERS             1
  case 22752 segment  [-1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]
```

Case 22752 is the `open_loop_flap_fbp` state with exactly one knob:
`Ind_CableControl(1) = -1`. The reference reads element 1, sees `-1`, raises
*"All open loop control indices must be greater than zero"*; the mutant starts
at element 3 and does not. R11 did exactly what it was written to do. Then,
twenty-four lines down, `any_gt(Ind_CableControl, 0) .AND. CC_Mode /= 2` reads
**the same array**, fires on the `11, 12, 13, ...` the knob did not touch, and —
there being no early return — overwrites the message. Both programs end with
`aviFAIL = -1` and the identical string. 18,103 of the 18,161 cases with a
negative cable index are dominated this way.

The recorded masking finding does not cover this. Its form is statistical —
*every case raises at least two errors, so the last wins* — and its repair IS
R11, a case that fails exactly one check. Here R11 succeeded and the case still
fails two, because the two checks read **one parameter**, so no one-at-a-time
perturbation can separate them. And the ramp repair guarantees it: a constant
zero array leaves `any_gt(..., 0)` false and the dominating check silent, so
making the array admissible-and-distinct is exactly what arms it. **The repair
for the first survivor is the mechanism that hides the second.** That is not an
argument against the repair — it moved the score from 8 to 78 — it is an
interaction between two corpus rules that nothing in the apparatus can see.

**Nothing was applied.** The repairs are named per mutant and withheld: a fourth
baseline state at `AWC_Mode = 2, AWC_NumModes = 2`; ramps that straddle the
tested literal inside the first `N+2` positions; and, for the (c) pair, a state
carrying `CC_Mode = 2` and `StC_Mode = 2` so the dominating check is already
satisfied when the knob moves the earlier one. Every one of those edits
`harness/baseline.CheckInputs.json`, which changes the corpus and invalidates
the five parts `0.4509` rests on — the exact thing the re-take existed to
establish. `equivalent_declared` stays at **0**: not one of the six agrees with
the reference on every admissible input, and declaring otherwise is the false
equivalence a threshold of 1.0 exists to shut.

**P12 still fails, at 78 of 173 = 0.4509, and the disposition stays `deferred`.**
The score did not move and was not supposed to. What is new is that every one of
the six now has a reason and a repair attached to it, and that the reason for two
of them is a shape no rule in this campaign detects.

### A fifth finding for the METHOD

5. **A corpus rule can be defeated by a second rule that its own repair
   strengthens, and no artifact relates the two.** R11 isolates a check by
   moving one quantity; `ramp` makes the array distinct so an index shift is
   visible. Together they arm a *later* check on the same parameter, and the
   isolation R11 claims in its `rule_coverage` prose is silently false for any
   parameter that two checks read. The claim is per-rule and the defect is
   between rules. This is finding 4 once more — the artifact carries the
   conclusion (*"exactly one quantity away from admissible"*) and withholds what
   would test it (which CHECKS that quantity reaches).

## 2026-08-13 10:30 — the evidence-hygiene audit, and why the predicate is NOT in the loop

`scripts/revcheck.py` asks three questions of every unit's result artifacts and
`scripts/revcheck.redtest.py` proves it can answer them wrongly. Standing result
on this campaign, re-runnable rather than recorded:

    BASE-SHA SPLIT   9 units    (4 over the five core families alone)
    NO loop_rev     17 units
    DIRTY TREE      12 units

**None of this shows any closed unit's result is WRONG.** It shows the evidence
cannot be checked for commensurability, which is a different and weaker claim —
the same distinction between "the corpus change is global" and "the
concentration is narrow". 24 of the findings are on closed units and are
reported as ADVISORY. Reopening 27 dispositions on evidence-hygiene grounds
would cost more than it buys, and `--strict` exists for the run where that is
actually the question.

**WHY THIS IS A CAMPAIGN SCRIPT AND NOT A PREDICATE IN `loop/done.py`.** The
obvious home is the done-condition: P1–P13 already live in the loop, and this is
the same kind of question. It is not there, and the reason is mechanical rather
than procedural.

`loop_rev` is the loop checkout's own HEAD. **Committing the predicate to the
loop bumps `loop_rev` for every artifact taken afterwards.** CheckInputs is
currently split `mutation @ 2e2295f` against `postintegration @ 813e7a2`, and
the fix is to re-run post-integration at `2e2295f`. Add the predicate to the
loop first and that re-run stamps the NEW rev, leaving `mutation` at `2e2295f`
alone — the split does not close, it MOVES, and the check would be reporting a
condition its own installation created. The tool that measures the corpus
generator cannot ship inside the corpus generator without perturbing what it
measures, which is this repo's read-only-evaluation rule arriving from an
unexpected direction.

So the order is: campaign script now, post-integration re-run at `2e2295f`,
**then** the loop-level predicate as a proposed method amendment — by which
time a rev bump costs nothing because nothing is mid-split.

### Finding 4, third instance, now with a shape to fix

Raised for the method rather than applied. `_mutation_merge.py:189` refuses
parts from different `loop_rev`s inside a merge; the done-condition asks nothing
equivalent across a unit's artifacts, and 17 units carry artifacts where the
question cannot be asked at all because the field is absent. The durable form is
a loop predicate asserting all three, with `harness/` and `gate/` writers
stamping `loop_rev` unconditionally so assertion 2 becomes unfalsifiable-by-
construction rather than commonly violated.

### A note on how both errors in this investigation were caught

The last two exchanges each contained one measurement error. A campaign-wide
grep for array intrinsics ran against an INTEGRATED tree, where 27 closed units
are wrappers, and reported zero intrinsic uses in CheckInputs while its
translation carried 18 helper call sites. A hand comparison of `loop_rev`s
compared the SUFFIXED strings and reported `interp1d` as split when it is
`20b0dbb` throughout.

Neither was caught by care. Both were caught by a second number contradicting
the first — and `revcheck.py` then reproduced the same shape twice in its own
output, printing `NO loop_rev on 0 unit(s)` directly beneath seventeen units
reporting it. That is the argument for the predicate stated more honestly than
any reasoning about diligence: the apparatus is what holds a fact between the
moment someone measures it and the moment anyone needs it again.

## 2026-08-13 12:15 — the 14 units with no reading, and the fix that would have hidden them

`revcheck.py` now asks five questions, not three. The last two were added by
DECIDING NOT TO FIX something, which is worth recording as a decision rather
than as an absence.

14 of 29 closed units carry no `evidence/<u>/done_check.txt`. The one-line
remedy exists — `bash scripts/capture_done_check.sh <u>` — and taking it would
have been a fresh instance of the defect this campaign has spent two days
removing. Every predicate `done_check` evaluates reads the CURRENT tree. A
capture made today for a unit closed on 08-11 describes today, while reading, to
anyone later, as the reading taken at the close. **A visible honest absence
would have become an invisible dishonest presence, and the check that would have
caught it is the one the capture had just satisfied.**

So the encoded question is not "does a file exist" — which generating a file
answers — but "was the reading taken while this unit was still the unit being
worked", measured with no time threshold: no other unit may have gained a
disposition between the closing commit and the capture's commit.

    NO RECORDED READING   14      RETROACTIVE READING   3

The three retroactive ones are this session's own re-captures of `Int2LStr`,
`interp1d` and `wrap_360`. They are accurate readings of today and those units
are genuinely COMPLETE 13/13 — and they are still not readings taken at the
close, so the check reports that about work committed an hour earlier. That is
the property worth having: it does not exempt its author.

### Two results from today that generalise past this campaign

**These defects are not biased toward over-claiming.** `Int2LStr`, `interp1d`
and `wrap_360` were PENALISED by their own evidence — recorded INCOMPLETE 12/13
when they are COMPLETE 13/13, the sole failure being an artifact of how the file
was captured. Every prior instance in this campaign flattered its subject, which
made "silent failures flatter" an easy assumption to hold without stating. It is
false. The failures are noise, and noise is symmetric; it was merely silent in
both directions.

**Four things surfaced today, and none was found by care.** The kill-count
anomaly, the integrated-tree grep, the suffixed-`loop_rev` comparison, and this
file's own summary printing `NO loop_rev on 0 unit(s)` beneath seventeen units
reporting it. Each surfaced because two numbers disagreed. A fifth: three
unrelated units reporting an identical P2 failure, which was the checker reading
a tree its own uncommitted output had dirtied — the same self-reference as the
`>` truncation, one level up, and a contradiction detector nobody designed.

The apparatus is not a substitute for attention. It is the thing that holds a
fact between the moment someone measures it and the moment anyone needs it
again, and today it was more reliable than either party to the conversation.

## 2026-08-13 13:40 — P14 landed in the loop; the Driver-level call has NOT, and carries a constraint

`loop/done.py` gained P14 at `08f8166`: the artifacts a unit's verdict rests on
must name the same instrument. Three assertions — absent `loop_rev`
(NOT_EVALUABLE), a `-dirty` suffix (FAIL), disagreeing base SHAs (FAIL). Suite
442/2 before, 447/2 after, including five tests that assert P14 discriminates
rather than merely passes.

**Three, not five, and the reason is sharper than "circular".** Whether a unit's
done-condition was ever read, and whether that reading was contemporaneous, are
audit-time questions about already-closed units. Asked at close they would not
fail uniformly — they would fail ASYMMETRICALLY. A first close has no capture
and would fail; a re-dispatch would find the previous attempt's file and pass.
The predicate would reward exactly the units that had trouble, and do it
silently. Those two stay in `scripts/revcheck.py`, where they are advisory on
closed units and cannot be cleared by generating files.

### The constraint on the Driver-level call, written down before it is needed

A Driver-level `revcheck` at unit close is the thing that makes assertions 4 and
5 non-opt-in. It is deferred, and it **inherits today's sequencing constraint
exactly**: `loop/driver.py` is in the loop repo, so landing it bumps `loop_rev`
for every artifact taken afterwards. It must therefore land BETWEEN UNITS, WITH
NOTHING IN FLIGHT — never mid-dispatch, or it manufactures the split it exists
to report. That constraint existed only in conversation until this paragraph,
and "we'll do it later" without it is precisely how the trap gets sprung during
a 40-unit run.

### A consequence for CheckInputs, now true

`loop_rev` is now `08f8166`. CheckInputs' nine artifacts are all at `2e2295f`,
so P14 passes — it compares artifacts to EACH OTHER, not to the current
generator. But the unit is one generation behind, and **re-taking any single one
of its artifacts would split it.** Whoever next works CheckInputs re-takes all of
them or none. That is the cost of the bump, it was accepted deliberately, and it
is cheapest now because the unit is closed as `deferred` and `next_unit` skips
it, so a restart begins at `ChkParseData` and cannot walk into this by accident.

### The bug in the predicate, which was the shape the predicate exists to catch

`except Exception` around the artifact parse swallowed a `NameError` from a
missing `import json` — this module imports json inside each reader — and P14
reported *"no configured artifact was readable at this commit"*. That sentence
reads as a fact about the campaign; it was a fact about the code. Nothing about
it looked wrong. What exposed it was three fixtures whose artifacts plainly
existed on disk: the sixth time today a contradiction, not care, did the work.
The except is now `ValueError`, narrow enough that a defect cannot pose as an
absence.

## 2026-08-13 14:20 — `loop_rev` is the loop's HEAD, not the generator's identity

**`db7a550`'s conclusion is narrowed here.** It said *"loop_rev is not a version
label, it is the corpus generator's identity"*. That was true of the case that
produced it: between `813e7a2` and `2e2295f`, `harness/generate.py` genuinely
moved 37 lines and the corpus genuinely changed. It is NOT true in general, and
the counterexample arrived one commit later.

`2e2295f -> 08f8166` (P14) touches `CLAUDE.md`, `README.md`, `loop/done.py`,
`tests/test_done.py`, `tests/test_driver.py`. **Nothing under `harness/` changes
at all** — not just `generate.py` but the whole generation path, confirmed by
diff and by object hash (`2e2295f:harness/generate.py` and
`08f8166:harness/generate.py` are the same blob). `loop_rev` is the loop REPO's
HEAD, a superset of the generator, and it moves for reasons a corpus never sees.

### So the all-or-none hazard recorded at a29e137 is mostly spurious

A re-take of any CheckInputs artifact at `08f8166` would run an unchanged
generation path over unchanged baseline states, and the regeneration determinism
was measured this morning at `21ea985` — same inputs, byte-identical 567 MB case
file. It would therefore produce the SAME corpus with a DIFFERENT stamp, and P14
would report a split between artifacts that are substantively identical. The
"all or none" cost is real as the tool behaves and largely fictional as the
evidence stands.

### And it generalises, which is the part that matters

The planned Driver-level `revcheck` call lives in `loop/driver.py`, so it bumps
`loop_rev` again without touching the generator. After it lands, **every unit
closed before it is "one generation behind" exactly as CheckInputs is now**, each
all-or-none on reopening for a reason having nothing to do with its corpus. Over
40 units with any further loop commits, that set grows by one unit every time
the loop is touched for any reason.

P14 is still correct as built — it compares artifacts WITHIN a unit, so it raises
nothing for units nobody reopens. The exposure is precisely at reopening, which
is when a spurious signal is least welcome.

### Promoting finding 4's generating-state hash

The fix is already written down as finding 4: a stamp derived from
`harness/generate.py` plus the baseline states, rather than from the loop's HEAD.
Such a stamp would not move on a `done.py` commit, which would leave CheckInputs'
nine artifacts commensurable with a future re-take instead of frozen against it.

**Promoted from "a method question" to the amendment AFTER the Driver-level
call.** Not urgent today — nothing is being reopened — but it is the thing that
stops the all-or-none set growing by one unit every time the loop is touched,
and the Driver call is itself the next event that would grow it.

### Loop-side commits now have a second copy

Three loop commits were made today (`813e7a2`, `2e2295f`, `08f8166`) and each
existed only in the campaign clone's working tree until a manual backup ran.
Canonical already has that clone registered as the remote `campaign-clone`, so
the durable copy is a FETCH, not a push: `git -C <canonical> fetch
campaign-clone` moved `campaign-clone/main` from `9119db2` to `08f8166` and
copied every object. No branch was created, canonical's `main` is untouched at
`891fde3`, and the name `campaign-work` was deliberately NOT reused — the clone
already has a `campaign-work` at `cf885e3` which is NOT an ancestor of this work,
so pushing this history under that name would have misdescribed it.

## 2026-08-13 15:10 — the restart configuration: 14400s, and what it actually costs

**`--timeout-s 14400`, and the number is only safe because something now reads
it.** A four-hour deadline with nothing consulting it is not generosity, it is a
later wall: a session starts a nine-minute sweep with four minutes left and dies
mid-mutant, which is what 7200 did to unit #29 and what 14400 alone would have
reproduced at 10:59 instead of 06:59. A four-hour deadline that
`run_if_time_remains.sh` can see is a session that declines the last piece and
closes clean. The generous number and the guard are one change and they land
together; neither is correct without the other.

That is also why the guard REFUSES on a missing deadline rather than shrugging.
A guard that passed when it did not know would be worse than no guard, because
it would occupy the slot where one is expected.

### What four hours can cost, stated correctly

An earlier estimate of ~$28 was for a unit whose extra time is mostly spent
BLOCKED on a foreground command — the legitimate case, and probably the common
one, since blocked wall-clock is nearly free. It is not the ceiling.

Cost tracks TURNS at a stable rate: $0.0749–$0.1927 per turn across 32 recorded
dispatches, a 2.6x spread. CheckInputs' first dispatch reached **$176.77 at
$0.1945/turn** — a perfectly ordinary rate sustained over 909 turns against a
mean of 144. It was a long session, not an expensive one. Four hours spent
actively working is therefore ~$50–200, and

**the per-unit ceiling is TWO dispatches, not one.** `--max-retries` defaults to
1, and `loop/driver.py:278` already names the consequence: *"At $25 with one
retry a unit can reach $50 unremarked, 66 more times."* So a pathological unit's
worst case is roughly double again — a few hundred dollars, not $28.

Against ~$3,130 remaining and a $19 mean across 40 units, the headroom absorbs
several such outliers comfortably. The answer is unchanged; the estimate is not,
and a reassuring number that is wrong is worth less than an uncomfortable one
that is right.

### The R11 note has a half-life, and something is already queued to end it

`ac17746` pinned the R11 question to `interp2d` (order 45) and
`ReadControlParameterFileSub` (order 64) as a plan.json `note`, and recorded
that it is a note and not a gate — nothing enforces reading it. The enforcing
version is a preflight predicate, rejected today because it lives in the loop
and a loop commit costs a `loop_rev` bump for no present benefit.

**That objection expires on its own.** The Driver-level `revcheck` call is
already queued and is itself a loop change, so it will bump `loop_rev` at some
point before order 45 is reached. When it lands, the preflight predicate rides
along for free. The two should be considered together rather than the note
quietly outliving its half-life — which is exactly what a note does if nobody
writes down when it stops being adequate.

## 2026-08-14 01:11 — the three queued loop changes landed, and a fourth defect found in the ledger

The ChkParseData shakedown closed COMPLETE 14/14 and the campaign halted by
design at 35 units, so the window with nothing in flight opened. All three
changes queued for it landed in that window, which is the constraint written
down at 13:40 and now exercised: `loop/driver.py` and `harness/` are in the loop
repo, so each bump `loop_rev` for every artifact taken afterwards.

    3587239  driver: a retried unit's row totals the unit, not its last dispatch
    98a633d  driver: revcheck at unit close, as an escalation and never as a gate
    ed623d9  harness: an artifact names the GENERATOR that produced it
    45a692b  run_campaign: --revcheck, so the Driver-level call is reachable

Suite 461/2, docs updated to 463 collected. Each was red-tested; two were caught
by their own tests mid-writing, both surfacing as UNVERIFIABLE verdicts, which is
the guarded boundary turning a programming error into a statement about the unit.

### A sixth finding for the METHOD

6. **A retried unit's ledger row recorded its LAST dispatch and called it
   `exact`.** ChkParseData cost $30.61 then $29.65; its escalation read
   `unit_overran:$60.26 across 2 dispatch(es)` and its row read `29.65`. Not a
   missing row — a present one, wearing the word that means trust this number,
   understating by 51%.

   MEASURED: 8 of 35 rows carry `retried: true`. Rows sum to $672.76 against a
   driver total of $752.55; of that $79.79 gap, $45.77 is directly attributable
   ($15.16 ExtController, $30.61 ChkParseData) because those two also breached
   the cap and so left an escalation naming the true figure. **The other six are
   understated by an amount that appears nowhere at all**, because `unit_overran`
   fires only on a breach: under the cap, the earlier dispatch's cost exists in
   no file. This is very likely the same object as the $43.50 gap carried since
   09:30, though the remaining $34.02 has not been independently confirmed.

   The driver already computed the total, for the escalation, and discarded it.
   Fixed by carrying the earlier `Usage` objects, so the row totals the unit AND
   can be decomposed: a total nobody can take apart is the next unreadable
   number. UNKNOWN stays contagious — one unpriced dispatch makes the total
   `None`, never a partial sum wearing `exact`.

### The fetch-not-push decision is superseded, and why

14:20 recorded the durable copy as a FETCH, with canonical's `main` deliberately
untouched at `891fde3`. Its two reasons were that pushing under the existing
`campaign-work` name would MISDESCRIBE the history (that branch at `cf885e3` is
not an ancestor of this work), and that a fetch sufficed for durability.

The second stopped being true. Every copy of these 11 commits — the clone's
working tree, canonical's object store, the backup bundles — was on ONE disk.
`clone_drift` had also gone ACUTE and would have fired on all 40 units of the
next run, which is noise that buries the firing that matters.

So: canonical fast-forwarded to `45a692b` (verified an ancestor first — 11
commits, no merge, nothing rewritten), suite re-run IN canonical before pushing
because a drifted checkout that fails its own suite is the `instrument_regressed`
hazard, then pushed to `github.nrel.gov`. The remote had been at `eead326`,
behind canonical, and that sha was confirmed an ancestor so nothing was lost.
The clone then fetched. Both drift conditions now verified clear by asking the
question `_instrument_drift` asks: HEAD is reachable from `refs/remotes/origin/main`,
and HEAD is attached.

The naming objection never applied to a fast-forward of `main`, which describes
the history exactly.

### Before the 40-unit launch

`--revcheck 'python3 scripts/revcheck.py --unit {name}'` must be passed, or the
banner prints `NOT CONFIGURED` and no unit is asked whether its done-condition
was read — the state that let 14 units close without one. The machine was on
BATTERY at the time of writing (4h13m), which is shorter than the run.

## Unit #31 — Debug — 2026-08-14

### The gate is not blind to this unit because of a corpus; it reads a different stream

`plan.json` predicted `the gate cannot decide` and the reason turns out to be
sharper than any of this campaign's earlier blind units. `Examples/vit_sim.py`
builds `baseline_arrays/*.npz` from the `avrSWAP` channels the controller
RETURNS. `Debug` writes `<RootName>.RO.dbg` and assigns no argument of its own
signature. It is called ~408,000 times across 23 of the 27 scenarios and does
its job every time — so the shape is neither #27's dead branches nor
#1/#21/#26's dead call sites. **No input could make the gate read a file it does
not open.**

The general question this raises, and it is a method question rather than a
target one: **for a unit whose effect is I/O, ask which STREAM the gate reads
before asking which INPUTS reach it.** Coverage answers the second and says
nothing about the first, and the two look identical in a red test that returns 0.

### A unit that assigns nothing in its signature has no generated harness, and the answer is not to give up on P13

`vit test-validate` compares a unit's mapped outputs. This unit has none, so the
generated differential harness would compare the empty set, and `vit_mutate.py`
would report every mutant surviving — a 0.000 that is a fact about the
instrument. `plan.json`'s proposed verification (`generated harness + mutation
score`) was therefore half unachievable as written, and it was re-specified at
the unit (C1) rather than waived.

What replaced it keeps the SHAPE of P11/P13 and changes only the oracle:
`scripts/dbgcheck.py` compares the bytes the unit writes, and
`scripts/dbgmutate.py` scores mutants against that same comparison by compiling
each into `libdiscon.so` and re-running scenarios. **Candidate method amendment:
P13's "the generated input domain IS the specification" should be read as "the
generated *oracle*", so that a respecify unit whose contract is (data in) →
(bytes out) is not disqualified from a mutation score by the shape of its
signature.**

### A mutation reference that CANNOT be the mutant, without needing a clean tree

Unit #29's finding is that `vit_mutate` on an integrated tree routes both sides
of its comparison through the harness's own copy of the mutant, and the remedy
recorded there is to run the sweep on a clean tree. That remedy is expensive —
reset, rebuild, regenerate — and it is not the only one.

Here the reference is a **committed archive of bytes**, written by a build that
contained no C++ `Debug` at all. It is fixed before the sweep starts and nothing
the sweep does can perturb it, so the sweep runs on the INTEGRATED tree and is
still a measurement. **Candidate: where an oracle can be frozen as data rather
than as a build, the clean-tree requirement is about the reference and not about
the tree.**

### One input parameter was worth more than any corpus rule

A third of this unit — 159 `LocalVarOutData` assignments, the 159-name heading
table, the `avrIndices` construction with both `AddToList` loops, the
vector-subscripted `avrSWAP(avrIndices)` write — was reached by nothing, because
all 14 shipped `DISCON*.IN` set `LoggingLevel = 1` and the two guards are
`> 1` and `> 2`. Adding `run_scenario_28` (scenario 3's configuration at
`LoggingLevel = 3`, outside `scenario_order` so no baseline moves) took the same
perturbation from 0 of 408,072 records to 15,999 of 48,014, and took
`compare_op` from 12 of 40 killed to 21 of 40.

**Ask of any unit gated on a configuration scalar: what values does that scalar
take in the shipped inputs, and which arms does that leave unreachable?** It is
one grep and it is not the same question as coverage — the guard LINE has hits
in 23 scenarios.

### Three mutator defects, and the third was found by running the sweep

`compare_op` rewriting a user-declared template's angle brackets (unit #21's
`static_cast` finding one shape out); `arith_op` reading `std::FILE* f`,
`return *p` and **the exponent sign of `1E-99`** as arithmetic; and
`_not_arithmetic` — the fix for the second — reading `drop_factor`'s RIGHT
OPERAND as its operator, so it fired for two rules of three.

The third is the one worth keeping. It was invisible in the diff and obvious in
the sweep: 6 of 7 no-compiles in the operator it was supposed to have fixed.
**A fix to a mutator is not verified by the operator that motivated it.**

And the `1E-99` case is worse than a ratio: it meant the two clamp constants of
the unit under test had **no `arith_op` mutant at all**, so an operator that
appeared to be measuring them was measuring nothing there. A no-compile is not
only noise in a denominator; it can be a hole in coverage wearing a denominator's
clothes.

### `_mid` tracks an ordinal, not a site

For a punctuation operator `_mid` is `(unit, operator, '<', '<=', nth)`. Under
the first mutator fix **twelve sites moved and three ids changed**. Unit #24
established that an ADDITION cannot move an existing id because `_mid` is
content-derived; the converse does not hold, and an artifact that reports only
ids cannot say where its survivors are. `scripts/dbgmutate.py` records
line/before/after per result. **Candidate: `vit_mutate.py`'s survivor records
should carry the line and the surrounding text for the same reason.**

### The merge refusing on `loop_rev` was worth its 30 minutes

Two mutator fixes landed mid-sweep, so the first union of nine parts named three
different instrument revisions and `dbgmutate_merge.py` refused. Six parts were
re-taken. The alternative — exempting the check because "those operators were
not affected" — is an argument, and the check exists because an argument is not
a measurement. `_mutation_merge.py` has the same rule and it is right.

### Two equivalent mutants were NOT declared, deliberately

`pos > buf.size()` → `>=` (the extra call is `append(0, ' ')`, a no-op) and
`if (UnDb)` → `if (!(UnDb))` on the close block (stdio flushes every open stream
at process exit, so the bytes are identical). Both are almost certainly
equivalent. Declaring an equivalence is a claim, and this dispatch did not have
the budget to prove either to the standard units #23–#26 set. The score carries
them: 0.6798 with them counted against it rather than 0.6910 with them excused.

## Unit #31 `Debug`, second dispatch

### The equivalences the first dispatch declined are now declared, and one of them was wrong

The entry above says `pos > buf.size()` → `>=` and `if (UnDb)` → `if (!(UnDb))`
are "almost certainly equivalent" and were left counted against the score for
want of budget to prove them. Proving them cost about ten minutes each and the
second one **did not survive the proof**: `if (!(UnDb))` differs from the
original exactly when `UnDb` is null at the close, which requires `std::fopen`
to have failed — and on that input the mutant calls `fclose(nullptr)`. It is
category (b), an input away, not (a). The first is declared, with 24 others.

The lesson is not "declare more". It is that **an undeclared equivalence and a
refuted one look identical in a score**, and the only thing that separates them
is writing the reason down where it can be read.

### An equivalence belongs to the merge, not to the sweep

`dbgmutate.py` took `--equivalences` and baked `equivalent_declared` into each
part, so revising a claim meant re-running the sweep that produced it. It is now
a `dbgmutate_merge.py` argument, which also makes it **refutable**: a mutant
declared equivalent that the corpus killed is a wrong declaration, and the merge
exits 2 and names it rather than quietly shrinking the denominator. Controlled
on this tree. **Candidate for the method: `_mutation_merge.py` should do the
same, and `vit_mutate.py`'s `--equivalences` should move out of the sweep.**

### PROPOSED AMENDMENT — ask what the unit WRITES, not only what it returns

The gate reads `avrSWAP`; `dbgcheck.py` read `*.RO.dbg`. Neither read `stdout`,
and nine of this unit's 57 survivors sat on a `WRITE(*,100)` line that goes to
unit 6. Adding that stream cost about forty lines in two scripts and found a
REAL DEFECT on its first comparison, before it scored a single mutant: 46 of
scenario 27's 110 stdout records differed, because `libgfortran` emits a
preconnected unit's record whole and a fully-buffered `stdout` does not.

The general form, and why it belongs to the method rather than to this campaign:
**a unit's observable set is a list of STREAMS, and the harness compares the
ones somebody thought of.** Enumerate them before choosing an oracle — return
values, arguments, files, `stdout`, `stderr`, sockets, the exit code. A stream
nobody enumerated is not a gap in coverage, it is a gap in the definition of
"green".

### PROPOSED AMENDMENT — an out-of-bounds mutant is a survivor no value oracle can kill

`c3a5bb71` turns `I < nDebugOuts` into `I <= nDebugOuts` and writes one element
past a 26-element `std::vector`. `Read_OL_Input` recorded the same shape reading
one byte past a buffer. Neither can be killed by comparing written bytes, at any
corpus size: the difference is not in the program's values, it is in whether the
program has defined behaviour at all.

The instrument is a sanitiser build — `-fsanitize=address,undefined` over the
same scenarios, with a mutant killed when the sanitiser reports. It is one
CMake option and it would convert every out-of-bounds mutant in the campaign
from a survivor into a kill. It is **method-level** because it changes what
"killed" means for all 31 translations at once, which is why it is proposed here
rather than done inside one unit's dispatch.

### A whole-simulation oracle cannot place a value on a constant

Six survivors need a computed double to equal exactly `1E-99` or `1E+99`, or to
be more negative than `-1E+99`. The corpus's inputs are wind speeds, mode flags
and a time step; what the mutants compare are values produced from them through
an estimator, filters and controllers. **No admissible input selects a debug
channel's value**, so this is not "the range is too narrow" — widening the
corpus cannot reach it in principle. Two of the six are further out of reach
still: `LocalVarOutData` is fed from `avrSWAP`, a 4-byte float, whose smallest
non-zero magnitude is about `1.4E-45`.

The answer is a direct driver over a synthetic `DebugVariables`, comparing the
Fortran reference's bytes against the translation's for a constructed state. It
is named and costed and NOT built here: it needs all 159 `LocalVariables` fields
set explicitly on both sides.

### `negate_cond` lost the condition, and a no-compile hid it (X2, `3f8ed43`)

`if\s*\(([^;{}]+)\)` is greedy and a brace-less `if` whose statement contains a
call runs past the condition's own closing paren. The artifact reports one
no-compile; what it is, is a condition with NO mutant. Nine conditions across
four of the campaign's 31 translations were in that state. Two of the three in
this unit died immediately, one on 99,214 records. Fixed in the repo that owns
it, cost measured over all 31 translations: 190 of 199 `negate_cond` mutants
unchanged.

### A scenario that earns nothing can be worth more than one that earns kills

Scenario 29 was added to kill `LoggingLevel > 0` → `>= 0` and killed nothing,
because `DISCON.F90:145` guards the call site with the same predicate — the
unit's own guard is dominated by its caller's. Scenario 34 was added to reach
`avrIndices`' deallocate arm and could not, because a second Init in one library
load is refused before `Debug` is called and a second Init after `kill_discon`
reloads the library. Both are now equivalence declarations resting on a
measurement instead of survivors resting on an argument. **Ask what the CALLER
guards before adding an input for a guard inside the callee.**

## Unit #36 `PitchSaturation` — 2026-08-14

### The differential harness failed 1292 of 1292, and the cause was the generator

`expand_derived` prefers a REAL Fortran field as an allocatable array's extent
(`_EXTENT_FIELD_PATTERNS`: `PS_BldPitchMin_N` for `PS_BldPitchMin`). VIT's view
struct carries `n_PS_BldPitchMin` for every allocatable field regardless, and
`emit.py`'s bridge-call emitter passes `&<inst>.n_PS_BldPitchMin`
unconditionally. So for exactly those fields the two halves of the harness
disagreed about which member holds the extent, and **nothing assigned the one
the bridge passes**: the reference allocated zero elements, the translation read
an extent of 0, and the C++ side interpolated over a table it believed was
empty.

Fixed in `translation-loop e54bef7`, one guarded line, which cannot fire where
the two names already coincide — i.e. for almost every array in the campaign.
The failing artifact is kept at
`evidence/PitchSaturation/harness_generator_defect/`.

**The exposure was measured, not assumed.** Five fields in this campaign's types
have a companion count: `WE_CP`, `WE_FOPoles`, `PS_BldPitchMin`,
`SU_LoadStages` (`ControlParameters`) and `ACC_INFILE` (`LocalVariables`). Two
committed translations read one of their extents, and **`CheckInputs` is one**:
its `n_SU_LoadStages` was 0 in every case, so `any_lt(SU_LoadStages, ...)` was
unreachable in both its harness green and its mutation score. That is a closed
unit whose evidence is weaker than it reads. **Not fixed here** — a re-take of
`CheckInputs` is a dispatch, not a paragraph — and it is put to the Driver.

### A pre-integration reference gates a deferred CHARACTER twice; the shipped build gates it once

The last 16 of the 1292 are the 16 consecutive capacities in `[14, 29]`, and
`len("PitchSaturation:")` is 16. That is not a coincidence to be explained away:
`interp1d`'s prefix fits in a 14-byte buffer and this unit's does not, so the
translation keeps the first write and refuses the second.

The reference does something else, and the reason is the tree it runs on.
`interp1d` is ALREADY INTEGRATED, so the Fortran `interp1d` the reference calls
re-enters C++ through its own bridge, which populates a FRESH view over the
module staging buffer `vit_defchar_errmsg_errorvariables` — a length that is
VIT's and not the case's. The inner write is therefore ungated, both prefixes
land in the (reallocating, unbounded) Fortran string, and the case's stated
capacity is applied ONCE, at the harness bridge's export, where it refuses the
whole thing and leaves the buffer at its input value.

**This generalises beyond this unit**, and that is why it is here rather than
only in the unit's evidence: *any* unit whose callee is already integrated and
whose callee writes a deferred-length CHARACTER output will double-gate its
reference under `R13_staging_capacity`. It is unit #29's finding — what the
harness measures before integration is not the program it measures after —
reached from a third direction.

`R13` was ABLATED for this unit's scored corpus, with the cost stated: the
refusal boundary of `assign_errmsg` is unreachable here, so the standing
capacity-guard declaration (fifth unit) does not stand beside a rule that could
have killed it. **Two candidate remedies, neither taken inside this unit:**

  * teach the generated comparison to count a case INADMISSIBLE when the
    reference's bridge REFUSED to export an output — it already prints a
    diagnostic when it does. `inadmissible` exists in the artifact and in
    `run.py`, and the GENERATED C++ never increments it. That is the principled
    fix and it changes what every unit's `checked` means (X3).
  * let `harness/ranges.toml` bound R13's ladder per unit. Cheaper, and it
    hard-codes a number that belongs to the unit's own output length.

### `--disable <rule>` renders an ablated rule as `N/A` with a FALSE reason (P6)

Measured on the same run. With `R13_staging_capacity` ablated the artifact's
`rule_coverage` reads

```
N/A  R13_staging_capacity  no deferred-length CHARACTER output -- no parameter
                           whose LENGTH the callee chooses, so no staging buffer
                           whose capacity could be swept
```

which is untrue: `ErrVar%ErrMsg` is exactly that, and the un-ablated run of the
same unit reports `applied R13_staging_capacity 256 case(s)`. So an ablation is
not merely invisible in the machine-readable file — it is recorded as an
inapplicability, which is the one thing it is not. **An artifact must not be
able to say a rule found nothing to do when it was told not to look.** Nothing
in this campaign's evidence should cite `rule_coverage` as proof that a rule was
applicable; this unit's narrowing is recorded in its own evidence and in
`STATUS.md` instead. Proposed as a loop-level fix: a third state, `off`, with
the flag that produced it.

### VIT counts a build failure as a demonstrated kernel red test, and returns

`vit/redtest.py:candidates` rewrites `return <expr>;` with `re.subn` over the
WHOLE FILE rather than over the translated function's body. This unit's
`errmsg_trim` helper returns a `std::string`, so the perturbed file contains
`return (std::string(...)) * 1.00001;` and does not compile — and
`cli.py:_run_red_test` takes the branch commented *"a perturbation that will not
build is a red of the crudest kind, and an honest one"* and RETURNS, so the
value-level perturbation is never attempted. `vit.yaml` then records
`red_test: demonstrated` for a kernel no perturbed build ever ran against.

It is unit #10's finding with the sign flipped — a stub that FAILED because
`make` never built it — and it lands in the one field a later reader would trust
for exactly this question. The same shape fires on any translation with a
non-arithmetic `return` in a helper; in this tree that is at least
`interp1d.cpp` and `sigma.cpp`, which carry the identical helper. **A
`red_test: demonstrated` in `vit.yaml` is therefore not evidence, on its own,
that any kernel can discriminate.**

Not fixed inside this unit: X2 says do not work around a tool bug we control,
X3 says do not change a verification default mid-run, and here they point in
opposite directions — the fix would change what `red_test` means for the 35
units already carrying one. Put to the Driver. What was done instead is the
measurement VIT did not make: seven stubs through the committed kernel.

### `*build*` in an inherited `.gitignore` silently drops evidence (K3)

`.gitignore:65` is upstream ROSCO's and matches by FILENAME anywhere in the
tree. Five evidence artifacts across four units were untracked by it, and every
one of them is the artifact that records a FAILURE — including
`evidence/PIIController/postintegration_transient_build_failure.txt`, whose
story `STATUS.md` tells in prose while the file existed on one machine only.
`git add -A` reports nothing; the commit looks complete. Closed by addition
(P5): three negations under `evidence/`. The same shape was already noted three
lines above in that file for `fixtures/bladed_stub/build.sh`, so this is the
second time the pattern has caught something somebody needed — **an inherited
ignore rule is a hazard to every artifact a campaign invents a name for.**

### The two ErrMsg helpers are copied for the fourth time, and a shared header is now the cheaper thing

`assign_errmsg` and `errmsg_trim` are byte-identical, modulo the unit name in
two diagnostic strings, across `interp1d.cpp`, `sigma.cpp` and now
`pitchsaturation.cpp`; `extcontroller.cpp`, `chkparsedata.cpp`,
`updatezeromq.cpp` and `checkinputs.cpp` carry the same shape. They were copied
here rather than shared, under P4 and with the copy stated in the file header,
because a shared header changes how every translation in this campaign is built
and compiled — X3's question, not a unit's.

**What the duplication costs is now measurable rather than aesthetic.** Five of
this unit's six declared equivalences are at sites in those two helpers, and
four of the five repeat a declaration an earlier unit already made at the same
site for the same reason. Every future unit that takes an `ErrVar` will pay the
same five declarations. A shared, once-mutated, once-declared helper would make
that one declaration instead of N. Proposed to the Driver as a phase-bracketed
change (K4), not as a unit's edit.

### A commutative intrinsic is a site `swap_call_args` can never kill

`max(LocalVar%PS_Min_Pitch, LocalVar%PRC_Min_Pitch)` was written `std::max(a, b)`
rather than as a hand-rolled `?:`, deliberately: a conditional would offer a
`compare_op` mutant whose kill would prove only that the corpus contains a pair
on the boundary, and would read as arithmetic the reference does not have. The
price is that the operand ORDER gets no killable mutant — measured, not assumed:
the exchange fails 0 of 1036 on a corpus that includes negative zero, where the
two operands have different bits. This is a property of MAX and no input can
change it, which makes it the one declaration in this unit that is a true
equivalence rather than a blind spot.

## Unit #38 — PreFilterMeasuredSignals

### An index can be an ARRAY, and all three sites that materialised one wrote a scalar

`CntrPar%F_GenSpdNotch_Ind` is an ALLOCATABLE INTEGER array every one of whose
ELEMENTS subscripts `F_NotchFreqs`, so `harness/signature.py` gives it
`role="index"` **and** `dims`. `generate.py::_case_impl` tests `p.role ==
"index"` BEFORE the `p.dims` branch and wrote `c[name] = 1` — one value where
the case stream expects `n` — and the failure surfaced four rules downstream as
`TypeError: 'int' object is not iterable`. `_random_over` and R5's
1/interior/`n` index sweep had the same shape. Fixed at all three sites
(`translation-loop 3ac5b4a`).

The `p.dims` branch one line below already carries the comment *"Branch on shape
before kind: an INTEGER allocatable array (`Ind_CableControl`) has kind 'int'
AND dims, and filling it as a scalar wrote one value where the stream expects
`n`"* — the identical defect, found once and fixed at one of the sites that
share the shape. **A fix applied at one of two sites that share a code shape is
a fix the other site escapes**, and here there were three.

The same commit makes `emit.py::write_cases` NAME the parameter instead of
raising a bare `TypeError`. That is this campaign's own measured rule (unit #29:
an error naming its parameter costs one diagnostic pass, one that does not costs
three) applied to the tool rather than quoted at it. It cost one.

### An unrestricted `USE M` puts M's names in scope, not every name

VIT's `_nested_type_use_statements` returned `[]` the moment any copied USE
statement lacked an `ONLY` list, on the stated reasoning that *"an unrestricted
USE is present; every name is in scope"*. That is false for the ordinary shape.
`PreFilterMeasuredSignals` sits in a module that carries `USE Constants` and
`USE Functions` — both unrestricted, neither re-exporting a derived type —
beside a procedure-level `USE ROSCO_Types, ONLY : ControlParameters,
LocalVariables, DebugVariables, ObjectInstances, ErrorVariables`. The generated
bridge then declares POINTERs to the five NESTED types that ONLY list does not
name, and gfortran refuses: `Derived type 'filterparameters' at (1) is being
used before it is defined`. Fixed in `vit fe22383`, and the property the
original reasoning protected — no bridge that compiled before gains a line — is
kept explicitly: an unrestricted USE **of the types module itself** still
suppresses the addition.

### `objInst%instLPF` needed the same judgement a second time, five times wider

`harness/ranges.toml` carries unit #34's pin `objInst_instLPF = { lo = 1, hi =
1024 }` with the reference's own SIGSEGV behind it. This unit calls five
different filters and passes five different counters, every one of them a free
scalar integer on R6's ladder and every one of them a subscript into an
`FP` array of `DIMENSION(1024)` **inside the callee** — where no compared
out-parameter exists for an inference to attach a role to. It also loops
`DO K = 1,LocalVar%NumBl` over `rootMOOP`, which is `DIMENSION(3)`.

Six pins, and the bound is arithmetic rather than a round number: this unit
advances `instNotch` up to eighteen times in ONE call, so 1000 rather than 1024.
The relation the pins stand in for is one the generator cannot see — it lives in
`ReadSetParameters.f90`, not in this procedure — and that is the same shape as
`F_GenSpdNotch_N == SIZE(F_GenSpdNotch_Ind)`, pinned here for the same reason.

**A standing candidate for the Driver:** a counter that is passed INOUT to a
callee which subscripts a fixed-size array with it is now the second and third
instance of one class. Neither `ranges.toml` nor any rule can state the relation
"this integer indexes an array the signature does not mention"; a `role="index"`
that the SIGNATURE could carry across a call boundary would.

### The corpus rule and its own arrays: R11's baseline reproduced unit #29's defect

The first `harness/baseline.PreFilterMeasuredSignals.json` wrote
`{"ramp": [1, 0]}` for both notch index arrays and `1` for both notch counts.
With every element equal to 1, `ind - 1` and `1 - ind` are the same number and a
shifted loop index reads an equal element — so ten of the first sweep's
twenty-two survivors were index shifts that no number of such cases could kill,
and the score was 0.8254. R11's own implementation comment records exactly this
from unit #29 (49 of 89 survivors) and the file reproduced it anyway.
`{"ramp": [1, 1]}` with the counts at 2 is what distinguishes them; the corpus
stayed at 9033 cases and the failing artifact is kept.

**The rule that generalises:** a baseline state is a corpus rule's own data, and
the constraint R5 puts on the generator — *array elements distinct* — applies to
it. Nothing checks that.

### `_mutation_merge.py` stated one identity twice and the two disagreed (C12)

`if killed + survived != behavioural: die(...)` refused every split sweep with a
declared equivalence in it, while `denom = behavioural - eq` five lines below
already read `mutants` as counting them. rosco-r2 unit #38 is the campaign's
first SPLIT sweep declaring any equivalence — #36 declares six and ran in one
part — so the gap could not have surfaced earlier. Fixed with the failing
message recorded at the site.

### The gate's failed red test on the `Flp_Mode == 2` arm is corroborated, not inferred

Disabling that arm moves **0 of 5,252,000** compared values, revert-verified.
The arm is reached by scenario 4 alone (12,000 hits) and scenario 4's own
docstring in `Examples/vit_sim.py` says *"blade root moments are near-zero"*.
Unit #35 measured the same fact from the other end: its `PIIController` kernel
found `-LocalVar%rootMOOPF(K)` identically zero in all 20 captured cases. Two
units, two instruments, one conclusion — the arm is executed and unobserved.

Widening scenario 4 to drive blade root moments would change what one of the
gate's 27 scenarios computes, which is what X3 forbids mid-run. It is the
highest-value thing the campaign could do for this arm and it is a Driver
decision.

## 2026-08-15 10:53 — interp2d: the prediction was right about one bridge and wrong about the other

`plan.json` carried `bridge_feasible: no` for this unit, on the conformance
matrix's `c_assumed_shape_2d` cell:

    compiles = "no"
    why = "UNSUPPORTED: a rank-2 assumed-shape dummy is passed as m(1:n_m), a
           rank-1 section. The C side already emits n_<x>_rows and n_<x>_cols;
           the bridge must declare the local with both extents."

**CONFIRMED for the differential harness's bridge.** `vit test-validate` wrote

    REAL(C_DOUBLE), INTENT(IN) :: zData(*)
    INTEGER(C_INT), VALUE, INTENT(IN) :: n_zData
    vit_result = interp2d(xData(1:n_xData), yData(1:n_yData), zData(1:n_zData), ...)

and gfortran said `Rank mismatch in argument 'zdata' at (1) (rank-2 and
rank-1)`. Two defects rather than one, and the matrix names only the first: the
PARAMETER LIST disagreed as well, because `interface_gen.build_c_params` has
always emitted `n_zData_rows` and `n_zData_cols` for this argument — the C++
scaffold VIT generated for the same unit minutes earlier has both. Had the ranks
agreed, the argument-list skew would have been a silent misread of the call
frame. `evidence/interp2d/bridge.rank2-before.txt`.

**REFUTED for the wrapper that ships.** `vit integrate --apply --reverse-copy`
emits a `BIND(C)` interface declaring `zData(*)` plus the two extents, and the
wrapper passes its own `zData(:,:)` to it by sequence association. It compiles,
links and installs; `libdiscon.so` is built from it and the gate runs against
it. The matrix cell is about `test_validate.generate_fortran_bridge` and reads
as though it were about the feature.

So the plan's `basis` was a true statement about one generator, recorded against
the unit as though it governed both. **A conformance cell should say WHICH
generator it measured.** Raised here as a proposed method amendment: it is the
same shape as the `c_alloc_inout` cell's own history, which that file already
records — "FIRST it read `compiles = no` from
`test_validate.generate_fortran_bridge` while the INTEGRATION wrapper compiled
clean and silently dropped the ALLOCATABLE attribute". That is the second time
one cell has stood for two generators that disagree.

The fix is in VIT (X2), not around it: `test_validate.py` grew a rank-2 branch
emitting the same `RESHAPE(...)` `interface_gen._build_bridge_call_args` has
emitted for the kernel bridge all along — copied, not re-written, because two
generators disagreeing about one calling convention is the defect being closed.
`INTENT(OUT)`/`INTENT(INOUT)` is refused with its reason: a `RESHAPE` is an
expression and therefore a copy. Positive control: `interp1d`'s bridge
regenerated on the same tree with and without the change is byte-identical.

`tests/conformance/matrix.toml` still reads `compiles = "no"` for that cell. It
is regenerated by `tests/test_conformance.py --update`, which was NOT run here —
running it would rewrite every cell from one dispatch's tool state, and that is a
bigger claim than this unit measured. Named so the next dispatch can take it.

## 2026-08-15 10:53 — interp2d: a callee bridge assumes a wrapper ran, and the differential harness has none

The campaign's first differential harness whose callee takes a VIEW TYPE.
`interp2d` calls `interp1d`, which takes `TYPE(ErrorVariables), INTENT(INOUT)`;
on a clean tree `harness.sh` keeps the generated `interp1d_c` bridge so both
sides share one callee. That bridge passes `vit_original_errorvariables`, and
the view module says what that is:

    ! Kernel stash: pointer to original Fortran type, set by wrapper before
    ! calling C++.
    TYPE(ErrorVariables), POINTER, SAVE :: vit_original_errorvariables => NULL()

**The differential harness calls the translation directly. There is no
wrapper.** SIGSEGV on case 0, no stdout at all, because the harness prints its
JSON after the loop. Three instrumented rebuilds narrowed it and the bridge was
then asked directly: `BRIDGE stash associated = F`.

Repairing only the pointer is not the fix, and the second half is the more
interesting one: both existing converters move a deferred-length CHARACTER field
through the MODULE STAGING BUFFER, which only a wrapper's own `vit_populate_<t>`
fills. So the copy IN silently left `ErrMsg` alone and the populate on the way
OUT REPOINTED `view%ErrMsg` away from the buffer the caller owns. **123 of 1117
cases disagreed**, every one on `ErrVar.ErrMsg` or `ErrVar.n_ErrMsg`.

Fixed additively in VIT: `view_populator` now also emits `vit_direct_<t>` and
the pair `vit_view_in_<t>` / `vit_view_out_<t>`, which convert against the
caller's own buffer and leave its pointer and capacity alone; a field kind
neither handles is an `ERROR STOP` naming it. `interface_gen` takes that path
only under `.NOT. ASSOCIATED(stash)` and `NULLIFY`s after, so nothing executes
differently when a wrapper set it. `evidence/interp2d/callee_bridge.null_stash.txt`.

**FOR THE METHOD.** Every earlier unit's callee bridge carried a CHARACTER or a
plain array, so this convention had never been exercised where the campaign's
own instrument is the caller. A generated bridge that reads `set by wrapper
before calling C++` is a bridge with a precondition, and nothing checked it. The
general shape — *a generated artifact whose comment states a precondition no
code tests* — is worth a check of its own, and it is not specific to rosco-r2.

## 2026-08-15 10:53 — interp2d: `ordered_only`, and a cost statement the sweep refuted

`interp2d` CHECKS that its two breakpoint vectors are strictly increasing,
REPORTS that they are not, and interpolates anyway. On a descending table every
interior query leaves the corner search at `j = 0` and `zData(i,j)` reads one
column before the array — so the reference and the translation each read what
precedes THEIR OWN buffer. Six cases of 1117 reached it and five disagreed
(`evidence/interp2d/harness.out-of-bounds-corner-census.txt`). Unit #41's rule:
a red result that is really an undefined reference cannot be told from a
translation defect, and `vit_mutate.py` refuses to score against a red baseline.

`harness/ranges.toml` gained a fourth judgement kind, `ordered_only`, which
takes an array out of BOTH rules that leave the ascending domain — R6's ordering
sweep and R10's reversed body. Excluding the ladder alone moved 5 failures to 4;
R10's reverse was the rest. 1117 → 1005 cases, 0 failed.

**AND THE COST WAS STATED WRONG, AND THE SWEEP SAID SO.** The entry claimed the
ordinary random draws would still reach the two checks "one time in six".
`_fill_array` draws strictly increasing bodies by construction, so once the two
exclusions are in, no case in the corpus has a non-increasing table at all. The
refutation came from the mutation artifact rather than from re-reading:
`ErrVar->aviFAIL = -1` inside the lambda those checks alone call, mutated to
`-2`, SURVIVED — and `aviFAIL` is compared on every one of the 1005 cases. 14 of
the 32 survivors are in that dead code. The wrong sentence is kept in
`ranges.toml` beside its correction (C12).

This is the third instance of one shape: **a stated cost is a hypothesis, and
the mutation score is the instrument that tests it.** Units #43 and #44 each
found the same thing from the other direction — a survivor list explained by
what the corpus could not reach. Proposed as a method note: a `ranges.toml`
exclusion should be re-read against the survivor list before the unit closes.

## 2026-08-15 11:35 — interp2d: the conformance matrix was RUN, and NOT committed

The `c_assumed_shape_2d` cell reads `compiles = "no"` while the tool now
compiles it, so the obvious close was `python3 tests/test_conformance.py
--update`. It was run four times and the file was reverted each time. What it
showed, in order, is worth more than the file would have been.

**Run 1 moved four cells, and three of them were a regression I had just
introduced.**

```
c_assumed_shape_2d   compiles     no -> yes    <- intended
c_alloc_field        integrates  yes -> no     Cannot open module file vit_alloctype_view.mod
c_char_field         integrates  yes -> no     Cannot open module file vit_charfieldtype_view.mod
c_nested_field       integrates  yes -> no     Cannot open module file vit_nestedtype_view.mod
```

**The isolation is the whole point, and it took one command.** Checking out
`vit/view_populator.py` and `vit/interface_gen.py` from the PARENT of the
direct-caller commit and re-running `--update` left exactly one diff — the
intended one. So the three flips were mine, and the missing `.mod` was three
cells away from the line that caused it:

    ERROR STOP 'VIT: vit_view_out_alloctype: v, m is an ALLOCATABLE, POINTER ...
    Error: Line truncated at (1) [-Werror=line-truncation]
    Error: Unterminated character constant beginning at (1)

Free-form Fortran truncates a source line at 132 characters. My `ERROR STOP`
carried the FIELD LIST, so it did not compile for any type with more than two or
three unhandled fields. Unit #5's 132-column bridge-generator finding, one
generator over, and **it would not have been found by this campaign at all**:
ROSCO's `ErrorVariables` has no unhandled field, so the branch never fires here.
VIT's own conformance fixture is what caught it. Fixed (`a16a7ab`): the field
names go in wrapped comments, the literal names only the subroutine.

**Run 4, after the fix, still leaves the file uncommittable.** All six columns of
the three cells are back to `yes`, and the writer nonetheless emits

    why = "TODO: integrate: Fatal Error: Cannot open module file ..."

on cells whose every column passed — a `why` carried from an earlier stage's
`detail` and attached to a measurement that succeeded. And the cell this
dispatch actually corrected keeps its old `why`, which now describes a defect
that is gone while `compiles` reads `yes`.

**So the matrix is NOT committed, and that is the choice rather than an
omission.** Committing it would put three unexplained `TODO`s into the file
`plan.json` derives every unit's `bridge_feasible` from, and this dispatch
measured neither the writer's `why` rule nor those three cells' semantics. What
the campaign gets instead is this entry and a named next step:

  1. `tests/test_conformance.py`'s writer attaches a stage's `detail` to a cell
     whose measurement passed. Fix that first; until then `--update` cannot be
     run to completion by anyone.
  2. Then re-run it and correct `c_assumed_shape_2d`'s `why`, which is currently
     a true statement about a defect that no longer exists.

**AND THE METHOD POINT, which is X4 with a specific shape.** The direct-caller
change shipped with a positive control — ROSCO's `vit_errorvariables_view.f90`
byte-identical up to the additions — and that control was *necessary and not
sufficient* in exactly the way P10 describes. It exercised the one type this
campaign has, and the defect lived in the branch that type does not reach. **A
generator's positive control must cover the branch the change ADDED, not the
input the campaign happens to own.**

---

## Unit #45 `interp2d`, second dispatch — 2026-08-15

### A mistranslation that four instruments passed, found by reading the reference

`interp2d` guards **both** corner searches against a NaN query. The translation
guarded one.

```
Functions.f90:231   IF (xq <= MINVAL(xData) .OR. (ieee_is_nan(xq))) THEN
Functions.f90:257   IF (yq <= MINVAL(yData) .OR. (ieee_is_nan(yq))) THEN
interp2d.cpp:230        if (xq <= xData_min || std::isnan(xq)) {
interp2d.cpp:253        if (yq <= yData_min) {                        <- the defect
```

`grep -c ieee_is_nan` on the reference is 2; `grep -c isnan` on the translation
was 1. On `yq` NaN with `xq` interior the reference returns
`interp1d(xData, zData(1,:), xq)` and the translation ran the y-loop to
completion, leaving `i = n_yData` and **`ii` read without having been written**.

**The four instruments and why each was blind.** The differential harness draws
no NaN query. The mutation sweep grades the corpus against *this translation* —
a missing statement is not a mutant of it. The post-integration harness runs the
same corpus with both sides in C++. The gate's 27 scenarios interpolate Cp/Ct/Cq
at `BldPitch*R2D` and `Lambda`, and a NaN there would already be an upstream
fault.

**The part that is a method finding rather than a ROSCO one.** The first
dispatch wrote the omission down as a property of the reference, in three
places — the translation header, `plan.json`'s `observability`, and STATUS.md —
and then *correctly derived* the translation's behaviour from it and closed the
question as oracle-less. By the time the sweep ran, three artifacts asserted the
same wrong thing and **not one of them was a reading of `Functions.f90`**. P7
says the oracle is the original source; nothing in the loop asks whether a
paragraph claiming to describe the reference was ever checked against it.

**Proposed method amendment, beside P7 and X4.** *A statement about the
reference, written in an artifact, is a claim that needs the same standing as a
green: it must name the file and the lines it was read from.* A cheap
enforcement exists for the specific shape that bit here — a check pairing every
guard in the reference's translated region with a guard in the translation, by
count of the intrinsics VIT already knows how to map (`ieee_is_nan`,
`ALLOCATED`, `PRESENT`). Recorded in `PUSHED_TO_MUTATION`/`TOOL_GAPS` terms:
this one HAS a precise detector, so it belongs in `vit/checks.py`, not in prose.

### `ordered_only` was one shape too wide, and the sweep is what said so

The first dispatch excluded every non-ascending body for both breakpoint arrays,
citing the reference's out-of-bounds `zData(i, 0)` on a reversed table. That
exclusion also removed the **adjacent equal pair** and the **constant array**,
which cannot produce that subscript at all: the corner search runs only in the
arm where `xq > MINVAL(xData)`, and on a non-decreasing body `MINVAL(xData)`
**is** `xData(1)`, so the first iteration always falls through.

The cost was measured, not argued: **13 of the unit's 32 surviving mutants** sat
in code no case executed, and `ErrVar->aviFAIL = -1 -> -2` inside the `fail`
lambda survived a 1005-case green in which `aviFAIL` is compared on every case.

`nondecreasing_only` (translation-loop `c7869e8`) is `ordered_only` with the
line drawn where the reference draws it. Corpus 1005 → 1147, 0 failed, and it
killed **17** of the 32 — the 13 above plus the four reduction-boundary
comparisons that only a **constant** table (for `xq <= MINVAL` against `<`) and a
**repeated maximum** (for `xq >= MAXVAL` against `>`) can separate.

**The general shape, offered as an amendment.** *An admissibility exclusion
should be stated at the predicate the reference actually fails, not at the
nearest coarser property.* The first spelling of this one cost 13 mutants and a
paragraph of self-consistent reasoning about why they were unreachable. The
second is one function, `_nondecreasing`, and it is checkable.

### Two survivors that no per-parameter judgement can reach — escalated

`5c2746a0` and `a155c90c` (`for (j = 1; ...)` → `j = 2`, and its y twin) are
killed by `xData = [3,1,2,4]`, `xq = 3`: the reference returns column 1 on the
search's first iteration, the mutant starts at 2 and interpolates between
columns 3 and 4. Both are fully defined.

No rule draws it because **admissibility here is joint**: an inverted body is
safe exactly when `xq >= xData(1)`, and every judgement `harness/ranges.toml`
can state is a property of one parameter. The rule that would reach it is R10's
shape with the scalar pinned to `body[0]` rather than to `MINVAL(body)`, over
*all* order shapes — pinning `xq` to `xData(1)` is safe on **every** ordering,
which is what makes it a rule rather than a special case. **Not taken here:** it
is a new generator block and this dispatch's clock went on the seventeen above.

### Two more at a conjunction no rule crosses — the second unit to reach it

`c2c4e0b7` and `922581aa` need `errmsg_trim` reached with `n_ErrMsg <= 1`. Each
half is in the corpus and the two never meet: the character ladder draws lengths
`[1, 2, 7, 12]` with every other input at its base draw, and `aviFAIL` is a
predicate knob whose four values are crossed with every other input at *its*
base draw. Measured over all 1147 cases
(`evidence/interp2d/errmsg_extremes_probe.cpp`): 27 trims, `n` 7..33, none at or
below 1.

This is `generate.py`'s own `ZMQ_Mode` finding — *a sample of a corpus is not a
sample of its conjunctions* — in a place where no rule makes the conjunction at
all. `interp1d` recorded the same gap at the same helper (`55e42d36`) and
declined the same fix for the same reason. **This dispatch does not declare
them.** Two units is a pattern; a rule crossing the character ladder with the
predicate knobs is the fix, and it is a generator block, not a `ranges.toml`
entry.

### A generator defect the change exposed, and a harness defect the red test did

`generate.py` bound `lengths` in two blocks of one function scope — the
CHARACTER block and the R6 order ladder — and assembled the coverage report
after both, so turning the order ladder on made the character sentence report
the ordering sweep's array extents. No case moved; the artifact was wrong and
only the artifact. Every unit until now had at most one of the two blocks live.
Recorded before the fix (`evidence/interp2d/generator.shadowed-lengths.txt`),
fixed at `eb5028e`, positive control: the corpus regenerated across the rename
is 1147 cases with every other field identical.

Separately, the post-integration re-take after the wrapper red test died on a
transient build failure, and because `harness.sh` removes its `--out` path
*before* the run and the `&& git commit` beside it was guarded by a pipeline
ending in `tail`, **the deletion of a passing artifact was committed under a
message claiming the green** (`e5be0140`). Kept in the history and recorded at
`evidence/interp2d/postintegration.retake-failed-once.txt` rather than amended
away. **Two campaign-wide consequences:** a `--out` path is destroyed by a run
that fails, so a failed measurement destroys the previous one; and
`cmd | tail && git commit` does not guard the commit, because a pipeline's exit
status is its last element's.

### The unit's `note` answered: the R11 replacement question does not arise here

`plan.json`'s note asked whether the `2e2295f` R11 change's cost on
`CheckInputs` — 8 net kills lost, 11 of 17 regressions concentrated in the
`any_lt`/`any_gt`/`all_lt`/`maxval`/`minval` helpers — generalises to this unit.
**It does not arise:** `interp2d`'s reference uses `MINVAL`/`MAXVAL` and two
`DO` loops, and the translation writes the reductions and both scans as explicit
loops. It calls none of those helpers, so there is no site for that regression
to land on. The corpus adequacy question the note really asks was answered the
other way, by measurement: this corpus was **not** adequate, 32 survivors said
so, and 17 of them died to one judgement.

### The generator control found the shadowing had been shipping for two units

`nondecreasing_only` and the `lengths` rename were claimed additive. The control
holds everything but the change constant — `interp1d`'s corpus generated on one
clean tree at `a523b00` and at `eb5028e` — and it is **795 cases, 0 failed, at
both**, every field identical but two sentences of `rule_coverage`.

One of those two sentences is the fix working on a unit that is not `interp2d`:
`[1, 2, 3]` → `[1, 2, 5, 10]`. And that is the finding, because
`evidence/interp2d/generator.shadowed-lengths.txt` and the generator's own
commit message both asserted that the two blocks had **never both fired before**.
A grep of the committed artifacts refutes it: **eight**, across `interp1d` and
`unwrap`, report the ordering sweep's extents in the character block's sentence.
No case moved in either — the same argument the control proves for `interp1d` —
so no green and no score is affected, and they are **not** re-taken here:
re-taking them would move them to a `loop_rev` their sibling artifacts are not
at, which is the `BASE-SHA SPLIT` `revcheck` exists to catch. Owed: a sweep of
the campaign's coverage reports for that one sentence, at the next occasion when
those units are re-measured for another reason.

**The mechanism is this dispatch's recurring one and is worth naming twice.**
The wrong paragraph was written from *reading the two code blocks* rather than
from *grepping the artifacts*, exactly as the `yq`-NaN paragraph was written
from reading the translation rather than the reference. Both were self-consistent
and both were checkable in one command. **A claim about what the corpus, the
reference or the artifacts contain is a measurement, and it needs a command
behind it, not a reading.**

A second, smaller inconsistency is stated rather than fixed: `eb5028e` appends a
`nondecreasing_only` clause to R10's coverage sentence **ungated**, so it renders
for every unit. It is phrased conditionally and is therefore true, but unlike the
R6 addition beside it, it is not gated on the judgement being used. Gating it
moves `loop_rev`, and this unit's twelve artifacts are all at `eb5028e` — the fix
would have meant re-taking all twelve, including a 282-second gate red test,
inside the dispatch's last half hour.

## 2026-08-15 13:00 — ratelimit: an initialisation arm can be *arithmetically* invisible, and no window reaches it (a proposed method amendment)

`ratelimit`'s kernel is blind to its reset arm, and the interesting part is that
this is **not** a windowing failure of the kind units #41, #42, #43 and #44 all
recorded and repaired.

Those four are the same rule seen four times: aim the window at the transition,
because the steady state is where a no-op reproduces the reference. Each was
repaired by arithmetic over the scenario's own patch dict, cross-checked against
a coverage count. This unit's window *was* aimed that way — the first range
starts at invocation 1 precisely because coverage places the reset arm there
(clean `Functions.f90:75` records 8 hits in every one of the 27 scenarios, which
is 1 + 3 + 3 + 1, the eight `ratelimit` calls of the first DISCON call). The arm
is **in** the corpus, three cases of 62, and it is still invisible:

```
the reset arm made unreachable    62 of 62 PASS
ResetValue ignored               62 of 62 PASS
```

The census says why, and it is arithmetic rather than instrumentation:

```
invocation 1  inst=6 reset=1 in=+0 last=+0 rv=+0 out=+0
invocation 2  inst=7 reset=1 in=+0 last=+0 rv=+0 out=+0
invocation 3  inst=8 reset=1 in=+0 last=+0 rv=+0 out=+0

THEN:  ResetValue_ = rv = 0        -> returns 0, stores 0
ELSE:  raw = (0 - 0)/DT = 0        -> returns 0 + saturate(0)*DT = 0
```

**The two arms compute the same answer.** Not "the difference is below
tolerance" and not "the case is unlucky" — the reset arm of a stateful unit runs
at exactly the invocation where every input is still at its initial value, and
for a unit whose two arms agree at zero that makes the arm indistinguishable
from its complement *by construction*.

The generalisation, offered as a method amendment because it is not about ROSCO:

> **An initialisation arm is the arm whose inputs are least varied, so it is the
> arm a capture window is least able to discriminate — and hitting it is not the
> same as seeing it.** Aiming a window at an edge answers "is the arm in the
> corpus". It does not answer "can the corpus tell this arm from the other one".
> Only a stub does, and the stub is cheap: two of this unit's eight cost one
> second each and both came back 62 of 62.

It belongs beside P9 (*coverage is not visibility*), of which it is the sharper
form: **presence in a capture window is not visibility either**, and the
instrument that distinguishes them is the same one P9 already implies.

What made it a finding rather than a hole is that the blind spot was then
**closed on numbers by two other layers** — the gate moves 370,796 of 5,252,000
when the same branch is perturbed, and the post-integration harness 604 of 2904
— and the two reach it for different reasons: the gate because 26 other
scenarios and 4 other call sites carry a non-zero `BlPitch` at *t* = 0, the
harness because it draws `has_ResetValue` freely as a flag.

## 2026-08-15 13:00 — ratelimit: `harness.sh --no-generate` skipped the one step it exists for (X2, fixed here)

`--no-generate` is documented as *"REBUILD THE HARNESS FOR THE CURRENT TREE,
KEEP THE CORPUS"*, and it returned three steps early — before step 2e, which is
the step that decides **per tree** whether `<callee>_c` comes from the generated
bridge or from an integrated `<callee>.cpp.o`. Step 1 has just rewritten
`<stem>_callees.f90` with every bridge restored, so the early return left a
freshly generated bridge set against whatever LIBS happened to be on disk.

Measured here, on the clean tree, with `saturate` integrated:

```
ld: saturate.cpp.o: in function `saturate_c': multiple definition of
    `saturate_c'; ratelimit_callees.o: first defined here
vit_mutate.py:  baseline is not green (nocompile); refusing to score
```

2e's `unlinked` branch is exactly the repair — keep the bridge, drop the stale
`saturate.cpp.o` from LIBS — and it never ran. Fixed by moving the return below
2e (`da95a3e7`), which is X2 rather than a workaround: this script is ours.

**It failed loudly only because `vit_mutate.py` refuses a non-green baseline.**
Had the link succeeded with the object instead of the bridge, both sides of the
comparison would have reached the same C++ `saturate` and the sweep would have
reported a score for a run in which the callee was not a control — the exact
shape 2e's own comment calls *"silently wrong rather than loudly wrong"*. The
refusal is the only thing between that comment and a wrong number, which is an
argument for the refusal and not for the fix.

## 2026-08-15 13:00 — ratelimit: two mutation gaps, and only one of them is unit #33's

`mutation/ratelimit.json` reports `score 1.000` on 17 behavioural mutants with
**0 declared equivalent**, and 2 excluded as no-compile. Both gaps were measured
by hand on the same 2904-case corpus, with the unmutated baseline at 0 of 2904
as the control (`evidence/ratelimit/hand_mutants.txt`).

**Gap 1 is new and is a defect in `cppmutate`, not a restriction.**
`swap_operands` swaps the two operands of the **text it matched**, and here the
matched text stops one token short of the expression:

```
const int i = *inst - 1;                     'inst - 1' -> '1 - inst'   =>  *1 - inst
(inputSignal - rlP->LastSignal[i]) / DT      'inputSignal - rlP->LastSignal'
                                             -> 'rlP->LastSignal - inputSignal'
                                                                       =>  inputSignal[i]
```

Neither compiles, so both are excluded from numerator and denominator — and
"excluded" is a statement about the mutant's text, not about the site. In the
shape the operator was reaching for, one dies on **911** cases and the other on
**58**. Not repaired here: changing an operator's matching changes the mutant set
of every unit already scored (X3, unit #22's rule), and the same reasoning that
held for unit #24's added operators applies in reverse. Recorded for the Driver
with the two shapes and their kill counts, which is what a re-take would need.

**The 58 is a number about an allocator, not about a corpus**, and it is worth
separating: `1 - *inst` subscripts `LastSignal` at −5..−7, so both the read and
the write are undefined behaviour and what the harness compares afterwards
depends on what lies before the array. The site is reachable — 58 cases say so —
but the count is not evidence about corpus adequacy in the way the other five
are.

**Gap 2 is unit #33's, unchanged.** All three call operators are gated on tables
of C standard-library callee names, so the one `saturate_c` call this unit makes
has no generated mutant. Hand-measured: the clamp dropped kills **741**, the
bounds transposed **885**, and `saturate_c(minRate, rate, maxRate)` kills **0** —
which is an **equivalence with a proof this campaign already owns** rather than a
survivor, since `saturate` is `fmin(fmax(v, lo), hi)` and unit #24 proved `fmax`
commutative on this toolchain for the only two inputs at which it could fail, a
signed zero and a NaN, both orders of both, compared as bits
(`evidence/saturate/minmax_probe.txt`). It is therefore **not** declared in
`mutation/ratelimit.json`: nothing was excused there, and this one is outside the
denominator because the operator never fired, not because it was argued away.

## 2026-08-15 13:00 — ratelimit: P9's `Status here` is owed and was deliberately not written

The invariant layer asks that a rule's `Status here` move from `inherited,
unexercised` to `exercised at unit #N` the first time it bears on the work, and
this unit is the clearest exercise of **P9 — coverage is not visibility** the
campaign has recorded: coverage placed the reset arm at 8 hits in every one of
27 scenarios, the window captured it three times, and two stubs still passed
62 of 62.

The line was **not** edited, because this dispatch's instruction was not to
modify the invariant layer of `RUNBOOK.md` — and editing it moves
`invariant_hash`, which the Driver reads as a proposed method amendment. So the
debt is recorded here rather than paid silently: **P9 is exercised at unit #46**,
and the same edit would also carry the proposed amendment above (*presence in a
capture window is not visibility either*), which is what makes it an amendment
rather than a bookkeeping change.

## Unit #47 — ActiveWakeControl — 2026-08-17

### No kernel, and it was decided against the arms rather than against the clock

`plan.json` allowed "kernel replay **or** direct-call harness; bit-identical".
This unit's whole body is one `IF/ELSEIF` chain over `CntrPar%AWC_Mode` with five
arms, and coverage puts each arm in a different scenario:

```
mode 1  scenario 11        15,999 calls        mode 4  scenarios 5, 8, 27
mode 2  scenario 15        15,999 calls        mode 5  scenario 22   15,999 calls
mode 3  scenario 21        15,999 calls
```

A KGen kernel is aimed at ONE call site in ONE scenario, so a kernel here could
have seen exactly one arm of five, and the other four would have needed four more
extractions with four `vit.yaml` edits between them. The differential harness
varies `AWC_Mode` itself and reaches all five from one corpus. Second reason,
independent of the first: this unit has **four implicitly-`SAVE` locals**, and
unit #44 measured that `kernel_driver.f90` calls the procedure three times per
case and verifies the third while the in-state carries only the dummies — so a
`SAVE` local is whatever the previous call left. `AWC_TiltYaw`, `Error` and
`StartTime` are all read on a path that does not write them.

### An indeterminate answer stored in a SAVE local outlives the case that produced it

Unit #39 recorded that `ResController` assigns its result only in its `ELSE`
branch, so `IF (reset)` returns whatever the result slot holds, and pinned
`reset` out of *its own* corpus by naming `vit_result` in `no_oracle`. That
repair does not carry here, because in this unit the value is not a return value:
it is stored into `AWC_TiltYaw(Imode)`, which is implicitly `SAVE`.

The first clean-tree sweep: **22 failed of 6436**, every recorded mismatch on
`DebugVar.axisTilt_1P`. All sixteen are `AWC_Mode == 4`; **thirteen** are at
`restart == 1` and **three** at `restart == 0`, each of those three one case
after one that is not. So the indeterminacy is not confined to the case that
produced it, and a corpus rule written per-case cannot see that.

`LocalVar_restart = { values = [0] }`, and the cost is stated where the pin is:
`restart` is a PURE PASS-THROUGH in this unit — it enters no predicate this unit
writes — so what leaves the corpus is the reset arm of `PIController` and of
`ResController`, both CALLED rather than inlined, both scored as units, and
`PIController`'s reset arm is perfectly determinate and is lost only as
collateral. `ranges.toml` states a judgement about ONE parameter and cannot say
"restart may be true unless AWC_Mode is 4".

### The mutation score is 0.7857 and nothing is declared

165 of 210 behavioural killed, 14 no-compile, 45 survivors. **Zero declarations**,
and that is the decision rather than an omission. Eight of the 45 are equivalent
by C++ semantics — they enlarge a declared array extent and no subscript names
the extra element — and the campaign could have declared those on an argument.
It has already recorded, at unit #45, that declaring on an argument rather than a
probe is what it does not want, and the measurement that would settle these eight
is an assembly diff that was not taken. The other 37 are unreachable IN THIS
CORPUS for six separately measured reasons, and declaring those would convert a
corpus weakness into a score. P12 fails and the disposition says so. Same
standing as `interp2d` at 0.9744, `unwrap` at 0.960 and `YawRateControl` at
0.8961.

### Two gaps that belong to the generator, not to this unit

**1. A `COMPLEX(DbKi)` view field is not compared.** `LocalVar%AWC_complexangle`
crosses in both directions as `vit_complex_double[3]` and the bridge carries it
correctly — the gate is bit-identical over 5,252,000 values with mode 1 live in
scenario 11. The generated differential test has **one** `VITCMP` for
`LocalVar.PitCom`, **one** for `DebugVar.axisTilt_1P` and **zero** for
`LocalVar.AWC_complexangle`: the field is absent from R4's out-parameter list
entirely. Five survivors live in the imaginary half, which reaches no other
output. Not repaired here: adding a comparison changes the compared set of every
unit already scored (X3), and this dispatch's clock went on the sweep.

**2. A predicate on a constant-subscript element of an ALLOCATABLE array is not a
knob.** `IF (CntrPar%AWC_harmonic(1) == 0)` guards two writes with **0 hits in
all 27 scenarios and 0 cases of 3231**. `harness/ranges.toml` cannot repair it:
`generate._fill_array` ramps an array monotonically across its bounds, so any
`lo`/`hi` that puts 0 in `AWC_harmonic` puts it at element 1 in EVERY case,
deleting the `ELSE` arms that all 27 scenarios do exercise. The repair is to make
such a predicate a knob — R6 already builds a cross product over the quantities
the reference tests, and it takes scalars. That is a generator change with an X3
cost (it moves the corpus of every later unit whose reference has one), so it is
recorded for the Driver with the number that would justify it: **5 of this unit's
45 survivors, and the campaign's only arm no instrument reaches.**

### A named next step this dispatch did not take

**11 survivors would die from one knob value.** `NumBl` is 0, 1 or 2 in all 91
live cases; the six cases with `NumBl == 3` all have `AWC_Mode` outside 1..5. So
blade 3's angle is computed and never read, and `phi3`, `AWC_angle(3)` and
`PitCom(3)` have no witness. `LocalVar_NumBl` is already pinned `[0, 3]`, so this
is not a pin problem either — R6's knob values are read out of the reference's own
literals and are `{0, 1, 2}`. Adding the pin's upper bound to the knob would be a
one-value change with the same X3 cost as the item above and, on this unit,
0.7857 → about 0.84. Not taken: the corpus is fixed the moment the first part of a
sweep is taken (unit #43's rule), so it would have meant re-running all 224
mutants and re-taking the harness green and its red test.

### A shim object with no symbols, one command after the command that wrote it

The post-integration green re-take after the wrapper red test died at the link
with `undefined reference to activewakecontrol_c`, against a **952-byte**
`vit_integration_shim.o` that `nm` reports as carrying **no symbols at all**,
where the identical `g++ -c` run by hand produces **1528 bytes** and the symbol.
The identical invocation, repeated with nothing changed, then passed at 3231
checked / 0 failed. This is the bind-mount write/read race the runbook already
records for `cp`, met on an object the same `docker exec` chain had just written
three commands earlier. Recorded rather than repaired: the failure is LOUD (a
link error, not a wrong number) and a retry is the whole fix.

---

## Unit #47 `ActiveWakeControl`, second dispatch — a mode selector written as an enumeration, and what it bought

**The dispatch was opened on one unmet condition: P12 at 0.7857 with 45
survivors.** It closes at **0.9200 with 16**, still below the 1.000 threshold and
still failing on purpose. 20 of the 45 are killed, 10 are declared equivalent
with reasons in `mutation/ActiveWakeControl.equivalences.md`, 15 still stand, and
one mutant the old corpus killed now survives.

### The finding worth carrying out of this unit

**`_base` puts a mode selector at the midpoint of its range, and a midpoint is
not a mode.** `CntrPar%AWC_Mode` had no stated range, so it sat at −600 —
`_base`'s midpoint of ±1e3 — in **3032 of 3231 cases**. The reference's body is
one `IF`/`ELSEIF` chain over that selector with **no `ELSE`**, so 94% of the
corpus executed nothing. Every ladder rung, every R6 magnitude, every negative
zero and every random fill was spent on the empty path. **91 of 3231 cases
reached an arm at all, and the unit's own no-op red test moved 86 of 3231 —
2.7%.**

Writing it in `harness/ranges.toml` as `values = [3, 4, 5, 1, 2, 0]` rather than
as `lo`/`hi` is what fixed it, and the difference between those two spellings is
the whole lesson:

| | `lo = 0, hi = 5` | `values = [...]` |
|---|---|---|
| what `_base` gives every non-flag case | ONE mode, `int(5·frac)` | `values[0]`, chosen |
| R2 `flag_values` | does not apply | applies: every value appears |
| R6 ladders | run under ONE mode | **re-run under EVERY mode** |
| corpus size | unchanged | × (sum of flag arities) |

`lo`/`hi` would have deepened exactly one arm, whichever `_base`'s frac happened
to select. `values` deepened all six. The cost is real and is a cost in TIME:
3231 → 21236 cases, and the mutation sweep from ~35 to ~50 minutes.

**Measured outcome:** 17697 of 21236 cases reach an arm (83%), the no-op red test
moves 17645 of 21236, and the post-integration wrapper red test moves 17521.

### The named next step from the first dispatch was reached WITHOUT the generator change

The first dispatch recorded "**11 survivors would die from one knob value**" —
`NumBl == 3` never co-occurring with a live `AWC_Mode` — and costed it as a
generator change with an X3 price paid by every later unit. **It did not need
one.** `NumBl == 3` arrives through the random-fill stratum, which R2 gives eight
cases under *every declared flag value*; with `AWC_Mode` declared, five live cases
now carry `NumBl == 3` in Mode 1 and all eleven of those survivors are dead.

**The general form, and it is the reusable part:** a stratum that already varies a
quantity becomes reachable *in a branch* the moment the branch selector is a
flag. Before reaching for a knob change, ask whether the selector is a declared
enumeration — the knob and the flag are different mechanisms and the flag is the
one `ranges.toml` can pull.

The same statement dissolved three of the five survivors that the first dispatch
attributed to a saturated controller output. It was not the window that was
narrow: `PC_MinPit`/`PC_MaxPit` were at ONE pair, (0, 600), in all 91 live cases,
because they too were at `_base`. They now take **537 distinct pairs, 311 of them
wider than 1e6.**

### The price, paid and reported: one mutant went from killed to alive

`ced1c24e` — `Error`'s first `SAVE` initialiser — was killed by the old corpus
and survives this one. This is the shape already recorded above as *"the kill
counts went down by REPLACEMENT"*, met a second time and with an exact mechanism
rather than a suspicion.

**A `SAVE` local's initialiser is an ORDERING property, not a value property.** It
is observable only where a read of the slot precedes every write of it across the
whole run. The old corpus happened to reach `Error(1)` first at case 3135, a
Mode 3 case whose `Time > StartTime` guard was false. The new corpus's case 0 is
a Mode 3 case at `Time = 600` — `_base` at frac 0.8, positive — so the loop runs
and writes the slot before `:289` reads it. 740 of the 7083 Mode 3/4 cases DO
have `Time <= 0`; every one comes after case 0.

**`values[0]` is the only lever `ranges.toml` has on case order**, and it was used
deliberately here — it is what killed `537e3fe0`, `AWC_TiltYaw`'s first
initialiser. It is not enough, because which side of zero `_base` puts
`LocalVar%Time` on is the midpoint of a range and not an admissibility judgement.
Pinning `Time` to reach it would be fitting the domain to the score.

**ESCALATED, as a generator capability that does not exist: no rule in this
generator orders cases.** Every rule chooses each parameter independently of the
case index. Two survivors here need a first case with specified state, and the
`ExtController` entry in `harness/ranges.toml` records the same wall from the
other side ("*ordering the case set instead — one `iStatus == 0` case first, the
rest not — is not something this generator can express*"). That is now two units.

### Four instrument gaps, each named to a function or a regex

Twelve of the sixteen standing survivors are these. None is repaired here: each
moves the corpus of every unit already scored, which is X3's business and the
Driver's, not a unit dispatch's.

1. **A COMPLEX view field is neither compared nor supplied non-zero — 5
   survivors.** `LocalVar%AWC_complexangle` crosses in both directions and the
   generated test source carries **0 of its 481 `VITCMP` lines** on it; it is
   supplied as (0,0) in all 21236 cases. Two changes: R4's out-parameter
   enumeration must emit a COMPLEX field as two outputs, and the case emitter
   must fill it. Carried over from the first dispatch, now with a fifth survivor
   attributed to it — `97b678b5`, previously mis-attributed to negative zero.

2. **A predicate on a constant-subscript array element is not a knob — 5
   survivors.** `IF (CntrPar%AWC_harmonic(1) == 0)`: **0 cases of 21236, 0 hits
   in 27 scenarios.** Carried over, and this dispatch narrows the fix to one
   line: `predicate_knobs_from`'s `_NAME` has no subscript group, while its two
   siblings in the same file — `relational_pairs_from` and
   `divisibility_pairs_from` — both carry `(?:\(\s*(\d+)\s*\))?` for exactly this
   shape. `predicate_knobs_from` already RETURNS `(param, index, values)` and
   `_knob_overrides` already handles a non-None index. **The machinery exists;
   the pattern does not reach it.**

3. **A nested derived type inside a view struct is zero-filled and never varied —
   2 survivors.** The harness reports this about itself
   (`UNOBSERVABLE LocalVar.resP: nested type, zero-initialised (not varied)`), and
   this is the first measured consequence. With all four `resP` history arrays at
   zero, `ResController` collapses from a Tustin biquad to
   `kp·error + 2·DT·ki·error/(4 + (2π·freq·DT)²)` — four of five terms multiply
   zero. `freq` then enters ONLY through that denominator, which at ±1e3
   magnitudes is of order 1e13 and annihilates the `ki` term. **A stated
   limitation becomes a measurement the moment a survivor lands on it.**

4. **A crossing rule against an EXPRESSION over a parameter — 1 survivor.**
   `LocalVar%Time .GT. 1/CntrPar%AWC_freq(1)`, `>` against `>=`, differs on one
   input and no case is on it. `relational_pairs_from` needs both sides to be
   parameters; `reduction_pairs_from` extended it to `MINVAL`/`MAXVAL` of an
   array for `interp1d`'s endpoint branches — whose survivors were this same
   pair of operators. This is that extension one step further out, to a
   reciprocal.

### An out-of-bounds store is not an equivalent program

`e9d4580d` stores one element past `static double AWC_TiltYaw[2]`. The first
dispatch's census grouped it with `df13f212`, which stores to a *valid* element
the loop below overwrites, and called both "changes nothing". **That is right
about the observable values and wrong about the program.** `df13f212` is declared
equivalent; `e9d4580d` stays an honest survivor, in the category unit #4 recorded
for `GetRoot` — a memory error no value comparison can see, whose instrument is
`-fsanitize=address` on the harness build.

### An assembly diff cannot confirm what a non-empty diff shows

The first dispatch left eight extent-enlarging mutants undeclared and named the
missing measurement: *"a diff of the generated assembly, which was not taken."*
**It was taken here and it answers nothing.** Growing a stack array by eight
bytes moves the frame size and reshuffles the register allocator across the whole
function; all eight diffs are non-empty (6 to 448 lines) and two change the
instruction mix. An empty diff would have confirmed equivalence; a non-empty one
is silent. What the declaration rests on instead is a COMPLETE enumeration of
every subscript on the four arrays against the bounds `harness/ranges.toml`
states — `evidence/ActiveWakeControl/mutation.extent_equivalence.txt`.

**The general form:** "compile both and diff" is a confirmation instrument, not a
decision instrument. Proposing one in a census commits the next dispatch to
running it; say which answer would settle the question before proposing it.

### The link race, met again

The post-integration green re-take after the wrapper red test died at the link
(`make: *** [Makefile:90: test] Error 1`) moments after the revert-and-rebuild,
with no source change between it and the run that passed at 21236/0. Identical to
the shim-object race recorded at the end of the first dispatch. Recorded, not
repaired: the failure is LOUD and a retry is the whole fix.

---

## Unit #48 — AeroDynTorque — 2026-08-17

### The staging-refusal convention is not compositional (a proposed method amendment)

The first differential-harness take was **RED at 14 of 1387**, and every one of
the fourteen is `ErrVar.ErrMsg`/`n_ErrMsg` at a contiguous block of cases inside
R13_staging_capacity's sweep. The window is exact arithmetic:

```
L_callee = len("interp1d:") + len(TRIM(entry msg)) = 9 + 7  = 16
L_final  = len("AeroDynTorque:") + L_callee        = 14 + 16 = 30

cap  <  16      both sides refuse everything    got  7  ref  7   PASS
16 <= cap < 30  the callee bridge writes 16,
                assign_errmsg refuses 30        got 16  ref  7   FAIL
cap >= 30       both write                      got 30  ref 30   PASS
```

`[16, 30)` is fourteen capacities and the failing case set is exactly
1140..1153, measured with the emitted test's own diff cap raised
(`evidence/AeroDynTorque/harness.first_take.all_14_cases.json`), not inferred.

**The mechanism.** The reference chain is Fortran → Fortran → Fortran with a
reallocating `ErrMsg`, and exactly ONE capacity gate: the generated bridge's
write-back, on the final 30 bytes. The translation chain is C++ →
`interp2d_c` bridge → Fortran, and it has TWO: the bridge's write-back on 16,
then `assign_errmsg` on 30. R13's own header states the model it was built on —
"the translation refusing an assignment that does not fit and the generated
Fortran bridge refusing *the same one*". With two staged assignments there are
two, and they have different lengths.

**Why this unit and not an earlier one.** `interp1d` (unit #23) established
"refuse rather than truncate" because a shortened message is the one wrong
answer a byte comparison cannot tell from a right one. `interp2d` (unit #45)
inherited the helper and never composed: its own `RoutineName//':'` prefix is
reached ONLY on the bilinear path, and the bilinear path is the one path on
which it does not call `interp1d`. That is why interp2d's `98926651` could be
declared UNREACHABLE with R13 applied — its R13 block never reaches
`assign_errmsg` at all. `AeroDynTorque` calls `interp2d` on every case and
prefixes on every case with `aviFAIL < 0`, so the two assignments land in one
buffer, one after the other.

**PROPOSED AMENDMENT.** A reference-side write-back that cannot carry the
reference's own answer should make the case **INADMISSIBLE**, not failed. The
harness already keeps an `inadmissible` count and the concept is exactly this
one: on those fourteen cases the reference's answer is 30 bytes, neither side
can carry it, and the value the oracle records — what `ErrMsg` held on ENTRY —
is a value upstream ROSCO never produces (P7: the oracle is the original
source). The fix needs the generated bridge to signal "capacity exceeded" back
to the test, which is a change to a generator every unit's evidence came from.

**NOT TAKEN IN THIS DISPATCH, and the reason is X3.** Changing the emitted
bridge changes what every unit's harness compares. What was done instead is
`--disable R13_staging_capacity` on this unit, with the price measured:

```
BASELINE   14 failed   cases 1140..1153            (evidence/AeroDynTorque/
88466711   15 failed   cases 1140..1153 + 1154      mutation.survivors_on_
11c1e326   14 failed   identical set                full_corpus.txt)
06f7d2c8   14 failed   identical set
26021804   14 failed   identical set
```

**Exactly one mutant, at exactly one case** — the `cap == 30` case that sizes
the buffer TO the composed message. `88466711` is left standing as an honest
survivor with that case named, and is deliberately NOT declared unreachable the
way five other units at that same site declared it.

The comparison is between failing case SETS and not between counts, on purpose:
both the baseline and every mutant are red on the full corpus, and a mutant that
turned one of the baseline's own failures into a pass while adding one elsewhere
would show the same count.

### `--disable` reported an ablated rule as one that had no site (X2, fixed)

`--disable R13_staging_capacity` wrote, into the committed artifact:

```
N/A  R13_staging_capacity  no deferred-length CHARACTER output -- no parameter
                           whose LENGTH the callee chooses
```

which is false. Every rule's `if <sites> and "<rule>" not in off:` leaves its
detail at the string it was INITIALISED to, and that string is the sentence for
"the rule found nothing to do". A reader concludes the rule was inapplicable,
when it was switched off and its 256 cases are missing from the corpus every
later number — the mutation score included — is taken over. `--disable` exists
to measure a rule's kill set, and the two runs it compares are not comparable if
neither says which one it is.

Fixed in `translation-loop@b875e83`: one rewrite after the table is built, not
ten edits at the sites. Three controls, all taken:

* the corpus hashes identically either side of the change (`0893b9eb…8972`), so
  it is additive (P5) — proved by hashing, not by reading the diff;
* the ablated row now says ABLATED, and does **not** quote the uncomputed detail
  as a counterfactual. The first version of the fix appended "Had it not been
  disabled: <detail>", which turns one false sentence into a false sentence
  wearing a hedge — whether the rule had a site is decided inside the block that
  did not run;
* `--disable R13_staging_capacityy` raises `ValueError` instead of silently
  ablating nothing.

`pytest`: 345 passed / 73 failed / 9 errors, the same counts as before the change
(the 73 are environmental — no `git` in `vit-dev`).

### A call site with fifteen thousand hits that the gate cannot see, because a gain is 0.0

The gate red test moved scenarios 1, 7, 8, 12, 16 and 25 — and **not 17**.
Scenario 17 is the only one of the 27 that reaches this unit's SECOND call site
(clean `ControllerBlocks.f90:389`, the I&I arm, `WE_Mode == 1 .AND. WE_Op > 0`),
15,062 hits there and 0 everywhere else.

Two probes, both committed:

```
gate.scenario17-probe.json    PI 3.14159265359 -> 3.2   0 of 208,000
                              (a 1.9% change in the rotor area)
gate.scenario17-probe2.json   the RETURN VALUE forced to >= 1.0e5 N-m
                                                        0 of 208,000
```

Scenario 17 is not a dead scenario — its baseline has 16,000 distinct
`gen_speed` values and 14,085 distinct `gen_torque` values. The cause is one
number in the input file, `Examples/DISCON.IN:123`, `WE_Gamma = 0.0`, against an
arm that reads

```
WE_VwIdot = WE_Gamma/WE_Jtot * (VS_LastGenTrq*GearboxRatio - Tau_r)
WE_Vw     = WE_VwI + WE_Gamma*RotSpeedF
```

so the returned torque is multiplied by exactly zero and the estimate never
moves. **A hit count does not say a call site is observable** — this is P9's
sharpest form and the second instance in two units, after #47's scenario 8
holding `AWC_amp` at 0.0.

The first reading of the absence — "the value only reaches a channel under
`WE_Mode == 2`" — was written into `gate.redtests.txt` and is WRONG; it is
corrected in the file rather than left standing beside the right one.

### The reference's parentheses are load-bearing, and the sweep says by how much

Both `assoc_reorder` mutants of this unit's two grouped products were killed,
and the margins are the argument for transcribing the shape rather than the
algebra:

```
PI * (R*R)         ->  (PI * R) * R          killed, 52 of 1131
0.5 * (rho*area)   ->  (0.5 * rho) * area    killed,  1 of 1131
```

**One case out of eleven hundred** is the entire difference for the second. A
corpus one rung narrower would have reported it as an equivalence.

### `vit integrate --apply` strips every comment from vit.yaml, third unit running

1091 lines to 246. Established that the ONLY semantic difference was the one
added entry by loading both with `yaml.safe_load` and walking them key by key,
rather than by reading a 61 KB diff; then `git checkout -- vit.yaml` and the
entry put back by hand, with `verification: simulation` dropped as for
`ratelimit` and `ActiveWakeControl`. Recorded a third time because three
occurrences is the point at which "somebody remembers" stops being a repair.

### Two survivors are not corpus gaps: this harness cannot kill them on any input (a proposed method amendment, and a category the loop has no name for)

Second dispatch of unit #48, sent to address exactly the four survivors. The
first dispatch left them standing with a measurement of the form "the corpus does
not contain the killing input"; that measurement is true and it is the wrong
question, because it cannot distinguish an input the corpus *does not draw* from
an input the harness *cannot score*.

**THE QUESTION THAT DECIDES A DISPOSITION.** For a mutant M of unit U, a corpus
can kill M only at a case where the ORIGINAL agrees with the reference and M does
not. So the question is not "is the killing input in the corpus" but

```
    does any input exist with   ORIGINAL == REFERENCE   and   MUTANT != REFERENCE ?
```

`evidence/AeroDynTorque/aerodyntorque.green-kill-probe.cpp` answers it by
executing both chains — the harness reference (one capacity gate, on the final
message, at the generated bridge's export) and the translation (two: the
`interp2d_c` bridge's write-back, then `assign_errmsg`) — over 203,700
configurations: 5 entry shapes × 3 fills for the bytes outside `[0, n)` ×
`n_ErrMsg` in [−2, 32] × capacity in [0, 96] × the four messages `interp2d` can
leave. Negative control: the ORIGINAL scored against itself, **0** green kills.

```
mutant     site                               GREENKILL  orig-red  kill-any
88466711   assign_errmsg  '>' -> '>='              2100     50616      3096
11c1e326   errmsg_trim    'n > 0' -> 'n > 1'         18     50616       504
06f7d2c8   errmsg_trim    ': 0' -> ': 1'              0     50616      1680
26021804   errmsg_trim    '+ 1' -> '+ 2'              0     50616     13344
```

**`06f7d2c8` AND `26021804` ARE DISPOSITION (c).** They are separated from the
original at 1,680 and 13,344 configurations respectively, and **every one of
those is inside the 50,616 at which the original is ALREADY RED**. The reason is
one inequality. To see a `n_ErrMsg <= 0` or a trailing blank, the trim must read
the buffer the *unit* was given, which requires the callee's staged write to have
been REFUSED — and a callee write of `L` bytes is refused only when `L > cap`,
while the reference's own answer is `14 + L` bytes and is refused for the same
`cap` *a fortiori*. So on every input that exposes them the reference has kept
its ENTRY value, the translation has moved, and the case is red before anything
is mutated. **P12 cannot reach 1.000 for this unit by any change to the inputs.**

It cannot reach it by the amendment the first dispatch proposed either, and that
is worth stating because the amendment looks like the fix. Making a case
INADMISSIBLE when the reference's write-back cannot carry the reference's answer
reclassifies *exactly* the 50,616 — an inadmissible case is excluded from
`checked` and is not a kill. The amendment repairs the harness's honesty about
those cases; it does not make these two mutants scoreable. **A mutant whose
entire kill set is inadmissible is a category this loop has no name for**, and it
is not "equivalent": the SHIPPED program does differ there
(`n_ErrMsg <= 0`, `cap == 14`, a callee that errored — the original writes 14
bytes, the mutant 15). Escalated as a proposed amendment: the mutation artifact
needs a third bucket beside `killed` and `equivalent_declared`, for
**unscoreable-by-this-oracle**, carrying the probe that proves it. Declaring
these two equivalent would raise the score to 1.000 and would be false.

**`88466711` AND `11c1e326` ARE DISPOSITION (b), AND THE INPUTS ARE NAMED.**

* `88466711` — 2100 green kills; the corpus's own is `cap == 30`, case 1154,
  which `--disable R13_staging_capacity` removed. Un-ablating R13 costs 14 red
  cases at 1140..1153, and the fix is a per-unit statement that the ladder may
  not state a capacity in `[L_callee, L_final)` — the window the first dispatch
  already derived and measured. NOT TAKEN IN THIS DISPATCH, and the reason is
  the clock, stated rather than dressed up: every result artifact of a unit must
  name one generator commit (`revcheck` (1)), so a change to `harness/generate.py`
  forces a re-take of all eleven of this unit's harness, mutation and gate
  artifacts inside one dispatch. Named as the next dispatch's first move.
* `11c1e326` — 18 green kills, and every one of them needs THREE things at once:
  `n_ErrMsg == 1` with a non-blank byte, `cap == 14`, and a callee that
  **replaced** the message (`|M| = 42`) rather than prefixing it. R13 sweeps the
  capacity around ONE base case whose other inputs never move, so the capacity
  ladder and the character ladder never meet. The rule that would draw it is the
  cross product, and it is the rule the first dispatch's census asked for without
  being able to name the third factor.

**THE FIRST MODEL WAS WRONG AND IS KEPT.**
`aerodyntorque.trim-composition-probe.cpp` tied the callee's message length to
the entry's (`L = 9 + LEN(TRIM(entry))`) and concluded all three trim mutants
were behaviour-preserving on every admissible input — a clean argument from
`len("AeroDynTorque:") = 14 > len("interp1d:") = 9`, and false.  `interp2d` and
`interp1d` do not only prefix; they **replace**:

```
ErrVar%ErrMsg = ' xData is not strictly increasing'      Functions.f90, clean
```

so `|M|` is 42 regardless of how short the entry was, and a 42-byte message over
a 14-byte capacity puts the entry length back in front of the trim. Had that
model been trusted, three mutants would have been declared equivalent and the
score would read 1.000. It was caught by running it rather than by reading it:
the probe printed differences where the argument predicted none. **A proof of
equivalence that has not been executed against the callee's actual message set is
a hypothesis.**

### The ablation was replaced by a stated hole, and `88466711` is dead

The disposition `(b)` above was acted on rather than filed. `--disable
R13_staging_capacity` dropped **all 256** capacities to avoid the **14** that
have no oracle — and one of the 242 it took with them is the capacity at which
the mutant R13 exists to kill actually dies.

`translation-loop@1b2ba64` adds the **sixth judgement kind** to
`harness/ranges.toml`:

```toml
ErrVar_ErrMsg = { staging_capacity_excludes = [16, 29], reason = "..." }
```

a closed interval of STATED CAPACITIES that R13's ladder may not put in the
corpus, per unit and per deferred-length CHARACTER output. It is the first entry
in that file that narrows a RULE'S LADDER rather than a parameter's values, and
the reason is carried in the entry, not in a commit message.

**IT IS STRICTLY MORE INPUT THAN WHAT IT REPLACES**, which is the whole argument
for it and is why it is not "declaring inputs inadmissible to make a number go
up". The excluded window was derived and published by the FIRST dispatch, before
this mutant was in question, and the killing case sits one capacity ABOVE it:

```
corpus      1131  (ablated)  ->  1373  (stated hole)      +242 capacities
mutation    28/32  0.8750    ->  29/32  0.9062
88466711    SURVIVED         ->  killed at 2 of 1373
red test    1127 of 1131     ->  1369 of 1373             same 4 immovable cases
post-integ  1097 of 1131     ->  1330 of 1373
gate        145,146 moved    ->  145,146 moved            unchanged, as it must be
```

The R13 coverage row now NAMES the fourteen and the reason, which is the one
thing `--disable` could not say even after `b875e83` taught it to say ABLATED.

**Four refusals rather than four silent no-ops**, because a narrowing that
narrows nothing reads as a cost that was paid — the same judgement that refused
`nondecreasing_only` on `PerfData_Beta_vec` on this unit: a malformed interval, a
missing `reason`, a parameter R13 does not sweep, and an interval that excludes
no case in the generated ladder. **P5 control**: a unit that states nothing gets
a corpus identical *case for case*, asserted by a test rather than by reading the
diff. Suite 124 passed / 2 failed against 118 / 2 before.

**THE PRINCIPLED FIX IS STILL NOT TAKEN, AND THE REASON IS NOW SHARPER THAN X3.**
A case whose reference-side write-back cannot carry the reference's own answer
should be INADMISSIBLE. The first dispatch declined it as "a change to a
generator every unit's evidence came from". The real obstacle is narrower and
worse: the refusal happens in `<stem>_bridge.f90`, which **`vit test-validate`**
writes — so the fix is a change to **VIT**, the instrument that produced this
unit's extract, verify and integrate evidence, and `scripts/harness.sh`'s own
header records why this campaign does not swap VIT mid-unit.

**AND IT WOULD NOT CLOSE P12 EVEN IF TAKEN**, which is the finding the section
above establishes: `06f7d2c8` and `26021804` are killed only on cases the
amendment would mark inadmissible, and an inadmissible case is not a kill.

**WHAT IS LEFT, NAMED SO THE NEXT DISPATCH DOES NOT HAVE TO REDERIVE IT.**
`11c1e326` needs `n_ErrMsg == 1` with a non-blank byte, `cap == 14`, and a callee
that REPLACED the message — three factors at once. R13 sweeps the capacity around
ONE base case whose other inputs never move, so the capacity ladder and the
character ladder never meet and the callee-error configuration is whatever the
base draw happened to be. The rule that would draw it crosses all three. That is
a rule, not a hand-picked case, and writing it is the next dispatch's work.

**ONE PROCESS FINDING, RECORDED BECAUSE IT NEARLY REVERTED A COMMIT.** Checking
whether two test failures pre-dated `b875e83` meant `git checkout b875e83~1` in
the loop repo with the work stashed; `git checkout -` from a detached HEAD
returns to the PREVIOUS detached commit, not to the branch, and popping the stash
there produced a tree that was `eb5028e` + this dispatch's edits with
`b875e83`'s change silently absent. Caught by grepping for a string that commit
introduced before committing anything. A stash pop onto the wrong base is a
merge that succeeds.

## Unit #49 — CableControl — 2026-08-17

### A red test's revert can delete the thing under test, and nothing fails (C12, and a proposed method amendment)

`evidence/CableControl/run_postintegration_redtest.sh` ends the way every
post-integration red-test runner in this campaign ends:

```bash
trap 'git checkout -- "$F"; build; echo "reverted and rebuilt $F"' EXIT
```

It ran before the integration was committed. `git checkout --` restores to HEAD,
and HEAD's `Controllers.f90` did not carry this unit's wrapper — so the revert
deleted the **wrapper** rather than the perturbation, and the "revert-verified"
green that followed compared the C++ against the ORIGINAL Fortran body with every
callee integrated:

```
POST-INTEGRATION PASS: checked 3354  failed 0
$ grep -c cablecontrol_c rosco/controller/src/Controllers.f90
0
```

A real green, about a tree nobody meant to measure. Nothing failed, nothing
warned, and the artifact is indistinguishable from the one it was supposed to be.
What caught it was `git diff --stat` reporting `Controllers.f90` unmodified when
an integration had just been applied to it. Recorded before the fix as
`evidence/CableControl/harness.postintegration.WRONG-TREE.json`, with the
explanation written into its own `notes` field rather than only here.

**THE RULE ALREADY EXISTS ONE ARTIFACT OVER AND IS ABOUT THE WRONG FILE.**
`run_harness_redtest.sh`'s header says, in every unit that has one:

> The EXIT trap restores the translation from git. COMMIT THE TRANSLATION FIRST:
> unit #43 measured that an uncommitted edit to it is destroyed silently here.

That is the same sentence about the `.cpp`. Nobody wrote it about the `.f90`, and
the wrapper is the more dangerous of the two: an uncommitted translation is
destroyed loudly the moment the next build differs, while a deleted wrapper still
builds, still runs 27 scenarios and still prints PASS.

**PROPOSED AMENDMENT.** A red-test runner that reverts a file with `git checkout
--` should assert that the file's committed version carries what the test is
perturbing — one `git show HEAD:$F | grep -q '<unit>_c'` before the trap is armed
— and refuse otherwise. Cheap, mechanical, and it turns a silent wrong-tree green
into a refusal. Stated as an amendment rather than applied here because it
changes every unit's runner, which is X3's question.

### The three survivors are (b) and the cause is the BASE DRAW, not the ladders — third unit at this wall

Three mutants stand in `mutation/CableControl.json`, all three in the two copied
CHARACTER helpers:

```
8d2724ea  `s.size() > cap` -> `>=`     needs cap == 18 + LEN(TRIM(ErrMsg))
97aefd80  `n > 0 ? n : 0` -> `n > 1`   needs n_ErrMsg == 1 with aviFAIL < 0
1a26a963  `n > 0 ? n : 0` -> `: 1`     needs n_ErrMsg <= 0
```

`evidence/CableControl/errmsg_census.txt` reads the scored corpus's own triples
and reports 0, 0 and 0 — and, decisively, 58 cases DO carry `n_ErrMsg == 1` and
not one of them carries `aviFAIL < 0`. The cause is one fact:

```
case 0:   aviFAIL = 300     CC_Mode = -300
```

Both are midpoints of the ±1e3 default range. R13's coverage line states its own
convention — 256 capacity cases "with every other input at its base draw" — and
R6's character-length rungs are taken the same way, so **every rung that could
supply one of the three inputs sits where `IF (ErrVar%aviFAIL < 0)` is false and
neither helper is entered at all.** The capacity ladder and the arm that reads
the capacity are disjoint. This is unit #47's "`_base` puts a mode selector at
the midpoint of its range and a midpoint is not a mode", met a second time and
for a second reason: there the midpoint emptied an `IF`/`ELSEIF` chain, here it
empties two ladders.

Unit #43 recorded the same three sites as unreached and attributed it to
`_baseline_state` skipping every `char[]` parameter. That is true and it is not
the whole reason: two of the three need only that `aviFAIL` be negative WHERE the
existing ladders already run, which no baseline state and no character rule has
to change.

**TWO LEVERS EXIST AND NEITHER WAS PULLED, WITH THE COST STATED.**

1. `ErrVar_aviFAIL = { lo = …, hi = … }` and `CntrPar_CC_Mode = { lo = 0, hi = 3 }`
   in `harness/ranges.toml` move the base draw onto a live arm. Cheap — the
   corpus does not grow — and it deepens one arm, which is exactly what these
   three need.
2. `values = [-1, 0, 1, 2]` makes `aviFAIL` an R2 FLAG whose `values[0]` is the
   base and re-runs every ladder rung under every value. Unit #47's fix, and it
   multiplied that unit's corpus 6.6×.

Either is a change to the generated corpus, so `harness/CableControl.json`, the
no-op red test and all three mutation parts would have to be re-taken over it —
about 13 minutes of foreground sweeps plus the reset/restore window. This
dispatch stopped at the measurement rather than starting a re-take it might not
finish, which is RUNBOOK step 2's instruction, and the gap is named rather than
absorbed.

**AND THE TWO LEVERS CLOSE TWO OF THE THREE, NOT THREE — WHICH IS WHY THE
AMENDMENT BELOW IS ABOUT THE GENERATOR AND NOT ABOUT THIS UNIT'S `ranges.toml`.**
`8d2724ea` and `97aefd80` need only that `aviFAIL` be negative WHERE R13's
capacity ladder and R6's character-length ladder already run, and a base-draw pin
does that. `1a26a963` needs `n_ErrMsg <= 0` — the field's LENGTH, not its
content — and **no judgement kind in `harness/ranges.toml` can state it.** The
file has exactly six, and the enumeration is checkable rather than asserted
(`vit_harness.py`'s five `split_*` functions plus `statevary.constrain`):

| kind | what it states | can it give `n_ErrMsg <= 0`? |
|---|---|---|
| `lo` / `hi` | a numeric parameter's bounds | no — `ErrMsg` is `char[]`, `n_ErrMsg` is not a parameter |
| `values` | a numeric parameter's declared values, R2 flag | no, same |
| `text = "..."` | a CHARACTER parameter's CONTENT, as code points | no — it pins ONE string in every case and destroys R6's ladder rather than extending it |
| `same_as` | two extents jointly admissible | no — this is not an extent |
| `no_oracle` | an OUTPUT has no answer | it would EXCLUDE the field, not reach the value |
| `ordered_only` / `nondecreasing_only` | an ARRAY's ordering ladder | no |
| `staging_capacity_excludes` | a hole in R13's capacity ladder | it removes capacities; it cannot move a length |

And `harness/baseline.CableControl.json` cannot do it either: `_baseline_state`
`continue`s on every `p.kind == "char[]"` (`harness/generate.py:1138`), with its
own comment saying a unit whose admissibility depends on a string's bytes is one
the convention would have to be extended for, **loudly**.

So the honest statement is stronger than "not pulled": **P12 cannot reach 1.000
for this unit by any entry in the files this campaign owns.** It needs R6's
character ladder to emit a zero or negative supplied length — the view's own
`VIT_CHAR_UNALLOCATED` state, which the C side is built to carry and the corpus
never produces — and that is a change to `harness/generate.py`, i.e. to the
instrument. Unit #43 recorded the same three boundaries as unreached and did not
say that no file could reach them; this unit checked.

**PROPOSED AMENDMENT, and it is about the generator rather than this unit.**
R13 and R6's character ladder both take "every other input at its base draw", and
the base draw is computed with no knowledge of whether it reaches the code the
ladder exists to exercise. A rule that swept its own quantity **crossed with the
unit's predicate knobs** — rather than at one arbitrary point of them — would
close all three of these and unit #43's three, at a cost proportional to the knob
count. Three units have now paid for it: #43, #48 and this one.

### A zero-initialised nested type can put a NaN in front of every coefficient a callee has

Not an amendment — a reading rule, and it cost this unit a whole sweep to find.
`PIController`'s `kp` and `ki` both survived the first sweep while the saturation
bounds beside them were killed on 1703 and 1 cases, which reads as "the
integrating arm is reached and clamped, so a gain inside the clamp changes
nothing" — an ordinary corpus gap with an ordinary fix. It is wrong. The kill
counts are about `saturate`, and what they cannot show is that the value being
saturated is NaN:

```
Filters.f90, SecLPFilter_Vel
  IF ((iStatus == 0) .OR. reset) THEN
      ... the six lpfV_ coefficients are computed HERE and only here ...
  ENDIF
  SecLPFilter_Vel = 1.0/FP%lpfV_a2(inst) * ( ... )
```

`LocalVar%FP` is a nested type the generator zero-initialises and does not vary —
it says so in its own coverage output — so on every case with `iStatus /= 0` and
`restart` false the divisor is 0.0 and the filter returns `Inf * 0.0`.

**AND THE SECOND HALF IS WHY NO SINGLE KNOB FIXES IT.** The corpus's only route
to an initialised filter is `restart`, which is the SAME value this unit hands
`PIController` as its `reset` — and that arm assigns `I0` and reads neither gain.
So the one configuration in which either gain can reach an answer is `iStatus ==
0` **with** `restart` false: two names, one of which must be on and one off, and
no ladder crosses them. A baseline state can state both at once and that is what
`harness/baseline.CableControl.json` does.

The general form: **when a survivor sits at a coefficient, check what the value
it multiplies actually IS on the cases that reach it, not only whether those
cases exist.** A NaN, an exact zero and a saturated constant all make a
coefficient unobservable, and only the third leaves a trace in the kill counts.

## Unit #49 — CableControl, second dispatch — 2026-08-17

### `harness/baseline.<Unit>.json` is a SEVENTH place a CHARACTER length can come from, and the first dispatch said it was none (a correction, and a proposed method amendment)

The first dispatch recorded, in three committed artifacts — `STATUS.md`,
`evidence/CableControl/errmsg_census.txt` and the `why` block of
`harness/baseline.CableControl.json` — that one surviving mutant could be closed
by nothing this campaign owns:

> `1a26a963` needs `n_ErrMsg <= 0`, the field's LENGTH, and `harness/ranges.toml`
> has exactly six judgement kinds, none of which states one … while
> `_baseline_state` skips every `char[]` parameter outright
> (`harness/generate.py:1138`).

**The line number is right, the reading of it is not, and the difference is
thirty lines.** That `continue` sits inside `for p in sig.inputs`:

```python
        if p.kind == "char[]":
            # A string is left to the corpus: R6's character ladder owns it …
            continue
```

— it skips the string's **bytes**. After that loop has closed:

```python
    # The extents the state states, applied last so an array body written
    # against them is not cut to the case's own extent.
    for k, v in vals.items():
        q = _maybe(sig, k)
        if q is not None and q.role == "extent":
            over[k] = int(v)
```

Every extent the state names is applied, **output extents included**, and
`_case_impl` folds an extent override into `extents` before it fills anything —
so the string is drawn by R6's corpus **at the length the state stated**. A
baseline state cannot say a string's bytes; it can say its length. The two
sentences are not the same sentence and the first dispatch collapsed them.

The general form, and it is what makes this worth an amendment rather than a
correction: **an enumeration of "the kinds of judgement a campaign can state" is
a claim about a set of FILES, and the enumeration this campaign wrote down
covered one file.** `harness/ranges.toml`'s six kinds were derived correctly, from
`vit_harness.py`'s `split_*` functions plus `statevary.constrain` — and then used
to answer a question about the input domain as a whole, which `baseline.
<Unit>.json` also constrains and which no part of that derivation looked at. The
check that would have caught it is one line long: before writing "no entry can
state X", grep the OTHER input file for the name of the thing X is a property of.

### The base draw is a rule's coverage, not one parameter's value

`R13_staging_capacity`'s own coverage line says its 256 cases are taken "with
every other input at its base draw", and R6's character rungs are taken the same
way. So a base draw that sits on a dead arm does not cost one case — **it costs
the whole rule.** This unit's `ErrVar_aviFAIL` was left at the unstated ±1e3
default, whose base draw at this parameter's index is `+300`, and `IF
(ErrVar%aviFAIL < 0)` is the guard on both ErrMsg helpers: R13's entire capacity
ladder and R6's entire character-length ladder ran where neither helper was
entered. 256 + 366 cases spent, and three mutants standing because of it.

```
harness/ranges.toml     ErrVar_aviFAIL = { values = [-1, 0, 1] }

                                first take   second take
  corpus                              3354          7640
  aviFAIL < 0                          223          2767
  cases with cap == need & aviFAIL<0     0             1
  cases with n == 1, trim 1, aviFAIL<0   0            59
  cases with n == 0 & aviFAIL < 0        0            15
  mutation                       82 of 85      85 of 85
                                   0.9647        1.0000
```

**`values` and not `lo`/`hi`, and the reason is a second-order effect worth
stating.** Both move the base draw — `_case_impl` takes `p.values[0]` for a
parameter that states values and `_base`'s fraction for one that states bounds.
But a `lo`/`hi` pin ALSO removes the parameter from R6's integer ladder
(`q.lo is None and q.hi is None` is the filter) and leaves its value to
`rng.randint`, and two of this unit's three mutants need `aviFAIL` at **exactly**
0 and **exactly** -1. A uniform draw over a 1500-wide interval supplies neither.
Stating the domain gets the base draw AND keeps the exact values, at the price of
the flag crossing — arity 2 → 5, corpus 2.3×.

**And a stated `values` list does not bound R7's knobs.** The corpus's aviFAIL
histogram after the change is `{-1: 2767, 0: 2446, 1: 2422, 2: 5}`: the knob
cross-product draws its values from the reference's own literals and is not
filtered by the declaration. That is a fact about what `values` means — it states
the base draw and the flag crossing — and it was measured rather than assumed,
because the entry's own cost paragraph had asserted the opposite.

### A dispatch that changes only the corpus must re-take every layer that reads it, and exactly those

The translation is byte-identical across both dispatches of this unit. What
changed is two entries in two committed input files, and what had to be re-taken
is everything downstream of the case file: the clean-tree harness green, its
no-op red test (7324 of 7640 — the same corpus count, unit #26's rule), the three
mutation parts and their merge, the post-integration green, its red test and the
green after its revert. **The gate was NOT re-taken and that is not an omission**:
it runs 27 simulations against the shipped library, never reads the corpus, and
neither the translation nor the wrapper moved. `revcheck` reporting all twelve
result artifacts at one revision is what makes that claim checkable rather than
asserted.

The corpus is **not** a strict extension this time, and the reason is worth
knowing before choosing the lever: R11 states append, so states 4 and 5 leave
every earlier case index alone — but a new FLAG renumbers every stage before R11.
A campaign that wants its greens comparable case-for-case across a corpus change
should reach for a baseline state first and a flag only when the base draw is
what is wrong.

## Unit #50 — FlapControl — 2026-08-17

### The reference reads a local it never writes, and the region where that matters is a RELATION between two inputs

`Controllers.f90:653` declares `REAL(DbKi) :: RootMyb_VelErr(3)` and no statement
in `FlapControl` — or anywhere else in ROSCO; `grep -rn` finds the declaration
and one use — ever assigns it. `Controllers.f90:670` passes `RootMyb_VelErr(K)`
to `PIIController` as its `error`. The value is undefined in Fortran, no
translation can reproduce it, and the first harness take was RED at 12 of 9721,
every mismatch on `LocalVar.piP`.

**The failing set was predicted before it was read and it matched exactly**:
`Flp_Mode == 2 ∧ iStatus == 0 ∧ restart == 0 ∧ NumBl >= 1`. `PIIController` reads
`error` on its `.NOT. reset` branch only, and the shipped program cannot reach
that branch there — `ReadAvrSWAP` runs at the top of every DISCON call, before
every controller, and ends with `LocalVar%restart = (LocalVar%iStatus == 0)`.
Nothing else in ROSCO assigns `restart`.

**So the admissible domain of this procedure is a RELATION, and
`harness/ranges.toml` had no spelling for one.** Every entry in that file narrows
ONE parameter, because that is all the generator could express: `_case_impl`
chooses each parameter independently and R2 crosses the flags as a free product.
The two remedies that existed both cost more than the defect — pinning either of
the pair deletes an ARM (`iStatus /= 0` deletes the whole initialisation arm;
`restart` held true sends the mode-2 and mode-3 arms down a branch that reads
neither gain), and `no_oracle` on the output it poisons deletes an ANSWER.

`implied_by` is the eighth judgement kind (`translation-loop@d947d92`):

```toml
LocalVar_restart = { implied_by = "LocalVar_iStatus", relation = "== 0", reason = "…" }
```

Applied after every stage, counted, reported in the coverage table as
`STATED_implication` (`rewrote 4782 of 9721`), and an ERROR if it rewrites no
case. Additive, proved by hash rather than by diff: `CableControl` states nothing
and its corpus is byte-identical either side (`b7a51f87…`, 7640 cases, harness
re-passing 7640/0).

### `no_oracle` on the output an undefined read happens to reach is not a fix for the read

This was the cheap option and it was rejected on a measurement rather than on
taste. All twelve failing cases held `Flp_Kp = 0` — R6's *isolating* pin, not the
base draw — so `kp*error` was 0 and the saturated sum hid the difference that
survived only in the integrator state. `no_oracle = "LocalVar_piP"` would have
been green on this corpus, silent about the cause, wrong at any non-zero
`Flp_Kp`, and would have cost the `piP` comparison on the 9709 cases where the
reference does have an answer for it.

**The general shape.** Which OUTPUT an undefined read reaches is a property of
the corpus; the READ is a property of the program. Fix the one that does not
move.

### A base draw stated before the survivors rather than after them

`CntrPar_Flp_Mode = { values = [2, 0, 1, 3] }` was written at the start of this
unit on units #47's and #49's finding — "a midpoint is not a mode" — and not on
any measurement of this unit's own. The unstated ±1e3 default puts the base draw
at **+300**, which passes `IF (Flp_Mode > 0)` and matches none of the four arms,
so every stage taken "with every other input at its base draw" — R6's
real-literal ladder above all, this unit's only instrument on `Flp_Kp`, `Flp_Ki`,
`Flp_MaxPit`, `DT`, `rootMOOPF` and `Azimuth` — would have been spent on a
fall-through. `2` is first because `_case_impl` takes `p.values[0]` and the
`Flp_Mode == 2` arm is the deepest this unit has.

**What this costs the campaign's own bookkeeping, and it is worth saying plainly:
there is now no measurement of what that entry bought.** Unit #49 could price its
levers because it had a dispatch on either side of them. This unit has the entry
and no counterfactual. A rule earned from two units and applied in advance to a
third is cheaper and less evidenced than one measured in place, and both those
things are true at once.

### The nine survivors are (c) because of a fact about the PROGRAM, not about the harness

All nine sit at two sites: the declaration of the never-written local, and the
argument list of the call that reads it. Eight are unkillable because `reset` is
true on every case that reaches that call and `PIIController` reads `error`,
`error2`, `kp`, `ki`, `ki2`, `minValue`, `maxValue` and `DT` only when it is
false. **Executed rather than argued** (unit #48's rule, whose own first model of
this kind of proof was false): 37 corpus cases reach the call, 37 with reset and
0 without, with the 37 as the positive control that the site is live. The ninth
is behaviour-preserving outright — the bridge converts `int` to LOGICAL with
`(reset /= 0)`, so `? 2 : 0` and `? 1 : 0` are the same argument.

The counter-check that the operator is not blind: the SAME
`0.0 - Flp_Angle → 0.0 + Flp_Angle` mutation at the mode-2 arm's call was
**killed**. It is the site that is dead, not the operator.

### A scale factor cannot be red-tested through an exact zero

`R2D` coarsened by 1.2% — four orders above the REAL(4) channel's resolution —
moved **0 of 5,252,000**. `baseline_arrays` says why in four lines: `flp_angle_*`
has **zero** non-zero samples in scenarios 3, 4, 7 and 16, and 15,999 of 16,000
in scenario 26. Scenario 4 sets `Flp_Angle = 0.0` and runs a 1-DOF simulation
whose `rootMOOPF` is exactly zero, so `saturate(0.0, …) * R2D` is 0.0 for every
R2D; the gains are non-zero and the INPUT is not.

Fourth instance in four units, and the first where the zero is the driving
SIGNAL rather than a configured gain or a state the run length never lets the
unit write. `gate/FlapControl.mode2-probe.json` separates annihilation from
blindness in one run by perturbing the same statement **additively**: 11,997
values, all three flap channels of scenario 4.

### Proposed method amendment — a campaign that can change its own generator should say so where the units can see it

`implied_by` is the second judgement kind this campaign has added to its own
harness (`staging_capacity_excludes` was unit #48's). Both were reached only
after a unit had spent time looking for a way to express something inside the
existing vocabulary and concluding there was none. The RUNBOOK's target layer now
records both, but the *invariant* layer says nothing about the loop repo being in
scope for a unit's own dispatch — and a unit that does not know it can extend the
instrument will reach for `--disable` or `no_oracle` instead, which is exactly
what units #48 and #49 first did and what this unit nearly did. Raised for the
Driver; not edited into the invariant layer here.

## Unit #51 — FloatingFeedback — 2026-08-17

### `no_oracle` on the RETURN VALUE is not available to a unit whose return value is its whole answer

Unit #39 (`ResController`) met the identical shape — a result variable assigned
on one arm of a two-arm split, returning an undefined slot on the other — and
answered it with `vit_result = { no_oracle = ... }` in `harness/ranges.toml`.
That entry is right for that unit and would have been wrong here, and the
difference is not taste.

`FloatingFeedback` writes four things: the return value, `LocalVar%Kp_Float`,
`LocalVar%piP` and `objInst%instPI`. The last three are all written by a CALLEE —
`interp1d` writes `Kp_Float`, `PIController` writes `piP` and post-increments
`instPI` — and on the clean tree both sides of the comparison reach the same
`interp1d` and the same `PIController` through the kept bridges. So the last
three are, by construction, the same implementation compared against itself. The
two arms are the only arithmetic this unit has, and the return value is where
they land. Excusing it would have left the primary layer green over nothing this
translation computes.

`ResController` could afford the entry because it has four out-parameters of its
own — `resP`'s four DIMENSION(1024) arrays — and because its unassigned path is
*reachable*, so no domain statement could remove it.

**The general form:** before writing `no_oracle` on an output, ask what the layer
is still comparing afterwards. `no_oracle` is a statement that ONE output has no
answer; on a unit with one output it is a statement that the layer has nothing to
do. Both entries are in the same file and they solve the same problem.

### The domain a procedure is CALLED on can be a narrowing that removes no arm

The pin taken instead is `CntrPar_Fl_Mode = { values = [1, 2] }`, and the reason
it is cheap is worth separating from the reason it is correct.

Correct: `Controllers.f90:95` guards DISCON's only call site with
`IF (CntrPar%Fl_Mode > 0)` and `checkinputs.cpp:297` (unit #29's translation of
the clean `ReadSetParameters.f90`) rejects `Fl_Mode > 2` with `aviFAIL = -1`
before the first controller call. Two statements, and between them `{1, 2}` is
exactly the set on which ROSCO calls this procedure.

Cheap: unit #50 rejected all four of its candidate single-parameter pins because
each removed an ARM with nine or more statements in it. The region excluded here
contains **no statement at all** — it is the absence of an `ELSE`. There is
nothing in it for the harness to have checked, so the narrowing costs the corpus
nothing but the flag arity.

### A stated `values` list bounding R7's predicate knob is checkable in one probe, and worth checking when the excluded region has no oracle

The `Fl_Mode` entry was written with a cost note carrying unit #49's measurement:
a stated `values` list does not bound R7's predicate-knob cross product
(`ErrVar_aviFAIL` at `{-1: 2767, 0: 2446, 1: 2422, 2: 5}` against a stated set of
three). The harness prints the same warning shape here —
`PREDICATE KNOB: CntrPar_Fl_Mode at [0.0, 1.0, 2.0, 3.0]` — and 0 and 3 are the
collating neighbours of the two literals this unit tests, both of which fall
through to the undefined slot.

**If the corpus held them, the green would have been a comparison of two
undefined slots that happened to agree** — unit #36's out-of-bounds pass in a
different costume, and unit #37 measured that such a comparison can report
`0 failed` just as easily as it can report denormals.

One `fprintf` and a `--no-generate` rebuild answered it (unit #47's probe, ~40 s):
6734 PROBE lines, `{1: 3518, 2: 3216}`, fall-through set EMPTY. The stated list
did bound the knob here.

**Stated narrowly on purpose.** This is one number, for one parameter, in one
run. It is not evidence that #49's `aviFAIL = 2` came from somewhere else, and it
is not a claim about the generator. The reason to record it is the asymmetry: the
opposite result would have invalidated the primary layer, and nothing but this
probe distinguishes the two outcomes.

### Two arm-scoped gate perturbations whose counts SUM to a shared one's is an arithmetic control on the arm attribution

Unit #40's rule is that two arms with disjoint scenario sets need two red tests,
and unit #47 sharpened it: a shared perturbation can cover the chain if the
artifact's `mismatched_channels` names the scenarios, so which arm each reached
is read rather than asserted.

This unit adds a third form, and it is stronger than either for the same cost:

```
the unit's answer + 0.01 rad     404,454     scenarios 3, 7, 19, 27
the mode-1 arm    + 0.01 rad     223,222     scenarios 3 and 7 ONLY
the mode-2 arm    + 0.01 rad     181,232     scenarios 19 and 27 ONLY
                                 -------
223,222 + 181,232              = 404,454     EXACTLY
```

The two arm-scoped runs partition the shared one **to the value**, and their
scenario sets are disjoint with the shared one's as their union. That is a check
that FAILS LOUDLY if the arms overlap at run time, if either arm is riding on the
other's scenarios, or if a perturbation matched more than the line it was aimed
at — none of which the scenario list alone would catch. It costs one extra gate
run (~286 s) over the two the rule already requires, and it replaces an argument
about mutual exclusion with a subtraction.

### `--reverse-copy` is decided by reading the emitted wrapper, not by remembering

The first `vit integrate --apply` here was run without the flag. The wrapper it
emitted had **no copy-back at all**, and this unit's only `LocalVariables` write
is the SCALAR `LocalVar%Kp_Float` — written by `interp1d` into the view struct
and read back by both arms to form the answer. Without
`vit_copy_scalars_to_localvariables` that write lives and dies inside the view.

It was caught because the wrapper was read before it was believed, which is unit
#49's recorded practice, and no artifact was ever taken against it. Had it not
been, the post-integration harness would have gone red on `Kp_Float` and the
diagnosis would have started at the translation.

**The rule the campaign already has is about a different file.** Every
`run_postintegration_redtest.sh` header says to commit the wrapper before arming
the trap. Nothing says to READ the wrapper after generating it. The check is
`grep -c vit_copy_scalars_to_ <the wrapper's line range>` and it is five seconds.

### An uninitialised C++ result and a zero-initialised one are both right, on different facts

`rescontroller.cpp:48` defines its result at `0.0` and says why: that unit's
unassigned path is the `reset` path, taken on the first call of every simulation,
and the caller stores the answer into `AWC_TiltYaw`. Returning an indeterminate
double there would be undefined BEHAVIOUR executed in production, not merely an
undefined VALUE.

`floatingfeedback.cpp` leaves its result uninitialised, because its unassigned
path is reachable by no ROSCO configuration (the two statements above) and
defining it would be the translation answering a question the reference does not
answer (P7).

The two choices look like a contradiction and are the same rule applied to two
different facts about reachability. Recorded together so a later reader diffing
the two files finds the distinction rather than an inconsistency.

### The Bash tool's 120 s default backgrounded a mutation sweep, and the RUNBOOK already said how to prevent it

The first mutation sweep (234 s) was routed correctly through
`scripts/run_if_time_remains.sh` and was still moved to the background at 120 s,
which is the tool's own default and not the guard's business. It was
harness-tracked, it reported back, and nothing was lost — the same outcome unit
#49 measured. The RUNBOOK's target layer already carries the fix
(`timeout: 600000`); it was applied to the second sweep and to every run
afterwards. No amendment proposed: the rule exists and this dispatch simply
proved it is easy to forget on the first long command of a session.
