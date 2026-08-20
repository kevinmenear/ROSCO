# Unit #61 — `WriteRestartFile` — evidence

`WriteRestartFile` (`rosco/controller/src/ROSCO_IO.f90:29-367`) writes ROSCO's
checkpoint file: 307 `WRITE( Un, IOSTAT=ErrStat)` statements to a unit it opens
and closes itself, `FORM='UNFORMATTED', ACCESS='STREAM'`. `plan.json`
classifies it `respecify` — its contract is *(data in) → (bytes out)*.

## What this unit's evidence rests on, and what it cannot

| layer | result | red-tested |
|---|---|---|
| kernel replay | **CANNOT EXIST** — KGen needs a live call site and there is none; and this unit's output is bytes on a stream, which KGen neither captures nor replays | — |
| generated differential harness | **CANNOT BE BUILT** — see "why not a generated harness" | — |
| **checkpoint identity, 6 scenarios** (`chkp.json`) | **195 files, 89,858,535 bytes, 0 mismatched** — this unit's primary evidence | three, one a **negative control** |
| **driver stdout identity, 6 scenarios** (`chkp.json`) | **511 records, 131,291 bytes, 0 mismatched** — added after the first sweep | one, `avifail`, at exactly 9 records |
| post-integration total (`harness/WriteRestartFile.postintegration.json`) | **706 checks, 0 failed**, two streams | four perturbations, three red and one predicted-zero |
| mutation score (`mutation/WriteRestartFile.json`) | **62 of 62, 1.0000**; 105 behavioural, 0 nocompile, 27 equivalent, 16 unreachable, **0 open** | the score *is* the red test, 62 times |
| gate, 27 scenarios (`gate/WriteRestartFile.json`) | 5,252,000 values / 351 channels, 0 mismatched — **and it establishes nothing** | perturbed: **0**; same-build control: **1,552,676** |

## The gate is blind by construction, and this is stronger than unit #31's

`Debug` is called 408,000 times and the gate reads a different output stream.
This unit is called **zero** times: `DISCON.F90:101` sits behind
`LocalVar%iStatus == -8`, and no gate scenario ever sets that status.
`coverage/line_coverage.json` has no entry for it, while `DISCON.F90:96` — a
`CALL` two lines up — carries 408,000, and `DISCON.F90:100`, the `iStatus == -8`
test immediately above the call, runs 407,976 times and is false on all of them.
Full derivation in `gate.blind-by-construction.txt`.

So `gate/WriteRestartFile.json`'s 5,252,000 compared and 0 mismatched are true
and say nothing about this translation. That is **measured**, not inferred:
perturbing this unit moves 0, and perturbing `VariableSpeedControl` on the same
build moves 1,552,676 — reproducing unit #60's recorded count for that exact
edit to the value.

## Why not a generated differential harness

Two reasons, and the second is specific to a unit whose output is a file it
names itself.

1. `vit test-validate` compares a unit's **mapped outputs**. This one assigns
   nothing in `CntrPar` or `objInst`, and `ErrVar` only on its two I/O-failure
   arms.
2. Both sides would **open the same path**. The filename is
   `TRIM(RootName)//TRIM(n_t_global)//'.RO.chkp'`, a function of the inputs, so
   the reference and the translation would write to one file and the second
   would erase the first. The comparison would be of one program against itself
   — unit #29's failure arriving through the filesystem instead of the link.

## What compares it instead

`scripts/chkpcheck.py`, over two archives that differ in **one** thing:

    pre    WriteRestartFile still FORTRAN, every other unit already integrated
    post   WriteRestartFile integrated, nothing else changed

`pre` was re-taken during this dispatch by reverting `ROSCO_IO.f90` alone to its
pre-integration content, rebuilding, capturing, restoring and rebuilding — in
one command, with nothing committed in between.

**Nothing in a checkpoint is exempt from comparison.** `Debug` had to excuse a
`Generated on <date> at <time>` header; an unformatted STREAM write emits no
record marker, no header and no wall clock, so all 460,813 bytes of every file
are compared. The layout is **derived** from the reference's own 307 `WRITE`
statements and checked against a real checkpoint: `layout.txt` reports
`LAYOUT CHECK: PASS`, 307 items predicting 460,813 bytes against a file of
exactly 460,813.

**The corpus had to be built before anything could be measured**, because no
existing scenario calls the unit. `Examples/vit_sim.py` scenarios 36–41 drive
`iStatus = -8` on a schedule, registered outside `scenario_order` so no gate run
and no committed baseline moves (P5, X3). It was widened twice against a
measured number rather than a hunch: the first census reported 116 of the 307
written items constant and **zero** across the corpus, and scenarios 39
(`t0 = 500`, which is what makes `CableControl` and `StructuralControl` write
their setpoints) and 40 (`AWC_Mode = 4`, the only path that drives `resP`) took
that to 96.

## The second stream, and why it was added late

The first sweep scored `compare_op` and `negate_cond` at 8 of 22, and **all
eleven survivors were in an error path**: `assign_errmsg`'s two refusals, the
six `fwrite` success tests, and the final `ErrStat /= 0` check. A file oracle
cannot see any of them — a mutant that flips a write's success test still writes
the same bytes.

The unit's other output was there and nobody had read it. It sets
`ErrVar%aviFAIL = 1` and an `ErrVar%ErrMsg`; DISCON's own
`print *, TRIM(ErrVar%ErrMsg)` is guarded by `aviFAIL < 0`, so a **positive**
`aviFAIL` never reaches stdout — but DISCON copies the message into `avcMSG`
two lines later regardless, and `ControllerInterface` holds that buffer.
`_chkp_drive` now prints it on every checkpoint step. Scenario 41 passes a
RootName under a directory that does not exist, so both sides take the
`Cannot open` arm and the message crosses out.

Re-scored over both streams, the same 22 mutants gave 17 killed instead of 8.
This is unit #31's fourth-stream finding arriving one unit later.

## Four red tests, four predictions, four exact

Full table in `chkp_redtest.txt`.

```
logwidth  wr_log writes one byte instead of four   195 files  467,542 bytes  0 stdout
moop      rootMOOP[0] -> [1]                       195 files      891 bytes  0 stdout
zmqpit    ZMQ_PitOffset[0] -> [1]                    0 files        0 bytes  0 stdout
avifail   the Cannot-open arm sets aviFAIL = 2       0 files        0 bytes  9 stdout
```

Every count was stated before the run, and two offsets with them: `logwidth`
was predicted to shorten each file by six bytes (two default `LOGICAL`s × 3) and
to first differ at offset 29, and it does — 460,807 against 460,813, first
difference at 29.

**The pair `(moop, zmqpit)` is the load-bearing one.** It is the *same edit* —
an index shifted by one on a three-wide `REAL(DbKi)` field — on the same build
in the same run. 195 and 0. The zero is a statement about the corpus, and it is
the control under every `unreachable` declaration below.

**`avifail` is why the second stream exists**: 0 files and 9 records. Without
it the perturbation is invisible.

## Mutation: 62 of 62, and every survivor answered

105 behavioural mutants, 0 nocompile, 62 killed, 43 survivors and **none open**.
`const_tweak` and `index_offset` both hit `cppmutate`'s cap of 40, so those two
were **sampled and not exhausted**; the artifact names them in
`capped_operators`.

**27 equivalent** (`mutation/WriteRestartFile.equivalences.json`), and 20 of the
27 rest on one fact:

> `Stream::stat` is read **exactly once**, at `writerestartfile.cpp:576`, after
> all 307 writes have run, and the last of those writes is
> `wr_int(Un, objInst->instRL)`. Every assignment to `u.stat` from a helper
> other than `wr_int` is overwritten before any read, on every path.

`grep -c u.stat` is 7 (the seven helper assignments) against one read. `wr_dbl`
performs 218 of the 307 writes and never the last one — which is why its four
mutants are equivalent and **`wr_int`'s predicate mutant was killed**. That
asymmetry is the check on the argument.

**16 unreachable** (`mutation/WriteRestartFile.unreachable.json`), of which
**10 are derived, not asserted**: `scripts/chkp_unreachable.py` reads the
mutated site out of the survivor record, resolves both fields against
`layout.txt`, and reports in how many of the 195 reference checkpoints they
differ. All ten report 0 of 195 (`unreachable.txt`). Two of the ten sit on
fields the census calls `VARIES` — `IPC_KI(1)` and `IPC_KP(1)` change between
scenarios and still equal their neighbours in every file, which a
constant-zero argument would have missed.

The remaining **6 are written by hand and counted as such**
(`unreachable.hand.txt`), because a script that re-derives a set is not proof
against a hand exception beside it.

## What no layer here can see

Each is named against the mutant that measures it.

* **96 of the 307 written items are constant and zero** across the corpus, so
  an index shift between two of them moves nothing. Ten mutants.
* **The `.TRUE.` representation of a default `LOGICAL`.** Both logicals this
  unit writes — `WriteThisStep` and `restart` — are `.FALSE.` in all 195
  checkpoints. The translation writes 1 for `.TRUE.`, gfortran writes 1, and
  **no case in this corpus checks that**. One mutant (`7cb6d9d1`).
* **A checkpoint index of 0 or 1.** That is the only input separating `I0.0`
  from `%d`: the standard renders zero with `d == 0` as blanks, so a checkpoint
  at `Time == 0` is named `<RootName>.RO.chkp` with no number. The corpus'
  indices run 40 to 21,560. One mutant (`818543b1`).
* **A RootName long enough to reach `CHARACTER(128)`'s truncation.** The longest
  name any scenario composes is 38. One mutant (`f3829a51`).

The last three are closable by a corpus and not by an argument, and each costs a
scenario plus a re-take of every layer that reads the corpus.

## Respecifications, stated rather than left to be found

* **`GetNewUnit` is absorbed.** It exists to find a free Fortran unit number
  between 10 and 1024 and to report failure when there is none. A C++
  implementation opens a `FILE*`; the unit number does not exist, so neither
  does its exhaustion arm. `plan.json` recorded `absorbs: []` and
  `depends_on: [Debug]`; both are corrected at C1 — this unit does not call
  `Debug` at all, and `GetNewUnit` is its only callee.
* **`I0.0` is transcribed, not replaced by `%d`.** See above.
* **`InFile` is `CHARACTER(128)`**, so the concatenation truncates into it and
  the reference opens a *different file* rather than failing. Transcribed.
* **`IOSTAT=ErrStat` is the LAST write's status**, not a sticky `ferror`.

## Disposition

**`integrated`.** Every layer this unit can have ran and is green, the two it
cannot have are absent for reasons measured rather than asserted, and the
mutation score is 1.0000 with zero open survivors.
