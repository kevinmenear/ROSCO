# Unit #31 — `Debug` — evidence

`Debug` (`rosco/controller/src/ROSCO_IO.f90:704-1197`, ~490 lines) is the
routine that writes ROSCO's `.RO.dbg` log files. `plan.json` classifies it
`respecify`: it opens, writes and closes three files itself, so its contract is
*(data in) → (bytes out)* and not *(arguments in) → (arguments out)*.

## What this unit's evidence rests on, and what it cannot

| layer | result | red-tested |
|---|---|---|
| kernel replay | **DOES NOT EXIST** — see "the kernel cannot exist" below | — |
| generated differential harness | **CANNOT BE BUILT** — the unit assigns nothing in its own signature, so the comparison set is empty | — |
| **file identity, 27 scenarios** (`dbg.json`) | **24 files, 408,072 records, 273,377,424 bytes, 0 mismatched** — **this unit's primary evidence** | five perturbations: 24 / 407,976 / 21,792 records, and two blind at 0 |
| mutation score (`mutation/Debug.json`) | see `mutation.md` | the score *is* the red test |
| gate, 27 scenarios | **blind by construction** | — |
| format fidelity vs gfortran (`fmt_probe.txt`) | 54 records, 4598 bytes, IDENTICAL | 1 and 26 records |

## The gate is blind to this unit, and the reason is structural

`scripts/gate.py` compares `baseline_arrays/scenario_N.npz`. Those arrays are
built by `Examples/vit_sim.py` from the `avrSWAP` channels the controller
returns. `Debug` returns nothing: `ErrVar` is written only on `GetNewUnit`'s
unit-number-exhaustion arm, `avrSWAP` is `INTENT(INOUT)` and never assigned, and
everything the unit produces goes into a file no part of the gate path opens.

This is not "the gate's corpus does not reach it" (unit #27's shape) nor "no
scenario calls it" (units #1, #21, #26). The unit is called **408,000 times**
across 23 of the 27 scenarios and every one of those calls does its job. The
gate simply reads a different output stream. `plan.json` said so before any of
this ran — *"generated harness + mutation score (P13); the gate cannot decide"* —
and the first half of that turns out to be wrong for the same reason the second
half is right, which is recorded under "what plan.json got wrong" below.

## The kernel cannot exist, and it is a stronger statement than "was not taken"

KGen compares captured **state**. This unit's outputs are not state: they are
bytes in a file that KGen neither captures nor replays. The only signature
argument it can write is `ErrVar`, on an arm that requires 1014 simultaneously
open Fortran unit numbers. A kernel here would compare its inputs against
themselves, which is unit #29's `CheckInputs` blindness with no live output at
all rather than one that always answers "fine".

## What compares it instead

`scripts/dbgcheck.py`. It runs the 27 scenarios, archives every `*.RO.dbg*` they
write, and compares two archives record by record.

    pre    Debug still FORTRAN, every other unit already integrated
    post   Debug integrated, nothing else changed

Two builds differing in one thing, so a difference is this unit's. The reference
side is a committed archive of bytes produced by a build containing no C++
`Debug` at all — which is why unit #29's *"the mutation run compared the mutant
against itself"* cannot happen here even though the sweep runs on an integrated
tree: the reference is fixed before the sweep starts and nothing the sweep does
can perturb it.

**The one record that cannot be compared is named rather than dropped.** Record
1 of every file is

    ' Generated on 10-Aug-2026 at 10:44:58 using ROSCO-2.10.1'

whose two variable fields come from `CurDate()`/`CurTime()`. It is compared
*structurally* — the pattern, the surrounding literal text, the record length —
with only those two fields excluded. 24 of the 408,072 records are that one. A
line dropped from a comparison is a line the comparison cannot report on.

## What the instrument can and cannot see

Five perturbations, each rebuilt and re-run over the same 24 files and 408,072
records the green was taken over (unit #26's rule). Full table in
`dbg_redtest.txt`.

```
CHARACTER(15) -> (16) on the heading literals             24 records
ES20.5E2 -> ES20.5E3 on the DebugOutData write       407,976 records
the `< 1E-99 -> 0` clamp deleted                      21,792 records
the `> 1E+99 -> 1E+99` clamp deleted                       0
ES20.5E2 -> E3 on the LocalVarOutData write                0
```

The three non-zeros **partition the file**: 24 is one heading record per file;
407,976 is 408,072 − 96, and 96 is the four records per file that carry no
ES-formatted number (the `Generated on` line, the two heading lines and the
empty record every text file ends with); 21,792 is the 5.3% of records holding
at least one value below `1E-99`.

The two zeros are **blindness with their control on the same build**:

* **The upper clamp is dead.** Nothing this controller computes exceeds `1E+99`,
  while its sibling one line above fires on 21,792 records. Unit #28's
  one-live-arm-one-dead shape, in a two-statement pair.
* **The whole `.RO.dbg2` / `.RO.dbg3` half of the unit is dead**, and the cause
  is one line of configuration rather than an unreached branch: all 14
  `Examples/DISCON*.IN` set `LoggingLevel = 1`, so `IF (LoggingLevel > 1)` and
  `> 2` are *reached* in 23 scenarios and *false* in all of them. Committed
  coverage agrees from the other side — `ROSCO_IO.f90:1108` and `:1116` have
  hits, `:1109`, `:1118`, `:1183` and `:1187` have none. What that hides is a
  third of the unit: 159 `LocalVarOutData` assignments and their clamps, the
  159-name heading table, the `avrIndices` construction with both `AddToList`
  loops, and the vector-subscripted `avrSWAP(avrIndices)` write.

Neither is repairable without changing what the 27 scenarios feed the
controller, which moves every baseline (X3).

## Format fidelity was measured before anything was integrated

The whole unit is edit descriptors, so `fmt_probe.{f90,cpp,sh}` compiles the
**shipped** helpers (the C++ side `#include`s `debug.cpp`) against gfortran over
26 values including both clamp boundaries, a signed zero, a subnormal whose
exponent overflows `E2`, and the narrow `F` fields of the status line: 54
records, 4598 bytes, IDENTICAL, red-tested at 1 and 26 records.

Two descriptors decide record **length** and both are transcribed rather than
approximated:

* `TRn` is positional. It moves the write position and emits nothing, so a
  *trailing* `TR5` produces no trailing blanks. This is why the reference's
  records are 670 bytes and not 675, and it is the first thing a C++
  reimplementation gets wrong.
* the `:` in `(99(a20,TR5:))` sits *after* the `TR5`, so the last item's tab is
  processed and then discarded by the rule above.

And one truncation is load-bearing: the heading literals are assigned into
`CHARACTER(15)` array elements, so `'NacHeadingTarget'` ships as
`'NacHeadingTarge'`. Five of the 185 literals are affected. The width is written
at exactly one site (`FChar<15>`), which is what makes the perturbation above
move 24 records rather than nothing.

## Two respecifications, stated rather than left to be found

* **`GetNewUnit` is absorbed** (`plan.json: absorbs`). It exists to find a free
  Fortran unit number between 10 and 1024 and to report failure when there is
  none. A C++ implementation opens a `FILE*`; the unit number does not exist, so
  neither does its exhaustion arm — and that arm is this unit's *only* write to
  `ErrVar`. The reference's arm is unreachable too (it needs 1014 open units),
  but the difference is real.
* **`Int2LStr` is not called.** Its only job here is to render a format string's
  repeat count as decimal text so the Fortran format parser can read it back as
  an integer. When the records are written by a loop, the integer never leaves
  the integer domain. No part of `Int2LStr`'s body appears anywhere in the
  translation, so this is not the inlining X1 forbids; what is dropped is the
  round trip. `AddToList` *is* called, through its `_c` bridge, because it
  reallocates the `avrIndices` descriptor.
* **`CurDate`/`CurTime` stay Fortran** (`plan.json: permanent_bridges`), called
  through `rosco/controller/src/vit_debug_bridges.f90`. They read the wall
  clock, so no instrument here can compare a translation of them, and
  reimplementing them walks into the locale-dependent `%b` the check registry
  already names.

## What `plan.json` got wrong, corrected here (C1)

* `verification: "generated harness + mutation score (P13); the gate cannot
  decide"` — the second clause is right and the first is not achievable as
  written. `vit test-validate` compares a unit's mapped signature; this unit has
  no output in it. The mutation score is real, but it is scored against the
  **file** oracle, not against a generated harness. See `mutation.md`.
* `depends_on: [AddToList, Int2LStr]` — after respecification, `AddToList` only.
