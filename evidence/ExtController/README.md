> **SUPERSEDED IN PART — 2026-08-11, second dispatch. Read this first.**
>
> Everything below is the FIRST dispatch's record. Its measurements stand; **one
> of its conclusions does not**, and it is the one the disposition rested on.
>
> §3 concludes *"the clean Fortran cannot be run to completion on any input this
> campaign possesses"* and the one-sentence summary generalises that to *"there is
> no runnable oracle for `ExtController` anywhere in this campaign."*
> **That is refuted.** The SIGSEGV is a property of the INPUT, not of the
> function: `DLL_FileName` is the literal string `"unused"` in all 14 inputs and
> no external Bladed-style library was shipped in the tree.
> `fixtures/bladed_stub/discon_stub.c` is that library, and with it the original
> runs — `exit_status: 0`, `returned_normally: true`,
> `probe_ext_mode_1_with_oracle.json`.
>
> §4's refusal is **closed** (VIT `a2e2c30`), and "What would unblock it" item 3
> is **not** a verification-default change: the differential harness does not run
> scenarios, so a fixture it links against touches nothing the gate measures.
>
> A third defect, found only by trying: `vit test-validate` was emitting bridges
> past Fortran's 132-column line limit, which had put **most of this campaign**
> outside the differential harness in silence. VIT `83d25f9`;
> `vit_defects/README.md`.
>
> The unit is still `blocked`, for a third and smaller reason. See
> `plan.json`'s escalation, `../../DECISIONS.md` and `../../STATUS.md`.
>
> The text below is kept unedited, deliberately: a conclusion that was wrong is
> worth more as a record than as a deletion.

# ExtController — unit #5, `blocked`

**Date:** 2026-08-11
**Disposition:** `blocked`
**Escalation:** `blocked_substrate` (SPEC §8.4 — the blockage is in the substrate
and in the campaign's fixtures, not in the function)

Four units closed before this one. Each was a question about *which* instrument
could see the unit. This one is a question about whether any instrument can run
at all, and the answer is measured, not argued.

---

## The one sentence

**There is no runnable oracle for `ExtController` anywhere in this campaign.**
The original Fortran is never executed by any scenario, and when it is forced to
execute it **segfaults** — so P7's "compare against the original implementation"
has nothing to compare against, and all four of this campaign's verification
layers are unavailable for reasons that have nothing to do with the translation.

---

## What was measured

### 1. The gate cannot see it, and two instruments say so

| instrument | measurement |
|---|---|
| `gcov`, all 27 scenarios | `ExtControl.f90` **0 of 28 executable lines**, in every scenario |
| gate red test, Record 49 | `LEN(avcMSG) + 1` → `+ 1000000` moved **0 of 5,252,000** values |
| gate red test, toward absence | first statement → `RETURN`, i.e. the unit becomes a **no-op**: moved **0 of 5,252,000** |
| gate, unperturbed | 5,252,000 compared, 0 mismatched — and it constrains nothing |

The denominator is the point. `coverage/line_coverage.json` stores only lines
with a **non-zero** hit count, so a file that never runs and a file that was
never instrumented are **the same empty dictionary**. `ExtControl.f90` reads as
empty there, as do `Constants.f90`, `ROSCO_Types.f90` and `ZeroMQInterface.f90`.
`coverage_extcontrol.json` was regenerated for this unit specifically to print
the denominator beside the numerator — `0/28`, 27 times — which distinguishes
the two. Same lesson as the `gcov` notes-file bug the script's own docstring
records, one level up.

Both red tests carry `replacements: 1`, `perturbed: true`, `reverted: true`,
`revert_verified: true` and `residual_dirt: []`, so the perturbed library was
genuinely built and run and the tree came back. The second is the stronger one,
per RUNBOOK: it perturbs toward **absence**, so a gate that still cannot move is
blind to the unit entirely rather than to the perturbation chosen.

**Why it is dead:** `Ext_Mode` is `0` in all 14 `Examples/*.IN` — one distinct
value across every input file — and `Examples/vit_sim.py` never patches it. The
call site `DISCON.F90:90` sits under `IF (CntrPar%Ext_Mode > 0 .AND. ...)`.

### 2. There is no kernel, for the same reason

No scenario reaches a call site, so there is no runtime state to capture.
`vit extract` was not run: RUNBOOK's unit #1 lesson is that a capture failure at
a call site with no hits is deadness, not a tool defect, and the coverage query
comes first. It did.

### 3. Forcing the oracle to run makes it CRASH

`probe_ext_mode_1.py` sets `Ext_Mode = 1` through `vit_sim.py`'s own
`write_discon(patches=...)` and makes the `iStatus == 0` call.

```
exit_status: -11        signal: 11 (SIGSEGV)
config: Ext_Mode = 1 ;  DLL_FileName = "unused"
last line printed:  " Library loaded successfully"
```

`probe_ext_mode_1.json` is the artifact. The cause is in the source and was read
before the probe was written:

* `DLL_FileName` is the literal string `"unused"` in all 14 inputs, and **no
  external Bladed-style library is shipped anywhere in the tree**.
* `ExtController` **does not check `ErrVar%ErrStat`** after `LoadDynamicLib`.
  So `dlopen` fails, `DLL_Ext%ProcAddr(1)` stays `C_NULL_FUNPTR`, the code
  prints *"Library loaded successfully"*, and the next two statements are
  `C_F_PROCPOINTER` on that null and a CALL through it.

This is what removes the route `AddToList` used. `AddToList` was equally dead and
still closed `integrated`, because the differential harness could **run the clean
Fortran** as an oracle and the mutation score could be taken against it. Here the
clean Fortran cannot be run to completion on any input this campaign possesses.

### 4. VIT cannot generate the bridge — deterministically, and it says so

```
$ vit translate ExtController -f rosco/controller/src/ExtControl.f90 --module ExtControl
ERROR: Cannot map Fortran type 'CHARACTER(:)' ... Deferred-length CHARACTER(:)
is not C-compatible. Use 'vit analyze-types --fix character' to convert to
fixed-length.
```

`ErrorVariables` has four fields. Exactly one blocks it:
`CHARACTER(:), ALLOCATABLE :: ErrMsg`. It is an honest refusal, not a defect —
and it is not confined to this unit: **37 of this campaign's 69 units take a
`TYPE(ErrorVariables)` dummy argument.**

The remedy VIT's message suggests is **refused here**. `vit analyze-types --fix
character` rewrites ROSCO's own type definition to a fixed length, which changes
`LEN(ErrVar%ErrMsg)`, which sizes `avcMSG`. That is a change to the oracle, so it
is a change to the answer (P7).

The function additionally assigns to it —
`ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)` — which is a
**reallocating** assignment on a deferred-length allocatable. Any bridge has to
give the C++ side a way to reallocate it, not merely to read it. That is the
shape of the feature, and it is why the RUNBOOK's descriptor-bridge entry lists
"a CHARACTER or derived-type element" as still refused.

### 5. A defect found on the way, recorded before it was fixed (C12)

With no `strategy: view` configured for `LocalVariables` and `ErrorVariables`,
`vit interface` emitted this, **with no diagnostic anywhere in its output**:

```
CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(ExtDLL_view))
```

Five arguments in the wrapper's dummy list, **three** in the bridge. `LocalVar`
and `ErrVar` are accepted and silently not forwarded. `LocalVar%iStatus` is the
guard on the entire initialisation branch, so that bridge could never have
loaded the library it exists to call — and it compiles and links.

`vit_interface.no_view_strategy.stdout.txt` is the wrong artifact, kept.
`vit_interface.view_strategy.stdout.txt` is the five-argument version, which
appears once the strategies are configured — and which then USEs
`vit_errorvariables_view`, a module nothing can generate (see §4). The tool has
two answers and neither was a working bridge.

Fixed in VIT (`f8ab74f`), not worked around: `dropped_derived_args` already
existed and only `test_validate` asked it, so the warning now lives in
`generate_fortran_wrapper`, which every path that ships a wrapper goes through.
Additive — no generated byte changes — and red-tested in both directions.
`vit_interface.no_view_strategy.stdout.txt` shows it firing.

### 6. A second defect, in this file's own first draft

The first version of `probe_ext_mode_1.py` restored `Examples/DISCON.IN` in a
`finally`. **A `finally` does not run through SIGSEGV.** The run left
`Ext_Mode = 1` in the gate's own input file — and `Examples/DISCON.IN` is
**gitignored** (`.gitignore:78`), so `git status` stayed clean and `done.py`'s
clean-tree predicate could not have caught it. Every gate run afterwards would
have measured a silently reconfigured controller, exactly the failure `gate.py`'s
`inputs_restored` exists to prevent, one step further along.

The probe now snapshots and restores in a **parent process** and the child does
the crashing. `discon_in_restored_by_parent: true` is recorded in the artifact.

---

## What would unblock it

Three things, and they are independent. Naming them is the point of the
escalation; none of them is unit work.

1. **VIT must map `CHARACTER(:), ALLOCATABLE` in a view struct**, with
   reallocating assignment from the C++ side. Blocks 37 of 69 units, so it has
   to be built regardless of what happens to this one. **This alone does not
   unblock `ExtController`** — see 3.
2. **`ExtDLL_Type` must cross**, for the `LoadDynamicLib` permanent bridge that
   `plan.json` already declares. Five fields, two of them opaque C pointer kinds
   (`TYPE(C_PTR)`, `TYPE(C_FUNPTR) :: ProcAddr(3)`) plus
   `CHARACTER(1024) :: ProcName(3)`, a CHARACTER array — which the differential
   harness also still refuses (RUNBOOK, unit #4). Blocks this unit only. X1
   forbids inlining `LoadDynamicLib` to avoid it.
3. **An oracle must be constructed**, and this is the one that decides. A
   minimal Bladed-style external library exporting a `DISCON` entry point, plus
   a scenario that sets `Ext_Mode = 1` and points `DLL_FileName` at it. That is
   an **addition**, which P5 permits, but it adds a gate scenario and therefore
   changes the gate's compared count and its baseline set — a verification-
   default change, which SPEC §8.4 says is the Driver's call, not a unit's.

Until 3 exists, no amount of tooling makes this unit verifiable: there would be
a translation and nothing to check it against.

---

## Artifacts

| file | what it is |
|---|---|
| `coverage_extcontrol.json` | per-scenario coverage regenerated for this unit |
| `coverage_extcontrol.denominators.txt` | the 27 `0/28` lines — numerator *and* denominator |
| `probe_ext_mode_1.py` | the oracle-availability probe, crash-safe |
| `probe_ext_mode_1.json` | its result: `signal: 11` |
| `vit_translate.stdout.txt` | VIT's `CHARACTER(:)` refusal |
| `vit_interface.no_view_strategy.stdout.txt` | the three-argument bridge, and the new warning |
| `vit_interface.view_strategy.stdout.txt` | the five-argument bridge that cannot compile |
| `extcontroller.scaffold.cpp` | the scaffold, 5 parameters — disagreeing with the 3-argument bridge |
| `gate_redtests/ExtController.redtest-record49.json` | 0 of 5,252,000 |
| `gate_redtests/ExtController.redtest-noop.json` | 0 of 5,252,000, unit made a no-op |
| `../../gate/ExtController.json` | 5,252,000 compared, 0 mismatched, constrains nothing |
| `../../gate/ExtController.redtest.json` | the no-op red test, canonical copy |

**No `harness/ExtController.json` and no `mutation/ExtController.json` exist**,
and their absence is the finding rather than an omission: P11 and P12 are
mandatory for every unit in this campaign, and neither can be produced for a
unit with no bridge and no runnable reference. `done_check.py` will report them
FAIL. That verdict is correct and is kept.
