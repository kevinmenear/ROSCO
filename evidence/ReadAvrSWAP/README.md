# ReadAvrSWAP — unit #16

`SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar, CntrPar, ErrVar)` in
`rosco/controller/src/ReadSetParameters.f90`. The controller's whole front door:
it unpacks the Bladed swap array into `LocalVar`, decides `iStatus`, refuses a
second load of the library, chooses where blade pitch comes from, and increments
the timestep counter. **One call site in the entire controller** —
`DISCON.F90:81` — and it runs about 444,000 times across the 27 gate scenarios.

**Five layers ran. All five are alive, and two of them can constrain fewer than
half of what the unit writes.**

| layer | result | red test |
|---|---|---|
| kernel replay, 62 cases | 14,135 field rows, ALL `IDENTICAL` | the no-op stub FAILS; **the stub with the 23 zero-fed outputs DELETED passes 62 of 62** |
| differential harness (clean Fortran) | **27,656** checked, 0 failed, 0 inadmissible | the unit as a no-op fails **27,656 of 27,656** |
| mutation score | **110 of 110 behavioural killed, 1.000**, 1 declared equivalent, 2 no-compile | the inherited corpus scores **0.874 with 14 survivors** — see `## Mutation` |
| post-integration harness (the wrapper) | 27,656 checked, 0 failed | the assumed-size array forwarded as `avrSWAP(2)` at the CALL fails **27,656 of 27,656**; revert, rebuild, green |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | perturbing `GenSpeed`'s index moves **1,487,557 of 5,252,000** — this unit's own signature, matching no other committed red test |

## The finding: 23 of 43 output fields are invisible to BOTH bit-exact layers

`readavrswap.zero-fed-outputs-deleted-stub.cpp` deletes the entire
`IF (CntrPar%Ext_Interface > 0)` block — 18 `Ptfm*` assignments — plus
`VS_MechGenPwr`, `FA_Acc_TT`, `SS_Acc_TT`, `NacIMU_FA_RAcc` and `FA_Acc_Nac`.

* kernel: **62 of 62 IDENTICAL** (`kernel.zero-fed-outputs-deleted-stub-PASSES.verify_fields.csv`)
* gate: disabling the same block moves **0 of 5,252,000**
  (`gate.ext-interface-block-disabled-MOVES-NOTHING.json`) on a build whose
  `GenSpeed` perturbation moves 1,487,557
* differential harness: **26,198 of 26,198 failed**, naming the fields
  (`harness.zero-fed-outputs-deleted-stub.json`)

So the blindness is the simulation's, not the method's. Three mechanisms, and
which one applies decides whether a fix is even possible:

1. **No data.** `avrSWAP(14)`, `(54)` and `(1001..1018)` are written by nothing
   in this tree. `control_interface.py` allocates 3000 zeros and writes 24
   indices; the scenarios add six. `Ext_Interface = 1` in all 14 `DISCON*.IN`
   and coverage puts the block at 443,972 executions — a real input surface a
   Python driver emulating ServoDyn does not supply.
2. **Data written and then overwritten.** Scenario 27 sets
   `controller_int.avrSWAP[52]` and `[82]` and then calls `call_controller`,
   whose first act includes `except KeyError: self.avrSWAP[82] = 0` against a
   `turbine_state` carrying neither key. Both injections are zeroed before the
   DLL sees them.
3. **No reader the gate reads.** The 18 `Ptfm*` fields have no consumer in the
   controller at all outside `ROSCO_IO`'s debug `WRITE`, its restart `READ` and
   the `LocalVarOutData` table — and `gate.py` compares `baseline_arrays`, never
   a `.RO.dbg`.

`FA_Acc_Nac` is the one to keep in view: its readers are live and unconditional
(`Filters.f90:372` and `:395`). It is invisible only because both of its inputs
are zero — mechanism (2), the one a driver fix would repair. **Not fixed here**:
repairing it changes what the scenarios feed the controller, which moves
`baseline_arrays` and the compared count (X3, SPEC §8.4 — the Driver's call).

## The kernel said FAILED and `vit verify` said 62/62, and both were right

`vit_defects/kernel.run-says-FAILED-61-of-62.txt` is the kernel's own summary;
`kernel.green.vit_verify.stdout.txt` is what the tool printed. The override is
VIT's design and it is correct: `run_kernel_verify` replays the ORIGINAL FORTRAN
through the same kernel first and, when the two field logs agree, calls the
per-case failures state-capture artifacts. Measured rather than trusted —
`kernel.original-fortran-replay-FAILS-case2.txt` is the original Fortran
producing the identical wrong answer on the identical case.

The cause is this campaign's own `vit.yaml`: `kgen.dll_persistence` inserts
`LocalVar%AlreadyInitialized = 0` before every `CALL ReadAvrSWAP` **in the
instrumented source**, between KGen's input capture and the call, and the
generated kernel does not carry it. What is missing in VIT is a PRINT, not a
verdict: `run_kernel_verify` computes a `Note: N state file artifacts detected`
line into `result.output` and `cmd_verify` never prints `result.output`.

## Files

| file | what it is |
|---|---|
| `readavrswap.final.cpp` | the translation as integrated |
| `kernel.green.verify_fields.csv` | the green field log — 62 cases, read the `status` column, not the verdict |
| `kernel.green.vit_verify.stdout.txt` | what the tool printed |
| `vit_defects/kernel.run-says-FAILED-61-of-62.txt` | what the kernel printed (C12: the wrong artifact, kept) |
| `kernel.original-fortran-replay-FAILS-case2.txt` | the original Fortran, same kernel, same wrong answer |
| `kernel.noop-stub-FAILS.verify_fields.csv` | the kernel can fail |
| `kernel.zero-fed-outputs-deleted-stub-PASSES.verify_fields.csv` | and it cannot see 23 of 43 outputs |
| `readavrswap.noop-stub.cpp`, `readavrswap.zero-fed-outputs-deleted-stub.cpp` | the two stubs |
| `harness.zero-fed-outputs-deleted-stub.json` | the differential harness fails that stub 26,198 of 26,198 |
| `gate.ext-interface-block-disabled-MOVES-NOTHING.json` | 0 of 5,252,000, all 27 scenarios alive |

## Mutation — 1.000, reached by closing a corpus gap

`mutation/ReadAvrSWAP.json`: **110 of 110 behavioural mutants killed, score
1.000**, 1 declared equivalent, 2 excluded as `killed (no compile)`,
`declared_but_killed` empty.

The inherited corpus scored **0.874 with 14 survivors**
(`mutation/ReadAvrSWAP.survivors_before_predicate_knobs.json`, kept). All
fourteen sat in two blocks the corpus could not reach, for two separate reasons:

* **A ladder aimed at a NAME is not a ladder over the quantity.**
  `LocalVar_iStatus` is a scalar integer parameter, so the integer decade ladder
  gave it `0` — and every one of those cases was dead, because the unit's first
  statement is `LocalVar%iStatus = NINT(avrSWAP(1))` and `_fill_array` puts
  element 1 of a 3000-element ramp at about **−999 in every case ever
  generated**. Same for `NINT(avrSWAP(61))` → `NumBl` and `avrSWAP(2)` → `Time`.
* **Two ladders that never cross cannot reach a branch that needs both.** The
  pitch-fault loop needs `PF_Mode == 1` AND `NumBl >= 2` AND `iStatus /= 0` in
  the SAME case; each was individually reachable and the conjunction was not.

Closed by addition — `predicate_knobs_from` (reads the reference's predicates
and follows each tested name back to the element it enters by) plus an R7
cross-product block in `generate.py`, appended last, bounded at 4096
combinations with an all-pairs fallback that reports itself. **26,198 → 27,656
cases**, and thirteen of the fourteen die.

The fourteenth is declared, not closed: `mutation/ReadAvrSWAP.equivalences.json`
carries `e82a5f90`, `'<=' -> '<'` on the `ErrMsg` capacity guard, which differ
only at a capacity of 58 — and the capacity is `LEN(ErrMsg) + 4096` on both
sides of the boundary. Same site, same reason, same declaration as
`ExtController`'s `11d2c9cc`.

## Post-integration — the wrapper

`harness/ReadAvrSWAP.postintegration.json`: 27,656 checked, 0 failed, against
the integrated build. Red-tested with the rebuild between the edit and the run
and anchored on the CALL rather than the `BIND(C)` interface block (unit #13's
rule — `readavrswap_c` appears at both, `ReadSetParameters.f90:29` and `:59`):
forwarding the assumed-size array as `avrSWAP(2)` fails **27,656 of 27,656**.
Revert, rebuild, green returns.
