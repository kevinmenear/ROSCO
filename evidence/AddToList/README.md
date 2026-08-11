# AddToList — the prediction, and what was actually measured

`plan.json` predicted this unit's signature **cannot** cross the C bridge:

    list: c_alloc_inout does not cross
    (UNSUPPORTED: as c_alloc_out. INTENT(INOUT) ALLOCATABLE needs an ALLOCA...)

**REFUTED — both halves, and it took two dispatches to say so.**

| dispatch | verdict on "does not cross" | verdict on the basis |
|---|---|---|
| first | CONFIRMED against VIT `d07a716` | REFUTED: the limit was VIT's, not Fortran's |
| second | **REFUTED**: VIT `37f8bdf` generates the descriptor bridge, and the unit is integrated | unchanged |

The first dispatch is not superseded and is kept in full below. It is what
established that the substrate crosses, and its artifacts are the record of a
generator that compiled and did nothing.

## What is here

| file | what it is |
|---|---|
| `vit_interface.stdout.txt` | `vit interface AddToList`, run 2026-08-10 under VIT `d07a716`, before anything was fixed |
| `vit_translate.stdout.txt` | `vit translate AddToList` — the scaffold prompt it emitted then |
| `addtolist.scaffold.cpp` | that scaffold: `void AddToList(int* list, int n_list, int element)` |
| `generators.py` / `generators.json` | which of VIT's generators the conformance matrix measures |
| `bridge_probe/` | three implementations, one driver, compared against the original |
| `bridge_probe/result.json` | the green run |
| `bridge_probe/result.redtest.json` | the same probe with one token of the CFI translation perturbed |
| `harness_redtests/` | the differential harness turned red, once per branch of the body |
| `../../harness/AddToList.json` | 43 differential cases against the **clean Fortran**, 0 failed |
| `../../harness/AddToList.postintegration.json` | the same cases against the **integrated build**, 0 failed |
| `../../harness/AddToList.postintegration.redtest.json` | that run with the wrapper corrupting an argument: 43 of 43 failed |
| `../../mutation/AddToList.json` | 23 of 23 behavioural mutants killed, score 1.000 |
| `../../mutation/AddToList.equivalences.json` | the 2 declared equivalent, each with its reason |
| `../../gate/AddToList.json` | the gate, green |
| `../../gate/AddToList.redtest.json` | the gate red test — which **failed**, on purpose |

`addtolist.scaffold.cpp` and `vit_*.stdout.txt` **cannot be regenerated**: the
generator that produced them no longer exists in either of its later forms. They
are kept because a campaign that keeps only its greens has no record of what its
instruments looked like while they were lying (C12).

## 1. The generated bridge compiled and did nothing (first dispatch)

`bridge_probe/run.sh` builds three implementations against one driver:

* `orig` — `ROSCO_Helpers.f90:1629-1657` verbatim. The oracle (P7).
* `vit`  — VIT `d07a716`'s generated interface block and wrapper, byte for byte,
  with the most faithful C++ body its signature permits.
* `cfi`  — a hand-written Fortran 2018 `BIND(C)` interface taking the
  `ALLOCATABLE` dummy, and a C++ body using `CFI_allocate`/`CFI_deallocate`.

Scenario A is the live ROSCO path (both call sites `ALLOCATE` first, then
append); scenario B exercises the `else` branch.

```
orig   A0 size=4  1 2 3 4      A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17   B1 size=1  B2 size=2
vit    A0 size=4  1 2 3 4      A1 size=4         A2 size=4         A3 size=4         B  UNRUNNABLE
cfi    A0 size=4  1 2 3 4      A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17   B1 size=1  B2 size=2
```

The generated wrapper's array **never grew**. Three appends, nothing appended.
`n_list` was passed BY VALUE and `list` as a bare pointer, so `allocate` →
copy → `move_alloc` — the function's entire contract — had no representation.
The best that body can do is write one past the end of the caller's heap block,
which is what `addtolist_vit.cpp` does. gfortran raised nothing; g++ raised
nothing; the run did not crash.

## 2. The substrate crosses, and now the generator does too

`cfi` reproduced the oracle **byte for byte on both branches**, including
`move_alloc`'s reset of the lower bound. Fortran 2018 permits an `ALLOCATABLE`
dummy in a `BIND(C)` interface, the C side receives the caller's own
`CFI_cdesc_t *`, and `CFI_deallocate`/`CFI_allocate` through it have exactly the
effect `move_alloc` has. gfortran 13.3 / g++ 13.3, aarch64.

VIT `37f8bdf` generates that. Nothing about this unit's integration is
hand-written: the interface block, the wrapper, the C++ scaffold, the
differential harness's bridge and the post-integration shim all come from
generators, which is what X1 and X2 require and what the first dispatch
deliberately declined to do by hand.

```fortran
    INTERFACE
        SUBROUTINE addtolist_c(list, element) BIND(C, NAME='addtolist_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), DIMENSION(:), ALLOCATABLE, INTENT(INOUT) :: list
            INTEGER(C_INT), VALUE :: element
        END SUBROUTINE addtolist_c
    END INTERFACE
```

So `INTENT(INOUT) ALLOCATABLE needs an ALLOCA` was never a fact about the
argument. It was a fact about VIT, and it is no longer true of VIT.

## 3. The conformance matrix was measuring a different generator

`generators.json`. `tests/test_conformance.py:probe()` fed its `bridge` and
`compiles` columns from `test_validate.generate_fortran_bridge` — the
**differential harness's** Fortran side, which is linked into a test binary and
never into the library. On AddToList that bridge did not compile:

    Error: Actual argument for 'list' must be ALLOCATABLE at (1)

which is where `[c_alloc_inout] compiles = "no"` came from, and where the plan's
basis came from. The matrix never called
`interface_gen.generate_fortran_interface_block` or `generate_fortran_wrapper`
— the two generators whose output `vit integrate` writes into the shipped
source. Those compiled clean, and emitted the no-op above.

**The prediction was right by luck of a different generator failing.** The cell
now reads `compiles = "yes"`, `integrates = "yes"`, and both columns measure
what they name.

## 4. What verified this unit, and what could not

| layer | result | red test |
|---|---|---|
| differential harness, 43 cases vs the **clean Fortran** | 0 failed | append branch perturbed → 18 of 43 fail; `else` branch perturbed → 25 of 43. 18 + 25 = 43, so **no case is inert** |
| mutation score | 23 of 23 behavioural killed, **1.000** | the mutator refuses to score unless the unmutated baseline is green |
| post-integration harness (the wrapper's marshalling) | 43 cases, 0 failed | wrapper corrupts one argument → **43 of 43** fail; green restored on revert |
| gate, 27 scenarios | 5,252,000 values, 0 mismatched | perturbing the **integrated C++** moved **0 of 5,252,000** — `RED_TEST_FAIL` |

**The gate cannot see this unit, and that has not changed.**
`coverage/line_coverage.json`: the body and all five call sites have zero hits
in all 27 scenarios. Re-measured against the integrated build by perturbing the
translation (`clist[nsize - 1] = element` → `element + 1000`), rebuilding, and
re-gating:

    RED TEST FAIL: perturbation moved 0 of 5252000 compared value(s)

`gate/AddToList.json` — 5,252,000 values, 0 mismatched — says nothing whatever
about AddToList, and is committed next to the red test so that it cannot later
be read as if it did. Everything this unit is verified by is in the three rows
above it.

Two mutants are declared equivalent rather than killed, and the distinction
between them and the two that were REMOVED is the point of
`mutation/AddToList.equivalences.json`: the declared pair cannot be
distinguished by ANY input (`CFI_allocate`'s `elem_len` is ignored for a
non-CHARACTER type, F2018 18.5.5.5), while the removed pair were memory errors
that no value comparison can see, and they were removed from the translation
rather than declared away.
