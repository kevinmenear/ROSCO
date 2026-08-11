# AddToList — the prediction, and what was actually measured

`plan.json` predicted this unit's signature **cannot** cross the C bridge:

    list: c_alloc_inout does not cross
    (UNSUPPORTED: as c_alloc_out. INTENT(INOUT) ALLOCATABLE needs an ALLOCA...)

**The verdict is CONFIRMED. The basis is REFUTED.** The argument crosses fine;
VIT's generator is what does not carry it.

## What is here

| file | what it is |
|---|---|
| `vit_interface.stdout.txt` | `vit interface AddToList`, run 2026-08-10 under VIT `d07a716`, before the fix below |
| `vit_translate.stdout.txt` | `vit translate AddToList` — the full scaffold prompt it emitted |
| `addtolist.scaffold.cpp` | the scaffold itself: `void AddToList(int* list, int n_list, int element)` |
| `generators.py` / `generators.json` | which of VIT's generators the conformance matrix measures |
| `bridge_probe/` | three implementations, one driver, compared against the original |
| `bridge_probe/result.json` | the green run |
| `bridge_probe/result.redtest.json` | the same probe with one token of the CFI translation perturbed |
| `../../gate/AddToList.json` | the gate, green |
| `../../gate/AddToList.redtest.json` | the gate red test — which **failed**, on purpose |

`addtolist.scaffold.cpp` and `vit_*.stdout.txt` **can no longer be
regenerated**: VIT now refuses this signature. They are kept because they are
the record of what it did before, and a campaign that keeps only its greens has
no record of what its instruments looked like while they were lying (C12).

## 1. The generated bridge compiles and does nothing

`bridge_probe/run.sh` builds three implementations against one driver:

* `orig` — `ROSCO_Helpers.f90:1629-1657` verbatim. The oracle (P7).
* `vit`  — VIT's generated interface block and wrapper, byte for byte, with the
  most faithful C++ body its signature permits.
* `cfi`  — a hand-written Fortran 2018 `BIND(C)` interface taking the
  `ALLOCATABLE` dummy, and a C++ body using `CFI_allocate`/`CFI_deallocate`.

Scenario A is the live ROSCO path (both call sites `ALLOCATE` first, then
append); scenario B exercises the `else` branch.

```
orig   A0 size=4  1 2 3 4      A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17   B1 size=1  B2 size=2
vit    A0 size=4  1 2 3 4      A1 size=4         A2 size=4         A3 size=4         B  UNRUNNABLE
cfi    A0 size=4  1 2 3 4      A1 size=5 ...91   A2 size=6 ...92   A3 size=7 ...17   B1 size=1  B2 size=2
```

The generated wrapper's array **never grows**. Three appends, nothing appended.
`n_list` is passed BY VALUE and `list` as a bare pointer, so `allocate` →
copy → `move_alloc` — the function's entire contract — has no representation.
The best the body can do is write one past the end of the caller's heap block,
which is what `addtolist_vit.cpp` does. gfortran raised nothing; g++ raised
nothing; the run did not crash.

Scenario B does not even start: the wrapper's dummy is declared
`INTEGER(4), INTENT(INOUT) :: list(:)` — no `ALLOCATABLE` — and it evaluates
`SIZE(list)` before the call.

## 2. The substrate crosses; the generator does not

`cfi` reproduces the oracle **byte for byte on both branches**, including
`move_alloc`'s reset of the lower bound. Fortran 2018 permits an `ALLOCATABLE`
dummy in a `BIND(C)` interface, the C side receives the caller's own
`CFI_cdesc_t *`, and `CFI_deallocate`/`CFI_allocate` through it have exactly
the effect `move_alloc` has. gfortran 13.3 / g++ 13.3, aarch64.

So `INTENT(INOUT) ALLOCATABLE needs an ALLOCA` is not a fact about the
argument. It is a fact about VIT.

Red-tested (X4): `bash run.sh --red-test` perturbs one token of the CFI body
(`clist[isize] = element` → `element + 1`) and the probe goes
`DIFFERS_FROM_ORACLE`. A green that could not go red would not be evidence.

## 3. The conformance matrix was measuring a different generator

`generators.json`. `tests/test_conformance.py:probe()` fed its `bridge` and
`compiles` columns from `test_validate.generate_fortran_bridge` — the
**differential harness's** Fortran side, which is linked into a test binary and
never into the library. On AddToList that bridge does not compile:

    Error: Actual argument for 'list' must be ALLOCATABLE at (1)

which is where `[c_alloc_inout] compiles = "no"` came from, and where the plan's
basis came from. The matrix never called
`interface_gen.generate_fortran_interface_block` or `generate_fortran_wrapper`
— the two functions whose output `vit integrate` writes into the shipped source.
Those compile clean.

The verdict was right by luck of a different generator failing.

## 4. The gate cannot see this unit at all

`coverage/line_coverage.json`: the body and all five call sites have **zero**
hits in all 27 scenarios. Measured independently of gcov by perturbing both
branches of the body (`= element` → `= element + 1000`, 2 replacements),
rebuilding, and re-gating:

    RED TEST FAIL: perturbation moved 0 of 5252000 compared value(s)

`gate/AddToList.redtest.json`, `verdict: RED_TEST_FAIL`. The green in
`gate/AddToList.json` — 5,252,000 values, 0 mismatched — says nothing whatever
about AddToList, and is committed so that it cannot later be read as if it did.
