# GetWords — kernel-layer evidence

Every file here was produced by `vit verify GetWords ... --kernel-dir kernel/GetWords`
against the 62-case window `0:0:1-20,0:0:12000-12020,0:0:23900-23920` captured at
the clean `ROSCO_Helpers.f90:1114` call site (inside `FindLine`), scenario 1.

| file | what it measures |
|---|---|
| `kernel.verify_fields.csv` | the translation: **62/62 `IDENTICAL`** |
| `kernel.noop-stub.verify_fields.csv` | a stub reading nothing and writing nothing: **62/62 `OUT_TOL`**. The comparison is ALIVE — this call site does not alias its arguments, so unit #7's mirror shape does not apply here |
| `kernel.constant-stub.verify_fields.csv` | a stub writing case 1's answer as a literal: **1/62 `IDENTICAL`, 61 `OUT_TOL`**. Unit #6's shape — the kernel is not a lookup table |
| `kernel.first-element-stub.verify_fields.csv` | the real translation with `Words(1)` overwritten: **62/62 `OUT_TOL`**. The kernel compares EVERY element of the CHARACTER array, not only `Words(WordInd)` — the one element any ROSCO consumer reads |
| `kernel-window.distinct-inputs.txt` | the captured `Line` arguments, deduplicated: **57 distinct lines** out of 62 cases |
| `kernel-window.statefiles.lst` | the case list; indices are exactly 1-20, 12000-12020, 23900-23920 = 20+21+21 = 62, the configured window |

Two `NumWords` values are captured: **41 cases at 2** (the `ParseInput_*` path) and
**21 at 31** (`ParseAry`'s `AryLen + 1`). Comment lines, blank lines, single-word
lines and 60-value table lines all appear. This is the first kernel in this
campaign whose window needed no widening and no defence.

`vit verify` reports `NON_DISCRIMINATING` because it constructs its automatic red
test by perturbing a by-value floating-point argument and this signature has none.
That verdict is correct and is not argued away; the three stubs above are the
measurement it declines to make.
