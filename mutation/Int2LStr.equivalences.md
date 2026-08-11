# Int2LStr — the five declared-equivalent mutants

Score without them: **0.737** (14 of 19). With them: **1.000** (14 of 14
behavioural, 5 declared, 0 nocompile).

Every one is a mutant of a BUFFER BOUND on an 11-byte `CHARACTER(11)` function
result, or of a condition that cannot change the result. None of them can be
killed by any value comparison, because none of them changes a byte the result
contains.

| id | line | mutation | why no input distinguishes it |
|---|---|---|---|
| `c509d52f` | 39 | `(Num < 0)` -> `(Num <= 0)` | the magnitude ternary. The two arms differ only when `Num != 0`; the conditions differ only at `Num == 0`, where both arms are `0`. |
| `768148eb` | 39 | `(Num < 0)` -> `(Num < 1)` | the same site, the same argument. |
| `cf251e70` | 67 | `first + i < W` -> `<=` | one extra shift iteration writes `result[W-first]`, which the very next loop overwrites with `' '` unconditionally. |
| `4a42dd3f` | 70 | `i < W` -> `<=` | one extra fill iteration writes index 11 only, which is past the declared width. |
| `5faaf5cb` | 24 | `const int W = 11` -> `12` | every index shifts by one; the shifted-in byte is read back from index 11, off the end. |

## The proof is exhaustive, not argued

RUNBOOK, unit #8: *"Before declaring a survivor equivalent on an out-of-bounds
argument, PROVE it exhaustively rather than reasoning about it."* That unit also
measured why: three identical runs of the identical command scored 0.983, 1.000,
0.983, because whether such a mutant dies depends on the heap. **The run that
reads 1.000 can be the one that measured least.**

`evidence/Int2LStr/mutant_equivalence_probe.cpp` runs the original and all five
mutants on a buffer whose 12th byte is poisoned to `'\x7f'` — a value no correct
output contains — and counts disagreements in bytes 0..10 only, over:

* every value with `|Num| <= 2,000,000` (widths 1-7, exhaustively)
* every value within 1000 of each decade boundary `10^k`, k <= 9, both signs
* every value within 1000 of `INT_MAX` and of `INT_MIN`

4,038,021 inputs. This is exhaustive over the only structure the function has:
its behaviour is a function of the decimal STRING of `Num`, so it can change
only where the digit count changes, and every such transition is swept whole.

    checked 4038021 input(s)
      mutant A <=0 ternary    differ-IN-BOUNDS 0
      mutant B <1 ternary     differ-IN-BOUNDS 0
      mutant C shift bound    differ-IN-BOUNDS 0
      mutant D fill bound     differ-IN-BOUNDS 0
      mutant E width 12       differ-IN-BOUNDS 0

## What was NOT declared away

The score reached 1.000 in three moves, and only the last one was a declaration.

1. **0.654 -> 0.731 by fixing the GENERATOR.** Nine mutants survived because
   `Num` was drawn uniformly from the default `+/-1e3` — a range nobody declared,
   for a parameter whose domain is the whole 32-bit integer. Every branch this
   unit has is a branch about the WIDTH of the number, and at `|Num| < 1000` the
   width is 1..4 in every case ever generated. An integer decade ladder now
   exists (loop repo, `harness/generate.py`), and two mutants died to it.
2. **0.731 -> 0.737 by fixing the TRANSLATION.** Four separate traversals of the
   same 11-byte buffer restated one bound three times, which is the shape units
   #1 and #4 both measured: 26 mutants became 19, and the sites that vanished
   were sites no input could make disagree.
3. **-> 1.000 by declaring the five that remain**, with the probe above.
