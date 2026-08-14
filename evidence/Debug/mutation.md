# `Debug` — the mutation score, and what is left alive

`mutation/Debug.json`: **143 of 155 killed, 0.9226**, over 181 behavioural
mutants (1 no-compile of 182), 26 declared equivalent, 10 operators, eight
scenarios, four streams. Below the campaign's 1.000 threshold, so the unit
closes `deferred` — with twelve survivors, each named and each classified.

The first dispatch scored **121 of 178 = 0.6798** against two scenarios and
three streams. Raw, before any equivalence declaration, this dispatch scores
**143 of 181 = 0.7901**. The declarations move it to 0.9226; the *kills* moved
it first, and they moved it because the inputs and the oracle changed, not
because anything was argued away.

## Where the 22 new kills came from

| what closed it | kills | mutants |
|---|---|---|
| the **stdout** stream, added to the oracle | 9 | the whole `WRITE(*,100)` status line: its two conversion constants, its `fmod` guard negated, its call dropped, its arguments swapped |
| **scenario 31** — both control modes off with their group arrays still declared, and a RootName carrying trailing blanks | 5 | `CC_Mode > 0`, `StC_Mode > 0`, and three `TRIM(RootName)` sites |
| **scenario 32** — two cable and two structural control groups | 3 | `Ind - 1` → `1 - Ind`, three sites |
| **scenario 33** — scenario 7's synthetic drive at `LoggingLevel = 3` | 2 | `DebugOutData[11]` and `[12]`, the two channels that are identically zero in 27 and 28 |
| **scenario 30** — `LoggingLevel = 2` | 1 | the `> 2` guard, whose boundary 1 and 3 both miss |
| the **cppmutate repair** (`3f8ed43`) | 2 | two conditions that previously had no `negate_cond` mutant at all |

Two of the six new scenarios earned nothing, and both facts are worth more than
the kills would have been:

* **Scenario 29 (`LoggingLevel = 0`) cannot reach the unit.** It was added to
  kill `LoggingLevel > 0` → `>= 0` on the open guard. `DISCON.F90:75` and
  `:145` guard both call sites with `IF (CntrPar%LoggingLevel > 0)`, so at 0
  the unit is never entered — the inner guard is dominated by the caller's.
  The mutant survived and wrote no file, which is the measurement.
* **Scenario 34 (two initialisations) cannot reach the deallocate arm.** A
  second Init inside one library load is refused by `ReadAvrSWAP`
  (`aviFAIL = -1`) and `DISCON.F90:145` then does not call `Debug` at all; a
  second Init after `kill_discon` reloads the library, which resets every SAVE
  datum. Scenario 34 performs two full Inits at `LoggingLevel = 3` and
  `avrIndices` was unallocated at both.

## The instrument, because it is not `vit_mutate.py`

`vit_mutate.py` scores a mutant by rebuilding the generated differential harness
and comparing the unit's **mapped outputs**. `Debug` assigns nothing in its own
signature, so every mutant would survive and the 0.000 would be a fact about the
instrument. `scripts/dbgmutate.py` uses the oracle that verified the unit
instead: each mutant is compiled into `libdiscon.so`, the eight scenarios are
run, and four streams are compared against `mutref` — a committed archive
written by a build containing no C++ `Debug` at all.

**The reference side cannot be the mutant**, which is the configuration question
unit #29 got wrong. `mutref` is bytes on disk, fixed before the sweep starts.

**The fourth stream is new here and it found a real defect before it scored
anything** (C12, `0dbf443`): `libgfortran` emits a preconnected unit's record
whole, while a fully-buffered `stdout` split one status record mid-field and
delivered eighteen more after the driver's own output. 46 of scenario 27's 110
stdout records differed. Fixed in `106d170`; the fix's own guard is under
measurement too — negating it (`7734fa25`) dies on 283 stdout records.

The sweep is ten foreground parts merged by `scripts/dbgmutate_merge.py`, which
asks `harness.cppmutate` for the operator population directly and refuses a
union that does not exhaust it.

## The 26 equivalence declarations, and the check on them

`mutation/Debug.equivalences.json`, one reason per id, applied at **merge** time
rather than at sweep time — so a claim can be revised without re-running 25
minutes of simulation, and so it can be **refuted**. The merge refuses a
declaration for a mutant the corpus killed. Controlled on this tree by declaring
`bcfcc5ae`, which died on 99,214 records; the merge exited 2 and named it.

Seven shapes, not 26 arguments:

| shape | n | why they agree |
|---|---|---|
| a buffer or table one element longer than it is read | 11 | `char c[W+1]`, `tmp[513]`, `date[12]`, `time[9]`, `*_src[27]`; every write and every read is bounded by the original extent |
| a comparison whose boundary case is a no-op | 3 | `pos >= size` appends 0 characters; `n >= W` assigns `W` to `n`, which holds `W`; `len == w` writes the same `w` bytes down either arm |
| the format helpers' strip-then-pad | 2 | the leading-zero strip is undone by the re-pad to exactly `e` digits |
| a width no call site passes | 2 | `w >= need` differs only at `w ∈ {8,9}`; the sites pass 5, 6, 7 and 20 |
| a guard the CALLER already applies | 2 | `LoggingLevel > 0` — `DISCON.F90:145` |
| a deallocate arm no Init reaches | 2 | measured by scenario 34, above |
| `fmod` vs `remainder` | 1 | each is zero exactly when `x/10` is an integer — the same predicate for **every** real, not merely for a clock |
| the asterisk-overflow length | 1 | `field()` returns `w` asterisks for any text longer than `w` |
| an unused descriptor dim slot | 1 | `CFI_CDESC_T` sizes storage; `CFI_establish` sets the rank |
| a null-guard downstream of its own allocate | 1 | every `avrIndices_size()` call is inside the arm that allocated it |

## The twelve that are still alive

Classified as the three categories ask: **(a)** equivalent, **(b)** the
harness cannot reach it — fix the inputs, **(c)** a blind spot no rule covers.
None of these is (a); that is why none of them is declared.

### (b) — two channels that are zero in every scenario. 2 mutants.

    c933177d  DebugOutData[13] -> [14]   Fl_PitCom
    4b100ace  DebugOutData[21] -> [22]   NacVaneOffset

Their siblings `[11]` and `[12]` died on 3,998 records the moment scenario 33
drove the nacelle IMU. `Fl_PitCom` and `NacVaneOffset` stay 0 there anyway:
scenario 33 sets `Fl_Mode` and `Y_ControlMode` and drives both the IMU and the
vane, but it runs at 9 m/s, **below rated**, where the floating-feedback pitch
contribution and the vane offset are not developed. **The next input is a
scenario 33 variant above rated**, and it is cheap — one `ws0` and one new
`sim_name`. It is NOT taken here because adding a scenario now would invalidate
the ten parts already swept: `dbgmutate_merge.py` refuses parts that ran
different scenarios, so the corpus is fixed once the first part is taken.

### (b) — a trim that never runs to the start, and an open that never fails. 3 mutants.

    901840cc  while (n > 0 ...)  ->  n >= 0
    1d75738a  while (n > 0 ...)  ->  n > 1
    867065d9  if (UnDb) { fclose } -> if (!(UnDb)) { fclose }

The first two differ from the original only for a RootName whose non-blank
characters have all been trimmed away — an **all-blank** name for `n >= 0`, and
one whose only non-blank is the first character for `n > 1`. Scenario 31 made
the loop body execute for the first time in this campaign (that is what killed
`33792ec2`, `5754fcef` and `c05d2bfe`), but its name still has eight non-blank
characters.

`867065d9` differs only when `UnDb` is null at the close, which requires
`std::fopen` to have FAILED — every other path reaching the close has an open
file, and skipping its `fclose` leaves the same bytes because stdio flushes it
at process exit. An input with an unwritable output directory would separate
them. Note the asymmetry that makes it worth doing: on that input the mutant
calls `fclose(nullptr)`.

**`901840cc` is the one to be careful about.** An all-blank RootName would make
the mutant read `RootName[-1]`, one byte before the buffer. What it does then is
not a property of this program, so an input that "kills" it would be measuring
the allocator. It is listed here as (b) because an input reaches it, and flagged
as (c)-shaped because what the input exposes is undefined.

### (c) — a difference that needs a computed double on an exact constant. 6 mutants.

    5b456058  fabs(DebugOutData[I]) < 1E-99   ->  <=
    be38d628  fabs(LocalVarOutData[I]) < 1E-99 ->  <=
    dd717e22  fabs(DebugOutData[I]) > 1E+99   ->  >=
    1e024411  fabs(LocalVarOutData[I]) > 1E+99 ->  >=
    c64442c8  fabs(DebugOutData[I]) > 1E+99   ->  DebugOutData[I] > 1E+99
    940f4622  fabs(LocalVarOutData[I]) > 1E+99 ->  LocalVarOutData[I] > 1E+99

The first four differ from the original on exactly one value each: `|x|` equal
to the double nearest `1E-99` or `1E+99`. The last two differ when `x` is more
negative than `-1E+99`.

**This is a category, not four unlucky mutants, and it is the escalation this
unit raises.** The oracle here is a whole simulation: inputs are wind speeds,
mode flags and a time step, and what the mutants read are values *computed* from
them through the estimator, the filters and the controllers. No admissible input
selects a debug channel's value; nothing in the corpus, and nothing that could
be added to it, places a double on a named constant. Two of the six are further
out of reach: `LocalVarOutData` is fed from `avrSWAP`, which is `REAL(ReKi)` —
a 4-byte float — so its smallest non-zero magnitude is about `1.4E-45` and
`1E-99` is not representable in the source domain at all.

What WOULD reach them is a different instrument: a driver that calls `Debug`
directly with a synthetic `DebugVariables`, comparing the Fortran reference's
bytes against the C++ translation's for the same constructed state. That is the
campaign's "custom Fortran test harness" shape and it is **not built here** —
it needs the 159 `LocalVariables` fields set explicitly on both sides, which is
more than this dispatch's remaining clock. Recorded as the unit's largest open
gap, above the two-zero-channels one, because widening the scenario corpus
cannot close it in principle rather than in practice.

### (c) — an out-of-bounds write that lands in padding. 1 mutant.

    c3a5bb71  for (int I = 0; I < nDebugOuts; ++I)  ->  I <= nDebugOuts

`DebugOutData` is a `std::vector<double>` of 26 elements and the mutant's extra
iteration reads and possibly writes element 26. The behaviour is undefined; in
practice it touches allocator padding and no written byte moves. **No byte
comparison of program output can see this**, and neither can a wider corpus:
the difference is not in the program's values, it is in whether the program has
defined behaviour at all. `Read_OL_Input` recorded the same shape with the sign
flipped — there the mutants read one byte PAST a buffer.

The instrument that answers it is a sanitiser build (`-fsanitize=address`) run
over the same eight scenarios, which is a **method-level** addition rather than
a corpus one: it would apply to all 31 translations and would turn every
out-of-bounds mutant in the campaign from a survivor into a kill. Raised in
`DECISIONS.md`.

## Why eight scenarios and not the 24 files

A mutant costs a rebuild plus every scenario in the corpus — about 25 seconds
across the eight — so 182 mutants is 76 minutes and ten foreground commands.
Scenario 28 is the only shipped-configuration input that reaches the
`.RO.dbg2`/`.RO.dbg3` half of the unit, 27 adds a second set of dynamics, and
29–34 were each chosen from a named survivor rather than for coverage.

**The corpus is still the largest lever and this dispatch proved it twice:** the
same six scenarios that cost about 1.5 seconds each bought 11 kills, and the
one measurement the first dispatch left behind — `DebugOutData[11] → [12]`
dying on 23,998 records of `vit_sim7.RO.dbg` — was closed by copying scenario
7's drive into a 100-second scenario.

## The no-compile

One of 182, `404c7b6a`: `arith_op` reading `esign + edig` — a `char` plus a
`std::string` — as arithmetic and producing `esign - edig`. It is a genuine
type error rather than a lost mutant, and it is reported rather than counted:
`nocompile` is excluded from both sides of the ratio.

Three mutator defects were fixed before the first dispatch's number existed and
a fourth before this one's (`3f8ed43`, X2): `negate_cond` matched to the last
`)` on the line rather than to the condition's own, so a brace-less `if` whose
statement contains a call produced code that did not compile — and the
CONDITION got no mutant at all. Nine conditions across four of the campaign's
31 translations were in that state. Two of the three in this unit died
immediately, one on 99,214 records.
