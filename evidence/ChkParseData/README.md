# Unit #30 — `ChkParseData` — evidence

`SUBROUTINE ChkParseData ( Words, ExpVarName, FileName, FileLineNum, ErrVar )`,
`rosco/controller/src/ROSCO_Helpers.f90:1023` (clean baseline `54dd134`).

```fortran
CHARACTER(20) :: ExpUCVarName, FndUCVarName

FndUCVarName = Words(1)
ExpUCVarName = ExpVarName
CALL Conv2UC ( FndUCVarName )
CALL Conv2UC ( ExpUCVarName )

IF ( TRIM( FndUCVarName ) == TRIM( ExpUCVarName ) )  THEN     ! arm 1
    NameIndx = 1
    ErrVar%aviFAIL = -1
    ErrVar%ErrMsg  = ' >> ... "'//TRIM( Words(1) )//'" ... line #'//TRIM(Int2LStr(FileLineNum))//'.'
    RETURN
ELSE
    FndUCVarName = Words(2)
    CALL Conv2UC ( FndUCVarName )
    IF ( TRIM( FndUCVarName ) == TRIM( ExpUCVarName ) )  THEN ! arm 2 -- writes NOTHING
        NameIndx = 2
    ELSE                                                      ! arm 3
        ErrVar%aviFAIL = -1
        ErrVar%ErrMsg  = ' >> ... "'//TRIM( ExpVarName )//'" ... line #'//TRIM(Int2LStr(FileLineNum))//'.'
        RETURN
    ENDIF
ENDIF
```

Its only outputs are `ErrVar%aviFAIL` and `ErrVar%ErrMsg` — the family unit #29
opened. Its two callees, `Conv2UC` (#4) and `Int2LStr` (#10), are already
translated.

| layer | result | red-tested |
|---|---|---|
| kernel replay | **NOT AVAILABLE — the unit is dead in all 27 scenarios** | — (§1) |
| differential harness vs clean Fortran | **1284** checked, 0 failed, 0 inadmissible — **this unit's primary evidence** | seven stubs, §3 |
| mutation score | **23 of 27 behavioural killed, 0.852**, 4 declared equivalent, 0 no-compile, 8 operators | the score *is* the red test, twenty-three times |
| post-integration harness (wrapper only) | see §6 | see §6 |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | see §5 — **the gate is blind to this unit** |

---

## 1. C1/C2 — the unit is dead, and the guard beside the call is what says so

Five call sites, all in `ROSCO_Helpers.f90`, and **every one has zero hits in
all 27 scenarios**. `coverage/line_coverage.json` records only non-zero counts
(`coverage.py:135`, `if n:`), so an absent key is zero rather than
un-instrumented — and the P10 control for that reading is two lines away, not
borrowed from another file.

```
clean 54dd134            what it is                          hits (27 scenarios)
ROSCO_Helpers.f90:1023   SUBROUTINE ChkParseData                   0
                 88      CALL ... in ParseInput_Int                0     (site 1)
                396      CALL ... in ParseInput_Dbl                0     (site 2)
                460      CALL ... in ParseInput_Str                0     (site 3)
                568      CALL ... in ParseDbAry                    0     (site 4)
                691      CALL ... in ParseInAry                    0     (site 5)
```

The five sites are dead for **two different reasons**, and only reading the
whole path from the guard to the call separates them — unit #26's rule.

**Sites 1, 2, 3 and 5 are in procedures nothing calls.** `ParseInput` and
`ParseAry` are generic interfaces with both a unit-number form and a
`FileLines` form; ROSCO reads its input file into an array and calls the
`_Opt` variants, so the `Un`-based ones resolve to nothing. Their own
`SUBROUTINE` lines (51, 359, 423, 612) have zero hits too.

**Site 4 is different, and it is the interesting one.** `ParseDbAry` IS called —
50 times across 24 scenarios, from `ReadCpFile`:

```
ROSCO_Helpers.f90:489   SUBROUTINE ParseDbAry                    50   (24 scenarios)
                552     ... inside the body                      50
                567     IF (CheckName_) THEN                     50   <- REACHED
                568         CALL ChkParseData (...)               0   <- and FALSE, 50 times
                572     ... after the block                      50
```

`ReadSetParameters.f90:822` and `:824` are the two calls, and both pass
`CheckName = .FALSE.` as a literal:

```fortran
CALL ParseAry(UnPerfParameters, CurLine, 'Pitch angle vector', PerfData%Beta_vec,
              CntrPar%PerfTableSize(1), TRIM(CntrPar%PerfFileName), ErrVar, .FALSE.)
```

So the guard is reached 50 times and is false 50 times — 2 calls × 25 scenarios.
Reading the call site alone would say "unreachable"; reading the guard beside it
says "reached, and the caller switched it off". The distinction matters because
the second kind is one argument away from being live.

**Consequence: there is no live call site to extract a kernel at.** C3 and the
kernel layer are not available for this unit, for the same reason as units #1
(`AddToList`), #21 (`UpdateZeroMQ`) and #26 (`unwrap`). The differential harness
is this unit's primary evidence, and the gate is blind to it (§5).

## 2. C4/C5 — the translation, and the four things that had to be exact

`vit translate` scaffold in `vit_translate.stdout.txt`; the scaffold as
generated in `chkparsedata.scaffold.cpp`; the shipped file in
`chkparsedata.final.cpp`.

* **`FndUCVarName = Words(1)` TRUNCATES.** The locals are `CHARACTER(20)` and
  the callers pass words of `MaxParamLength = 200`, so a Fortran CHARACTER
  assignment drops everything past character 20 with no diagnostic — and two
  parameter names agreeing to character 20 therefore compare EQUAL. This is not
  theoretical: VIT's `narrowing-local` check names **this unit** as its
  canonical instance and `F_VSRefSpdCornerFreq`, a real ROSCO parameter, as a
  live 20-character example.
* **`TRIM(a) == TRIM(b)`** is a blank-padded comparison of two already-trimmed
  strings, so it succeeds only on equal length and equal bytes. `TRIM` strips
  blanks and blanks only — a NUL is not a blank, which is why every length is
  carried explicitly and nothing uses `strlen`.
* **`ErrVar%ErrMsg = ...` REALLOCATES** to exactly `LEN` of the right-hand side.
  `assign_errmsg` is copied from `checkinputs.cpp` (P4), including the `memset`
  to NUL past the new length: unit #29 measured leaving the previous message's
  tail at 16,729 of 16,769 failures and blank-filling at 16,769 of 16,769.
* **`CHARACTER(*) :: Words(2)`** crosses as one element width plus a
  column-major layout, element *j* at `(j-1)*len_Words`. The stride is written
  once, as `GetWords` (#8) writes it. The generated wrapper stages it the same
  way, which is what makes the two agree:
  `Words_c((vit_j - 1) * (LEN(Words)) + vit_i) = Words(vit_j)(vit_i:vit_i)`.

**`NameIndx` is deliberately not translated.** It is a local — not
`INTENT(OUT)`, not `SAVE`d — assigned 1 in arm 1 and 2 in arm 2 and never read.
A transcribed copy would be two mutable sites no input can make observable, and
units #1 and #4 measured what that costs (Conv2UC: six such sites, score 0.696).
The same argument removed four more restatements after the first mutation sweep;
see §4.

## 3. C6 — the differential harness, and the three arms counted exactly

**1284 cases against the clean Fortran, 0 failed, 0 inadmissible**
(`harness/ChkParseData.json`). Both callees cross through generated bridges to
the ORIGINAL Fortran `Conv2UC` and `Int2LStr`, so a mismatch is attributable to
this unit alone — see §7 for what it took to make that true.

Seven red tests, **all taken at the same 1284 cases as the green** (unit #26),
each through `run_harness_stub.sh`, which hash-verifies the stub from inside the
container before building:

| perturbation | failed of 1284 |
|---|---|
| the whole unit as a no-op (both error arms deleted) | **1282** |
| the `Words(1)`-matches arm alone | **52** |
| the neither-matches arm alone | **1230** |
| arm 3 reporting `TRIM(Words(1))` where the reference reports `TRIM(ExpVarName)` | **1230** |
| the `ErrMsg` staging tail blank-filled instead of cleared to NUL | **1282** |
| the reference's SILENT arm made to write `aviFAIL = -7` | **2** |
| **the `CHARACTER(20)` truncation removed** | **0** |

**The partition closes and every arm is counted.** `52 + 1230 = 1282`, which is
exactly what the whole-unit no-op moves; and the arm-2 probe pins the remainder
at 2 rather than leaving it as "at most 2", so

```
arm 1   Words(1) is the expected name      52
arm 2   Words(2) is the expected name       2      cases 360 and 821
arm 3   neither                           1230
                                          ----
                                          1284
```

Arm 2 is the reference's only path that writes nothing at all, and it is two
cases wide. A no-op stub is identical on it *by definition* — which is why the
probe that makes it WRITE is the only way to measure it, and why "1284 − 1282 =
2" would have been an inference rather than a count.

The fourth row is this unit's own hazard. The two error messages the reference
writes differ in exactly one substring, and nothing but the corpus distinguishes
them; it does, 1230 times.

**The zero is the finding, and it is precise.** Distinguishing the truncation
needs a case where `Words(1)` or `ExpVarName` exceeds 20 characters and agrees
with the other to character 20. R6 generates strings at lengths **[1, 2, 6, 11]**
— the artifact's own `rule_coverage` says so — every one of them shorter than
the boundary, so no case in this corpus reaches a 21st character at all. Three
of the four undeclared mutation survivors are the same gap seen from the other
side (§4). The defect is caught **statically** by VIT's `narrowing-local` check
and **dynamically by nothing**, which is a sharper statement than "untested".

## 4. C6 — the mutation score, and what the first sweep bought

**31 mutants over 8 operators, 23 killed, 4 declared equivalent, 4 undeclared
survivors, 0 failed to compile. Score 0.852**, scored on the CLEAN tree against
the Fortran reference (`compared_against: fortran_reference_on_a_clean_tree`).

The FIRST sweep scored **0.610** with 16 survivors, and seven of them were the
shape units #1 and #4 named: a quantity restated where nothing downstream can
read it. The remedy those units set is to delete the restatement and put the
proof in the file. Four were deleted:

| removed | why it was safe | survivors it took |
|---|---|---|
| `if (n > 0)` around `memcpy` | `n = min(len_src, len_dst) >= 0`, and a zero-length `memcpy` on valid pointers is defined | 2 |
| `if (len_dst > n)` around `memset` | `len_dst - n >= 0`, and `dst + len_dst` is a legal one-past-the-end address | 2 |
| the written-out backward scan in `ftrim` | `find_last_not_of`, the shape `checkinputs.cpp` ships; `npos + 1 == 0` handles the all-blank case with no branch | 2 (one of them `n >= 0`, which reads `s[-1]`) |
| `if (n_ErrMsg_cap > s.size())` around the tail `memset` | the refusal above already establishes `s.size() <= n_ErrMsg_cap` | 2 |

and `(len_src < len_dst) ? len_src : len_dst` became `std::min`, which has no
mutable `<` — trading one *unobservable* mutant for one *truly equivalent* one.
All eight red tests were re-taken against the new body and reproduce their
counts exactly, so §3's table describes the shipped translation and not its
predecessor.

**Four declared equivalent** (`mutation/ChkParseData.equivalences.json`), each
with a proof rather than a search: three are a buffer declared ONE BYTE LARGER
whose every access is bounded by a separate constant the mutation did not touch,
and the fourth is the symmetry of `std::min`.

**Four NOT declared** (`mutation/ChkParseData.undeclared.json`) — they are the
whole distance from 0.852 to 1.000, and three of them are one cause:

```
'20' -> '21'                                the CHARACTER(20) truncation boundary
drop_call min(len_src,len_dst) -> len_src   the same boundary, from the other side
'11' -> '12'                                reads a byte the Int2LStr bridge never writes
'>' -> '>=' on the ErrMsg cap refusal       a message exactly n_ErrMsg_cap bytes long
```

The third is the Conv2UC shape: an out-of-bounds read whose answer is a property
of the stack rather than of the input, so no corpus kills it deterministically
and it is not equivalent either. Zero-initialising the buffer would kill it and
is deliberately not done — that would define bytes the correct program never
reads, purely to move a number.

## 5. C7–C9 — integration, and a gate that cannot see this unit

`vit integrate --apply --reverse-copy`. `--reverse-copy` is required and the
test is unit #23's: the one `INTENT(INOUT)` dummy is `TYPE(ErrorVariables)`, and
the unit writes the SCALAR `ErrVar%aviFAIL` — without the reverse copy that
write would be dropped on the floor. Links clean:

```
T ChkParseData(char*, int, char*, int, char*, int, int, errorvariables_view_t*)
T __rosco_helpers_MOD_chkparsedata
T chkparsedata_c
no undefined chkparsedata symbol
```

The gate was taken **after** the integrate, so unit #23's re-take rule is
satisfied by construction rather than by inspection.

See `gate/ChkParseData.json`, `gate/ChkParseData.redtest.json` and
`gate.control-*.json` for the numbers and §7 of this file for what they mean.

## 6. E4.5 — the post-integration harness

See `harness/ChkParseData.postintegration.json` and its red test. It is the only
layer that can see a defect in the generated bridge — and for this unit the
bridge does something none of the earlier ones did: it stages a CHARACTER ARRAY
column-major from `Words(2)`.

## 7. What none of the layers can see

* **Anything at all, in simulation.** All five call sites are dead in all 27
  scenarios (§1), so the gate's 5,252,000 compared values constrain nothing
  about this unit. That is measured, not assumed: see §5.
* **The `CHARACTER(20)` truncation.** §3's zero and three of §4's four
  survivors. Closing it needs the generator to put a string LONGER than the
  reference's own fixed-width CHARACTER locals into the corpus; R6 already mines
  the reference's literals and named constants, and its length ladder is the one
  input that is not derived from the reference. Recorded as a proposed rule in
  `DECISIONS.md` rather than implemented here.
* **A message exactly `n_ErrMsg_cap` bytes long.** The `>`/`>=` survivor. The
  guard itself is not dead — negating it is killed at 1282 of 1284 — only its
  boundary is untested.
* **`NameIndx`.** Nothing observes it, in the reference either. It is not
  translated and no layer can tell.

## Files

```
README.md                              this file
done_check.txt                         the done-condition at close
chkparsedata.final.cpp                 the shipped translation, as committed
chkparsedata.scaffold.cpp              the scaffold, as vit translate wrote it
vit_translate.stdout.txt               the scaffold prompt, as generated
run_harness_stub.sh                    one stub through the differential harness
chkparsedata.noop-stub.cpp             both error arms deleted
chkparsedata.no-arm1-stub.cpp          the Words(1)-matches arm deleted
chkparsedata.no-arm3-stub.cpp          the neither-matches arm deleted
chkparsedata.sibling-message-stub.cpp  arm 3 reports Words(1), not ExpVarName
chkparsedata.errmsg-blank-fill-stub.cpp the ErrMsg tail blank-filled, not NUL
chkparsedata.arm2-writes-stub.cpp      the silent arm made to write
chkparsedata.no-truncation-stub.cpp    the CHARACTER(20) locals widened
harness.*-stub.json                    the seven red-test artifacts
coverage_deadness.py / .txt            the five call sites and the guard beside site 4
```
