# GetRoot — what verified it, and what could not

Unit #7 of rosco-r2. Definition `ROSCO_Helpers.f90:1253` (clean numbering), the
**one** call site `DISCON.F90:67`, scenario 1.

```fortran
CALL GetRoot(RootName,RootName)          ! DISCON.F90:67 -- BOTH arguments are
                                         ! the same variable
```

```fortran
IF ( ( TRIM( GivenFil ) == "." ) .OR. (  TRIM( GivenFil ) == ".." ) )  THEN
   RootName = TRIM( GivenFil ) ;  RETURN
END IF
DO I=LEN_TRIM( GivenFil ),1,-1
   IF ( GivenFil(I:I) == '.' )  THEN
      IF ( I < LEN_TRIM( GivenFil ) ) THEN
         IF ( INDEX( '\/', GivenFil(I+1:I+1)) == 0 ) THEN
            RootName = GivenFil(:I-1)                  ! strip the extension
         ELSE ;  RootName = GivenFil ;  END IF
      ELSE
         IF ( I == 1 ) THEN ; RootName = ''
         ELSE ; RootName = GivenFil(:I-1) ; END IF
      END IF
      RETURN
   END IF
END DO
RootName =  GivenFil
```

| layer | result | red test |
|---|---|---|
| kernel replay, 62 cases, scenario 1 | 62/62 `IDENTICAL` on the full `CHARACTER(8)` | **A NO-OP ALSO SCORES 62/62 `IDENTICAL`.** A wrong-constant stub scores 62/62 `OUT_TOL`, so the instrument is alive — see below |
| differential harness, 726 cases vs clean Fortran | 0 failed | no-op → 700/726 failed, naming `RootName` |
| mutation | 60/60 behavioural killed, 1.000, **2 declared equivalent**, 0 nocompile | refuses to score unless the baseline is green |
| post-integration harness (wrapper marshalling) | 726 cases, 0 failed | `LEN(RootName)` → `LEN(RootName) - 1` in the wrapper's CALL → 596/726 failed; green restored |
| gate, 27 scenarios | 5,252,000 values / 351 channels, 0 mismatched | **FAILED — 0 of 5,252,000 moved.** Two perturbations; see below |

8 of the 60 mutation kills are CRASHES rather than case mismatches, so the
killed-by-comparison count is **52 of 60**.

## THE KERNEL IS A MIRROR: A NO-OP PASSES IT

Unit #6 found a kernel that a *constant* stub passed. This one is a step
further and the difference is worth naming.

`DISCON.F90:67` is `CALL GetRoot(RootName,RootName)`. The INTENT(IN) and the
INTENT(OUT) dummy are **the same actual argument**, so the value KGen captures as
the input is bit-for-bit the value it captures as the expected output. The state
file says so in one line:

```
$ strings kernel/GetRoot/GetRoot.0.0.1
vit_sim1vit_sim1
```

Four measurements, all committed:

| stub | verdict | artifact |
|---|---|---|
| the translation | 62/62 `IDENTICAL` | `kernel.verify_fields.csv` |
| **reads nothing, writes nothing** | **62/62 `IDENTICAL`** | `kernel.noop-stub.verify_fields.csv` |
| reads nothing, writes the literal `vit_sim1` | 62/62 `IDENTICAL` | `kernel.constant-stub-PASSES.verify_fields.csv` |
| reads nothing, writes `XXXXXXXX` | 62/62 `OUT_TOL` | `kernel.wrong-constant-stub.verify_fields.csv` |

**This inverts unit #6's recipe, and a session that followed it would have read
the result backwards.** RUNBOOK's entry says *"the no-op says the kernel is
alive, the constant stub says whether being alive buys anything"*. Here the
no-op PASSES, so it says nothing about liveness; the wrong-constant stub is what
establishes the comparison can move. Whenever a call site aliases an INTENT(IN)
argument with an INTENT(OUT) one, the no-op stub stops being a liveness test
and becomes a vacuity test.

`vit verify` declines to build its own red test and says why — correct, because
this signature has no by-value floating-point argument — and prints
`NON_DISCRIMINATING` beside its own 62/62. That verdict is kept in
`vit_verify.stdout.txt`.

The 62 cases are 62 copies of one input: every one of them is `vit_sim1`,
because `RootName = TRANSFER(avcOUTNAME, RootName)` on the line above rebuilds
the same string each call and `vit_sim.py` passes a fixed `sim_name` per
scenario.

## THE GATE CANNOT SEE THIS UNIT — AND FOR TWO INDEPENDENT REASONS

| perturbation | artifact | moved |
|---|---|---|
| `char_assign`'s copy writes `'X'` — the unit returns a WRONG name | `gate_redtests/GetRoot.redtest-wrong-output.json` | 0 of 5,252,000 |
| the unit made a NO-OP | `gate_redtests/GetRoot.redtest-noop.json` | 0 of 5,252,000 |

Both carry `perturbation.replacements: 1`, `perturbed: true`,
`revert_verified: true`, `residual_dirt: []`.

**The second one is DEGENERATE here, and that is the point.** A no-op leaves
`RootName` at its incoming value, which — because the call site aliases — is
`GivenFil`; and `GetRoot(GivenFil) == GivenFil` for every input any scenario
supplies. So the no-op is not a wrong implementation at all, it is the RIGHT
answer on the exercised domain, and its green says nothing about the gate. The
campaign's standard red test would have reported `0 moved` for a reason that has
nothing to do with observability. The first perturbation is the one that carries
the claim: it makes the unit return `XXXXXXXX` and the gate still cannot move.

### Reason 1 — the exercised domain is the identity

Every scenario's `avcOUTNAME` is `vit_sim<N>`, which contains no `'.'`. Coverage
says it precisely (clean `ROSCO_Helpers.f90`, all 27 scenarios):

| clean line | statement | hits |
|---|---|---|
| 1269 | `IF ( TRIM(GivenFil) == "." .OR. ... )` | 444,000 |
| 1270 | `RootName = TRIM( GivenFil )` | **0** |
| 1277 | `DO I=LEN_TRIM( GivenFil ),1,-1` | 4,304,000 |
| 1280 | `IF ( GivenFil(I:I) == '.' )` | 4,304,000 |
| 1283–1290 | every assignment inside that IF | **0** |
| 1302 | `RootName = GivenFil` | 444,000 |

The unit is called 444,000 times, scans 4.3 million characters, and takes the
**same one** of its five leaves every time. Its whole branching body is dead in
this campaign's scenarios.

### Reason 2 — the result is consumed OUTSIDE the gate's surface

`RootName` has six reader sites and every one of them uses it to BUILD A
FILENAME. Five of the six are DEAD in all 27 scenarios — measured, not inferred:

| reader (clean source) | statement | hits, all 27 |
|---|---|---|
| `ROSCO_IO.f90:1102` | `OPEN(unit=UnDb, FILE=TRIM(RootName)//'.RO.dbg')` | **24**, in 23 scenarios |
| `ROSCO_IO.f90:1110` | `...//'.RO.dbg2'` | 0 |
| `ROSCO_IO.f90:1142` | `...//'.RO.dbg3'` | 0 |
| `ROSCO_IO.f90:30` | `...//'.RO.chkp'` (write restart) | 0 |
| `ROSCO_IO.f90:373` | `...//'.RO.chkp'` (read restart) | 0 |
| `ReadSetParameters.f90:360` | `EchoFilename = TRIM(RootName)//'.RO.echo'` | 0 |

The one that runs OPENS A FILE. The gate compares
`baseline_arrays/scenario_N.npz`, which `Examples/vit_sim.py` builds from the
arrays it passes to and receives from the DLL; it never opens a `.RO.dbg`. So
the unit's only live consumer takes the result out of the process through a
channel the instrument does not read — corrupting it renames a file rather than
changing a number.

**This is a FIFTH distinct shape of P9**, after unexercised line (#1), argument
constant in every scenario (#3), result cancelled downstream (#4) and result
produced-but-never-consumed (#6). The new one: *the result IS consumed, and the
consumption is a side effect outside what the gate measures.* Coverage cannot
express it — `ROSCO_IO.f90:1102` is hit in every scenario that logs.

## What DOES constrain it

726 differential cases against the clean Fortran, 0 failed, and a mutation score
of 1.000 with two declared equivalents whose reasons are in
`../../mutation/GetRoot.equivalences.md`.

**Getting there took three fixes to the harness generator, and the first run's
green was worthless.** The first harness reported `224 checked, 0 failed` while
never once executing the branch the procedure exists for. The mutation score,
not the harness verdict, is what exposed it: 25 survivors, six of them in
`RootName = GivenFil(:I-1)` under `INDEX('\/', ...) == 0`.

| gap | why the generator could not reach it | after |
|---|---|---|
| R6's literal miner reads **single-character** literals only, so it never saw the SET `'\/'` — the corpus contained **no backslash at all** | a character SET is one multi-character literal | mine every character of every literal passed to `INDEX`/`SCAN`/`VERIFY` |
| `char_corpus` puts each literal beside its own collating neighbours and the mixed shape lays them down in corpus order, so **every** string containing `'.'` had `'/'` — a separator — directly after it | `'/'` is `'.'`+1 in ASCII | `_planted_chars` / `_planted_pair` / `_blank_tail` |
| the length ladder was `{1, 4, 9}`; at length 1 the first character is also the last, and at 4 both sides of an `I == 1` test are false together | no length 2 | ladder `{1, 2, 4, 9}` |

Measured end to end: **224 cases / 0.648 → 726 cases / 0.968**, the rest by
removing a restatement (below). Fixed in the loop repo (`0e92a72`), not worked
around. `harness/GetRoot.json`'s `rule_coverage` states the new counts.

## The translation departs from literal transcription once, with a proof

**There is no `len_trim` helper.** The reference names `LEN_TRIM(GivenFil)` at
two sites and this translation uses `len_GivenFil` at both. The three-part proof
is in `translations/ROSCO_Helpers/getroot.cpp`; in short, (1) `TRIM(X) == "."`
and `X == "."` are the same predicate because Fortran's comparison blank-pads,
(2) the extra positions the wider scan visits are blanks and a blank is not a
`'.'`, and (3) the two forms of `I < ...` differ only at `I == LEN_TRIM < LEN`,
where the character at `I+1` is a blank so both paths write `GivenFil(:I-1)`.

Transcribed literally it scored 0.886 with **six of eight survivors inside the
helper** (`mutation/GetRoot.survivors_len_trim.json`) — the same shape unit #4
measured on the same intrinsic in `Conv2UC`, now with a second independent
measurement. Three of those six (`s[i+1]`, `s[1-i]`, `s[i-2]`) are
**out-of-bounds reads**: a mutant whose behaviour is undefined cannot honestly be
declared equivalent, so deleting the site is the only move that is both true and
closes it.

Two smaller things, both precedented at unit #6:

* the separator set is `static const char Separators[] = { '\\', '/' }` with
  `sizeof`, not the string literal `"\\/"` with a `2` beside it — the digit is a
  mutable site restating what the array fixes, and its mutant reads the NUL
  terminator a C string literal carries and this Fortran set does not;
* `char_assign`'s two loops are 1-based, for the reason unit #6 measured.

## Two KGen defects, both fixed in KGen (X2)

Extraction succeeded, `vit verify` built the kernel, and it compared **nothing**:

```
✗ VERIFICATION FAILED: 62/62 passed
  FAILED: kernel compared 0 output variables — nothing was verified
```

`kgen_callsite.aliased_args_no_verify.F90` is that kernel, with an empty
`!local verify variables` section.

`update_state_info` promotes a call-site variable to STATE_OUT by finding its
position in the actual-argument list and reading the matching dummy's INTENT. It
used `arglist.items.index(argobj)`, and `Fortran2003.Base.__cmp__` compares nodes
by CONTENT — so two occurrences of one name compare equal and `list.index`
returns 0 for both. Measured, not read:

```
Actual_Arg_Spec_List('RootName, RootName')  ->  index(items[1]) == 0
Actual_Arg_Spec_List('A, B')                ->  index(items[1]) == 1
```

Argument 0 is `GivenFil`, INTENT(IN), so `rootname` stayed an input. **The
dangerous version of this is a call site with a second out-argument**: the kernel
would have compared that one, dropped this one, and said nothing.

Fixing it exposed a second defect one file over. `get_typedecl_subpname` builds a
procedure name out of the declaration's selector, and `RootName` is declared
`CHARACTER(LEN=size(avcoutname))`, so the verify subroutine came out as

```fortran
RECURSIVE SUBROUTINE kv_discon_character_size(avcoutname)_(...)
```

— parentheses in an identifier. `c839e1a` had already fixed exactly this on the
GENCORE side; the VERIFICATION side has the same two lines and never got it,
because no unit had generated a verify subroutine for such a type before. The
sanitiser now lives in `kgutils` and both generators import it.

Red-tested in both directions (KGen `4457cd2`):

* **red** — the pre-fix artifacts above, and the post-fix kernel with
  `kgenref_rootname` read and `kv_discon_character_size_avcoutname__` generated;
* **green** — re-extracted `GetPath` (`ReadSetParameters.f90:331`, distinct
  arguments) with the patch stashed and unstashed. The two generated kernels are
  **byte-identical**: `kgen_control.GetPath-kernel-WITH-fix.f90` and
  `kgen_control.GetPath-kernel-WITHOUT-fix.f90`.

## A red test that stayed green because the perturbation never reached the binary

The post-integration harness links the campaign's **prebuilt** Fortran objects.
Editing `ROSCO_Helpers.f90` and re-running `harness.sh --post-integration`
therefore measures the OLD wrapper, and the first attempt at the wrapper red test
reported `checked 726 failed 0` — indistinguishable from a harness that cannot
fail. Rebuilding the controller between the edit and the run turns it red,
596 of 726. The artifact's `red_test.perturbation` says so.

## `vit check`'s two findings both belong to other procedures — third sighting

`narrowing-local` on `ExpUCVarName` (clean line 1051, inside `FindLine`) and
`delimiter-set` on `':/'` (clean line 1332, inside `PathIsRelative`). Neither is
in `GetRoot`'s line range 1253–1306. Re-attributed by hand, as RUNBOOK says to.

It flagged a third finding that WAS about this unit and was a false positive:
`delimiter-set` on `'\/'`, because the check read only string literals and this
translation spells the set as a character array — the form the campaign's own
"name a size once" rule asks for. Fixed in VIT (`87a3847`) rather than worked
around by reverting the translation.

## Files

| file | what it is |
|---|---|
| `vit_interface.stdout.txt` | the wrapper VIT emits, read attribute by attribute before any C++ was written |
| `vit_translate.stdout.txt` | the scaffolding prompt |
| `getroot.scaffold.cpp` | the scaffold, unedited |
| `getroot.final.cpp` | the translation as committed (copy of `translations/ROSCO_Helpers/getroot.cpp`) |
| `vit_check.stdout.txt` | 2 findings, BOTH belonging to other procedures in the file |
| `vit_verify.stdout.txt` | the kernel run, including VIT's own `NON_DISCRIMINATING` verdict |
| `kernel.verify_fields.csv` | 62 cases, `IDENTICAL` |
| `kernel.noop-stub.verify_fields.csv` | **the same 62 against a no-op: `IDENTICAL`** |
| `kernel.constant-stub-PASSES.verify_fields.csv` | the same 62 against a lookup table: `IDENTICAL` |
| `kernel.wrong-constant-stub.verify_fields.csv` | the same 62 against a wrong answer: `OUT_TOL` — the instrument is alive |
| `getroot.{noop,constant,wrong-constant}-stub.cpp` | the three stubs |
| `kernel.GetRoot.0.0.1.statefile` | the captured case: `vit_sim1vit_sim1` |
| `kernel-generated-DISCON.F90` | KGen's generated comparison AFTER the fix |
| `kernel-generated-kernel_driver.f90` | the driver |
| `kgen_callsite.aliased_args_no_verify.F90` | the callsite file BEFORE the fix — empty verify section |
| `kgen_driver.aliased_args_no_verify.f90` | its driver |
| `kgen_control.GetPath-kernel-WITH{,OUT}-fix.f90` | the green-direction control, byte-identical |
| `gate_redtests/*.json` | the two gate perturbations, each moving 0 values |
