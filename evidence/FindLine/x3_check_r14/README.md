# X3 check for R14, and it is a byte comparison rather than a case count

R14 (`translation-loop` `552edb1`) is the fifth shared-instrument change unit
#32 has made, and the one that changes the CORPUS. X3 says a verification
default is not changed mid-run; the campaign's reading of that, settled at unit
#30, is that a generator change is allowed when its effect on every
already-scored unit is MEASURED rather than argued.

The previous x3 check (`../x3_check/README.md`) measured a case COUNT. This one
measures the case BYTES, because R14's whole claim is that it appends:

```
                 R14 off        R14 on      first N bytes identical
ChkParseData   102,577 B     110,533 B      yes        1552 -> 1624 cases
GetWords        56,701 B      56,701 B      yes        1373 -> 1373  (N/A)
FindLine     4,990,604 B   5,370,716 B      yes        2370 -> 2514 cases
```

A prefix that is identical byte-for-byte is the strongest available statement
that no case index an already-scored unit was measured on has moved. The case
count alone cannot say that: two corpora of 1552 cases can differ in every one.

**The R14-off column reproduces the committed numbers exactly** — `ChkParseData`
1552 (`../x3_check/ChkParseData.harness.json`) and `FindLine` 2370
(`harness/FindLine.json` before this dispatch). So the ablation is a real
ablation and not a differently-broken generator.

## Who has the shape, and who does not

R14 needs a CHARACTER **array** input and a free scalar CHARACTER input. Two of
this campaign's 32 units have both:

* `FindLine` — `FileLines(:)` and `ParamName`. This dispatch's unit.
* `ChkParseData` — `Words(2)`, `ExpVarName`, `FileName`. **+72 cases, still
  1624 checked / 0 failed**, so the translation already agrees with the
  reference on inputs it had never been given.

**That "two of 32" is checked rather than recalled.** Every non-output CHARACTER
ARRAY dummy in the clean sources, by procedure:

```bash
git show 54dd134:rosco/controller/src/ROSCO_Helpers.f90 | \
    grep -nE 'CHARACTER.*INTENT *\( *IN' | grep -E '\(\s*[:0-9]|DIMENSION'
```

Among the 32 units with a translation, exactly `ChkParseData` and `FindLine`
have one. (`Conv2UC`'s `Str` and `Debug`'s `RootName` are scalars whose comments
contain parentheses, which is what a looser grep reports as hits.) Seven
procedures NOT yet translated have the shape — `ParseDbAry_Opt`,
`ParseInAry_Opt`, the three `ParseInput_*_Opt`, `ReadControlParameterFileSub`,
`SetParameters` — and every one of them is a `FileLines(:)` or an `accINFILE`
parser, i.e. exactly the family this rule was written for. They inherit it for
free; that is a future benefit, not a future X3 cost.

`GetWords` is the near miss worth naming: it has `Words(NumWords)`, but the
array is `INTENT(OUT)`. There is no element for a scalar to be a word *of*, so
the rule reports N/A rather than firing — which is P6, and is asserted by
`test_R14_is_not_applicable_without_a_CHARACTER_ARRAY_and_says_so`.

## What is NOT done here, and why

`ChkParseData` is a closed unit. Its committed `harness/ChkParseData.json`
(1552) and `mutation/ChkParseData.json` no longer reproduce on today's
generator, and **72 new cases that its translation passes are 72 cases its
mutation score was not taken over**. That is a real finding about another unit
and it is left as one, exactly as unit #32's first x3 check left GetWords'
1370-vs-1373 drift: re-taking a closed unit's evidence inside this dispatch
would put a number in its artifact that no commit of its own explains. Raised
in `DECISIONS.md`.

## Reproduce

```bash
bash scripts/reset_to_clean.sh
bash evidence/FindLine/x3_check_r14/run.sh
bash scripts/restore_integrated.sh
```

The `.bin` files the comparison reads are ~10 MB and are NOT committed; `run.sh`
regenerates them and prints the table above.
