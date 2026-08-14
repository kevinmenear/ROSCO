# X3 check for unit #32's four generator changes

Unit #32 changed two shared instruments (vit `c4eb0ad`, `d2de28c`; loop
`9eeaf3f`, `024982b`), and each commit message argues from the SOURCE that no
already-scored unit's corpus can move. An argument is not a measurement, and
this campaign's own rule is that the two are not the same thing — so the two
units that could plausibly move were re-taken.

They are the only two: the changes touch a CHARACTER **array** dummy, a
CHARACTER dummy whose fixed width is a **name** rather than a literal, a scalar
`LOGICAL, INTENT(OUT)`, and the `vit check` Fortran scope. `ChkParseData` has a
CHARACTER array (`Words(2)`, explicit shape) and a `CHARACTER(20)` narrowing
local; `GetWords` has a CHARACTER array whose count is another dummy
(`Words(NumWords)`). No other scored unit has any of the shapes.

```
                committed        re-taken, my HEAD
ChkParseData    1552 / 0         1552 / 0        unchanged
GetWords        1370 / 0         1373 / 0        +3 cases
```

**The three cases are NOT this dispatch's**, and attributing them took one more
run rather than one more argument: with the loop repo checked out at `12dbaa0`
— the revision it stood at when this dispatch opened, before any of the four
changes — GetWords generates **1373** cases as well.

```
GetWords, loop at 12dbaa0 (this dispatch's starting point)   1373 / 0
GetWords, loop at 024982b (this dispatch's HEAD)             1373 / 0
```

So the corpus is identical across every change made here, and the +3 is drift
that accumulated between GetWords' own dispatch (`loop_rev 5b40e1c`, in
`harness/GetWords.json`) and the start of this one.

**THAT IS A REAL FINDING ABOUT ANOTHER UNIT AND IT IS LEFT AS ONE.**
`harness/GetWords.json` records 1370 cases and `mutation/GetWords.json` records
57 of 58 at 1.000 against that corpus; neither reproduces on today's loop. It is
not repaired here — re-taking a closed unit's evidence is that unit's business,
and doing it inside this dispatch would put a number in its artifact that no
commit of its own explains. Raised in DECISIONS.md.

Reproduce:

```bash
bash scripts/reset_to_clean.sh
bash scripts/harness.sh ChkParseData ROSCO_Helpers chkparsedata \
     rosco/controller/src/ROSCO_Helpers.f90 --against translation \
     --out evidence/FindLine/x3_check/ChkParseData.harness.json
bash scripts/harness.sh GetWords ROSCO_Helpers getwords \
     rosco/controller/src/ROSCO_Helpers.f90 --against translation \
     --out evidence/FindLine/x3_check/GetWords.harness.json
# and for the attribution:
git -C ../translation-loop checkout 12dbaa0 -- harness/ scripts/
bash scripts/harness.sh GetWords ... --out .../GetWords.harness.at-12dbaa0.json
git -C ../translation-loop checkout HEAD -- harness/ scripts/
bash scripts/restore_integrated.sh
```

NOTE ON THE `loop_rev` STAMP in these three artifacts: it reads `024982b` in all
of them, including the one produced with `12dbaa0`'s code, because
`_harness_stamp.py` reads the repository's HEAD rather than the content of the
files it ran. The measurement here is the CASE COUNT, which is a property of
what actually ran; the stamp is not evidence for this particular question and
should not be read as if it were.
