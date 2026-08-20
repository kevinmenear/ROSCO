# The 52 mutation survivors, and the answer each one gets

`mutation/ReadControlParameterFileSub.json` reports 155 of 155 scored killed.
The denominator is 207 behavioural mutants less 17 declared equivalent and 35
declared unreachable, so the score is only as good as these 52 arguments. They
are written here, next to the line each mutant sits on, so they can be
DISPUTED -- which is the whole difference between explaining a survivor and
excusing one.

The two dispositions are not interchangeable:

  EQUIVALENT   a claim about the PROGRAMS. The mutant and the original agree
               on every admissible input, so no corpus could ever kill it.
  UNREACHABLE  a claim about the CORPUS. The difference is real and this
               corpus cannot reach it. Each carries an evidence path, and
               `corpus_facts.py` additionally names the file that would
               refute it -- an unreachable set with one common cause is a
               corpus gap wearing a declaration's name.

Measured, not asserted: `site_census.py` builds the translation with a counter
on each site and runs the 63-case corpus, reporting EVALUATIONS and TRUE
outcomes separately (`site_census.txt`). `corpus_facts.py` measures the four
claims that are about the .IN files rather than the run (`corpus_facts.txt`).

## EQUIVALENT (17)

**9e2c7da0**  line 126  `compare_op`  `<` -> `<=`
> `const int n = static_cast<int>(src.size()) < width`

`size < width ? size : width` and `size <= width ? size : width` yield the SAME value at equality (both give width), so no input can separate them. Census site 17 shows the corpus never reaches equality either, which is corroboration and not the argument.

**f61b02eb**  line 128  `compare_op`  `>` -> `>=`
> `if (n > 0) std::memcpy(dst, src.data(), static_cast<size_t>(n));`

at n == 0 the mutant memcpys 0 bytes. 1,575 of 15,594 fassign calls have n == 0 (census sites 3 and 16), UBSan is silent over all of them under the sanitised sweep, so `src.data()` is non-null and a zero-length memcpy is a no-op.

**305ab529**  line 129  `negate_cond`  `if (n < width)` -> `if (!(n < width))`
> `if (n < width) std::memset(dst + n, ' ', static_cast<size_t>(width - n));`

the blank fill is a RESTATEMENT of both callers' initialisation, and the two callers are the whole program: FileLines is `std::vector<char>(n*width, ' ')` -- already the byte the memset writes -- and EchoFilename is `std::vector<char>(128)`, zero-filled, whose only consumer is `fopen(echo_path.c_str())` and a C string ends at the first NUL. Flagged as the campaign's known shape: a translation's score rises by REMOVING restatements. Removing this one would change the translation and re-take every layer, so it is declared rather than deleted.

**492b8a91**  line 129  `compare_op`  `<` -> `<=`
> `if (n < width) std::memset(dst + n, ' ', static_cast<size_t>(width - n));`

at n == width the mutant memsets `width - n` = 0 bytes, which is a no-op; below it the two agree. Nothing separates them on any input.

**dd1c697f**  line 192  `const_tweak`  `1` -> `2`
> `CFI_CDESC_T(1) d;`

`CFI_CDESC_T(1)` -> `(2)` enlarges the descriptor's `dim` array. CFI_establish sets rank 1 and every read and write in this file is `dim[0]`; the extra element is storage nothing addresses.

**47d1c4b8**  line 300  `const_tweak`  `1` -> `2`
> `const int NumLines = static_cast<int>(records.size()) + 1;`

`records.size() + 1` -> `+ 2` gives the callees one more FileLines row than the reference has. Every row is blank-filled and the callees search by VarName, so a blank row matches nothing; the corpus's three parse-error cases show the callee messages do not carry the line count either.

**11a7f5a3**  line 365  `const_tweak`  `0` -> `1`
> `parse_int("Echo", &CntrPar->Echo, 0, 0, 0, 0);`

the UnEc VALUE at the Echo parse -- dead twice over: has_UnEc is 0, and the callee discards the argument anyway.

**4b852d50**  line 365  `const_tweak`  `0` -> `1`
> `parse_int("Echo", &CntrPar->Echo, 0, 0, 0, 0);`

has_UnEc at the Echo parse. UnEc is discarded by the callee (parseinput_int_opt.cpp:590), so present-or-absent cannot be distinguished.

**7f31af91**  line 365  `const_tweak`  `0` -> `1`
> `parse_int("Echo", &CntrPar->Echo, 0, 0, 0, 0);`

the AllowDefault VALUE at the Echo parse, with has_AllowDefault = 0 beside it. The bridge does not pass a value it is told is absent, so the literal is dead.

**dd68b590**  line 377  `const_tweak`  `0` -> `1`
> `int UnEc = 0;`

`int UnEc = 0` -> `1`, same argument from the other side: the initial value is replaced by 10 on the echo arm, and on every other path the only consumers are the five callees that discard it and a `UnEc > 0` guard that is anded with `ec != nullptr`.

**e185a05b**  line 388  `const_tweak`  `10` -> `11`
> `UnEc = 10;   // GetNewUnit's StartUnit; see above for why this is here`

`UnEc = 10` -> `11`. UnEc is modelled as a flag: this unit tests it only as `UnEc > 0` (line 711, and that guarded by `ec != nullptr`), and all five Parse*_Opt callees discard it -- `(void)UnEc` at translations/ROSCO_Helpers/parseinput_int_opt.cpp:590, a divergence those units measured and recorded. No value above 0 differs from another.

**4736a555**  line 400  `index_offset`  `[8]` -> `[8 + 1]`
> `char date[11], time[8];`

`char time[8]` -> `[9]`, same argument: vit_curtime_c writes 8, consumers read 8.

**4e6bad47**  line 400  `const_tweak`  `11` -> `12`
> `char date[11], time[8];`

the same declaration through the const_tweak operator rather than index_offset -- same argument.

**832c8c42**  line 400  `const_tweak`  `8` -> `9`
> `char date[11], time[8];`

the same declaration through const_tweak -- same argument.

**b9007d3d**  line 400  `index_offset`  `[11]` -> `[11 + 1]`
> `char date[11], time[8];`

`char date[11]` -> `[12]` enlarges a stack buffer. vit_curdate_c writes exactly 11 bytes and every consumer reads exactly 11; the 12th byte is never read.

**0a761c52**  line 846  `index_offset`  `[11]` -> `[11 + 1]`
> `char n11[11];`

`char n11[11]` -> `[12]` in the cable-control loop. int2lstr_c writes 11 bytes and `ftrim(n11, 11)` reads 11.

**6162a812**  line 862  `index_offset`  `[11]` -> `[11 + 1]`
> `char n11[11];`

the same declaration in the structural-control loop -- same argument.

## UNREACHABLE (35)

**95fe8ae1 (line 101)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

MaxLineLength 2048 -> 2049 changes the width FileLines truncates a record to. The corpus's longest record is 875 bytes over 15,609 records (corpus widths block), so no record reaches either width and the two programs see the same FileLines.

**8d2cf1af (line 112)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

PriPath_LEN 1024 -> 1025 changes where GetPath's result truncates. The longest input path the probe passes is 67 characters (corpus widths block).

**731a6a07 (line 113)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

EchoFilename_LEN 128 -> 129 changes where <RootName>.RO.echo truncates. The longest input path is 67 characters, so the echo name is at most 75 (corpus widths block).

**90f86b06 (line 114)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

OL_String_LEN 1024 -> 1025 changes where the open-loop channel string truncates. The longest OL_String the corpus produces is 57 characters -- the `ROSCO: Implementing open loop control for BldPitch1 BldPitch2 BldPitch3 GenTq YawRate Cable1 StC1` line the probe prints, 98 characters including its 41-character prefix.

**92bdcceb (line 119), 977ffe3c (line 119)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

ftrim's `n > 0` guard decides only for an ALL-BLANK argument, and site 15 counts ftrim calls that returned an empty view: 0 of 163. `n >= 0` would then read s[-1] and `n > 1` would return one blank instead of none; neither can happen on an input that never trims to empty.

**26c265e9 (line 128)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

fassign's `n > 0` and `n > 1` differ only for a source of exactly one character. Site 16 counts those: 0 of 15,594 calls.

**053fd9c4 (line 136), bac58b04 (line 136), 0f63c6b3 (line 143), dbc3dfcc (line 143), a516ae11 (line 153), f7aa6c27 (line 153), 0d556ac9 (line 154), 2ee34a73 (line 154), 38f4adf2 (line 163), 8c902400 (line 163), 9a3771b3 (line 163), bb4c75b6 (line 163), 4b9987ce (line 165), 86dc7559 (line 165)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

assign_errmsg and errmsg_trim are NOT CALLED by any of the 63 cases: census sites 5, 6, 7 and 8 all report 0 evaluations. The unit has exactly two callers of them -- the `Cannot open file <echo>` arm (site 13: 1 evaluation, 0 true) and the closing `RoutineName//':'//TRIM(ErrMsg)` block, which is reached only when aviFAIL is negative at the END and every path that sets it negative hits one of the unit's own early RETURNs first (the plan's RT6, independently confirmed by the probe red test's 36-of-37 count). Parse errors set ErrMsg inside the CALLEES, not here.

**4555590d (line 198), d1ff17ab (line 198), 695d8151 (line 201), 764bfb17 (line 201), b5f3c0b1 (line 201), 8005d02f (line 202), 805abaed (line 202), 49a0fabd (line 203), c15d4328 (line 203)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

The `if (base != nullptr)` arm of establish() -- the one that fills a CFI descriptor for an ALREADY-ALLOCATED array field -- is evaluated 3,183 times and is true 0 times (census site 9). The probe empties CntrPar between cases because the reference requires every ALLOCATABLE unallocated on entry (ParseAry reports `array was already allocated` otherwise), so no case can arrive with one allocated. All nine mutants are inside that arm.

**cad91b36 (line 261)**
> evidence: `evidence/ReadControlParameterFileSub/corpus_facts.txt`

std::_Exit(2) -> _Exit(3) is the abort the translation takes when fopen on the input file fails, matching the Fortran runtime's own message and status. The reference aborts the PROCESS on the same input, so a case that reached it would take the probe down and produce no comparison at all -- unreachable by the shape of the instrument, not by the file list. corpus_facts reports 0 corpus files that do not exist.

**bf876a89 (line 365)**
> evidence: `evidence/ReadControlParameterFileSub/corpus_facts.txt`

The literal is ParseInput's has_AllowDefault at the Echo call. Absent AllowDefault means ALLOWED (translations/ROSCO_Helpers/parseinput_int_opt.cpp:455), so passing an explicit .FALSE. makes Echo REQUIRED -- which decides only for a file with no Echo line, and corpus_facts counts 0 of 63.

**d3ed5ecb (line 398)**
> evidence: `evidence/ReadControlParameterFileSub/site_census.txt`

ErrVar%aviFAIL = 1 -> 2 sits in the `the echo file could not be opened` arm. Census site 13: 1 evaluation (echo1.IN), 0 true.

**fdfd4622 (line 474)**
> evidence: `evidence/ReadControlParameterFileSub/corpus_facts.txt`

`(VS_ControlMode < 2)` and `(VS_ControlMode <= 2)` differ only AT 2, and then only if F_VSRefSpdCornerFreq is missing. corpus_facts: 27 files have VS_ControlMode == 2 and all 27 carry the line.

**8a8c4e4d (line 478)**
> evidence: `evidence/ReadControlParameterFileSub/corpus_facts.txt`

`(F_GenSpdNotch_N == 0)` is evaluated only inside `IF (F_GenSpdNotch_N > 0)`, so it is FALSE wherever it runs and the mutant is TRUE -- allowed instead of required. It decides only for a file with N > 0 and no F_GenSpdNotch_Ind line; corpus_facts lists all 21 files with N > 0 and every one carries the line.
