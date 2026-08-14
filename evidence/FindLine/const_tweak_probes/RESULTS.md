# Which `const_tweak '1' -> '2'` sites this corpus can see

`mutation/FindLine.json` names its survivors by operator, before and after, and
seven sites in findline.cpp match `'1' -> '2'`. Two of them survived and the
artifact does not say WHICH -- the id is content-derived and the mutator records
no line. Guessing would have put a claim in the evidence that nobody could
check, so each of the seven was built and run through the differential harness
separately. The `.cpp` beside this file is the exact perturbation in each case.

| site | what changes | cases failed of 2370 |
|---|---|---|
| `L125_wordind` | `WordInd = AryLen + 1` -> `AryLen + 2`: the name is looked for in the NEXT word | **0** |
| `L177_loop_start` | the search loop starts at line 2: the first line of the file is never searched | **0** |
| `L215_foundline_true` | `*FoundLine = 1` -> `2`: a true that is not the wrapper's true | **47** |
| `L67_loop_start` | `char_assign`'s loop starts at 2: byte 1 of every destination is left as it arrived | **32** |
| `L68_dst_index` | `dst[i - 1]` -> `dst[i - 2]`: the whole destination shifted down one | **47** |
| `L68_src_index` | `src[i - 1]` -> `src[i - 2]`: the whole source shifted down one | **47** |
| `L80_element_stride` | `element`'s `(j - 1)` -> `(j - 2)`: every array element addressed one early | **23** |

**The two survivors are `L125_wordind` and `L177_loop_start`, and neither is a
mystery once the match count is known.** `evidence/FindLine/harness.match-count-probe.json`
measures that exactly **47 of 2370 cases reach the match arm at all** -- the
probe writes `LineNum = -7` inside it, so its failing count IS that number.

* `L125` changes WHICH WORD carries the name, and that can only change an answer
  in a case where some word does. Its blindness is the same one
  `harness.arylen-ignored-stub.json` reports as 0 of 2370.
* `L177` skips the FIRST line of the file, which changes an answer only when a
  case's ONLY match is on line 1. Of the 47 matching cases, none is.

Note that four of the seven fail exactly 47, and `L67`/`L80` fail fewer -- 32 and
23 -- because those two disturb `Line` and `ParamNameUC` in ways that can also
DESTROY a match that would otherwise have happened, which subtracts from the
same 47.
