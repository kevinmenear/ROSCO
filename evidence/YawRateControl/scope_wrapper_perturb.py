#!/usr/bin/env python3.12
"""Delete the LocalVariables scalar copy-back from THIS UNIT'S wrapper, and only it.

    python3.12 evidence/YawRateControl/scope_wrapper_perturb.py <the .f90>

THE SCOPING IS THE POINT, and it is unit #41's finding restated one unit later.
`CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)` appears once
per integrated unit in Controllers.f90 whose view argument carries scalar
outputs -- six of them as this unit lands -- so a whole-file replace perturbs
every one and the harness attributes the failure to whichever wrapper it happens
to be testing. Unit #41 ran that version, asserted on a count, edited nothing,
and got `POST-INTEGRATION RED TEST FAILED (stayed green)` against an
UNPERTURBED tree: a verdict correct about what it measured and readable as a
defect in the wrapper.

WHY THIS LINE FOR THIS UNIT. Of everything `YawRateControl` writes, exactly one
output travels through the LocalVariables view BY VALUE:

    LocalVar%WindDir = wrap_180(LocalVar%NacHeading + LocalVar%NacVane)

`avrSWAP` is a bare pointer and is unaffected; `DebugVar` and `objInst` are
passed as `C_LOC` of the Fortran derived types themselves, with no view and no
copy-back, so the five DebugVar stores and the LPFilter instance counter reach
the caller whatever this line does. Deleting it therefore isolates `WindDir`,
and the count it produces is a statement about ONE field rather than about the
unit -- which is why it is expected to be much smaller than the no-op's 1012.

Exits non-zero rather than editing if the wrapper or the line is not where it is
expected, because a perturbation that silently does nothing is the same failure
one step quieter.
"""
import re
import sys

path = sys.argv[1]
src = open(path).read()
m = re.search(r"    SUBROUTINE YawRateControl\(.*?    END SUBROUTINE YawRateControl", src, re.S)
if m is None:
    sys.exit("scope_wrapper_perturb: YawRateControl's wrapper is not in " + path)
body = m.group(0)
line = "        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)"
if body.count(line) != 1:
    sys.exit(f"scope_wrapper_perturb: the copy-back line appears {body.count(line)} "
             f"time(s) in YawRateControl's wrapper; expected exactly 1")
whole = src.count(line)
perturbed = body.replace(line, "        ! VIT RED TEST: copy-back deleted")
open(path, "w").write(src.replace(body, perturbed))
print(f"scope_wrapper_perturb: deleted the copy-back from YawRateControl's wrapper "
      f"only; the same line stands {whole - 1} more time(s) elsewhere in {path}")
