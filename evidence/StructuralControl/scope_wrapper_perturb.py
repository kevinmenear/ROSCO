#!/usr/bin/env python3.12
"""Delete the scalar copy-back from THIS UNIT'S wrapper, and only from it.

    python3.12 evidence/StructuralControl/scope_wrapper_perturb.py <the .f90>

THE SCOPING IS THE POINT. `CALL vit_copy_scalars_to_localvariables(LocalVar_view,
LocalVar)` appears more than once in Controllers.f90 -- every integrated unit in
the module whose view argument carries scalar outputs writes the same call -- so
a whole-file replace perturbs several wrappers and the harness attributes the
failure to whichever one it happens to be testing. Unit #41 ran the first version
of this red test as a whole-file replace, asserted on a count and edited nothing,
and the harness then reported "RED TEST FAILED (stayed green)" against an
UNPERTURBED tree. That verdict was correct about what it measured and would have
been read as a defect in the wrapper.

WHY THIS LINE FOR THIS UNIT. `StructuralControl`'s principal output is
`LocalVar%StC_Input`, a FIXED-SIZE `REAL(DbKi) :: StC_Input(12)` field, which the
view carries BY VALUE (`double StC_Input[12]` in vit_types.h) rather than as a
C_LOC'd pointer. Deleting the copy-back therefore discards every write the
`StC_Mode == 1` step arm and the `StC_Mode == 2` open-loop arm make. `avrSWAP`
does travel by pointer and is unaffected, so what stays green under this
perturbation is exactly the copy loop's effect on `avrSWAP` and what goes red is
`StC_Input` -- which is why the count below is smaller than the no-op's.

Exits non-zero rather than editing if the wrapper or the line is not where it is
expected, because a perturbation that silently does nothing is the same failure
one step quieter.
"""
import re
import sys

path = sys.argv[1]
src = open(path).read()
m = re.search(r"    SUBROUTINE StructuralControl\(.*?    END SUBROUTINE StructuralControl", src, re.S)
if m is None:
    sys.exit("scope_wrapper_perturb: StructuralControl's wrapper is not in " + path)
body = m.group(0)
line = "        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)"
if body.count(line) != 1:
    sys.exit(f"scope_wrapper_perturb: the copy-back line appears {body.count(line)} "
             f"time(s) inside StructuralControl's wrapper; expected exactly 1")
open(path, "w").write(
    src[:m.start()]
    + body.replace(line, "        ! RED TEST: the scalar copy-back deleted")
    + src[m.end():])
print("scope_wrapper_perturb: deleted the copy-back from StructuralControl's wrapper only")
