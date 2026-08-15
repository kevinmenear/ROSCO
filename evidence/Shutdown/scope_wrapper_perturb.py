#!/usr/bin/env python3.12
"""Delete the scalar copy-back from THIS UNIT'S wrapper, and only from it.

    python3.12 evidence/Shutdown/scope_wrapper_perturb.py <the .f90>

THE SCOPING IS THE POINT. `CALL vit_copy_scalars_to_localvariables(LocalVar_view,
LocalVar)` appears FIVE times in ControllerBlocks.f90 -- every integrated unit in
the module whose view argument carries scalar outputs writes the same call -- so
a whole-file replace perturbs five wrappers and the harness attributes the
failure to whichever one it happens to be testing. The first run of the red test
did a whole-file replace, asserted on the count of 5 and edited nothing, and the
harness then reported "RED TEST FAILED (stayed green)" against an UNPERTURBED
tree. That verdict was correct about what it measured and would have been read
as a defect in the wrapper.

Exits non-zero rather than editing if the wrapper or the line is not where it is
expected, because a perturbation that silently does nothing is the same failure
one step quieter.
"""
import re
import sys

path = sys.argv[1]
src = open(path).read()
m = re.search(r"    SUBROUTINE Shutdown\(.*?    END SUBROUTINE Shutdown", src, re.S)
if m is None:
    sys.exit("scope_wrapper_perturb: Shutdown's wrapper is not in " + path)
body = m.group(0)
line = "        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)"
if body.count(line) != 1:
    sys.exit(f"scope_wrapper_perturb: the copy-back line appears {body.count(line)} "
             f"time(s) inside Shutdown's wrapper; expected exactly 1")
open(path, "w").write(
    src[:m.start()]
    + body.replace(line, "        ! RED TEST: the scalar copy-back deleted")
    + src[m.end():])
print("scope_wrapper_perturb: deleted the copy-back from Shutdown's wrapper only")
