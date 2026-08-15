#!/usr/bin/env python3.12
"""Delete the ErrorVariables scalar copy-back from THIS UNIT'S wrapper, and only it.

    python3.12 evidence/interp2d/scope_wrapper_perturb.py <the .f90>

THE SCOPING IS THE POINT, and it is unit #41's finding, now on its fourth unit.
`CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)` stands FOUR times
in `Functions.f90` as this unit lands -- once per integrated unit whose view
argument carries scalar outputs -- so a whole-file replace perturbs all four and
the harness attributes the failure to whichever wrapper it happens to be
testing. Unit #41 ran that version, asserted on a count, edited nothing, and got
`POST-INTEGRATION RED TEST FAILED (stayed green)` against an UNPERTURBED tree.

WHY THIS LINE FOR THIS UNIT. `interp2d` has one output that is not its return
value, and it travels through the ErrorVariables view: `ErrVar%aviFAIL` and
`ErrVar%ErrMsg`, written on four error paths and in the RoutineName prefix.
`aviFAIL` is a scalar and reaches the caller ONLY through this line; `ErrMsg`
crosses through the view's staging buffer and is reallocated onto the real field
by the same call. Deleting it is exactly the wrapper `vit integrate` emits
WITHOUT `--reverse-copy`, which unit #23 measured one level down on `interp1d`
and which the kernel and the gate both passed.

Exits non-zero rather than editing if the wrapper or the line is not where it is
expected, because a perturbation that silently does nothing is the same failure
one step quieter.
"""
import re
import sys

path = sys.argv[1]
src = open(path).read()
m = re.search(r"    FUNCTION interp2d\(.*?    END FUNCTION interp2d", src, re.S)
if m is None:
    sys.exit("scope_wrapper_perturb: interp2d's wrapper is not in " + path)
body = m.group(0)

LINE = "        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)\n"
if body.count(LINE) != 1:
    sys.exit("scope_wrapper_perturb: expected exactly one copy-back in interp2d's "
             "wrapper, found %d. The wrapper is not the one this test was written "
             "against." % body.count(LINE))

whole = src.count(LINE)
new = src.replace(body, body.replace(LINE, ""), 1)
open(path, "w").write(new)
print("scope_wrapper_perturb: removed 1 of %d copy-back(s) in %s, scoped to "
      "interp2d's wrapper" % (whole, path))
