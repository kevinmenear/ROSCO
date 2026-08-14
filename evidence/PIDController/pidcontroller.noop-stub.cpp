// RED TEST: the whole unit as a NO-OP. Reads no argument, writes no output,
// returns a constant. If the differential harness's green means anything, every
// case must fail -- and the mismatch list must NAME the outputs this unit
// writes, not merely count them.
//
// WHY THE DEAD `if (false)` BLOCK IS NOT DECORATION. `vit test-validate` derives
// the callee-bridge set from the TEXT OF THE C++ TRANSLATION
// (`generate_callee_bridges(Path(args.cpp_file).read_text(), ...)`, vit/cli.py),
// and passes `callee_bridge=bool(callee_src)` to the Makefile generator. A stub
// with no `_c(` call therefore produces a Makefile that does not link
// `pidcontroller_callees.o` -- and the link dies on a symbol that has nothing to
// do with this unit:
//
//   picontroller.cpp.o: in function `PIController(...)':
//     undefined reference to `saturate_c'
//
// because `reset_to_clean.sh` leaves every earlier unit's `.cpp.o` in the build
// tree and `vit test-validate` globs them into LIBS. So a bare no-op stub does
// not measure a weaker instrument, it measures a DIFFERENT one, and unit #26's
// rule is that a red result and the green it certifies must name the same
// instrument. Naming both callees keeps the generated bridge set, the Makefile
// and the link identical to the green run; `if (false)` keeps them unreachable,
// so nothing here can contribute to an output.
#include "vit_types.h"

double PIDController(double error, double kp, double ki, double kd, double tf, double minValue, double maxValue, double DT, double I0, piparams_t* piP, int32_t reset, objectinstances_t* objInst, localvariables_view_t* LocalVar) {
    if (false) {
        (void)saturate_c(0.0, 0.0, 0.0);
        (void)lpfilter_c(0.0, 0.0, 0.0, &LocalVar->FP, 0, 0, &objInst->instLPF, 0, 0.0);
    }
    return 0.0;
}
