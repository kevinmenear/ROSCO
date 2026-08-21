// RED TEST FOR THE DIFFERENTIAL HARNESS -- unit #65, WindSpeedEstimator.
//
// The unit as a NO-OP: every argument taken, nothing written. If the harness
// cannot tell this from the translation, its green says nothing.
//
// THE PREDICTION, WRITTEN BEFORE THE RUN THAT TESTS IT: 13,868 of 13,868.
// Four outputs are assigned on EVERY path through this subroutine, with no
// enclosing IF --
//
//     LocalVar%WE_Op_Last, LocalVar%RestartWSE, LocalVar%HorWindV_F,
//     DebugVar%WE_b / WE_w / WE_t, DebugVar%WE_Vw
//
// -- and the corpus draws each of them independently of the inputs they are
// computed from. A case can only pass if the value already sitting in every one
// of the 213 compared out-parameters happens to be the value the unit would
// write, so a shortfall is not noise: it NAMES the cases where that held, and
// it should be read rather than absorbed.
//
// THE GUARDED CALLEE CALLS BELOW ARE LOAD-BEARING AND ARE NOT DEAD CODE.
// Unit #45's rule: `vit/test_validate.generate_callee_bridges` finds callees by
// running `_CALLEE_CALL.finditer` over the .cpp TEXT, and the Makefile compiles
// and links `<stem>_callees.o` only if a bridge was generated. A stub that
// calls nothing gets no bridge, and the link then dies on symbols OTHER
// integrated units need. `vit_red_test_never` is a `volatile` read of a zero,
// so no case reaches the block and the compiler cannot fold the calls away.

#include "vit_types.h"

#include <cfloat>
#include <cmath>

namespace {
volatile int vit_red_test_never = 0;
}  // namespace

void WindSpeedEstimator(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
                        objectinstances_t* objInst, performancedata_view_t* PerfData,
                        debugvariables_t* DebugVar, errorvariables_view_t* ErrVar) {
    if (vit_red_test_never) {
        // Unreachable. Present so that every one of this unit's six callee
        // bridges is still generated and linked -- see the header.
        double I3[9];
        identity_c(3, I3);
        LocalVar->WE_Vw =
            saturate_c(LocalVar->BlPitchCMeas, CntrPar->PC_MinPit, 0.0) +
            lpfilter_c(LocalVar->HorWindV, LocalVar->DT, CntrPar->F_WECornerFreq, &LocalVar->FP,
                       LocalVar->iStatus, 0, &objInst->instLPF, 0, 0.0) +
            aerodyntorque_c(LocalVar->RotSpeedF, LocalVar->BlPitchCMeas, LocalVar, CntrPar,
                            PerfData, ErrVar) +
            interp1d_c(CntrPar->WE_FOPoles_v, CntrPar->n_WE_FOPoles_v, CntrPar->WE_FOPoles,
                       CntrPar->n_WE_FOPoles, 0.0, ErrVar) +
            interp2d_c(PerfData->Beta_vec, PerfData->n_Beta_vec, PerfData->TSR_vec,
                       PerfData->n_TSR_vec, PerfData->Cp_mat, PerfData->n_Cp_mat_rows,
                       PerfData->n_Cp_mat_cols, 0.0, 0.0, ErrVar) +
            I3[0];
    }
}
