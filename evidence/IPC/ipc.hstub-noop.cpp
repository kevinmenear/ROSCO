// RED-TEST STUB, not a translation. `IPC` as a NO-OP.
//
// The whole unit does nothing: no Coleman transform, no sigma, no PIController,
// no LPFilter, no loop, no write.
//
// WHAT THE NUMBER THIS PRODUCES IS FOR: it is the count the green in
// harness/IPC.json is a statement about. A corpus this no-op PASSES is a corpus
// that never reaches the unit.
//
// THE PREDICTION, WRITTEN BEFORE THE RUN: EVERY CASE FAILS, and the reason is
// two statements that have no guard above them at all --
//
//     CALL ColemanTransform(LocalVar%rootMOOPF, LocalVar%Azimuth, NP_1, &
//                           LocalVar%axisTilt_1P, LocalVar%axisYaw_1P)
//     CALL ColemanTransform(..., NP_2, LocalVar%axisTilt_2P, LocalVar%axisYaw_2P)
//
// -- the first two lines of the body, writing four `LocalVariables` scalars on
// every invocation whatever any input holds. `ColemanTransform` computes
// `axTOut = 2/NumBl * SUM(rootMOOP(K) * cos(...))` over three blades, so the
// only way its four outputs can equal what they arrived as is a coincidence of
// the draw, not a configuration.
//
// The corollary matters more than the prediction: this stub CANNOT distinguish
// the six arms below those two calls. A pass set, if there is one, is a
// statement about the Coleman outputs and nothing else, so the arm-level red
// tests are the mutation sweep's job and not this one's. Read the run's printed
// out-parameter list before explaining any pass.
//
// THE CALLEE CALLS ARE DELIBERATE AND ARE NOT DEAD CODE. Unit #45 measured that
// `vit/test_validate.generate_callee_bridges` decides which bridges to emit by
// running `_CALLEE_CALL.finditer` over the .cpp TEXT, and the generated
// Makefile compiles and links `<stem>_callees.o` only when at least one bridge
// was emitted. On the CLEAN tree `scripts/harness.sh` step 2e keeps every
// bridge and drops the corresponding `<callee>.cpp.o` from LIBS, so a stub that
// mentions no callee changes the harness's own LINK -- and a red test that
// fails to BUILD is not a red test (unit #42).
//
// ALL SIX OF THIS UNIT'S CALLEES ARE INTEGRATED, so all six are in that trade
// and all six are named below. The guard is unsatisfiable on this corpus and on
// any corpus: `LocalVar` is always a real object here, so `LocalVar == nullptr`
// is never true, and the block never executes. Its only job is to be TEXT the
// bridge generator can see.
//
// DO NOT DELETE IT AS DEAD CODE.

#include "vit_types.h"

void IPC(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
         objectinstances_t* objInst, debugvariables_t* DebugVar,
         errorvariables_view_t* ErrVar) {
    if (LocalVar == nullptr) {
        // Unreachable. See the header: this exists so that the six callee
        // bridges are emitted and the harness links.
        double tmp[3];
        colemantransform_c(LocalVar->rootMOOPF, LocalVar->Azimuth, 1,
                           &LocalVar->axisTilt_1P, &LocalVar->axisYaw_1P);
        colemantransforminverse_c(LocalVar->IPC_AxisTilt_1P,
                                  LocalVar->IPC_AxisYaw_1P, LocalVar->Azimuth,
                                  1, CntrPar->IPC_aziOffset[0], tmp);
        LocalVar->IPC_KP[0] = sigma_c(LocalVar->WE_Vw, CntrPar->IPC_Vramp[0],
                                      CntrPar->IPC_Vramp[1], 0.0,
                                      CntrPar->IPC_KP[0], ErrVar);
        LocalVar->IPC_IntSat = wrap_360_c(LocalVar->NacHeading);
        LocalVar->IPC_PitComF[0] = lpfilter_c(
            tmp[0], LocalVar->DT, CntrPar->IPC_CornerFreqAct, &LocalVar->FP,
            LocalVar->iStatus, LocalVar->restart ? 1 : 0, &objInst->instLPF, 0, 0.0);
        LocalVar->IPC_PitComF[1] = picontroller_c(
            tmp[1], LocalVar->IPC_KP[0], LocalVar->IPC_KI[0],
            -LocalVar->IPC_IntSat, LocalVar->IPC_IntSat, LocalVar->DT, 0.0,
            &LocalVar->piP, LocalVar->restart ? 1 : 0, &objInst->instPI);
    }
    (void)CntrPar;
    (void)LocalVar;
    (void)objInst;
    (void)DebugVar;
    (void)ErrVar;
}
