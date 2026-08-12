// STUB — NOT the translation. Evidence input for unit #20.
//
// The unit as a NO-OP: reads nothing, writes nothing. Both outputs
// (LocalVar%PC_State, LocalVar%VS_State) are fields of the INTENT(INOUT)
// argument, so they arrive carrying the PREVIOUS call's value. This is unit
// #7's aliasing shape: where the state does not change between consecutive
// timesteps, "write nothing" and "write the right answer" are the same bytes,
// and a no-op stops being a liveness test.
#include "vit_types.h"

void StateMachine(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar) {
    (void)CntrPar;
    (void)LocalVar;
}
