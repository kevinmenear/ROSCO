// RED TEST STUB for CheckInputs -- NOT the translation.
//
// Reads no argument and writes a determinate, finite, wrong constant into the
// only scalar the unit can move. -7 is chosen against the CALL SITE's own
// state and not against the type (unit #25): SetParameters sets
// ErrVar%aviFAIL = 0 two dozen lines above the call and every check in
// CheckInputs writes -1, so both 0 and -1 are values a correct run can
// produce here and neither would discriminate.
#include "vit_types.h"

void CheckInputs(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
                 float* avrSWAP, errorvariables_view_t* ErrVar, int size_avcMSG) {
    (void)LocalVar; (void)CntrPar; (void)avrSWAP; (void)size_avcMSG;
    ErrVar->aviFAIL = -7;
}
