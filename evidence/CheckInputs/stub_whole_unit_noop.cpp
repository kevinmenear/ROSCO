// RED TEST STUB for CheckInputs -- NOT the translation.
//
// The whole body deleted. Every one of the ~180 checks is gone; the unit
// becomes a no-op on its arguments. What this measures is not the translation
// but the CAPTURE: any case in which the reference itself raised no error is a
// case this stub reproduces exactly.
#include "vit_types.h"

void CheckInputs(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
                 float* avrSWAP, errorvariables_view_t* ErrVar, int size_avcMSG) {
    (void)LocalVar; (void)CntrPar; (void)avrSWAP; (void)ErrVar; (void)size_avcMSG;
}
