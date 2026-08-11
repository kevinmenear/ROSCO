    SUBROUTINE ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar)
        USE ISO_C_BINDING
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables
        USE vit_extcontroltype_view, ONLY: extcontroltype_view_t, vit_populate_extcontroltype
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(EXTCONTROLTYPE), INTENT(INOUT), TARGET :: ExtDLL
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(extcontroltype_view_t), TARGET :: ExtDLL_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_extcontroltype(ExtDLL, ExtDLL_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(ExtDLL_view), C_LOC(ErrVar_view))
    END SUBROUTINE ExtController