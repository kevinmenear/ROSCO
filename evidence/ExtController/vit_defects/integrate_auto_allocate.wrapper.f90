**With:**
```fortran
    SUBROUTINE ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar)
        USE ISO_C_BINDING
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_extcontroltype_view, ONLY: extcontroltype_view_t, vit_populate_extcontroltype, vit_copy_scalars_to_extcontroltype
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
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
        INTEGER(4) :: ErrStat2
        CHARACTER(ErrMsgLen) :: ErrMsg2
        CHARACTER(*), PARAMETER :: RoutineName = 'ExtController'
        INTEGER :: vit_alloc_i
        ! --- Pre-allocate arrays (auto-extracted from Fortran source) ---
        IF (LocalVar%iStatus == 0) THEN
        CALL LoadDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
            IF (ErrStat2 /= 0) THEN
                CALL SetErrStat(ErrID_Fatal, "Error in LoadDynamicLib.", ErrStat, ErrMsg, RoutineName)
                RETURN
            END IF
        END IF
        CALL DLL_Legacy_Subroutine (ExtDLL%avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG )
            IF (ErrStat2 /= 0) THEN
                CALL SetErrStat(ErrID_Fatal, "Error in DLL_Legacy_Subroutine.", ErrStat, ErrMsg, RoutineName)
                RETURN
            END IF
        ! WARNING: These allocations could not be auto-extracted safely.
        ! Review and add ALLOCATE statements manually if needed:
        !   ExtDLL%avrSWAP(max_avr_entries) — local/computed size [direct]
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_extcontroltype(ExtDLL, ExtDLL_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(ExtDLL_view), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_extcontroltype(ExtDLL_view, ExtDLL)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ExtController
```

