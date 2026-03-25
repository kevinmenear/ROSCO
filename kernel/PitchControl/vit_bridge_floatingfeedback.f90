! VIT: Kernel callee bridge for FloatingFeedback
! Allows C++ translations to call the original Fortran function.
    FUNCTION floatingfeedback_bridge(LocalVar, CntrPar, objInst, ErrVar) &
        BIND(C, NAME='floatingfeedback_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Controllers, ONLY: FloatingFeedback
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        IMPLICIT NONE
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: objInst
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(LocalVariables), POINTER :: LocalVar_f
        TYPE(ObjectInstances), POINTER :: objInst_f
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        CALL C_F_POINTER(objInst, objInst_f)
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        bridge_result = REAL(FloatingFeedback(LocalVar_f, vit_original_controlparameters, objInst_f, ErrVar_f), C_DOUBLE)
    END FUNCTION floatingfeedback_bridge