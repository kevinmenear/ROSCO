! VIT: Kernel callee bridge for PitchSaturation
! Allows C++ translations to call the original Fortran function.
    FUNCTION pitchsaturation_bridge(LocalVar, CntrPar, objInst, DebugVar, ErrVar) &
        BIND(C, NAME='pitchsaturation_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE ControllerBlocks, ONLY: PitchSaturation
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        IMPLICIT NONE
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: objInst
        TYPE(C_PTR), VALUE :: DebugVar
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(LocalVariables), POINTER :: LocalVar_f
        TYPE(ObjectInstances), POINTER :: objInst_f
        TYPE(DebugVariables), POINTER :: DebugVar_f
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        CALL C_F_POINTER(objInst, objInst_f)
        CALL C_F_POINTER(DebugVar, DebugVar_f)
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        bridge_result = REAL(PitchSaturation(LocalVar_f, vit_original_controlparameters, objInst_f, DebugVar_f, ErrVar_f), C_DOUBLE)
    END FUNCTION pitchsaturation_bridge