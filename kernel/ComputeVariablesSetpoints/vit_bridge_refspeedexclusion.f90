! VIT: Kernel callee bridge for RefSpeedExclusion
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE refspeedexclusion_bridge(LocalVar, CntrPar, objInst, DebugVar) &
        BIND(C, NAME='refspeedexclusion_c')
        USE ISO_C_BINDING
        USE ControllerBlocks, ONLY: RefSpeedExclusion
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        IMPLICIT NONE
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: objInst
        TYPE(C_PTR), VALUE :: DebugVar
        TYPE(LocalVariables), POINTER :: LocalVar_f
        TYPE(ObjectInstances), POINTER :: objInst_f
        TYPE(DebugVariables), POINTER :: DebugVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        CALL C_F_POINTER(objInst, objInst_f)
        CALL C_F_POINTER(DebugVar, DebugVar_f)
        CALL RefSpeedExclusion(LocalVar_f, vit_original_controlparameters, objInst_f, DebugVar_f)
    END SUBROUTINE refspeedexclusion_bridge