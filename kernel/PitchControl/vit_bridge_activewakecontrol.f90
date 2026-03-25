! VIT: Kernel callee bridge for ActiveWakeControl
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE activewakecontrol_bridge(CntrPar, LocalVar, DebugVar, objInst) &
        BIND(C, NAME='activewakecontrol_c')
        USE ISO_C_BINDING
        USE Controllers, ONLY: ActiveWakeControl
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, DebugVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        IMPLICIT NONE
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: DebugVar
        TYPE(C_PTR), VALUE :: objInst
        TYPE(LocalVariables), POINTER :: LocalVar_f
        TYPE(DebugVariables), POINTER :: DebugVar_f
        TYPE(ObjectInstances), POINTER :: objInst_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        CALL C_F_POINTER(DebugVar, DebugVar_f)
        CALL C_F_POINTER(objInst, objInst_f)
        CALL ActiveWakeControl(vit_original_controlparameters, LocalVar_f, DebugVar_f, objInst_f)
    END SUBROUTINE activewakecontrol_bridge