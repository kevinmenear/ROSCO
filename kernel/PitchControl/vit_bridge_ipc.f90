! VIT: Kernel callee bridge for IPC
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE ipc_bridge(CntrPar, LocalVar, objInst, DebugVar, ErrVar) &
        BIND(C, NAME='ipc_c')
        USE ISO_C_BINDING
        USE Controllers, ONLY: IPC
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        IMPLICIT NONE
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: objInst
        TYPE(C_PTR), VALUE :: DebugVar
        TYPE(C_PTR), VALUE :: ErrVar
        TYPE(LocalVariables), POINTER :: LocalVar_f
        TYPE(ObjectInstances), POINTER :: objInst_f
        TYPE(DebugVariables), POINTER :: DebugVar_f
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        CALL C_F_POINTER(objInst, objInst_f)
        CALL C_F_POINTER(DebugVar, DebugVar_f)
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        CALL IPC(vit_original_controlparameters, LocalVar_f, objInst_f, DebugVar_f, ErrVar_f)
    END SUBROUTINE ipc_bridge