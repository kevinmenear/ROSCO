! VIT: Kernel callee bridge for ForeAftDamping
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE foreaftdamping_bridge(CntrPar, LocalVar, objInst) &
        BIND(C, NAME='foreaftdamping_c')
        USE ISO_C_BINDING
        USE Controllers, ONLY: ForeAftDamping
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        IMPLICIT NONE
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: objInst
        TYPE(LocalVariables), POINTER :: LocalVar_f
        TYPE(ObjectInstances), POINTER :: objInst_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        CALL C_F_POINTER(objInst, objInst_f)
        CALL ForeAftDamping(vit_original_controlparameters, LocalVar_f, objInst_f)
    END SUBROUTINE foreaftdamping_bridge