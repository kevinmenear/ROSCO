! VIT: Kernel callee bridge for AeroDynTorque
! Allows C++ translations to call the original Fortran function.
    FUNCTION aerodyntorque_bridge(RotSpeed, BldPitch, LocalVar, CntrPar, PerfData, ErrVar) &
        BIND(C, NAME='aerodyntorque_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: AeroDynTorque
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, PerformanceData, ErrorVariables
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        USE vit_performancedata_view, ONLY: vit_original_performancedata
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: RotSpeed
        REAL(C_DOUBLE), VALUE :: BldPitch
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: PerfData
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(LocalVariables), POINTER :: LocalVar_f
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        bridge_result = REAL(AeroDynTorque(RotSpeed, BldPitch, LocalVar_f, vit_original_controlparameters, vit_original_performancedata, ErrVar_f), C_DOUBLE)
    END FUNCTION aerodyntorque_bridge