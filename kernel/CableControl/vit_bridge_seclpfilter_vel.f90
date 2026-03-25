! VIT: Kernel callee bridge for SecLPFilter_Vel
! Allows C++ translations to call the original Fortran function.
    FUNCTION seclpfilter_vel_bridge(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_InitialValue, InitialValue) &
        BIND(C, NAME='seclpfilter_vel_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Filters, ONLY: SecLPFilter_Vel
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: InputSignal
        REAL(C_DOUBLE), VALUE :: DT
        REAL(C_DOUBLE), VALUE :: CornerFreq
        REAL(C_DOUBLE), VALUE :: Damp
        TYPE(C_PTR), VALUE :: FP
        INTEGER(C_INT), VALUE :: iStatus
        LOGICAL(C_BOOL), VALUE :: reset
        INTEGER(C_INT), INTENT(INOUT) :: inst
        INTEGER(C_INT), VALUE :: has_InitialValue
        REAL(C_DOUBLE), VALUE :: InitialValue
        REAL(C_DOUBLE) :: bridge_result
        TYPE(FilterParameters), POINTER :: FP_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(FP, FP_f)
        IF (has_InitialValue /= 0) THEN
        bridge_result = REAL(SecLPFilter_Vel(InputSignal, DT, CornerFreq, Damp, FP_f, iStatus, LOGICAL(reset), inst, InitialValue), C_DOUBLE)
        ELSE
        bridge_result = REAL(SecLPFilter_Vel(InputSignal, DT, CornerFreq, Damp, FP_f, iStatus, LOGICAL(reset), inst), C_DOUBLE)
        END IF
    END FUNCTION seclpfilter_vel_bridge