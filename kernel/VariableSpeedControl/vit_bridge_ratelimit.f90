! VIT: Kernel callee bridge for ratelimit
! Allows C++ translations to call the original Fortran function.
    FUNCTION ratelimit_bridge(inputSignal, minRate, maxRate, DT, reset, rlP, inst, has_ResetValue, ResetValue) &
        BIND(C, NAME='ratelimit_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: ratelimit
        USE ROSCO_Types, ONLY : rlParams
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: inputSignal
        REAL(C_DOUBLE), VALUE :: minRate
        REAL(C_DOUBLE), VALUE :: maxRate
        REAL(C_DOUBLE), VALUE :: DT
        LOGICAL(C_BOOL), VALUE :: reset
        TYPE(C_PTR), VALUE :: rlP
        INTEGER(C_INT), INTENT(INOUT) :: inst
        INTEGER(C_INT), VALUE :: has_ResetValue
        REAL(C_DOUBLE), VALUE :: ResetValue
        REAL(C_DOUBLE) :: bridge_result
        TYPE(rlParams), POINTER :: rlP_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(rlP, rlP_f)
        IF (has_ResetValue /= 0) THEN
        bridge_result = REAL(ratelimit(inputSignal, minRate, maxRate, DT, LOGICAL(reset), rlP_f, inst, ResetValue), C_DOUBLE)
        ELSE
        bridge_result = REAL(ratelimit(inputSignal, minRate, maxRate, DT, LOGICAL(reset), rlP_f, inst), C_DOUBLE)
        END IF
    END FUNCTION ratelimit_bridge