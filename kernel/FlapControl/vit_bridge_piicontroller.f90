! VIT: Kernel callee bridge for PIIController
! Allows C++ translations to call the original Fortran function.
    FUNCTION piicontroller_bridge(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) &
        BIND(C, NAME='piicontroller_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Controllers, ONLY: PIIController
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: error
        REAL(C_DOUBLE), VALUE :: error2
        REAL(C_DOUBLE), VALUE :: kp
        REAL(C_DOUBLE), VALUE :: ki
        REAL(C_DOUBLE), VALUE :: ki2
        REAL(C_DOUBLE), VALUE :: minValue
        REAL(C_DOUBLE), VALUE :: maxValue
        REAL(C_DOUBLE), VALUE :: DT
        REAL(C_DOUBLE), VALUE :: I0
        TYPE(C_PTR), VALUE :: piP
        LOGICAL(C_BOOL), VALUE :: reset
        INTEGER(C_INT), INTENT(INOUT) :: inst
        REAL(C_DOUBLE) :: bridge_result
        TYPE(piParams), POINTER :: piP_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(piP, piP_f)
        bridge_result = REAL(PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP_f, LOGICAL(reset), inst), C_DOUBLE)
    END FUNCTION piicontroller_bridge