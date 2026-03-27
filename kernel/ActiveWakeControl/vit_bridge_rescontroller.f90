! VIT: Kernel callee bridge for ResController
! Allows C++ translations to call the original Fortran function.
    FUNCTION rescontroller_bridge(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) &
        BIND(C, NAME='rescontroller_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Controllers, ONLY: ResController
        USE ROSCO_Types, ONLY : resParams
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: error
        REAL(C_DOUBLE), VALUE :: kp
        REAL(C_DOUBLE), VALUE :: ki
        REAL(C_DOUBLE), VALUE :: freq
        REAL(C_DOUBLE), VALUE :: minValue
        REAL(C_DOUBLE), VALUE :: maxValue
        REAL(C_DOUBLE), VALUE :: DT
        TYPE(C_PTR), VALUE :: resP
        LOGICAL(C_BOOL), VALUE :: reset
        INTEGER(C_INT), INTENT(INOUT) :: inst
        REAL(C_DOUBLE) :: bridge_result
        TYPE(resParams), POINTER :: resP_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(resP, resP_f)
        bridge_result = REAL(ResController(error, kp, ki, freq, minValue, maxValue, DT, resP_f, LOGICAL(reset), inst), C_DOUBLE)
    END FUNCTION rescontroller_bridge