! VIT: Kernel callee bridge for sigma
! Allows C++ translations to call the original Fortran function.
    FUNCTION sigma_bridge(x, x0, x1, y0, y1, ErrVar) &
        BIND(C, NAME='sigma_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: sigma
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: x
        REAL(C_DOUBLE), VALUE :: x0
        REAL(C_DOUBLE), VALUE :: x1
        REAL(C_DOUBLE), VALUE :: y0
        REAL(C_DOUBLE), VALUE :: y1
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        bridge_result = REAL(sigma(x, x0, x1, y0, y1, ErrVar_f), C_DOUBLE)
    END FUNCTION sigma_bridge