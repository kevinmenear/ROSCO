! VIT: Kernel callee bridge for unwrap
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE unwrap_bridge(x, n_x, ErrVar, unwrap_result) &
        BIND(C, NAME='unwrap_c')
        USE ISO_C_BINDING
        USE Functions, ONLY: unwrap
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE
        REAL(C_DOUBLE), INTENT(IN) :: x(*)
        INTEGER(C_INT), VALUE :: n_x
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE), INTENT(OUT) :: unwrap_result(*)
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        CALL unwrap(x(1:n_x), ErrVar_f)
    END SUBROUTINE unwrap_bridge