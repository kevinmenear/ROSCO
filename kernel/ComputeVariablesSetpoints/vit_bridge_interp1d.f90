! VIT: Kernel callee bridge for interp1d
! Allows C++ translations to call the original Fortran function.
    FUNCTION interp1d_bridge(xData, n_xData, yData, n_yData, xq, ErrVar) &
        BIND(C, NAME='interp1d_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: interp1d
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE
        REAL(C_DOUBLE), INTENT(IN) :: xData(*)
        INTEGER(C_INT), VALUE :: n_xData
        REAL(C_DOUBLE), INTENT(IN) :: yData(*)
        INTEGER(C_INT), VALUE :: n_yData
        REAL(C_DOUBLE), VALUE :: xq
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        bridge_result = REAL(interp1d(xData(1:n_xData), yData(1:n_yData), xq, ErrVar_f), C_DOUBLE)
    END FUNCTION interp1d_bridge