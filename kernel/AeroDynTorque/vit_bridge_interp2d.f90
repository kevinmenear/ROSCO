! VIT: Kernel callee bridge for interp2d
! Allows C++ translations to call the original Fortran function.
    FUNCTION interp2d_bridge(xData, n_xData, yData, n_yData, zData, n_zData_rows, n_zData_cols, xq, yq, ErrVar) &
        BIND(C, NAME='interp2d_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: interp2d
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE
        REAL(C_DOUBLE), INTENT(IN) :: xData(*)
        INTEGER(C_INT), VALUE :: n_xData
        REAL(C_DOUBLE), INTENT(IN) :: yData(*)
        INTEGER(C_INT), VALUE :: n_yData
        REAL(C_DOUBLE), INTENT(IN) :: zData(*)
        INTEGER(C_INT), VALUE :: n_zData_rows
        INTEGER(C_INT), VALUE :: n_zData_cols
        REAL(C_DOUBLE), VALUE :: xq
        REAL(C_DOUBLE), VALUE :: yq
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(ErrorVariables), POINTER :: ErrVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(ErrVar, ErrVar_f)
        bridge_result = REAL(interp2d(xData(1:n_xData), yData(1:n_yData), RESHAPE(zData(1:n_zData_rows*n_zData_cols), (/ n_zData_rows, n_zData_cols /)), xq, yq, ErrVar_f), C_DOUBLE)
    END FUNCTION interp2d_bridge