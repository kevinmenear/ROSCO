! VIT: Kernel callee bridge for saturate
! Allows C++ translations to call the original Fortran function.
    FUNCTION saturate_bridge(inputValue, minValue, maxValue) &
        BIND(C, NAME='saturate_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: saturate
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: inputValue
        REAL(C_DOUBLE), VALUE :: minValue
        REAL(C_DOUBLE), VALUE :: maxValue
        REAL(C_DOUBLE) :: bridge_result
        bridge_result = REAL(saturate(inputValue, minValue, maxValue), C_DOUBLE)
    END FUNCTION saturate_bridge