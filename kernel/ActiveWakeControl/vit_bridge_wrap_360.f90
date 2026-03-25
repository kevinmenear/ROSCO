! VIT: Kernel callee bridge for wrap_360
! Allows C++ translations to call the original Fortran function.
    FUNCTION wrap_360_bridge(x) &
        BIND(C, NAME='wrap_360_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: wrap_360
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: x
        REAL(C_DOUBLE) :: bridge_result
        bridge_result = REAL(wrap_360(x), C_DOUBLE)
    END FUNCTION wrap_360_bridge