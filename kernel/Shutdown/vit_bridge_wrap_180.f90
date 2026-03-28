! VIT: Kernel callee bridge for wrap_180
! Allows C++ translations to call the original Fortran function.
    FUNCTION wrap_180_bridge(x) &
        BIND(C, NAME='wrap_180_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY: wrap_180
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: x
        REAL(C_DOUBLE) :: bridge_result
        bridge_result = REAL(wrap_180(x), C_DOUBLE)
    END FUNCTION wrap_180_bridge