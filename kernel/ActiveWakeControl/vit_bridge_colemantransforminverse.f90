! VIT: Kernel callee bridge for ColemanTransformInverse
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE colemantransforminverse_bridge(axTIn, axYIn, aziAngle, nHarmonic, aziOffset, PitComIPC) &
        BIND(C, NAME='colemantransforminverse_c')
        USE ISO_C_BINDING
        USE Functions, ONLY: ColemanTransformInverse
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: axTIn
        REAL(C_DOUBLE), VALUE :: axYIn
        REAL(C_DOUBLE), VALUE :: aziAngle
        INTEGER(C_INT), VALUE :: nHarmonic
        REAL(C_DOUBLE), VALUE :: aziOffset
        REAL(C_DOUBLE), INTENT(OUT) :: PitComIPC(*)
        CALL ColemanTransformInverse(axTIn, axYIn, aziAngle, nHarmonic, aziOffset, PitComIPC)
    END SUBROUTINE colemantransforminverse_bridge