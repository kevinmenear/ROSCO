! VIT: Kernel callee bridge for ColemanTransform
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE colemantransform_bridge(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut) &
        BIND(C, NAME='colemantransform_c')
        USE ISO_C_BINDING
        USE Functions, ONLY: ColemanTransform
        IMPLICIT NONE
        REAL(C_DOUBLE), INTENT(IN) :: rootMOOP(*)
        REAL(C_DOUBLE), VALUE :: aziAngle
        INTEGER(C_INT), VALUE :: nHarmonic
        REAL(C_DOUBLE), INTENT(OUT) :: axTOut
        REAL(C_DOUBLE), INTENT(OUT) :: axYOut
        CALL ColemanTransform(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut)
    END SUBROUTINE colemantransform_bridge