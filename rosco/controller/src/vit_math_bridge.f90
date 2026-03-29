! VIT Math Bridge
! Wraps Fortran math intrinsics with BIND(C) entry points so C++
! translations use the exact same implementation as the Fortran original.
! Required during the mixed Fortran/C++ phase because gfortran's COS()
! and g++'s std::cos() can differ by 1 ULP on aarch64.

MODULE vit_math_bridge
    USE ISO_C_BINDING
    IMPLICIT NONE
CONTAINS

    REAL(C_DOUBLE) FUNCTION vit_cos(x) BIND(C, NAME='vit_cos')
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: x
        vit_cos = COS(x)
    END FUNCTION

    REAL(C_DOUBLE) FUNCTION vit_sin(x) BIND(C, NAME='vit_sin')
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: x
        vit_sin = SIN(x)
    END FUNCTION

    REAL(C_DOUBLE) FUNCTION vit_atan2(y, x) BIND(C, NAME='vit_atan2')
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: y, x
        vit_atan2 = ATAN2(y, x)
    END FUNCTION

    ! Complete EKF prediction + measurement update using Fortran expressions
    ! This ensures bit-identical results with the original Fortran WSE
    ! Includes xh prediction (xh += DT*dxh), P prediction, and full measurement update
    SUBROUTINE vit_ekf_update(xh, P, K, F, Q, dxh, R_m, DT, WE_Inp_Speed, om_r) BIND(C, NAME='vit_ekf_update')
        REAL(C_DOUBLE), INTENT(INOUT) :: xh(3,1), P(3,3), K(3,1)
        REAL(C_DOUBLE), INTENT(IN)    :: F(3,3), Q(3,3), dxh(3,1)
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: R_m, DT, WE_Inp_Speed, om_r
        REAL(C_DOUBLE) :: H(1,3), S(1,1)

        H = RESHAPE((/1.0d0, 0.0d0, 0.0d0/), (/1,3/))

        ! State prediction
        xh = xh + DT * dxh

        ! P prediction update
        P = P + DT*(MATMUL(F,P) + MATMUL(P,TRANSPOSE(F)) + Q - MATMUL(K * R_m, TRANSPOSE(K)))

        ! Measurement update
        S = MATMUL(H, MATMUL(P, TRANSPOSE(H))) + R_m
        K = MATMUL(P, TRANSPOSE(H)) / S(1,1)
        xh = xh + K * (WE_Inp_Speed - om_r)
        P = MATMUL(RESHAPE((/1.0d0, 0.0d0, 0.0d0, 0.0d0, 1.0d0, 0.0d0, 0.0d0, 0.0d0, 1.0d0/), (/3,3/)) - MATMUL(K, H), P)
    END SUBROUTINE

END MODULE vit_math_bridge
