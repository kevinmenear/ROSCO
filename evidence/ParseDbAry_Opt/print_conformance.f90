! Emit the reference's OWN default-warning record for a set of cases, and the
! same cases in a form the C++ replay can read back exactly.
!
! WHY. Nine of this unit's 32 surviving mutants are in the PRINT record, and no
! layer of this unit's evidence compares it: the differential harness compares
! out-parameters and the gate compares simulation channels. That is an
! INSTRUMENT gap rather than a corpus gap -- the negative control for it is
! gate/ParseDbAry_Opt.redtest.survivor-ae3f319a.json, a PRINT mutant that moves
! 0 of 5,252,000 gate values. This is the missing instrument, built the way unit
! #55 built `parser_conformance` for the READ: the reference's own runtime on
! one side, the SHIPPED translation on the other.
!
! THE STATEMENT IS COPIED FROM ROSCO_Helpers.f90 AT 54dd134, NOT PARAPHRASED:
!
!   PRINT *, "ROSCO Warning: Did not find correct size "//TRIM( ParamName )// &
!            " in input file.  Using default value of [", Ary, "]"
!
! The values are chosen to sit ON the boundaries the surviving mutants edit:
! `decexp` at -1 and at 16 (the arm bounds), `d == 0`, a four-digit exponent,
! the sign of the exponent, NaN and both infinities, and the widths at which
! `field()`'s 26-byte column would overflow.
!
! Writes:
!   print_conformance.ref   one record per case, exactly as the runtime wrote it
!   print_conformance.bin   name length, name, n, then n IEEE doubles, per case
PROGRAM print_conformance
    IMPLICIT NONE
    INTEGER, PARAMETER :: DbKi = KIND(1.0D0)
    INTEGER, PARAMETER :: MAXN = 4
    REAL(DbKi) :: Ary(MAXN)
    CHARACTER(200) :: ParamName
    INTEGER :: n, c, i, u_ref, u_bin
    REAL(DbKi) :: nan, pinf, ninf

    OPEN(NEWUNIT=u_ref, FILE='print_conformance.ref', STATUS='REPLACE', &
         ACTION='WRITE', FORM='FORMATTED')
    OPEN(NEWUNIT=u_bin, FILE='print_conformance.bin', STATUS='REPLACE', &
         ACTION='WRITE', FORM='UNFORMATTED', ACCESS='STREAM')

    nan  = 0.0_DbKi
    pinf = 0.0_DbKi
    ninf = 0.0_DbKi
    nan  = TRANSFER(INT(Z'7FF8000000000000', KIND=8), 1.0_DbKi)
    pinf = TRANSFER(INT(Z'7FF0000000000000', KIND=8), 1.0_DbKi)
    ninf = TRANSFER(INT(Z'FFF0000000000000', KIND=8), 1.0_DbKi)

    DO c = 1, 44
        CALL build(c, ParamName, Ary, n, nan, pinf, ninf)
        CALL emit(u_ref, u_bin, ParamName, Ary, n)
    END DO

    CLOSE(u_ref)
    CLOSE(u_bin)

CONTAINS

    SUBROUTINE build(c, ParamName, Ary, n, nan, pinf, ninf)
        INTEGER, INTENT(IN) :: c
        CHARACTER(*), INTENT(OUT) :: ParamName
        REAL(DbKi), INTENT(OUT) :: Ary(:)
        INTEGER, INTENT(OUT) :: n
        REAL(DbKi), INTENT(IN) :: nan, pinf, ninf
        INTEGER :: k

        ParamName = 'F_NotchCornerFreq'
        n = 1
        Ary = 0.0_DbKi

        SELECT CASE (c)
        ! The default arm's own value, and the neighbours of zero.
        CASE (1);  Ary(1) = 0.0_DbKi
        CASE (2);  Ary(1) = -0.0_DbKi
        CASE (3);  Ary(1) = 1.0_DbKi
        CASE (4);  Ary(1) = -1.0_DbKi
        ! decexp from -1 to 16: the two bounds of `if (decexp >= -1 && decexp <= 16)`
        ! and every value between, which is where `16 - decexp` and `d == 0` live.
        CASE (5:22)
            k = c - 6                       ! -1 .. 16
            Ary(1) = 1.2345678901234567_DbKi * (10.0_DbKi ** k)
        ! Just outside the arm on each side.
        CASE (23); Ary(1) = 1.2345678901234567D-2
        CASE (24); Ary(1) = 1.2345678901234567D17
        CASE (25); Ary(1) = 9.9999999999999998D16
        ! Exponent sign and a THREE- and FOUR-digit exponent, which is where
        ! `edig.size() > 3` and the `3 - edig.size()` zero-fill live.
        CASE (26); Ary(1) = 1.0D-100
        CASE (27); Ary(1) = 1.0D+100
        CASE (28); Ary(1) = 1.0D-300
        CASE (29); Ary(1) = 1.0D+300
        CASE (30); Ary(1) = TINY(1.0_DbKi)
        CASE (31); Ary(1) = HUGE(1.0_DbKi)
        CASE (32); Ary(1) = TINY(1.0_DbKi) / 4.0_DbKi     ! subnormal
        ! The non-finite words.
        CASE (33); Ary(1) = nan
        CASE (34); Ary(1) = pinf
        CASE (35); Ary(1) = ninf
        ! Several elements, so the no-separator-between-reals rule is exercised.
        CASE (36); n = 2; Ary(1) = 0.0_DbKi;   Ary(2) = 1.5_DbKi
        CASE (37); n = 3; Ary(1) = -2.5_DbKi;  Ary(2) = 3.75D-8; Ary(3) = 0.0_DbKi
        CASE (38); n = 4; Ary(1) = pinf; Ary(2) = nan; Ary(3) = 1.0_DbKi; Ary(4) = ninf
        ! A zero-length array: the record is the two character items alone.
        CASE (39); n = 0
        ! The CHARACTER half: an empty name, a one-character name, and a name
        ! long enough that a copy truncating at the wrong width would show.
        CASE (40); ParamName = ' '
        CASE (41); ParamName = 'X'
        CASE (42); ParamName = REPEAT('A', 60)
        CASE (43); ParamName = REPEAT('B', 199)
        CASE (44); ParamName = 'IPC_Vramp'; n = 2
                   Ary(1) = 9.1199999999999992_DbKi
                   Ary(2) = 11.400000000000000_DbKi
        END SELECT
    END SUBROUTINE build

    SUBROUTINE emit(u_ref, u_bin, ParamName, Ary, n)
        INTEGER, INTENT(IN) :: u_ref, u_bin, n
        CHARACTER(*), INTENT(IN) :: ParamName
        REAL(DbKi), INTENT(IN) :: Ary(:)
        INTEGER :: L, i

        ! THE STATEMENT UNDER TEST, item list copied from the reference. It goes
        ! to a FILE rather than to unit 6 so the record can be read back whole;
        ! list-directed output of the same items produces the same record on any
        ! formatted sequential unit.
        WRITE (u_ref,*) "ROSCO Warning: Did not find correct size "//TRIM( ParamName )// &
                        " in input file.  Using default value of [", Ary(1:n), "]"

        L = LEN_TRIM(ParamName)
        WRITE (u_bin) L
        IF (L > 0) THEN
            WRITE (u_bin) (ParamName(i:i), i = 1, L)
        END IF
        WRITE (u_bin) n
        DO i = 1, n
            WRITE (u_bin) Ary(i)
        END DO
    END SUBROUTINE emit

END PROGRAM print_conformance
