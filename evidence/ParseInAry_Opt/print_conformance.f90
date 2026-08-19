! Emit the reference's OWN default-warning record for a set of cases, and the
! same cases in a form the C++ replay can read back exactly.
!
! WHY. Three of this unit's four surviving mutants are in the PRINT record, and
! no layer of this unit's evidence compares it: the differential harness
! compares out-parameters, the gate compares simulation channels, and
! `vit_mutate.py` reads a JSON PAYLOAD out of `./test`'s stdout **precisely
! because the reference may PRINT** -- so both sides' records are discarded
! before any comparison happens. That is an INSTRUMENT gap rather than a corpus
! gap, and this is the missing instrument, built the way `parser_conformance`
! was built for the READ: the reference's own runtime on one side, the SHIPPED
! translation on the other.
!
! COPIED IN SHAPE from `evidence/ParseDbAry_Opt/print_conformance.f90` (P4) --
! the two files' `emit` and their binary format are the same file with one
! declaration changed. WHAT IS **NOT** COPIED IS THE CASE SET, and that is the
! whole difference between the two units: the sibling's 44 cases ladder a REAL's
! decimal exponent, its subnormals and its three non-finite words, none of which
! an INTEGER(4) has. This unit's list-directed output has exactly one shape --
! a right-justified 12-byte field for every value in the type's range -- so the
! cases ladder the RANGE, the WIDTH, the element COUNT and the NAME instead.
!
! THE STATEMENT IS COPIED FROM ROSCO_Helpers.f90 AT 54dd134, NOT PARAPHRASED:
!
!   PRINT *, "ROSCO Warning: Did not find correct size "//TRIM( ParamName )// &
!            " in input file.  Using default value of [", Ary, "]"
!
! Writes:
!   print_conformance.ref   one record per case, exactly as the runtime wrote it
!   print_conformance.bin   name length, name, n, then n INTEGER(4)s, per case
PROGRAM print_conformance
    IMPLICIT NONE
    INTEGER, PARAMETER :: IntKi = 4
    INTEGER, PARAMETER :: MAXN = 5
    INTEGER(IntKi) :: Ary(MAXN)
    CHARACTER(200) :: ParamName
    INTEGER :: n, c, u_ref, u_bin

    OPEN(NEWUNIT=u_ref, FILE='print_conformance.ref', STATUS='REPLACE', &
         ACTION='WRITE', FORM='FORMATTED')
    OPEN(NEWUNIT=u_bin, FILE='print_conformance.bin', STATUS='REPLACE', &
         ACTION='WRITE', FORM='UNFORMATTED', ACCESS='STREAM')

    DO c = 1, 34
        CALL build(c, ParamName, Ary, n)
        CALL emit(u_ref, u_bin, ParamName, Ary, n)
    END DO

    CLOSE(u_ref)
    CLOSE(u_bin)

CONTAINS

    SUBROUTINE build(c, ParamName, Ary, n)
        INTEGER, INTENT(IN) :: c
        CHARACTER(*), INTENT(OUT) :: ParamName
        INTEGER(IntKi), INTENT(OUT) :: Ary(:)
        INTEGER, INTENT(OUT) :: n

        ParamName = 'PerfTableSize'
        n = 1
        Ary = 0

        SELECT CASE (c)
        ! The default arm's own value, and the neighbours of zero. Case 1 is the
        ! ONLY value this arm can actually produce in the shipped program --
        ! `Ary = 0` on the line above the PRINT -- and every other case here is
        ! the statement asked a question the shipped call sites do not.
        CASE (1);  Ary(1) = 0
        CASE (2);  Ary(1) = 1
        CASE (3);  Ary(1) = -1
        ! The width ladder: 1 .. 10 digits, unsigned and signed, which is where
        ! the 12-byte field's padding is decided. `field()` right-justifies into
        ! 12, so the record's column positions move with every one of these.
        CASE (4);  Ary(1) = 9
        CASE (5);  Ary(1) = 99
        CASE (6);  Ary(1) = 999
        CASE (7);  Ary(1) = 9999
        CASE (8);  Ary(1) = 99999
        CASE (9);  Ary(1) = 999999
        CASE (10); Ary(1) = 9999999
        CASE (11); Ary(1) = 99999999
        CASE (12); Ary(1) = 999999999
        CASE (13); Ary(1) = -9
        CASE (14); Ary(1) = -99999999
        CASE (15); Ary(1) = -999999999
        ! The representable extremes. `-2147483648` is ELEVEN characters, the
        ! widest an INTEGER(4) can be, and it is the case that decides whether a
        ! 12-wide field ever overflows -- the question `field()`'s deleted `'*'`
        ! fill was the answer to.
        CASE (16); Ary(1) = 2147483647
        CASE (17); Ary(1) = -2147483647
        CASE (18); Ary(1) = -2147483647 - 1
        ! Several elements, so the NO-separator-between-integers rule is
        ! exercised, and so the element loop's start and stop are visible.
        CASE (19); n = 2; Ary(1) = 0;           Ary(2) = 0
        CASE (20); n = 2; Ary(1) = 1;           Ary(2) = -1
        CASE (21); n = 3; Ary(1) = 0;           Ary(2) = 0;          Ary(3) = 0
        CASE (22); n = 3; Ary(1) = 2147483647;  Ary(2) = -2147483647 - 1
                          Ary(3) = 0
        CASE (23); n = 4; Ary(1) = -1;          Ary(2) = 22;         Ary(3) = -333
                          Ary(4) = 4444
        CASE (24); n = 5; Ary(1) = 1;           Ary(2) = 2;          Ary(3) = 3
                          Ary(4) = 4;           Ary(5) = 5
        ! A zero-length array: the record is the two CHARACTER items alone, and
        ! it is the one case in which the element loop's start cannot matter.
        CASE (25); n = 0
        ! The CHARACTER half: an empty name, a one-character name, and names
        ! long enough that a copy truncating at the wrong width would show.
        CASE (26); ParamName = ' '
        CASE (27); ParamName = 'X'
        CASE (28); ParamName = 'AB'
        CASE (29); ParamName = REPEAT('A', 60)
        CASE (30); ParamName = REPEAT('B', 199)
        CASE (31); ParamName = REPEAT('C', 200)
        ! A name with INTERIOR blanks, so that TRIM's "trailing only" rule is
        ! separated from "all blanks".
        CASE (32); ParamName = 'A B  C'
        ! The two names this unit's shipped call sites actually pass with more
        ! than one element, taken from Examples/DISCON.IN's own parameter list.
        CASE (33); ParamName = 'Ind_BldPitch'; n = 3
                   Ary(1) = 3; Ary(2) = 4; Ary(3) = 5
        CASE (34); ParamName = 'F_GenSpdNotch_Ind'; n = 2
                   Ary(1) = 0; Ary(2) = 0
        END SELECT
    END SUBROUTINE build

    SUBROUTINE emit(u_ref, u_bin, ParamName, Ary, n)
        INTEGER, INTENT(IN) :: u_ref, u_bin, n
        CHARACTER(*), INTENT(IN) :: ParamName
        INTEGER(IntKi), INTENT(IN) :: Ary(:)
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
