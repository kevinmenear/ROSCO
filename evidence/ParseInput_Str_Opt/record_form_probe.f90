! The REFERENCE half: gfortran's own `READ (Words(1),'(A)',IOSTAT=e) Variable`
! over every record form this unit's corpus can contain, crossed with the item
! lengths its shipped callers use and the two boundaries at the record width.
!
! WHY THIS PROBE EXISTS. The translation's whole item half rests on one claim --
! that an `A` edit descriptor with no field width, applied to a CHARACTER item
! over a padded internal record, is Fortran's own CHARACTER assignment and has
! NO failure mode. That claim decides whether `IF (ErrStatLcl /= 0)` is a live
! arm or a dead one, and a dead arm is a set of `unreachable` declarations. It
! is read off Fortran 2018 13.7.4.1 and 12.6.4.5.3; this measures it.
!
! THE ITEM IS PRE-SET TO A SENTINEL ('~' in every byte) so an UNTOUCHED item is
! distinguishable from a stored one -- the same discipline unit #57 used with
! -987654, and here it is what separates "the READ failed and stored nothing"
! from "the READ stored blanks".
!
! THE STORAGE IS THE REFERENCE'S OWN: `CHARACTER(MaxParamLength) :: Words(2)`,
! one contiguous 400-byte object whose byte 201 is Words(2)(1:1). A translation
! that read `len_Variable` bytes out of that object rather than out of Words(1)
! would splice the parameter NAME into the value on every shipped call, and the
! `full` forms below are what catch it: each carries a 200-byte word and a
! neighbour that is not blank.
PROGRAM record_form_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: MaxParamLength = 200
    INTEGER, PARAMETER :: NL = 9
    INTEGER, PARAMETER :: LENS(NL) = (/ 1, 5, 6, 7, 199, 200, 201, 256, 1024 /)
    INTEGER, PARAMETER :: NF = 16

    CHARACTER(MaxParamLength) :: Words(2)
    CHARACTER(:), ALLOCATABLE :: item
    CHARACTER(32)             :: tag
    INTEGER                   :: f, k, e

    DO f = 1, NF
        CALL form(f, tag, Words)
        DO k = 1, NL
            IF (ALLOCATED(item)) DEALLOCATE(item)
            ALLOCATE(CHARACTER(LENS(k)) :: item)
            item = REPEAT('~', LENS(k))
            e = 0
            CALL do_read(Words(1), item, e)
            CALL emit(tag, LENS(k), e, item)
        END DO
    END DO

CONTAINS

    ! THE STATEMENT UNDER TEST, in the reference's own context: the item is a
    ! `CHARACTER(*), INTENT(INOUT)` dummy, exactly as `Variable` is.
    SUBROUTINE do_read(rec, Variable, ErrStatLcl)
        CHARACTER(*), INTENT(IN   ) :: rec
        CHARACTER(*), INTENT(INOUT) :: Variable
        INTEGER,      INTENT(  OUT) :: ErrStatLcl
        READ (rec,'(A)',IOSTAT=ErrStatLcl)  Variable
    END SUBROUTINE do_read

    SUBROUTINE form(k, tg, W)
        INTEGER,      INTENT(IN   ) :: k
        CHARACTER(*), INTENT(  OUT) :: tg
        CHARACTER(*), INTENT(  OUT) :: W(2)
        W(1) = ''
        W(2) = 'Aa'
        SELECT CASE (k)
        ! -- admissible to the UNIT: a word GetWords can produce ------------
        CASE ( 1); tg = 'plain';     W(1) = 'unused'
        CASE ( 2); tg = 'path';      W(1) = 'Cp_Ct_Cq.txt'
        CASE ( 3); tg = 'digits';    W(1) = '12345'
        CASE ( 4); tg = 'slash';     W(1) = 'a/b'
        CASE ( 5); tg = 'star';      W(1) = '3*7'
        CASE ( 6); tg = 'nul';       W(1) = 'ab'//CHAR(0)//'cd'
        CASE ( 7); tg = 'cr';        W(1) = 'ab'//CHAR(13)//'cd'
        CASE ( 8); tg = 'lf';        W(1) = 'ab'//CHAR(10)//'cd'
        CASE ( 9); tg = 'full';      W(1) = REPEAT('Z', 200)
        CASE (10); tg = 'full199';   W(1) = REPEAT('Y', 199)
        CASE (11); tg = 'fullcr';    W(1) = REPEAT(CHAR(13), 200)
        CASE (12); tg = 'fullnul';   W(1) = REPEAT(CHAR(0), 200)
        CASE (13); tg = 'blankrec';  W(1) = ''
        ! `blankrec` with a NON-blank neighbour is the form that separates the
        ! 200-byte record from the 400-byte object: if anything of Words(2)
        ! reaches the item, it shows here and nowhere else.
        CASE (14); tg = 'blankrecN'; W(1) = '';  W(2) = REPEAT('N', 200)
        ! -- admissible to the FUNCTION and not to the UNIT -----------------
        ! GetWords splits on blank, comma, semicolon, '!', quote and tab, so
        ! none of these can be Words(1). Tagged, not omitted: unit #56 read a
        ! function-only record as a corpus lever twice.
        CASE (15); tg = 'fn:tab';    W(1) = 'a'//CHAR(9)//'b'
        CASE (16); tg = 'fn:lead';   W(1) = '   x'
        END SELECT
    END SUBROUTINE form

    ! One row, in a shape the C++ half reproduces byte for byte.
    SUBROUTINE emit(tg, L, e, s)
        CHARACTER(*), INTENT(IN) :: tg
        INTEGER,      INTENT(IN) :: L, e
        CHARACTER(*), INTENT(IN) :: s
        CHARACTER(200) :: pre
        INTEGER(8)     :: h
        INTEGER        :: i, n
        h = 0
        DO i = 1, LEN(s)
            h = MOD(h * 31_8 + INT(ICHAR(s(i:i)), 8), 2147483647_8)
        END DO
        pre = ''
        n = 0
        DO i = 1, MIN(24, LEN(s))
            CALL esc(s(i:i), pre, n)
        END DO
        WRITE (*,'(A,1X,A,1X,I0,1X,A,1X,I0,1X,A,1X,I0,1X,A,1X,I0,1X,A,A,A)') &
            'R', TRIM(tg), L, 'iostat=', e, 'lentrim=', LEN_TRIM(s), &
            'h=', h, 'p="', pre(1:n), '"'
    END SUBROUTINE emit

    SUBROUTINE esc(c, buf, n)
        CHARACTER(1), INTENT(IN   ) :: c
        CHARACTER(*), INTENT(INOUT) :: buf
        INTEGER,      INTENT(INOUT) :: n
        CHARACTER(2) :: hx
        INTEGER      :: v
        v = ICHAR(c)
        IF (v >= 32 .AND. v < 127 .AND. c /= '\' .AND. c /= '"') THEN
            buf(n+1:n+1) = c
            n = n + 1
        ELSE
            WRITE (hx,'(Z2.2)') v
            buf(n+1:n+4) = '\x'//hx
            n = n + 4
        END IF
    END SUBROUTINE esc

END PROGRAM record_form_probe
