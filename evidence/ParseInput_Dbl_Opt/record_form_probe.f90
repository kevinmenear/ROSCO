! The REFERENCE half of `record_form_probe.cpp`: eighteen candidate record forms
! read by gfortran's own list-directed READ, out of the SAME storage the unit
! reads out of.
!
! WHY IT EXISTS, and it is the runbook's own rule rather than caution: a corpus
! change that could make the REFERENCE do something new is a measurement to
! take, not a risk to accept. Every form below is a record `harness/generate.py`
! is about to plant, and if gfortran and the translation disagree on any one of
! them the differential harness goes RED -- which takes the mutation layer with
! it, for a reason that has nothing to do with the mutants the record was
! planted for.
!
! THE STORAGE IS THE POINT. `ParseInput_Dbl_Opt` declares
!
!     CHARACTER(MaxParamLength) :: Words(2)
!
! and READs from `Words(1)`. The two elements are one contiguous 400-byte
! object, so byte 201 -- one past the record the READ is given -- is
! `Words(2)(1:1)`. Eight of the forms below vary exactly that byte, because a
! translation's `p < len` guard and its `p <= len` mutant differ only there.
!
!   build: gfortran -O0 -ffp-contract=off record_form_probe.f90 -o rfp_ref
!   run:   ./rfp_ref
PROGRAM record_form_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: MaxParamLength = 200
    CHARACTER(MaxParamLength) :: Words(2)
    CHARACTER(64)  :: label
    CHARACTER(220) :: lead
    CHARACTER(8)   :: nb
    REAL(8) :: Variable
    INTEGER :: ios, i, n
    INTEGER(8) :: bits

    DO i = 1, 30
        CALL form(i, label, lead, nb, n)
        Words(1) = lead(1:n)
        Words(2) = nb
        Variable = -987.654D0
        READ (Words(1), *, IOSTAT=ios) Variable
        bits = TRANSFER(Variable, bits)
        WRITE (*, '(A,A16,A,I6,A,Z16.16)') 'REF ', label, ' iostat=', ios, &
            ' bits=', bits
    END DO

CONTAINS

    ! `n` is the length of the WORD, not of the buffer: `GetWords` copies the
    ! word into a blank-padded 200-byte element, so a record is always a word
    ! left-justified with blanks after it, and a scan reaches byte 200 only when
    ! the word is 200 characters long.
    SUBROUTINE form(k, lab, txt, neigh, ln)
        INTEGER, INTENT(IN) :: k
        CHARACTER(*), INTENT(OUT) :: lab, txt, neigh
        INTEGER, INTENT(OUT) :: ln
        CHARACTER(200) :: digits200, frac200
        digits200 = REPEAT('0', 199)//'1'
        frac200   = '1.'//REPEAT('0', 197)//'5'
        SELECT CASE (k)
        CASE (1);  lab = 'dig';     txt = digits200; neigh = '7E'; ln = 200
        CASE (2);  lab = 'frac';    txt = frac200;   neigh = '7E'; ln = 200
        CASE (3);  lab = 'expl';    txt = digits200; neigh = 'E7'; ln = 200
        CASE (4);  lab = 'expq';    txt = digits200; neigh = 'Q7'; ln = 200
        CASE (5);  lab = 'sign';    txt = digits200; neigh = '+7'; ln = 200
        CASE (6);  lab = 'neg';     txt = digits200; neigh = '-7'; ln = 200
        CASE (7);  lab = 'point';   txt = digits200; neigh = '.7'; ln = 200
        CASE (8);  lab = 'fracE';   txt = frac200;   neigh = 'E7'; ln = 200
        CASE (9);  lab = 'rep';     txt = '3*7';     neigh = 'Aa'; ln = 3
        CASE (10); lab = 'repbad';  txt = '0*7';     neigh = 'Aa'; ln = 3
        CASE (11); lab = 'expsign'; txt = '1.5E+2';  neigh = 'Aa'; ln = 6
        CASE (12); lab = 'expneg';  txt = '1.5E-2';  neigh = 'Aa'; ln = 6
        CASE (13); lab = 'bare';    txt = '1.5+2';   neigh = 'Aa'; ln = 5
        CASE (14); lab = 'baren';   txt = '1.5-2';   neigh = 'Aa'; ln = 5
        CASE (15); lab = 'dot';     txt = '.';       neigh = 'Aa'; ln = 1
        CASE (16); lab = 'dotdot';  txt = '..';      neigh = 'Aa'; ln = 2
        CASE (17); lab = 'dote';    txt = '.e';      neigh = 'Aa'; ln = 2
        CASE (18); lab = 'nanpay';  txt = 'nan('//REPEAT('a', 58)//')'
                   neigh = 'Aa'; ln = 63
        ! A repeat count so wide that the value it repeats starts at byte 198.
        ! `match_word`'s own bound is `p + LEN(word) > len`, and two mutants of
        ! it are declared equivalent on the premise that `p` is always 0 or 1.
        CASE (19); lab = 'repwide';  txt = REPEAT('9', 196)//'*nan'
                   neigh = 'Aa'; ln = 200
        CASE (20); lab = 'repwide2'; txt = REPEAT('9', 195)//'*nan '
                   neigh = 'Aa'; ln = 199
        CASE (21); lab = 'repbig';   txt = REPEAT('9', 21)//'*7'
                   neigh = 'Aa'; ln = 23
        ! libgfortran's repeat-count ceiling, at the boundary. Unit #55
        ! measured it for an INTEGER item; a repeat count is record GRAMMAR
        ! rather than item TYPE, so it should carry -- and that is the claim
        ! being checked here rather than assumed.
        CASE (22); lab = 'repceil0'; txt = '199999999*7'; neigh = 'Aa'; ln = 11
        CASE (23); lab = 'repceil';  txt = '200000000*7'; neigh = 'Aa'; ln = 11
        CASE (24); lab = 'repover';  txt = '200000001*7'; neigh = 'Aa'; ln = 11
        ! A count of 3 behind 195 leading zeros: the count is small and legal,
        ! and the VALUE it repeats starts at byte 198 -- which is the only way
        ! `match_word`'s `p + LEN(word) > len` bound is reached with both sides
        ! live once the ceiling above rejects a wide count.
        CASE (25); lab = 'repzeros'; txt = REPEAT('0', 195)//'3*nan'
                   neigh = 'Aa'; ln = 200
        ! Five more that use a repeat count behind leading zeros to move the
        ! VALUE's start, so the value ENDS on the record's last byte with its
        ! last digits still significant to a double. A 200-digit run cannot do
        ! that: its trailing digits are past a double's precision, which is why
        ! the plain `frac` tail leaves the fraction scan's guard alive.
        CASE (26); lab = 'fracsig';  txt = REPEAT('0', 185)//'3*1.23456789012'
                   neigh = '7E'; ln = 200
        CASE (27); lab = 'expsig';   txt = REPEAT('0', 193)//'3*1.5e2'
                   neigh = '7E'; ln = 200
        CASE (28); lab = 'pointend'; txt = REPEAT('0', 189)//'200000000*-'
                   neigh = '.7'; ln = 200
        CASE (29); lab = 'over';     txt = '1'//REPEAT('0', 199)//'1'
                   neigh = '7E'; ln = 201
        CASE (30); lab = 'repone';   txt = '1*7'; neigh = 'Aa'; ln = 3
        END SELECT
    END SUBROUTINE form

END PROGRAM record_form_probe
