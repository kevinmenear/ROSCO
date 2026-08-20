! The REFERENCE half of `record_form_probe.cpp`: every candidate record form
! this unit's corpus can contain, read by gfortran's own list-directed READ, out
! of the SAME storage the unit reads out of.
!
! WHY IT EXISTS, and it is the runbook's own rule rather than caution (unit #54,
! sharpened by unit #56): a corpus change that could make the REFERENCE do
! something new is a measurement to take, not a risk to accept -- and a form the
! two sides disagree on turns the differential harness RED, which takes the
! mutation layer with it for a reason that has nothing to do with the mutants
! the record was planted for. On unit #56 BOTH of that unit's defects appeared
! as a DIFFERS row here, before anything was planted and before any sweep ran.
!
! THE STORAGE IS THE POINT. `ParseInput_Int_Opt` declares
!
!     CHARACTER(MaxParamLength) :: Words(2)
!
! and READs from `Words(1)`. The two elements are one contiguous 400-byte
! object, so byte 201 -- one past the record the READ is given -- is
! `Words(2)(1:1)`, the first character of the parameter name `FindLine` just
! matched. A translation's `p < len` guard and its `p <= len` mutant differ only
! there, so the forms below vary that byte deliberately: it is an INPUT, and
! unit #56 measured that R14 had always held it constant at 'A'.
!
! AND WHAT `GetWords` ALLOWS. The record is not arbitrary. `GetWords` splits on
! ' ,!;''"' and TAB and copies the word LEFT-JUSTIFIED into a blank-padded
! 200-byte element, so no record of this unit ever contains a blank, a comma, a
! semicolon, a quote, a '!' or a tab, and every record is `word` followed by
! blanks out to byte 200. Forms that could only be handed to the FUNCTION and
! never to the UNIT are excluded on purpose -- unit #56 recorded the two
! survivors that were misread for exactly that reason. What IS reachable, and is
! exercised below: digits, sign, '*', '/', '.', letters, and the boundary.
!
!   build: gfortran -O0 -ffp-contract=off record_form_probe.f90 -o rfp_ref
!   run:   ./rfp_ref
PROGRAM record_form_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: MaxParamLength = 200
    INTEGER, PARAMETER :: NFORM = 41
    CHARACTER(MaxParamLength) :: Words(2)
    CHARACTER(16)  :: label
    CHARACTER(260) :: lead
    CHARACTER(8)   :: nb
    INTEGER(4) :: Variable
    INTEGER :: ios, i, n

    DO i = 1, NFORM
        CALL form(i, label, lead, nb, n)
        Words(1) = lead(1:n)
        Words(2) = nb
        Variable = -987654
        READ (Words(1), *, IOSTAT=ios) Variable
        WRITE (*, '(A,A16,A,I6,A,I12)') 'REF ', label, ' iostat=', ios, &
            ' value=', Variable
    END DO

CONTAINS

    ! `n` is the length of the WORD, not of the buffer: `GetWords` copies the
    ! word into a blank-padded 200-byte element, so a record is always a word
    ! left-justified with blanks after it, and a VALUE scan reaches byte 200
    ! only when the word is 200 characters long. The SEPARATOR scan reaches it
    ! on every short word, which is the opposite of unit #55's `Line` record and
    ! is why no `_BLANK_TAILS` equivalent is needed here.
    SUBROUTINE form(k, lab, txt, neigh, ln)
        INTEGER, INTENT(IN) :: k
        CHARACTER(*), INTENT(OUT) :: lab, txt, neigh
        INTEGER, INTENT(OUT) :: ln
        SELECT CASE (k)
        ! --- ordinary values, and the type's boundary ---------------------
        CASE (1);  lab = 'plain';    txt = '42';           neigh = 'Aa'; ln = 2
        CASE (2);  lab = 'signed';   txt = '+42';          neigh = 'Aa'; ln = 3
        CASE (3);  lab = 'neg';      txt = '-42';          neigh = 'Aa'; ln = 3
        CASE (4);  lab = 'imax';     txt = '2147483647';   neigh = 'Aa'; ln = 10
        CASE (5);  lab = 'imaxp1';   txt = '2147483648';   neigh = 'Aa'; ln = 10
        CASE (6);  lab = 'imin';     txt = '-2147483648';  neigh = 'Aa'; ln = 11
        CASE (7);  lab = 'iminm1';   txt = '-2147483649';  neigh = 'Aa'; ln = 11
        CASE (8);  lab = 'zeros';    txt = '0000000000000000042'
                   neigh = 'Aa'; ln = 19
        CASE (9);  lab = 'zerosbig'; txt = '002147483648'; neigh = 'Aa'; ln = 12
        ! --- forms that are NOT an integer -------------------------------
        CASE (10); lab = 'realpt';   txt = '1.5';          neigh = 'Aa'; ln = 3
        CASE (11); lab = 'realexp';  txt = '1e2';          neigh = 'Aa'; ln = 3
        CASE (12); lab = 'trailing'; txt = '5abc';         neigh = 'Aa'; ln = 4
        CASE (13); lab = 'signonly'; txt = '-';            neigh = 'Aa'; ln = 1
        CASE (14); lab = 'dot';      txt = '.';            neigh = 'Aa'; ln = 1
        CASE (15); lab = 'word';     txt = 'nan';          neigh = 'Aa'; ln = 3
        ! --- the '/' terminator. NOT a GetWords separator, so a word may
        !     contain one and this arm IS reachable from the unit.
        CASE (16); lab = 'slash';    txt = '/';            neigh = 'Aa'; ln = 1
        CASE (17); lab = 'slashaft'; txt = '7/9';          neigh = 'Aa'; ln = 3
        CASE (18); lab = 'slashpre'; txt = '/7';           neigh = 'Aa'; ln = 2
        ! --- repeat counts, and libgfortran's ceiling at the boundary.
        !     A repeat count is record GRAMMAR rather than item TYPE, so unit
        !     #55's measured constant should carry -- that is the CLAIM being
        !     checked here rather than assumed.
        CASE (19); lab = 'rep';      txt = '3*7';          neigh = 'Aa'; ln = 3
        CASE (20); lab = 'repone';   txt = '1*7';          neigh = 'Aa'; ln = 3
        CASE (21); lab = 'repzero';  txt = '0*7';          neigh = 'Aa'; ln = 3
        CASE (22); lab = 'repnull';  txt = '3*';           neigh = 'Aa'; ln = 2
        CASE (23); lab = 'repceil0'; txt = '199999999*7';  neigh = 'Aa'; ln = 11
        CASE (24); lab = 'repceil';  txt = '200000000*7';  neigh = 'Aa'; ln = 11
        CASE (25); lab = 'repover';  txt = '200000001*7';  neigh = 'Aa'; ln = 11
        CASE (26); lab = 'repwide';  txt = REPEAT('9', 21)//'*7'
                   neigh = 'Aa'; ln = 23
        ! --- FULL-WIDTH LEADS. The word is exactly 200 characters, so the
        !     VALUE scan runs to the record's last byte and `p < len` finally
        !     has two sides. Leading zeros keep the value small and legal while
        !     the significant digits sit at the END of the record -- the only
        !     way an integer's last digits are still the answer.
        CASE (27); lab = 'tailmax';  txt = REPEAT('0', 190)//'2147483647'
                   neigh = 'Aa'; ln = 200
        CASE (28); lab = 'tailover'; txt = REPEAT('0', 190)//'2147483648'
                   neigh = 'Aa'; ln = 200
        CASE (29); lab = 'tailrep';  txt = REPEAT('0', 197)//'3*7'
                   neigh = 'Aa'; ln = 200
        CASE (30); lab = 'tailstar'; txt = REPEAT('0', 199)//'*'
                   neigh = 'Aa'; ln = 200
        ! --- THE BYTE PAST A FULL-WIDTH RECORD IS AN INPUT (unit #56's
        !     `_NEIGHBOUR_TAILS`). Same 200-byte lead, four different
        !     `Words(2)` first characters -- a digit, a sign, a '*' and a '/'.
        !     Each is what one of the translation's boundary guards TESTS, and
        !     R14 has always planted a letter there.
        CASE (31); lab = 'nbdigit';  txt = REPEAT('0', 190)//'2147483647'
                   neigh = '7X'; ln = 200
        CASE (32); lab = 'nbsign';   txt = REPEAT('0', 190)//'2147483647'
                   neigh = '-7'; ln = 200
        CASE (33); lab = 'nbstar';   txt = REPEAT('0', 197)//'3*7'
                   neigh = '*7'; ln = 200
        CASE (34); lab = 'nbslash';  txt = REPEAT('0', 190)//'2147483647'
                   neigh = '/7'; ln = 200
        ! --- THE THREE FORMS THE MUTATION SEARCH NAMED, priced BEFORE they are
        !     planted. `1*` is the repeat-with-no-value at count ONE, which is
        !     the only count that distinguishes the null-fill loop's `k = 0`
        !     from `k = 1`; `3*` is the same block at a count above one, which
        !     does NOT distinguish it and is here as the control; `7x` is a
        !     legal integer followed by exactly ONE non-terminator, which is
        !     what the value-terminator test's own `rec[p]` reads.
        CASE (35); lab = 'repnull1'; txt = '1*';            neigh = 'Aa'; ln = 2
        CASE (36); lab = 'repnull3'; txt = '3*';            neigh = 'Aa'; ln = 2
        CASE (37); lab = 'junkend';  txt = '7x';            neigh = 'Aa'; ln = 2
        ! --- THE THREE FORMS THE SECOND ROUND OF SURVIVORS NAMED.
        !     `0*/` is a zero repeat count terminated by a slash: the count is
        !     rejected, and the only thing that can make the rejection visible
        !     to the UNIT is whether what follows turns it into a SUCCESS.
        !     `repstar` is a full-width lead whose digit run ends on the
        !     record's last byte behind a legal repeat count, paired with a
        !     `'*'` neighbour -- the byte the repeat lookahead's own bound
        !     tests. `over1` is a lead of 201 characters whose LAST character
        !     is significant: truncated at 200 it is 2147483647 and read at 201
        !     it overflows, which is the only shape that can observe the buffer
        !     WIDTH constant for an INTEGER item.
        CASE (38); lab = 'repzslash'; txt = '0*/';           neigh = 'Aa'; ln = 3
        CASE (39); lab = 'repstar';  txt = REPEAT('0', 189)//'3*123456789'
                   neigh = '*7'; ln = 200
        CASE (40); lab = 'over1';    txt = REPEAT('0', 190)//'21474836470'
                   neigh = 'Aa'; ln = 201
        ! --- AND THE CORRECTION TO `repstar`, which is the finding rather than
        !     the form. A repeat-count LOOKAHEAD scans digits and then asks
        !     whether the character it stopped on is a `*`; a record that
        !     CONTAINS a `*` stops that scan AT the star, so it can never reach
        !     the record's last byte. The scan reaches the boundary only on a
        !     record with no star in it at all -- and the mutant that reads one
        !     past the end then finds its star in the NEIGHBOUR. The digit run
        !     must also be a LEGAL count, which leading zeros give for free.
        CASE (41); lab = 'digstar';  txt = REPEAT('0', 199)//'1'
                   neigh = '*7'; ln = 200
        END SELECT
    END SUBROUTINE form

END PROGRAM record_form_probe
