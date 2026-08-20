! Can a CR or an LF byte reach `FileLines` -- and therefore `Words(1)` -- from
! an input file the shipped program actually reads?
!
! WHY IT EXISTS. Two of this unit's mutants live on `is_eol(rec[p])`, and the
! first dispatch left them open on the reading that "no corpus this campaign
! generates contains a CR or an LF byte anywhere". That is a statement about the
! CORPUS. Whether the byte is ADMISSIBLE is a statement about the CALLER, and
! the caller is
!
!     rosco/controller/src/ReadSetParameters.f90:269-284 (clean baseline 54dd134)
!         OPEN (unit=..., file=accINFILE(1), status='old', action='read')
!         READ (UnControllerParameters,'(A)',IOSTAT=IOS) FileLines(I_LINE)
!
! -- a DEFAULT-FORMATTED SEQUENTIAL read of a user-supplied text file into
! `CHARACTER(MaxLineLength)`. So the question is exactly: which byte sequences
! in that file survive into the string, measured on the same gfortran runtime
! the reference is compiled against, rather than recalled from its source.
!
! FOUR RECORDS, and each one is a different claim:
!
!   plain   a Unix line                       the control: no CR, no LF
!   crlead  a bare CR as the FIRST byte       does a lone CR survive?
!   crlf    a DOS line ending                 is the terminator's CR stripped?
!   crmid   a bare CR between two words       does a lone CR survive mid-line?
!
! `crlf` is the one that could go either way and it is the one that decides how
! the finding is WORDED: libgfortran strips a CR that immediately precedes the
! record's LF, so a Windows-authored file does NOT of itself put a CR in the
! string. A CR that is not the terminator is a different byte and nothing strips
! it -- and a ROSCO input file authored on a classic-Mac editor, or edited by a
! tool that writes a lone CR, is an ORDINARY file for this OPEN statement: the
! `OPEN` names no `ACCESS`, no `FORM` and no `ENCODING`, so every byte that is
! not a record terminator is data.
!
! The file is written through STREAM access so the bytes are exactly these and
! no runtime gets to choose a terminator on our behalf.
!
!   build: gfortran -O0 eol_admissibility_probe.f90 -o eolprobe
!   run:   ./eolprobe
PROGRAM eol_admissibility_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: MaxLineLength = 2048
    CHARACTER(1), PARAMETER :: LF = CHAR(10)
    CHARACTER(1), PARAMETER :: CR = CHAR(13)
    CHARACTER(MaxLineLength) :: Line
    INTEGER :: u, ios, i, j, n

    ! --- write the file byte for byte -------------------------------------
    !
    ! Each record carries a UNIQUE leading digit, so the mapping from records
    ! WRITTEN to records READ is read off the output rather than assumed. That
    ! is the whole reason the first run of this probe was ambiguous: with the
    ! labels attached to the loop index, a runtime that splits one written line
    ! into two records silently relabels every row after it.
    OPEN (NEWUNIT=u, FILE='/tmp/eol_admissibility.IN', ACCESS='stream', &
          FORM='unformatted', STATUS='replace')
    WRITE (u) '1 plainA' // LF                       ! a Unix line
    WRITE (u) '2' // CR // 'crMID' // LF             ! a bare CR inside a line
    WRITE (u) '3 dosB' // CR // LF                   ! a DOS line ending
    WRITE (u) CR // '4 crLEAD' // LF                 ! a bare CR first
    WRITE (u) '5 tail' // CR // LF                   ! a DOS line ending again
    CLOSE (u)

    ! --- read it back exactly as ReadControlParameterFileSub does ---------
    !
    ! Read to END rather than a fixed count, so a runtime that produces MORE
    ! records than were written shows that instead of hiding it.
    OPEN (NEWUNIT=u, FILE='/tmp/eol_admissibility.IN', STATUS='old', &
          ACTION='read')
    i = 0
    DO
        Line = ' '
        READ (u, '(A)', IOSTAT=ios) Line
        IF (ios /= 0) EXIT
        i = i + 1
        n = LEN_TRIM(Line)
        WRITE (*, '(A,I2,A,I4,A)', ADVANCE='no') &
            'REC ', i, ' lentrim=', n, ' bytes='
        DO j = 1, MIN(n, 24)
            WRITE (*, '(1X,I0)', ADVANCE='no') ICHAR(Line(j:j))
        END DO
        WRITE (*, '(A)') ''
    END DO
    WRITE (*, '(A,I2,A)') 'TOTAL ', i, ' record(s) read; 5 written'
    CLOSE (u)

END PROGRAM eol_admissibility_probe
