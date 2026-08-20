! FORTRAN SIDE of the three-question semantics control for WriteRestartFile.
!
! WHY IT EXISTS. The unit's corpus -- 195 checkpoints and 6 stdout streams --
! cannot reach three things, and `mutation/WriteRestartFile.unreachable.json`
! declares three mutants against them. A declaration says "this corpus cannot
! tell the two apart"; it does not say the translation is RIGHT. These three
! are pure Fortran-semantics questions with no dependence on runtime state, so
! they can be answered by a standalone program at a cost the corpus cannot
! match.
!
!   Q1  what does gfortran write for a DEFAULT LOGICAL, .TRUE. and .FALSE.,
!       to an unformatted STREAM unit?    -- the translation's `wr_log` widens
!       the view's int8_t back to four bytes and writes 1 for true. Both
!       logicals this unit writes are .FALSE. in all 195 checkpoints, so the
!       .TRUE. half is compared ZERO times by the corpus.
!   Q2  what does `WRITE(s,'(I0.0)') n` produce for n = 0 and n = 1?  -- the
!       standard says a zero value with d == 0 is BLANK, so a checkpoint at
!       Time == 0 is named `<RootName>.RO.chkp` with no number. The corpus'
!       indices run 40 to 21560.
!   Q3  what does a CHARACTER(128) assignment do to a longer concatenation, and
!       what is TRIM of the result?  -- the longest name any scenario composes
!       is 38.
!
! Both sides write the same two files under different names and the runner
! `cmp`s them. See semantics_probe.sh.
PROGRAM semantics_probe
    IMPLICIT NONE
    INTEGER            :: un, i, n
    LOGICAL            :: t, f
    CHARACTER(128)     :: s, InFile
    CHARACTER(200)     :: root
    INTEGER, PARAMETER :: NN = 7
    INTEGER, PARAMETER :: vals(NN) = (/ 0, 1, -1, 7, 40, 21560, -21560 /)

    t = .TRUE.
    f = .FALSE.

    ! Q1 -- the raw bytes, exactly as WriteRestartFile writes them.
    OPEN(NEWUNIT=un, FILE='semantics_probe.f.bin', STATUS='REPLACE', &
         FORM='UNFORMATTED', ACCESS='STREAM', ACTION='WRITE')
    WRITE(un) t
    WRITE(un) f
    WRITE(un) t
    CLOSE(un)

    OPEN(NEWUNIT=un, FILE='semantics_probe.f.txt', STATUS='REPLACE', ACTION='WRITE')

    ! Q2 -- I0.0 across the sign and the zero.
    DO i = 1, NN
        n = vals(i)
        WRITE(s, '(I0.0)') n
        WRITE(un, '(A,I0,A,A,A,I0)') 'I0.0(', n, ') = [', TRIM(s), '] len=', LEN(TRIM(s))
    END DO

    ! Q3 -- the CHARACTER(128) truncation, at and around the boundary.
    !   126, 127, 128 and 129 characters of RootName before the 8-byte suffix,
    !   so the composed value crosses 128 in both directions.
    DO i = 116, 124
        root = REPEAT('R', i)
        WRITE(s, '(I0.0)') 7
        InFile = TRIM(root)//TRIM(s)//'.RO.chkp'
        WRITE(un, '(A,I0,A,I0,A,A,A)') 'root=', i, ' lenTrim=', LEN(TRIM(InFile)), &
              ' [', TRIM(InFile), ']'
    END DO
    CLOSE(un)
END PROGRAM semantics_probe
