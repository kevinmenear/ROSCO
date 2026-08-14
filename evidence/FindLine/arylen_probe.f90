! WHAT THE REFERENCE DOES FOR EACH VALUE OF FindLine's OPTIONAL `AryLen`.
!
! The generated corpus varies a defaulted integer over R6's ladder -- decade
! boundaries and the representable extremes of a 32-bit INTEGER, signed -- and
! the generated program SEGFAULTED at case 0. `harness/ranges.toml` is the only
! place a human judgement about an admissible domain is recorded, and an entry
! resting on "that looks wrong" does not belong there. So this asks the
! REFERENCE, one value per process, and the answer is its exit status.
!
!   AryLen  ->  WordInd = AryLen + 1  ->  ALLOCATE(Words(WordInd))
!
! WordInd <= 0 gives a ZERO-SIZED array, and then `GetWords` writes `Words(1)`
! on any non-blank line and `FileLineUC = Words(WordInd)` reads `Words(0)` or
! below -- outside the allocation in both directions. There is no reference
! behaviour there to mirror.
PROGRAM arylen_probe
    USE ROSCO_Helpers, ONLY : FindLine
    IMPLICIT NONE
    CHARACTER(5)     :: FileLines(3)
    CHARACTER(2048)  :: Line
    INTEGER(4)       :: LineNum, AryLen
    LOGICAL          :: FoundLine
    CHARACTER(32)    :: arg

    CALL GET_COMMAND_ARGUMENT(1, arg)
    READ(arg, *) AryLen

    ! Three ordinary non-blank lines, the shape the generator produces.
    FileLines(1) = 'a b c'
    FileLines(2) = 'd e f'
    FileLines(3) = 'g h i'

    Line = ' '
    WRITE(*,'(A,I12)', ADVANCE='NO') 'AryLen=', AryLen
    FLUSH(6)
    CALL FindLine(FileLines, 'C', FoundLine, Line, LineNum, AryLen)
    WRITE(*,'(A,L2,A,I4,A)') '  RETURNED  FoundLine=', FoundLine, &
        '  LineNum=', LineNum, '  Line="'//TRIM(Line)//'"'
END PROGRAM arylen_probe
