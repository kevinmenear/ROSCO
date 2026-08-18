! VIT: callee bridges for the ParseDbAry_Opt test-validate harness.
! Each `<callee>_c` calls the ORIGINAL Fortran, so both sides of the
! comparison share one callee implementation and a mismatch is
! attributable to the unit under test.

! VIT: Kernel callee bridge for Int2LStr
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE int2lstr_bridge(Num, Int2LStr_result) &
        BIND(C, NAME='int2lstr_c')
        USE ISO_C_BINDING
        USE ROSCO_Helpers, ONLY : Int2LStr
        IMPLICIT NONE
        INTEGER(C_INT), VALUE :: Num
        CHARACTER(KIND=C_CHAR), INTENT(OUT) :: Int2LStr_result(*)
        CHARACTER(11) :: vit_result_int2lstr
        INTEGER :: vit_ri_int2lstr
        vit_result_int2lstr = Int2LStr(Num)
        ! Copy the CHARACTER function result back to the C buffer
        DO vit_ri_int2lstr = 1, 11
            Int2LStr_result(vit_ri_int2lstr) = &
                vit_result_int2lstr(vit_ri_int2lstr:vit_ri_int2lstr)
        END DO
    END SUBROUTINE int2lstr_bridge
! VIT: Kernel callee bridge for FindLine
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE findline_bridge(FileLines, n_FileLines, len_FileLines, ParamName, len_ParamName, FoundLine, Line, LineNum, has_AryLen, AryLen) &
        BIND(C, NAME='findline_c')
        USE ISO_C_BINDING
        USE ROSCO_Helpers, ONLY : FindLine
        IMPLICIT NONE
        CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileLines(*)
        INTEGER(C_INT), VALUE :: n_FileLines
        INTEGER(C_INT), VALUE :: len_FileLines
        CHARACTER(KIND=C_CHAR), INTENT(IN) :: ParamName(*)
        INTEGER(C_INT), VALUE :: len_ParamName
        INTEGER(C_INT), INTENT(OUT) :: FoundLine
        CHARACTER(KIND=C_CHAR), INTENT(OUT) :: Line(*)
        INTEGER(C_INT), INTENT(OUT) :: LineNum
        INTEGER(C_INT), VALUE :: has_AryLen
        INTEGER(C_INT), VALUE :: AryLen
        CHARACTER(len_FileLines) :: local_FileLines(n_FileLines)
        INTEGER :: vit_i_FileLines, vit_j_FileLines
        CHARACTER(len_ParamName) :: local_ParamName
        INTEGER :: vit_i_ParamName
        CHARACTER(maxlinelength) :: local_Line
        INTEGER :: vit_i_Line
        ! Stage CHARACTER args into Fortran-shaped locals
        DO vit_j_FileLines = 1, n_FileLines
            DO vit_i_FileLines = 1, len_FileLines
                local_FileLines(vit_j_FileLines)(vit_i_FileLines:vit_i_FileLines) = &
                    FileLines((vit_j_FileLines - 1) * (len_FileLines) + vit_i_FileLines)
            END DO
        END DO
        DO vit_i_ParamName = 1, len_ParamName
            local_ParamName(vit_i_ParamName:vit_i_ParamName) = ParamName(vit_i_ParamName)
        END DO
        DO vit_i_Line = 1, maxlinelength
            local_Line(vit_i_Line:vit_i_Line) = Line(vit_i_Line)
        END DO
        IF (has_AryLen /= 0) THEN
        CALL FindLine(local_FileLines, local_ParamName, FoundLine, local_Line, LineNum, AryLen)
        ELSE
        CALL FindLine(local_FileLines, local_ParamName, FoundLine, local_Line, LineNum)
        END IF
        ! Copy CHARACTER results back to the C buffers
        DO vit_i_Line = 1, maxlinelength
            Line(vit_i_Line) = local_Line(vit_i_Line:vit_i_Line)
        END DO
    END SUBROUTINE findline_bridge
! VIT: Kernel callee bridge for GetWords
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE getwords_bridge(Line, len_Line, Words, len_Words, NumWords) &
        BIND(C, NAME='getwords_c')
        USE ISO_C_BINDING
        USE ROSCO_Helpers, ONLY : GetWords
        IMPLICIT NONE
        CHARACTER(KIND=C_CHAR), INTENT(IN) :: Line(*)
        INTEGER(C_INT), VALUE :: len_Line
        CHARACTER(KIND=C_CHAR), INTENT(OUT) :: Words(*)
        INTEGER(C_INT), VALUE :: len_Words
        INTEGER(C_INT), VALUE :: NumWords
        CHARACTER(len_Line) :: local_Line
        INTEGER :: vit_i_Line
        CHARACTER(len_Words) :: local_Words((NUMWORDS))
        INTEGER :: vit_i_Words, vit_j_Words
        ! Stage CHARACTER args into Fortran-shaped locals
        DO vit_i_Line = 1, len_Line
            local_Line(vit_i_Line:vit_i_Line) = Line(vit_i_Line)
        END DO
        DO vit_j_Words = 1, (NUMWORDS)
            DO vit_i_Words = 1, len_Words
                local_Words(vit_j_Words)(vit_i_Words:vit_i_Words) = &
                    Words((vit_j_Words - 1) * (len_Words) + vit_i_Words)
            END DO
        END DO
        CALL GetWords(local_Line, local_Words, NumWords)
        ! Copy CHARACTER results back to the C buffers
        DO vit_j_Words = 1, (NUMWORDS)
            DO vit_i_Words = 1, len_Words
                Words((vit_j_Words - 1) * (len_Words) + vit_i_Words) = &
                    local_Words(vit_j_Words)(vit_i_Words:vit_i_Words)
            END DO
        END DO
    END SUBROUTINE getwords_bridge
