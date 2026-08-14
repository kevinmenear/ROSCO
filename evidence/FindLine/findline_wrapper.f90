    SUBROUTINE FindLine(FileLines, ParamName, FoundLine, Line, LineNum, AryLen)
        USE ISO_C_BINDING
        IMPLICIT NONE
        LOGICAL, INTENT(OUT) :: FoundLine
        INTEGER(4), INTENT(OUT) :: LineNum
        INTEGER(4), INTENT(IN), OPTIONAL :: AryLen
        CHARACTER(*), INTENT(IN) :: FileLines(:)
        CHARACTER(*), INTENT(IN) :: ParamName
        CHARACTER(MAXLINELENGTH), INTENT(OUT) :: Line
        CHARACTER(KIND=C_CHAR) :: FileLines_c((LEN(FileLines)) * (SIZE(FileLines)))
        INTEGER :: vit_i_FileLines, vit_j_FileLines
        CHARACTER(KIND=C_CHAR) :: ParamName_c(LEN(ParamName))
        INTEGER :: vit_i_ParamName
        CHARACTER(KIND=C_CHAR) :: Line_c(maxlinelength)
        INTEGER :: vit_i_Line
        INTEGER(C_INT) :: FoundLine_lgc

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_AryLen_flag
        INTEGER(C_INT) :: AryLen_val

        has_AryLen_flag = 0
        AryLen_val = 0
        IF (PRESENT(AryLen)) THEN
            has_AryLen_flag = 1
            AryLen_val = INT(AryLen, C_INT)
        END IF
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_j_FileLines = 1, SIZE(FileLines)
            DO vit_i_FileLines = 1, LEN(FileLines)
                FileLines_c((vit_j_FileLines - 1) * (LEN(FileLines)) + vit_i_FileLines) = &
                    FileLines(vit_j_FileLines)(vit_i_FileLines:vit_i_FileLines)
            END DO
        END DO
        DO vit_i_ParamName = 1, LEN(ParamName)
            ParamName_c(vit_i_ParamName) = ParamName(vit_i_ParamName:vit_i_ParamName)
        END DO
        DO vit_i_Line = 1, maxlinelength
            Line_c(vit_i_Line) = Line(vit_i_Line:vit_i_Line)
        END DO
        FoundLine_lgc = 0
        CALL findline_c(FileLines_c, SIZE(FileLines), LEN(FileLines), ParamName_c, LEN(ParamName), FoundLine_lgc, Line_c, LineNum, has_AryLen_flag, AryLen_val)
        FoundLine = (FoundLine_lgc /= 0)
        ! Copy C_CHAR arrays back to CHARACTER args (INTENT OUT/INOUT)
        DO vit_i_Line = 1, maxlinelength
            Line(vit_i_Line:vit_i_Line) = Line_c(vit_i_Line)
        END DO
    END SUBROUTINE FindLine