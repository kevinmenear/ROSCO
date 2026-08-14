    ! Auto-generated interface for C++ implementation of FindLine
    INTERFACE
        SUBROUTINE findline_c(FileLines, n_FileLines, len_FileLines, ParamName, len_ParamName, FoundLine, Line, LineNum, has_AryLen, AryLen) BIND(C, NAME='findline_c')
            USE ISO_C_BINDING
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
        END SUBROUTINE findline_c
    END INTERFACE