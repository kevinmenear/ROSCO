! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Helper functions, primarily borrowed from NWTC_IO, for reading inputs and carrying out other helpful tasks

MODULE ROSCO_Helpers

    USE, INTRINSIC :: ISO_C_Binding
    USE ROSCO_Types
    USE CONSTANTS
    USE SysSubs


    IMPLICIT NONE

    ! Global Variables
    LOGICAL, PARAMETER     :: DEBUG_PARSING = .FALSE.      ! debug flag to output parsing information, set up Echo file later
    
    INTERFACE ParseInput                                                         ! Parses a character variable name and value from a string.
        MODULE PROCEDURE ParseInput_Str                                             ! Parses a character string from a string.
        MODULE PROCEDURE ParseInput_Dbl                                             ! Parses a double-precision REAL from a string.
        MODULE PROCEDURE ParseInput_Int                                             ! Parses an INTEGER from a string.
        MODULE PROCEDURE ParseInput_Int_Opt                                             ! Parses an INTEGER from a string.  Optional input.
        MODULE PROCEDURE ParseInput_Dbl_Opt                                             ! Parses an double-precision REAL from a string.  Optional input.
        MODULE PROCEDURE ParseInput_Str_Opt                                             ! Parses an character string from a string.  Optional input.
        ! MODULE PROCEDURE ParseInput_Log                                             ! Parses an LOGICAL from a string.
    END INTERFACE

    INTERFACE ParseAry                                                         ! Parse an array of numbers from a string.
        MODULE PROCEDURE ParseDbAry                                             ! Parse an array of double-precision REAL values.
        MODULE PROCEDURE ParseInAry                                             ! Parse an array of whole numbers.
        MODULE PROCEDURE ParseInAry_Opt                                         ! Parse an array of whole numbers. Optional inputs.
        MODULE PROCEDURE ParseDbAry_Opt                                         ! Parse an array of double-precision REAL values.  Optional inputs.
    END INTERFACE

    INTEGER(IntKi), PARAMETER       :: MaxLineLength    = 2048      ! characters
    INTEGER(IntKi), PARAMETER       :: MaxParamLength   = 200         ! characters, file paths can be long


    ! Auto-generated interface for C++ implementation of AddToList
    INTERFACE
        SUBROUTINE addtolist_c(list, element) BIND(C, NAME='addtolist_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), DIMENSION(:), ALLOCATABLE, INTENT(INOUT) :: list
            INTEGER(C_INT), VALUE :: element
        END SUBROUTINE addtolist_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of Conv2UC
    INTERFACE
        SUBROUTINE conv2uc_c(Str, len_Str) BIND(C, NAME='conv2uc_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: Str(*)
            INTEGER(C_INT), VALUE :: len_Str
        END SUBROUTINE conv2uc_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of GetPath
    INTERFACE
        SUBROUTINE getpath_c(GivenFil, len_GivenFil, PathName, len_PathName) BIND(C, NAME='getpath_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: GivenFil(*)
            INTEGER(C_INT), VALUE :: len_GivenFil
            CHARACTER(KIND=C_CHAR), INTENT(OUT) :: PathName(*)
            INTEGER(C_INT), VALUE :: len_PathName
        END SUBROUTINE getpath_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of GetRoot
    INTERFACE
        SUBROUTINE getroot_c(GivenFil, len_GivenFil, RootName, len_RootName) BIND(C, NAME='getroot_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: GivenFil(*)
            INTEGER(C_INT), VALUE :: len_GivenFil
            CHARACTER(KIND=C_CHAR), INTENT(OUT) :: RootName(*)
            INTEGER(C_INT), VALUE :: len_RootName
        END SUBROUTINE getroot_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of GetWords
    INTERFACE
        SUBROUTINE getwords_c(Line, len_Line, Words, len_Words, NumWords) BIND(C, NAME='getwords_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: Line(*)
            INTEGER(C_INT), VALUE :: len_Line
            CHARACTER(KIND=C_CHAR), INTENT(OUT) :: Words(*)
            INTEGER(C_INT), VALUE :: len_Words
            INTEGER(C_INT), VALUE :: NumWords
        END SUBROUTINE getwords_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of Int2LStr
    INTERFACE
        SUBROUTINE int2lstr_c(Num, Int2LStr_result) BIND(C, NAME='int2lstr_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), VALUE :: Num
            CHARACTER(KIND=C_CHAR), INTENT(OUT) :: Int2LStr_result(*)
        END SUBROUTINE int2lstr_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of NonDecreasing
    INTERFACE
        FUNCTION nondecreasing_c(Array, n_Array) BIND(C, NAME='nondecreasing_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), INTENT(IN) :: Array(*)
            INTEGER(C_INT), VALUE :: n_Array
            INTEGER(C_INT) :: nondecreasing_c
        END FUNCTION nondecreasing_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PathIsRelative
    INTERFACE
        FUNCTION pathisrelative_c(GivenFil, len_GivenFil) BIND(C, NAME='pathisrelative_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: GivenFil(*)
            INTEGER(C_INT), VALUE :: len_GivenFil
            INTEGER(C_INT) :: pathisrelative_c
        END FUNCTION pathisrelative_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of Read_OL_Input
    INTERFACE
        SUBROUTINE read_ol_input_c(OL_InputFileName, Unit_OL_Input, NumChannels, Channels_cptr, n_Channels_rows, n_Channels_cols, ErrVar) BIND(C, NAME='read_ol_input_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: OL_InputFileName(*)
            INTEGER(C_INT), VALUE :: Unit_OL_Input
            INTEGER(C_INT), VALUE :: NumChannels
            TYPE(C_PTR), INTENT(OUT) :: Channels_cptr
            INTEGER(C_INT), INTENT(OUT) :: n_Channels_rows
            INTEGER(C_INT), INTENT(OUT) :: n_Channels_cols
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE read_ol_input_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ChkParseData
    INTERFACE
        SUBROUTINE chkparsedata_c(Words, len_Words, ExpVarName, len_ExpVarName, FileName, len_FileName, FileLineNum, ErrVar) BIND(C, NAME='chkparsedata_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: Words(*)
            INTEGER(C_INT), VALUE :: len_Words
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: ExpVarName(*)
            INTEGER(C_INT), VALUE :: len_ExpVarName
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileName(*)
            INTEGER(C_INT), VALUE :: len_FileName
            INTEGER(C_INT), VALUE :: FileLineNum
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE chkparsedata_c
    END INTERFACE


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


    ! Auto-generated interface for C++ implementation of ParseDbAry_Opt
    INTERFACE
        SUBROUTINE parsedbary_opt_c(FileLines, n_FileLines, len_FileLines, ParamName, len_ParamName, Ary, AryLen, FileName, len_FileName, ErrVar, has_AllowDefault, AllowDefault, has_UnEc, UnEc) BIND(C, NAME='parsedbary_opt_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileLines(*)
            INTEGER(C_INT), VALUE :: n_FileLines
            INTEGER(C_INT), VALUE :: len_FileLines
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: ParamName(*)
            INTEGER(C_INT), VALUE :: len_ParamName
            REAL(C_DOUBLE), DIMENSION(:), ALLOCATABLE, INTENT(INOUT) :: Ary
            INTEGER(C_INT), VALUE :: AryLen
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileName(*)
            INTEGER(C_INT), VALUE :: len_FileName
            TYPE(C_PTR), VALUE :: ErrVar
            INTEGER(C_INT), VALUE :: has_AllowDefault
            INTEGER(C_INT), VALUE :: AllowDefault
            INTEGER(C_INT), VALUE :: has_UnEc
            INTEGER(C_INT), VALUE :: UnEc
        END SUBROUTINE parsedbary_opt_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ParseInAry_Opt
    INTERFACE
        SUBROUTINE parseinary_opt_c(FileLines, n_FileLines, len_FileLines, ParamName, len_ParamName, Ary, AryLen, FileName, len_FileName, ErrVar, has_AllowDefault, AllowDefault, has_UnEc, UnEc) BIND(C, NAME='parseinary_opt_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileLines(*)
            INTEGER(C_INT), VALUE :: n_FileLines
            INTEGER(C_INT), VALUE :: len_FileLines
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: ParamName(*)
            INTEGER(C_INT), VALUE :: len_ParamName
            INTEGER(C_INT), DIMENSION(:), ALLOCATABLE, INTENT(INOUT) :: Ary
            INTEGER(C_INT), VALUE :: AryLen
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileName(*)
            INTEGER(C_INT), VALUE :: len_FileName
            TYPE(C_PTR), VALUE :: ErrVar
            INTEGER(C_INT), VALUE :: has_AllowDefault
            INTEGER(C_INT), VALUE :: AllowDefault
            INTEGER(C_INT), VALUE :: has_UnEc
            INTEGER(C_INT), VALUE :: UnEc
        END SUBROUTINE parseinary_opt_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ParseInput_Dbl_Opt
    INTERFACE
        SUBROUTINE parseinput_dbl_opt_c(FileLines, n_FileLines, len_FileLines, VarName, len_VarName, Variable, FileName, len_FileName, ErrVar, has_AllowDefault, AllowDefault, has_UnEc, UnEc) BIND(C, NAME='parseinput_dbl_opt_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileLines(*)
            INTEGER(C_INT), VALUE :: n_FileLines
            INTEGER(C_INT), VALUE :: len_FileLines
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: VarName(*)
            INTEGER(C_INT), VALUE :: len_VarName
            REAL(C_DOUBLE), INTENT(INOUT) :: Variable
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: FileName(*)
            INTEGER(C_INT), VALUE :: len_FileName
            TYPE(C_PTR), VALUE :: ErrVar
            INTEGER(C_INT), VALUE :: has_AllowDefault
            INTEGER(C_INT), VALUE :: AllowDefault
            INTEGER(C_INT), VALUE :: has_UnEc
            INTEGER(C_INT), VALUE :: UnEc
        END SUBROUTINE parseinput_dbl_opt_c
    END INTERFACE

CONTAINS

    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Int(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(MaxLineLength)                    :: Line
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )       :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )       :: FileName   ! Input file unit
        INTEGER(IntKi),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(ErrorVariables),   INTENT(INOUT)       :: ErrVar   ! Current line of input
        CHARACTER(MaxParamLength)                   :: Words       (2)               ! The two "words" parsed from the line

        INTEGER(IntKi),             INTENT(INOUT)   :: Variable   ! Variable
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Int

    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Int_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(*),           INTENT(IN   ), DIMENSION(:) :: FileLines   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: FileName   ! Input file unit
        TYPE(ErrorVariables),   INTENT(INOUT)               :: ErrVar   ! Current line of input
        INTEGER(IntKi),         INTENT(INOUT)               :: Variable   ! Variable
        Integer(IntKi), OPTIONAL, INTENT(IN   )               :: UnEc   ! Variable


        ! Flag (usually control mode) specifying whether default is allowed, 0 - yes, nonzero - no
        LOGICAL, OPTIONAL,      INTENT(IN   )        :: AllowDefault   
        
        INTEGER(IntKi)                          :: CurLine   ! Current line of input
        CHARACTER(MaxParamLength)               :: Words       (2)               ! The two "words" parsed from the line
        CHARACTER(MaxParamLength)               :: VarNameUC
        CHARACTER(MaxLineLength)                :: Line
        INTEGER(IntKi)                          :: ErrStatLcl           ! Error status local to this routine.
        INTEGER(IntKi)                          :: I, VarLineIndex                    ! Line indexer
        LOGICAL                                 :: AllowDefault_, FoundLine
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInput_Int_Opt'


        ! Figure out if we allow default
        AllowDefault_ = .TRUE.
        if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault    

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            CALL FindLine(FileLines, VarName, FoundLine, Line, CurLine)

            ! Separate line again
            CALL GetWords ( Line, Words, 2 )  

            ! PRINT *, "Line: ", Line

            ! Print warning with default
            IF (.NOT. FoundLine) THEN
                IF (.NOT. AllowDefault_) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Missing or default values are not allowed for '//TRIM( VarName )//'. Please check control modes.'
                    RETURN
                ENDIF

                Variable = 0     ! Default of integer inputs is 0 for now
                PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//" in input file.  Using default value of ", Variable
            ENDIF

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0 .AND. FoundLine) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            IF ( PRESENT(UnEc))  THEN
                IF ( UnEc > 0 )  WRITE (UnEc,*)  CurLine, Tab, VarName, Tab, Variable
            END IF

        END IF

    END subroutine ParseInput_Int_Opt

     !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    SUBROUTINE ParseInput_Dbl_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(8), INTENT(INOUT) :: Variable
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        LOGICAL, INTENT(IN), OPTIONAL :: AllowDefault
        INTEGER(4), INTENT(IN), OPTIONAL :: UnEc
        CHARACTER(*), INTENT(IN) :: FileLines(:)
        CHARACTER(*), INTENT(IN) :: VarName
        CHARACTER(*), INTENT(IN) :: FileName
        CHARACTER(KIND=C_CHAR) :: FileLines_c((LEN(FileLines)) * (SIZE(FileLines)))
        INTEGER :: vit_i_FileLines, vit_j_FileLines
        CHARACTER(KIND=C_CHAR) :: VarName_c(LEN(VarName))
        INTEGER :: vit_i_VarName
        CHARACTER(KIND=C_CHAR) :: FileName_c(LEN(FileName))
        INTEGER :: vit_i_FileName
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_AllowDefault_flag
        INTEGER(C_INT) :: AllowDefault_val
        INTEGER(C_INT) :: has_UnEc_flag
        INTEGER(C_INT) :: UnEc_val

        has_AllowDefault_flag = 0
        AllowDefault_val = 0
        IF (PRESENT(AllowDefault)) THEN
            has_AllowDefault_flag = 1
            AllowDefault_val = MERGE(1_C_INT, 0_C_INT, AllowDefault)
        END IF
        has_UnEc_flag = 0
        UnEc_val = 0
        IF (PRESENT(UnEc)) THEN
            has_UnEc_flag = 1
            UnEc_val = INT(UnEc, C_INT)
        END IF
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_j_FileLines = 1, SIZE(FileLines)
            DO vit_i_FileLines = 1, LEN(FileLines)
                FileLines_c((vit_j_FileLines - 1) * (LEN(FileLines)) + vit_i_FileLines) = &
                    FileLines(vit_j_FileLines)(vit_i_FileLines:vit_i_FileLines)
            END DO
        END DO
        DO vit_i_VarName = 1, LEN(VarName)
            VarName_c(vit_i_VarName) = VarName(vit_i_VarName:vit_i_VarName)
        END DO
        DO vit_i_FileName = 1, LEN(FileName)
            FileName_c(vit_i_FileName) = FileName(vit_i_FileName:vit_i_FileName)
        END DO
        CALL parseinput_dbl_opt_c(FileLines_c, SIZE(FileLines), LEN(FileLines), VarName_c, LEN(VarName), Variable, FileName_c, LEN(FileName), C_LOC(ErrVar_view), has_AllowDefault_flag, AllowDefault_val, has_UnEc_flag, UnEc_val)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ParseInput_Dbl_Opt

        !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    subroutine ParseInput_Str_Opt(FileLines, VarName, Variable, FileName, ErrVar, AllowDefault, UnEc)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(*),           INTENT(IN   ), DIMENSION(:) :: FileLines   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )               :: FileName   ! Input file unit
        TYPE(ErrorVariables),   INTENT(INOUT)               :: ErrVar   ! Current line of input
        CHARACTER(*),           INTENT(INOUT)               :: Variable   ! Variable
        Integer(IntKi), OPTIONAL, INTENT(IN   )               :: UnEc   ! Variable

        ! Flag (usually control mode) specifying whether default is allowed, 0 - yes, nonzero - no
        LOGICAL, OPTIONAL,      INTENT(IN   )        :: AllowDefault   
        
        INTEGER(IntKi)                          :: CurLine   ! Current line of input
        CHARACTER(MaxParamLength)               :: Words       (2)               ! The two "words" parsed from the line
        CHARACTER(MaxParamLength)               :: VarNameUC
        CHARACTER(MaxLineLength)                :: Line
        INTEGER(IntKi)                          :: ErrStatLcl           ! Error status local to this routine.
        INTEGER(IntKi)                          :: I, VarLineIndex                    ! Line indexer
        LOGICAL                                 :: AllowDefault_, FoundLine
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInput_Str_Opt'


        ! Figure out if we allow default
        AllowDefault_ = .TRUE.
        if (PRESENT(AllowDefault)) AllowDefault_ = AllowDefault    

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            CALL FindLine(FileLines, VarName, FoundLine, Line, CurLine)


            ! Separate line again
            CALL GetWords ( Line, Words, 2 )  

            ! PRINT *, "Line: ", TRIM(Line)

            ! Print warning with default
            IF (.NOT. FoundLine) THEN
                IF (.NOT. AllowDefault_) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Missing or default values are not allowed for '//TRIM( VarName )//'. Please check control modes.'
                    RETURN
                ENDIF

                Variable = 'unused'     ! Default of string input is unused for now
                PRINT *, "ROSCO Warning: Did not find "//TRIM( VarName )//" in input file.  Using default value of ", TRIM(Variable)
            ENDIF

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            IF (ErrVar%aviFAIL >= 0 .AND. FoundLine) THEN        

                ! Read the variable
                READ (Words(1),'(A)',IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            IF ( PRESENT(UnEc))  THEN
                IF ( UnEc > 0 )  WRITE (UnEc,*)  CurLine, Tab, VarName, Tab, Variable
            END IF

        END IF

    END subroutine ParseInput_Str_Opt


    !=======================================================================
    ! Parse double input, this is a copy of ParseInput_Int and a change in the variable definitions
    subroutine ParseInput_Dbl(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(IntKi),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(20)                           :: Words       (2)               ! The two "words" parsed from the line
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        REAL(DbKi),             INTENT(INOUT)      :: Variable   ! Variable
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
            print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Dbl

    !=======================================================================
    ! Parse string input, this is a copy of ParseInput_Int and a change in the variable definitions
    subroutine ParseInput_Str(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE ROSCO_Types, ONLY : ErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(IntKi),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(200)                          :: Words       (2)               ! The two "words" parsed from the line
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        CHARACTER(*),           INTENT(INOUT)   :: Variable   ! Variable
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            if (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),'(A)',IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid STRING value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Str

!=======================================================================
!> This subroutine parses the specified line of text for AryLen REAL values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
    SUBROUTINE ParseDbAry ( Un, LineNum, ParamName, Ary, AryLen, FileName, ErrVar, CheckName )

        USE ROSCO_Types, ONLY : ErrorVariables

        ! Arguments declarations.
        INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
        INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

        REAL(DbKi), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

        INTEGER(IntKi),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
        CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


        CHARACTER(*),           INTENT(IN   )   :: ParamName                       !< The array name we are trying to fill.

        TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input

        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName


        ! Local declarations.

        CHARACTER(1024)                         :: Line
        INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.
        INTEGER(IntKi)                              :: i

        CHARACTER(200), ALLOCATABLE             :: Words_Ary       (:)               ! The array "words" parsed from the line.
        CHARACTER(1024)                         :: Debug_String 
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseDbAry'
        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN
            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Allocate array and handle errors
            ALLOCATE ( Ary(AryLen) , STAT=ErrStatLcl )
            IF ( ErrStatLcl /= 0 ) THEN
                IF ( ALLOCATED(Ary) ) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( ParamName )//' array; array was already allocated.'
                ELSE
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( ParamName )//' array.'
                END IF
            END IF
        
            ! Allocate words array
            ALLOCATE ( Words_Ary( AryLen + 1 ) , STAT=ErrStatLcl )
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Fatal error allocating memory for the Words array.'
                CALL Cleanup()
                RETURN
            ENDIF

            ! Separate line string into AryLen + 1 words, should include variable name
            CALL GetWords ( Line, Words_Ary, AryLen + 1 )  

            ! Debug Output
            IF (DEBUG_PARSING) THEN
                Debug_String = ''
                DO i = 1,AryLen+1
                    Debug_String = TRIM(Debug_String)//TRIM(Words_Ary(i))
                    IF (i < AryLen + 1) THEN
                        Debug_String = TRIM(Debug_String)//','
                    END IF
                END DO
                print *, 'Read: '//TRIM(Debug_String)//' on line ', LineNum
            END IF

            ! Check that Variable Name is at the end of Words, will also check length of array
            IF (CheckName_) THEN
                CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), ParamName, FileName, LineNum, ErrVar )
            END IF
        
            ! Read array
            READ (Line,*,IOSTAT=ErrStatLcl)  Ary
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                                //TRIM( FileName )//'".'//NewLine//  &
                                ' >> The "'//TRIM( ParamName )//'" array was not assigned valid REAL values on line #' &
                                //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                                //'    "'//TRIM( Line )//'"' 
                RETURN
                CALL Cleanup()         
            ENDIF

            LineNum = LineNum + 1
            CALL Cleanup()
        ENDIF

        RETURN

        !=======================================================================
        CONTAINS
        !=======================================================================
            SUBROUTINE Cleanup ( )

                ! This subroutine cleans up the parent routine before exiting.

                ! Deallocate the Words array if it had been allocated.

                IF ( ALLOCATED( Words_Ary ) ) DEALLOCATE( Words_Ary )


                RETURN

            END SUBROUTINE Cleanup

  END SUBROUTINE ParseDbAry

  !=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
  SUBROUTINE ParseInAry ( Un, LineNum, ParamName, Ary, AryLen, FileName, ErrVar, CheckName )

    USE ROSCO_Types, ONLY : ErrorVariables

    ! Arguments declarations.
    INTEGER(IntKi),             INTENT(IN   )   :: Un   ! Input file unit
    INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

    INTEGER(IntKi), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

    INTEGER(IntKi),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
    CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


    CHARACTER(*),           INTENT(IN   )   :: ParamName                       !< The array name we are trying to fill.

    TYPE(ErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input

    LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

    ! Local declarations.

    CHARACTER(1024)                         :: Line
    INTEGER(IntKi)                              :: ErrStatLcl                    ! Error status local to this routine.
    INTEGER(IntKi)                              :: i

    CHARACTER(200), ALLOCATABLE             :: Words_Ary       (:)               ! The array "words" parsed from the line.
    CHARACTER(1024)                         :: Debug_String 
    CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInAry'

    LOGICAL                                 :: CheckName_

    ! Figure out if we're checking the name, default to .TRUE.
    CheckName_ = .TRUE.
    if (PRESENT(CheckName)) CheckName_ = CheckName    

    ! If we've already failed, don't read anything
    IF (ErrVar%aviFAIL >= 0) THEN
        ! Read the whole line as a string
        READ(Un, '(A)') Line

        ! Allocate array and handle errors
        ALLOCATE ( Ary(AryLen) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 ) THEN
            IF ( ALLOCATED(Ary) ) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( ParamName )//' array; array was already allocated.'
            ELSE
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( ParamName )//' array.'
            END IF
        END IF
    
        ! Allocate words array
        ALLOCATE ( Words_Ary( AryLen + 1 ) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 )  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = RoutineName//':Fatal error allocating memory for the Words array.'
            CALL Cleanup()
            RETURN
        ENDIF

        ! Separate line string into AryLen + 1 words, should include variable name
        CALL GetWords ( Line, Words_Ary, AryLen + 1 )  

        ! Debug Output
        IF (DEBUG_PARSING) THEN
            Debug_String = ''
            DO i = 1,AryLen+1
                Debug_String = TRIM(Debug_String)//TRIM(Words_Ary(i))
                IF (i < AryLen + 1) THEN
                    Debug_String = TRIM(Debug_String)//','
                END IF
            END DO
            print *, 'Read: '//TRIM(Debug_String)//' on line ', LineNum
        END IF

        ! Check that Variable Name is at the end of Words, will also check length of array
        IF (CheckName_) THEN
            CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), ParamName, FileName, LineNum, ErrVar )
        END IF
    
        ! Read array
        READ (Line,*,IOSTAT=ErrStatLcl)  Ary
        IF ( ErrStatLcl /= 0 )  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                            //TRIM( FileName )//'".'//NewLine//  &
                            ' >> The "'//TRIM( ParamName )//'" array was not assigned valid INTEGER values on line #' &
                            //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                            //'    "'//TRIM( Line )//'"' 
            RETURN
            CALL Cleanup()         
        ENDIF

    !  IF ( PRESENT(UnEc) )  THEN
    !     IF ( UnEc > 0 )  WRITE (UnEc,'(A)')  TRIM( FileInfo%Lines(LineNum) )
    !  END IF

        LineNum = LineNum + 1
        CALL Cleanup()
    ENDIF

    RETURN

    !=======================================================================
    CONTAINS
    !=======================================================================
        SUBROUTINE Cleanup ( )

            ! This subroutine cleans up the parent routine before exiting.

            ! Deallocate the Words array if it had been allocated.

            IF ( ALLOCATED( Words_Ary ) ) DEALLOCATE( Words_Ary )


            RETURN

        END SUBROUTINE Cleanup

END SUBROUTINE ParseInAry

!=======================================================================

!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
    SUBROUTINE ParseInAry_Opt(FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        INTEGER(4), INTENT(INOUT), ALLOCATABLE :: Ary(:)
        INTEGER, INTENT(IN) :: AryLen
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        LOGICAL, INTENT(IN), OPTIONAL :: AllowDefault
        INTEGER(4), INTENT(IN), OPTIONAL :: UnEc
        CHARACTER(*), INTENT(IN) :: FileLines(:)
        CHARACTER(*), INTENT(IN) :: ParamName
        CHARACTER(*), INTENT(IN) :: FileName
        CHARACTER(KIND=C_CHAR) :: FileLines_c((LEN(FileLines)) * (SIZE(FileLines)))
        INTEGER :: vit_i_FileLines, vit_j_FileLines
        CHARACTER(KIND=C_CHAR) :: ParamName_c(LEN(ParamName))
        INTEGER :: vit_i_ParamName
        CHARACTER(KIND=C_CHAR) :: FileName_c(LEN(FileName))
        INTEGER :: vit_i_FileName
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_AllowDefault_flag
        INTEGER(C_INT) :: AllowDefault_val
        INTEGER(C_INT) :: has_UnEc_flag
        INTEGER(C_INT) :: UnEc_val

        has_AllowDefault_flag = 0
        AllowDefault_val = 0
        IF (PRESENT(AllowDefault)) THEN
            has_AllowDefault_flag = 1
            AllowDefault_val = MERGE(1_C_INT, 0_C_INT, AllowDefault)
        END IF
        has_UnEc_flag = 0
        UnEc_val = 0
        IF (PRESENT(UnEc)) THEN
            has_UnEc_flag = 1
            UnEc_val = INT(UnEc, C_INT)
        END IF
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
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
        DO vit_i_FileName = 1, LEN(FileName)
            FileName_c(vit_i_FileName) = FileName(vit_i_FileName:vit_i_FileName)
        END DO
        CALL parseinary_opt_c(FileLines_c, SIZE(FileLines), LEN(FileLines), ParamName_c, LEN(ParamName), Ary, AryLen, FileName_c, LEN(FileName), C_LOC(ErrVar_view), has_AllowDefault_flag, AllowDefault_val, has_UnEc_flag, UnEc_val)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ParseInAry_Opt

!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
    SUBROUTINE ParseDbAry_Opt(FileLines, ParamName, Ary, AryLen, FileName, ErrVar, AllowDefault, UnEc)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(8), INTENT(INOUT), ALLOCATABLE :: Ary(:)
        INTEGER, INTENT(IN) :: AryLen
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        LOGICAL, INTENT(IN), OPTIONAL :: AllowDefault
        INTEGER(4), INTENT(IN), OPTIONAL :: UnEc
        CHARACTER(*), INTENT(IN) :: FileLines(:)
        CHARACTER(*), INTENT(IN) :: ParamName
        CHARACTER(*), INTENT(IN) :: FileName
        CHARACTER(KIND=C_CHAR) :: FileLines_c((LEN(FileLines)) * (SIZE(FileLines)))
        INTEGER :: vit_i_FileLines, vit_j_FileLines
        CHARACTER(KIND=C_CHAR) :: ParamName_c(LEN(ParamName))
        INTEGER :: vit_i_ParamName
        CHARACTER(KIND=C_CHAR) :: FileName_c(LEN(FileName))
        INTEGER :: vit_i_FileName
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_AllowDefault_flag
        INTEGER(C_INT) :: AllowDefault_val
        INTEGER(C_INT) :: has_UnEc_flag
        INTEGER(C_INT) :: UnEc_val

        has_AllowDefault_flag = 0
        AllowDefault_val = 0
        IF (PRESENT(AllowDefault)) THEN
            has_AllowDefault_flag = 1
            AllowDefault_val = MERGE(1_C_INT, 0_C_INT, AllowDefault)
        END IF
        has_UnEc_flag = 0
        UnEc_val = 0
        IF (PRESENT(UnEc)) THEN
            has_UnEc_flag = 1
            UnEc_val = INT(UnEc, C_INT)
        END IF
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
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
        DO vit_i_FileName = 1, LEN(FileName)
            FileName_c(vit_i_FileName) = FileName(vit_i_FileName:vit_i_FileName)
        END DO
        CALL parsedbary_opt_c(FileLines_c, SIZE(FileLines), LEN(FileLines), ParamName_c, LEN(ParamName), Ary, AryLen, FileName_c, LEN(FileName), C_LOC(ErrVar_view), has_AllowDefault_flag, AllowDefault_val, has_UnEc_flag, UnEc_val)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ParseDbAry_Opt



!=======================================================================

 !> This subroutine checks the data to be parsed to make sure it finds
    !! the expected variable name and an associated value.
    SUBROUTINE ChkParseData(Words, ExpVarName, FileName, FileLineNum, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        INTEGER(4), INTENT(IN) :: FileLineNum
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        CHARACTER(*), INTENT(IN) :: Words(2)
        CHARACTER(*), INTENT(IN) :: ExpVarName
        CHARACTER(*), INTENT(IN) :: FileName
        CHARACTER(KIND=C_CHAR) :: Words_c((LEN(Words)) * ((2)))
        INTEGER :: vit_i_Words, vit_j_Words
        CHARACTER(KIND=C_CHAR) :: ExpVarName_c(LEN(ExpVarName))
        INTEGER :: vit_i_ExpVarName
        CHARACTER(KIND=C_CHAR) :: FileName_c(LEN(FileName))
        INTEGER :: vit_i_FileName
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_j_Words = 1, (2)
            DO vit_i_Words = 1, LEN(Words)
                Words_c((vit_j_Words - 1) * (LEN(Words)) + vit_i_Words) = &
                    Words(vit_j_Words)(vit_i_Words:vit_i_Words)
            END DO
        END DO
        DO vit_i_ExpVarName = 1, LEN(ExpVarName)
            ExpVarName_c(vit_i_ExpVarName) = ExpVarName(vit_i_ExpVarName:vit_i_ExpVarName)
        END DO
        DO vit_i_FileName = 1, LEN(FileName)
            FileName_c(vit_i_FileName) = FileName(vit_i_FileName:vit_i_FileName)
        END DO
        CALL chkparsedata_c(Words_c, LEN(Words), ExpVarName_c, LEN(ExpVarName), FileName_c, LEN(FileName), FileLineNum, C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ChkParseData

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

!=======================================================================
subroutine ReadEmptyLine(Un,CurLine)
    INTEGER(IntKi),         INTENT(IN   )          :: Un   ! Input file unit
    INTEGER(IntKi),         INTENT(INOUT)          :: CurLine   ! Current line of input

    CHARACTER(1024)                            :: Line

    READ(Un, '(A)') Line
    CurLine = CurLine + 1

END subroutine ReadEmptyLine

!=======================================================================
!> This subroutine is used to get the NumWords "words" from a line of text.
!! It uses spaces, tabs, commas, semicolons, single quotes, and double quotes ("whitespace")
!! as word separators. If there aren't NumWords in the line, the remaining array elements will remain empty.
!! Use CountWords (nwtc_io::countwords) to count the number of words in a line.
    SUBROUTINE GetWords(Line, Words, NumWords)
        USE ISO_C_BINDING
        IMPLICIT NONE
        INTEGER, INTENT(IN) :: NumWords
        CHARACTER(*), INTENT(IN) :: Line
        CHARACTER(*), INTENT(OUT) :: Words(NUMWORDS)
        CHARACTER(KIND=C_CHAR) :: Line_c(LEN(Line))
        INTEGER :: vit_i_Line
        CHARACTER(KIND=C_CHAR) :: Words_c((LEN(Words)) * ((NUMWORDS)))
        INTEGER :: vit_i_Words, vit_j_Words
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_i_Line = 1, LEN(Line)
            Line_c(vit_i_Line) = Line(vit_i_Line:vit_i_Line)
        END DO
        DO vit_j_Words = 1, (NUMWORDS)
            DO vit_i_Words = 1, LEN(Words)
                Words_c((vit_j_Words - 1) * (LEN(Words)) + vit_i_Words) = &
                    Words(vit_j_Words)(vit_i_Words:vit_i_Words)
            END DO
        END DO
        CALL getwords_c(Line_c, LEN(Line), Words_c, LEN(Words), NumWords)
        ! Copy C_CHAR arrays back to CHARACTER args (INTENT OUT/INOUT)
        DO vit_j_Words = 1, (NUMWORDS)
            DO vit_i_Words = 1, LEN(Words)
                Words(vit_j_Words)(vit_i_Words:vit_i_Words) = &
                    Words_c((vit_j_Words - 1) * (LEN(Words)) + vit_i_Words)
            END DO
        END DO
    END SUBROUTINE GetWords
!=======================================================================
!> Let's parse the path name from the name of the given file.
!! We'll count everything before (and including) the last "\" or "/".
    SUBROUTINE GetPath(GivenFil, PathName)
        USE ISO_C_BINDING
        IMPLICIT NONE
        CHARACTER(*), INTENT(IN) :: GivenFil
        CHARACTER(*), INTENT(OUT) :: PathName
        CHARACTER(KIND=C_CHAR) :: GivenFil_c(LEN(GivenFil))
        INTEGER :: vit_i_GivenFil
        CHARACTER(KIND=C_CHAR) :: PathName_c(LEN(PathName))
        INTEGER :: vit_i_PathName
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_i_GivenFil = 1, LEN(GivenFil)
            GivenFil_c(vit_i_GivenFil) = GivenFil(vit_i_GivenFil:vit_i_GivenFil)
        END DO
        DO vit_i_PathName = 1, LEN(PathName)
            PathName_c(vit_i_PathName) = PathName(vit_i_PathName:vit_i_PathName)
        END DO
        CALL getpath_c(GivenFil_c, LEN(GivenFil), PathName_c, LEN(PathName))
        ! Copy C_CHAR arrays back to CHARACTER args (INTENT OUT/INOUT)
        DO vit_i_PathName = 1, LEN(PathName)
            PathName(vit_i_PathName:vit_i_PathName) = PathName_c(vit_i_PathName)
        END DO
    END SUBROUTINE GetPath
!=======================================================================
!> Let's parse the root file name from the name of the given file.
!! We'll count everything after the last period as the extension.
!! Borrowed from NWTC_IO...thanks!

    SUBROUTINE GetRoot(GivenFil, RootName)
        USE ISO_C_BINDING
        IMPLICIT NONE
        CHARACTER(*), INTENT(IN) :: GivenFil
        CHARACTER(*), INTENT(OUT) :: RootName
        CHARACTER(KIND=C_CHAR) :: GivenFil_c(LEN(GivenFil))
        INTEGER :: vit_i_GivenFil
        CHARACTER(KIND=C_CHAR) :: RootName_c(LEN(RootName))
        INTEGER :: vit_i_RootName
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_i_GivenFil = 1, LEN(GivenFil)
            GivenFil_c(vit_i_GivenFil) = GivenFil(vit_i_GivenFil:vit_i_GivenFil)
        END DO
        DO vit_i_RootName = 1, LEN(RootName)
            RootName_c(vit_i_RootName) = RootName(vit_i_RootName:vit_i_RootName)
        END DO
        CALL getroot_c(GivenFil_c, LEN(GivenFil), RootName_c, LEN(RootName))
        ! Copy C_CHAR arrays back to CHARACTER args (INTENT OUT/INOUT)
        DO vit_i_RootName = 1, LEN(RootName)
            RootName(vit_i_RootName:vit_i_RootName) = RootName_c(vit_i_RootName)
        END DO
    END SUBROUTINE GetRoot
!=======================================================================
!> This routine determines if the given file name is absolute or relative.
!! We will consider an absolute path one that satisfies one of the
!! following four criteria:
!!     1. It contains ":/"
!!     2. It contains ":\"
!!     3. It starts with "/"
!!     4. It starts with "\"
!!   
!! All others are considered relative.
    FUNCTION PathIsRelative(GivenFil) RESULT(PathIsRelative_result)
        USE ISO_C_BINDING
        IMPLICIT NONE
        CHARACTER(*), INTENT(IN) :: GivenFil
        LOGICAL :: PathIsRelative_result
        CHARACTER(KIND=C_CHAR) :: GivenFil_c(LEN(GivenFil))
        INTEGER :: vit_i_GivenFil
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_i_GivenFil = 1, LEN(GivenFil)
            GivenFil_c(vit_i_GivenFil) = GivenFil(vit_i_GivenFil:vit_i_GivenFil)
        END DO
        PathIsRelative_result = pathisrelative_c(GivenFil_c, LEN(GivenFil))
    END FUNCTION PathIsRelative
!=======================================================================
! ------------------------------------------------------
    ! Read Open Loop Control Inputs
    ! 
    ! Timeseries or lookup tables of the form
    ! index (time or wind speed)   channel_1 \t channel_2 \t channel_3 ...
    ! This could be used to read any group of data of unspecified length ...
    SUBROUTINE Read_OL_Input(OL_InputFileName, Unit_OL_Input, NumChannels, Channels, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        INTEGER(4), INTENT(IN) :: Unit_OL_Input
        INTEGER(4), INTENT(IN) :: NumChannels
        REAL(8), INTENT(OUT), ALLOCATABLE :: Channels(:,:)
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        CHARACTER(1024), INTENT(IN) :: OL_InputFileName
        CHARACTER(KIND=C_CHAR) :: OL_InputFileName_c(1024)
        INTEGER :: vit_i_OL_InputFileName
        ! VIT allocate-on-return: the C++ side discovers the extent,
        ! allocates, and hands back a pointer; the wrapper copies into the
        ! Fortran ALLOCATABLE and frees. C_NULL_PTR means 'not allocated'.
        INTERFACE
            SUBROUTINE vit_free(vit_p) BIND(C, NAME='vit_free')
                USE ISO_C_BINDING
                TYPE(C_PTR), VALUE :: vit_p
            END SUBROUTINE vit_free
        END INTERFACE
        TYPE(C_PTR) :: Channels_cptr
        INTEGER(C_INT) :: n_Channels_rows
        INTEGER(C_INT) :: n_Channels_cols
        REAL(8), POINTER :: Channels_fptr(:,:)
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_i_OL_InputFileName = 1, 1024
            OL_InputFileName_c(vit_i_OL_InputFileName) = OL_InputFileName(vit_i_OL_InputFileName:vit_i_OL_InputFileName)
        END DO
        Channels_cptr = C_NULL_PTR
        n_Channels_rows = 0
        n_Channels_cols = 0
        CALL read_ol_input_c(OL_InputFileName_c, Unit_OL_Input, NumChannels, Channels_cptr, n_Channels_rows, n_Channels_cols, C_LOC(ErrVar_view))
        IF (C_ASSOCIATED(Channels_cptr)) THEN
            CALL C_F_POINTER(Channels_cptr, Channels_fptr, [n_Channels_rows, n_Channels_cols])
            ALLOCATE(Channels(n_Channels_rows, n_Channels_cols))
            Channels = Channels_fptr
            CALL vit_free(Channels_cptr)
        END IF
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE Read_OL_Input

!=======================================================================
!> This routine returns the next unit number greater than 9 that is not currently in use.
!! If it cannot find any unit between 10 and 99 that is available, it either aborts or returns an appropriate error status/message.   
   SUBROUTINE GetNewUnit ( UnIn, ErrVar )



      ! Argument declarations.

   INTEGER,        INTENT(OUT)            :: UnIn                                         !< Logical unit for the file.                           !< The error message, if an error occurred
   TYPE(ErrorVariables), INTENT(INOUT)             :: ErrVar


      ! Local declarations.

   INTEGER                                :: Un                                           ! Unit number
   LOGICAL                                :: Opened                                       ! Flag indicating whether or not a file is opened.
   INTEGER(IntKi), PARAMETER              :: StartUnit = 10                               ! Starting unit number to check (numbers less than 10 reserved)
   ! NOTE: maximum unit numbers in fortran 90 and later is 2**31-1.  However, there are limits within the OS.
   !     macos -- 256  (change with ulimit -n)
   !     linux -- 1024 (change with ulimit -n)
   !     windows -- 512 (not sure how to change -- ADP)
   INTEGER(IntKi), PARAMETER              :: MaxUnit   = 1024                             ! The maximum unit number available (or 10 less than the number of files you want to have open at a time)

      ! Initialize subroutine outputs

   Un = StartUnit

      ! See if unit is connected to an open file. Check the next largest number until it is not opened.

   DO

      INQUIRE ( UNIT=Un , OPENED=Opened )

      IF ( .NOT. Opened )  EXIT
      Un = Un + 1

      IF ( Un > MaxUnit ) THEN

         ErrVar%aviFAIL = -1
         ErrVar%ErrMsg  = 'GetNewUnit() was unable to find an open file unit specifier between '//TRIM(Int2LStr(StartUnit)) &
                                                                            //' and '//TRIM(Int2LStr(MaxUnit))//'.'

         EXIT           ! stop searching now

      END IF


   END DO

   UnIn = Un

   RETURN
   END SUBROUTINE GetNewUnit

!=======================================================================
!> This function returns a character string encoded with the time in the form "hh:mm:ss".
    FUNCTION CurTime( )

    ! Function declaration.

    CHARACTER(8)                 :: CurTime                                      !< The current time in the form "hh:mm:ss".


    ! Local declarations.

    CHARACTER(10)                :: CTime                                        ! String to hold the returned value from the DATE_AND_TIME subroutine call.



    CALL DATE_AND_TIME ( TIME=CTime )

    CurTime = CTime(1:2)//':'//CTime(3:4)//':'//CTime(5:6)


    RETURN
    END FUNCTION CurTime

!=======================================================================
! This function checks whether an array is non-decreasing
    FUNCTION NonDecreasing(Array) RESULT(NonDecreasing_result)
        USE ISO_C_BINDING
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: Array(:)
        LOGICAL :: NonDecreasing_result
        NonDecreasing_result = nondecreasing_c(Array, SIZE(Array))
    END FUNCTION NonDecreasing

!=======================================================================
!> This routine converts all the text in a string to upper case.
    SUBROUTINE Conv2UC(Str)
        USE ISO_C_BINDING
        IMPLICIT NONE
        CHARACTER(*), INTENT(INOUT) :: Str
        CHARACTER(KIND=C_CHAR) :: Str_c(LEN(Str))
        INTEGER :: vit_i_Str
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_i_Str = 1, LEN(Str)
            Str_c(vit_i_Str) = Str(vit_i_Str:vit_i_Str)
        END DO
        CALL conv2uc_c(Str_c, LEN(Str))
        ! Copy C_CHAR arrays back to CHARACTER args (INTENT OUT/INOUT)
        DO vit_i_Str = 1, LEN(Str)
            Str(vit_i_Str:vit_i_Str) = Str_c(vit_i_Str)
        END DO
    END SUBROUTINE Conv2UC

!=======================================================================
     !> This function returns a left-adjusted string representing the passed numeric value. 
    !! It eliminates trailing zeroes and even the decimal point if it is not a fraction. \n
    !! Use Num2LStr (nwtc_io::num2lstr) instead of directly calling a specific routine in the generic interface.   
    FUNCTION Int2LStr(Num) RESULT(Int2LStr_result)
        USE ISO_C_BINDING
        IMPLICIT NONE
        INTEGER, INTENT(IN) :: Num
        CHARACTER(11) :: Int2LStr_result
        CHARACTER(KIND=C_CHAR) :: Int2LStr_result_c(11)
        INTEGER :: vit_i_result
        CALL int2lstr_c(Num, Int2LStr_result_c)
        DO vit_i_result = 1, 11
            Int2LStr_result(vit_i_result:vit_i_result) = Int2LStr_result_c(vit_i_result)
        END DO
    END FUNCTION Int2LStr

!=======================================================================

    SUBROUTINE AddToList(list, element)
        USE ISO_C_BINDING
        IMPLICIT NONE
        INTEGER(4), INTENT(INOUT), ALLOCATABLE :: list(:)
        INTEGER(4), INTENT(IN) :: element
        CALL addtolist_c(list, element)
    END SUBROUTINE AddToList

    !-------------------------------------------------------------------------------------------------------------------------------
    ! Copied from NWTC_IO.f90
    !> This function returns a character string encoded with today's date in the form dd-mmm-ccyy.
    FUNCTION CurDate( )

    ! Function declaration.

    CHARACTER(11)                :: CurDate                                      !< 'dd-mmm-yyyy' string with the current date


    ! Local declarations.

    CHARACTER(8)                 :: CDate                                        ! String to hold the returned value from the DATE_AND_TIME subroutine call.



    !  Call the system date function.

    CALL DATE_AND_TIME ( CDate )


    !  Parse out the day.

    CurDate(1:3) = CDate(7:8)//'-'


    !  Parse out the month.

    SELECT CASE ( CDate(5:6) )
    CASE ( '01' )
        CurDate(4:6) = 'Jan'
    CASE ( '02' )
        CurDate(4:6) = 'Feb'
    CASE ( '03' )
        CurDate(4:6) = 'Mar'
    CASE ( '04' )
        CurDate(4:6) = 'Apr'
    CASE ( '05' )
        CurDate(4:6) = 'May'
    CASE ( '06' )
        CurDate(4:6) = 'Jun'
    CASE ( '07' )
        CurDate(4:6) = 'Jul'
    CASE ( '08' )
        CurDate(4:6) = 'Aug'
    CASE ( '09' )
        CurDate(4:6) = 'Sep'
    CASE ( '10' )
        CurDate(4:6) = 'Oct'
    CASE ( '11' )
        CurDate(4:6) = 'Nov'
    CASE ( '12' )
        CurDate(4:6) = 'Dec'
    END SELECT


    !  Parse out the year.

    CurDate(7:11) = '-'//CDate(1:4)


    RETURN
    END FUNCTION CurDate


END MODULE ROSCO_Helpers
