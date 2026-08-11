! What are `LEN(avcMSG)`, `LEN_TRIM(ExtRootName)` and `LEN(ErrMsg)` actually?
!
! Record 49 is `LEN(avcMSG) + 1`, and `avcMSG` is declared
! `CHARACTER(KIND=C_CHAR) :: avcMSG(LEN(ErrVar%ErrMsg)+1)` -- an ARRAY whose
! ELEMENT length is the default 1. `LEN` of a CHARACTER array is the ELEMENT
! length, not the array size, so record 49 is the constant 2 and NOT the size of
! the message buffer it is documented to be. Measured here rather than reasoned
! about, because the translation has to reproduce whichever it is and the two
! answers differ by three orders of magnitude.
PROGRAM len_probe
    USE, INTRINSIC :: ISO_C_BINDING
    IMPLICIT NONE
    CHARACTER(:), ALLOCATABLE :: ErrMsg
    CHARACTER(100), PARAMETER :: ExtRootName = 'external_control'
    CHARACTER(1024)           :: DLL_InFile

    ALLOCATE(CHARACTER(4096) :: ErrMsg)
    ErrMsg = ''
    DLL_InFile = 'some/input/file.IN'

    BLOCK
        CHARACTER(KIND=C_CHAR) :: avcMSG(LEN(ErrMsg)+1)
        CHARACTER(KIND=C_CHAR) :: avcOUTNAME(LEN_TRIM(ExtRootName)+1)
        CHARACTER(KIND=C_CHAR) :: accINFILE(LEN_TRIM(DLL_InFile)+1)
        PRINT '(A,I0)', 'SIZE(avcMSG)      = ', SIZE(avcMSG)
        PRINT '(A,I0)', 'LEN(avcMSG)       = ', LEN(avcMSG)
        PRINT '(A,I0)', 'record 49         = ', LEN(avcMSG) + 1
        PRINT '(A,I0)', 'SIZE(avcOUTNAME)  = ', SIZE(avcOUTNAME)
        PRINT '(A,I0)', 'record 51         = ', LEN_TRIM(ExtRootName) + 1
        PRINT '(A,I0)', 'SIZE(accINFILE)   = ', SIZE(accINFILE)
        PRINT '(A,I0)', 'record 50         = ', LEN_TRIM(DLL_InFile) + 1
        PRINT '(A,I0)', 'LEN(ErrMsg)       = ', LEN(ErrMsg)
        PRINT '(A,I0)', 'LEN(RoutineName)  = ', LEN('ExtController')
    END BLOCK
END PROGRAM len_probe
