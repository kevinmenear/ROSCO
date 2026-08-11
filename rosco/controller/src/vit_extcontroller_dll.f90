! VIT production bridge for ExtController's dynamic-library state.
!
! WHY THIS FILE EXISTS.
!
! `ExtController` has one local that cannot cross into C++ and one callee that
! must not be inlined (X1), and they are the same thing:
!
!     TYPE(ExtDLL_Type), SAVE :: DLL_Ext
!     CALL LoadDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
!
! `ExtDLL_Type` has five fields, three of which have no C representation worth
! mirroring -- `TYPE(C_PTR) :: FileAddrX`, `TYPE(C_FUNPTR) :: ProcAddr(3)` and
! `CHARACTER(1024) :: ProcName(3)`, a CHARACTER ARRAY that VIT's view generator
! and the differential harness both still refuse. And it is SAVE: its lifetime
! spans calls, so it is ambient state in `harness/contract.py`'s sense, not a
! value the signature carries.
!
! `plan.json` therefore declares `LoadDynamicLib` a PERMANENT BRIDGE for this
! unit. This module is that bridge. It owns the SAVE variable, on the Fortran
! side, where the type is expressible -- and it CALLS `LoadDynamicLib` rather
! than reproducing what it does, which is the whole of X1: the campaign has
! already paid for inlining a callee three times.
!
! WHAT CROSSES, and what deliberately does not:
!
!   vit_extcontroller_loaddll_c   the two names in, the error status and error
!                                 message out. Everything the SAVE variable
!                                 holds stays here.
!   vit_extcontroller_procaddr_c  the procedure address, as a bare C function
!                                 pointer. The C++ casts it and calls through
!                                 it, which is exactly what `C_F_PROCPOINTER`
!                                 followed by `CALL` compiles to. That is a
!                                 language feature being used on the other side
!                                 of the boundary, not a callee being inlined.
!
! The DLL handle itself never crosses. A C++ translation that dlopen()ed the
! library itself would be a second loader with its own refcount and its own
! idea of which symbols were bound -- two implementations of the thing under
! test, which is the shape this project exists to remove.

MODULE vit_extcontroller_dll

    USE, INTRINSIC :: ISO_C_BINDING
    USE ROSCO_Types
    USE SysSubs

    IMPLICIT NONE

    ! THE SAVE LOCAL, moved rather than copied. `ExtController`'s own
    ! declaration is `TYPE(ExtDLL_Type), SAVE :: DLL_Ext`; this is that
    ! variable, and there is exactly one of it in the program.
    TYPE(ExtDLL_Type), SAVE :: vit_DLL_Ext

CONTAINS

    ! DLL_Ext%FileName = TRIM(CntrPar%DLL_FileName)
    ! DLL_Ext%ProcName = TRIM(CntrPar%DLL_ProcName)
    ! CALL LoadDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
    !
    ! `ProcName` is an ARRAY of three and the original assigns a SCALAR to it,
    ! so all three elements get the same name; `LoadDynamicLibProc` then looks
    ! up each one whose LEN_TRIM is non-zero. Transcribed as the same scalar
    ! assignment rather than as a loop, so the shape is the original's.
    SUBROUTINE vit_extcontroller_loaddll_c(fileName, fileNameLen, &
                                           procName, procNameLen, &
                                           errStat, errMsg, errMsgLen) &
            BIND(C, NAME='vit_extcontroller_loaddll_c')
        CHARACTER(KIND=C_CHAR), INTENT(IN)    :: fileName(*)
        INTEGER(C_INT), VALUE                 :: fileNameLen
        CHARACTER(KIND=C_CHAR), INTENT(IN)    :: procName(*)
        INTEGER(C_INT), VALUE                 :: procNameLen
        INTEGER(C_INT), INTENT(OUT)           :: errStat
        CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: errMsg(*)
        ! LEN(ErrVar%ErrMsg), NOT a capacity. `LoadDynamicLib`'s dummy is
        ! `CHARACTER(*), INTENT(OUT)`, so the assignments inside it blank-pad
        ! or truncate to the length the actual argument already had -- a
        ! deferred-length allocatable passed to an assumed-length dummy is not
        ! reallocated. The C++ side may not grow it here, and does not.
        INTEGER(C_INT), VALUE                 :: errMsgLen

        CHARACTER(fileNameLen) :: vit_file
        CHARACTER(procNameLen) :: vit_proc
        CHARACTER(MAX(errMsgLen, 0)) :: vit_msg
        INTEGER :: i

        DO i = 1, fileNameLen
            vit_file(i:i) = fileName(i)
        END DO
        DO i = 1, procNameLen
            vit_proc(i:i) = procName(i)
        END DO

        vit_DLL_Ext%FileName = vit_file
        vit_DLL_Ext%ProcName = vit_proc

        CALL LoadDynamicLib(vit_DLL_Ext, errStat, vit_msg)

        DO i = 1, errMsgLen
            errMsg(i) = vit_msg(i:i)
        END DO
    END SUBROUTINE vit_extcontroller_loaddll_c

    ! CALL C_F_PROCPOINTER( DLL_Ext%ProcAddr(1), DLL_Legacy_Subroutine )
    !
    ! Returned rather than called here, so the CALL itself stays in the
    ! translation where the original puts it. A bridge that made the call would
    ! be translating the statement rather than bridging the state.
    SUBROUTINE vit_extcontroller_procaddr_c(addr) &
            BIND(C, NAME='vit_extcontroller_procaddr_c')
        TYPE(C_FUNPTR), INTENT(OUT) :: addr
        addr = vit_DLL_Ext%ProcAddr(1)
    END SUBROUTINE vit_extcontroller_procaddr_c

END MODULE vit_extcontroller_dll
