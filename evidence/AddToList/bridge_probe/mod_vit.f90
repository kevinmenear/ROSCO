! VIT's OWN OUTPUT, copied verbatim from
!   vit interface AddToList -f rosco/controller/src/ROSCO_Helpers.f90
! (VIT d07a716, run 2026-08-10, saved as vit_interface.stdout.txt).
!
! Only the MODULE wrapper around it is added, so it can be compiled and called.
! The interface block and the wrapper body are byte-for-byte what VIT emitted.
MODULE AddToList_Vit
    IMPLICIT NONE
CONTAINS

    SUBROUTINE AddToList(list, element)
        USE ISO_C_BINDING
        IMPLICIT NONE
        INTEGER(4), INTENT(INOUT) :: list(:)
        INTEGER(4), INTENT(IN) :: element
    ! Auto-generated interface for C++ implementation of AddToList
    INTERFACE
        SUBROUTINE addtolist_c(list, n_list, element) BIND(C, NAME='addtolist_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), INTENT(INOUT) :: list(*)
            INTEGER(C_INT), VALUE :: n_list
            INTEGER(C_INT), VALUE :: element
        END SUBROUTINE addtolist_c
    END INTERFACE
        CALL addtolist_c(list, SIZE(list), element)
    END SUBROUTINE AddToList

END MODULE AddToList_Vit
