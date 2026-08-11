! The oracle (P7): AddToList copied VERBATIM from
! rosco/controller/src/ROSCO_Helpers.f90:1629-1657, with IntKi bound to the
! campaign's kind alias (vit.yaml fortran.kind_aliases: IntKi -> INTEGER(4)).
! Nothing else is changed. Do not "clean up" this body.
MODULE AddToList_Orig
    IMPLICIT NONE
    INTEGER, PARAMETER :: IntKi = 4
CONTAINS

    subroutine AddToList(list, element)
        ! Credit to: https://stackoverflow.com/questions/28048508/how-to-add-new-element-to-dynamical-array-in-fortran-90
        ! This is set up for integers, will need to make interface for other types
          IMPLICIT NONE

          integer :: i, isize
          Integer(IntKi), intent(in) :: element
          Integer(IntKi), dimension(:), allocatable, intent(inout) :: list
          Integer(IntKi), dimension(:), allocatable :: clist


          if(allocated(list)) then
              isize = size(list)
              allocate(clist(isize+1))
              do i=1,isize
              clist(i) = list(i)
              end do
              clist(isize+1) = element

              deallocate(list)
              call move_alloc(clist, list)

          else
              allocate(list(1))
              list(1) = element
          end if


      end subroutine AddToList

END MODULE AddToList_Orig
