// AddToList through a Fortran 2018 C descriptor.
//
// The dummy arrives as CFI_cdesc_t* -- the caller's own descriptor, not a copy
// of its data pointer -- so CFI_deallocate/CFI_allocate here have the same
// effect on the caller that `deallocate` + `move_alloc` have in the Fortran.
//
// Transcribed statement for statement from ROSCO_Helpers.f90:1640-1654.

#include <ISO_Fortran_binding.h>
#include <cstdlib>
#include <cstring>

extern "C" void addtolist_cfi_c(CFI_cdesc_t* list, int element) {
    // if (allocated(list)) then
    if (list->base_addr != nullptr) {
        // isize = size(list)
        const CFI_index_t isize = list->dim[0].extent;
        const CFI_index_t lb = list->dim[0].lower_bound;

        // allocate(clist(isize+1)) ; do i=1,isize ; clist(i) = list(i) ; end do
        int* clist = static_cast<int*>(std::malloc(
            static_cast<size_t>(isize + 1) * sizeof(int)));
        for (CFI_index_t i = 0; i < isize; ++i) {
            CFI_index_t sub = lb + i;
            clist[i] = *static_cast<int*>(CFI_address(list, &sub));
        }
        // clist(isize+1) = element
        clist[isize] = element;

        // deallocate(list) ; call move_alloc(clist, list)
        //
        // move_alloc gives the result clist's bounds, and `allocate(clist(n))`
        // means 1:n -- so the lower bound after the call is 1, whatever the
        // caller's was. That is the Fortran's behaviour, transcribed, not an
        // improvement on it.
        CFI_deallocate(list);
        CFI_index_t lower = 1, upper = isize + 1;
        CFI_allocate(list, &lower, &upper, 0);
        std::memcpy(list->base_addr, clist,
                    static_cast<size_t>(isize + 1) * sizeof(int));
        std::free(clist);
    } else {
        // allocate(list(1)) ; list(1) = element
        CFI_index_t lower = 1, upper = 1;
        CFI_allocate(list, &lower, &upper, 0);
        CFI_index_t sub = 1;
        *static_cast<int*>(CFI_address(list, &sub)) = element;
    }
}
