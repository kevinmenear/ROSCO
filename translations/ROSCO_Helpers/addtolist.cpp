// VIT Translation Scaffold
// Function: AddToList
// Source: ROSCO_Helpers.f90
// Module: ROSCO_Helpers
// Fortran: SUBROUTINE AddToList(list, element)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: cee72a522b49
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-11T02:14:38Z

#include <ISO_Fortran_binding.h>

#include <cstring>
#include <vector>

// `list` is ALLOCATABLE, INTENT(INOUT), so it arrives as the CALLER'S OWN
// Fortran descriptor. CFI_deallocate/CFI_allocate through it have the effect
// `deallocate` + `move_alloc` have in the original: they replace the caller's
// data pointer AND its extent.
//
// Transcribed statement for statement from ROSCO_Helpers.f90:1627-1657. The
// body is the one measured against the Fortran oracle at unit #1 before VIT
// could generate the bridge for it (evidence/AddToList/bridge_probe/), moved
// here unchanged apart from the function name.
void AddToList(CFI_cdesc_t* list, int element) {
    // if (allocated(list)) then
    if (list->base_addr != nullptr) {
        // isize = size(list)
        const CFI_index_t isize = list->dim[0].extent;
        const CFI_index_t lb = list->dim[0].lower_bound;

        // SIZE(clist), written once. It appeared three times -- the allocation,
        // the new upper bound and the copy length -- and two of the three were
        // then unobservable: a mutant that changed only the allocation's copy
        // over-allocated and survived, and one that changed only the copy
        // length ran past the buffer without changing any output. One name for
        // one quantity leaves a single site, and that site decides the extent
        // the caller sees.
        const CFI_index_t nsize = isize + 1;

        // allocate(clist(isize+1)) ; do i=1,isize ; clist(i) = list(i) ; end do
        //
        // The loop keeps the Fortran's 1-based bounds rather than being
        // rewritten 0-based. That is not decoration either: `i <= isize`
        // perturbed to `i < isize` leaves an element uncopied, which the
        // comparison sees, while the 0-based `i < isize` perturbed to
        // `i <= isize` writes a slot the next statement overwrites and is
        // invisible.
        //
        // The copy happens BEFORE the deallocate: CFI_deallocate frees the
        // storage CFI_address reads through.
        std::vector<int> clist(static_cast<size_t>(nsize));
        for (CFI_index_t i = 1; i <= isize; ++i) {
            CFI_index_t sub = lb + (i - 1);
            clist[i - 1] = *static_cast<int*>(CFI_address(list, &sub));
        }
        // clist(isize+1) = element
        clist[nsize - 1] = element;

        // deallocate(list) ; call move_alloc(clist, list)
        //
        // move_alloc gives the result clist's bounds, and `allocate(clist(n))`
        // means 1:n -- so the lower bound after the call is 1, whatever the
        // caller's was. That is the Fortran's behaviour, transcribed, not an
        // improvement on it.
        CFI_deallocate(list);
        CFI_index_t lower = 1, upper = nsize;
        CFI_allocate(list, &lower, &upper, 0);
        std::memcpy(list->base_addr, clist.data(),
                    sizeof(int) * static_cast<size_t>(nsize));
    } else {
        // allocate(list(1)) ; list(1) = element
        CFI_index_t lower = 1, upper = 1;
        CFI_allocate(list, &lower, &upper, 0);
        CFI_index_t sub = 1;
        *static_cast<int*>(CFI_address(list, &sub)) = element;
    }
}
