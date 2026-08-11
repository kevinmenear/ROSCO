// The translation VIT's scaffold asks for, filled in as faithfully as its
// signature permits.
//
//     void AddToList(int* list, int n_list, int element)
//
// "Do not change the function signature."  So this is the whole budget:
//
//   list     a bare pointer to the caller's data.  Not the descriptor.
//   n_list   the extent, BY VALUE.  Writing to it is invisible to the caller.
//   element  the value to append.
//
// AddToList's entire contract is `allocate(clist(isize+1))` -> copy ->
// `move_alloc(clist, list)`: it replaces the caller's data pointer AND its
// extent.  Neither is reachable from here.  There is no out-parameter for a
// new pointer, no out-parameter for a new length, and n_list is a copy.
//
// So the only thing this body can do that resembles the Fortran is write the
// element one past the end of the buffer it was handed, and hope.  That is
// what it does, and it is a heap overrun.  It is written out rather than
// replaced with a `return` so the probe measures the BEST case for the
// generated bridge, not a strawman.
//
// The `else` branch of the Fortran -- allocate(list(1)) when list is
// unallocated -- has no representation at all, and is unreachable anyway:
// VIT's wrapper evaluates SIZE(list) before the call.

extern "C" void addtolist_c(int* list, int n_list, int element) {
    // Fortran: clist(1:isize) = list(1:isize) -- a copy into fresh storage.
    // Here list IS the storage, so the copy is a no-op and is elided.
    // Fortran: clist(isize+1) = element
    list[n_list] = element;          // one past the end of the caller's array
    // Fortran: call move_alloc(clist, list)  -- NOT EXPRESSIBLE.
    // The caller's descriptor keeps its old pointer and its old extent.
}
