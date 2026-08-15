// VIT Translation Scaffold
// Function: Shutdown
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE Shutdown(LocalVar, CntrPar, objInst, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 84dc2630e515
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T05:01:55Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature; the body is transcribed statement for statement.
//
// WHAT THE SIMULATION CAN SEE OF THIS UNIT, read from
// coverage/line_coverage.json against the coverage-era source (Shutdown at
// ControllerBlocks.f90:638-760) and stated here BEFORE any green is taken, so
// that a reader knows which arms a kernel or gate result is about:
//
//     :655 IF (iStatus == 0)                 11,999 calls / arm taken ONCE
//     :671 the trigger window                11,999 / the four tests 10,002
//     :683 SD_Trigger = 4  (SD_EnableTime)        1  <- the ONLY trigger fired
//     :674 / :677 / :680  Trigger = 1 / 2 / 3     0  <- pitch, yaw, genspeed
//     :688 SD_Method == 1                    11,999 <- always
//     :691 SD_Stage == 0    10,002    |  :702 the stage arm      1,997
//     :708 SD_Stage = SD_Stage + 1            0  <- never advances
//     :711 the ELSE (Stage > SD_Stage_N)      0
//     :713-:748 the whole SD_Method == 2 arm  0
//
// And scenario 9 is the only one of the 27 that reaches ANY of it: the call
// site DISCON.F90:107 sits behind `IF (CntrPar%SD_Mode > 0)` and carries hits
// under scenario 9 alone. So more than half of this unit's statements are
// outside every simulation the campaign runs, and the differential harness --
// which draws `SD_Method`, the four `SD_Enable*` flags and `SD_Stage` freely --
// is the only instrument that can reach them. Where each arm is actually
// measured is recorded in evidence/Shutdown/, not asserted here.

#include "vit_types.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// CHARACTER(*), PARAMETER :: RoutineName = 'Shutdown'
constexpr std::string_view RoutineName = "Shutdown";

// The Fortran reads `R2D`/`D2R` from the Constants module by name. Restated
// here to the digit, as `prefiltermeasuredsignals.cpp` and `readavrswap.cpp`
// already do: the reference's own literals, not a recomputed 180/Pi, because
// 57.2957795130 and 0.01745329251 are TRUNCATED decimals and not the correctly
// rounded conversions -- 180/Pi is 57.29577951308232, and the difference is
// visible in the last places of every angle this unit filters.
constexpr double R2D = 57.2957795130;
constexpr double D2R = 0.01745329251;

// THE TWO HELPERS BELOW ARE COPIED FROM `translations/ControllerBlocks/
// pitchsaturation.cpp` -- the unit in this module that established the shape,
// which took them in turn from `interp1d.cpp` -- with the literal name in the
// two diagnostic strings changed to this unit's. They are not re-derived from
// the prose (P4).

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one.
//
// The capacity test is reachable in principle: the message written is nine
// characters longer than the one it was handed (`Shutdown` plus the colon), so
// an `ErrMsg` arriving within nine bytes of the buffer's capacity overflows it.
// Which inputs reach it is a question for the corpus and not for this comment.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: Shutdown: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: Shutdown: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// TRIM(ErrVar%ErrMsg): trailing blanks only, off the field's CURRENT length.
// `find_last_not_of` rather than a hand-written backward scan: a loop written
// `while (n > 0 && s[n-1] == ' ')` offers a `> 0` -> `>= 0` mutant that reads
// the byte BEFORE the buffer, which is undefined behaviour rather than a wrong
// answer and which no value comparison can be relied on to catch.
//
// A NEGATIVE length is the view's NOT-ALLOCATED convention and collapses to the
// same empty string a zero LENGTH gives, which is correct for both.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

void Shutdown(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,
              objectinstances_t* objInst, errorvariables_view_t* ErrVar) {
    // RED TEST STUB 1 -- THE WHOLE UNIT AS A NO-OP.
    //
    // Not the shipped translation. It reads nothing and writes nothing, so
    // every output the kernel compares must come back as whatever the captured
    // state held on entry. A kernel that still reports 62/62 IDENTICAL would be
    // comparing nothing this function computes.
    //
    // ONE DISCARDED CALLEE CALL IS KEPT. `harness.sh` and VIT's kernel setup
    // both generate callee bridges by READING THE TRANSLATION for calls, and a
    // stub that calls nothing keeps nothing -- which produces an undefined
    // reference that reads like a broken instrument rather than a red test.
    // Its arguments are constants and its result is discarded, so it cannot
    // make this stub agree with the reference on any output.
    (void)CntrPar;
    (void)ErrVar;
    (void)wrap_180_c(0.0);
    (void)lpfilter_c(0.0, 0.0, 0.0, &LocalVar->FP, 1, 0, &objInst->instLPF, 0, 0.0);
}
