// VIT Translation Scaffold
// Function: StructuralControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE StructuralControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 368852b1f848
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-15T09:46:46Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature, and the body is transcribed statement for statement.
//
// ONE OF THE FIVE ARGUMENTS IS NEVER READ AND NEVER WRITTEN. `objInst` is
// declared INTENT(INOUT) by the reference and appears nowhere in its body --
// the same standing PitchSaturation and PowerControlSetpoints record for their
// own unread arguments. It is named here because the SIGNATURE is the contract,
// and left untouched because the reference leaves it untouched (P6).
//
// TWO OF THIS UNIT'S THREE ARMS ARE OUTSIDE EVERY SIMULATION THE CAMPAIGN RUNS.
// Read from coverage/line_coverage.json against the clean source, all 27
// scenarios (`StructuralControl` at Controllers.f90:967-1029 at 54dd134):
//
//   :991  IF (StC_Mode == 1)             23,999 under 7 and 27, 15,999 under 3
//   :994  IF (Time > 500)                tested on every one of those calls
//   :996-:998  the three constants        3,998 under 7, 3,998 under 27, 0 under 3
//   :1003 ELSEIF (StC_Mode == 2)              0 hits, ALL 27 SCENARIOS
//   :1008 the interp1d call                   0 hits, ALL 27 SCENARIOS
//   :1020/:1021 the avrSWAP copy loop     47,998 = 2 x 23,999, StC_Group_N = 1
//   :1026 the ErrMsg prefix                   0 hits, ALL 27 SCENARIOS
//
// So the kernel and the gate can see the `StC_Mode == 1` arm, both sides of the
// `Time > 500` test, and the copy loop at ONE group. They cannot see the whole
// open-loop arm -- which is this unit's only call to `interp1d` and the only
// reader of `Ind_StructControl`, `OL_Breakpoints` and `OL_StructControl` -- and
// they cannot see the error-message prefix. No shipped `Examples/DISCON*.IN`
// sets `StC_Mode` to 2, and the three scenarios that set it at all set it to 1.
// The differential harness draws `StC_Mode`, `StC_Group_N` and the open-loop
// arrays freely and is the instrument that reaches them; where that is measured
// is recorded in evidence/StructuralControl/, not asserted here.

#include "vit_types.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

// CHARACTER(*), PARAMETER :: RoutineName = 'StructuralControl'
constexpr std::string_view RoutineName = "StructuralControl";

// THE TWO HELPERS BELOW ARE COPIED VERBATIM FROM
// `translations/ControllerBlocks/pitchsaturation.cpp` -- with the literal
// `PitchSaturation` in the two diagnostic strings and in the comment changed to
// this unit's name -- which in turn took them verbatim from
// `translations/Functions/interp1d.cpp`, this unit's only callee and the unit
// that established the shape. They are not re-derived from prose (P4). The
// standing case for hoisting them into a shared header is DECISIONS.md's, not
// this unit's: it changes how every translated unit in this campaign is built,
// which is X3's question.

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one.
//
// The capacity test is REACHABLE in principle here, as in PitchSaturation and
// unlike interp1d: the message written is eighteen characters longer than the
// one it was handed, so an `ErrMsg` arriving within eighteen bytes of the
// buffer's capacity overflows it. Which inputs actually reach it is a question
// for the corpus and not for this comment.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: StructuralControl: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: StructuralControl: ErrVar%%ErrMsg needs %d bytes, the staging "
                     "buffer holds %d; the assignment is refused\n",
                     static_cast<int>(s.size()), static_cast<int>(ErrVar->n_ErrMsg_cap));
        return;
    }
    std::memcpy(ErrVar->ErrMsg, s.data(), s.size());
    ErrVar->n_ErrMsg = static_cast<int32_t>(s.size());
}

// TRIM(ErrVar%ErrMsg): trailing blanks only, off the field's CURRENT length.
// `find_last_not_of` rather than a hand-written backward scan, for unit #15's
// and unit #17's reason: a loop written `while (n > 0 && s[n-1] == ' ')` offers
// a `> 0` -> `>= 0` mutant that reads the byte BEFORE the buffer, which is
// undefined behaviour rather than a wrong answer and which no value comparison
// can be relied on to catch.
//
// A NEGATIVE length is the view's NOT-ALLOCATED convention, and it collapses to
// the same empty string a zero LENGTH gives -- which is correct for both: the
// reference's `RoutineName//':'//TRIM(ErrMsg)` on a zero-length ErrMsg is
// exactly `'StructuralControl:'`.
// No `== npos` branch: `find_last_not_of` returns `npos`, which is `SIZE_MAX`,
// and `npos + 1` is 0 by the defined wraparound of an unsigned type -- so the
// all-blank string falls out of the same expression as every other.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 0);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

void StructuralControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                       localvariables_view_t* LocalVar, objectinstances_t* objInst,
                       errorvariables_view_t* ErrVar) {
    // The reference does not name this one; see the header. Voided so that the
    // signature can carry it without a warning suppressing itself into a habit
    // -- the day it IS read, the void goes away with the read.
    (void)objInst;

    // Integer(IntKi) :: I_GROUP
    int I_GROUP;

    // IF (CntrPar%StC_Mode == 1) THEN
    if (CntrPar->StC_Mode == 1) {
        // ! User defined control, step example
        //
        // IF (LocalVar%Time > 500) THEN
        //
        // `LocalVar%Time` is REAL(DbKi) and 500 is an INTEGER literal, so the
        // comparison converts the 500 to real and is exact. Written `500.0`
        // here for the same reason: the value, not the spelling, is what the
        // reference compares.
        if (LocalVar->Time > 500.0) {
            // ! Step change in input of -4500 N
            // LocalVar%StC_Input(1) = -1.234e+06
            // LocalVar%StC_Input(2) = 2.053e+06
            // LocalVar%StC_Input(3) = -7.795e+05
            //
            // The three literals carry no kind suffix, so under this campaign's
            // -fdefault-real-8 each is parsed at kind 8 and assigned to a
            // REAL(DbKi) field with no narrowing anywhere. All three are whole
            // numbers -- 1234000, 2053000, 779500 -- and therefore exact in
            // binary64 and in binary32 alike, so the decimal text below denotes
            // the same value the reference stores no matter which way it is
            // read. Transcribed as written rather than as -1234000.0, because
            // the shape the reference has is the shape a reader will diff.
            //
            // THE SUBSCRIPTS ARE LITERAL 1, 2, 3 AND ARE NOT GUARDED BY
            // StC_Group_N. `StC_Input` is `REAL(DbKi) :: StC_Input(12)`, a
            // fixed-size field, so all three are in bounds unconditionally --
            // and this arm writes all three even when StC_Group_N is 1, which
            // is what every scenario that reaches it does. Elements 2 and 3 are
            // then never copied to avrSWAP by the loop below.
            LocalVar->StC_Input[0] = -1.234e+06;
            LocalVar->StC_Input[1] = 2.053e+06;
            LocalVar->StC_Input[2] = -7.795e+05;
        // END IF
        }

    // ELSEIF (CntrPar%StC_Mode == 2) THEN
    } else if (CntrPar->StC_Mode == 2) {

        // DO I_GROUP = 1,CntrPar%StC_Group_N
        for (I_GROUP = 1; I_GROUP <= CntrPar->StC_Group_N; ++I_GROUP) {
            // IF (CntrPar%Ind_StructControl(I_GROUP) > 0) THEN
            if (CntrPar->Ind_StructControl[I_GROUP - 1] > 0) {
                // LocalVar%StC_Input(I_GROUP) = interp1d(CntrPar%OL_Breakpoints, &
                //                                        CntrPar%OL_StructControl(I_GROUP,:), &
                //                                        LocalVar%Time, ErrVar)
                //
                // `OL_StructControl(I_GROUP,:)` IS A STRIDED ROW, NOT A
                // POINTER. The array is `REAL(DbKi), DIMENSION(:,:)` and the
                // view exposes it column-major
                // (`vit_controlparameters_view.f90:811-814`), so the row's
                // elements are `n_OL_StructControl_rows` apart and its LENGTH
                // is the COLUMN count. It has to be gathered.
                //
                // Gathering is what the reference does too, one level down.
                // `interp1d`'s own dummy is assumed-shape `REAL(8) :: yData(:)`
                // and takes the section as a descriptor without copying -- but
                // `interp1d` is itself integrated now, and its wrapper passes
                // that descriptor on to `interp1d_c`, whose dummy is
                // `REAL(C_DOUBLE) :: yData(*)`. An assumed-size dummy requires
                // contiguity, so gfortran materialises exactly this contiguous
                // copy at that call. The gather is the reference's own
                // behaviour transcribed, not an accommodation for C++.
                //
                // SIZE(yData) is therefore the column count, and it is passed
                // as such. interp1d's reference ABORTS on
                // `SIZE(xData) /= SIZE(yData)` while its translation continues
                // (evidence/interp1d/reference.size-mismatch-aborts.txt); at
                // this call site the Fortran CAN produce a mismatch, unlike
                // PowerControlSetpoints' -- `Read_OL_Input` allocates
                // `OL_StructControl` with `SIZE(CntrPar%OL_Channels, DIM=1)`
                // columns and `OL_Breakpoints` from the same read, so they
                // agree in the shipped program, but nothing in THIS procedure
                // enforces it and the harness draws the two extents freely.
                //
                // THE GATHER IS WRITTEN WITH NO DEFENSIVE GUARD AND ITS LENGTH
                // IS READ BACK OFF THE BUFFER, and both are the mutation score
                // talking rather than taste. The first version sized the buffer
                // `std::vector<double> row(cols > 0 ? (size_t)cols : 0)` and
                // passed `cols` as `SIZE(yData)`. That cost FOUR of the first
                // sweep's eleven survivors and every one of them was unkillable
                // BY CONSTRUCTION:
                //
                //   `cols > 0` -> `cols >= 0`   three mutants of a guard on a
                //   `cols > 0` -> `cols > 1`    quantity R5 never draws below 3,
                //   `: 0`      -> `: 1`         so all three spell `cols` -- the
                //                               reference has no such test and
                //                               neither should this
                //   `j < cols` -> `j <= cols`   writes past the buffer, which
                //                               passing `cols` as the LENGTH
                //                               hides from the comparison
                //
                // So the guard is gone -- `cols` is `INT(SIZE(...), C_INT32_T)`
                // or 0 in the view populator and is never negative, which makes
                // the test dead code the reference does not have -- and the
                // length passed to `interp1d_c` is `row.size()`, which is what
                // the reference's `SIZE(yData)` actually denotes. A loop bound
                // that gathers one element too many then CHANGES that length,
                // and the mutant dies instead of corrupting the heap in silence.
                // Unit #37's standing about `std::max` is the same rule seen
                // from the other side: prefer the spelling that offers a
                // killable mutant, and say where it cannot.
                const int32_t rows = CntrPar->n_OL_StructControl_rows;
                const int32_t cols = CntrPar->n_OL_StructControl_cols;
                std::vector<double> row;
                for (int32_t j = 0; j < cols; ++j) {
                    row.push_back(
                        CntrPar->OL_StructControl[static_cast<size_t>(j) * rows +
                                                  (I_GROUP - 1)]);
                }
                LocalVar->StC_Input[I_GROUP - 1] =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               row.data(), static_cast<int>(row.size()),
                               LocalVar->Time, ErrVar);
            // ENDIF
            }
        // ENDDO
        }

    // END IF
    //
    // NO THIRD ARM. An `StC_Mode` outside {1,2} -- including 0, which is what
    // every scenario that does not enable this unit holds it at -- falls
    // through both tests and `StC_Input` keeps the values it arrived with,
    // which the copy loop below then still writes to `avrSWAP`. That is a real,
    // reachable behaviour of the reference and not an oversight to be closed
    // here (P7). It is also unreachable through DISCON, whose call site sits
    // behind `IF (CntrPar%StC_Mode > 0)`; the harness calls this procedure
    // directly and does reach it.
    }

    // ! Assign to avrSWAP
    // DO I_GROUP = 1, CntrPar%StC_Group_N
    //     avrSWAP(CntrPar%StC_GroupIndex(I_GROUP)) = LocalVar%StC_Input(I_GROUP)
    // END DO
    //
    // UNGUARDED ON THREE COUNTS, ALL THREE THE REFERENCE'S. `StC_Group_N` is a
    // free INTEGER of the configuration and bounds neither `StC_GroupIndex`
    // (ALLOCATABLE, sized by the input file) nor `StC_Input` (fixed at 12); and
    // `StC_GroupIndex(I_GROUP)` is an arbitrary integer used as an `avrSWAP`
    // subscript. The reference has no test for any of the three, so neither
    // does this, and the relations that hold in the shipped program are stated
    // in `harness/ranges.toml` where the corpus is constrained -- adding a
    // guard here would be a behaviour the reference does not have.
    //
    // THE ASSIGNMENT NARROWS. `avrSWAP` is REAL(ReKi) = REAL(4) and `StC_Input`
    // is REAL(DbKi) = REAL(8), so the Fortran assignment rounds double to
    // single. The cast is written out because a silent implicit conversion here
    // is the one place a reader could mistake this for a copy.
    for (I_GROUP = 1; I_GROUP <= CntrPar->StC_Group_N; ++I_GROUP) {
        avrSWAP[CntrPar->StC_GroupIndex[I_GROUP - 1] - 1] =
            static_cast<float>(LocalVar->StC_Input[I_GROUP - 1]);
    }

    // ! Add RoutineName to error message
    // IF (ErrVar%aviFAIL < 0) THEN
    //     ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    // ENDIF
    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }
}
