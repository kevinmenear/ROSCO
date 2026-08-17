// VIT Translation Scaffold
// Function: CableControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE CableControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 12501103d6a8
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-17T19:46:39Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature, and the body is transcribed statement for statement.
//
// THIS UNIT IS `StructuralControl`'s TWIN AND THE DIFFERENCES ARE THE WHOLE
// POINT. Unit #43 is the same five-argument shape, the same three arms, the
// same `Time > 500` step and the same `avrSWAP` write loop. Three things are
// NOT the same and each one is a place a transcription can go wrong:
//
//   1. The write loop writes TWO slots per group, at `CC_GroupIndex(I)` and at
//      `CC_GroupIndex(I) + 1`, so the last index the reference can touch is one
//      past the drawn one.
//   2. Between the arms and the write loop there is a SECOND loop that calls
//      two integrated callees, `SecLPFilter_Vel` and `PIController`, both of
//      which carry per-instance state indexed by an `objInst` counter they
//      post-increment.
//   3. `RoutineName` is the string `'StructuralControl'`. That is an upstream
//      copy-paste in ROSCO and it is transcribed as written -- see the comment
//      at its declaration below.
//
// WHAT THE 27 SCENARIOS CAN SEE, read from coverage/line_coverage.json against
// the clean source (`CableControl` at Controllers.f90:884-964 at 54dd134):
//
//   :907  IF (CC_Mode == 1)              15,999 under 3; 23,999 under 7 and 27
//   :910  IF (Time > 500)                tested on every one of those calls
//   :912-:914  the three constants        3,998 under 7, 3,998 under 27, 0 under 3
//   :919  ELSEIF (CC_Mode == 2)               0 hits, ALL 27 SCENARIOS
//   :926  the interp1d call                   0 hits, ALL 27 SCENARIOS
//   :941/:945  SecLPFilter_Vel, PIController  47,998 = 2 x 23,999, CC_Group_N = 1
//   :954/:955  the two avrSWAP writes     47,998 and 47,998, same loop
//   :961  the ErrMsg prefix                   0 hits, ALL 27 SCENARIOS
//
// So the gate sees the `CC_Mode == 1` arm, both sides of the `Time > 500` test,
// the callee loop and the double write, all at ONE group. It cannot see the
// open-loop arm -- this unit's only call to `interp1d` and the only reader of
// `Ind_CableControl` and `OL_CableControl` -- and it cannot see the
// error-message prefix. Scenario 3 reaches the unit but never the step, because
// it runs to t = 400 and the guard is `Time > 500`; that is the same arithmetic
// StructuralControl recorded for the same scenario. The differential harness
// draws `CC_Mode`, `CC_Group_N` and the open-loop arrays freely and is the
// instrument that reaches them; where that is measured is recorded in
// evidence/CableControl/, not asserted here.

#include "vit_types.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace {

// REAL(DbKi), PARAMETER :: PI = 3.14159265359      (Constants.f90:24)
//
// ROSCO's own decimal, not `M_PI` and not a longer expansion: the reference
// multiplies by exactly this twelve-digit value, and the double nearest to it
// is not the double nearest to pi. Same spelling as unit #47's and unit #48's.
constexpr double PI = 3.14159265359;

// CHARACTER(*), PARAMETER :: RoutineName = 'StructuralControl'
//
// NOT A TYPO HERE -- A TYPO IN THE REFERENCE, TRANSCRIBED. `CableControl`'s
// `RoutineName` really is the string `'StructuralControl'` in ROSCO
// (Controllers.f90:903 at 54dd134); the procedure was written by copying its
// neighbour and this line was not changed with the rest. It is the value the
// reference prefixes onto `ErrVar%ErrMsg`, so it is the value that has to be
// prefixed here (P7: the oracle is the original source). Correcting it would
// make this translation disagree with the program it is replacing on every
// input that reaches the prefix, and the disagreement would be invisible to
// the gate, which never reaches that statement at all.
constexpr std::string_view RoutineName = "StructuralControl";

// THE TWO HELPERS BELOW ARE COPIED FROM
// `translations/Controllers/structuralcontrol.cpp` -- which took them from
// `translations/ControllerBlocks/pitchsaturation.cpp`, which took them from
// `translations/Functions/interp1d.cpp`, the unit that established the shape.
// They are not re-derived from prose (P4). The only edits are the two
// diagnostic strings, which name the UNIT so a reader of stderr knows which
// translation refused; they are deliberately NOT `RoutineName`, because
// `RoutineName` here is the neighbouring procedure's name and a diagnostic
// carrying it would send a reader to the wrong file.
//
// The standing case for hoisting them into a shared header is DECISIONS.md's,
// not this unit's: it changes how every translated unit in this campaign is
// built, which is X3's question.

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one.
//
// THE CAPACITY TEST IS REACHABLE, and this unit is one where the composition
// unit #48 measured CAN occur: the message written is eighteen characters
// longer than the one it was handed (`'StructuralControl'` plus `':'`), and on
// the `CC_Mode == 2` path the callee `interp1d` may itself have written
// `ErrVar%ErrMsg` through its own bridge. Whether the two staged assignments
// actually compose on any case in the corpus is a question for the corpus and
// is answered in evidence/CableControl/, not asserted here.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: CableControl: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: CableControl: ErrVar%%ErrMsg needs %d bytes, the staging "
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

void CableControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                  localvariables_view_t* LocalVar, objectinstances_t* objInst,
                  errorvariables_view_t* ErrVar) {
    // Integer(IntKi) :: I_GROUP
    int I_GROUP;

    // IF (CntrPar%CC_Mode == 1) THEN
    if (CntrPar->CC_Mode == 1) {
        // ! User defined control
        //
        // IF (LocalVar%Time > 500) THEN
        //
        // `LocalVar%Time` is REAL(DbKi) and 500 is an INTEGER literal, so the
        // comparison converts the 500 to real and is exact. Written `500.0`
        // here for the same reason as unit #43: the value, not the spelling, is
        // what the reference compares.
        if (LocalVar->Time > 500.0) {
            // ! Shorten first group by 4 m
            // LocalVar%CC_DesiredL(1) = -14.51
            // LocalVar%CC_DesiredL(2) = 1.58
            // LocalVar%CC_DesiredL(3) = -10.332
            //
            // The three literals carry no kind suffix, so under this campaign's
            // -fdefault-real-8 each is parsed at kind 8 and assigned to a
            // REAL(DbKi) field with no narrowing anywhere. UNLIKE unit #43's
            // three, NONE of these is a whole number, so each denotes the
            // binary64 value NEAREST the decimal rather than the decimal
            // itself -- and that is exactly why they are transcribed as decimal
            // text rather than expanded: gfortran and this compiler both round
            // the same decimal literal to the same double, and any hand
            // expansion would be a second, unnecessary rounding.
            //
            // THE SUBSCRIPTS ARE LITERAL 1, 2, 3 AND ARE NOT GUARDED BY
            // CC_Group_N. `CC_DesiredL` is `REAL(DbKi) :: CC_DesiredL(12)`, a
            // fixed-size field, so all three are in bounds unconditionally --
            // and this arm writes all three even when CC_Group_N is 1, which is
            // what every scenario that reaches it does. Elements 2 and 3 are
            // then never read by the loops below.
            LocalVar->CC_DesiredL[0] = -14.51;
            LocalVar->CC_DesiredL[1] = 1.58;
            LocalVar->CC_DesiredL[2] = -10.332;
        // END IF
        }

    // ELSEIF (CntrPar%CC_Mode == 2) THEN
    } else if (CntrPar->CC_Mode == 2) {
        // ! Open loop control

        // DO I_GROUP = 1,CntrPar%CC_Group_N
        for (I_GROUP = 1; I_GROUP <= CntrPar->CC_Group_N; ++I_GROUP) {
            // IF (CntrPar%Ind_CableControl(I_GROUP) > 0) THEN
            if (CntrPar->Ind_CableControl[I_GROUP - 1] > 0) {
                // LocalVar%CC_DesiredL(I_GROUP) = interp1d(CntrPar%OL_Breakpoints, &
                //                                          CntrPar%OL_CableControl(I_GROUP,:), &
                //                                          LocalVar%Time, ErrVar)
                //
                // `OL_CableControl(I_GROUP,:)` IS A STRIDED ROW, NOT A POINTER.
                // The array is `REAL(DbKi), DIMENSION(:,:)` and the view exposes
                // it column-major (`vit_controlparameters_view.f90:811-814`), so
                // the row's elements are `n_OL_CableControl_rows` apart and its
                // LENGTH is the COLUMN count. It has to be gathered.
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
                // NO DEFENSIVE GUARD ON `cols`, AND THE LENGTH PASSED IS THE
                // BUFFER'S OWN. Unit #43 measured what the other spelling costs:
                // sizing the vector `cols > 0 ? (size_t)cols : 0` and passing
                // `cols` as `SIZE(yData)` produced four survivors that were
                // unkillable by construction -- three mutants of a test on a
                // quantity the generator never draws below 3, and a `j <= cols`
                // that writes past the vector in a way passing `cols` as the
                // length hides. `cols` is `INT(SIZE(...), C_INT32_T)` or 0 in
                // the populator and is never negative, so the test is dead code
                // the reference does not have.
                const int32_t rows = CntrPar->n_OL_CableControl_rows;
                const int32_t cols = CntrPar->n_OL_CableControl_cols;
                std::vector<double> row;
                for (int32_t j = 0; j < cols; ++j) {
                    row.push_back(
                        CntrPar->OL_CableControl[static_cast<size_t>(j) * rows +
                                                 (I_GROUP - 1)]);
                }
                LocalVar->CC_DesiredL[I_GROUP - 1] =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               row.data(), static_cast<int>(row.size()),
                               LocalVar->Time, ErrVar);
            // ENDIF
            }
        // ENDDO
        }

    // END IF
    //
    // NO THIRD ARM. A `CC_Mode` outside {1,2} -- including 0, which is what
    // every scenario that does not enable this unit holds it at -- falls through
    // both tests and `CC_DesiredL` keeps the values it arrived with, which the
    // two loops below then still filter, integrate and write to `avrSWAP`. That
    // is a real, reachable behaviour of the reference and not an oversight to be
    // closed here (P7). It is also unreachable through DISCON, whose call site
    // sits behind `IF (CntrPar%CC_Mode > 0)`; the harness calls this procedure
    // directly and does reach it.
    }

    // ! Convert desired to actuated line length and delta length for all groups
    // DO I_GROUP = 1, CntrPar%CC_Group_N
    //
    // UNGUARDED, AS THE REFERENCE IS. `CC_Group_N` bounds this loop and bounds
    // none of the arrays it subscripts: `CC_DesiredL`, `CC_ActuatedDL` and
    // `CC_ActuatedL` are all fixed at 12 by the type. The relation that holds in
    // the shipped program is the input reader's, and it is stated in
    // `harness/ranges.toml` where the corpus is constrained -- adding a test
    // here would be a behaviour the reference does not have.
    for (I_GROUP = 1; I_GROUP <= CntrPar->CC_Group_N; ++I_GROUP) {
        // ! Get Actuated deltaL
        // LocalVar%CC_ActuatedDL(I_GROUP) = SecLPFilter_Vel( &
        //     LocalVar%CC_DesiredL(I_GROUP), LocalVar%DT, 2*PI/CntrPar%CC_ActTau, &
        //     REAL(1.0,DbKi), LocalVar%FP, LocalVar%iStatus, LocalVar%restart, &
        //     objInst%instSecLPFV)
        //
        // `2*PI/CntrPar%CC_ActTau`: `*` and `/` have equal precedence and
        // associate LEFT TO RIGHT, so this is `(2*PI) / CC_ActTau` and NOT
        // `2 * (PI / CC_ActTau)`. Transcribed in that order. The integer 2 is
        // converted to real before the multiply; `2.0 * PI` is an exact scaling
        // by a power of two, so the only rounding in the expression is the
        // division -- which is the reference's single rounding too.
        //
        // `REAL(1.0,DbKi)` is the `Damp` argument and is exactly 1.0.
        //
        // THE OPTIONAL `InitialValue` IS NOT SUPPLIED. `SecLPFilter_Vel`'s last
        // dummy is OPTIONAL and this call site omits it, so the bridge's
        // presence flag is 0 and the value slot is unread; `0, 0.0` is the same
        // spelling every other caller in this campaign uses.
        //
        // `LocalVar%restart` is LOGICAL and crosses as `int8_t`; `? 1 : 0`
        // matches the `MERGE(1_C_INT, 0_C_INT, reset)` the generated wrapper
        // writes. `objInst%instSecLPFV` is INOUT -- the callee post-increments
        // it -- so it is passed by address and the caller sees the advance.
        LocalVar->CC_ActuatedDL[I_GROUP - 1] = seclpfilter_vel_c(
            LocalVar->CC_DesiredL[I_GROUP - 1], LocalVar->DT,
            2.0 * PI / CntrPar->CC_ActTau, 1.0,
            &LocalVar->FP, LocalVar->iStatus, LocalVar->restart ? 1 : 0,
            &objInst->instSecLPFV, 0, 0.0);

        // ! Integrate
        // LocalVar%CC_ActuatedL(I_GROUP) = PIController( &
        //     LocalVar%CC_ActuatedDL(I_GROUP), 0.0_DbKi, 1.0_DbKi, &
        //     -1000.0_DbKi, 1000.0_DbKi, LocalVar%DT, LocalVar%CC_ActuatedDL(1), &
        //     LocalVar%piP, LocalVar%restart, objInst%instPI)
        //
        // THE SEVENTH ARGUMENT IS ELEMENT 1, NOT ELEMENT I_GROUP. `I0` is the
        // value `PIController` uses to (re)initialise its integrator on a reset,
        // and the reference reads `CC_ActuatedDL(1)` on every iteration -- so on
        // the first iteration it is the value the line above has just written,
        // and on every later one it is the FIRST group's, not this group's.
        // Transcribed as the subscript the reference writes; a `I_GROUP - 1`
        // here would be indistinguishable at CC_Group_N = 1, which is what every
        // scenario runs.
        LocalVar->CC_ActuatedL[I_GROUP - 1] = picontroller_c(
            LocalVar->CC_ActuatedDL[I_GROUP - 1], 0.0, 1.0,
            -1000.0, 1000.0, LocalVar->DT, LocalVar->CC_ActuatedDL[0],
            &LocalVar->piP, LocalVar->restart ? 1 : 0, &objInst->instPI);
    // END DO
    }

    // ! Assign to avrSWAP
    // DO I_GROUP = 1, CntrPar%CC_Group_N
    //     avrSWAP(CntrPar%CC_GroupIndex(I_GROUP))     = LocalVar%CC_ActuatedL(I_GROUP)
    //     avrSWAP(CntrPar%CC_GroupIndex(I_GROUP)+1)   = LocalVar%CC_ActuatedDL(I_GROUP)
    // END DO
    //
    // TWO SLOTS PER GROUP, AND THE SECOND IS THE ONE THAT MAKES THIS UNIT
    // DIFFERENT FROM #43. `CC_GroupIndex(I_GROUP)` is a drawn VALUE used as an
    // `avrSWAP` subscript, and the reference also writes one PAST it, so the
    // last element it can touch is `CC_GroupIndex(I_GROUP) + 1`. `avrSWAP` is an
    // assumed-size dummy, `REAL(ReKi) :: avrSWAP(*)`, so the reference has no
    // extent to check against even in principle; the bound the corpus is allowed
    // to draw is stated in `harness/ranges.toml` and it is one lower than unit
    // #43's for exactly this reason.
    //
    // BOTH ASSIGNMENTS NARROW. `avrSWAP` is REAL(ReKi) = REAL(4) and both
    // `CC_ActuatedL` and `CC_ActuatedDL` are REAL(DbKi) = REAL(8), so the
    // Fortran assignment rounds double to single. The casts are written out
    // because a silent implicit conversion here is the one place a reader could
    // mistake this for a copy.
    for (I_GROUP = 1; I_GROUP <= CntrPar->CC_Group_N; ++I_GROUP) {
        avrSWAP[CntrPar->CC_GroupIndex[I_GROUP - 1] - 1] =
            static_cast<float>(LocalVar->CC_ActuatedL[I_GROUP - 1]);
        avrSWAP[CntrPar->CC_GroupIndex[I_GROUP - 1] + 1 - 1] =
            static_cast<float>(LocalVar->CC_ActuatedDL[I_GROUP - 1]);
    }

    // ! Add RoutineName to error message
    // IF (ErrVar%aviFAIL < 0) THEN
    //     ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    // ENDIF
    //
    // `RoutineName` is `'StructuralControl'` -- see its declaration. The
    // concatenation is the reference's, character for character.
    if (ErrVar->aviFAIL < 0) {
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }
}
