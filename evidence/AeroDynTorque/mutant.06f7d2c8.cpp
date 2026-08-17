// VIT Translation Scaffold
// Function: AeroDynTorque
// Source: Functions.f90
// Module: Functions
// Fortran: FUNCTION AeroDynTorque(RotSpeed, BldPitch, LocalVar, CntrPar, PerfData, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: e668698745be
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-17T17:37:09Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature; the body is transcribed statement for statement. Six executable
// statements, one callee, one branch.
//
// THREE GROUPING DECISIONS, AND EVERY ONE OF THEM IS A ROUNDING DECISION.
//
//   PI*CntrPar%WE_BladeRadius**2         `**` binds tighter than `*`, so this
//                                        is PI * (R*R), NOT (PI*R) * R. The
//                                        check registry's `exponent-grouping`
//                                        rule; 1-3 ULP if got wrong.
//   LocalVar%WE_Vw**3                    an INTEGER exponent, so gfortran
//                                        expands it by repeated multiplication
//                                        rather than calling libm `pow`. Its
//                                        addition chain for 3 is x**2 * x, and
//                                        multiplication is commutative in IEEE,
//                                        so (v*v)*v is the shape and is also
//                                        the only shape.
//   RotSpeed*R/WindSpeed                 `*` and `/` have EQUAL precedence and
//                                        associate left to right: (RotSpeed*R)
//                                        / WindSpeed, one multiply then one
//                                        divide. Not RotSpeed * (R/WindSpeed).
//
// The final product is likewise left-associated exactly as written:
//
//     0.5*(RhoAir*RotorArea)*(Vw3/RotSpeed)*Cp
//   = ((0.5 * (RhoAir*RotorArea)) * (Vw3/RotSpeed)) * Cp
//
// and the two parenthesised groups are the reference's own parentheses, kept.
//
// `WE_Vw` IS READ TWICE AND THE TWO READS ARE NOT THE SAME QUANTITY. The
// denominator of the tip-speed ratio is `MAX(WE_Vw, EPSILON(1.0_DbKi))`; the
// cube in the torque is the RAW `WE_Vw`. So a negative or zero wind speed is
// floored for `Lambda` and is not floored for the cube -- at `WE_Vw = 0` the
// reference computes `Lambda = RotSpeed*R/2.22e-16` (enormous, off the top of
// the TSR table) and a numerator of exactly 0. Transcribed as written; the
// asymmetry is upstream's.
//
// MAX IS `fmax`, NOT A TERNARY, and it is measured rather than read: unit #24
// ran gfortran's own MIN(MAX(...)) at this campaign's flags against all three
// spellings over 12,167 triples and both branch spellings differ at a signed
// zero and at a NaN (evidence/saturate/saturate_expr_sweep.*). gfortran's
// intrinsic IS fmax. Both of this unit's MAXes take it.
//
// THE ERROR-MESSAGE BRANCH IS REACHED BY NO GATE SCENARIO. Clean
// `Functions.f90:447` -- the prefix assignment -- has ZERO hits in all 27
// scenarios, while :446 (the `IF`) and every other executable line of this
// procedure carry 78,756 in scenario 1 alone. `ErrVar%aviFAIL < 0` never holds
// at a Cp lookup in a running simulation. The differential harness is the layer
// that reaches it: `aviFAIL` is a plain INTEGER scalar the generator varies,
// and the callee can also set it. Recorded here so that a survivor at this
// branch is read as a gate blind spot rather than as an unkillable site.

#include "vit_types.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

namespace {

// CHARACTER(*), PARAMETER :: RoutineName = 'AeroDynTorque'
constexpr std::string_view RoutineName = "AeroDynTorque";

// Constants.f90:24 and :22, as the reference's own decimal literals. NOT
// `M_PI`, and not `PI/180`: `PI` here is 3.14159265359, twelve significant
// digits, which is NOT the nearest double to pi -- it differs from `M_PI` in
// the 12th digit and the gate red test for unit #47 measured 97,118 of
// 5,252,000 values moving on exactly that perturbation. `R2D` is likewise
// 57.2957795130 and not `180/M_PI`.
constexpr double PI = 3.14159265359;
constexpr double R2D = 57.2957795130;

// `ErrVar%ErrMsg = <expr>` on a `CHARACTER(:), ALLOCATABLE` field is a
// REALLOCATING assignment: the field's new LEN is the right-hand side's. The
// view carries a finite staging buffer, so an assignment that does not fit is
// REFUSED and reported rather than truncated -- a shortened message is the one
// wrong answer a byte comparison cannot tell from a right one. Copied from
// interp1d (unit #23) and interp2d (unit #45), the two units this one's
// messages come back through.
void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {
    if (ErrVar->ErrMsg == nullptr) {
        std::fprintf(stderr,
                     "VIT: AeroDynTorque: ErrVar%%ErrMsg has no staging buffer; "
                     "the assignment of %d bytes is refused\n",
                     static_cast<int>(s.size()));
        return;
    }
    if (static_cast<int>(s.size()) > ErrVar->n_ErrMsg_cap) {
        std::fprintf(stderr,
                     "VIT: AeroDynTorque: ErrVar%%ErrMsg needs %d bytes, the staging "
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
// undefined behaviour rather than a wrong answer. A NEGATIVE length is the
// view's NOT-ALLOCATED convention and collapses to the same empty string a zero
// length gives, which is correct for both.
std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
    const int n = ErrVar->n_ErrMsg;
    const std::string_view v(ErrVar->ErrMsg, n > 0 ? static_cast<size_t>(n) : 1);
    return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
}

}  // namespace

double AeroDynTorque(double RotSpeed, double BldPitch,
                     localvariables_view_t* LocalVar,
                     controlparameters_view_t* CntrPar,
                     performancedata_view_t* PerfData,
                     errorvariables_view_t* ErrVar) {
    double AeroDynTorque_result;

    // Find Torque
    // RotorArea = PI*CntrPar%WE_BladeRadius**2
    const double RotorArea = PI * (CntrPar->WE_BladeRadius * CntrPar->WE_BladeRadius);
    // WindSpeed = MAX(LocalVar%WE_Vw,EPSILON(1.0_DbKi))
    //
    // EPSILON(1.0_DbKi) is the double's machine epsilon -- the spacing above
    // 1.0, 2.220446049250313e-16 -- which is exactly `DBL_EPSILON`. It is NOT
    // TINY (the smallest normal, 2.2e-308) and it is not `nextafter`.
    const double WindSpeed = std::fmax(LocalVar->WE_Vw, DBL_EPSILON);
    // Lambda = RotSpeed*CntrPar%WE_BladeRadius/WindSpeed
    const double Lambda = RotSpeed * CntrPar->WE_BladeRadius / WindSpeed;

    // Compute Cp
    // Cp = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat,
    //               BldPitch*R2D, Lambda, ErrVar)
    //
    // THE TABLE AXES ARE PASSED IN THE ORDER THE REFERENCE PASSES THEM, and it
    // is not the order the field names suggest: `Beta_vec` is interp2d's
    // `xData` and `TSR_vec` is its `yData`, so `Cp_mat` is indexed
    // (TSR, Beta) -- rows are TSR, columns are Beta. `n_Cp_mat_rows` is
    // therefore the count interp2d checks against `SIZE(yData)`, which is
    // `n_TSR_vec`, and `n_Cp_mat_cols` against `n_Beta_vec`. Getting the two
    // extents the other way round would make both of interp2d's size checks
    // fire on every square-table case and neither on a rectangular one.
    const double Cp = interp2d_c(PerfData->Beta_vec, PerfData->n_Beta_vec,
                                 PerfData->TSR_vec, PerfData->n_TSR_vec,
                                 PerfData->Cp_mat, PerfData->n_Cp_mat_rows,
                                 PerfData->n_Cp_mat_cols,
                                 BldPitch * R2D, Lambda, ErrVar);

    // AeroDynTorque = 0.5*(CntrPar%WE_RhoAir*RotorArea)*(LocalVar%WE_Vw**3/RotSpeed)*Cp
    AeroDynTorque_result = 0.5 * (CntrPar->WE_RhoAir * RotorArea) *
                           (((LocalVar->WE_Vw * LocalVar->WE_Vw) * LocalVar->WE_Vw) / RotSpeed) *
                           Cp;
    // AeroDynTorque = MAX(AeroDynTorque, 0.0_DbKi)
    AeroDynTorque_result = std::fmax(AeroDynTorque_result, 0.0);

    // Add RoutineName to error message
    if (ErrVar->aviFAIL < 0) {
        // ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));
    }

    return AeroDynTorque_result;
}
