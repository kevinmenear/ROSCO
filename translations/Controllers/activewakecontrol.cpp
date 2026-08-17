// VIT Translation Scaffold
// Function: ActiveWakeControl
// Source: Controllers.f90
// Module: Controllers
// Fortran: SUBROUTINE ActiveWakeControl(CntrPar, LocalVar, DebugVar, objInst)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: ad1999f288c0
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-17T14:27:35Z
//
// CONTRACT: mirror (plan.json). Every input and every output crosses the
// signature -- the four dummies are the whole state this unit reads and writes
// -- and the body is transcribed statement for statement.
//
// FIVE ARMS, AND EVERY ONE OF THEM IS EXERCISED BY A DIFFERENT SCENARIO. Read
// from coverage/line_coverage.json against the clean source, all 27 scenarios
// (`ActiveWakeControl` at Controllers.f90:714-881 at 54dd134):
//
//   :748  IF (AWC_Mode == 1)                 tested in 5, 8, 11, 15, 21, 22, 27
//   :750-:766  the SNL complex-angle arm     scenario 11 ONLY,  15,999 calls
//   :770  ELSEIF (AWC_Mode == 2)             tested in 5, 8, 15, 21, 22, 27
//   :773-:794  the open-loop Coleman arm     scenario 15 ONLY,  15,999 calls
//   :797  ELSEIF (AWC_Mode == 3 .OR. == 4)   tested in 5, 8, 21, 22, 27
//   :801-:840  the closed-loop PI/PR arm     5, 8, 21, 27
//   :815  the ResController call (Mode 4)    5, 8, 27
//   :818  the PIController  call (Mode 3)    scenario 21 ONLY
//   :843  ELSEIF (AWC_Mode == 5)             tested in 22
//   :845-:876  the Strouhal arm              scenario 22 ONLY,  15,999 calls
//   :805-:806  IF (AWC_harmonic(1) == 0)     0 HITS, ALL 27 SCENARIOS
//   :830       the AWC_harmonic(1) == 0 arm  0 HITS, ALL 27 SCENARIOS
//
// So the gate can see all five modes, and it cannot see either arm guarded by
// `AWC_harmonic(1) == 0` -- no shipped DISCON input sets that harmonic to zero.
// Both are reachable by the differential harness, which draws `AWC_harmonic`
// freely; where that is measured is recorded in evidence/ActiveWakeControl/,
// not asserted here.
//
// FOUR OF THE REFERENCE'S LOCALS ARE IMPLICITLY `SAVE`. A Fortran declaration
// that carries an initialiser has the SAVE attribute whether or not the keyword
// appears, so these four are one instance for the whole program and NOT
// re-initialised per call:
//
//     COMPLEX(DbKi)            :: complexI    = (0.0, 1.0)   never written
//     REAL(DbKi), DIMENSION(2) :: AWC_TiltYaw = [0.0, 0.0]   LOAD-BEARING
//     REAL(DbKi), DIMENSION(2) :: Error       = [0.0, 0.0]   LOAD-BEARING
//     REAL(DbKi)               :: StartTime   = 0.0          LOAD-BEARING
//
// Three of the four are read on a path that does not write them on that call:
//
//   * `StartTime` is assigned only inside `IF (AWC_harmonic(1) == 0)` and is
//     compared against `LocalVar%Time` on every Mode 3/4 call. With a non-zero
//     harmonic -- which is every shipped scenario -- it holds 0.0 forever.
//   * `AWC_TiltYaw` is reset per call ONLY in Mode 2. In Modes 3, 4 and 5 the
//     assignment sits inside `IF (LocalVar%Time > StartTime)` / `IF (Time >
//     1/AWC_freq(1))`, so a call that fails that guard passes the PREVIOUS
//     call's tilt/yaw into the inverse Coleman transform.
//   * `Error` is written only inside the same guarded loop and is read
//     unconditionally at `DebugVar%axisYaw_2P = Error(1)`.
//
// They become function-local `static`, which is what VIT's own check registry
// states the construct maps to and what gfortran compiles SAVE into: one
// instance for the whole program, in .data with the declared initialiser.
// `complexI` is never written and could have been a constant; it is declared
// the way the reference declares it because the contract is mirror.
//
// THE COMPLEX ARITHMETIC IS CALLED, NOT EXPANDED. `EXP(complexI*AWC_angle(K))`
// is gfortran's COMPLEX(8) `EXP`, which lowers to libm's `cexp`.
// `std::exp(std::complex<double>)` in libstdc++ is `__builtin_cexp`, i.e. the
// same libm entry point, so the two agree bit for bit. Expanding it by hand as
// `cos(t) + i*sin(t)` would be a re-implementation with its own rounding and is
// deliberately not done.

#include "vit_types.h"

#include <cmath>
#include <complex>

namespace {

// The Fortran reads `PI`/`D2R` from the Constants module by name. Restated here
// rather than recomputed, and the distinction is load-bearing: 3.14159265359
// and 0.01745329251 are TRUNCATED decimals and not the correctly rounded
// values -- Pi is 3.141592653589793 -- so `M_PI` is a different number. Copied
// character for character from `rosco/controller/src/Constants.f90:23-24`, the
// same way `translations/Controllers/yawratecontrol.cpp` and
// `translations/Filters/prefiltermeasuredsignals.cpp` restate them.
constexpr double PI  = 3.14159265359;
constexpr double D2R = 0.01745329251;

}  // namespace

void ActiveWakeControl(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                       debugvariables_t* DebugVar, objectinstances_t* objInst) {
    // REAL(DbKi), PARAMETER :: phi1 = 0.0
    // REAL(DbKi), PARAMETER :: phi2 = 2.0/3.0*PI
    // REAL(DbKi), PARAMETER :: phi3 = 4.0/3.0*PI
    //
    // `2.0/3.0*PI` associates left to right -- the division happens first and
    // its result is then multiplied. Transcribed in that shape, not as
    // `2.0*PI/3.0`.
    constexpr double phi1 = 0.0;
    constexpr double phi2 = 2.0 / 3.0 * PI;
    constexpr double phi3 = 4.0 / 3.0 * PI;

    // REAL(DbKi), DIMENSION(3) :: AWC_angle
    //
    // No initialiser, so no SAVE: an ordinary automatic. Written by
    // ColemanTransformInverse (all three elements) on every path that reads it
    // except Mode 2 with `AWC_NumModes == 0`, where the reference reads it
    // uninitialised. That case is INADMISSIBLE rather than translated: see
    // harness/ranges.toml's pin on `CntrPar_AWC_NumModes`.
    double AWC_angle[3];

    // COMPLEX(DbKi), DIMENSION(3) :: AWC_complexangle
    //
    // Declared by the reference and never referenced by it -- the body uses
    // `LocalVar%AWC_complexangle`, the view field, throughout. A declaration is
    // not a statement, so nothing is transcribed for it.

    // COMPLEX(DbKi) :: complexI = (0.0, 1.0)   -- implicitly SAVE, never written
    static const std::complex<double> complexI(0.0, 1.0);

    // INTEGER(IntKi) :: Imode, K
    int Imode;
    int K;
    // REAL(DbKi) :: clockang, omega, amp
    double clockang;
    double omega;
    double amp;

    // REAL(DbKi), DIMENSION(2) :: AWC_TiltYaw = [0.0, 0.0]   -- implicitly SAVE
    // REAL(DbKi), DIMENSION(2) :: Error       = [0.0, 0.0]   -- implicitly SAVE
    static double AWC_TiltYaw[2] = {0.0, 0.0};
    static double Error[2] = {0.0, 0.0};

    // REAL(DbKi), DIMENSION(2) :: FixedFrameM   -- no initialiser, automatic
    double FixedFrameM[2];
    // REAL(DbKi) :: StrAzimuth                  -- no initialiser, automatic
    double StrAzimuth;
    // REAL(DbKi) :: StartTime = 0.0             -- implicitly SAVE
    static double StartTime = 0.0;

    // Compute the AWC pitch settings, complex number approach
    if (CntrPar->AWC_Mode == 1) {
        // LocalVar%AWC_complexangle = 0.0D0
        //
        // Whole-array assignment: all three elements, regardless of NumBl, and
        // both components of each -- assigning a REAL to a COMPLEX sets the
        // imaginary part to zero.
        for (int i = 0; i < 3; ++i) {
            LocalVar->AWC_complexangle[i].re = 0.0;
            LocalVar->AWC_complexangle[i].im = 0.0;
        }

        for (Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode) {
            clockang = CntrPar->AWC_clockangle[Imode - 1] * PI / 180.0;
            omega = CntrPar->AWC_freq[Imode - 1] * PI * 2.0;
            // `CntrPar%AWC_n(Imode)` is INTEGER; the reference multiplies it
            // into a REAL expression, so it converts to REAL first.
            AWC_angle[0] = omega * LocalVar->Time
                           - static_cast<double>(CntrPar->AWC_n[Imode - 1])
                                 * (LocalVar->Azimuth + phi1 + clockang);
            AWC_angle[1] = omega * LocalVar->Time
                           - static_cast<double>(CntrPar->AWC_n[Imode - 1])
                                 * (LocalVar->Azimuth + phi2 + clockang);
            AWC_angle[2] = omega * LocalVar->Time
                           - static_cast<double>(CntrPar->AWC_n[Imode - 1])
                                 * (LocalVar->Azimuth + phi3 + clockang);
            // Add the forcing contribution to LocalVar%AWC_complexangle
            amp = CntrPar->AWC_amp[Imode - 1] * PI / 180.0;
            for (K = 1; K <= LocalVar->NumBl; ++K) {  // Loop through all blades
                const std::complex<double> acc(LocalVar->AWC_complexangle[K - 1].re,
                                               LocalVar->AWC_complexangle[K - 1].im);
                const std::complex<double> contrib =
                    acc + amp * std::exp(complexI * AWC_angle[K - 1]);
                LocalVar->AWC_complexangle[K - 1].re = contrib.real();
                LocalVar->AWC_complexangle[K - 1].im = contrib.imag();
            }
        }

        for (K = 1; K <= LocalVar->NumBl; ++K) {  // Loop through all blades, apply AWC_angle
            // REAL(z) with no KIND on a COMPLEX(DbKi) is the real part at
            // default real kind, which is 8 under -fdefault-real-8.
            LocalVar->PitCom[K - 1] =
                LocalVar->PitCom[K - 1] + LocalVar->AWC_complexangle[K - 1].re;
        }

        // Open-loop Coleman transform method
    } else if (CntrPar->AWC_Mode == 2) {
        // Calculate reference OL blade signals
        AWC_TiltYaw[0] = 0.0;
        AWC_TiltYaw[1] = 0.0;
        for (Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode) {
            AWC_TiltYaw[Imode - 1] =
                D2R * CntrPar->AWC_amp[Imode - 1]
                * std::sin(LocalVar->Time * 2.0 * PI * CntrPar->AWC_freq[Imode - 1]
                           + CntrPar->AWC_clockangle[Imode - 1] * D2R);
            if (CntrPar->AWC_NumModes == 1) {
                AWC_TiltYaw[1] = D2R * CntrPar->AWC_amp[0]
                                 * std::sin(LocalVar->Time * 2.0 * PI * CntrPar->AWC_freq[0]
                                            + 2.0 * CntrPar->AWC_clockangle[0] * D2R);
            }

            // Inverse Coleman Transformation with phase offset
            colemantransforminverse_c(AWC_TiltYaw[0], AWC_TiltYaw[1], LocalVar->Azimuth,
                                      CntrPar->AWC_harmonic[Imode - 1],
                                      CntrPar->AWC_phaseoffset * D2R, AWC_angle);
        }

        for (K = 1; K <= LocalVar->NumBl; ++K) {  // Loop through all blades, apply AWC_angle
            LocalVar->PitCom[K - 1] = LocalVar->PitCom[K - 1] + AWC_angle[K - 1];
        }

        // DEBUG VARIABLES
        DebugVar->axisTilt_2P = AWC_TiltYaw[0];
        DebugVar->axisYaw_2P = AWC_TiltYaw[1];
        // ColemanTransform OVERWRITES both elements of AWC_TiltYaw here, which
        // is why the 2P debug pair is read before the call and the 1P pair
        // after it. The order is the reference's, not an accident of layout.
        colemantransform_c(LocalVar->BlPitch, LocalVar->Azimuth, CntrPar->AWC_harmonic[0],
                           &AWC_TiltYaw[0], &AWC_TiltYaw[1]);

        DebugVar->axisTilt_1P = AWC_TiltYaw[0];
        DebugVar->axisYaw_1P = AWC_TiltYaw[1];

        // Closed-loop PI / PR controller
    } else if ((CntrPar->AWC_Mode == 3) || (CntrPar->AWC_Mode == 4)) {
        //! If AWC_harmonic > 0, use AWC_NumModes=2

        colemantransform_c(LocalVar->rootMOOPF, LocalVar->Azimuth, CntrPar->AWC_harmonic[0],
                           &FixedFrameM[0], &FixedFrameM[1]);

        if (CntrPar->AWC_harmonic[0] == 0) {
            // Calculate mean moments, subtract later to get zero-mean tilt and yaw
            LocalVar->TiltMean = LocalVar->TiltMean + FixedFrameM[0];
            // Integer 1 divided by a REAL is a REAL division, not an integer one.
            StartTime = 1.0 / CntrPar->AWC_freq[0];
        }

        if (LocalVar->Time > StartTime) {
            for (Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode) {
                // `LocalVar%n_DT` is INTEGER: `n_DT+1` is evaluated in integer
                // arithmetic and only then converted for the division.
                Error[Imode - 1] =
                    CntrPar->AWC_amp[Imode - 1]
                        * std::sin(LocalVar->Time * 2.0 * PI * CntrPar->AWC_freq[Imode - 1]
                                   + CntrPar->AWC_clockangle[Imode - 1] * D2R)
                    + (FixedFrameM[Imode - 1]
                       - LocalVar->TiltMean / static_cast<double>(LocalVar->n_DT + 1));

                if (CntrPar->AWC_Mode == 4) {
                    AWC_TiltYaw[Imode - 1] = rescontroller_c(
                        Error[Imode - 1], CntrPar->AWC_CntrGains[0], CntrPar->AWC_CntrGains[1],
                        CntrPar->AWC_freq[Imode - 1], CntrPar->PC_MinPit, CntrPar->PC_MaxPit,
                        LocalVar->DT, &LocalVar->resP, LocalVar->restart ? 1 : 0,
                        &objInst->instRes);
                } else {
                    AWC_TiltYaw[Imode - 1] = picontroller_c(
                        Error[Imode - 1], CntrPar->AWC_CntrGains[0], CntrPar->AWC_CntrGains[1],
                        CntrPar->PC_MinPit, CntrPar->PC_MaxPit, LocalVar->DT, 0.0,
                        &LocalVar->piP, LocalVar->restart ? 1 : 0, &objInst->instPI);
                }
            }
        }

        // Pass tilt and yaw axis through the inverse Coleman transform to get the commanded
        // pitch angles
        colemantransforminverse_c(AWC_TiltYaw[0], AWC_TiltYaw[1], LocalVar->Azimuth,
                                  CntrPar->AWC_harmonic[0], CntrPar->AWC_phaseoffset * D2R,
                                  AWC_angle);

        for (K = 1; K <= LocalVar->NumBl; ++K) {  // Loop through all blades, apply AWC_angle
            if (CntrPar->AWC_harmonic[0] == 0) {
                LocalVar->PitCom[K - 1] = LocalVar->PitCom[K - 1] + AWC_TiltYaw[0];
            } else {
                LocalVar->PitCom[K - 1] = LocalVar->PitCom[K - 1] + AWC_angle[K - 1];
            }
        }

        // DEBUG VARIABLES
        DebugVar->axisTilt_1P = AWC_TiltYaw[0];
        DebugVar->axisYaw_1P =
            -FixedFrameM[0] + LocalVar->TiltMean / static_cast<double>(LocalVar->n_DT + 1);
        DebugVar->axisTilt_2P =
            CntrPar->AWC_amp[0]
            * std::sin(LocalVar->Time * 2.0 * PI * CntrPar->AWC_freq[0]
                       + CntrPar->AWC_clockangle[0] * D2R);
        DebugVar->axisYaw_2P = Error[0];

        // Closed-loop Strouhal transform method
    } else if (CntrPar->AWC_Mode == 5) {
        StrAzimuth = wrap_360_c(360.0 * LocalVar->Time * CntrPar->AWC_freq[0]) * D2R;

        colemantransform_c(LocalVar->rootMOOPF, LocalVar->Azimuth, CntrPar->AWC_harmonic[0],
                           &FixedFrameM[0], &FixedFrameM[1]);

        // Calculate mean tilt and yaw moments to subtract
        LocalVar->TiltMean = LocalVar->TiltMean + FixedFrameM[0];
        LocalVar->YawMean = LocalVar->YawMean + FixedFrameM[1];

        // Calculate error with zero-mean moments
        Error[0] = CntrPar->AWC_amp[0]
                   + std::sin(StrAzimuth + CntrPar->AWC_clockangle[0] * D2R)
                         * (FixedFrameM[0]
                            - LocalVar->TiltMean / static_cast<double>(LocalVar->n_DT + 1))
                   + std::sin(StrAzimuth + CntrPar->AWC_clockangle[1] * D2R)
                         * (FixedFrameM[1]
                            - LocalVar->YawMean / static_cast<double>(LocalVar->n_DT + 1));

        // PI Control
        if (LocalVar->Time > 1.0 / CntrPar->AWC_freq[0]) {
            AWC_TiltYaw[0] = picontroller_c(Error[0], CntrPar->AWC_CntrGains[0],
                                            CntrPar->AWC_CntrGains[1], CntrPar->PC_MinPit,
                                            CntrPar->PC_MaxPit, LocalVar->DT, 0.0,
                                            &LocalVar->piP, LocalVar->restart ? 1 : 0,
                                            &objInst->instPI);
        }

        // Pass tilt and yaw axis through the inverse Strouhal + Coleman transform to get the
        // commanded pitch angles
        colemantransforminverse_c(
            std::sin(StrAzimuth + CntrPar->AWC_clockangle[0] * D2R) * AWC_TiltYaw[0],
            std::sin(StrAzimuth + CntrPar->AWC_clockangle[1] * D2R) * AWC_TiltYaw[0],
            LocalVar->Azimuth, CntrPar->AWC_harmonic[0], CntrPar->AWC_phaseoffset * D2R,
            AWC_angle);

        for (K = 1; K <= LocalVar->NumBl; ++K) {  // Loop through all blades, apply AWC_angle
            LocalVar->PitCom[K - 1] = LocalVar->PitCom[K - 1] + AWC_angle[K - 1];
        }

        // DEBUG VARIABLES
        DebugVar->axisTilt_1P =
            std::sin(StrAzimuth + CntrPar->AWC_clockangle[0] * D2R) * AWC_TiltYaw[0];
        DebugVar->axisYaw_1P =
            -FixedFrameM[0] + LocalVar->TiltMean / static_cast<double>(LocalVar->n_DT + 1);
        DebugVar->axisTilt_2P =
            CntrPar->AWC_amp[0] * std::sin(StrAzimuth + CntrPar->AWC_clockangle[0] * D2R);
        DebugVar->axisYaw_2P = Error[0];
    }
}
