// COUNTING PROBE, not a translation. The real body with one counter per arm
// and an atexit that writes them out. It answers ONE question, which the
// harness's own PASS cannot: does the generated corpus ever ENTER the OpenLoop
// arm, and does it ever reach the `WRITE(401,*)` inside it? A green over 3596
// cases is a statement about the arms those cases run, and nothing else.
//
//   bash evidence/PowerControlSetpoints/run_probe.sh \
//        evidence/PowerControlSetpoints/probes/arm_census.cpp \
//        evidence/PowerControlSetpoints/probes/arm_census.json "arm census"
//
// Every counter increments on the C++ side only, so a count of zero means the
// TRANSLATION never took that path on that corpus. The Fortran reference takes
// the same path on the same case by construction -- the harness supplies both
// sides the same inputs -- which is why one side is enough to answer it.
#include <cstdio>
#include <cstdlib>
namespace probe {
long n_call = 0, n_mode2 = 0, n_constant = 0, n_openloop = 0, n_zmq = 0,
     n_fallthrough = 0, n_else = 0, n_speed_interp = 0, n_speed_one = 0,
     n_torque_interp = 0, n_torque_one = 0, n_pitch_interp = 0, n_pitch_one = 0,
     n_write401 = 0, n_minpitch_interp = 0;
struct Dump {
    ~Dump() {
        std::FILE* f = std::fopen(
            "/workspace/ROSCO-r2/evidence/PowerControlSetpoints/probes/arm_census.txt", "w");
        if (!f) return;
        std::fprintf(f, "calls                        %ld\n", n_call);
        std::fprintf(f, "PRC_Mode == 2                %ld\n", n_mode2);
        std::fprintf(f, "  PRC_Comm == Constant       %ld\n", n_constant);
        std::fprintf(f, "  PRC_Comm == OpenLoop       %ld\n", n_openloop);
        std::fprintf(f, "    Ind_R_Speed  > 0         %ld\n", n_speed_interp);
        std::fprintf(f, "    Ind_R_Speed <= 0         %ld\n", n_speed_one);
        std::fprintf(f, "    WRITE(401,*) executed    %ld\n", n_write401);
        std::fprintf(f, "    Ind_R_Torque  > 0        %ld\n", n_torque_interp);
        std::fprintf(f, "    Ind_R_Torque <= 0        %ld\n", n_torque_one);
        std::fprintf(f, "    Ind_R_Pitch  > 0         %ld\n", n_pitch_interp);
        std::fprintf(f, "    Ind_R_Pitch <= 0         %ld\n", n_pitch_one);
        std::fprintf(f, "  PRC_Comm == ZMQ            %ld\n", n_zmq);
        std::fprintf(f, "  PRC_Comm matched NOTHING   %ld\n", n_fallthrough);
        std::fprintf(f, "  PRC_Min_Pitch = interp1d   %ld\n", n_minpitch_interp);
        std::fprintf(f, "the ELSE arm                 %ld\n", n_else);
        std::fclose(f);
    }
} dump;
}  // namespace probe

#include "vit_types.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

// ---------------------------------------------------------------------------
// `WRITE(401,*) <REAL(8)>`
//
// THE THREE HELPERS BELOW ARE THE SHAPE `translations/ROSCO_IO/debug.cpp`
// ESTABLISHED for this campaign's Fortran output -- `field`, `nonfinite_text`
// and an snprintf-based numeric conversion -- narrowed to the one descriptor
// this unit needs. They are taken from that file rather than re-derived from a
// description of Fortran (P4). Two things had to be ADDED rather than copied,
// and both are measured:
//
//   * debug.cpp's `put_f` states that gfortran's `d == 0` case has no
//     reachable input THERE, every F descriptor in that unit being F20.5.
//     Here it is reachable and it is the single difference between a model
//     that matched and one that did not: `%.0f` drops the decimal point and
//     Fortran's F21.0 keeps it, on 488 of 22,526 records.
//   * list-directed output chooses between the F and E forms per value, which
//     no fixed descriptor in debug.cpp does.
//
// THE RULE, MEASURED. `evidence/PowerControlSetpoints/list_directed_corpus.f90`
// writes 22,526 doubles both as list-directed records and as raw bits; the
// model below reproduces all 22,526 records byte for byte, and the count is in
// evidence/PowerControlSetpoints/ld_probe.txt rather than in this comment.
//
//   every record          exactly 26 characters, then the record terminator
//   non-finite            "NaN" / "Infinity" / "-Infinity" in that 26
//   0.1 <= |v| < 1e17,    F-form at 17 significant digits, right-justified in
//   or v == 0             21, then FIVE trailing blanks
//   otherwise             E-form, one digit and sixteen decimals, exponent
//                         signed and padded to three digits, in the 26
//
// The 21 and the 5 are G-editing's own arithmetic for a three-digit exponent:
// the F field is the record less `e + 2`, and the blanks are where the
// exponent would have gone. They are written as the two numbers gfortran was
// measured to produce, not as `26 - (3 + 2)`, because the measurement is what
// this rests on.
// ---------------------------------------------------------------------------

// Right-justify `text` in a field of `w`, or fill the field with '*' when it
// does not fit -- Fortran's overflow behaviour for every numeric descriptor.
// Copied from debug.cpp.
std::string field(const std::string& text, int w) {
    if (static_cast<int>(text.size()) > w) {
        return std::string(static_cast<size_t>(w), '*');
    }
    return std::string(static_cast<size_t>(w) - text.size(), ' ') + text;
}

// gfortran renders a non-finite value as a WORD, not as digits: "NaN" with no
// sign, "Infinity" carrying it. Copied from debug.cpp, minus the narrow-field
// "Inf" arm -- the field here is 26 and "Infinity" is 8, so that arm has no
// reachable input at this one descriptor and an unreachable arm is worse than
// an absent one. Returns false when the value is finite.
bool nonfinite_text(double v, std::string& out) {
    if (std::isnan(v)) { out = "NaN"; return true; }
    if (std::isinf(v)) {
        out = std::signbit(v) ? "-Infinity" : "Infinity";
        return true;
    }
    return false;
}

// One list-directed record for a REAL(8), without the terminator.
std::string list_directed_real(double v) {
    std::string text;
    if (nonfinite_text(v, text)) {
        return field(text, 26);
    }

    char tmp[512];

    // The decimal exponent is READ OFF the rounded conversion rather than
    // computed with log10: 17 significant digits identify a double uniquely, so
    // this is the exponent the printed digits actually have, and there is no
    // boundary where a log10 and a rounding disagree.
    std::snprintf(tmp, sizeof tmp, "%.16E", v);
    std::string s(tmp);
    const size_t epos = s.find('E');
    const int decexp = std::atoi(s.c_str() + epos + 1);

    // 0.1 <= |v| < 1e17, and v == 0 lands here because "%.16E" of a zero
    // reports an exponent of 0 -- which is the arm gfortran puts it in.
    if (decexp >= -1 && decexp <= 16) {
        // 17 significant digits: `decexp + 1` of them are before the point.
        const int d = 16 - decexp;
        std::snprintf(tmp, sizeof tmp, "%.*f", d, v);
        text = tmp;
        if (d == 0) {
            text += '.';
        }
        return field(text, 21) + std::string(5, ' ');
    }

    const std::string mant = s.substr(0, epos);
    const char esign = s[epos + 1];
    std::string edig = s.substr(epos + 2);
    while (edig.size() > 1 && edig[0] == '0') {
        edig.erase(0, 1);
    }
    if (edig.size() > 3) {
        return field(std::string(27, '*'), 26);
    }
    edig.insert(0, 3 - edig.size(), '0');
    return field(mant + "E" + esign + edig, 26);
}

// The unit itself. Fortran connects unit 401 on the first WRITE with an
// implicit OPEN -- sequential, formatted, positioned at the start of a file
// named `fort.401` in the working directory -- and holds it open until the
// program ends. A function-local static is that connection: opened once, on
// the first record, truncating, and never closed. Same shape as debug.cpp's
// SAVE unit numbers, which are `FILE*` for the same reason.
void write_401(double v) {
    static std::FILE* unit401 = nullptr;
    if (unit401 == nullptr) {
        unit401 = std::fopen("fort.401", "w");
        if (unit401 == nullptr) {
            return;
        }
    }
    const std::string rec = list_directed_real(v);
    std::fwrite(rec.data(), 1, rec.size(), unit401);
    std::fputc('\n', unit401);
}

// Constants.f90:73-75. Copied as the reference declares them; PRC_Comm is
// compared against these names in the Fortran, so the translation compares
// against the same three values rather than against bare literals.
constexpr int PRC_Comm_Constant = 0;
constexpr int PRC_Comm_OpenLoop = 1;
constexpr int PRC_Comm_ZMQ = 2;

}  // namespace

void PowerControlSetpoints(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar,
                           objectinstances_t* objInst, debugvariables_t* DebugVar,
                           errorvariables_view_t* ErrVar) {
    // The reference does not name these two; see the header.
    (void)objInst;
    (void)DebugVar;

    // ! Set up power control
    // IF (CntrPar%PRC_Mode == 2) THEN  ! Using power reference control
    ++probe::n_call;
    if (CntrPar->PRC_Mode == 2) {
        ++probe::n_mode2;
        // IF (CntrPar%PRC_Comm == PRC_Comm_Constant) THEN  ! Constant, from DISCON
        if (CntrPar->PRC_Comm == PRC_Comm_Constant) {
            ++probe::n_constant;
            // LocalVar%PRC_R_Speed  = CntrPar%PRC_R_Speed
            // LocalVar%PRC_R_Torque = CntrPar%PRC_R_Torque
            // LocalVar%PRC_R_Pitch  = CntrPar%PRC_R_Pitch
            //
            // The three scalars are fields of a view-type INOUT argument, so
            // they do not travel back through a C_LOC'd buffer the way an
            // allocatable array does: the integration needs `--reverse-copy`.
            LocalVar->PRC_R_Speed = CntrPar->PRC_R_Speed;
            LocalVar->PRC_R_Torque = CntrPar->PRC_R_Torque;
            LocalVar->PRC_R_Pitch = CntrPar->PRC_R_Pitch;

        // ELSEIF (CntrPar%PRC_Comm == PRC_Comm_OpenLoop) THEN  ! Open loop
        } else if (CntrPar->PRC_Comm == PRC_Comm_OpenLoop) {
            ++probe::n_openloop;

            // IF (CntrPar%Ind_R_Speed > 0) THEN
            //     LocalVar%PRC_R_Speed = interp1d(CntrPar%OL_Breakpoints,
            //                                     CntrPar%OL_R_Speed,
            //                                     LocalVar%OL_Index, ErrVar)
            //     WRITE(401,*) LocalVar%PRC_R_Speed
            // ELSE
            //     LocalVar%PRC_R_Speed = 1.0_DbKi
            // ENDIF
            //
            // The two extents are passed independently because the C signature
            // takes them independently. interp1d's own reference ABORTS on
            // `SIZE(xData) /= SIZE(yData)` while the translation continues
            // (evidence/interp1d/reference.size-mismatch-aborts.txt); at this
            // call site the Fortran cannot produce a mismatch, since
            // `Read_OL_Input` allocates every OL_ array to one row count.
            if (CntrPar->Ind_R_Speed > 0) {
                ++probe::n_speed_interp;
                LocalVar->PRC_R_Speed =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_R_Speed, CntrPar->n_OL_R_Speed,
                               LocalVar->OL_Index, ErrVar);
                ++probe::n_write401;
                write_401(LocalVar->PRC_R_Speed);
            } else {
                ++probe::n_speed_one;
                LocalVar->PRC_R_Speed = 1.0;
            }

            // IF (CntrPar%Ind_R_Torque > 0) THEN
            //     LocalVar%PRC_R_Torque = interp1d(CntrPar%OL_Breakpoints,
            //                                      CntrPar%OL_R_Torque,
            //                                      LocalVar%OL_Index, ErrVar)
            // ELSE
            //     LocalVar%PRC_R_Torque = 1.0_DbKi
            // ENDIF
            if (CntrPar->Ind_R_Torque > 0) {
                ++probe::n_torque_interp;
                LocalVar->PRC_R_Torque =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_R_Torque, CntrPar->n_OL_R_Torque,
                               LocalVar->OL_Index, ErrVar);
            } else {
                ++probe::n_torque_one;
                LocalVar->PRC_R_Torque = 1.0;
            }

            // IF (CntrPar%Ind_R_Pitch > 0) THEN
            //     LocalVar%PRC_R_Pitch = interp1d(CntrPar%OL_Breakpoints,
            //                                     CntrPar%OL_R_Pitch,
            //                                     LocalVar%OL_Index, ErrVar)
            // ELSE
            //     LocalVar%PRC_R_Pitch = 1.0_DbKi
            // ENDIF
            if (CntrPar->Ind_R_Pitch > 0) {
                ++probe::n_pitch_interp;
                LocalVar->PRC_R_Pitch =
                    interp1d_c(CntrPar->OL_Breakpoints, CntrPar->n_OL_Breakpoints,
                               CntrPar->OL_R_Pitch, CntrPar->n_OL_R_Pitch,
                               LocalVar->OL_Index, ErrVar);
            } else {
                ++probe::n_pitch_one;
                LocalVar->PRC_R_Pitch = 1.0;
            }

        // ELSEIF (CntrPar%PRC_Comm == PRC_Comm_ZMQ) THEN  ! ZeroMQ
        } else if (CntrPar->PRC_Comm == PRC_Comm_ZMQ) {
            ++probe::n_zmq;
            // LocalVar%PRC_R_Speed  = LocalVar%ZMQ_R_Speed
            // LocalVar%PRC_R_Torque = LocalVar%ZMQ_R_Torque
            // LocalVar%PRC_R_Pitch  = LocalVar%ZMQ_R_Pitch
            LocalVar->PRC_R_Speed = LocalVar->ZMQ_R_Speed;
            LocalVar->PRC_R_Torque = LocalVar->ZMQ_R_Torque;
            LocalVar->PRC_R_Pitch = LocalVar->ZMQ_R_Pitch;

        // ENDIF
        //
        // NO FOURTH ARM. A `PRC_Comm` outside {0,1,2} falls through all three
        // tests and the three setpoints keep the values they arrived with --
        // which is a real, reachable behaviour of the reference and not an
        // oversight to be closed here (P7).
        } else {
            ++probe::n_fallthrough;
        }

        // ! Set min pitch for power control, will be combined with peak
        // ! shaving min pitch
        // LocalVar%PRC_Min_Pitch = interp1d(CntrPar%PRC_R_Table,
        //                                   CntrPar%PRC_Pitch_Table,
        //                                   LocalVar%PRC_R_Pitch, ErrVar)
        //
        // Inside the PRC_Mode arm and AFTER the PRC_Comm chain, so it reads the
        // PRC_R_Pitch that chain just wrote. Kept in that order rather than
        // folded into the branches: the reference runs it once for all four
        // paths through the chain, including the fall-through above.
        ++probe::n_minpitch_interp;
        LocalVar->PRC_Min_Pitch =
            interp1d_c(CntrPar->PRC_R_Table, CntrPar->n_PRC_R_Table,
                       CntrPar->PRC_Pitch_Table, CntrPar->n_PRC_Pitch_Table,
                       LocalVar->PRC_R_Pitch, ErrVar);

    // ELSE
    } else {
        ++probe::n_else;
        // LocalVar%PRC_R_Speed   = 1.0_DbKi
        // LocalVar%PRC_R_Torque  = 1.0_DbKi
        // LocalVar%PRC_R_Pitch   = 1.0_DbKi
        // LocalVar%PRC_Min_Pitch = CntrPar%PC_FinePit
        LocalVar->PRC_R_Speed = 1.0;
        LocalVar->PRC_R_Torque = 1.0;
        LocalVar->PRC_R_Pitch = 1.0;
        LocalVar->PRC_Min_Pitch = CntrPar->PC_FinePit;
    // ENDIF
    }
}
