// VIT Translation Scaffold
// Function: StateMachine
// Source: ControllerBlocks.f90
// Module: ControllerBlocks
// Fortran: SUBROUTINE StateMachine(CntrPar, LocalVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
// Source MD5: 84900fbeb047
// VIT: 0.1.0
// Status: unverified
// Generated: 2026-08-12T16:38:41Z

#include "vit_types.h"
#include <cstdio>
#include <cstdlib>
// BRANCH-REACHABILITY PROBE -- not the translation. Counts, per leaf, how many
// corpus cases reach it, and prints the tally at exit. Everything else is the
// shipped translation byte for byte.
namespace vitprobe {
static long n[16];
static const char* nm[16] = {
 "init: iStatus==0","init: PitCom(1)>=Rgn3Pitch","init/rgn3: ConstPwr","init/rgn3: ConstTrq",
 "init: Region_2+PC_Disabled","op: PC_ControlMode==1 -> Enabled","op: PC_Disabled",
 "op: BlPitchCMeas>=Rgn3Pitch","op/rgn3: ConstPwr","op/rgn3: ConstTrq",
 "op: GenArTq>=MaxOMTq*1.01","op: Region_2_5","op: Region_3_FBP","op: Region_2",
 "op: Region_1_5","op: Error"};
struct Dump { ~Dump(){ for(int i=0;i<16;i++) fprintf(stderr,"PROBE %-34s %ld\n",nm[i],n[i]); } };
static Dump d;
}
#define HIT(i) (vitprobe::n[i]++)

// The state constants are PARAMETERs in `Constants.f90`, USEd by the
// ControllerBlocks module (the subroutine's own USE names only ROSCO_Types).
// Restated here as constexpr rather than as bare literals for the reason
// `readavrswap.cpp` restates R2D/D2R: the Fortran reads by name, and a
// translation that reads `5` where the reference reads
// `VS_State_Region_3_ConstPwr` is a transcription nobody can check by eye.
// Every value below is copied from `rosco/controller/src/Constants.f90`
// lines 50-70; none is derived.
namespace {
constexpr int VS_Mode_ConstPwr = 1;             // Constants.f90:50
constexpr int VS_FBP_Variable_Pitch = 0;        // Constants.f90:53

constexpr int VS_State_Error = 0;               // Constants.f90:59
constexpr int VS_State_Region_1_5 = 1;          // Constants.f90:60
constexpr int VS_State_Region_2 = 2;            // Constants.f90:61
constexpr int VS_State_Region_2_5 = 3;          // Constants.f90:62
constexpr int VS_State_Region_3_ConstTrq = 4;   // Constants.f90:63
constexpr int VS_State_Region_3_ConstPwr = 5;   // Constants.f90:64
constexpr int VS_State_Region_3_FBP = 6;        // Constants.f90:65

constexpr int PC_State_Disabled = 0;            // Constants.f90:69
constexpr int PC_State_Enabled = 1;             // Constants.f90:70
}  // namespace

void StateMachine(controlparameters_view_t* CntrPar, localvariables_view_t* LocalVar) {
    // The whole unit is a two-output decision tree: it writes `LocalVar%PC_State`
    // and `LocalVar%VS_State` and nothing else, and every path through it writes
    // both. There is no arithmetic to round, so this translation is a
    // transcription of PREDICATES -- which is what makes the comparison
    // operators the load-bearing part of it.
    //
    // Both outputs are INTEGER scalars of the INOUT view argument, so the
    // integration needs `--reverse-copy`: they do not travel back through a
    // C_LOC'd allocatable buffer the way an array field does.

    // IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
    if (LocalVar->iStatus == 0) {
        HIT(0);
        // IF (LocalVar%PitCom(1) >= LocalVar%VS_Rgn3Pitch) THEN ! We are in region 3
        // `PitCom` is DIMENSION(3); the Fortran subscript 1 is element 0 here.
        // Note the initialisation branch tests the COMMANDED pitch and the
        // operational branch below tests the MEASURED pitch (`BlPitchCMeas`) --
        // two different quantities against the same threshold. Transcribed as
        // the reference has it.
        if (LocalVar->PitCom[0] >= LocalVar->VS_Rgn3Pitch) {
            HIT(1);
            LocalVar->PC_State = PC_State_Enabled;
            // IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr) THEN ! Constant power tracking
            if (CntrPar->VS_ConstPower == VS_Mode_ConstPwr) {
                LocalVar->VS_State = VS_State_Region_3_ConstPwr; HIT(2);
            } else {  // Constant torque tracking
                LocalVar->VS_State = VS_State_Region_3_ConstTrq; HIT(3);
            }
        } else {  // We are in Region 2
            // The reference writes VS_State first here and PC_State first in
            // the branch above. The two are distinct fields, so the order is
            // unobservable; it is transcribed in source order anyway, because
            // nothing justifies moving it.
            LocalVar->VS_State = VS_State_Region_2;
            LocalVar->PC_State = PC_State_Disabled; HIT(4);
        }

    // Operational States
    } else {
        // --- Pitch controller state machine ---
        // IF (CntrPar%PC_ControlMode == 1) THEN
        // The reference tests the literal 1, not a named PC_ControlMode
        // constant -- `Constants.f90` declares none for this parameter.
        if (CntrPar->PC_ControlMode == 1) {
            LocalVar->PC_State = PC_State_Enabled; HIT(5);
        } else {
            LocalVar->PC_State = PC_State_Disabled; HIT(6);
        }

        // --- Torque control state machine ---
        // IF (LocalVar%BlPitchCMeas >= LocalVar%VS_Rgn3Pitch) THEN
        if (LocalVar->BlPitchCMeas >= LocalVar->VS_Rgn3Pitch) {
            HIT(7);
            // IF (CntrPar%VS_ConstPower == VS_Mode_ConstPwr) THEN ! Region 3
            if (CntrPar->VS_ConstPower == VS_Mode_ConstPwr) {
                LocalVar->VS_State = VS_State_Region_3_ConstPwr; HIT(8);  // Constant power tracking
            } else {
                LocalVar->VS_State = VS_State_Region_3_ConstTrq; HIT(9);  // Constant torque tracking
            }
        } else {
            // IF (LocalVar%GenArTq >= CntrPar%VS_MaxOMTq*1.01) THEN
            // `1.01` is a default REAL literal under `-fdefault-real-8`, i.e.
            // REAL(8), so it is the same value C++ gives `1.01`. The product is
            // formed before the comparison in both languages.
            if (LocalVar->GenArTq >= CntrPar->VS_MaxOMTq * 1.01) {
                HIT(10);
                // IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
                if (CntrPar->VS_FBP == VS_FBP_Variable_Pitch) {
                    LocalVar->VS_State = VS_State_Region_2_5; HIT(11);
                } else {
                    LocalVar->VS_State = VS_State_Region_3_FBP; HIT(12);
                }
            // ELSEIF ((LocalVar%GenSpeedF < CntrPar%VS_RefSpd) .AND. &
            //         (LocalVar%GenBrTq >= CntrPar%VS_MinOMTq)) THEN
            // `.AND.` is written as `&&`, which short-circuits where Fortran
            // need not evaluate both operands either. Neither operand has a
            // side effect, so the two are indistinguishable.
            } else if ((LocalVar->GenSpeedF < CntrPar->VS_RefSpd) &&
                       (LocalVar->GenBrTq >= CntrPar->VS_MinOMTq)) {
                LocalVar->VS_State = VS_State_Region_2; HIT(13);
            // ELSEIF (LocalVar%GenBrTq < CntrPar%VS_MinOMTq) THEN ! Region 1 1/2
            // Note this is NOT the negation of the second operand above: the
            // pair is `>=` there and `<` here on the same two quantities, so
            // the two ELSEIFs overlap only through the `GenSpeedF` term. A
            // translation that folded them would change which branch a case
            // with `GenSpeedF >= VS_RefSpd` and `GenBrTq >= VS_MinOMTq` takes
            // -- that case is the ELSE, the error state.
            } else if (LocalVar->GenBrTq < CntrPar->VS_MinOMTq) {
                LocalVar->VS_State = VS_State_Region_1_5; HIT(14);
            } else {  // Error state, Debug
                LocalVar->VS_State = VS_State_Error; HIT(15);
            }
        }
    }
}
