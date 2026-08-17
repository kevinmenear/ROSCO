// RED-TEST STUB, not a translation. `FlapControl` as a NO-OP.
//
// The whole unit does nothing: no `Flp_Mode` test, no initialisation, no PII
// loop, no Coleman pair, no copy to `avrSWAP`. Every output the harness
// compares must therefore come back as the caller supplied it, and every case
// in which the reference writes ANY of them has to fail.
//
// WHAT THE NUMBER THIS PRODUCES IS FOR: it is the count the green in
// harness/FlapControl.json is a statement about. A corpus this no-op PASSES is
// a corpus that never reaches the unit. The difference between the corpus size
// and the failure count is the number of cases in which the reference leaves
// every compared output alone -- and for this unit that set is NOT small and is
// known in advance: `IF (CntrPar%Flp_Mode > 0)` guards the entire body, so
// every case with `Flp_Mode <= 0` writes nothing and must PASS here. With
// `CntrPar_Flp_Mode = { values = [2, 0, 1, 3] }` stated in harness/ranges.toml,
// that is a whole flag value's worth of the corpus and then some.
//
// A second, smaller set also passes: `Flp_Mode` above 3 falls through all four
// arms, so the only writes are the three `avrSWAP` slots -- which do change,
// so those cases FAIL. And `iStatus /= 0` with `Flp_Mode == 1` runs only three
// self-assignments before the same three `avrSWAP` writes, so those fail too.
// The pass set really is the `Flp_Mode <= 0` set.
//
// THE FIVE CALLEE CALLS ARE DELIBERATE AND ARE NOT DEAD CODE. Unit #45 measured
// that `vit/test_validate.generate_callee_bridges` decides which bridges to
// emit by running `_CALLEE_CALL.finditer` over the .cpp TEXT, and the generated
// Makefile compiles and links `<stem>_callees.o` only when at least one bridge
// was emitted. On the CLEAN tree `scripts/harness.sh` step 2e keeps every
// bridge and drops the corresponding `<callee>.cpp.o` from LIBS, so a stub that
// mentions no callee changes the harness's own LINK -- and a red test that
// fails to BUILD is not a red test (unit #42). All five of this unit's callees
// -- ColemanTransform, ColemanTransformInverse, PIController, PIIController and
// saturate -- are themselves integrated, so all five are in that trade.
//
// The guard is unsatisfiable on this corpus and on any corpus: `objInst` is
// always a real object here, so `objInst == nullptr` is never true, and the
// calls never execute. Their only job is to be TEXT the bridge generator can
// see.
//
// DO NOT DELETE THEM AS DEAD CODE.

#include "vit_types.h"

void FlapControl(float* avrSWAP, controlparameters_view_t* CntrPar,
                 localvariables_view_t* LocalVar, objectinstances_t* objInst) {
    if (objInst == nullptr) {
        // Unreachable. See the header: this exists so that the five callee
        // bridges are emitted and the harness links.
        double axisTilt_1P, axisYaw_1P;
        colemantransform_c(LocalVar->rootMOOPF, LocalVar->Azimuth, 1,
                           &axisTilt_1P, &axisYaw_1P);
        LocalVar->Flp_Angle[0] = picontroller_c(
            axisTilt_1P, CntrPar->Flp_Kp, CntrPar->Flp_Ki,
            -CntrPar->Flp_MaxPit, CntrPar->Flp_MaxPit, LocalVar->DT, 0.0,
            &LocalVar->piP, LocalVar->restart ? 1 : 0, &objInst->instPI);
        LocalVar->Flp_Angle[1] = piicontroller_c(
            axisYaw_1P, 0.0, CntrPar->Flp_Kp, CntrPar->Flp_Ki, 0.05,
            -CntrPar->Flp_MaxPit, CntrPar->Flp_MaxPit, LocalVar->DT, 0.0,
            &LocalVar->piP, LocalVar->restart ? 1 : 0, &objInst->instPI);
        LocalVar->Flp_Angle[2] = saturate_c(LocalVar->Flp_Angle[1],
                                            -CntrPar->Flp_MaxPit,
                                            CntrPar->Flp_MaxPit);
        colemantransforminverse_c(axisTilt_1P, axisYaw_1P, LocalVar->Azimuth, 1,
                                  0.0, LocalVar->Flp_Angle);
    }
    (void)avrSWAP;
    (void)CntrPar;
    (void)LocalVar;
    (void)objInst;
}
