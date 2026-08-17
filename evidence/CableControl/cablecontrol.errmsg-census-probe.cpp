// THE ERRMSG CENSUS PROBE, unit #49 `CableControl`.
//
// Inserted into the GENERATED test source immediately before the call, so it
// reads the case's INPUTS and nothing the two chains compute. Re-applying it:
//
//     <paste the block below just above `CableControl(avrSWAP_a.data(), ...);`
//      in translations/Controllers/cablecontrol_test/cablecontrol_test.cpp>
//     bash scripts/harness.sh CableControl Controllers cablecontrol \
//         rosco/controller/src/Controllers.f90 --no-generate
//     docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/translations/Controllers/cablecontrol_test \
//         && make test && ./test cablecontrol_cases.bin 2> /tmp/census.err"
//
// `--no-generate` keeps `cablecontrol_cases.bin` byte for byte, so the numbers
// are about the SCORED corpus and not about a re-derived one (unit #47).
// STRIP IT AFTERWARDS -- a later --no-generate keeps the test source too, and a
// mutation sweep would print 3354 lines per mutant.
//
// `need = 18 + trim` is the length this unit's own assignment asks for:
// len('StructuralControl') + len(':') + LEN(TRIM(ErrMsg)).

        {   // ERRMSG CENSUS PROBE -- inputs only, before either side runs.
            int n = ErrVar_a.n_ErrMsg;
            int len = n > 0 ? n : 0;
            int t = len; while (t > 0 && ErrVar_a.ErrMsg[t-1] == ' ') t--;
            std::fprintf(stderr, "CENSUS c=%d aviFAIL=%d n=%d cap=%d trim=%d need=%d ccmode=%d ccn=%d\n",
                c, ErrVar_a.aviFAIL, n, ErrVar_a.n_ErrMsg_cap, t, 18 + t,
                CntrPar_a.CC_Mode, CntrPar_a.CC_Group_N);
        }
