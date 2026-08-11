// VIT Translation
// Function: ExtController
// Source: ExtControl.f90
// Module: ExtControl
// Fortran: SUBROUTINE ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar)
// Reference built with: -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
//
// ExtController hands the Bladed-style swap array to an external dynamic
// library and copies the answer back. Three things about it are not visible
// from the Fortran and were MEASURED before this file was written; each is
// marked at its use site with the artifact that measured it.

#include "vit_types.h"

#include <cstdio>
#include <cstring>
#include <vector>

// THE PRODUCTION BRIDGE. `rosco/controller/src/vit_extcontroller_dll.f90`.
//
// `DLL_Ext` is a `TYPE(ExtDLL_Type), SAVE` local: a C pointer, an array of
// three C function pointers and a CHARACTER(1024) array, living across calls.
// It has no C representation worth mirroring and it is ambient state rather
// than a value the signature carries, so it stays on the Fortran side and
// `LoadDynamicLib` is CALLED through this bridge rather than reproduced here
// (X1). `plan.json` declares that bridge for this unit.
//
// Declared here rather than included: VIT injects declarations for units IT
// generated, and this bridge is hand-written.
extern "C" {
void vit_extcontroller_loaddll_c(const char* fileName, int fileNameLen,
                                 const char* procName, int procNameLen,
                                 int* errStat, char* errMsg, int errMsgLen);
void vit_extcontroller_procaddr_c(void** addr);
}

namespace {

// LEN_TRIM: the position of the last non-BLANK character.
//
// Blanks only. Fortran's LEN_TRIM does not stop at a NUL, and a translation
// that used `strlen` would agree with it on every string a C caller builds and
// disagree on every string a Fortran caller does -- which is the direction the
// values actually come from here.
int len_trim(const char* s, int n) {
    while (n > 0 && s[n - 1] == ' ') --n;
    return n;
}

// The Bladed legacy DISCON entry point, from ExtControl.f90's own
// ABSTRACT INTERFACE `BladedDLL_Legacy_Procedure`, which is BIND(C).
// `REAL(ReKi)` is `C_FLOAT`, so `avrSWAP` is `float*` and NOT `double*`:
// an explicit kind is not promoted by -fdefault-real-8.
using BladedDLL_Legacy_Procedure = void (*)(float* avrSWAP, int* aviFAIL,
                                            const char* accINFILE,
                                            char* avcOUTNAME, char* avcMSG);

}  // namespace

void ExtController(float* avrSWAP, controlparameters_view_t* CntrPar,
                   localvariables_view_t* LocalVar,
                   extcontroltype_view_t* ExtDLL,
                   errorvariables_view_t* ErrVar) {
    // CHARACTER(100), PARAMETER :: ExtRootName = 'external_control'
    // CHARACTER(*),   PARAMETER :: RoutineName = 'ExtController'
    // Each length is taken from its own literal, once: naming a size twice is
    // how a restatement becomes an unobservable mutation site.
    static const char ExtRootName[] = "external_control";
    static const char RoutineName[] = "ExtController";
    const int len_ExtRootName = (int)sizeof(ExtRootName) - 1;   // LEN_TRIM = 16
    const int len_RoutineName = (int)sizeof(RoutineName) - 1;   // LEN      = 13

    const int max_avr_entries = 2000;

    const int len_DLL_InFile = len_trim(CntrPar->DLL_InFile,
                                        (int)sizeof(CntrPar->DLL_InFile));
    // LEN(ErrVar%ErrMsg). The view carries it because the length of a
    // deferred-length CHARACTER is part of its allocation.
    const int len_ErrMsg = (int)ErrVar->n_ErrMsg;

    // CHARACTER(KIND=C_CHAR) :: accINFILE(LEN_TRIM(CntrPar%DLL_InFile)+1)
    // CHARACTER(KIND=C_CHAR) :: avcOUTNAME(LEN_TRIM(ExtRootName)+1)
    // CHARACTER(KIND=C_CHAR) :: avcMSG(LEN(ErrVar%ErrMsg)+1)
    //
    // Each size is NAMED ONCE, because the Fortran writes the same expression
    // twice: `accINFILE`'s bound is `LEN_TRIM(DLL_InFile)+1` and so is record
    // 50, and `avcOUTNAME`'s bound is `LEN_TRIM(ExtRootName)+1` and so is
    // record 51. Transcribing both occurrences gives two sites computing one
    // quantity, and the buffer-size one is unobservable -- nothing reads a
    // std::vector's size back. That is the shape unit #1 and unit #4 both hit;
    // an unobservable site is not an equivalent mutant, it is a restatement,
    // and the fix is to remove it.
    //
    // Record 49 is NOT one of these. It is `LEN(avcMSG)+1`, which is 2, while
    // avcMSG's bound is `LEN(ErrVar%ErrMsg)+1` -- see the record-49 comment.
    const int n_accINFILE = len_DLL_InFile + 1;
    const int n_avcOUTNAME = len_ExtRootName + 1;
    std::vector<char> accINFILE((size_t)n_accINFILE, '\0');
    std::vector<char> avcOUTNAME((size_t)n_avcOUTNAME, '\0');
    std::vector<char> avcMSG((size_t)len_ErrMsg + 1, '\0');

    int aviFAIL = 0;

    // avcMSG     = TRANSFER( C_NULL_CHAR,                          avcMSG     )
    // avcOUTNAME = TRANSFER( TRIM(ExtRootName)//C_NULL_CHAR,       avcOUTNAME )
    // accINFILE  = TRANSFER( TRIM(CntrPar%DLL_InFile)//C_NULL_CHAR, accINFILE )
    //
    // MEASURED, because the first of those is not what it looks like.
    // `TRANSFER(SOURCE, MOLD)` with an array MOLD returns an array just large
    // enough to hold SOURCE -- ONE element for one character -- and gfortran
    // assigns a nonconforming right-hand side by copying min(size) elements.
    // So `avcMSG = TRANSFER(C_NULL_CHAR, avcMSG)` writes byte 1 and leaves
    // bytes 2..N of an automatic array INDETERMINATE.
    // `evidence/ExtController/transfer_probe.f90` prints `0 90 90 ...`, the 'Z'
    // the buffer happened to hold beforehand. The other two buffers are exactly
    // LEN_TRIM+1 long, so their TRANSFERs fill them whole.
    //
    // The three buffers are ZERO-INITIALISED, which is both the NUL terminator
    // each TRANSFER appends and, for avcMSG, the whole statement. That is a
    // DEPARTURE from literal transcription, twice over, and here is the proof
    // for each:
    //
    //   the terminator   writing it again after the memcpy is a second site
    //                    setting a byte the initialiser already set. It cannot
    //                    be observed and it cannot be got wrong; the mutants on
    //                    it survived because there was nothing to survive.
    //   avcMSG's tail    nothing reads past byte 1. The external library is
    //                    handed a C string whose first byte is NUL, and the
    //                    oracle fixture reads exactly that byte and no more
    //                    (`fixtures/bladed_stub/discon_stub.c` says why -- a
    //                    stub that folded in indeterminate bytes would make the
    //                    two sides of the comparison disagree for a reason that
    //                    is about neither implementation). Reproducing
    //                    "indeterminate" is not something a C++ program can do
    //                    without undefined behaviour of its own.
    std::memcpy(avcOUTNAME.data(), ExtRootName, (size_t)len_ExtRootName);
    std::memcpy(accINFILE.data(), CntrPar->DLL_InFile, (size_t)len_DLL_InFile);

    if (LocalVar->iStatus == 0) {
        // DLL_Ext%FileName = TRIM(CntrPar%DLL_FileName)
        // DLL_Ext%ProcName = TRIM(CntrPar%DLL_ProcName)
        // CALL LoadDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
        //
        // All three are the bridge's, because all three touch the SAVE local.
        // ErrMsg is written by LoadDynamicLib through a `CHARACTER(*),
        // INTENT(OUT)` dummy, so its LENGTH does not change -- an assumed-length
        // dummy blank-pads or truncates rather than reallocating -- and the
        // bridge is told that length rather than a capacity.
        const int len_DLL_FileName = len_trim(CntrPar->DLL_FileName,
                                              (int)sizeof(CntrPar->DLL_FileName));
        const int len_DLL_ProcName = len_trim(CntrPar->DLL_ProcName,
                                              (int)sizeof(CntrPar->DLL_ProcName));
        vit_extcontroller_loaddll_c(CntrPar->DLL_FileName, len_DLL_FileName,
                                    CntrPar->DLL_ProcName, len_DLL_ProcName,
                                    &ErrVar->ErrStat, ErrVar->ErrMsg, len_ErrMsg);

        // ALLOCATE(ExtDLL%avrSWAP(max_avr_entries))
        //
        // NOT represented here, and the proof is one statement below: the very
        // next thing the original does is `ExtDLL%avrSWAP = avrSWAP(1:2000)`,
        // a whole-array assignment to an ALLOCATABLE, which REALLOCATES it to
        // 2000 elements on every path -- allocated or not, and whatever size it
        // had. So the extent this ALLOCATE establishes is established again
        // unconditionally, and writing it twice would add a mutation site
        // computing a quantity nothing downstream can read.
        //
        // What the ALLOCATE does that the assignment does not is ABORT when the
        // field is already allocated. That is not reproduced: it is a crash,
        // not a value, and it is unreachable in the shipped program (this
        // branch runs on iStatus == 0, the first call, where the field is
        // unallocated). `harness/ranges.toml` states the same restriction for
        // the generated cases, and says so.

        std::printf(" ROSCO is calling an external dynamic library for control input:\n");
        std::printf(" DLL_FileName:%.*s\n",
                    len_trim(CntrPar->DLL_FileName,
                             (int)sizeof(CntrPar->DLL_FileName)),
                    CntrPar->DLL_FileName);
        std::printf(" DLL_InFile:%.*s\n", len_DLL_InFile, CntrPar->DLL_InFile);
        std::printf(" DLL_ProcName:%.*s\n",
                    len_trim(CntrPar->DLL_ProcName,
                             (int)sizeof(CntrPar->DLL_ProcName)),
                    CntrPar->DLL_ProcName);
        std::printf(" Library loaded successfully\n");
    }

    // ExtDLL%avrSWAP = avrSWAP(1:max_avr_entries)
    //
    // The reallocating assignment. The extent it fixes is the one the caller
    // sees, and it is named HERE and nowhere else.
    ExtDLL->n_avrSWAP = (int32_t)max_avr_entries;
    for (int i = 1; i <= max_avr_entries; i++) {
        ExtDLL->avrSWAP[i - 1] = avrSWAP[i - 1];
    }

    // ExtDLL%avrSWAP(49) = LEN(avcMSG)  + 1
    //
    // MEASURED, and it is not what the comment beside it in ROSCO says.
    // `avcMSG` is an ARRAY of `CHARACTER(KIND=C_CHAR)`, whose ELEMENT length is
    // the default 1 -- `LEN` of a CHARACTER array is the element length, not
    // the array size. So record 49 is the CONSTANT 2, not the size of the
    // message buffer it is documented to be ("Maximum number of characters in
    // the MESSAGE argument"). `evidence/ExtController/len_probe.txt`:
    // SIZE(avcMSG) = 1, LEN(avcMSG) = 1, record 49 = 2, for a 4096-character
    // ErrMsg. That is a defect in upstream ROSCO; it is transcribed, not
    // corrected, because the original is the oracle (P7).
    const int len_avcMSG = 1;
    ExtDLL->avrSWAP[49 - 1] = (float)(len_avcMSG + 1);
    // ExtDLL%avrSWAP(50) = LEN_TRIM(CntrPar%DLL_InFile) + 1
    ExtDLL->avrSWAP[50 - 1] = (float)n_accINFILE;
    // ExtDLL%avrSWAP(51) = LEN_TRIM(ExtRootName) + 1
    ExtDLL->avrSWAP[51 - 1] = (float)n_avcOUTNAME;

    // CALL C_F_PROCPOINTER( DLL_Ext%ProcAddr(1), DLL_Legacy_Subroutine )
    // CALL DLL_Legacy_Subroutine( ExtDLL%avrSWAP, aviFAIL, accINFILE,
    //                             avcOUTNAME, avcMSG )
    void* proc = nullptr;
    vit_extcontroller_procaddr_c(&proc);
    BladedDLL_Legacy_Procedure DLL_Legacy_Subroutine =
        reinterpret_cast<BladedDLL_Legacy_Procedure>(proc);
    DLL_Legacy_Subroutine(ExtDLL->avrSWAP, &aviFAIL, accINFILE.data(),
                          avcOUTNAME.data(), avcMSG.data());

    // IF (ErrVar%aviFAIL < 0) THEN
    //     ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    //     print *, TRIM(ErrVar%ErrMsg)
    // ENDIF
    //
    // `ErrVar%aviFAIL`, NOT the local `aviFAIL` this routine zeroed and handed
    // to the library. Two different variables one letter apart; the guard is
    // on the FIELD, which is an input.
    if (ErrVar->aviFAIL < 0) {
        // A REALLOCATING assignment: the result is longer than what was there,
        // so the field's length changes and the C++ has to say so. The view
        // carries a capacity for exactly this, and going past it is REFUSED and
        // REPORTED rather than truncated -- a shortened error message is a
        // plausible wrong answer, the one kind a bit-for-bit comparison cannot
        // catch.
        const int lt = len_trim(ErrVar->ErrMsg, len_ErrMsg);
        const int len_new = len_RoutineName + 1 + lt;
        if (len_new <= (int)ErrVar->n_ErrMsg_cap) {
            // Built right to left, in place: the source overlaps the
            // destination, so memmove and not memcpy.
            std::memmove(ErrVar->ErrMsg + len_RoutineName + 1, ErrVar->ErrMsg,
                         (size_t)lt);
            std::memcpy(ErrVar->ErrMsg, RoutineName, (size_t)len_RoutineName);
            ErrVar->ErrMsg[len_RoutineName] = ':';
            ErrVar->n_ErrMsg = (int32_t)len_new;
            std::printf(" %.*s\n", len_trim(ErrVar->ErrMsg, len_new),
                        ErrVar->ErrMsg);
        } else {
            std::fprintf(stderr,
                         "VIT: ExtController: ErrVar%%ErrMsg needs %d bytes, "
                         "past the %d-byte buffer; left unchanged\n",
                         len_new, (int)ErrVar->n_ErrMsg_cap);
        }
    }
}
