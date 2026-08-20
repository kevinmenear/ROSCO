#!/usr/bin/env python3
"""Count, per SITE, how often the corpus evaluates it and how often it is TRUE.

WHY. A mutation survivor has three possible answers and only one of them is a
claim about the corpus: `unreachable`. That claim is cheaper to assert than the
other two, so it is checked harder -- P12 refuses a declaration without a reason
AND an evidence path, and fails the unit outright if the corpus then kills the
mutant. The campaign's own coverage file cannot carry this claim: it stores only
NON-ZERO hit counts, so `never ran` and `never instrumented` are the same empty
dict.

So the evidence is measured here, on the same 55-case corpus the sweep scores
against, by the same probe. Every predicate below is rewritten

    if (COND)   ->   if (VITCNT(k, COND))

which counts the EVALUATIONS and the TRUE outcomes separately, and a plain
statement site is counted with a `VITHIT(k)` immediately before it. A site with
0 evaluations is one no case reaches; a site with N evaluations and 0 or N
true outcomes is one the corpus holds constant. Those are different findings and
the difference is the point.

    python3 evidence/ReadControlParameterFileSub/site_census.py

Writes the table to stdout. Reads the tree; writes only to /tmp and to the
container. Run from the repository root.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import rcpfsmutate as R  # noqa: E402

CPP = ROOT / "translations/ReadSetParameters/readcontrolparameterfilesub.cpp"

PREAMBLE = r'''
// ---- SITE CENSUS INSTRUMENTATION (evidence/.../site_census.py) ------------
#include <cstdio>
namespace vitcensus {
static long ev[64], tr[64];
inline bool cnt(int k, bool c) { ++ev[k]; if (c) ++tr[k]; return c; }
inline void hit(int k) { ++ev[k]; ++tr[k]; }
struct Report { ~Report() {
    for (int k = 0; k < 64; ++k)
        if (ev[k]) std::fprintf(stderr, "CENSUS %d eval %ld true %ld\n", k, ev[k], tr[k]);
    std::fprintf(stderr, "CENSUS end\n");
} };
static Report report_at_exit;
}
#define VITCNT(k, c) (vitcensus::cnt((k), (c)))
#define VITHIT(k) (vitcensus::hit((k)))
// --------------------------------------------------------------------------
'''

# (key, FIND, REPLACE, what the site is). An explicit pair rather than a
# rule, because two of these sites are the SAME text twice in the file and the
# instrumentation has to land on both -- the replacements are applied in order,
# each to the first occurrence left.
SITES = [
    (15, "return std::string_view(s, static_cast<size_t>(n));",
     "return std::string_view(s, static_cast<size_t>((VITCNT(15, n == 0), n)));",
     "ftrim RETURNED AN EMPTY view -- the only input on which `n > 0` decides "
     "anything, and the one a `n >= 0` mutant would read s[-1] on"),
    (1, "n > 0 && s[n - 1] == ' '", "VITCNT(1, n > 0 && s[n - 1] == ' ')",
     "ftrim's trailing-blank loop; `true` is one more blank trimmed. `n > 0` "
     "decides only for an ALL-BLANK argument, which is `evals - true` per call"),
    (2, "static_cast<int>(src.size()) < width",
     "VITCNT(2, static_cast<int>(src.size()) < width)",
     "fassign's truncation choice: true means the source FITS"),
    (3, "    if (n > 0) std::memcpy",
     "    VITCNT(16, n == 1); VITCNT(17, n == width);\n    if (VITCNT(3, n > 0)) std::memcpy",
     "fassign's copy guard"),
    (16, "@@none@@", "@@none@@",
     "fassign called with a source of exactly ONE character -- the only input "
     "on which `n > 0` and `n > 1` differ"),
    (17, "@@none@@", "@@none@@",
     "fassign called with a source EXACTLY as long as the destination -- the "
     "only input on which `<` and `<=` differ at either of the two sites"),
    (4, "if (n < width) std::memset", "if (VITCNT(4, n < width)) std::memset",
     "fassign's blank fill: true means there is a tail to blank"),
    (5, "if (ErrVar->ErrMsg == nullptr) {",
     "if (VITCNT(5, ErrVar->ErrMsg == nullptr)) {",
     "assign_errmsg: the message buffer has never been allocated"),
    (6, "if (static_cast<int32_t>(s.size()) > ErrVar->n_ErrMsg_cap) {",
     "if (VITCNT(6, static_cast<int32_t>(s.size()) > ErrVar->n_ErrMsg_cap)) {",
     "assign_errmsg: the message does not fit, so the buffer is reallocated"),
    (7, "if (ErrVar->n_ErrMsg_cap > static_cast<int32_t>(s.size())) {",
     "if (VITCNT(7, ErrVar->n_ErrMsg_cap > static_cast<int32_t>(s.size()))) {",
     "assign_errmsg: there is a tail past the message to clear"),
    (8, "if (ErrVar->ErrMsg == nullptr || n <= 0) return std::string();",
     "if (VITCNT(8, ErrVar->ErrMsg == nullptr || n <= 0)) return std::string();",
     "errmsg_view: nothing to read back"),
    (9, "if (base != nullptr) {", "if (VITCNT(9, base != nullptr)) {",
     "establish(): the array field is ALREADY allocated on entry -- the arm "
     "whose callee message is `array was already allocated`"),
    (10, "if (!cur.empty() && cur.back() == '\\r') cur.pop_back();",
     "if (VITCNT(10, !cur.empty() && cur.back() == '\\r')) cur.pop_back();",
     "read_records: a CR stripped at a \\n boundary (FIRST site, inside the loop)"),
    (14, "if (!cur.empty() && cur.back() == '\\r') cur.pop_back();",
     "if (VITCNT(14, !cur.empty() && cur.back() == '\\r')) cur.pop_back();",
     "read_records: a CR stripped at the unterminated last record (SECOND site)"),
    (11, "if (any) {", "if (VITCNT(11, any)) {",
     "read_records: the file's last line has no trailing newline"),
    (12, "if (CntrPar->Echo > 0) {", "if (VITCNT(12, CntrPar->Echo > 0)) {",
     "the echo arm"),
    (13, "if (ec == nullptr) {", "if (VITCNT(13, ec == nullptr)) {",
     "the echo file could not be opened"),
]


def instrument(src: str) -> str:
    out = src.replace("namespace {", PREAMBLE + "\nnamespace {", 1)
    for k, find, repl, _why in SITES:
        if find == "@@none@@":      # counted by another site's replacement
            continue
        if find not in out:
            raise SystemExit(f"site {k}: no match for {find!r}")
        out = out.replace(find, repl, 1)
    return out


def corpus_metrics() -> None:
    """The corpus's own widths, for the survivors that sit on a TRUNCATION.

    `MaxLineLength`, `PriPath_LEN`, `EchoFilename_LEN` and `OL_String_LEN` are
    the reference's own CHARACTER widths, and a mutant that widens one by a
    byte differs only for an input that reaches the width. These are the
    numbers that say whether any input in the corpus does."""
    import glob
    recs, longest = 0, 0
    paths = sorted(glob.glob(str(ROOT / "Examples" / "DISCON*.IN"))) + \
        sorted(glob.glob(str(ROOT / "evidence" / "ReadControlParameterFileSub"
                             / "corpus" / "*.IN")))
    for f in paths:
        for line in Path(f).read_bytes().split(b"\n"):
            recs += 1
            longest = max(longest, len(line.rstrip(b"\r")))
    # The probe passes `DISCON*.IN` from Examples/ and `../evidence/.../*.IN`,
    # which is the string the unit sees as accINFILE(1) and as RootName.
    argv = [Path(f).name if "Examples" in f else
            "../evidence/ReadControlParameterFileSub/corpus/" + Path(f).name
            for f in paths]
    print(f"corpus widths: {len(paths)} file(s), {recs} record(s)")
    print(f"  longest record            {longest:>6}   vs MaxLineLength 2048")
    print(f"  longest input path        {max(len(a) for a in argv):>6}   "
          f"vs PriPath_LEN 1024 / EchoFilename_LEN 128")


def main() -> int:
    src = CPP.read_text()
    inst = instrument(src)
    rc, out = R.dexec(R.setup(""), timeout=300)
    if rc:
        print(out, file=sys.stderr)
        return 2
    R.dexec(f"cat > {R.MUTDIR}/unit.cpp", stdin=inst)
    rc, out = R.dexec(R.compile_cmd(""), timeout=300)
    if rc:
        print(out[-3000:], file=sys.stderr)
        return 2
    run = R.RUN.replace("$VIT_SAN_ENV", "")
    rc, out = R.dexec(run, timeout=180)
    counts = {}
    for line in out.splitlines():
        if line.startswith("CENSUS ") and " eval " in line:
            _, k, _, ev, _, tr = line.split()
            counts[int(k)] = (int(ev), int(tr))
    cases = sum(1 for l in out.splitlines()
                if l.startswith("  ok ") or l.startswith("  FAIL "))
    field = next((l for l in out.splitlines() if l.startswith("cases ")), "")
    print(f"corpus: {cases} case(s)   {field}")
    print(f"{'site':>4}  {'evals':>8}  {'true':>8}   what it is")
    for k, _f, _r, why in SITES:
        ev, tr = counts.get(k, (0, 0))
        print(f"{k:>4}  {ev:>8}  {tr:>8}   {why}")
    print()
    corpus_metrics()
    print("\nA site with 0 evaluations is one NO case reaches. A site with N "
          "evaluations and 0 or N true is one the corpus holds constant -- "
          "reached, but not deciding.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
