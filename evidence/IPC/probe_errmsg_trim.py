#!/usr/bin/env python3
"""COUNT what this unit's corpus can do to `errmsg_trim`, mutant by mutant.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        python3 evidence/IPC/probe_errmsg_trim.py"

WHY THIS EXISTS. Three of the eight open survivors of `mutation/IPC.json` are
`const_tweak` mutants of six lines of `translations/Controllers/ipc.cpp`:

    std::string errmsg_trim(const errorvariables_view_t* ErrVar) {
        const int n = ErrVar->n_ErrMsg;
        const std::string_view v(ErrVar->ErrMsg, n > 0 ? (size_t)n : 0);
        return std::string(v.substr(0, v.find_last_not_of(' ') + 1));
    }

    A  `n > 0`  -> `n > 1`   differs iff n == 1 at the call
    B  `: 0`    -> `: 1`     differs iff n <= 0 and byte 0 is not a blank
    C  `+ 1`    -> `+ 2`     differs iff the string has a TRAILING BLANK
                             (all-blank included: npos + 2 is 1, not 0)

`evidence/IPC/README.md` argued from the SHAPE of the corpus that no case
supplies any of the three -- "R6's character stage runs the string at lengths
[1, 2, 7, 12] and in the padded shape and holds `aviFAIL` at its base draw;
R7's knob puts -1 in `aviFAIL` and holds the string at ITS base; the two ladders
never cross". That is a claim about the CORPUS, and unit #50's rule is that a
claim about the corpus is executed by COUNTING rather than by arguing. This
counts it, and it counts the thing that actually decides each mutant rather
than the thing on the case: `n` at the call is not the case's `n_ErrMsg`,
because four `sigma` calls prefix `'sigma:'` onto the same buffer first.

THE CHAIN IS SIMULATED, AND EVERY TERM OF IT IS READ OFF THE GENERATOR RATHER
THAN GUESSED.

  * `harness/emit.py:1393-1409` sizes the C++ side's staging buffer at
    `max(n_ErrMsg, 0) + headroom` and takes `headroom` from the case when the
    case states one; `emit.py:1133-1134` substitutes `ALLOC_HEADROOM = 4096`
    when it does not. Only R13's capacity ladder states one
    (`harness/generate.py:2984`, `<param>_cap_headroom`), so EVERY OTHER RULE
    in this corpus runs at a capacity of `n + 4096`.
  * The reference side gets `n + 4096` unconditionally, so it never refuses.
  * `sigma` is reached through its bridge on the clean tree. Each call ends in
    `ErrVar%ErrMsg = 'sigma'//':'//TRIM(ErrVar%ErrMsg)` and the bridge's
    write-back refuses what does not fit the staging buffer, leaving the buffer
    and its length alone. Fortran `TRIM` is a right strip of blanks, so a
    successful prefix DELETES any trailing blank the entry message had.
  * `IPC`'s own tail then writes `'IPC:'//TRIM(...)` through `assign_errmsg`,
    which gates on the same capacity.

So the probe replays those five gates per case and asks each mutant's question
at the point `errmsg_trim` is actually called. `aviFAIL >= 0` on entry means the
function is never called at all, which is the first thing counted.

IT GENERATES BUT DOES NOT EMIT -- `evidence/IPC/probe_fmin_site.py`'s trick,
unchanged: monkeypatch the name `vit_harness` imported `generate` under, count
off `gen.cases`, exit before `emit` writes 870 MB. 30 seconds and no build.

IT CARRIES ITS OWN POSITIVE CONTROL (P10). A count of zero from a probe that
cannot produce anything else is not evidence. `controls` re-runs each mutant's
predicate with ONE term relaxed -- the term the claim rests on -- and reports a
non-zero count for each, so the zero above it is a property of the corpus and
not of the loop.

IT MUST RUN ON A CLEAN TREE, for `probe_fmin_site.py`'s reason: the generator
reads the Fortran source, and on an integrated tree that source is the wrapper.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path("/workspace/ROSCO-r2")
if not ROOT.is_dir():
    ROOT = Path(__file__).resolve().parents[2]
LOOP = Path("/workspace/translation-loop")
if not LOOP.is_dir():
    LOOP = ROOT.parent / "translation-loop"

sys.path.insert(0, str(LOOP / "scripts"))
sys.path.insert(0, str(LOOP))

import vit_harness  # noqa: E402

OUT = ROOT / "evidence" / "IPC" / "probe_errmsg_trim.json"

AVIFAIL = "ErrVar_aviFAIL"
MSG = "ErrVar_ErrMsg"
MSG_N = "ErrVar_ErrMsg_n"
HEADROOM = "ErrVar_ErrMsg_cap_headroom"
ALLOC_HEADROOM = 4096          # harness/emit.py:117
N_SIGMA = 4                    # the four sigma calls at ipc.cpp:341-348
BLANK = 32


def _bytes(v, n):
    """The supplied message, as the emitter writes it: `n` bytes, or none."""
    if n <= 0 or not isinstance(v, (list, tuple)):
        return b""
    return bytes(int(x) & 0xFF for x in v[:n])


def _chain(msg: bytes, n: int, cap: int, n_sigma: int = N_SIGMA):
    """(buffer, length) at the point `errmsg_trim` is called.

    `msg` is the buffer's first `n` bytes on entry. Each gate writes
    `'sigma:' + rstrip(current)` when it fits the capacity and does nothing when
    it does not -- the translation's `assign_errmsg` refuses rather than
    truncating, and the generated bridge's write-back does the same.
    """
    cur = msg
    ln = n
    for _ in range(n_sigma):
        cand = b"sigma:" + (cur[:max(ln, 0)].rstrip(b" ") if ln > 0 else b"")
        if len(cand) <= cap:
            cur, ln = cand, len(cand)
    return cur, ln


def _trim(buf: bytes, n: int, plus: int = 1, floor: int = 0, gate: int = 0):
    """`errmsg_trim` itself, with each of the three constants as a parameter.

    `gate` is the `n > 0` constant, `floor` the `: 0` one and `plus` the `+ 1`.
    The original is (gate=0, floor=0, plus=1); each mutant moves exactly one.
    """
    length = n if n > gate else floor
    if length < 0:
        length = 0
    v = buf[:length] if length <= len(buf) else buf + b"\0" * (length - len(buf))
    idx = -1
    for i in range(len(v) - 1, -1, -1):
        if v[i] != BLANK:
            idx = i
            break
    # `find_last_not_of` returns npos on an all-blank string and npos + k is
    # k - 1 by the defined wraparound of size_t; the C++ then clamps to size().
    take = (idx + plus) if idx >= 0 else (plus - 1)
    return v[:max(0, min(take, len(v)))]


def _tail(buf: bytes, n: int, cap: int, **kw):
    """The final buffer state after `IPC`'s own gated `'IPC:'//TRIM(...)`."""
    out = b"IPC:" + _trim(buf, n, **kw)
    if len(out) <= cap:
        return out, len(out)
    return buf[:max(n, 0)], n


def _stats(cases):
    st = {
        "cases": len(cases),
        "reaches_errmsg_trim": 0,
        "not_reached_avifail_nonneg": 0,
        "n_at_call_histogram": {},
        "capacity_histogram": {},
        "entry_length_histogram": {},
        "entry_has_trailing_blank": 0,
        "all_four_sigma_gates_refused": 0,
        "mutants": {
            "A_n_gt_0_to_n_gt_1": {"differs_at_trim": 0, "differs_in_output": 0,
                                   "needs": "n == 1 at the call", "witness": None},
            "B_floor_0_to_1": {"differs_at_trim": 0, "differs_in_output": 0,
                               "needs": "n <= 0 at the call and a non-blank byte 0",
                               "witness": None},
            "C_plus_1_to_2": {"differs_at_trim": 0, "differs_in_output": 0,
                              "needs": "a trailing blank at the call",
                              "witness": None},
        },
        # P10: each control relaxes the ONE term the zero above rests on.
        "controls": {
            "A_n_le_31_instead_of_eq_1": 0,
            # `n <= 0` RELAXED BY ONE, because the strict form is zero on both
            # sides of the chain and a control that also reads zero proves
            # nothing about the loop. `n_entry <= 1` is the nearest predicate
            # the corpus DOES satisfy: it names the 196 cases at the shortest
            # length R6's character stage supplies, and B needs one shorter
            # still -- an UNALLOCATED field, which is the negative extent this
            # corpus never states.
            "B_n_entry_le_1_reaching_the_call": 0,
            "B_n_le_0_before_the_chain": 0,
            "C_trailing_blank_ON_ENTRY_reaching_the_call": 0,
            "C_trailing_blank_if_the_chain_did_not_run": 0,
        },
    }
    for c in cases:
        avi = c.get(AVIFAIL)
        n0 = int(c.get(MSG_N) or 0)
        msg = _bytes(c.get(MSG), n0)
        hr = c.get(HEADROOM)
        cap = max(n0, 0) + (ALLOC_HEADROOM if hr is None else int(hr))

        st["entry_length_histogram"][str(n0)] = \
            st["entry_length_histogram"].get(str(n0), 0) + 1
        st["capacity_histogram"][str(cap)] = \
            st["capacity_histogram"].get(str(cap), 0) + 1
        entry_blank = n0 > 0 and msg[:n0].endswith(b" ")
        if entry_blank:
            st["entry_has_trailing_blank"] += 1

        if avi is None or int(avi) >= 0:
            st["not_reached_avifail_nonneg"] += 1
            # The C control still has something to say about a case the unit
            # never trims: whether the chain is what removes the blank.
            if entry_blank:
                st["controls"]["C_trailing_blank_if_the_chain_did_not_run"] += 1
            continue

        st["reaches_errmsg_trim"] += 1
        buf, n = _chain(msg, n0, cap)
        if n == n0 and buf[:max(n0, 0)] == msg:
            st["all_four_sigma_gates_refused"] += 1
        st["n_at_call_histogram"][str(n)] = \
            st["n_at_call_histogram"].get(str(n), 0) + 1

        base_trim = _trim(buf, n)
        base_out = _tail(buf, n, cap)
        for key, kw in (("A_n_gt_0_to_n_gt_1", dict(gate=1)),
                        ("B_floor_0_to_1", dict(floor=1)),
                        ("C_plus_1_to_2", dict(plus=2))):
            m = st["mutants"][key]
            if _trim(buf, n, **kw) != base_trim:
                m["differs_at_trim"] += 1
            if _tail(buf, n, cap, **kw) != base_out:
                m["differs_in_output"] += 1
                if m["witness"] is None:
                    m["witness"] = {"n_entry": n0, "cap": cap, "n_at_call": n,
                                    "buffer": buf[:max(n, 0)].decode("latin1")}

        if n <= 31:
            st["controls"]["A_n_le_31_instead_of_eq_1"] += 1
        if n0 <= 1:
            st["controls"]["B_n_entry_le_1_reaching_the_call"] += 1
        if n0 <= 0:
            st["controls"]["B_n_le_0_before_the_chain"] += 1
        if entry_blank:
            st["controls"]["C_trailing_blank_ON_ENTRY_reaching_the_call"] += 1
    return st


_real = vit_harness.generate


def _wrapped(signature, *a, **kw):
    gen = _real(signature, *a, **kw)
    st = _stats(gen.cases)
    doc = {
        "unit": "IPC",
        "site": "translations/Controllers/ipc.cpp:130-134 -- errmsg_trim, "
                "reached only from the `aviFAIL < 0` tail at :607",
        "loop_rev": vit_harness.loop_rev(),
        "gen_rev": vit_harness.gen_rev(),
        "model": {
            "sigma_calls": N_SIGMA,
            "alloc_headroom_default": ALLOC_HEADROOM,
            "capacity": "max(ErrVar_ErrMsg_n, 0) + headroom, headroom from the "
                        "case when R13's ladder states one and 4096 otherwise "
                        "(harness/emit.py:1133-1134, 1393-1409)",
        },
        **st,
    }
    OUT.write_text(json.dumps(doc, indent=1) + "\n")
    print(json.dumps(doc, indent=1))
    print(f"\nwrote {OUT}", file=sys.stderr)
    raise SystemExit(0)


vit_harness.generate = _wrapped

sys.argv = ["vit_harness.py", "IPC",
            "--root", str(ROOT),
            "--file", "rosco/controller/src/Controllers.f90",
            "--cpp", "translations/Controllers/ipc.cpp",
            "--module", "Controllers"]
raise SystemExit(vit_harness.main())
