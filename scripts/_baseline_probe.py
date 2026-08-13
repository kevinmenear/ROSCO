#!/usr/bin/env python3
"""Find and check the ADMISSIBLE BASELINE a corpus rule needs, against the
translation, with no Fortran build and no reset window.

WHY THIS EXISTS. `CheckInputs` writes one pair of outputs -- `ErrVar%aviFAIL`
and `ErrVar%ErrMsg` -- from about 180 sites and never returns early, so only the
LAST site that fired is observable. Unit #29 measured that: a first-writer-wins
perturbation of the message sink fails 16,769 of 16,769 differential cases, so
every case in the corpus raises at least TWO errors and 165 of 173 mutants sit
above the last failing check where nothing can see them.

The repair is a corpus in which exactly one quantity is away from admissible,
and that needs a state on which the reference raises NOTHING -- the fixed point
of the unit's own checks. It cannot be derived; it has to be found. Finding it
by running the FORTRAN means the reset window, a full clean rebuild and a
generated harness for every attempt. Running the TRANSLATION instead costs one
`g++` per attempt, and the translation is not taken on faith: the baseline it
produces goes into the corpus as a case, and the two artifacts COMPOSE: the
probe says the C++ raises nothing on this state, the differential run says the
Fortran agrees with the C++ on it, so the Fortran raises nothing on it either.

Neither says that alone, and the harness is the one to be careful about: a case
on which BOTH sides raise the same error PASSES. A green corpus therefore cannot
see an inadmissible baseline by itself -- it is the probe's `aviFAIL 0` beside
it that makes the pair conclusive.

    python3 scripts/_baseline_probe.py CheckInputs \\
        --plan   /workspace/r2-planscratch/plan_dump.json \\
        --types  translations/ReadSetParameters/checkinputs_test/vit_types.h \\
        --cpp    translations/ReadSetParameters/checkinputs.cpp \\
        --baseline harness/baseline.CheckInputs.json \\
        --out /tmp/blprobe

`--all-messages` reports EVERY check that fired rather than the last one, by
inserting one `fprintf` at the message sink in a COPY of the translation. That
is the difference between one attempt per failing check and one attempt per
round: the unit's own masking applies to this probe exactly as it does to the
harness.

A parameter the probe cannot place in a view struct is a REFUSAL naming that
parameter, never a skip -- a baseline value that lands on no field is a value
that was never applied, and the state would read as admissible without being it.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# `<Prefix>_` on a harness parameter name -> the view struct it belongs to.
PREFIXES = {
    "CntrPar": "controlparameters_view_t",
    "LocalVar": "localvariables_view_t",
    "ErrVar": "errorvariables_view_t",
}


def parse_structs(header: str) -> dict[str, dict[str, str]]:
    """{struct name: {field: ctype}}, pointers keeping their `*`."""
    out: dict[str, dict[str, str]] = {}
    cur: dict[str, str] = {}
    for line in header.splitlines():
        s = line.strip()
        if s.startswith("typedef struct {"):
            cur = {}
            continue
        m = re.match(r"^\}\s*([A-Za-z_][A-Za-z0-9_]*)\s*;", s)
        if m:
            out[m.group(1)] = cur
            cur = {}
            continue
        s = re.sub(r"//.*$", "", s).strip()
        m = re.match(r"^([A-Za-z_][A-Za-z0-9_]*)\s*(\*?)\s*"
                     r"([A-Za-z_][A-Za-z0-9_]*)\s*(\[[0-9]+\])?\s*;$", s)
        if m and cur is not None:
            ctype, star, name, arr = m.groups()
            cur[name] = ctype + star + (arr or "")
    return out


def place(param: str, structs: dict[str, dict[str, str]]) -> tuple[str, str, str]:
    """(view, field, ctype) for one harness parameter name."""
    prefix, _, rest = param.partition("_")
    sname = PREFIXES.get(prefix)
    if sname is None:
        raise KeyError(param)
    fields = structs[sname]
    for cand in (rest,
                 f"n_{rest[:-2]}" if rest.endswith("_n") else None,
                 f"n_{rest}" if not rest.startswith("n_") else None):
        if cand and cand in fields:
            return prefix, cand, fields[cand]
    raise KeyError(param)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("unit")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--types", required=True)
    ap.add_argument("--cpp", required=True)
    ap.add_argument("--baseline", default=None)
    ap.add_argument("--out", default="/tmp/blprobe")
    ap.add_argument("--all-messages", action="store_true")
    ap.add_argument("--state", default=None,
                    help="which state of a multi-state baseline file")
    ap.add_argument("--link", action="append", default=[],
                    help="extra .cpp to compile in (a translated callee)")
    ap.add_argument("--container", default="vit-dev")
    ap.add_argument("--workdir", default="/workspace/ROSCO-r2")
    a = ap.parse_args()

    plan = json.loads(Path(a.plan).read_text())
    structs = parse_structs(Path(a.types).read_text())
    values: dict[str, object] = {}
    if a.baseline and Path(a.baseline).is_file():
        doc = json.loads(Path(a.baseline).read_text())
        states = doc.get("states") or [{"name": "baseline",
                                        "values": doc.get("values", doc)}]
        named = {s.get("name"): s for s in states}
        if a.state is None and len(states) > 1:
            print(f"{a.baseline} carries {len(states)} state(s) "
                  f"({', '.join(map(str, named))}); name one with --state",
                  file=sys.stderr)
            return 2
        st = named[a.state] if a.state else states[0]
        values = dict(st.get("values") or {})
        print(f"# state {st.get('name')}: {len(values)} value(s)")

    ext = plan["extents"]
    varied = {p["name"]: p for p in plan["varied"]}
    unknown = sorted(k for k in values if k not in varied)
    if unknown:
        print(f"baseline names {len(unknown)} parameter(s) that are not varied: "
              f"{', '.join(unknown)}", file=sys.stderr)
        return 2

    decl: list[str] = []
    body: list[str] = []
    for name, p in varied.items():
        if name in ("avrSWAP", "n_avrSWAP", "size_avcMSG"):
            continue
        view, field, ctype = place(name, structs)
        n = p["n"]
        if p["dims"]:                                   # an array body
            elem = "int" if p["kind"].startswith("int") else \
                   "char" if p["kind"] == "char[]" else "double"
            v = values.get(name)
            if v is None:
                v = [0] * n
            elif isinstance(v, dict):
                # The committed file's two array forms, applied exactly as
                # `harness/generate.py::_baseline_state` applies them: `fill`
                # over every element, then index overrides on top.
                src = values[name]
                if "ramp" in src:
                    start, step = src["ramp"]
                    v = [start + i * step for i in range(n)]
                else:
                    v = [src.get("fill", 0)] * n
                for k, x in src.items():
                    if k not in ("fill", "ramp"):
                        v[int(k)] = x
            if len(v) != n:
                print(f"{name}: baseline gives {len(v)} element(s), the extents "
                      f"say {n}", file=sys.stderr)
                return 2
            pad = 4096 if elem == "char" else 0
            cast = {"double": "(double)", "int": "(int)", "char": "(char)"}[elem]
            init = ", ".join(f"{cast}{float(x)!r}" if elem == "double"
                             else f"{cast}{int(x)}" for x in v)
            decl.append(f"static std::vector<{elem}> b_{name} = {{{init}}};")
            if pad:
                decl.append(f"// staging buffer, as the emitter sizes it")
                decl.append(f"static std::vector<{elem}> s_{name}"
                            f"(b_{name}.size() + {pad}, 0);")
                body.append(f"std::copy(b_{name}.begin(), b_{name}.end(), s_{name}.begin());")
                body.append(f"{view}.{field} = s_{name}.data();")
                body.append(f"{view}.n_{field}_cap = (int)s_{name}.size();")
            else:
                body.append(f"{view}.{field} = b_{name}.data();")
        else:
            v = values.get(name, 0)
            cast = "(int)" if "int" in ctype else "(double)"
            body.append(f"{view}.{field} = {cast}{float(v) if 'int' not in ctype else int(v)!r};")

    # avrSWAP is not in the loop above: it is a bare argument, not a view field.
    # It takes the same two array forms, and it MUST -- reading a `{index:
    # value}` map as a list here put 27.0 into element 0 and left element 28 at
    # zero, which the reference then reported as "Ptch_Cntrl in ServoDyn has a
    # value of 0" against a state that names exactly that element.
    n_avr = ext["n_avrSWAP"]
    v_avr = values.get("avrSWAP")
    if isinstance(v_avr, dict):
        if "ramp" in v_avr:
            start, step = v_avr["ramp"]
            body_avr = [float(start + i * step) for i in range(n_avr)]
        else:
            body_avr = [float(v_avr.get("fill", 0.0))] * n_avr
        for k, x in v_avr.items():
            if k not in ("fill", "ramp"):
                body_avr[int(k)] = float(x)
    else:
        body_avr = [float(x) for x in (v_avr or [0.0] * n_avr)]
    if len(body_avr) != n_avr:
        print(f"avrSWAP: baseline gives {len(body_avr)} element(s), the extents "
              f"say {n_avr}", file=sys.stderr)
        return 2
    decl.append(f"static std::vector<float> b_avrSWAP({n_avr}, 0.0f);")
    body.append("".join(f"b_avrSWAP[{i}] = {x!r}f; "
                        for i, x in enumerate(body_avr) if x != 0.0))

    cpp = Path(a.cpp).read_text()
    if a.all_messages:
        anchor = "void assign_errmsg(errorvariables_view_t* ErrVar, std::string_view s) {"
        if cpp.count(anchor) != 1:
            print(f"the message sink anchor occurs {cpp.count(anchor)} time(s), "
                  f"not once -- refusing to instrument by matching", file=sys.stderr)
            return 2
        cpp = cpp.replace(anchor, anchor + '\n    std::fprintf(stderr, "FIRED %.*s\\n", '
                                           '(int)s.size(), s.data());')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "unit.cpp").write_text(cpp)

    probe = f"""// GENERATED by scripts/_baseline_probe.py -- do not edit.
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>
#include "vit_types.h"
#if __has_include("{a.unit.lower()}_callees.h")
#include "{a.unit.lower()}_callees.h"   // the translation's callees, as the harness declares them
#endif
#include "unit.cpp"

{chr(10).join(decl)}

int main() {{
    controlparameters_view_t CntrPar = {{}};
    localvariables_view_t    LocalVar = {{}};
    errorvariables_view_t    ErrVar = {{}};
{chr(10).join("    " + b for b in body)}
    CheckInputs(&LocalVar, &CntrPar, b_avrSWAP.data(), &ErrVar,
                (int)ErrVar.size_avcMSG);
    std::printf("aviFAIL %d  ErrStat %d  n_ErrMsg %d  ErrMsg '%.*s'\\n",
                ErrVar.aviFAIL, ErrVar.ErrStat, (int)ErrVar.n_ErrMsg,
                (int)ErrVar.n_ErrMsg, ErrVar.ErrMsg ? ErrVar.ErrMsg : "");
    return ErrVar.aviFAIL == 0 ? 0 : 1;
}}
"""
    (out / "probe.cpp").write_text(probe)
    for h in ("vit_types.h", "checkinputs_callees.h"):
        src = Path(a.types).parent / h
        if src.is_file():
            (out / h).write_text(src.read_text())

    # The compile runs in the container, so the path has to be the one the bind
    # mount gives it. `/tmp` on the host is not `/tmp` in there, and a probe
    # written to a path the compiler cannot see fails as "no such file" three
    # steps away from the cause.
    host_mount = "/Users/kmenear/Artifacts/vit_translation"
    rel = str(out.resolve())
    if not rel.startswith(host_mount):
        print(f"--out {rel} is outside {host_mount}, which is what the container "
              f"mounts at /workspace -- the compile could not see it",
              file=sys.stderr)
        return 2
    rel = rel.replace(host_mount, "/workspace", 1)
    cmd = (f"cd {rel} && g++ -O0 -g -fPIC -ffp-contract=off -std=c++17 "
           f"-I. probe.cpp "
           + " ".join(f"{a.workdir}/{p}" for p in a.link)
           # -lgfortran: the CFI_* descriptor calls an ALLOCATABLE callee
           # argument crosses on live in libgfortran, not in the translation.
           + " -lgfortran -lm -o probe && ./probe")
    r = subprocess.run(["docker", "exec", a.container, "bash", "-lc", cmd],
                       capture_output=True, text=True)
    sys.stdout.write(r.stdout)
    sys.stderr.write(r.stderr)
    return r.returncode


if __name__ == "__main__":
    raise SystemExit(main())
