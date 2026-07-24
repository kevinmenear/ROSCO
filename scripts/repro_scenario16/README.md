# Scenario 16 `2^-70` residual — reproduction procedure

Reproduces ROSCO's single tolerated mismatch (`bld_pitch[4444]` = `2^-70` in Fortran vs
`0.0` in C++, Scenario 16, Flp_Mode=3) from clean builds, captures the **actual** operands
at the store, and proves the true cause.

Full analysis: dev note `202607240344-scenario16-residual-reproduced-true-root-cause.md`.

## What it establishes

- The residual is the ordinary non-FMA IEEE result of `LastSignal + rate*DT` for the real
  operands — **fully reproducible**, no "compiler context" needed.
- The rate limiter compiles to **identical arithmetic** in gfortran and g++: given identical
  operands, both probes return `2^-70` (and both return `0` on the operands the earlier
  investigation used).
- The real cause is a **sub-float32 difference in the internal double `LastSignal` state**
  between the two builds, invisible in the float32 `avrSWAP` outputs, amplified at the
  rate-limiter zero crossing. Not a rate-limiter code-generation difference.

## Why the earlier "returns zero in both languages" result was misleading

The prior probe used `LastSignal = 1.126e-6` (wrong invocation) and `DT = 0.025` as a full
double. The true `LastSignal` is `7.5367e-6` and the true `DT` is **float32 0.025**
(`0.02500000037…`, as delivered through the float32 `avrSWAP` interface). With the wrong
operands the expression cancels to `0`; with the real ones it yields `2^-70` in both languages.

## Run

Prerequisites: the `vit-dev` container running (ARM64 Linux, gfortran/g++ 13.3.0), mounting
this workspace at `/workspace`. Then, from the ROSCO repo root:

```bash
bash scripts/repro_scenario16/run.sh
```

The script (a) extracts upstream Fortran `e8010f0` and integrated C++ `integrated` into
isolated scratch trees — **the tracked working tree is never modified**; (b) instruments both
rate limiters; (c) builds both with `-ffp-contract=off` and runs Scenario 16; (d) prints
`bld_pitch[4444]` and the captured operands for each build; (e) runs the cross-language probes.

It temporarily swaps `rosco/lib/libdiscon.so` to load each scratch build and always restores it.

## Files

| File | Role |
|---|---|
| `run.sh` | Host orchestrator (needs git; drives builds/runs via `docker exec vit-dev`). |
| `instrument_ratelimit_fortran.py` | Inserts an operand dump into `Functions.f90`'s `ratelimit`. |
| `instrument_ratelimit_cpp.py` | Inserts the same dump into `src/Functions/ratelimit.cpp`. |
| `probe.f90` / `probe.cpp` | Standalone replays of the rate-limiter final update from hex operand bits. |

Probe arguments are five hex doubles: `inputSignal LastSignal DT minRate maxRate`.
Example (true operands): `probe_f 0 3EDF9C7998447C18 3F999999A0000000 BFC65604189374BC 3FC65604189374BC` → `result=0x3B90000000000000`.

## Extending: find the upstream origin

The dump threshold (`|result| < 1e-9`) captures the zero-crossing neighborhood. To locate the
function whose double output first diverges sub-float32, widen the threshold and instrument the
calls feeding `LastSignal` at steps 4440–4443 in both builds, then diff the doubles.
