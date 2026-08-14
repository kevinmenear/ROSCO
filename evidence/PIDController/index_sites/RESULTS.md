# WHICH `[i]` SITES THE TWO SURVIVING `index_offset` MUTANTS ARE

`mutation/PIDController.json` names them by (operator, before, after) only
-- `'[i]' -> '[i + 1]'`, twelve times over -- so the artifact cannot say
which. Built each candidate and ran it (unit #32's rule) rather than
reasoning about it: one variant per occurrence, against the same 4610-case
corpus the green was taken on.

The counts reproduce the sweep's own per-mutant counts IN ORDER
(2303, 2303, 2303, 2307, 8, 2307, 46, SURVIVED, SURVIVED, 2307, 2307, 2307),
which is what says these variants ARE those mutants rather than merely the
same shape.

| site | line:col | failing | statement |
|---|---|---|---|
| site01 | 76:18 | 2303 of 4610 | `piP->ITerm[i] = I0;` |
| site02 | 77:22 | 2303 of 4610 | `piP->ITermLast[i] = I0;` |
| site03 | 78:18 | 2303 of 4610 | `piP->ELast[i] = 0.0;` |
| site04 | 91:18 | 2307 of 4610 | `piP->ITerm[i] = piP->ITerm[i] + DT * ki * error;` |
| site05 | 91:34 | 8 of 4610 | `piP->ITerm[i] = piP->ITerm[i] + DT * ki * error;` |
| site06 | 100:18 | 2307 of 4610 | `piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);` |
| site07 | 100:45 | 46 of 4610 | `piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);` |
| site08 **SURVIVOR** | 112:53 | 0 of 4610 | `const double DTerm = kd * (EFilt - piP->ELast[i]) / DT;` |
| site09 **SURVIVOR** | 121:60 | 0 of 4610 | `PIDController_result = saturate_c(PTerm + piP->ITerm[i] + DTerm,` |
| site10 | 132:22 | 2307 of 4610 | `piP->ITermLast[i] = piP->ITerm[i];` |
| site11 | 132:38 | 2307 of 4610 | `piP->ITermLast[i] = piP->ITerm[i];` |
| site12 | 133:18 | 2307 of 4610 | `piP->ELast[i] = EFilt;` |

Both survivors are READ sites whose only consumer is the return value:

```
site08   kd * (EFilt - piP->ELast[i]) / DT        -> DTerm
site09   saturate_c(PTerm + piP->ITerm[i] + DTerm, minValue, maxValue)
```

and `evidence/PIDController/probes/outer-clamp-inactive.json` reports the
outer clamp INACTIVE in **0 of 4610** cases. So neither site can move an
output in this corpus, and both survivors are the same one number as the
four arithmetic survivors -- six of seven survivors, one measurement.

The two SMALL non-zero counts are the same fact seen from inside:
site05 (the ITerm read that feeds the integrator update) kills 8, and
`probes/iterm-clamp-inactive.json` reports the ITerm clamp inactive in
exactly **8 of 4610**. site07 (the ITerm read the clamp itself takes)
kills 46, which is larger because a clamp reading the WRONG element can
differ wherever that element's value is on the other side of a bound.
