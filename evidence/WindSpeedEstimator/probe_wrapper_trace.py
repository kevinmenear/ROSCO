from pathlib import Path
P = Path("/workspace/ROSCO-r2/rosco/controller/src/ControllerBlocks.f90")
s = P.read_text()
start = s.index("    SUBROUTINE WindSpeedEstimator(")
end = s.index("    END SUBROUTINE WindSpeedEstimator", start)
body = s[start:end]
probe = ("        vit_probe_n = vit_probe_n + 1\n"
         "        WRITE(97,'(I8,9(1X,Z16))') vit_probe_n, TRANSFER(LocalVar%WE_Vw,vit_probe_b), &\n"
         "            TRANSFER(LocalVar%HorWindV_F,vit_probe_b), TRANSFER(LocalVar%WE%v_m,vit_probe_b), &\n"
         "            TRANSFER(LocalVar%WE%v_t,vit_probe_b), TRANSFER(LocalVar%WE%v_h,vit_probe_b), &\n"
         "            TRANSFER(LocalVar%WE%om_r,vit_probe_b), TRANSFER(LocalVar%WE%P(1,1),vit_probe_b), &\n"
         "            TRANSFER(LocalVar%WE%K(1,1),vit_probe_b), TRANSFER(REAL(LocalVar%WE_Op,8),vit_probe_b)\n")
decls = "        INTEGER, SAVE :: vit_probe_n = 0\n        INTEGER(8) :: vit_probe_b\n"
mark = "        IMPLICIT NONE\n"
assert mark in body
body = body.replace(mark, mark + decls, 1)
body = body + probe
P.write_text(s[:start] + body + s[end:])
print("probe installed inside WindSpeedEstimator only")
