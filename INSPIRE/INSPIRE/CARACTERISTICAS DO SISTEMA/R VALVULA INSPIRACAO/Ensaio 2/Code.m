R1 = P1_cmH2O./(F1_LPM/60);

R1_cmH2O_LPS = mean(R1(562:742));

R2 = P2_cmH2O./(F2_LPM/60);

R2_cmH2O_LPS = mean(R2(227:403));

Rvalv_cmH2O_LPS = mean([R1_cmH2O_LPS R2_cmH2O_LPS]);