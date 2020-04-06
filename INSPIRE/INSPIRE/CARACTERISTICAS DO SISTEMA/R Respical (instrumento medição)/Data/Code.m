R = P_cmH2O./(F_LPM/60);

R1 = mean(R(293:326));
R2 = mean(R(712:737));

Rmean_cmH2O_LPS = (R1+R2)/2;