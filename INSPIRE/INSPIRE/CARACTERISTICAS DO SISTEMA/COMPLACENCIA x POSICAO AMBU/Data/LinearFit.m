Line1 = 1:3;
Line2 = 3:5;

%Line 1
p = polyfit(Pos_mm(Line1), C_L_cmH2O(Line1),1);

%0 to 50mm
C = 2.1e-5*Pos+0.0027;

%Line 2
p = polyfit(Pos_mm(Line2), C_L_cmH2O(Line2),1);

C = -2.06e-5*Pos+0.048;