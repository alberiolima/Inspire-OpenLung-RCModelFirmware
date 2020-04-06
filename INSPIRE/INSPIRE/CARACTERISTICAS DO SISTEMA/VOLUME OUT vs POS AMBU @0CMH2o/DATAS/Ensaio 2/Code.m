%Trata dados

Fs = 100;
dt = 1/Fs;

subplot(3,1,1)
plot(F_LPM_100mms)
subplot(3,1,2)
plot(F_raw_100mms)
subplot(3,1,3)
plot(Pos_mm_100mms)

[b,a] = butter(4,10/(Fs/2));

Pos_mm_100mms = (filtfilt(b,a,Pos_mm_100mms));
Pos_mm_50mms = (filtfilt(b,a,Pos_mm_50mms));

%100mms
plot(F_LPM_100mms(230:360))
hold on
plot(F_LPM_100mms(536:666))
plot(F_LPM_100mms(843:973))

V1_L_100mms = cumsum((F_LPM_100mms(230:360)/60)*dt);
V2_L_100mms = cumsum((F_LPM_100mms(536:666)/60)*dt);
V3_L_100mms = cumsum((F_LPM_100mms(843:973)/60)*dt);

plot(V1_L_100mms)
hold on
plot(V2_L_100mms)
plot(V3_L_100mms)

V_L_100ms = (V1_L_100mms+V2_L_100mms+V3_L_100mms)/3;

P1_mm_100mms = 100-Pos_mm_100mms(339:469);
P2_mm_100mms = 100-Pos_mm_100mms(646:776);

plot(P1_mm_100mms)
hold on
plot(P2_mm_100mms)

P_mm_100mms = (P1_mm_100mms + P2_mm_100mms)/2;


%50mms
subplot(3,1,1)
plot(F_LPM_50mms)
subplot(3,1,2)
plot(F_raw_50mms)
subplot(3,1,3)
plot(Pos_mm_50mms)


V1_L_50mms = cumsum((F_LPM_50mms(332:332+150)/60)*dt);
V2_L_50mms = cumsum((F_LPM_50mms(755:755+150)/60)*dt);
plot(V1_L_50mms)
hold on
plot(V2_L_50mms)

V_L_50ms = (V1_L_50mms+V2_L_50mms)/2;

P1_mm_50mms = 100-Pos_mm_50mms(339:339+150);
P2_mm_50mms = 100-Pos_mm_50mms(646:646+150);

plot(P1_mm_50mms)
hold on
plot(P2_mm_50mms)

P_mm_50mms = (P1_mm_50mms + P2_mm_50mms)/2;


plot(P_mm_100mms,V_L_100ms);
hold on
plot(P_mm_50mms,V_L_50ms);

p = polyfit(P_mm_100mms,V_L_100ms,2)

load('Datas.mat');


V_L = cumsum(F_LPS * (1/Fs));


%Volume do ambu x Posição P cnst
plot(Pos_mm,V_L);


V_L = interp1(Pos_mm,V_L,P_mm_100mms);

plot(V_L)
hold on
plot(V_L_100ms)

V_final_L = (V_L + V_L_100ms)/2;

p = polyfit(P_mm_100mms,V_final_L,2)

plot(P_mm_100mms,V_final_L)
hold on
plot(P_mm_100mms,p(1) .* P_mm_100mms.^2 + p(2) .* P_mm_100mms + p(3));


