load('Datas.mat');


V_L = cumsum(F_LPS * (1/Fs));


%Volume do ambu x Posi��o P cnst
plot(Pos_mm,V_L);