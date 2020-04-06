//Ambu parameters
//Volume(Position) = VPos_A2 * Position^2 + VPos_A1 * Position + VPos_A0
//Volume L
//Position mm
#define VPos_A2 7.683171170513934e-05
#define VPos_A1 0.003627158900468
#define VPos_A0 0.013912662412795
//Position(Volume) = PosV_A3 * Volume^3 + PosV_A2 * Volume^2+  Volume * Position + PosV
//Volume L
//Position mm
#define PosV_A3 56.294006995575636
#define PosV_A2 -1.411793743886704e+02
#define PosV_A1 1.763924015751891e+02
#define PosV_A0 0.165070048003216
//Compliance
#define Ca_L_cmH2O 3.1502e-3

#define AmbuV_L 1.8
#define P_amb_cmH2O 944.89

#define Rvalve_cmH2O_LPS  5.000483442373419


float AmbuGetPosition2VolumeOut_mm(float VolumeOut_L,float PressureOnVolumeOut_cmH2O)
{
  float TotalVolume_L = VolumeOut_L ;//+ PressureOnVolumeOut_cmH2O * Ca_L_cmH2O;
  float Postion_mm = PosV_A3 * pow(TotalVolume_L,3) + PosV_A2 * pow(TotalVolume_L,2) +  PosV_A1 * TotalVolume_L + PosV_A0;
  return Postion_mm;
}


float AmbuGetRvalve_cmH2O_LPS()
{
  return Rvalve_cmH2O_LPS;
}
