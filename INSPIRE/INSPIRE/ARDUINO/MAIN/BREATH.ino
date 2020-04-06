#include "TimerOne.h"
#include "debug.h"

//Inicial Breath parameters
#define R_cmH2O_LPS_0 50
#define C_L_cmH2O_0 0.06
#define Pplat_cmH2O_0 20
#define RespiRate_BPM_0 15
#define RespRatio_0 0.5
#define tPlat_s_0 1.0

//Breath Control coeffs
#define MaxR_cmH2O_LPS 50.0
#define MinR_cmH2O_LPS 1
#define MaxC_L_cmH2O 0.1
#define MinC_L_cmH2O 0.01

//Timer
#define Fs  15.0

//State machine states
#define STATE_START_CYCLE 0
#define STATE_INS_FLUX_START 1
#define STATE_INS_FLUX_END 2
#define STATE_EX_FLUX_START 3
#define STATE_EX_FLUX_END 4

#define STATE_GET_PVF0  0
#define STATE_GET_PVF1  1
#define STATE_GET_PVF2  2
#define STATE_GET_PVF_STOP  3

//PEPP
#define MEASURE_PEEP_CYCLES 5
#define PEEP_TIME2_MEASURE_s 4.0

//Control
#define CONTROL_PPLAT_COEFF 0.5
#define CONTROL_PPLAT_MAX_DP_cmH2O 2.5



//constants
const float dt = 1 / Fs;
const unsigned int Timer1Time = dt * 1e6;

//vars
float R_cmH2O_L_s = R_cmH2O_LPS_0;
float C_L_cmH2O = C_L_cmH2O_0;

float Pplat_cmH2O = Pplat_cmH2O_0;

float Pplat_correction_cmH2O = 0;

float Ppeep_cmH2O = 0;

float tPlat_s = round(tPlat_s_0 / dt) * dt;

unsigned char RespiRate_BPM = RespiRate_BPM_0;
float RespRatio = RespRatio_0;

float InsMaxTime_s = (60.0 / ( float) RespiRate_BPM_0) * RespRatio_0;
float CycleMaxTime_s = 60.0 / ( float) RespiRate_BPM_0;

float CycleTime = 0;
bool IsInsTimerOn = false;

float W1 = ((Pplat_cmH2O - Pplat_correction_cmH2O) / tPlat_s) * C_L_cmH2O;
float W2 = -1 / (R_cmH2O_L_s * C_L_cmH2O);
float W3 = W1 * (exp(-W2*tPlat_s) - 1);
float W4 = W1 / W2;
float W5 = W3 / W2;
float W6 = W1 * tPlat_s  + W4 * (1 - exp(W2*tPlat_s)) - W5 * exp(W2*tPlat_s);

float CyclePlatPress_cmH2O = 0;
unsigned char CyclePlatPress_samples = 0;

unsigned char StateMachine = STATE_START_CYCLE;

unsigned char Count2PeepMeasure = 0;
unsigned char PeepMeanSamples_samples = 0;

float P0_cmH2O = 0;
float V0_L = 0;
float F0_Ls = 0;
float P1_cmH2O = 0;
float V1_L = 0;
float F1_Ls = 0;
float P2_cmH2O = 0;
float V2_L = 0;
float F2_Ls = 0;
unsigned char State_Machine_GetPVF = STATE_GET_PVF0;

float VOut_L = 0;


void BreathSetup()
{
  Timer1.initialize(Timer1Time);
  Timer1.attachInterrupt(BreathTimer);
}

void BreathLoop()
{
  switch (StateMachine)
  {
    case STATE_START_CYCLE:
      BreathAdjustRC();
      CycleMaxTime_s = 60.0 / ( float) RespiRate_BPM;
      InsMaxTime_s = (60.0 / ( float) RespiRate_BPM) * RespRatio;
      CycleTime = 0;
      CyclePlatPress_cmH2O = 0;
      CyclePlatPress_samples = 0;
      State_Machine_GetPVF = STATE_GET_PVF0;
      W1 = ((Pplat_cmH2O - Pplat_correction_cmH2O) / tPlat_s) * C_L_cmH2O;
      W2 = -1 / (R_cmH2O_L_s * C_L_cmH2O);
      W3 = W1 * (exp(-W2 * tPlat_s) - 1);
      W4 = W1 / W2;
      W5 = W3 / W2;
      W6 = W1 * tPlat_s  + W4 * (1 - exp(W2 * tPlat_s)) - W5 * exp(W2 * tPlat_s);
      DBG_PRINTLN("NEW CYCLE START" );
      DBG_PRINT("Sensor Pressure cmH2O=" ); DBG_PRINTLN(PressureGet_cmH2O());
      DBG_PRINT("InsMaxTime=" ); DBG_PRINTLN(InsMaxTime_s);
      DBG_PRINT("CycleMaxTime_s=" ); DBG_PRINTLN(CycleMaxTime_s);
      DBG_PRINT("Pplat_cmH2O=" ); DBG_PRINTLN(Pplat_cmH2O);
      DBG_PRINT("Ppeep_cmH2O=" ); DBG_PRINTLN(Ppeep_cmH2O);
      DBG_PRINT("Pplat_correction_cmH2O=" ); DBG_PRINTLN(Pplat_correction_cmH2O);
      DBG_PRINT("tPlat_s=" ); DBG_PRINTLN(tPlat_s);
      DBG_PRINT("R_cmH2O_L_s=" ); DBG_PRINTLN(R_cmH2O_L_s);
      DBG_PRINT("C_L_cmH2O=" ); DBG_PRINTLN(C_L_cmH2O);
      DBG_PRINT("RespiRate_BPM=" ); DBG_PRINTLN(RespiRate_BPM);
      DBG_PRINT("W1=" ); DBG_PRINTLN(W1);
      DBG_PRINT("W2=" ); DBG_PRINTLN(W2);
      DBG_PRINT("W3=" ); DBG_PRINTLN(W3);
      DBG_PRINT("P0_cmH2O=" ); DBG_PRINTLN(P0_cmH2O);
      DBG_PRINT("F0_Ls=" ); DBG_PRINTLN(F0_Ls);
      DBG_PRINT("V0_L=" ); DBG_PRINTLN(V0_L);
      DBG_PRINT("P1_cmH2O=" ); DBG_PRINTLN(P1_cmH2O);
      DBG_PRINT("F1_Ls=" ); DBG_PRINTLN(F1_Ls);
      DBG_PRINT("V1_L=" ); DBG_PRINTLN(V1_L);
      DBG_PRINT("P2_cmH2O=" ); DBG_PRINTLN(P2_cmH2O);
      DBG_PRINT("F2_Ls=" ); DBG_PRINTLN(F2_Ls);
      DBG_PRINT("V2_L=" ); DBG_PRINTLN(V2_L);
      DBG_PRINT("dt=" ); DBG_PRINTLN(dt);
      StateMachine = STATE_INS_FLUX_START;
      break;
    case STATE_INS_FLUX_START:
      DBG_PRINTLN("STATE_INS_FLUX_START");
      MotorGo2MaxPos_SpeedMode(0);
      IsInsTimerOn = true;
      StateMachine = STATE_INS_FLUX_END;
      break;

    case STATE_INS_FLUX_END:
      if (IsInsTimerOn == false)
      {
        DBG_PRINTLN("STATE_INS_FLUX_END");
        StateMachine = STATE_EX_FLUX_START;
      }
      break;

    case STATE_EX_FLUX_START:
      DBG_PRINTLN("STATE_EX_FLUX_START");
      MotorGoExp_AccelMode();
      StateMachine = STATE_EX_FLUX_END;
      Ppeep_cmH2O = 0;
      PeepMeanSamples_samples = 0;
      break;

    case STATE_EX_FLUX_END:
      if (MotorIsOnStartPosition() && BreathIsCycleTimeEnd())
      {
        DBG_PRINTLN("STATE_EX_FLUX_END");
        //BreathAdjustPplat();
        StateMachine = STATE_START_CYCLE;
        DBG_PRINTLN("...");
        Ppeep_cmH2O = Ppeep_cmH2O / (float)(PeepMeanSamples_samples);
      }
      break;

    default:
      StateMachine = STATE_START_CYCLE;
      break;
  }
}

void BreathTimer()
{
  CycleTime = CycleTime + dt;
  DBG_PRINT(millis());DBG_PRINT(",");DBG_PRINTLN(VOut_L);
  //Pplat adjust
  if (CycleTime >= 0.9 * CycleMaxTime_s)
  {
    Ppeep_cmH2O = Ppeep_cmH2O + PressureGet_cmH2O();
    PeepMeanSamples_samples++;
  }
  if (IsInsTimerOn)
  {
    //Get PVF to adjust R and C
    switch (State_Machine_GetPVF)
    {
      case STATE_GET_PVF0:
        if (CycleTime > 0.25*tPlat_s)
        {
          P0_cmH2O = PressureGet_cmH2O();
          V0_L = VOut_L;
          F0_Ls = BreathInsCalcFlux_L_s(CycleTime - dt);
          State_Machine_GetPVF = STATE_GET_PVF1;
        }
        break;
      case STATE_GET_PVF1:
        if (CycleTime > 0.5*tPlat_s)
        {
          P1_cmH2O = PressureGet_cmH2O();
          V1_L = VOut_L;
          F1_Ls = BreathInsCalcFlux_L_s(CycleTime - dt);
          State_Machine_GetPVF = STATE_GET_PVF2;
        }
        break;
      case STATE_GET_PVF2:
        if (CycleTime > 0.75*tPlat_s)
        {
          P2_cmH2O = PressureGet_cmH2O();
          V2_L = VOut_L;
          F2_Ls = BreathInsCalcFlux_L_s(CycleTime - dt);
          State_Machine_GetPVF = STATE_GET_PVF_STOP;
        }
        break;
    }
    //Set Motor Speed on Inspiration
    float MotorNewPosition_mm ;
    float Pambu_cmH2O;
    if (CycleTime < tPlat_s)
    {
      Pambu_cmH2O = (CycleTime* ((Pplat_cmH2O - Ppeep_cmH2O) / tPlat_s) + Ppeep_cmH2O) 
      + (BreathInsCalcFlux_L_s(CycleTime) * AmbuGetRvalve_cmH2O_LPS());
      VOut_L = BreathInsOutVolume_L(CycleTime);
      MotorNewPosition_mm  = AmbuGetPosition2VolumeOut_mm(VOut_L, Pambu_cmH2O);
    }
    else
    {
      Pambu_cmH2O = Pplat_cmH2O + (BreathInsCalcFlux_L_s(CycleTime) * AmbuGetRvalve_cmH2O_LPS());
      VOut_L = BreathInsOutVolume_L(CycleTime);
      MotorNewPosition_mm  = AmbuGetPosition2VolumeOut_mm(VOut_L, Pambu_cmH2O);
    }
    MotorSet_InsNewPosition_mm(MotorNewPosition_mm, dt);
    if ( MotorIsOnMaxPosition() || (CycleTime >= InsMaxTime_s))
      IsInsTimerOn = false;
    if ( CycleTime > tPlat_s * 1.1)
    {
      CyclePlatPress_cmH2O = CyclePlatPress_cmH2O + PressureGet_cmH2O();
      CyclePlatPress_samples ++;
    }
  }
}

void BreathAdjustPplat()
{
  CyclePlatPress_cmH2O = CyclePlatPress_cmH2O / ((float) CyclePlatPress_samples);
  DBG_PRINT("CyclePlatPress_samples=" ); DBG_PRINTLN(CyclePlatPress_samples);
  DBG_PRINT("CyclePlatPressure_cmH2O=" ); DBG_PRINTLN(CyclePlatPress_cmH2O);
  float PplatError = CyclePlatPress_cmH2O - Pplat_cmH2O;
  float Adjust = PplatError * CONTROL_PPLAT_COEFF;
  if (abs(Adjust) > CONTROL_PPLAT_MAX_DP_cmH2O)
    Adjust = (Adjust / abs(Adjust)) * CONTROL_PPLAT_MAX_DP_cmH2O;
  DBG_PRINT("Adjust Pplat=");
  DBG_PRINTLN(Adjust);
  Pplat_correction_cmH2O = Pplat_correction_cmH2O + Adjust;
}

void BreathAdjustRC()
{
  float dV0 = V1_L - V0_L;
  float dV1 = V2_L - V1_L;
  float dP0 = P1_cmH2O - P0_cmH2O;
  float dP1 = P2_cmH2O - P1_cmH2O;
  float dF0 = F1_Ls - F0_Ls;
  float dF1 = F2_Ls - F1_Ls;
  DBG_PRINT("dV0  = "); DBG_PRINTLN(dV0 );
  DBG_PRINT("dV1 = "); DBG_PRINTLN(dV1);
  DBG_PRINT("dP0  = "); DBG_PRINTLN(dP0 );
  DBG_PRINT("dP1 = "); DBG_PRINTLN(dP1);
  DBG_PRINT("dF0  = "); DBG_PRINTLN(dF0 );
  DBG_PRINT("dF1 = "); DBG_PRINTLN(dF1);
  float Rest_cmH2O_Ls = ((dV0 * dP1) - (dV1 * dP0)) / ((dV0 * dF1) - (dV1 * dF0));
  DBG_PRINT("Rest_cmH2O_Ls = "); DBG_PRINTLN(Rest_cmH2O_Ls);
  float Cest_L_cmH2O = dV0 / (dP0 - (Rest_cmH2O_Ls * dF0));
  DBG_PRINT("Cest_L_cmH2O = "); DBG_PRINTLN(Cest_L_cmH2O);
}

bool BreathIsCycleTimeEnd()
{
  return CycleTime >= CycleMaxTime_s;
}

float BreathInsCalcFlux_L_s(float t)
{
  float Flux_L_s;
  if (t < tPlat_s)
    Flux_L_s = W1 * (1 - exp(W2 * t));
  else
    Flux_L_s = W3 * exp(W2 * t);
  return Flux_L_s;
}

float BreathInsOutVolume_L(float t)
{
  float Volume_L_s;
  if (t < tPlat_s)
    Volume_L_s = W1 * t + W4 * (1 - exp(W2 * t));
  else
    Volume_L_s = W6 + W5 * exp(W2 * t);
  return Volume_L_s;
}


void BreathChangeR(float New_R_cmH2O_L_s)
{
  R_cmH2O_L_s = New_R_cmH2O_L_s;
}

void BreathChangeC(float New_C_L_cmH2O)
{
  C_L_cmH2O = New_C_L_cmH2O;
}

void BreathChangePplat(float New_Pplat_cmH2O)
{
  Pplat_cmH2O = New_Pplat_cmH2O;
}

void BreathChangePpeep(float New_Ppeep_cmH2O)
{
  Ppeep_cmH2O  = New_Ppeep_cmH2O;
}

void BreathChangeB(float NewRespiRate_BPM)
{
  RespiRate_BPM = (unsigned char) NewRespiRate_BPM;
}

void BreathChangeTPlat(float NewTPlat_s)
{
  tPlat_s = NewTPlat_s;
}
