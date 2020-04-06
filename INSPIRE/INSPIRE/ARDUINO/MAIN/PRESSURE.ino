#define CmH20_0 207.0     //A0 val @0cmH2O
#define CmH2O_20  528.0   //A0 val @20cmH2O
#define PRESSURE_PIN  A0  

//Constants
const float deltaCmH2O = 20 / (CmH2O_20 - CmH20_0);

float PressureGet_cmH2O()
{
  float Pressure = (float) analogRead(PRESSURE_PIN ) - CmH20_0;
  return  (deltaCmH2O * Pressure);
}

int PressureGet_raw()
{
  return  analogRead(PRESSURE_PIN );
}

float PressureRaw2CmH2O(int RawPress)
{
  float Pressure = (float) RawPress - CmH20_0;
  return  (deltaCmH2O * Pressure);
}
