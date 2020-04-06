
#include <avr/wdt.h>

void setup() 
{
  wdt_enable(WDTO_2S);
  SerialSetup();
  MotorSetup();
  BreathSetup();
}

void loop() 
{
  SerialLoop();
  BreathLoop();
  MotorLoop();
  wdt_reset();
}
