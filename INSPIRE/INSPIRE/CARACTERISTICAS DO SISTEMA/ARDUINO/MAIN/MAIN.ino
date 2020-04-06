#include "AccelStepper.h"
#include <avr/wdt.h>

//Pins
#define STP_PIN 4
#define DIR_PIN 3
#define ENA_PIN 2
#define HOM_PIN 12

//Max
#define MAX_ACCEL_mms2 500.0
#define MAX_SPEED_mms  75.0
#define MAX_POS_mm 100.0

//Stepper Motor parameters
#define STEPS_by_REV  400.0
#define mm_by_REV 25.0

//Home Parameters
#define HOME_SPEED_mms  10.0
#define HOME_INCREMENT_mm  1.0
#define HOME_PRESS_POSITION_mm  2.0

//Consts
const unsigned int MAX_ACCEL_STEPS2 = (MAX_ACCEL_mms2 *  STEPS_by_REV) / mm_by_REV;
unsigned int MAX_SPEED_STEPS = (MAX_SPEED_mms *  STEPS_by_REV) / mm_by_REV;
const unsigned int HOME_SPEED_STEPS = (HOME_SPEED_mms *  STEPS_by_REV) / mm_by_REV;
const unsigned int HOME_INCREMENT_STEP = (HOME_INCREMENT_mm  *  STEPS_by_REV) / mm_by_REV;
const unsigned int HOME_PRESS_POSITION_STEP = (HOME_PRESS_POSITION_mm  *  STEPS_by_REV) / mm_by_REV;

//vars
unsigned int MaxPos_step =  (MAX_POS_mm *  STEPS_by_REV) / mm_by_REV;
bool CycleOn = true;
bool HomeDone = false;

AccelStepper stepper(AccelStepper::DRIVER, STP_PIN, DIR_PIN );

void setup()
{
  Serial.begin(250000);
  pinMode(10  , OUTPUT);
  pinMode(STP_PIN  , OUTPUT);
  pinMode(DIR_PIN    , OUTPUT);
  pinMode(ENA_PIN    , OUTPUT);
  digitalWrite(ENA_PIN , HIGH);
  stepper.setMaxSpeed(MAX_SPEED_STEPS);
  stepper.setAcceleration(MAX_ACCEL_STEPS2);
  MotorGoHome();
  stepper.setMaxSpeed(MAX_SPEED_STEPS);
  stepper.setAcceleration(MAX_ACCEL_STEPS2);
  wdt_enable(WDTO_2S);
}


void MotorGoHome()
{
  int CurrentPos = 0;
  Serial.println("Finding Home...");
  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(HOME_SPEED_STEPS);
  while (digitalRead(HOM_PIN) == HIGH)
  {
    CurrentPos = CurrentPos - HOME_INCREMENT_STEP;
    stepper.runToNewPosition(CurrentPos);
  }
  stepper.setCurrentPosition(0);
  stepper.runToNewPosition(HOME_PRESS_POSITION_STEP);
  stepper.setCurrentPosition(0);
  HomeDone = true;
  Serial.println("Home OK.");
}

void loop()
{
  wdt_reset();
  if (CycleOn)
  {
    if (stepper.currentPosition() == 0)
    {
      delay(500);
      stepper.moveTo(MaxPos_step);
    }
    else if (stepper.currentPosition() == MaxPos_step)
    {
      delay(500);
      stepper.moveTo(0);
    }
  }
  analogWrite(10, map(stepper.currentPosition(), 0, MaxPos_step, 0, 250));
  stepper.run();
  if((digitalRead(HOM_PIN) == LOW) && (HomeDone)) while(1);

  if (Serial.available() > 1)
  {
    char Option = Serial.read();
    if (Option == 'P')
    {
      float NewPos = Serial.parseFloat();
      stepper.moveTo((NewPos *  STEPS_by_REV) / mm_by_REV);
    }
    else if (Option == 'C')
    {
      CycleOn = !CycleOn ;
    }
    else if (Option == 'S')
    {
      float NewSpeed= Serial.parseFloat();
      stepper.setMaxSpeed((NewSpeed *  STEPS_by_REV) / mm_by_REV);
    }
    else if (Option == 'H')
    {
      MotorGoHome();
    }
  }
}
