#include "AccelStepper.h"

//Pins
#define STP_PIN 4
#define DIR_PIN 3
#define ENA_PIN 2
#define HOM_PIN 12

//Max
#define MAX_ACCEL_mms2 450.0
#define MAX_SPEED_mms  200.0
#define MAX_POS_mm 93.0

//Home Parameters
#define HOME_SPEED_mms  10.0
#define HOME_INCREMENT_mm  1.0
#define HOME_PRESS_POSITION_mm  2.0

//Stepper Motor parameters
#define STEPS_by_REV  400.0
#define mm_by_REV 25.0

//Expiration Speed
#define EXP_SPEED_mms 150
#define EXP_ACCEL_mms2 100

//Consts
const unsigned int MAX_ACCEL_STEPS2 = (MAX_ACCEL_mms2 *  STEPS_by_REV) / mm_by_REV;
const unsigned int MAX_SPEED_STEPS = (MAX_SPEED_mms *  STEPS_by_REV) / mm_by_REV;
const unsigned int HOME_SPEED_STEPS = (HOME_SPEED_mms *  STEPS_by_REV) / mm_by_REV;
const unsigned int HOME_INCREMENT_STEP = (HOME_INCREMENT_mm  *  STEPS_by_REV) / mm_by_REV;
const unsigned int HOME_PRESS_POSITION_STEP = (HOME_PRESS_POSITION_mm  *  STEPS_by_REV) / mm_by_REV;
const unsigned int MAX_POS_STEP =  (MAX_POS_mm *  STEPS_by_REV) / mm_by_REV;
const unsigned int EXP_SPEED_STEP =  (EXP_SPEED_mms*  STEPS_by_REV) / mm_by_REV;
const unsigned int EXP_ACCEL_STEP2 =  (EXP_ACCEL_mms2*  STEPS_by_REV) / mm_by_REV;

//vars
AccelStepper motor(AccelStepper::DRIVER, STP_PIN, DIR_PIN );
bool Speed_mode = false;
bool Home_Done = false;

void MotorSetup()
{
  pinMode(STP_PIN  , OUTPUT);
  pinMode(DIR_PIN    , OUTPUT);
  pinMode(ENA_PIN    , OUTPUT);
  pinMode(HOM_PIN    , INPUT);
  digitalWrite(ENA_PIN , HIGH);
  motor.setAcceleration(MAX_ACCEL_STEPS2);
  MotorGoHome();
  motor.setMaxSpeed(MAX_SPEED_STEPS);
}

void MotorGoHome()
{
  long CurrentPos = 0;
  Home_Done = false;
  DBG_PRINTLN("Finding Home...");
  motor.setCurrentPosition(0);
  motor.setMaxSpeed(HOME_SPEED_STEPS);
  unsigned long tempoMaximoGoHome = millis() + 20000UL; //tempo máximo para ir para o home, se não chegar reinicia
  while ((digitalRead(HOM_PIN) == HIGH)&&(millis() < tempoMaximoGoHome))
  {
    CurrentPos = CurrentPos - HOME_INCREMENT_STEP;
    motor.runToNewPosition(CurrentPos);
    wdt_reset();
  }
  if (digitalRead(HOM_PIN) == HIGH) {
    DBG_PRINTLN(F("Não chegou em Home, reiniciando..."));
    while (1);
  } else {
    motor.setCurrentPosition(0);
    motor.runToNewPosition(HOME_PRESS_POSITION_STEP);
    motor.setCurrentPosition(0);
    DBG_PRINTLN("Home OK.");
    Home_Done = true;
  }
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void MotorLoop()
{
  if ((digitalRead(HOM_PIN) == LOW) && (Home_Done)) while (1);
  if (Speed_mode)
    motor.runSpeed();
  else
    motor.run();
}

float MotorGetCurrentPos_mm()
{
  return ((float)motor.currentPosition() /  STEPS_by_REV) * mm_by_REV;
}


float MotorGetCurrentSpeed_mms()
{
  return (motor.speed() /  STEPS_by_REV) * mm_by_REV;
}

void MotorGo2MaxPos_AccelMode(float Speed_mms)
{
  if (Speed_mms > MAX_SPEED_mms) Speed_mms = MAX_SPEED_mms;
  if (Speed_mms < 0) Speed_mms = 0;
  Speed_mode = false;
  motor.setSpeed(0);
  motor.setAcceleration(0);
  motor.setAcceleration((Speed_mms *  STEPS_by_REV) / mm_by_REV);
  motor.moveTo(MAX_POS_STEP);
}


void MotorGo2StartPos_AccelMode(float Speed_mms)
{
  if (Speed_mms > MAX_SPEED_mms) Speed_mms = MAX_SPEED_mms;
  if (Speed_mms < 0) Speed_mms = 0;
  Speed_mode = false;
  motor.setSpeed(0);
  motor.setAcceleration(0);
  motor.setMaxSpeed((Speed_mms *  STEPS_by_REV) / mm_by_REV);
  motor.moveTo(0);
}

void MotorGo2MaxPos_SpeedMode(float Speed_mms)
{
  if (Speed_mms > MAX_SPEED_mms) Speed_mms = MAX_SPEED_mms;
  if (Speed_mms < 0) Speed_mms = 0;
  Speed_mode = true;
  motor.setSpeed((Speed_mms *  STEPS_by_REV) / mm_by_REV);
  motor.moveTo(MAX_POS_STEP);
}

void MotorGo2StartPos_SpeedMode(float Speed_mms)
{
  if (Speed_mms > MAX_SPEED_mms) Speed_mms = MAX_SPEED_mms;
  if (Speed_mms < 0) Speed_mms = 0;
  Speed_mode = true;
  motor.setSpeed((Speed_mms *  STEPS_by_REV) / mm_by_REV);
  motor.moveTo(0);
}

void MotorSetSpeed(float Speed_mms)
{
  if (Speed_mms > MAX_SPEED_mms) Speed_mms = MAX_SPEED_mms;
  if (Speed_mms < 0) Speed_mms = 0;
  motor.setSpeed((Speed_mms *  STEPS_by_REV) / mm_by_REV);
}

void MotorSetAccel(float Accel_mms2)
{
  if (Accel_mms2 > MAX_ACCEL_mms2) Accel_mms2 = MAX_ACCEL_mms2;
  if (Accel_mms2 < 0) Accel_mms2 = 0;
  motor.setAcceleration((Accel_mms2 *  STEPS_by_REV) / mm_by_REV);
}

bool MotorIsOnMaxPosition()
{
  return (motor.currentPosition() >= MAX_POS_STEP);
}

bool MotorIsOnStartPosition()
{
  return (motor.currentPosition() <= 0);
}

void MotorSet_InsNewPosition_mm(float MotorNewPosition_mm, float dt)
{
  float Speed_mms = (MotorNewPosition_mm - MotorGetCurrentPos_mm())/dt;
  MotorSetSpeed(Speed_mms);
}

void MotorGoExp_AccelMode()
{
  MotorGo2StartPos_AccelMode(EXP_ACCEL_STEP2);
}

void MotorGoExp_SpeedMode()
{
  MotorGo2StartPos_SpeedMode(EXP_SPEED_STEP);
}
