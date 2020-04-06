
void SerialSetup()
{
  Serial.begin(250000);
}

void SerialLoop()
{
if (Serial.available() > 1)
  {
    char Option = Serial.read();
    if (Option == 'R')
    {
      BreathChangeR(Serial.parseFloat());
    }
    else if (Option == 'P')
    {
      BreathChangePplat(Serial.parseFloat());
    }
    else if (Option == 'T')
    {
      BreathChangeTPlat(Serial.parseFloat());
    }
    else if (Option == 'C')
    {
      BreathChangeC(Serial.parseFloat());
    }
    else if (Option == 'B')
    {
      BreathChangeB(Serial.parseFloat());
    }
    else if (Option == 'R')
    {
      while(1);
    }
  }
}
