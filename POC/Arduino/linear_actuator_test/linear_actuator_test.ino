#include "LinearActuator.h"

LinearActuator actuator1(6, 7, A0, 483, 487);
LinearActuator actuator2(8, 9, A1, 483, 487);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.print(actuator1.CurrentValue());
  Serial.print("\t");
  Serial.println(actuator2.CurrentValue());
  if (Serial.available() > 0)
  {
    char serialdata = Serial.read();
    if (serialdata == 'a')
    {
      actuator1.forward();
    }
    else if (serialdata == 'd')
    {
      actuator1.backward();
    }
    else if (serialdata == 's')
    {
      actuator1.hault();
    }

    if (serialdata == 'q')
    {
      actuator1.forward();
    }
    else if (serialdata == 'e')
    {
      actuator1.backward();
    }
    else if (serialdata == 'w')
    {
      actuator1.hault();
    }
  }
}
