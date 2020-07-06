#include "LinearActuator.h"

LinearActuator actuator1(6, 7, A0, 483, 487);
LinearActuator actuator2(8, 9, A1, 483, 487);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
}

bool FullLengthTest = false;
bool HalfLengthTest = false;
int halfcount_forward, halfcount_backward, halfcount;
int fullcount_forward, fullcount_backward;

int RawValue = 0;
int sys_rate = 100;
int sys_steps = 20;
int delay_value = sys_rate / sys_steps;

int counter = 0;
double raw_mean = 0;

void loop()
{
  // put your main code here, to run repeatedly:
  RawValue = actuator1.CurrentValue();
  Serial.println(RawValue);
  if (Serial.available() > 0)
  {
    char serialdata = Serial.read();
    FullLengthTest = HalfLengthTest = false;
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
    else if (serialdata == 'q')
    {
      // Test for full forward and backward
      FullLengthTest = true;
      fullcount_forward = fullcount_backward = 0;
      Serial.println("-----Initiating Full Length Test-----");
      Serial.println("Assuming the length of actuator is 0");
      actuator1.forward();
    }
    else if (serialdata == 'w')
    {
      // Test half forward and then backward
      HalfLengthTest = true;
      halfcount_forward = fullcount_forward / 2;
      halfcount_backward = fullcount_backward / 2;
      Serial.println("-----Initiating Half Length Test-----");
      Serial.println("Assuming the length of actuator is 0");
    }
  }

  delay(delay_value);

  counter++;
  raw_mean += RawValue;
  if (counter == sys_steps)
  {
    counter = 0;
    raw_mean = raw_mean / sys_steps;

    if (FullLengthTest)
    {
      //if (raw_mean < forwardThres)
      if (raw_mean < 483)
      {
        fullcount_forward++;
      }
      //else if (raw_mean > backwardThres)
      else if (raw_mean > 487)
      {
        fullcount_backward++;
      }
      else
      {
        // No current
        Serial.println(fullcount_forward);
        if (fullcount_backward == 0)
        {
          Serial.println(raw_mean);
          actuator1.backward();
        }
        else
        {
          Serial.println(fullcount_backward);
          Serial.println(raw_mean);
          actuator1.hault();
          // show stats
          FullLengthTest = false;
          Serial.println("---Stats---");
          Serial.print("Total Time: ");
          Serial.println((fullcount_forward + fullcount_backward) / 10.0);
          Serial.print("Forward Rate per inch: ");
          Serial.println(fullcount_forward / 10.16);
          Serial.print("Backward Rate per inch: ");
          Serial.println(fullcount_backward / 10.16);
        }
      }
    }
    else if (HalfLengthTest)
    {
      halfcount++;
      if (halfcount_forward > 0)
      {
        halfcount_forward--;
        actuator1.forward();
      }
      else if (halfcount_backward > 0)
      {
        halfcount_backward--;
        actuator1.backward();
      }
      else
      {
        // No current
        actuator1.hault();
        // show stats
        HalfLengthTest = false;
        Serial.println("---Stats---");
        Serial.print("Total Time: ");
        Serial.println(halfcount * 100);
      }
    }
    raw_mean = 0;
  }
}

// void GetCurrentReading()
// {
//   RawValue = analogRead(analogIn);
//   //  Voltage = (RawValue / 1024.0) * 5000; // Gets you mV
//   //  Amps = ((Voltage - ACSoffset) / mVperAmp);
//   Serial.println(RawValue);
//   //  Serial.print("\tVoltage: ");
//   //  Serial.print(Voltage);
//   //  Serial.print("\tCurrent: ");
//   //  Serial.println(Amps);
// }
