/*
Author : Antariksh Narain
Email  : antariksh.cloud@gmail.com
Project: LEAPFROG|SERC|ISI|USC
License: MIT License
*/

# include "Arduino.h"
# include "LinearActuator.h"

LinearActuator::LinearActuator(int digitalPin1, int digitalPin2, int analogPin, double thresholdF, double thresholdB)
{
    // Set class variables
    this->FORWARD_pin = digitalPin1;
    this->BACKWARD_pin = digitalPin2;
    this->analogIn = analogPin;
    this->forwardThres = thresholdF;
    this->backwardThres = thresholdB;

    // Initialize Pins
    pinMode(this->FORWARD_pin, OUTPUT);
    pinMode(this->BACKWARD_pin, OUTPUT);
}
void LinearActuator::forward()
{
    digitalWrite(this->FORWARD_pin, LOW);
    digitalWrite(this->BACKWARD_pin, HIGH);
    Serial.println("Forward");
    //delay(100);
}

void LinearActuator::backward()
{
    digitalWrite(this->FORWARD_pin, HIGH);
    digitalWrite(this->BACKWARD_pin, LOW);
    Serial.println("Backward");
    //delay(100);
}

void LinearActuator::hault()
{
    digitalWrite(this->FORWARD_pin, HIGH);
    digitalWrite(this->BACKWARD_pin, HIGH);
    Serial.println("Stop");
}

int LinearActuator::CurrentValue()
{
    // Calibrate sensor Value
    return analogRead(this->analogIn);
}
