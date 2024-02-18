#include <Arduino.h>
#include "motor.h"

#define MIN_RIGHT (35)
#define MIN_LEFT (25)

Motor::Motor(unsigned aIn, unsigned pwmA, unsigned bIn, unsigned pwmB, unsigned standby)
{
    aInPin = aIn;
    pwmAPin = pwmA;
    bInPin = bIn;
    pwmBPin = pwmB;
    standbyPin = standby;

    pinMode(aInPin, OUTPUT);
    pinMode(pwmAPin, OUTPUT);
    pinMode(bInPin, OUTPUT);
    pinMode(pwmBPin, OUTPUT);
    pinMode(standbyPin, OUTPUT);
}

void Motor::stop(void)
{
    digitalWrite(pwmAPin, LOW);
    digitalWrite(pwmBPin, LOW);
    digitalWrite(standbyPin, HIGH);
}

static int scaleSpeed(unsigned speed, unsigned min)
{
  int scaledSpeed;

  if (speed < 10)
  {
    scaledSpeed = 0;
  }
  else
  {
    scaledSpeed = map((int)speed, 0, 255, min, 255);
  }

  return scaledSpeed;
}

void Motor::forward(unsigned speed)
{
    
    // int rightCommand = scaleSpeed(speed, MIN_RIGHT);
    // int leftCommand = scaleSpeed(speed, MIN_LEFT);
    digitalWrite(aInPin, LOW);
    digitalWrite(bInPin, LOW);
    analogWrite(pwmAPin, speed);
    analogWrite(pwmBPin, speed);
    digitalWrite(standbyPin, HIGH);
}

void Motor::backward(unsigned speed)
{
    // int rightCommand = scaleSpeed(speed, MIN_RIGHT);
    // int leftCommand = scaleSpeed(speed, MIN_LEFT);
    digitalWrite(aInPin, HIGH);
    digitalWrite(bInPin, HIGH);
    analogWrite(pwmAPin, speed);
    analogWrite(pwmBPin, speed);
    digitalWrite(standbyPin, HIGH);   
}