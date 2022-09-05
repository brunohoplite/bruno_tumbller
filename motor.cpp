#include <Arduino.h>
#include "motor.h"

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

void Motor::forward(unsigned speed)
{
    digitalWrite(aInPin, LOW);
    digitalWrite(bInPin, LOW);
    analogWrite(pwmAPin, speed);
    analogWrite(pwmBPin, speed);
    digitalWrite(standbyPin, HIGH);
}

void Motor::backward(unsigned speed)
{
    digitalWrite(aInPin, HIGH);
    digitalWrite(bInPin, HIGH);
    analogWrite(pwmAPin, speed);
    analogWrite(pwmBPin, speed);
    digitalWrite(standbyPin, HIGH);   
}