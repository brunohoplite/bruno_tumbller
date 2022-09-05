
#pragma once

class Pid {
    float kp;
    float kd;
    float ki;
    float setPoint;

  public:
    Pid(float, float, float, float);
    float computePid(float, float);
    float getError(void);

  private:
    float integral;
    float previousError;
};

Pid::Pid(float p, float i, float d, float target)
{
    kp = p;
    ki = i;
    kd = d;
    setPoint = target;
    integral = 0;
}

float Pid::computePid(float input, float timeDelta)
{
    float error = setPoint - input;
    float derivative = (error - previousError) / timeDelta;
    previousError = error;
    integral += error * timeDelta;

    return (kp * error) + (ki * integral) + (kd * derivative);
}

float Pid::getError(void)
{
    return previousError;
}