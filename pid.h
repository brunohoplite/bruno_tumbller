
#pragma once

class Pid {
    float kp;
    float kd;
    float ki;

  public:
    Pid(float, float, float);
    float computePid(float, float, float);
    float getError(void);

  private:
    float integral;
    float previousError;
};

Pid::Pid(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
    integral = 0;
    previousError = 0;
}

float Pid::computePid(float input, float setPoint, float timeDelta)
{
    float error = setPoint - input;
    float derivative = (error - previousError) / timeDelta;
    integral += error * timeDelta;
    previousError = error;

    return (kp * error) + (ki * integral) + (kd * derivative);
}

float Pid::getError(void)
{
    return previousError;
}