
#include "pid_phil.h"

#define ANTI_WINDUP (0)

PIDController::PIDController(float p, float i, float d,
                             float set_tau, float set_limMin, float set_limMax,
                             float set_limMinInt, float set_limMaxInt, float set_T)
{
    Kp = p;
    Ki = i;
    Kd = d;
    tau = set_tau;
    limMin = set_limMin;
    limMax = set_limMax;
    limMinInt = set_limMinInt;
    limMaxInt = set_limMaxInt;
    T = set_T;
    integrator = 0.f;
    prevError = 0.f;
    differentiator = 0.f;
    prevMeasurement = 0.f;
}

float PIDController::Update(float setpoint, float measurement)
{
    /*
	* Error signal
	*/
    float error = setpoint - measurement;


    /*
    * Proportional
    */
    float proportional = this->Kp * error;


    /*
    * Integral
    */
    this->integrator = this->integrator + 0.5f * this->Ki * this->T * (error + this->prevError);

    /* Anti-wind-up via integrator clamping */
#if ANTI_WINDUP
    if (this->integrator > this->limMaxInt) {

        this->integrator = this->limMaxInt;

    } else if (this->integrator < this->limMinInt) {

        this->integrator = this->limMinInt;

    }
#endif

    /*
    * Derivative (band-limited differentiator)
    */
		
    this->differentiator = -(2.0f * this->Kd * (measurement - this->prevMeasurement));   /* Note: derivative on measurement, therefore minus sign in front of equation! */
                        //    + (2.0f * this->tau - this->T) * this->differentiator)
                        //    / (2.0f * this->tau + this->T);


    /*
    * Compute output and apply limits
    */
    float out = proportional + this->integrator + this->differentiator;

    if (out > this->limMax) {

        out = this->limMax;

    } else if (out < this->limMin) {

        out = this->limMin;

    }

    /* Store error and measurement for later use */
    this->prevError       = error;
    this->prevMeasurement = measurement;

    /* Return controller output */
    return out;
}