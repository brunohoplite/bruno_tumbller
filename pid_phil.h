
#ifndef PID_PHIL_H
#define PID_PHIL_H

class  PIDController
{
private:
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

public:
    PIDController(float, float, float, float, float, float, float, float);
    float Update(float setpoint, float measurement, float timeStep);
};

#endif // PID_PHIL_H