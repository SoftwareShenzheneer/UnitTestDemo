#ifndef __PID_H__
#define __PID_H__

struct PIDController{
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

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;
	float differentiator;
	float prevMeasurement;

	float out;
};

void PIDController_init(struct PIDController *pid);
float PIDController_update(struct PIDController *pid, float setpoint, float measurement);
void PIDController_toString(struct PIDController* pid);

#endif /* __PID_H__ */

