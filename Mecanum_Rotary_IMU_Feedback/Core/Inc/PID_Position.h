#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
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
	
	/* Lowpass Filter */
	float alpha;
	/* Controller "memory" */
	float integrator[3];
	float prevError[3];			/* Required for integrator */
	float differentiator[3];
	float prevMeasurement[3];		/* Required for differentiator */

	/* Controller output */
	float out[4];
	

} PIDController;

void  PID_Position_Init(PIDController *pid, int nMotor);
float PID_Position(PIDController *pid, float setpoint, float measurement,float Kp,float Ki,float Kd, int i);

#endif
