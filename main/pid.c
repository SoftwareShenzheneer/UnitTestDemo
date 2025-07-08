#include <stdio.h>

#include "pid.h"
#include "gpioconfig.h"

#define PID_KP  2.0f
#define PID_KI  3.5f
#define PID_KD  0.25f
#define PID_TAU 0.02f
#define PID_LIM_MIN -100.0f
#define PID_LIM_MAX  100.0f
#define PID_LIM_MIN_INT -85.0f
#define PID_LIM_MAX_INT  85.0f

#define SAMPLE_TIME_S 0.01f

static const char* TAG = "PID";

void PIDController_toString(struct PIDController* pid) {
  /* Stub */
}

void PIDController_init(struct PIDController *pid) {
  pid->Kp = PID_KP;
  pid->Ki = PID_KI;
  pid->Kd = PID_KD;

  pid->tau = PID_TAU;

  pid->limMin = PID_LIM_MIN;
  pid->limMax = PID_LIM_MAX;

  pid->limMinInt = PID_LIM_MIN_INT;
  pid->limMaxInt = PID_LIM_MAX_INT;

  pid->T = SAMPLE_TIME_S;

  pid->integrator = 0.0f;
  pid->prevError  = 0.0f;

  pid->differentiator  = 0.0f;
  pid->prevMeasurement = 0.0f;

  pid->out = 0.0f;
}

float PIDController_update(struct PIDController *pid, float setpoint, float measurement) {
  float error = setpoint - measurement;
  float proportional = pid->Kp * error;
  uint16_t duty = 0;

  pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

  if (pid->integrator > pid->limMaxInt) {
    pid->integrator = pid->limMaxInt;
  } else if (pid->integrator < pid->limMinInt) {
    pid->integrator = pid->limMinInt;
  }

  pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
      + (2.0f * pid->tau - pid->T) * pid->differentiator)
    / (2.0f * pid->tau + pid->T);
  pid->out = proportional + pid->integrator + pid->differentiator;
  if (pid->out > pid->limMax) {
    pid->out = pid->limMax;
  } else if (pid->out < pid->limMin) {
    pid->out = pid->limMin;
  }
  pid->prevError       = error;
  pid->prevMeasurement = measurement;

  duty = 8192 * pid->out / 100;
  gpioconfig_setDc(duty);

  return pid->out;
}

