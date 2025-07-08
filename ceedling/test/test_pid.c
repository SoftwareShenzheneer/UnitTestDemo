#ifndef TEST
#define TEST
#endif

#ifdef TEST

#include <stdlib.h>
#include "unity.h"

#include "mock_gpioconfig.h"

#include "pid.h"

struct PIDController* pid = NULL;

void setUp(void) {
  pid = (struct PIDController*)malloc(sizeof(struct PIDController));
  TEST_ASSERT_NOT_NULL(pid);
}

void tearDown(void) {
  free(pid);
  pid = NULL;
}

static void resetPid(void) {
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float tau = 0;
  float limMin = 0;
  float limMax = 0;
  float limMinInt = 0;
  float limMaxInt = 0;
  float T = 0;
  float integrator = 0;
  float prevError = 0;
  float differentiator = 0;
  float prevMeasurement = 0;
  float out = 0;

  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->tau = tau;
  pid->limMin = limMin;
  pid->limMax = limMax;
  pid->limMinInt = limMinInt;
  pid->limMaxInt = limMaxInt;
  pid->T = T;
  pid->integrator = integrator;
  pid->prevError = prevError;
  pid->differentiator = differentiator;
  pid->prevMeasurement = prevMeasurement;
  pid->out = out;
}

static void setMinimalSaneValues(void) {
  float tau = 1;
  float limMin = -100;
  float limMax = 100;
  float limMinInt = -100;
  float limMaxInt = 100;
  float T = 1;

  pid->tau = tau;
  pid->limMin = limMin;
  pid->limMax = limMax;
  pid->limMinInt = limMinInt;
  pid->limMaxInt = limMaxInt;
  pid->T = T;
}

void test_setParameters(void) {
  resetPid();

  float Kp = 12.3;
  float Ki = 12.3;
  float Kd = 12.3;
  float tau = 12.3;
  float limMin = 12.3;
  float limMax = 12.3;
  float limMinInt = 12.3;
  float limMaxInt = 12.3;
  float T = 12.3;
  float integrator = 12.3;
  float prevError = 12.3;
  float differentiator = 12.3;
  float prevMeasurement = 12.3;
  float out = 12.3;

  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->tau = tau;
  pid->limMin = limMin;
  pid->limMax = limMax;
  pid->limMinInt = limMinInt;
  pid->limMaxInt = limMaxInt;
  pid->T = T;
  pid->integrator = integrator;
  pid->prevError = prevError;
  pid->differentiator = differentiator;
  pid->prevMeasurement = prevMeasurement;
  pid->out = out;

  TEST_ASSERT_EQUAL_FLOAT(pid->Kp, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->Ki, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->Kd, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->tau, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMin, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMax, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMinInt, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMaxInt, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->T, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->integrator, 12.3);
  TEST_ASSERT_EQUAL_FLOAT(pid->prevError, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->differentiator, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->prevMeasurement, 12.3f);
  TEST_ASSERT_EQUAL_FLOAT(pid->out, 12.3f);
}

void test_resetParameters(void) {
  resetPid();

  TEST_ASSERT_EQUAL_FLOAT(pid->Kp, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->Ki, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->Kd, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->tau, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMin, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMax, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMinInt, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->limMaxInt, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->T, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->integrator, 0.0);
  TEST_ASSERT_EQUAL_FLOAT(pid->prevError, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->differentiator, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->prevMeasurement, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(pid->out, 0.0f);
}

/* Test struct to string method */
void test_tostring(void) {
  PIDController_toString(pid);
}

/* 
 * Test struct initialization
 * */
void test_init(void) {
  struct PIDController expected_pid = { 0 };
  expected_pid.Kp = 2.0f;
  expected_pid.Ki = 3.5f;
  expected_pid.Kd = 0.25f;

  expected_pid.tau = 0.02f;
  
  expected_pid.limMin = -100.0f;
  expected_pid.limMax = 100.0f;
  
  expected_pid.limMinInt = -85.0f;
  expected_pid.limMaxInt = 85.0f;
  
  expected_pid.T = 0.01f;
  
  expected_pid.integrator = 0.0f;
  expected_pid.prevError = 0.0f;
  
  expected_pid.differentiator = 0.0f;
  expected_pid.prevMeasurement = 0.0f;
  
  expected_pid.out = 0.0f;
  PIDController_init(pid);
  TEST_ASSERT_EQUAL_MEMORY(pid, &expected_pid, sizeof(struct PIDController));
}

/* 
 * Test proportional gain by:
 * 1. Reset pid values to 0
 * 2. Init pid with sane minimal configuration values
 * 3. Set setpoint, input and proportional gain
 * 4. Calculate expected result and compare to actual result
 * */
void test_proportionalGain(void) {
  resetPid();
  setMinimalSaneValues();
  
  float setpoint = 69.0f;
  float input = 40.0f;
  pid->Kp = 2.0f;

  float output = 0.0f;
  float expectedOutput = 2.0f * (setpoint - input);

  gpioconfig_setDc_Expect(8192 * expectedOutput / 100);
  output = PIDController_update(pid, setpoint, input);

  TEST_ASSERT_EQUAL(output, expectedOutput);
}

/* 
 * Test integral gain by:
 * 1. Reset pid values to 0
 * 2. Init pid with sane minimal configuration values
 * 3. Set setpoint, input and integral gain
 * 4. Calculate expected result and compare to actual result
 */
void test_integralGain(void) {
  /* pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError); */
  resetPid();
  setMinimalSaneValues();

  float setpoint = 69.0f;
  float input = 40.0f;
  pid->Ki = 2.0f;

  float output = 0.0f;
  float expectedOutput = pid->integrator + 0.5f * 2.0 * pid->T * (69.0 - 40.0 + pid->prevError);

  gpioconfig_setDc_Expect(8192 * expectedOutput / 100);
  output = PIDController_update(pid, setpoint, input);

  TEST_ASSERT_EQUAL(output, expectedOutput);
}

/* 
 * Test derivative gain by:
 * 1. Reset pid values to 0
 * 2. Init pid with sane minimal configuration values
 * 3. Set setpoint, input and derivative gain
 * 4. Calculate expected result and compare to actual result
 */
void test_derivativeGain(void) {
  resetPid();
}

/* 
 * Test
 * */
void test_maxOutputCap(void) {
  resetPid();
}

/* 
 * Test
 * */
void test_setpointReached(void) {
  resetPid();
}

/* 
 * Test
 * */
void test_resetStates(void) {
  resetPid();
}

/* 
 * Test
 * */
void test_noiseResponse(void) {
  resetPid();
}

#endif // TEST
