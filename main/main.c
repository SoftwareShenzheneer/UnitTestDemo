#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "pid.h"
#include "sdkconfig.h"
#include "gpioconfig.h"

#define MILLISECONDS_PER_TICK 20
#define SAMPLE_TIME_S 0.01f

static const char* TAG = "main";
static uint32_t tickCount = 0;

float generateOutput(float inp);

void app_main(void) {
  gpioconfig_init();

  struct PIDController* pid = (struct PIDController*)malloc(sizeof(struct PIDController));
  if (NULL == pid) {
    ESP_LOGE(TAG, "Error allocating memory for PID controller.\r\n");
  }
  PiDController_Init(pid);
  PIDController_toString(pid);

  float setpoint = 75.0;

  ESP_LOGI(TAG, "Time (ticks)\tSystem Output\tController Output\r\n");
  while (true) {
    float measurement = generateOutput(pid->out);
    PIDController_update(pid, setpoint, measurement);
    ESP_LOGI(TAG, "%lu\t%.2f\t%.2f\r\n", tickCount, measurement, pid->out);

    tickCount++;
    vTaskDelay(MILLISECONDS_PER_TICK / portTICK_PERIOD_MS);
  }
}

float generateOutput(float inp) {
  static float x = 0.0f;      // position
  static float v = 0.0f;      // velocity

  // system parameters
  const float mass = 1.0f;    // kg
  const float damping = 1.2f; // Ns/m (adjust <2.0 for overshoot)
  const float spring = 20.0f; // N/m

  // Compute acceleration: F = ma => a = (k*u - k*x - c*v)/m
  float force = spring * (inp - x) - damping * v;
  float a = force / mass;

  // Integrate velocity and position
  v += a * SAMPLE_TIME_S;
  x += v * SAMPLE_TIME_S;

  return x;
}
