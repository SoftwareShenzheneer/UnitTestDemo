#include <stdio.h>
#include "driver/ledc.h"
#include "esp_log.h"

#include "gpioconfig.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (23)
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_DUTY               (4096)
#define LEDC_FREQUENCY          (4000)

void gpioconfig_init(void) {
  ledc_timer_config_t ledc_timer = {
    .speed_mode       = LEDC_MODE,
    .duty_resolution  = LEDC_DUTY_RES,
    .timer_num        = LEDC_TIMER,
    .freq_hz          = LEDC_FREQUENCY,
    .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {
    .speed_mode     = LEDC_MODE,
    .channel        = LEDC_CHANNEL,
    .timer_sel      = LEDC_TIMER,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = LEDC_OUTPUT_IO,
    .duty           = 0,
    .hpoint         = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  gpioconfig_setDc(2095);
}

void gpioconfig_setDc(uint16_t duty) {
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

