#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define PWM

#define TAG "drive-bt"

#define LED1 GPIO_NUM_35
#define LED2 GPIO_NUM_36
// i reversed steer and drive outputs on the layout
#define STEER_A GPIO_NUM_10
#define STEER_B GPIO_NUM_9
#define MOTOR_A GPIO_NUM_7
#define MOTOR_B GPIO_NUM_6

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
//#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_OUTPUT_IO          (7) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (1096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (200) // Frequency in Hertz. Set frequency at 4 kHz


void init_io(void);

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 4096, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void app_main(void)
{
	ESP_LOGI(TAG, "hello");
	init_io();

	uint8_t led = 1;
	gpio_set_level(MOTOR_A, 0);
	gpio_set_level(MOTOR_B, 1);

	gpio_set_level(STEER_A, 0);
	gpio_set_level(STEER_B, 1);

#ifdef PWM
     // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    //ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    //ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    uint32_t dc = 0;
    int32_t delta = 250;
    int32_t dir = 1;
ESP_LOGI(TAG, "Setting to 4000");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 4000));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(3000 / portTICK_PERIOD_MS);
ESP_LOGI(TAG, "Setting to 6000");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 6000));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(3000 / portTICK_PERIOD_MS);
ESP_LOGI(TAG, "Setting to 8000");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 8000));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        #endif

	while(1) {
		gpio_set_level(LED1, led);
		gpio_set_level(LED2, !led);
        //gpio_set_level(STEER_A, led);
        //gpio_set_level(STEER_B, !led);
        led = !led;
#ifdef PWM
        ESP_LOGI(TAG, "setting dc to %lu", dc);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, dc));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        dc += (dir * delta);
        if(dc > 7000) {
            dir = -1 * dir;
        }
        if(dc < 250) {
            dir = -1 * dir;
        }
        #endif
        vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

}

void init_io(void) {
	gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED1);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << LED2);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << STEER_A);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << STEER_B);
    gpio_config(&io_conf);
    #ifndef PWM
    io_conf.pin_bit_mask = (1ULL << MOTOR_A);
    gpio_config(&io_conf);
    #endif
    io_conf.pin_bit_mask = (1ULL << MOTOR_B);
    gpio_config(&io_conf);
}