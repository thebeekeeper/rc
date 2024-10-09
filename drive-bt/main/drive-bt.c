#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"



#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "blecent.h"

#define PWM

#define TAG "drive-bt"

void init_io(void);

#define LED1 GPIO_NUM_35
#define LED2 GPIO_NUM_36

#define STEER_A GPIO_NUM_7
#define STEER_B GPIO_NUM_6
#define MOTOR_A GPIO_NUM_10
#define MOTOR_B GPIO_NUM_9

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
//#define LEDC_OUTPUT_IO (MOTOR_A) // Define the output GPIO
//#define LEDC_OUTPUT_B (MOTOR_B)
#define MOTOR_A_CHANNEL LEDC_CHANNEL_0
#define MOTOR_B_CHANNEL LEDC_CHANNEL_1
#define STEER_A_CHANNEL LEDC_CHANNEL_2
#define STEER_B_CHANNEL LEDC_CHANNEL_3
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
//#define LEDC_DUTY (1096)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY (200)            // Frequency in Hertz. Set frequency at 4 kHz

// called from the BLE handler
void update_outputs(int8_t drive, int8_t steer) {
    uint32_t dc = 0;
    if(drive > 10) {
        dc = drive * 53; 
        ESP_LOGI(TAG, "setting drive dc forward: %lu", dc);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_A_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_A_CHANNEL));
        // turn off channel B
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_B_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_B_CHANNEL));
    }
    else if(drive < 10) {
        dc = drive * 53; 
        ESP_LOGI(TAG, "setting drive dc reverse: %lu", dc);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_A_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_A_CHANNEL));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_B_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_B_CHANNEL));
    }
    else {
        dc = 0;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_A_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_A_CHANNEL));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, MOTOR_B_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, MOTOR_B_CHANNEL));
    }

    if(steer> 10) {
        dc = steer* 53; 
        ESP_LOGI(TAG, "setting steer left: %lu", dc);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, STEER_A_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, STEER_A_CHANNEL));
        // turn off channel B
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, STEER_B_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, STEER_B_CHANNEL));
    }
    else if(drive < 10) {
        dc = drive * 53; 
        ESP_LOGI(TAG, "setting steer right: %lu", dc);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, STEER_A_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, STEER_A_CHANNEL));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, STEER_B_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, STEER_B_CHANNEL));
    }
    else {
        dc = 0;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, STEER_A_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, STEER_A_CHANNEL));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, STEER_B_CHANNEL, dc));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, STEER_B_CHANNEL));
    }
}


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_A_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_A,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel_config_t ledc_channel_b = {
        .speed_mode = LEDC_MODE,
        .channel = MOTOR_B_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_B,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));

    ledc_channel_config_t ledc_channel_c = {
        .speed_mode = LEDC_MODE,
        .channel = STEER_A_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = STEER_A,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_c));

    ledc_channel_config_t ledc_channel_d = {
        .speed_mode = LEDC_MODE,
        .channel = STEER_B_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = STEER_B,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_d));
}

void app_main(void)
{
    ESP_LOGI(TAG, "hello");
    init_io();

    uint8_t led = 1;
    //gpio_set_level(MOTOR_A, 0);
    //gpio_set_level(MOTOR_B, 0);

    //gpio_set_level(STEER_A, 0);
    //gpio_set_level(STEER_B, 0);

#ifdef PWM
    // Set the LEDC peripheral configuration
    example_ledc_init();
    init_ble();
#endif

    while (1)
    {
        gpio_set_level(LED1, led);
        gpio_set_level(LED2, !led);
        // useful for testing hardware
#ifndef PWM
        gpio_set_level(STEER_A, led);
        gpio_set_level(STEER_B, !led);
        gpio_set_level(MOTOR_A, led);
        gpio_set_level(MOTOR_B, !led);
#endif
        led = !led;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void init_io(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED1);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << LED2);
    gpio_config(&io_conf);
#ifndef PWM
    io_conf.pin_bit_mask = (1ULL << STEER_A);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << STEER_B);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << MOTOR_A);
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << MOTOR_B);
    gpio_config(&io_conf);
#endif
}