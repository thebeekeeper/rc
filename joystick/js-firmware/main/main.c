#define I2C_ADC

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c.h"
#include "nimble.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "js_data.h"

#define TAG "js"

#define GPIO_LED_1 7
#define GPIO_LED_2 10

#define GPIO_INPUT_BUTTON_1 18
#define GPIO_INPUT_BUTTON_2 19
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_BUTTON_1)
#define GPIO_INPUT_PIN2_SEL (1ULL << GPIO_INPUT_BUTTON_2)

#define SDA_PIN 3
#define SCL_PIN 4
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define ADC_ADDR 0x33
#define N_CHANNELS 5
#define VOLT_SCALE 0.000500122

js_reading js;

static int i2c_port = 0;
void init_adc(void)
{
        i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400000, // select frequency specific to your project
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    // --F = setup byte, SEL[2..0] = internal reference, ref output, always on--
    // --2 = internal clock, unipolar, dont reset, dc--
    // D = SEL[2..0] = internal, not connected, always on
    // A = external clock (SCL), unipolar, don't reset, dontcare
    //uint8_t data = 0xDA;
    // r1 has an external reference hooked up - SEL = 0b010
    // setup byte
    uint8_t data = 0xAA;
    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_port, ADC_ADDR, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    data = 0b00001001;
    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_port, ADC_ADDR, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

}
void read_joysticks(js_reading *js)
{
    uint8_t configByte = 0b00000001 | ((N_CHANNELS - 1) << 1);
    size_t i;
    uint8_t data[N_CHANNELS * 2];
    uint16_t reading = 0;
    uint16_t readings[N_CHANNELS] = { 0 };

    // writes config during the same operation as reading all input channels
    i2c_master_write_read_device(i2c_port, ADC_ADDR, &configByte, 1, &data[0], N_CHANNELS * 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    for (i = 0; i < N_CHANNELS; i++)
    {
        reading = (((uint16_t)(data[2 * i] & 0x0F)) << 8) | data[(2 * i) + 1];
        //printf("reading %i: %u\n", i, reading);
        // printf("%zu - %u\n", i, last_raw_value[i]);
        //float volts = (float)reading * VOLT_SCALE;
        //pressures[i] = (volts - V_MIN) * PSI_SCALE;
        readings[i] = reading;
        //ESP_LOGI(TAG, "value %i: %f", i, volts);
    }
    js->left_horizontal = (int8_t)((readings[4] - 2054) / 20);
    js->left_vertical = (int8_t)((readings[2] - 1984) / 19);
    js->right_horizontal = -1 * ((int8_t)((readings[1] - 2048) / 20));
    // this one isn't really working, think i bent a lead
    js->right_vertical = (int8_t)(readings[0]);

    //printf("%u\t%u\t%u\t%u\t%u\n", readings[0], readings[1], readings[2], readings[3], readings[4]);
}

void init_io(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = GPIO_INPUT_PIN2_SEL;
    gpio_config(&io_conf);

    // set up LEDs
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1uLL << GPIO_LED_1);
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1uLL << GPIO_LED_2);
    gpio_config(&io_conf);
}

void app_main(void)
{
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_io();
    init_adc();
    startBLE();
    uint8_t led = 0;

    xTaskCreate(vTasksendNotification, "vTasksendNotification", 4096, NULL, 1, &xHandle);

    while (true)
    {
        read_joysticks(&js);
        ESP_LOGI(TAG, "drive: %i\tturn: %i", js.left_vertical, js.right_horizontal);
        led++;
        if(led == 1) {
         gpio_set_level(GPIO_LED_1, 1);
        } else {
            gpio_set_level(GPIO_LED_1, 0);
        }
        if(led > 5) {
            led = 0;
        }
    
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
