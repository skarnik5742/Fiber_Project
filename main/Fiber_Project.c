/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_check.h"

// ===================================================
// CONFIGURATION
// ===================================================

#define TAG "LED_CONTROLLER"

#define I2C_SDA_PIN           17
#define I2C_SCL_PIN           18

#define I2C_PORT_NUM          0
#define I2C_FREQ_HZ           100000

#define TCA9548A_ADDRESS      0x70

#define NUM_CHANNELS          3
#define NUM_EXPANDERS         6
#define LEDS_PER_EXPANDER     8
#define LEDS_PER_CHANNEL      48
#define TOTAL_LEDS            144

// ===================================================
// TYPES
// ===================================================

typedef struct
{
    uint8_t mux_channel;
    uint8_t expander_index;
    uint8_t pcf_address;
    uint8_t pcf_pin;
} led_location_t;

// ===================================================
// GLOBALS
// ===================================================

static i2c_master_bus_handle_t bus_handle;

static i2c_master_dev_handle_t mux_handle;

static i2c_master_dev_handle_t pcf_handles[NUM_CHANNELS][NUM_EXPANDERS];

static uint8_t pcf_states[NUM_CHANNELS][NUM_EXPANDERS];

// ===================================================
// I2C INITIALIZATION
// ===================================================

static void init_i2c(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_cfg, &bus_handle));

    ESP_LOGI(TAG, "I2C initialized");
}

// ===================================================
// TCA9548A
// ===================================================

static void init_mux(void)
{
    i2c_device_config_t mux_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9548A_ADDRESS,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            bus_handle,
            &mux_cfg,
            &mux_handle));

    ESP_LOGI(TAG, "MUX initialized");
}

static void mux_select_channel(uint8_t channel)
{
    if (channel > 7) {
        return;
    }

    uint8_t cmd = (1 << channel);

    ESP_ERROR_CHECK(
        i2c_master_transmit(
            mux_handle,
            &cmd,
            1,
            -1));
}

// ===================================================
// PCF8574A
// ===================================================

static void init_pcf_devices(void)
{
    uint8_t addresses[NUM_EXPANDERS] = {
        0x38,
        0x39,
        0x3A,
        0x3B,
        0x3C,
        0x3D
    };

    for (int channel = 0; channel < NUM_CHANNELS; channel++) {

        mux_select_channel(channel);

        for (int expander = 0; expander < NUM_EXPANDERS; expander++) {

            i2c_device_config_t cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addresses[expander],
                .scl_speed_hz = I2C_FREQ_HZ,
            };

            ESP_ERROR_CHECK(
                i2c_master_bus_add_device(
                    bus_handle,
                    &cfg,
                    &pcf_handles[channel][expander]));

            pcf_states[channel][expander] = 0x00;

            ESP_ERROR_CHECK(
                i2c_master_transmit(
                    pcf_handles[channel][expander],
                    &pcf_states[channel][expander],
                    1,
                    -1));
        }
    }

    ESP_LOGI(TAG, "PCF8574A devices initialized");
}

// ===================================================
// LED MAPPING
// ===================================================

static led_location_t get_led_location(uint16_t led_num)
{
    led_location_t loc = {0};

    uint16_t index = led_num - 1;

    loc.mux_channel = index / LEDS_PER_CHANNEL;

    uint8_t channel_position =
        index % LEDS_PER_CHANNEL;

    loc.expander_index =
        channel_position / LEDS_PER_EXPANDER;

    loc.pcf_address =
        0x38 + loc.expander_index;

    loc.pcf_pin =
        channel_position % LEDS_PER_EXPANDER;

    return loc;
}

// ===================================================
// LED CONTROL
// ===================================================

static void set_led_on(uint16_t led_num)
{
    if ((led_num < 1) || (led_num > TOTAL_LEDS)) {
        return;
    }

    led_location_t loc =
        get_led_location(led_num);

    mux_select_channel(loc.mux_channel);

    pcf_states[loc.mux_channel]
              [loc.expander_index]
        |= (1 << loc.pcf_pin);

    ESP_ERROR_CHECK(
        i2c_master_transmit(
            pcf_handles
                [loc.mux_channel]
                [loc.expander_index],
            &pcf_states
                [loc.mux_channel]
                [loc.expander_index],
            1,
            -1));
}

static void set_led_off(uint16_t led_num)
{
    if ((led_num < 1) || (led_num > TOTAL_LEDS)) {
        return;
    }

    led_location_t loc =
        get_led_location(led_num);

    mux_select_channel(loc.mux_channel);

    pcf_states[loc.mux_channel]
              [loc.expander_index]
        &= ~(1 << loc.pcf_pin);

    ESP_ERROR_CHECK(
        i2c_master_transmit(
            pcf_handles
                [loc.mux_channel]
                [loc.expander_index],
            &pcf_states
                [loc.mux_channel]
                [loc.expander_index],
            1,
            -1));
}

static void all_on(void)
{
    for (int led = 1; led <= TOTAL_LEDS; led++) {
        set_led_on(led);
    }
}

static void all_off(void)
{
    for (int led = 1; led <= TOTAL_LEDS; led++) {
        set_led_off(led);
    }
}

// ===================================================
// UART COMMANDS
// ===================================================

static void process_command(char *cmd)
{
    if (strncmp(cmd, "all_on", 6) == 0) {

        all_on();

        printf("ALL LEDs ON\r\n");
        return;
    }

    if (strncmp(cmd, "all_off", 7) == 0) {

        all_off();

        printf("ALL LEDs OFF\r\n");
        return;
    }

    if (strncmp(cmd, "on ", 3) == 0) {

        int led = atoi(&cmd[3]);

        set_led_on(led);

        printf("LED %d ON\r\n", led);
        return;
    }

    if (strncmp(cmd, "off ", 4) == 0) {

        int led = atoi(&cmd[4]);

        set_led_off(led);

        printf("LED %d OFF\r\n", led);
        return;
    }

    printf("Unknown command\r\n");
}

// ===================================================
// APP MAIN
// ===================================================

void app_main(void)
{
    init_i2c();

    init_mux();

    init_pcf_devices();

    printf("\r\n");
    printf("Commands:\r\n");
    printf("on 57\r\n");
    printf("off 57\r\n");
    printf("all_on\r\n");
    printf("all_off\r\n");
    printf("\r\n");

    char rx_buffer[64];

    while (1) {

        if (fgets(rx_buffer,
                  sizeof(rx_buffer),
                  stdin) != NULL) {

            rx_buffer[strcspn(rx_buffer,
                              "\r\n")] = 0;

            process_command(rx_buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}