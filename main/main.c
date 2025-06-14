#include <esp_err.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lp_core_i2c.h>
#include <lp_core_main.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <ulp_lp_core.h>

#define FILE_TAG "main"

extern const uint8_t lp_core_main_bin_start[] asm(
    "_binary_lp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[] asm("_binary_lp_core_main_bin_end");

static void load_lp_core_bin(void);
static void start_lp_core_bin(void);
static void lp_i2c_init(void);

void app_main(void) {
    const char *TAG = FILE_TAG ".app_main";

    ESP_LOGI(TAG, "Initializing LP I2C...");
    lp_i2c_init();

    ESP_LOGI(TAG, "Loading LP Core Binary...");
    load_lp_core_bin();

    ESP_LOGI(TAG, "Starting LP Core Binary...");
    start_lp_core_bin();

    // Wait for the LP Core to finish its initialization
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Data from the LP Core - raw: %d, error: %d",
             (int16_t)ulp_raw_reading, (esp_err_t)ulp_err);
}

static void load_lp_core_bin(void) {
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(
        lp_core_main_bin_start,
        (lp_core_main_bin_end - lp_core_main_bin_start)));
}

static void start_lp_core_bin(void) {
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
    };

    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));
}

static void lp_i2c_init(void) {
    const lp_core_i2c_cfg_t i2c_cfg = LP_CORE_I2C_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg));
}