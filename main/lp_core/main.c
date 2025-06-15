#include <esp_err.h>
#include <stdint.h>

#include "ads1115.h"

#define LP_CORE_FREQUENCY 16  // LP Core frequency in MHz

volatile int32_t raw_reading = 1234;
volatile esp_err_t err = 1234;
volatile uint64_t time_taken_us = 1234;
volatile bool done = false;

static inline uint64_t rdcycle64(void);

int main(void) {
    ads1115_t ads =
        ads1115_config(0, 0x48);  // The i2c_port_t is ignored at this moment by
                                  // lp_core_i2c_master_ operations

    ads1115_set_mode(&ads, ADS1115_MODE_SINGLE);
    ads1115_set_pga(&ads, ADS1115_FSR_1_024);
    ads1115_set_sps(&ads, ADS1115_SPS_860);
    ads1115_set_mux(&ads, ADS1115_MUX_0_GND);

    uint64_t start_cycles = rdcycle64();

    int32_t raw_sum = 0;
    for (int i = 0; i < 100; i++) {
        int16_t raw = 0;
        err = ads1115_get_raw(&ads, &raw);
        raw_sum += raw;
    }

    uint64_t end_cycles = rdcycle64();
    uint64_t elapsed_cycles = end_cycles - start_cycles;

    raw_reading = (int32_t)(raw_sum / 100);  // Average the readings
    time_taken_us =
        (elapsed_cycles / LP_CORE_FREQUENCY) / 100;  // in microseconds

    done = true;
}

static inline uint64_t rdcycle64(void) {
    uint32_t hi, lo, hi2;
    __asm__ volatile(
        "1: csrr %0, mcycleh\n"
        "   csrr %1, mcycle\n"
        "   csrr %2, mcycleh\n"
        "   bne  %0, %2, 1b\n"
        : "=&r"(hi), "=&r"(lo), "=&r"(hi2));
    return ((uint64_t)hi << 32) | lo;
}
