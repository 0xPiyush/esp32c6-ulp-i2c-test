#ifndef ADS1115_H
#define ADS1115_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ulp_lp_core_i2c.h>

typedef enum {  // register address
    ADS1115_CONVERSION_REGISTER_ADDR = 0,
    ADS1115_CONFIG_REGISTER_ADDR,
    ADS1115_LO_THRESH_REGISTER_ADDR,
    ADS1115_HI_THRESH_REGISTER_ADDR,
    ADS1115_MAX_REGISTER_ADDR
} ads1115_register_addresses_t;

typedef enum {  // multiplex options
    ADS1115_MUX_0_1 = 0,
    ADS1115_MUX_0_3,
    ADS1115_MUX_1_3,
    ADS1115_MUX_2_3,
    ADS1115_MUX_0_GND,
    ADS1115_MUX_1_GND,
    ADS1115_MUX_2_GND,
    ADS1115_MUX_3_GND,
} ads1115_mux_t;

typedef enum {  // full-scale resolution options
    ADS1115_FSR_6_144 = 0,
    ADS1115_FSR_4_096,
    ADS1115_FSR_2_048,
    ADS1115_FSR_1_024,
    ADS1115_FSR_0_512,
    ADS1115_FSR_0_256,
} ads1115_fsr_t;

typedef enum {  // samples per second
    ADS1115_SPS_8 = 0,
    ADS1115_SPS_16,
    ADS1115_SPS_32,
    ADS1115_SPS_64,
    ADS1115_SPS_128,
    ADS1115_SPS_250,
    ADS1115_SPS_475,
    ADS1115_SPS_860
} ads1115_sps_t;

typedef enum {
    ADS1115_MODE_CONTINUOUS = 0,
    ADS1115_MODE_SINGLE
} ads1115_mode_t;

typedef union {  // configuration register
    struct {
        uint16_t COMP_QUE : 2;   // bits 0..  1  Comparator queue and disable
        uint16_t COMP_LAT : 1;   // bit  2       Latching Comparator
        uint16_t COMP_POL : 1;   // bit  3       Comparator Polarity
        uint16_t COMP_MODE : 1;  // bit  4       Comparator Mode
        uint16_t DR : 3;         // bits 5..  7  Data rate
        uint16_t MODE : 1;       // bit  8       Device operating mode
        uint16_t
            PGA : 3;  // bits 9..  11 Programmable gain amplifier configuration
        uint16_t MUX : 3;  // bits 12.. 14 Input multiplexer configuration
        uint16_t OS : 1;   // bit  15      Operational status or single-shot
                           // conversion start
    } bit;
    uint16_t reg;
} ADS1115_CONFIG_REGISTER_Type;

typedef struct {
    ADS1115_CONFIG_REGISTER_Type config;
    i2c_port_t i2c_port;
    uint8_t address;
    ads1115_register_addresses_t last_reg;  // save last accessed register
    bool changed;       // save if a value was changed or not
    int32_t max_ticks;  // maximum wait ticks for i2c bus
} ads1115_t;

// initialize device
ads1115_t ads1115_config(i2c_port_t i2c_port,
                         uint8_t address);  // set up configuration

void ads1115_set_mux(ads1115_t* ads, ads1115_mux_t mux);     // set multiplexer
void ads1115_set_pga(ads1115_t* ads, ads1115_fsr_t fsr);     // set fsr
void ads1115_set_mode(ads1115_t* ads, ads1115_mode_t mode);  // set read mode
void ads1115_set_sps(ads1115_t* ads, ads1115_sps_t sps);  // set sampling speed
void ads1115_set_max_ticks(
    ads1115_t* ads, int32_t max_ticks);  // maximum wait ticks for i2c bus

esp_err_t ads1115_get_raw(ads1115_t* ads, int16_t* raw);  // get voltage in bits
double ads1115_get_voltage(ads1115_t* ads);  // get voltage in volts

#endif  // ifdef ADS1115_H