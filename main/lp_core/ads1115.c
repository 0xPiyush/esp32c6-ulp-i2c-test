
#include "ads1115.h"

static esp_err_t ads1115_write_register(ads1115_t* ads,
                                        ads1115_register_addresses_t reg,
                                        uint16_t data) {
    uint8_t buf[3] = {
        reg,
        (uint8_t)(data >> 8),   // MSB
        (uint8_t)(data & 0xFF)  // LSB
    };

    esp_err_t ret = lp_core_i2c_master_write_to_device(
        ads->i2c_port, ads->address, buf, sizeof(buf), ads->max_ticks);

    ads->last_reg = reg;  // change the internally saved register
    return ret;
}

static esp_err_t ads1115_read_register(ads1115_t* ads,
                                       ads1115_register_addresses_t reg,
                                       uint8_t* data, uint8_t len) {
    esp_err_t ret;

    if (ads->last_reg !=
        reg) {  // if we're not on the correct register, change it
        uint8_t buf[1] = {reg};
        ret = lp_core_i2c_master_write_to_device(
            ads->i2c_port, ads->address, buf, sizeof(buf),
            ads->max_ticks);  // write the register address to the device
        if (ret != ESP_OK) {
            return ret;  // if we could not write, return error
        }
        ads->last_reg = reg;
    }

    ret = lp_core_i2c_master_read_from_device(
        ads->i2c_port, ads->address, data, len,
        ads->max_ticks);  // read the data from the device
    return ret;
}

ads1115_t ads1115_config(i2c_port_t i2c_port, uint8_t address) {
    ads1115_t ads;          // setup configuration with default values
    ads.config.bit.OS = 1;  // always start conversion
    ads.config.bit.MUX = ADS1115_MUX_0_GND;
    ads.config.bit.PGA = ADS1115_FSR_4_096;
    ads.config.bit.MODE = ADS1115_MODE_SINGLE;
    ads.config.bit.DR = ADS1115_SPS_64;
    ads.config.bit.COMP_MODE = 0;
    ads.config.bit.COMP_POL = 0;
    ads.config.bit.COMP_LAT = 0;
    ads.config.bit.COMP_QUE = 0b11;

    ads.i2c_port = i2c_port;                   // save i2c port
    ads.address = address;                     // save i2c address
    ads.last_reg = ADS1115_MAX_REGISTER_ADDR;  // say that we accessed invalid
                                               // register last
    ads.changed = 1;     // say we changed the configuration
    ads.max_ticks = -1;  // default, wait forever for i2c bus
    return ads;          // return the completed configuration
}

void ads1115_set_mux(ads1115_t* ads, ads1115_mux_t mux) {
    ads->config.bit.MUX = mux;
    ads->changed = 1;
}

void ads1115_set_pga(ads1115_t* ads, ads1115_fsr_t fsr) {
    ads->config.bit.PGA = fsr;
    ads->changed = 1;
}

void ads1115_set_mode(ads1115_t* ads, ads1115_mode_t mode) {
    ads->config.bit.MODE = mode;
    ads->changed = 1;
}

void ads1115_set_sps(ads1115_t* ads, ads1115_sps_t sps) {
    ads->config.bit.DR = sps;
    ads->changed = 1;
}

void ads1115_set_max_ticks(ads1115_t* ads, int32_t max_ticks) {
    ads->max_ticks = max_ticks;
}

esp_err_t ads1115_get_raw(ads1115_t* ads, int16_t* raw) {
    const uint16_t OS_MASK = 1u << 15; /* bit 15 in config reg */
    esp_err_t err;

    /* --------------------------------------------------------------------
     * 1)  Start a conversion if we are in single-shot mode
     *     or the application changed any ADS1115 setting.
     * ------------------------------------------------------------------ */
    if (ads->config.bit.MODE == ADS1115_MODE_SINGLE || ads->changed) {
        uint16_t cfg = ads->config.reg | OS_MASK; /* OS = 1 → start */
        err = ads1115_write_register(ads, ADS1115_CONFIG_REGISTER_ADDR, cfg);
        if (err != ESP_OK) {
            return err;
        }
        ads->changed = false;
    }

    /* --------------------------------------------------------------------
     * 2)  Wait until the conversion is done.
     *     (OS goes low while busy and returns high when ready.)
     * ------------------------------------------------------------------ */
    uint8_t cfg_buf[2];
    uint16_t cfg_rd;

    do {
        err = ads1115_read_register(ads, ADS1115_CONFIG_REGISTER_ADDR, cfg_buf,
                                    sizeof cfg_buf);
        if (err != ESP_OK) {
            return err;
        }
        cfg_rd = ((uint16_t)cfg_buf[0] << 8) | cfg_buf[1];
    } while ((cfg_rd & OS_MASK) == 0); /* 0 → still converting */

    /* --------------------------------------------------------------------
     * 3)  Read the freshly-finished conversion result.
     * ------------------------------------------------------------------ */
    uint8_t data[2];
    err = ads1115_read_register(ads, ADS1115_CONVERSION_REGISTER_ADDR, data,
                                sizeof data);
    if (err != ESP_OK) {
        return err;
    }

    *raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    return ESP_OK;
}

double ads1115_get_voltage(ads1115_t* ads) {
    const double fsr[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
    const int16_t bits = (1L << 15) - 1;
    int16_t raw;

    raw = ads1115_get_raw(ads, &raw);
    return (double)raw * fsr[ads->config.bit.PGA] / (double)bits;
}