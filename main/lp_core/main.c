#include <esp_err.h>
#include <stdint.h>

#include "ads1115.h"

int16_t raw_reading = 1234;
esp_err_t err = 1234;

int main(void) {
    ads1115_t ads =
        ads1115_config(0, 0x48);  // The i2c_port_t is ignored at this moment by
                                  // lp_core_i2c_master_ operations

    ads1115_set_mode(&ads, ADS1115_MODE_SINGLE);
    ads1115_set_pga(&ads, ADS1115_FSR_1_024);
    ads1115_set_sps(&ads, ADS1115_SPS_860);
    ads1115_set_mux(&ads, ADS1115_MUX_0_GND);

    err = ads1115_get_raw(&ads, &raw_reading);
}
