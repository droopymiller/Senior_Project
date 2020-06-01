/**
 * DAC.c
 */

#include "DAC.h"
#include "I2C.h"

void init_DAC(){
    // Init DAC
    const uint8_t DAC_VOLT [DAC_DATA_LENGTH] = {0x00FF & (DAC_MAX_VOLT >> 2), 0x00FF & (DAC_MAX_VOLT << 6)};
    I2C_Master_WriteReg(DAC_ADDR, DAC_PWR_ON_CONFIG, (uint8_t *)DAC_VOLT, DAC_DATA_LENGTH);
}

void set_DAC_mV(uint16_t DAC_mV){
    uint16_t DAC_DATA;

    if(DAC_mV > 3300){  // Ensure data is within acceptable range
        return;
    }
    else{
        DAC_DATA = (uint16_t)( ( ( uint32_t)DAC_mV * 1023) / 3300);
        uint8_t DATA[DAC_DATA_LENGTH] = {0x00FF & (DAC_DATA >> 2), 0x00FF & (DAC_DATA << 6)};
        I2C_Master_WriteReg(DAC_ADDR, DAC_VOL_WR_CONFIG, (uint8_t *)DATA, DAC_DATA_LENGTH);
    }
}

void set_target_mV(uint16_t target_mV){
    uint16_t target_mV_scaled;

    // Voltage divider ratio is 20/303
    target_mV_scaled = (uint16_t)( ( (uint32_t)target_mV * 20) / 303);
    set_DAC_mV(target_mV_scaled);
}
