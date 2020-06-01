/**
 * DAC.h
 */

#include <msp430.h>
#include <stdint.h>


//******************************************************************************
// DAC Defines *****************************************************************
//******************************************************************************

#define DAC_ADDR            0x63

// Commands
#define DAC_WRITE_ALL       (0b011 << 5)
#define DAC_WRITE_VOL       (0b010 << 5)

// Configuration
#define DAC_VDD_UNBUF       (0b00 << 3)

#define DAC_NOT_PWR_DWN     (0b00 << 1)

#define DAC_GAIN_1X         (0b0)
#define DAC_GAIN_2X         (0b1)



#define DAC_PWR_ON_CONFIG     (DAC_WRITE_ALL | DAC_VDD_UNBUF | DAC_NOT_PWR_DWN | DAC_GAIN_1X)

#define DAC_VOL_WR_CONFIG     (DAC_WRITE_VOL | DAC_VDD_UNBUF | DAC_NOT_PWR_DWN | DAC_GAIN_1X)

#define DAC_MAX_VOLT          0x0258       // 600 in hex
#define MAX_TARGET_VOLT       29300        // mV
/* Max Volt = 30V-0.7V * V_div = 1.93399
 * 30V = Reverse Standoff voltage of TVS Diode
 * 0.7V = Diode Drop
 * V_div = voltage divider ratio (R17, R18, R19)
 */

#define DAC_DATA_LENGTH         2



//******************************************************************************
// DAC Functions****************************************************************
//******************************************************************************

void init_DAC();

void set_DAC_mV();

void set_target_mV(uint16_t target_mV);
