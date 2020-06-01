/**
 * LP3947.h
 */

#include <msp430.h>
#include <stdint.h>

//******************************************************************************
// Battery Charger Defines *****************************************************
//******************************************************************************

// Using CHGR in USB Mode

#define CHGR_ADDR           0x47

#define EOC_SEL_0            0x01        // 0.1C
#define EOC_SEL_1            0x02        // 0.15C
#define EOC_SEL_2            0x03        // 0.2C

#define CHGR_REG_1           0x00
#define CHGR_REG_2           0x01
#define CHGR_REG_3           0x02

#define CHGR_VOLT            0x10         // Set this bit for 4.2V, else 4.1V

#define CHGR_RST_TIME        (5*60*60)    // Number of secs in 5 hrs (uint16_t)

//******************************************************************************
// Battery Charger Functions****************************************************
//******************************************************************************

// Init charger GPIO and EN switch before init_battery_chgr()

void init_battery_chgr();

void set_chgr_current(uint8_t index);

void init_chgr_gpio();

void chgr_switch_en();

void chgr_switch_dis();
