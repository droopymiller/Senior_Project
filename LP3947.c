/**
 * LP3947.c
 */

#include "LP3947.h"
#include "I2C.h"

const uint8_t CHG_SEL[14] = {0x00,      // 100mA
                             0x01,      // 150mA
                             0x02,      // 200mA
                             0x03,      // 250mA
                             0x04,      // 300mA
                             0x05,      // 350mA
                             0x06,      // 400mA
                             0x07,      // 450mA
                             0x08,      // 500mA
                             0x09,      // 550mA
                             0x0A,      // 600mA
                             0x0B,      // 650mA
                             0x0C,      // 700mA
                             0x0D};     // 750mA


void init_battery_chgr(){
    uint8_t data = CHGR_VOLT;
    I2C_Master_WriteReg(CHGR_ADDR, CHGR_REG_1, &data, 1); // V_bat_4.2V, AC chg at 100mA
    data = CHG_SEL[0];
    I2C_Master_WriteReg(CHGR_ADDR, CHGR_REG_3, &data, 1); // Set charge rate to 100mA for startup

}

void set_chgr_current(uint8_t index){
    uint8_t data = CHG_SEL[index];
    if(index > 13){
        chgr_switch_dis();
        // Disable charging if index invalid
    }
    else{
        chgr_switch_en();
        __delay_cycles(2000); // Wait for 2ms
        I2C_Master_WriteReg(CHGR_ADDR, CHGR_REG_3, &data, 1);
    }
}

void init_chgr_gpio(){
   P2DIR |= BIT0;          // Set to output
   P2OUT &= ~(BIT0);       // Turn pin off
}

void chgr_switch_en(){
   P2OUT |= BIT0;          // Turn pin on
}

void chgr_switch_dis(){
   P2OUT &= ~BIT0;        // Turn pin off
}
