#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>
#include "I2C.h"


/**
 * main.c
 */


//******************************************************************************
// DAC Defines *****************************************************************
//******************************************************************************

#define DAC_ADDR 0x63

// Commands
#define DAC_WRITE_ALL       (0b011 << 5)
#define DAC_WRITE_VOL       (0b010 << 5)

// Configuration
#define DAC_VDD_UNBUF       (0b00 << 3)

#define DAC_NOT_PWR_DWN     (0b00 << 1)

#define DAC_GAIN_1X         (0b0)
#define DAC_GAIN_2X         (0b1)




/* CMD_TYPE_X_SLAVE are example commands the master sends to the slave.
 * The slave will send example SlaveTypeX buffers in response.
 *
 * CMD_TYPE_X_MASTER are example commands the master sends to the slave.
 * The slave will initialize itself to receive MasterTypeX example buffers.
 * */

#define CMD_TYPE_0_SLAVE      0
#define CMD_TYPE_1_SLAVE      1
#define CMD_TYPE_2_SLAVE      2

#define DAC_PWR_ON_CONFIG     DAC_WRITE_ALL | DAC_VDD_UNBUF | DAC_NOT_PWR_DWN | DAC_GAIN_1X
#define DAC_MAX_VOLT          0x0258       // 600 in hex
/* Max Volt = 30V-0.7V * V_div = 1.93399
 * 30V = Reverse Standoff voltage of TVS Diode
 * 0.7V = Diode Drop
 * V_div = voltage divider ratio (R17, R18, R19)
 */
#define CMD_TYPE_1_MASTER      4
#define CMD_TYPE_2_MASTER      5

#define DAC_DATA_LENGTH  2
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   6


/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

// uint8_t MasterType2 [TYPE_2_LENGTH] = {'F', '4', '1', '9', '2', 'B'};

uint8_t DAC_VOLT [DAC_DATA_LENGTH] = {0x00FF & (DAC_MAX_VOLT >> 2), 0x00FF & (DAC_MAX_VOLT << 6)};




void init_cs(){
    if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
    {
      while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
    DCOCTL = CALDCO_1MHZ;
}

void delay_ms(uint8_t ms){
    while(ms){
        __delay_cycles(1000);  // 1000 cycles for 1MHz
        ms--;
    }
}

void init_zero_crossing_detection(){
    P1DIR &= ~(BIT1 | BIT2);
    P1IES &= ~(BIT1); // Set P1.1 to interrupt on rising edge
    P1IES |= BIT2;    // Set P1.2 to interrupt on falling edge
    P1IFG = 0;        // Clear interrupt flag in case it was set when configuring interrupt edge
    P1IE = BIT1 | BIT2;         // Enable interrupts
    __enable_interrupt();
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    init_cs();
    init_I2C();
    init_zero_crossing_detection();

    // Temporary: Keep charging switch off
    P2DIR |= BIT0;
    P2OUT &= ~(BIT0);

    // Temporary: Used for testing zero crossing detection
    P1DIR |= BIT5;
    P1OUT &= ~(BIT5);

    // After MSP430 init, wait 20ms for 3.3V bus to rise to 3.3V
    delay_ms(20);

    I2C_Master_WriteReg(DAC_ADDR, DAC_PWR_ON_CONFIG, DAC_VOLT, DAC_DATA_LENGTH);

    while(1);
}

//******************************************************************************
// Zero Crossing Detection Interrupt********************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) PORT1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if(P1IFG & BIT1){
        P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
        P1IFG &= ~BIT1; // Clear interrupt flag
    }
    if(P1IFG & BIT2){
        P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
        P1IFG &= ~BIT2; // Clear interrupt flag
    }


}
