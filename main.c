#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "I2C.h"
#include "ADC.h"
#include "TimerA.h"


/**
 * main.c
 */


#define debug true

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


//******************************************************************************
// Battery Charger Defines *****************************************************
//******************************************************************************

// Using CHGR in USB Mode

#define CHGR_ADDR           0x47

uint8_t CHG_SEL[14]      =  {0x00,      // 100mA
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

#define EOC_SEL_0            0x01        // 0.1C
#define EOC_SEL_1            0x02        // 0.15C
#define EOC_SEL_2            0x03        // 0.2C

#define CHGR_REG_1           0x00
#define CHGR_REG_2           0x01
#define CHGR_REG_3           0x02

#define CHGR_VOLT            0x10         // Set this bit for 4.2V, else 4.1V


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


//******************************************************************************
// State Machine ***************************************************************
//******************************************************************************

typedef enum CPU_ModeEnum{
    I_MEAS,
    UPDATE_DAC_and_CHGR,
    IDLE
} CPU_Mode;

CPU_Mode PS = IDLE;
CPU_Mode NS = IDLE;

bool timerA1_IFG = false;
bool timerA0_IFG = false;

//******************************************************************************
// Current Measurement *********************************************************
//******************************************************************************
uint16_t input_current;

//******************************************************************************
// TimerA1 *********************************************************************
//******************************************************************************

// Seconds variable used to keep track of time
// This is not a reliable way to keep track of time due to oscillator variation
uint16_t seconds = 0; // Variable to keep track of time

//******************************************************************************
// General *********************************************************************
//******************************************************************************

void init_cs(){
    if (CALBC1_1MHZ==0xFF)                    // If calibration constant erased
    {
      while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = XT2OFF | CALBC1_1MHZ;           // Set DCO
    DCOCTL = CALDCO_1MHZ;
    BCSCTL3 = LFXT1S_2;                       // Use VLO?

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

//******************************************************************************
// Battery Charger *************************************************************
//******************************************************************************

void init_battery_chgr(){
    uint8_t data = CHGR_VOLT;
    I2C_Master_WriteReg(CHGR_ADDR, CHGR_REG_1, &data, 1); // V_bat_4.2V, AC chg at 100mA
    data = CHG_SEL[0];
    I2C_Master_WriteReg(CHGR_ADDR, CHGR_REG_3, &data, 1); // Set charge rate to 100mA for startup

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

//******************************************************************************
// Main*************************************************************************
//******************************************************************************

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    init_cs();
    init_chgr_gpio();
    init_I2C();
    init_ADC();
    init_TimerA0();
    init_TimerA1();
    init_zero_crossing_detection();

    // Temporary: Used for testing zero crossing detection
    P1DIR |= BIT5;
    P1OUT &= ~(BIT5);

    // After MSP430 init, wait 20ms for 3.3V bus to rise to 3.3V
    delay_ms(20);

    // Init DAC
    I2C_Master_WriteReg(DAC_ADDR, DAC_PWR_ON_CONFIG, DAC_VOLT, DAC_DATA_LENGTH);

    chgr_switch_en();    // Turn on switch to battery charger
    init_battery_chgr(); // Init battery charger


    // State machine loop
    while(1){
        switch(PS) {
        case I_MEAS:
            while(!ADC_conversion_complete());     // Wait for conversion to finish
            input_current = get_ADC_data();        // Store data once conversion is finished
            NS = IDLE;                             // Set next state
        default:
            if(timerA1_IFG){
                NS = UPDATE_DAC_and_CHGR;          // Update DAC and CHGR when timer interrupt occurs
                timerA1_IFG = false;               // Clear flag
            }
            else{
                NS = IDLE;                         // Set next state
            }

        }
        PS = NS;                                   // Set present state to next state
    }
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
    __disable_interrupt();
    if(P1IFG & BIT1){
        start_TimerA0();
        NS = I_MEAS;   // Update State
        // #ifdef debug
        // P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
        // #endif
        P1IFG &= ~BIT1; // Clear interrupt flag
    }
    if(P1IFG & BIT2){
        start_TimerA0();
        NS = I_MEAS;   // Update State
        // #ifdef debug
        // P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
        // #endif
        P1IFG &= ~BIT2; // Clear interrupt flag
    }
    __enable_interrupt();

}

//******************************************************************************
// Timer A0 Interrupt***********************************************************
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    __disable_interrupt();
    start_ADC_conversion();                    // Start ADC Conversion
    TA0CTL = (TA0CTL & ~TA0_MODE_MASK) | MC_0; // Stop timer
    TA0R = 0x0000;                             // Reset TA0R register

    // #ifdef debug
    // P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
    // #endif
    __enable_interrupt();
}

//******************************************************************************
// Timer A1 Interrupt***********************************************************
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A1_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    __disable_interrupt();
    seconds++; // Increment seconds
    timerA1_IFG = true; // Set flag for state machine

    #ifdef debug
    P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
    #endif
    __enable_interrupt();
}
