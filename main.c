#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "I2C.h"
#include "ADC.h"
#include "TimerA.h"
#include "DAC.h"
#include "LP3947.h"


/**
 * main.c
 */


#define debug true


//******************************************************************************
// I2C Stuff********************************************************************
//******************************************************************************

/* CMD_TYPE_X_SLAVE are example commands the master sends to the slave.
 * The slave will send example SlaveTypeX buffers in response.
 *
 * CMD_TYPE_X_MASTER are example commands the master sends to the slave.
 * The slave will initialize itself to receive MasterTypeX example buffers.
 * */

#define CMD_TYPE_0_SLAVE      0
#define CMD_TYPE_1_SLAVE      1
#define CMD_TYPE_2_SLAVE      2



#define CMD_TYPE_1_MASTER       4
#define CMD_TYPE_2_MASTER       5

#define TYPE_1_LENGTH           2
#define TYPE_2_LENGTH           6


/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

// uint8_t MasterType2 [TYPE_2_LENGTH] = {'F', '4', '1', '9', '2', 'B'};




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

volatile bool timerA1_IFG = false;
volatile bool timerA0_IFG = false;

//******************************************************************************
// Current Measurement and Reactions********************************************
//******************************************************************************
#define DC_OFFSET       1650  // DC Offset should be 1/2 of Vref [mV]
#define TRANSCONDUCTANCE 2    // 5mOhm * 100V/V = 0.5 Ohm, 1/0.5 = 2 [I/V]
#define VREF            3300  // ADC reference voltage [mV]
#define SS_CURRENT      120   // Steady State Current of 5V supply [mA] w/ 20% tol.

int16_t current_conversion(uint16_t input_current){

    uint32_t input_current_mV;    // Input current [mV]
    int16_t current_mA;    // Input current [mA]

    input_current_mV = (uint32_t)input_current * VREF;
    input_current_mV = input_current_mV / 1023; // Divide by ADC resolution - 1

    current_mA = TRANSCONDUCTANCE * ((int16_t)input_current_mV - DC_OFFSET);
    current_mA = abs(current_mA);

    current_mA = (int16_t)( ((int32_t)current_mA * 10) / 14); // Calc RMS value

    return current_mA;
}

uint16_t calc_target_mV(int16_t input_current_mA){
    if(input_current_mA > 1000){
        return 15000;
    }
    return 29300; // For now just return the max target voltage
}

uint8_t calc_chgr_current_idx(int16_t input_current_mA){
    // This function returns the index value for the appropriate charging current
    // The battery charger should be disabled if 15 is returned

    input_current_mA = input_current_mA - SS_CURRENT;

    // If input current is less than smallest charging current, don't charge
    if(input_current_mA < 100){
        return 15;
    }
    else{
        input_current_mA = (input_current_mA-100)/50; // Divide into size of chg incr.
        if(input_current_mA > 13){
            return 13;
        }
        else{
            return (uint8_t)input_current_mA;
        }

    }
}

//******************************************************************************
// TimerA1 *********************************************************************
//******************************************************************************

// Seconds variable used to keep track of time
// This is not a reliable way to keep track of time due to oscillator variation
volatile uint16_t seconds = 0; // Variable to keep track of time

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
// Main*************************************************************************
//******************************************************************************

int main(void)
{
    volatile uint16_t input_current;       // Raw input current
    volatile int16_t input_current_mA;     // Input current [mA] (peak value, not RMS)

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
    // Do not initialize external hardware before this delay
    delay_ms(20);
    init_DAC();

    chgr_switch_en();    // Turn on switch to battery charger
    delay_ms(2);         // Wait for charger to turn on
    init_battery_chgr(); // Init battery charger


    // State machine loop
    while(1){

        __enable_interrupt();

        if(timerA0_IFG){
            while(!ADC_conversion_complete());     // Wait for conversion to finish
            input_current = get_ADC_data();        // Store data once conversion is finished
            input_current_mA = current_conversion(input_current);
            timerA0_IFG = false;
        }

        if(timerA1_IFG){
            if(seconds == CHGR_RST_TIME){
                chgr_switch_dis();      // Turn off charger
                /* Set DAC voltage here */
            }
            else if(seconds > CHGR_RST_TIME){
                chgr_switch_en();       // Turn on Charger
                delay_ms(2);            // Wait for chip to be "on"
                set_target_mV(calc_target_mV(input_current_mA));
                set_chgr_current(calc_chgr_current_idx(input_current_mA));

                seconds = 0;            // Reset seconds variable after 5 hrs
            }
            else{
                set_target_mV(calc_target_mV(input_current_mA));
                set_chgr_current(calc_chgr_current_idx(input_current_mA));
            }
            timerA1_IFG = false;               // Clear flag
        }

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
    if(P1IFG & BIT1){
        start_TimerA0();
//        NS = I_MEAS;   // Update State
        // #ifdef debug
        // P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
        // #endif
        P1IFG &= ~BIT1; // Clear interrupt flag
    }
    if(P1IFG & BIT2){
        start_TimerA0();
//        NS = I_MEAS;   // Update State
        // #ifdef debug
        // P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
        // #endif
        P1IFG &= ~BIT2; // Clear interrupt flag
    }

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
    start_ADC_conversion();                    // Start ADC Conversion
    TA0CTL = (TA0CTL & ~TA0_MODE_MASK) | MC_0; // Stop timer
    TA0R = 0x0000;                             // Reset TA0R register
    timerA0_IFG = true;                        // Set flag

    // #ifdef debug
    // P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
    // #endif
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
    seconds++; // Increment seconds
    timerA1_IFG = true; // Set flag for state machine

    #ifdef debug
    P1OUT ^= BIT5; // Toggle P1.5 to indicate rising edge "received"
    #endif
}
