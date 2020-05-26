/**
 * TimerA.c
 */

#include "TimerA.h"

void init_TimerA0(){
    TA0CTL = TASSEL_2 | ID_0 | MC_0; // SMCLK, DIV=0, STOP MODE
    TA0R = 0x0000; // Ensure TimerA register is set to 0
    TA0CCR0 = 4166; // Set to measure peak of 60Hz sine wave
    TA0CCTL0 = CM_0 | CCIE; // Compare MODE, Interrupt enabled
    __enable_interrupt();
}

void start_TimerA0(){
    TA0R = 0x0000; // Ensure TimerA register is set to 0
    TA0CTL = (TA0CTL & ~TA0_MODE_MASK) | MC_1; // Set timer to up mode
}

void init_TimerA1(){
   TA1CTL = TASSEL_1 | ID_0 | MC_1; // ACLK, DIV=0, UP MODE
   TA1CCR0 = 10638;     // Set CCR0 for 1 sec interrupts, "calibrated" at room temp on MSP-EXP430G2ET
   TA1CCTL0 = CM_0 | CCIE; // Compare MODE, Interrupt enabled
   __enable_interrupt();
}
