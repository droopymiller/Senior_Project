/**
 * TimerA.c
 */

#include "TimerA.h"

void init_TimerA(){
    TA0CTL = TASSEL_2 | ID_0 | MC_0 | TAIE; // SMCLK, DIV=0, STOP MODE, INTERRUPTS EN
    TA0R = 0x0000; // Ensure TimerA register is set to 0
    TA0CCR0 = 4166; // Set to measure peak of 60Hz sine wave
    TA0CCTL0 = CM_0 | CCIE; // Compare MODE, Interrupt enabled
    __enable_interrupt();
}

void start_TimerA(){
    TA0R = 0x0000; // Ensure TimerA register is set to 0
    TA0CTL = (TA0CTL & ~TA0_MODE_MASK) | MC_1; // Set timer to up mode
}
