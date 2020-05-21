/**
 * ADC.c
 */
#include "ADC.h"

static bool conversion_status = false;
static uint16_t ADC_data;

void init_ADC(){
    ADC10CTL0 = SREF_0 | ADC10SHT_2 | ADC10SR | ADC10ON | ADC10IE;
    ADC10CTL1 = INCH_0 | SHS_0 | ADC10DIV_0 | ADC10SSEL_0 | CONSEQ_0;
    ADC10AE0 |= BIT0;  // Enable analog input to A0
}

void start_ADC_conversion(){
    ADC10CTL0 |= ENC | ADC10SC;   // Start conversion
    conversion_status = false;
}

// Return True when conversion has been completed, False while waiting for conversion to finish
bool ADC_conversion_complete(){
    return conversion_status;
}

uint16_t get_ADC_data(){
    return ADC_data;
}




//******************************************************************************
// I2C Interrupt For Start, Restart, Nack, Stop ********************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
    __disable_interrupt();
    // ADC10IFG is automatically reset when interrupt request is accepted
    ADC_data = ADC10MEM; // Transfer memory contents to a variable
    conversion_status = true; // Set conversion status to complete
    __enable_interrupt();
}

