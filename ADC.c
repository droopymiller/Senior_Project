/**
 * ADC.c
 */

static uint8_t conversion_status = 0;
static uint16_t ADC_data;

void init_ADC(){
    ADC10CTL0 = SREF_0 | ADC10SHT_2 | ADC10SR | ADC10ON | ADC10IE;
    ADC10CTL1 = INCH0 | SHS_0 | ADC10DIV_0 | ADC10SSEL_0 | CONSEQ_0;
}

void start_ADC_conversion(){
    ADC10CTL0 |= ENC | ADC10SC;   // Start conversion
    conversion_status = 0;
}

// Return 1 when conversion has been completed, 0 while waiting for conversion to finish
uint8_t check_ADC_conversion(){
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
    conversion_status = 1; // Set conversion status to complete
    __enable_interrupt();
}

