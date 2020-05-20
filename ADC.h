/**
 * ADC.h
 */


void init_ADC();

void start_ADC_conversion();

// Return 1 when conversion has been completed, 0 while waiting for conversion to finish
uint8_t check_ADC_conversion();

uint16_t get_ADC_data();
