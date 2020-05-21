/**
 * ADC.h
 */

#include <stdbool.h>
#include <stdint.h>
#include <msp430.h>

void init_ADC();

void start_ADC_conversion();

// Return True when conversion has been completed, False while waiting for conversion to finish
bool ADC_conversion_complete();

uint16_t get_ADC_data();
