#ifndef _ADS1115_H_
#define _ADS1115_H_

#include "pico.h"
#include "hardware/i2c.h"
#include "registers.h"

typedef struct ads1115_ADC {
	i2c_inst_t *I2C_inst;
	uint8_t address;
	uint16_t config;
} ads1115_ADC_t;


void ads1115_init(i2c_inst_t *port, uint8_t address, ads1115_ADC_t *adc);

void ads1115_readADC(ads1115_ADC_t *adc,uint16_t *adc_value);

void ads1115_readConfig(ads1115_ADC_t *adc);

void ads1115_writeConfig(ads1115_ADC_t *adc);

void ads1115_set_MUX(ads1115_ADC_t *adc,enum ADS1115_MUX_t MUX);


#endif