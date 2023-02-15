#include "ads1115.h"


void ads1115_init(i2c_inst_t *I2C_inst,uint8_t address,ads1115_ADC_t *adc){
    adc->I2C_inst = I2C_inst;
    adc->address = address;
}

void ads1115_readADC(ads1115_ADC_t *adc, uint16_t *value){
    i2c_write_blocking(adc->I2C_inst,adc->address,&ADS1115_CONV_REG,1,true);
    uint8_t readVal[2];
    i2c_read_blocking(adc->I2C_inst,adc->address,(uint8_t *)&readVal,2,false);
    *value = (readVal[0] << 8) | readVal[1];
}

void ads1115_set_MUX(ads1115_ADC_t *adc,enum ADS1115_MUX_t MUX){
    adc->config &= ~ADS1115_MUX_MASK;
    adc->config |= MUX;
}

void ads1115_set_MODE(ads1115_ADC_t *adc,enum ADS1115_MODE_t MODE){
    adc->config &= ~ADS1115_MODE_MASK; 
    adc->config |= MODE;
}

void ads1115_set_DR(ads1115_ADC_t *adc,enum ADS1115_DR_t RATE){
    adc->config &= ~ADS1115_DR_MASK;
    adc->config |= RATE;
}

void ads1115_set_PGA(ads1115_ADC_t *adc,enum ADS1115_PGA_t PGA){
    adc->config &= ~ADS1115_PGA_MASK;
    adc->config |= PGA;
}