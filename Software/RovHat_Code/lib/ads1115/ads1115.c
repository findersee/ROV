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


void ads1115_writeConfig(ads1115_ADC_t *adc){

    uint8_t src[3];
    src[0] = ADS1115_CONF_REG;
    src[1] = (uint8_t)(adc->config>>8);
    src[2] = (uint8_t)(adc->config & 0xFF);

    i2c_write_blocking(adc->I2C_inst,adc->address,(uint8_t *)&src,3,false);
}

void ads1115_readConfig(ads1115_ADC_t *adc){

    uint8_t dst[2];

    i2c_write_blocking(adc->I2C_inst,adc->address,&ADS1115_CONF_REG,1,true);

    i2c_read_blocking(adc->I2C_inst,adc->address,(uint8_t *)&dst,2,false);

    adc->config = (dst[0]<<8) | dst[1];
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

float ads1115_convert_raw(ads1115_ADC_t *adc,uint16_t adcValue){
    float fullScale;
    uint16_t pga = adc->config & ADS1115_PGA_MASK;
    uint16_t value = adcValue;
    switch(pga) {
        case PGA_6144:
        fullScale = 6.144;
        break;

        case PGA_4096:
        fullScale = 4.096;
        break;

        case PGA_2048:
        fullScale = 2.048;
        break;

        case PGA_1024:
        fullScale = 1.024;
        break;

        case PGA_512:
        fullScale = 0.512;
        break;

        case PGA_256:
        fullScale = 0.256;
        break;
    }

    float voltOut;

    if(value & 0x8000){
        value = (value ^ 0xffff) + 1;
        voltOut = (float)value * -fullScale / 0x8000;
    }
    else
        voltOut = (float)value * fullScale / 0x8000;
    return voltOut;
}