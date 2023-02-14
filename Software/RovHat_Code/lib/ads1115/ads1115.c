#include "ads1115.h"


void ads1115_init(i2c_inst_t *I2C_inst,uint8_t address,ads1115_ADC_t *adc){
    adc->I2C_inst = I2C_inst;
    adc->address = address;
}
