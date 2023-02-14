#ifndef _REGISTERS_H_
#define _REGISTERS_H_

//00 : Conversion register
//01 : Config register
//10 : Lo_thresh register
//11 : Hi_thresh register
static const uint8_t ADS1115_CONV_REG = 0x00;
static const uint8_t ADS1115_CONF_REG = 0x01;
static const uint8_t ADS1115_LO_REG = 0x02;
static const uint8_t ADS1115_HI_REG = 0x03;

static const uint16_t ADS1115_STATUS_MASK = 0x8000;
enum ADS1115_STATUS_t {
    STATUS_START = 0x01,
    STATUS_BUSY = 0x00
};

static const uint16_t ADS1115_MUX_MASK = 0x7000;
enum ADS1115_MUX_t{
    MUX_DIFF_1 = 0x0000,
    MUX_DIFF_2 = 0x1000,
    MUX_DIFF_3 = 0x2000,
    MUX_DIFF_4 = 0x3000,
    MUX_SINGLE_1 = 0x4000,
    MUX_SINGLE_2 = 0x5000,
    MUX_SINGLE_3 = 0x6000,
    MUX_SINGLE_4 = 0x7000
};

#endif