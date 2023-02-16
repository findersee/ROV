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

static const uint16_t ADS1115_MODE_MASK = 0x0100;
enum ADS1115_MODE_t{
    MODE_CONT = 0x0000,
    SINGLE = 0x0100,
};

static const uint16_t ADS1115_PGA_MASK = 0x0E00;
enum ADS1115_PGA_t{
    PGA_6144 = 0x000,
    PGA_4096 = 0x200,
    PGA_2048 = 0x400,
    PGA_1024 = 0x600,
    PGA_512 = 0x800,
    PGA_256 = 0xA00
};

static const uint16_t ADS1115_DR_MASK = 0x00E0;
enum ADS1115_DR_t{
    SPS_8 = 0x00,
    SPS_16 = 0x20,
    SPS_32 = 0x40,
    SPS_64 = 0x60,
    SPS_128 = 0x80,
    SPS_250 = 0xA0,
    SPS_475 = 0xC0,
    SPS_860 = 0xE0
};

#endif