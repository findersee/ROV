#ifndef _REGISTERS_H_
#define _REGISTERS_H_

//00 : Conversion register
//01 : Config register
//10 : Lo_thresh register
//11 : Hi_thresh register

#define ADS1115_CONV_REG 0x00
#define ADS1115_CONF_REG 0x01
#define ADS1115_LO_REG 0x02
#define ADS1115_HI_REG 0x03

const uint16_t ADS1115_CONFIG_MASK = 0x8000

const uint16_t ADS1115_MUX_MASK = 0x7000