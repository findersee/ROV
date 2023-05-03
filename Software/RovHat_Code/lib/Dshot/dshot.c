#include "dshot.h"
#include <stdio.h>

uint32_t dshot_parse_throttle(uint16_t *value,bool telemetry){    

    uint32_t msg=0;
    volatile uint8_t crc = 0;
    volatile uint16_t Data = *value+48;
    uint16_t csdata = (uint16_t)((Data<<1)|telemetry);
    for (int i = 0; i < 3; i++) {
        crc ^=  csdata;   // xor data by nibbles
        csdata >>= 4;
    }    
    crc &= 0xf;
    msg = (Data<<5)|(telemetry<<4)|crc;

    return msg<<16;
}

uint32_t dshot_parse_cmd(uint8_t cmd,bool telemetry){
    
    uint32_t msg=0;
    uint8_t crc = 0;
    //printf("Telemetry: %d ",telemetry);
    uint16_t csdata = (uint16_t)((cmd<<1)|telemetry);
    //printf("csdata: %x ",csdata);
    for (int i = 0; i < 3; i++) {
        crc ^=  csdata;   // xor data by nibbles
        csdata >>= 4;
    }    
    crc &= 0xf;
    //printf("csdata: %x crc: %x \r\n",csdata,crc);
    msg = (cmd<<5)|(telemetry<<4)|crc;

    return msg<<16;
}