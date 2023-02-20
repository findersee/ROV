#include "dshot.h"
#include <stdio.h>

uint32_t dshot_parse_throttle(uint16_t *value,bool telemetry){    

    uint32_t msg=0;
    uint8_t crc = 0;
    uint16_t csdata = ((*value<<1)|telemetry);
    for (int i = 0; i < 3; i++) {
        crc ^=  csdata;   // xor data by nibbles
        csdata >>= 4;
    }    
    crc &= 0xf;
    msg = (*value<<5)|(telemetry<<4)|crc;

    return msg;
}

uint32_t dshot_parse_cmd(uint16_t cmd,bool telemetry){
    
    uint32_t msg=0;
    uint8_t crc = 0;
    //printf("Telemetry: %d ",telemetry);
    uint16_t csdata = ((cmd<<1)|telemetry);
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