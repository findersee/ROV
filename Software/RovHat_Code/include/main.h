#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "Dshot.pio.h"

#include <ads1115.h> //ADS1115 Driver
#include <dshot.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "tasks.h"

#define I2C_PORT i2c0
#define I2C_FREQ 400000
#define ADS1115_I2C_ADDR 0x48

const uint8_t SDA_PIN = 0;
const uint8_t SCL_PIN = 1;

const uint8_t TX_PIN = 8;
const uint8_t RX_PIN = 9;

const uint8_t Prop_Right = 22;
const uint8_t Prop_Left = 26;

const uint8_t Depth_Right = 27;
const uint8_t Depth_Left = 28;

const uint8_t led_pin = 25;

const uint8_t NMOS_1 = 19;
const uint8_t NMOS_2 = 18;
const uint8_t NMOS_3 = 17;
const uint8_t NMOS_4 = 16;

const uint8_t PWM_1 = 6;
const uint8_t PWM_2 = 7;

struct ads1115_adc adc;

PIO pio = pio0;
uint sm[4];

static int received_msgs = 0;



// UART Initialization
void UART_RX();

void UART_INIT();

#define UART_ID uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

char msg_buf[33];
uint8_t buf_cnt;
bool buf_rdy = true;


// FreeRtos Stuff
typedef struct Ctrl_Message{
    char message[33];
    uint received;
}Ctrl_Message;

typedef struct ADC_Message{
    float voltage;
    uint8_t channel;
}ADC_Message;

typedef struct motorMessage{
    uint16_t Prop_Right;
    uint16_t Prop_Left;
    uint16_t Depth_Right;
    uint16_t Depth_Left;
}motorMessage;

void freeRTOS_setup();

static SemaphoreHandle_t I2C_mutex;

static QueueHandle_t msg_queue;
static QueueHandle_t ADC_queue;
static QueueHandle_t motor_queue;

void ADC_task();
void motorControl_task();
#endif