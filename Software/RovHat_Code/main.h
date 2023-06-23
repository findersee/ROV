#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <string.h>

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
#include "hardware/i2c.h"
#include "hardware/dma.h" 
#include "hardware/timer.h"

#define I2C_PORT i2c0
#define I2C_FREQ 400000
#define ADS1115_I2C_ADDR 0x48
//Pin Definitions
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


int uartTX_DMA_chan;

// ADC stuff
struct ads1115_adc adc;
volatile float voltages[4];

// DShot PIO stuff
PIO pio = pio0;
uint sm[4];
uint32_t sm_data[4];

repeating_timer_t DShot_Timer;

bool Dshot_timer_callback(repeating_timer_t *rt);





static int received_msgs = 0;

void HW_setup();

// UART Stuff
void UART_RX();

void UART_INIT();

#define UART_ID uart1
#define BAUD_RATE 57600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define rxBufSize 64

//__attribute__((aligned(64)))
//static char msg_buf[64];
typedef struct UART_buffer{
    uint8_t head;
    uint8_t tail;
    uint8_t bufSize;
    char data[rxBufSize];
}UART_buffer_t;
UART_buffer_t rxBuffer;




uint8_t bufferSize(UART_buffer_t *buffer);
bool bufferEmpty(UART_buffer_t *buffer);
void bufferTake(UART_buffer_t *buffer,char *dst, uint8_t elements);


uint8_t buf_cnt;
bool buf_rdy = true;

char transmitMsg[30];


// FreeRtos Stuff

bool Auto_Hold_Active = false;

typedef struct ADC_Message{
    float voltage;
    uint8_t channel;
}ADC_Message_t;

typedef struct motorMessage{
    uint16_t Right;
    uint16_t Left;
}motorMessage_t;

typedef struct AutoHoldMessage{
    float Level;
    bool Active;
}AutoHoldMessage_t;


TaskHandle_t uartHandle;

void freeRTOS_setup();

static SemaphoreHandle_t I2C_mutex;
static SemaphoreHandle_t AutoDepth_mutex;

static QueueHandle_t msg_queue;
static QueueHandle_t ADC_queue;
static QueueHandle_t Propulsion_motor_queue;
static QueueHandle_t Depth_motor_queue;

static QueueHandle_t AutoHold_queue;

void ADC_task(void *pvParameters);
void motorControl_task(void *pvParameters);
void UART_Handler_Task(void *pvParameters);
void Depth_Hold_task(void *pvParameters);
#endif