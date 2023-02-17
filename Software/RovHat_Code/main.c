#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "Dshot.pio.h"

#include <ads1115.h> //ADS1115 Driver
/*
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
*/

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
uint sm = 0;

int init()
{
    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    stdio_init_all();

    // Initialise I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    //gpio_pull_up(SDA_PIN);
    //gpio_pull_up(SCL_PIN);
    gpio_set_function(TX_PIN,GPIO_FUNC_UART);
    gpio_set_function(RX_PIN,GPIO_FUNC_UART);

    gpio_set_function(NMOS_1,GPIO_FUNC_PWM);
    gpio_set_function(NMOS_2,GPIO_FUNC_PWM);
    gpio_set_function(NMOS_3,GPIO_FUNC_PWM);
    gpio_set_function(NMOS_4,GPIO_FUNC_PWM);

    pwm_config Config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&Config,4);
    pwm_config_set_wrap(&Config,2500);
    pwm_init((pwm_gpio_to_slice_num(NMOS_1)),&Config,true);
    pwm_init((pwm_gpio_to_slice_num(NMOS_2)),&Config,true);
    pwm_init((pwm_gpio_to_slice_num(NMOS_3)),&Config,true);
    pwm_init((pwm_gpio_to_slice_num(NMOS_4)),&Config,true);
    pwm_set_gpio_level(NMOS_4,(2500/8));
    pwm_set_gpio_level(NMOS_3,(2500/4));
    pwm_set_gpio_level(NMOS_2,(2500/2));
    pwm_set_gpio_level(NMOS_1,(2500));
    // Initialise ADC
    ads1115_init(I2C_PORT, ADS1115_I2C_ADDR, &adc);

    sm = pio_claim_unused_sm(pio,true);

    uint offset = pio_add_program(pio,&Dshot_program);

    float div = 31.25f; //(float)clock_get_hz(clk_sys) / 40000.f;

    Dshot_program_init(pio,sm,offset,Prop_Right,div);

    pio_sm_set_enabled(pio,sm,true);
    //pio_sm_put_blocking(pio,sm,0x5555);
}


int main() {

    init();
    uint16_t value;
    // Loop forever
    while (true) {

        
        // Blink LED
        printf("Blinking!\r\n");
        gpio_put(led_pin, true);
        sleep_ms(1000);
        gpio_put(led_pin, false);
        pio_sm_put_blocking(pio,sm,0x0000FE55);

        ads1115_readADC(&adc,&value);
        printf("ADC value:%u Converted:%f\r\n",value,((float)clock_get_hz(clk_sys)));
        sleep_ms(1000);
    }
}