#include <stdio.h>
#include "pico/stdlib.h"
#include "ads1115.h"

#define I2C_PORT i2c0
#define I2C_FREQ 400000
#define ADS1115_I2C_ADDR 0x48
const uint8_t SDA_PIN = 0;
const uint8_t SCL_PIN = 1;
const uint8_t TX_PIN = 8;
const uint8_t RX_PIN = 9;


struct ads1115_adc adc;




int main() {

    const uint led_pin = 25;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    stdio_init_all();

    // Initialise I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Initialise ADC
    ads1115_init(I2C_PORT, ADS1115_I2C_ADDR, &adc);


    // Loop forever
    while (true) {

        // Blink LED
        printf("Blinking!\r\n");
        gpio_put(led_pin, true);
        sleep_ms(1000);
        gpio_put(led_pin, false);
        sleep_ms(1000);
    }
}