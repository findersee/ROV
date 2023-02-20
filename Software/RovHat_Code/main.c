#include "include/main.h"
#include "include/tasks.h"

void HW_setup()
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

    sm[0] = pio_claim_unused_sm(pio,true);
    sm[1] = pio_claim_unused_sm(pio,true);
    sm[2] = pio_claim_unused_sm(pio,true);  
    sm[3] = pio_claim_unused_sm(pio,true);

    uint offset = pio_add_program(pio,&Dshot_program);

    float div = 31.25f; //(float)clock_get_hz(clk_sys) / 40000.f;

    Dshot_program_init(pio,sm[0],offset,Prop_Right,div);
    Dshot_program_init(pio,sm[1],offset,Prop_Left,div);
    Dshot_program_init(pio,sm[2],offset,Depth_Right,div);
    Dshot_program_init(pio,sm[3],offset,Depth_Left,div);

    for(uint8_t cnt=0;cnt<4;cnt++){
        pio_sm_set_enabled(pio,sm[cnt],true);
    }
    //pio_sm_put_blocking(pio,sm,0x5555);
}

int main() {
    timer_hw->dbgpause = 0;
    HW_setup();
    uint16_t value;
    // Loop forever 
    uint16_t pwr=1047;

    freeRTOS_setup();


    xTaskCreate(ADC_task,"ADC_TASK",36,NULL,1,NULL);

    printf("Starting scheduler...nr");
    vTaskStartScheduler();
    while (true) {

        
        // Blink LED
        //printf("Blinking!\r\n");
        //gpio_put(led_pin, true);
        //sleep_ms(1000);
        //gpio_put(led_pin, false);
        //pio_sm_put_blocking(pio,sm[0],0x0000FE55);
        //pio_sm_put_blocking(pio,sm[1],0x000001AA);
        //pio_sm_put_blocking(pio,sm[2],dshot_parse_throttle(&pwr,false)<<16);
        //pio_sm_put_blocking(pio,sm[3],dshot_parse_cmd(DSHOT_CMD_SPIN_DIRECTION_NORMAL,false)<<16);

        //ads1115_readADC(&adc,&value);
        //printf("Parsed Dshot:%x\r\n",);
        //sleep_ms(1000);
    }
}

// UART Code
void UART_RX(){
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        
        if(ch == '#'){
            buf_cnt = 0;
            buf_rdy = false;
        }
        else if(!buf_rdy && buf_cnt < 32){
            msg_buf[buf_cnt] = ch;
        }
        else if(ch == ';'){
            msg_buf[buf_cnt] = ch;
            buf_rdy = true;
            received_msgs++;
        }

        if(buf_rdy){
            xQueueSendFromISR(msg_queue,(void *) &msg_buf,NULL);
        }
        
    }
}

void UART_INIT(){
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, UART_RX);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    uart_puts(UART_ID, "\nHello, uart interrupts\n");
}

void freeRTOS_setup(){

    msg_queue = xQueueCreate(5,sizeof(Ctrl_Message));
    ADC_queue = xQueueCreate(8,sizeof(ADC_Message));
    motor_queue = xQueueCreate(1,sizeof(motorMessage));
    I2C_mutex = xSemaphoreCreateMutex();

}

// Tasks

void ADC_task(){

    uint8_t ch=0;
    uint16_t raw=0;
    float conversion;
    while(ADC_queue == NULL);
    while(I2C_mutex == NULL);
    TickType_t LastRun;
    ADC_Message aMessage;


    while(true){
        if( xSemaphoreTake( I2C_mutex, ( TickType_t ) portMAX_DELAY ) == pdTRUE ){
            switch(ch){
                case 0:
                    ads1115_set_MUX(&adc,MUX_SINGLE_1);
                    break;
                case 1:
                    ads1115_set_MUX(&adc,MUX_SINGLE_2);
                    break;
                case 2:
                    ads1115_set_MUX(&adc,MUX_SINGLE_3);
                    break;
                case 3:
                    ads1115_set_MUX(&adc,MUX_SINGLE_4);
                    break;                                                            
            }
            ads1115_writeConfig(&adc);
            ads1115_readADC(&adc,&raw);
            xSemaphoreGive( I2C_mutex );
        }
            conversion = ads1115_convert_raw(&adc,raw);
            aMessage.voltage = conversion;
            aMessage.channel = ch;
            ch++;
            ch = ch % 4;
            xQueueSendToBack(ADC_queue,(void *) & aMessage, (TickType_t) 10);
            xTaskDelayUntil(&LastRun,100);
        }
}

void motorControl_task(){

    motorMessage motorValues;
    while(true){

        xQueueReceive(motor_queue,(void *) &motorValues,(TickType_t) portMAX_DELAY);

        pio_sm_put_blocking(pio,sm[0],dshot_parse_cmd(motorValues.Prop_Right,false));
        pio_sm_put_blocking(pio,sm[1],dshot_parse_cmd(motorValues.Prop_Left,false));
        pio_sm_put_blocking(pio,sm[2],dshot_parse_cmd(motorValues.Depth_Right,false));
        pio_sm_put_blocking(pio,sm[3],dshot_parse_cmd(motorValues.Depth_Left,false));

    }
}