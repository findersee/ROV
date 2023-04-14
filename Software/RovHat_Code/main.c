#include "main.h"

    int main() {
    timer_hw->dbgpause = 0;
    HW_setup();
    uint16_t value;
    // Loop forever 
    uint16_t pwr=1047;

    freeRTOS_setup();




    printf("Starting scheduler...nr");
    vTaskStartScheduler();
    while (true) {
        asm("nop");
    }
}

float ascToInt(char *point,uint8_t length){
    
    uint8_t out = 0;
 
    out = (*point++ - 0x30)*100;
    out = out + (*point++ - 0x30)*10;
    out = out + (*point-0x30);

    return (float)out/100;
}


void HW_setup()
{
    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    //stdio_init_all();
    //stdio_uart_init_full(uart0,115200,)

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

    UART_INIT();
}

uint8_t bufferSize(UART_buffer_t *buffer){
    if(buffer->head < buffer->tail)
        return buffer->tail-(buffer->head+64);
    return buffer->head-buffer->tail;

}

bool bufferEmpty(UART_buffer_t *buffer){
    return buffer->tail == buffer->head;
}

void bufferTake(UART_buffer_t *buffer, char *dst, uint8_t elements){

    for(uint8_t i;i<elements;i++){
        dst[i] = buffer->data[buffer->tail];
        buffer->tail=(buffer->tail+1)%rxBufSize;
    }
}

// UART Code
void UART_RX(){
    while (uart_is_readable(UART_ID)) {
        //uint8_t ch = uart_getc(UART_ID);
        rxBuffer.data[rxBuffer.head] = uart_getc(UART_ID);
        rxBuffer.head = ((rxBuffer.head+1)%rxBufSize);
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

    rxBuffer.bufSize = 64;
    rxBuffer.tail = 0;
    rxBuffer.head = 0;

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    uart_puts(UART_ID, "\nHello, uart interrupts\n");


}

void freeRTOS_setup(){
    //Initialize queues
    ADC_queue = xQueueCreate(8,sizeof(ADC_Message_t));

    Propulsion_motor_queue = xQueueCreate(1,sizeof(motorMessage_t));
    Depth_motor_queue = xQueueCreate(1,sizeof(motorMessage_t));

    AutoHold_queue = xQueueCreate(4,sizeof(AutoHoldMessage_t));    

    //Initialize mutex
    I2C_mutex = xSemaphoreCreateMutex();
    AutoDepth_mutex = xSemaphoreCreateMutex();




    xTaskCreate(ADC_task,"ADC_TASK",configMINIMAL_STACK_SIZE,NULL,1,NULL);

    xTaskCreate(motorControl_task,"MOTOR_TASK",configMINIMAL_STACK_SIZE,NULL,4,NULL);

    xTaskCreate(UART_RX_Handler_Task,"UART HANDLER",configMINIMAL_STACK_SIZE,NULL,5,NULL);

    //xTaskCreate(Depth_Hold_task,"Auto Depth Hold",configMINIMAL_STACK_SIZE,NULL,4,NULL);
}

// Tasks

void ADC_task(void *pvParameters){

    uint8_t ch=0;
    uint16_t raw=0;
    float conversion;
    while(ADC_queue == NULL){
        vTaskDelay((TickType_t) 100);
    }
    while(I2C_mutex == NULL){
        vTaskDelay((TickType_t) 100);
    }
    TickType_t LastRun;
    ADC_Message_t aMessage;


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

void motorControl_task(void *pvParameters){

    TickType_t LastRun;

    motorMessage_t Propulsion_motorValues;
    motorMessage_t Depth_motorValues;
    
    Propulsion_motorValues.Left = 1047;
    Propulsion_motorValues.Right = 1047;

    Depth_motorValues.Left = 1047;
    Depth_motorValues.Right = 1047;

    while(Propulsion_motor_queue == NULL){
        vTaskDelay((TickType_t) 100);
    }

    while(Depth_motor_queue == NULL){
        vTaskDelay((TickType_t) 100);
    }

    while(true){

        xQueueReceive(Propulsion_motor_queue,(void *) &Propulsion_motorValues,(TickType_t) 0); // Get new propulsion motor values, don't block if nothing new

        xQueueReceive(Depth_motor_queue,(void *) &Depth_motorValues,(TickType_t) 0); // Get new propulsion motor values, don't block if nothing new

        //Send updated values to right propulsion motor using Dshot
        if(Propulsion_motorValues.Right != 1047)
            pio_sm_put_blocking(pio,sm[0],dshot_parse_throttle(&Propulsion_motorValues.Right,false));
        else
            pio_sm_put_blocking(pio,sm[0],dshot_parse_cmd(DSHOT_CMD_MOTOR_STOP,false));

        //Send updated values to left propulsion motor using Dshot
        if(Propulsion_motorValues.Left != 1047)
            pio_sm_put_blocking(pio,sm[1],dshot_parse_throttle(&Propulsion_motorValues.Left,false));
        else
            pio_sm_put_blocking(pio,sm[1],dshot_parse_cmd(DSHOT_CMD_MOTOR_STOP,false));

        //Send updated values to right depth motor using Dshot
        if(Depth_motorValues.Right != 1047)
            pio_sm_put_blocking(pio,sm[2],dshot_parse_throttle(&Depth_motorValues.Right,false));
        else
            pio_sm_put_blocking(pio,sm[2],dshot_parse_cmd(DSHOT_CMD_MOTOR_STOP,false));

        //Send updated values to right depth motor using Dshot
        if(Depth_motorValues.Left != 1047)
            pio_sm_put_blocking(pio,sm[3],dshot_parse_throttle(&Depth_motorValues.Left,false));
        else
            pio_sm_put_blocking(pio,sm[3],dshot_parse_cmd(DSHOT_CMD_MOTOR_STOP,false));



        xTaskDelayUntil(&LastRun,( TickType_t )500); // Loop Motor update every 500 ms

    }
}



void UART_RX_Handler_Task(void *pvParameters){


    char uart_msg[33];

    motorMessage_t PropMsg,DepthMsg;

    AutoHoldMessage_t AutoHoldMsg;

    AutoHoldMsg.Active = false;
    AutoHoldMsg.Level = 1000.f;


    PropMsg.Left = 1047;
    PropMsg.Right = 1047;

    DepthMsg.Left = 1047;
    DepthMsg.Right = 1047;

    while(AutoHold_queue == NULL){
        vTaskDelay((TickType_t) 100);
    }

    while(true){

        // Message format #R100L100+100I100C:(CMD)\0

        
        if(bufferSize(&rxBuffer)>4){
            memset (&uart_msg,0,sizeof(uart_msg));
            bufferTake(&rxBuffer,(char *)&uart_msg,bufferSize(&rxBuffer));
            
            uint8_t msgLen = strnlen((char *)&uart_msg,33);
            uint8_t msgCnt = 0;
            for(;msgCnt < msgLen;msgCnt++){
                if(uart_msg[msgCnt] == '#')
                    break;
            }

            for(;msgCnt < msgLen;msgCnt++){
                
                switch (uart_msg[msgCnt])
                {
                case 'R':
                    msgCnt++;

                    PropMsg.Right = (uint16_t)((ascToInt((char *)&uart_msg[msgCnt],3)*1000)+999);
                    msgCnt += 2;
                    
                    break;
                    
                case 'r':
                    msgCnt++;

                    PropMsg.Right = (uint16_t)((1.f-ascToInt((char *)&uart_msg[msgCnt],3))*999);
                    msgCnt += 2;

                    break;

                case 'L':
                    msgCnt++;

                    PropMsg.Left = (uint16_t)((ascToInt((char *)&uart_msg[msgCnt],3)*1000)+999);
                    msgCnt += 2;
                    
                    break;
                    
                case 'l':
                    msgCnt++;

                    PropMsg.Left = (uint16_t)((1.f-ascToInt((char *)&uart_msg[msgCnt],3))*999);
                    msgCnt += 2;

                    break;

                case '+':
                    msgCnt++;

                    DepthMsg.Right = (uint16_t)((ascToInt((char *)&uart_msg[msgCnt],3)*1000)+999);
                    DepthMsg.Left = (uint16_t)((ascToInt((char *)&uart_msg[msgCnt],3)*1000)+999);
                    msgCnt += 2;
                    
                    break;
                    
                case '-':
                    msgCnt++;

                    DepthMsg.Right = (uint16_t)((1.f-ascToInt((char *)&uart_msg[msgCnt],3))*999);
                    DepthMsg.Left = (uint16_t)((1.f-ascToInt((char *)&uart_msg[msgCnt],3))*999);                
                    msgCnt += 2;

                    break;

                case 'I':
                    msgCnt++;

                    pwm_set_gpio_level(NMOS_1,(uint16_t)(ascToInt((char *)&uart_msg[msgCnt],3)*2500.f));

                    msgCnt += 2; 
                    break;

                case 'C':
                    msgCnt += 2;
                    
                    /*
                    Commands:
                    H = Enable Autohold
                    h = Disable Autohold
                    C = Calibrate IMU
                    P = Reset Pressure
                    */



                    switch (uart_msg[msgCnt])
                    {
                    case 'H':
                        AutoHoldMsg.Active = true;
                        AutoHoldMsg.Level = 1200.f;
                        xQueueSend(AutoHold_queue,(void *) &AutoHoldMsg,(TickType_t)0);
                        break;
                    case 'h':
                        AutoHoldMsg.Active = false;
                        AutoHoldMsg.Level = 800.f;
                        xQueueSend(AutoHold_queue,(void *) &AutoHoldMsg,(TickType_t)0);
                        break;
                    default:
                        break;
                    }
                    msgCnt ++;
                    break;

                default:
                    break;
                }
            }


        }
        else
            vTaskDelay((TickType_t) 100);

        xQueueOverwrite(Propulsion_motor_queue,(void *) &PropMsg);
        if(!Auto_Hold_Active)
            xQueueOverwrite(Depth_motor_queue,(void *) &DepthMsg);

    }
}

// Define the task function for the PID loop
void Depth_Hold_task(void *pvParameters) {
    // Initialize variables for the PID loop
    float setpoint = 0.0f;
    float input = 0.0f;
    float output = 0.0f;
    float kp = 1.0f; // Proportional gain
    float ki = 0.5f; // Integral gain
    float kd = 0.2f; // Derivative gain
    float error = 0.0f;
    float error_integral = 0.0f;
    float error_derivative = 0.0f;
    float last_error = 0.0f;
    float sample_time = 0.1f; // Time between PID calculations in seconds
    
    motorMessage_t DepthMsg;

    AutoHoldMessage_t HoldMsg;
    
    AutoHoldMessage_t AutoHoldMsg;


    while(AutoHold_queue == NULL){
        vTaskDelay((TickType_t) 100);
    }


    // Main loop for the PID controller
    while (1) {

        xQueueReceive(AutoHold_queue,(void *) &AutoHoldMsg,(TickType_t) 0);

        while(!Auto_Hold_Active){
            vTaskDelay((TickType_t)1000);
        };
        // Read the input value from the sensor or other source
        //input = readInputValue();
        
        // Calculate the error between the setpoint and input
        error = setpoint - input;
        
        // Calculate the integral of the error over time
        error_integral += error * sample_time;
        
        // Calculate the derivative of the error with respect to time
        error_derivative = (error - last_error) / sample_time;
        
        // Calculate the output value using the PID formula
        output = kp * error + ki * error_integral + kd * error_derivative;
        
        // Save the current error for the next iteration
        last_error = error;
        
        // Send the output value to the actuator or other destination
        DepthMsg.Right = 1047;
        DepthMsg.Left = 1047;
        
        // Wait for the next iteration of the PID loop
        
        if(AutoHoldMsg.Active){
        xQueueOverwrite(Depth_motor_queue,(void *) &DepthMsg);
        }
        vTaskDelay(pdMS_TO_TICKS(sample_time * 1000));
        
    }
}