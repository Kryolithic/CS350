/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 *  Modified By: Gary Clark
 *  CS350
 *  Southern New Hampshire University
 */
#pragma diag_suppress 225 //suppressing warning from snprintf calls
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/timer/TimerCC32XX.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Definition for UART output interface*/
#define DISPLAY(x) UART_write(uart, &output, x);

/* Task Structure */
typedef struct task {
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFct)(int);
} task;

//array to hold tasks
task tasks[3];

//maintenance variables
unsigned long seconds = 0;
unsigned char ledState, raiseTmp, lowerTmp, heat = 0;
unsigned int setpoint = 20;
unsigned int curTemp, outTimer = 0;

//task scheduler variables
const unsigned char tasksNum = 3;
const unsigned long periodGCD = 100000;
const unsigned long periodBtnCheck = 200000;
const unsigned long periodTmpCheck = 500000;


//state enums for tasks
enum BTN_States { BTN_1, BTN_2, BTN_RST};
enum CK_States {CK_1};
enum TMP_States { TMP_HIGH, TMP_LOW};

//tick function prototypes
int TickFct_BtnState(int state);
int TickFct_CheckTmp(int state);
int TickFct_TmpState(int state);

//initializing task array with values of corresponding tasks
void initTasks(){
            tasks[0].state = BTN_RST;
            tasks[0].period = periodBtnCheck;
            tasks[0].elapsedTime = tasks[0].period;
            tasks[0].TickFct = &TickFct_BtnState;

            tasks[1].state = CK_1;
            tasks[1].period = periodTmpCheck;
            tasks[1].elapsedTime = tasks[1].period;
            tasks[1].TickFct = &TickFct_CheckTmp;

            tasks[2].state = TMP_HIGH;
            tasks[2].period = periodTmpCheck;
            tasks[2].elapsedTime = tasks[2].period;
            tasks[2].TickFct = &TickFct_TmpState;
}

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

void initUART(void)
{
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}


//timer callback prototype
void timerCallback(Timer_Handle myHandle, int_fast16_t status);

//Timer flag variable
volatile unsigned char TimerFlag = 0;

//timer handle
Timer_Handle timer0;

//timer init function
void initTimer(void){

    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = periodGCD; //timer period equal to GCD of tasks periods
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
         /* Failed to initialized timer */
         while (1) {}
     }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
         /* Failed to start timer */
         while (1) {}
     }
}



// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
     { 0x48, 0x0000, "11X" },
     { 0x49, 0x0000, "116" },
     { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
        found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{

    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY( snprintf(output, 64, "Error reading temperature sensor(%d)\n\r", i2cTransaction.status))
        DISPLAY( snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

/*
 *  Button callbacks
 * button 0 raises setpoint by 1 only if button 1 has not been pressed during task period, then assigns state of task
 * button 1 lowers setpoint by 1 only if button 0 has not been pressed during task period, then assigns state of task
 */
void gpioButtonFxn0(uint_least8_t index)
{
    if(lowerTmp != 1){
        tasks[0].state = BTN_1;
        raiseTmp = 1;
    }
}

void gpioButtonFxn1(uint_least8_t index)
{
    if(raiseTmp != 1){
        tasks[0].state = BTN_2;
        lowerTmp = 1;
    }
}

/*
 * task scheduler using RIMS architecture from textbook
 * iterates through tasks checking to see if the elapsed time is greater than or equal to the period of the task
 * if it is, the tick function is called for that task passing the state value
 * if not then it adds the GCD of the tasks periods
  */
void TimerISR() {

    unsigned char i;
    for (i = 0; i < tasksNum; ++i) {
        if(tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += periodGCD;

    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    //init functions
    initUART();
    initI2C();
    initTimer();
    GPIO_init();
    initTasks();

    //enabling GPIO for buttons and LED
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn off user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    //runs indefinitely
    while(1)
    {
        //call to task scheduler
        TimerISR();

        /* *****************************************************************
         * checks to see if 10 periods have passed (100000us * 10 = 1s)
         * if it has then function updates LED state and outputs data to console
         * LED is on when heat is on, off when heat is off
         * resets the number of periods to 0
         * ********************************************************************/
        if(outTimer == 10){
            if(heat >= 1){
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
            DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", curTemp, setpoint, heat, seconds))
            outTimer = 0;
        }

        //waits for timer period
        while(!TimerFlag){}
        //resets timer flag
        TimerFlag = 0;
        seconds += periodGCD; //adds period length to seconds counter, seconds will count forever unless board is reset (could lead to overflow after board is on for extremely long time period)
        ++outTimer; //increments number of periods
    }
}
//timer callback
void timerCallback(Timer_Handle myHandle, int_fast16_t status){
    TimerFlag = 1; //timer flag is engaged
}

//button task function
int TickFct_BtnState(int state){
    switch(state){
    case BTN_1:
        setpoint += 1; //increment setpoint by 1
        raiseTmp = 0; //reset raiseTemp flag
        state = BTN_RST; //reset button state to rest
        break;
    case BTN_2:
        setpoint -= 1;
        lowerTmp = 0;
        state = BTN_RST;
        break;
    case BTN_RST: //when button is in rest state does nothing -> ensures setpoint only increments by 1 degree per button press per period
        break;
    default:
        break;
    }
    return state;
}

//check temperature task function
int TickFct_CheckTmp(int state){
    switch(state){
    case CK_1: //the only state in the check temp function
        heat = 0; //heat is default off
        curTemp = readTemp(); //reads temp from sensor
        //if current temp is less than the setpoint temp turns heat on and changes temp state to low
        if(curTemp < setpoint) {
            tasks[2].state = TMP_LOW;
            heat = 1;
        }
        state = CK_1; //passes the same state as it is only state within task
        break;
    default:
        break;
    }
    return state;
}

//temperature state task function -- redundancy check on the state of the LED and the environment temp. if temp is warmer than setpoint state is high and led is off.
int TickFct_TmpState(int state){
    switch(state){
    case TMP_HIGH:
        ledState = 0;
        state = TMP_HIGH;
        break;
    case TMP_LOW:
        ledState = 1;
        state = TMP_LOW;
        break;
    default:
        break;
    }
    return state;
}



