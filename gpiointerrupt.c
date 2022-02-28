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
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*Timer Driver*/
#include <ti/drivers/Timer.h>

//enum for message for reusability
enum Message
{
    SOS,
    OK
};

//creating variable to hold enum value
enum Message message = SOS;

//enum to hold morse code language for reusability
enum Morse {
    DOT,
    DASH
};

//two arrays holding Morse enum data types -- THIS STORES THE MORSE CODE FOR EACH MESSAGE --
enum Morse morseSOS[] = {DOT, DOT, DOT, DASH, DASH, DASH, DOT, DOT, DOT};
enum Morse morseOK[] = {DASH, DASH, DASH, DASH, DOT, DASH};

//maintenance variables for led logic and program behavior
int tickCount = 0;
int changeMsg = 0;
int dash, btwnWord, wordPos = 0;

//timer callback function prototype
void timerCallback(Timer_Handle myHandle, int_fast16_t status);

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    changeMsg = 1; //sets changeMsg variable to 1 which is used to inform timer callback that it should change the message once the current one is finished
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();

    //timer init data
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;                         //period of 500000us = 500ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;   //continuous callback
    params.timerCallback = timerCallback;           //associating callback function


    timer0 = Timer_open(CONFIG_TIMER_0, &params);

         if (timer0 == NULL) {
             /* Failed to initialized timer */
             while (1) {}
         }
         if (Timer_start(timer0) == Timer_STATUS_ERROR) {
             /* Failed to start timer */
             while (1) {}
         }

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn off user LED by default */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);


    return (NULL);
}

//timer callback function
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{

    if(GPIO_read(CONFIG_GPIO_LED_0)==1) //dot red led toggles off after 1 period
        {
            GPIO_toggle(CONFIG_GPIO_LED_0);
            tickCount = 0; //reset tick counter
            wordPos+=1; //iterate word position by 1
        }

    if(GPIO_read(CONFIG_GPIO_LED_1)==1 && tickCount == 3) //dash green led remains on for 3 period cycles or tick counts, then program turns it off
        {
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
            tickCount = 0;
            wordPos+=1;
        }

    if(btwnWord == 1 && tickCount == 7){                                    //if the program is between words, function will execute after 7 period cycles or tick counts (3500ms)
            btwnWord = 0;                                                   //resets between word as the program has waited 7 period cycles and is now able to execute next message
            if(message == SOS && changeMsg == 1){                           //change message variable will only be equal to 1 if the button callback executes, statement checks which message is the current message
                message = OK;                                               //changes the value of message to the other enum value
                changeMsg = 0;                                              //resets change message variable to 0
            }
            if(message == OK && changeMsg == 1){ //same logic but with reversed current message
                message = SOS;
                changeMsg = 0;
            }
            tickCount = 1;//sets tickCount to 1 so next message can execute
        }
    //logic to output SOS in morse code with leds -- tick count must be >= 1 which cycles through 1 period before flashing next led -- program will not enter code if it is between words
    //***********************************************************************************************************************************************************************************
    if(message == SOS && tickCount >= 1 && btwnWord == 0) {
        if(morseSOS[wordPos] == 0 && wordPos < 9){                  //morse enum DOT = 0 -- SOS requires 9 led flashes and morseSOS has 9 index entries, by verifying word position < 9 we avoid access errors
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);      //red led for dot
        }
        else if(morseSOS[wordPos] == 1 && wordPos < 9){ //morse enum DASH = 1

            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON); //green led for dash
        }
        else{ //this statement executes once the program reaches the end of the message
            btwnWord = 1; //lets program know that it is now in between words
            wordPos = 0; //resets word position
        }
    }
    //logic to output OK in morse code with leds
    if(message == OK && tickCount >= 1 && btwnWord == 0){
        if(morseOK[wordPos] == 0 && wordPos < 6){ //same logic as SOS configuration except word position must be < 6 as OK requires 6 flashes in morse
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        }
        else if(morseOK[wordPos] == 1 && wordPos < 6){
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
        else{
             btwnWord = 1;
             wordPos = 0;
        }
    }

    tickCount+=1; //iterates tickCount at end of timer interrupt callback

}
