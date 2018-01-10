/******************************************************************************

 @file board_led.c

 @brief This file contains the interface to the SRF06EB LED Service

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc13x0_sdk_1_50_00_
 Release Date: 2017-09-27 23:49:29
 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/

#include <xdc/std.h>

#include <ti/drivers/PIN.h>

#include "icall.h"

#include "board.h"

#include "timer.h"
#include "board_led.h"

/******************************************************************************
 Constants
 *****************************************************************************/
#if !defined(BOARD_LED_BLINK_PERIOD)
#define BOARD_LED_BLINK_PERIOD 500     /* in milliseconds */
#endif

typedef enum
{
    BLINKING_STATUS_ON,
    BLINKING_STATUS_OFF,
    BLINKING_STATUS_DONE
} blinkStatus;

/******************************************************************************
 Typedefs
 *****************************************************************************/

typedef struct
{
    board_led_state state; /* Off, On or Blink */
    blinkStatus status; /* What is led status (on, off, or done) */
} board_led_status_t;

/******************************************************************************
 Local Variables
 *****************************************************************************/

static Clock_Struct blinkClkStruct;

static board_led_status_t ledStatus[MAX_LEDS];

#if defined(CC13XX_LAUNCHXL) || defined(CC13X2R1_LAUNCHXL)
#if defined(Board_DIO1_RFSW)
#define Board_DIO_RFSW Board_DIO1_RFSW
#elif defined(Board_DIO30_RFSW)
#define Board_DIO_RFSW Board_DIO30_RFSW
#endif
#endif

static PIN_Config ledPinTable[] =
{
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL
            | PIN_DRVSTR_MAX, /* LED1 initially off */
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL
            | PIN_DRVSTR_MAX, /* LED2 initially off */
#if defined(CC13XX_LAUNCHXL) || defined(CC13X2R1_LAUNCHXL)
#if !defined(FREQ_2_4G)
    Board_DIO_RFSW   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,   /* RF SW Switch defaults to sub-1GHz path*/
#else
    Board_DIO_RFSW   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,   /* RF SW Switch defaults to 2GHz path*/
#endif
#endif
#if defined(CC13XX_LAUNCHXL) && !defined(COPROCESSOR)
#if (CONFIG_RANGE_EXT_MODE == APIMAC_NO_EXTENDER)
    Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,    /* External RF Switch is powered on by default */
#endif
#endif
    PIN_TERMINATE /* Terminate list     */
};

/* LED pin state */
static PIN_State ledPinState;

/* LED Pin Handle */
static PIN_Handle ledPinHandle;

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/
static void board_led_blinkTimeoutCB(UArg a0);
static bool board_led_anyBlinking(void);
static void board_led_blinkLed(void);
static unsigned int board_led_convertLedType(board_led_type led);
static uint32_t board_led_convertLedValue(board_led_state state);

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize LEDs

 Public function defined in board_led.h
 */
void Board_Led_initialize(void)
{
    uint8_t x;
    unsigned int index;
    uint32_t value;

    /* Open LED PIN driver */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);

    value = board_led_convertLedValue(board_led_state_OFF);

    for(x = 0; x < MAX_LEDS; x++)
    {
        ledStatus[x].state = board_led_state_OFF;
        ledStatus[x].status = BLINKING_STATUS_DONE;

        index = board_led_convertLedType((board_led_type) x);

        PIN_setOutputValue(ledPinHandle, index, value);
    }

    Timer_construct(&blinkClkStruct, board_led_blinkTimeoutCB,
    BOARD_LED_BLINK_PERIOD,
                        0, false, 0);
}

/*!
 Control the state of an LED

 Public function defined in board_led.h
 */
void Board_Led_control(board_led_type led, board_led_state state)
{
    unsigned int gpioType;
    uint32_t value;
    ICall_CSState CsState;

    /* Look for invalid parameters */
    if((led >= MAX_LEDS) || (state > board_led_state_BLINKING))
    {
        return;
    }

    /* Convert to GPIO types */
    gpioType = board_led_convertLedType(led);
    value = board_led_convertLedValue(state);

    /* Save state and status */
    ledStatus[led].state = state;
    if((state == board_led_state_BLINK) || (state == board_led_state_BLINKING))
    {
        ledStatus[led].status = BLINKING_STATUS_ON;
    }

    /* Enter critical section so this function is thread safe*/
    CsState = ICall_enterCriticalSection();

    /* Update hardware LEDs */
    PIN_setOutputValue(ledPinHandle, gpioType, value);

    /* Exit critical section */
    ICall_leaveCriticalSection(CsState);


    /* Are any LEDs are blinking? */
    if(board_led_anyBlinking())
    {
        if(Timer_isActive(&blinkClkStruct) == false)
        {
            Timer_start(&blinkClkStruct);
        }
    }
    else
    {
        if(Timer_isActive(&blinkClkStruct) == true)
        {
            Timer_stop(&blinkClkStruct);
        }
    }
}

/*!
 Toggle the state of an LED

 Public function defined in board_led.h
 */
void Board_Led_toggle(board_led_type led)
{
    board_led_state newState = board_led_state_OFF;

    /* Look for invalid parameter */
    if(led < MAX_LEDS)
    {
        /* Toggle state */
        if(ledStatus[led].state == board_led_state_OFF)
        {
            newState = board_led_state_ON;
        }

        /* Set new state */
        Board_Led_control(led, newState);
    }
}

/******************************************************************************
 Local Functions
 *****************************************************************************/

/*!
 * @brief       Timeout handler function
 *
 * @param       a0 - ignored
 */
static void board_led_blinkTimeoutCB(UArg a0)
{
    /* Update blinking LEDs */
    board_led_blinkLed();

    if(board_led_anyBlinking())
    {
        /* Setup for next time */
        Timer_start(&blinkClkStruct);
    }
}

/*!
 * @brief       Are there any blinking LEDs?
 *
 * @return      true, yes at least one.  false if none
 */
static bool board_led_anyBlinking(void)
{
    uint8_t x;

    for(x = 0; x < MAX_LEDS; x++)
    {
        if((ledStatus[x].state == board_led_state_BLINKING) ||
                        ((ledStatus[x].state == board_led_state_BLINK)
                        && (ledStatus[x].status != BLINKING_STATUS_DONE)))
        {
            return (true);
        }
    }

    return (false);
}

/*!
 * @brief       Blink LEDs
 */
static void board_led_blinkLed(void)
{
    uint8_t x;
    ICall_CSState CsState;

    for(x = 0; x < MAX_LEDS; x++)
    {
        unsigned int index;
        uint32_t value;

        if(ledStatus[x].state == board_led_state_BLINKING)
        {
            index = board_led_convertLedType((board_led_type) x);

            if(ledStatus[x].status == BLINKING_STATUS_OFF)
            {
                value = board_led_convertLedValue(board_led_state_ON);
                ledStatus[x].status = BLINKING_STATUS_ON;
            }
            else
            {
                value = board_led_convertLedValue(board_led_state_OFF);
                ledStatus[x].status = BLINKING_STATUS_OFF;
            }

            /* Enter critical section so this function is thread safe*/
            CsState = ICall_enterCriticalSection();

            PIN_setOutputValue(ledPinHandle, index, value);

            /* Exit critical section */
            ICall_leaveCriticalSection(CsState);
        }
        else if((ledStatus[x].state == board_led_state_BLINK) && (ledStatus[x]
                        .status
                                                                  != BLINKING_STATUS_DONE))
        {
            index = board_led_convertLedType((board_led_type) x);
            value = board_led_convertLedValue(board_led_state_OFF);
            ledStatus[x].status = BLINKING_STATUS_DONE;

            /* Enter critical section so this function is thread safe*/
            CsState = ICall_enterCriticalSection();

            PIN_setOutputValue(ledPinHandle, index, value);

            /* Exit critical section */
            ICall_leaveCriticalSection(CsState);
        }
    }
}

/*!
 * @brief       Convert from board_led type to PIN led type
 *
 * @param       led - board_led type
 *
 * @return      PIN Led Type
 */
static unsigned int board_led_convertLedType(board_led_type led)
{
    if (led == board_led_type_LED1)
    {
        return(Board_LED0);
    }
#if defined(CC13XX_LAUNCHXL) || defined(CC13X2R1_LAUNCHXL)
    else if (led == board_rfSwitch_select)
    {
        return(Board_DIO_RFSW);
    }
#endif
#if defined(CC13XX_LAUNCHXL) && !defined(COPROCESSOR)
#if (CONFIG_RANGE_EXT_MODE == APIMAC_NO_EXTENDER)
    else if (led == board_rfSwitch_pwr)
    {
        return(Board_DIO30_SWPWR);
    }
#endif
#endif
    return(Board_LED1);
}

/*!
 * @brief       Convert from board_led value to GPIO value
 *
 * @param       led - board_led type
 *
 * @return      GPIO value
 */
static uint32_t board_led_convertLedValue(board_led_state state)
{
    uint32_t value;

    switch(state)
    {
        case board_led_state_ON:
        case board_led_state_BLINK:
        case board_led_state_BLINKING:
            value = Board_LED_ON;
            break;

        default:
            value = Board_LED_OFF;
            break;
    }

    return (value);
}
