/*!
 * \file      gpio.c
 *
 * \brief     GPIO driver implementation
 *
 * \remark: Relies on the specific board GPIO implementation as well as on
 *          IO expander driver implementation if one is available on the target
 *          board.
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "gpio-board.h"
#include "board-config.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>

void GpioInit( Gpio_t *obj, PinNames pin, PinModes mode,  PinConfigs config, PinTypes type, uint32_t value )
{
    // GpioMcuInit( obj, pin, mode, config, type, value );
    obj->pin=pin;
    if(mode==PIN_OUTPUT)
    {
        GpioWrite(obj,value);
    }
    else
    {
        printf("PinName is %d,PinModes is %d\n",pin,mode);
    }

}

void GpioSetContext( Gpio_t *obj, void* context )
{
    GpioMcuSetContext( obj, context );
}

void GpioSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    // GpioMcuSetInterrupt( obj, irqMode, irqPriority, irqHandler );
}

void GpioRemoveInterrupt( Gpio_t *obj )
{
    GpioMcuRemoveInterrupt( obj );
}

void GpioWrite( Gpio_t *obj, uint32_t value )
{
    // GpioMcuWrite( obj, value );
    if(obj->pin==LED_1)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 , ( GPIO_PinState )value );
    }
    else if(obj->pin==LED_2)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 , ( GPIO_PinState )value );
    }
    else if(obj->pin==RADIO_RESET) // reset
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 , ( GPIO_PinState )value );
    }
    else if(obj->pin==RADIO_NSS)// spi_nss
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 , ( GPIO_PinState )value );
    }
     else if(obj->pin==RADIO_MOSI)// spi_mosi
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 , ( GPIO_PinState )value );
    }
     else if(obj->pin==RADIO_MISO)// spi_miso
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 , ( GPIO_PinState )value );
    }
     else if(obj->pin==RADIO_SCLK)// spi_sck
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 , ( GPIO_PinState )value );
    }
    else
    {
        printf("the pinname is %d\n",obj->pin);
        printf("gpio write\n");
        ;
    }
}

void GpioToggle( Gpio_t *obj )
{
    // GpioMcuToggle( obj );
    if(obj->pin==LED_1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1 );
    }
    else if(obj->pin==LED_2)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0 );
    }
    // else if(obj->pin==RADIO_RESET) // reset
    // {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 , ( GPIO_PinState )value );
    // }
    // else if(obj->pin==RADIO_NSS)// spi_nss
    // {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 , ( GPIO_PinState )value );
    // }
    // else if(obj->pin==LED_2)
    // {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 , ( GPIO_PinState )value );
    // }
    // else if(obj->pin==LED_2)
    // {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 , ( GPIO_PinState )value );
    // }
    else
    {
        printf("the pinname is %d\n",obj->pin);
        printf("gpio toggle\n");
        ;
    }
}

uint32_t GpioRead( Gpio_t *obj )
{
    // return GpioMcuRead( obj );
    if(obj->pin==RADIO_BUSY) //busy
    {
        // printf("read busy line\n");
        // return 0x00;
        return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    }
    // else if(obj->pin==LED_2)
    // {
    //     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 , ( GPIO_PinState )value );
    // }
    // else if(obj->pin==RADIO_RESET) // reset
    // {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 , ( GPIO_PinState )value );
    // }
    // else if(obj->pin==RADIO_NSS)// spi_nss
    // {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 , ( GPIO_PinState )value );
    // }
    else
    {
        printf("the pinname is %d\n",obj->pin);
        printf("gpio read\n");
        return 0x00;
    }
}
