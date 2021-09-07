/**
  ******************************************************************************
  * @file    gpio.c
  * @author  Marco, Roldan L.
  * @date    September 07, 2021
  * @brief   
  ******************************************************************************
  *
  * Copyright (C) 2021  Marco, Roldan L.
  * 
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  * 
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  * 
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see https://www.gnu.org/licenses/gpl-3.0.en.html.
  * 
  * 
  * https://github.com/rmarco30
  * 
  ******************************************************************************
**/

#include "gpio.h"



/**
 * @brief    Initializes a GPIO
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @param    gpio_init: pointer to GPIO_Init_t structure where desired GPIO
 *                      function was initialized.
 * @retval   none
 */
void gpio_init(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx, GPIO_Init_t* gpio_init)
{
    /* check if alternate function is required */
    uint32_t alt_func_en = 0;

    if( (gpio_init->MODE != 0) && (gpio_init->CONFIG > 0x01) )
    {
        alt_func_en = 0x01;
    }

    /* Turn on the clock */
    uint32_t gpiox_base = (uint32_t)GPIOx;

    switch( gpiox_base )
    {
        case GPIOA_BASE:
            RCC->APB2ENR |= ( RCC_APB2ENR_IOPAEN | alt_func_en );
            break;
        
        case GPIOB_BASE:
            RCC->APB2ENR |= ( RCC_APB2ENR_IOPBEN | alt_func_en );
            break;
        
        case GPIOC_BASE:
            RCC->APB2ENR |= ( RCC_APB2ENR_IOPCEN | alt_func_en );
            break;
        
        case GPIOD_BASE:
            RCC->APB2ENR |= ( RCC_APB2ENR_IOPDEN | alt_func_en );
            break;
        
        case GPIOE_BASE:
            RCC->APB2ENR |= ( RCC_APB2ENR_IOPEEN | alt_func_en );
            break;

        default:
            break;
    }

    /* check input mode if pull up/down is needed */
    uint32_t pull_updown = 0;
    
    if( (gpio_init->MODE == 0)  && (gpio_init->CONFIG > 0x01) )
    {
        if( gpio_init->CONFIG == 0x02 )
        {
            /* save pull up pin value */
            pull_updown = (1U << GPIO_PINx);
        }
        else
        {
            /* save pull down pin value */
            pull_updown = (1U << (GPIO_PINx + 16U) );
            /* modify config to valid state */
            gpio_init->CONFIG = 0x02;
        }
    }

    /* configure the pin */
    uint32_t tmp = ( (gpio_init->CONFIG << 2) | gpio_init->MODE );

    if( GPIO_PINx < 8 )
    {
        GPIOx->CRL &= ~( 0x0f << ( GPIO_PINx * 4) );
        GPIOx->CRL |= tmp << ( GPIO_PINx * 4);
    }
    else
    {
        GPIOx->CRH &= ~( 0x0f << ( (GPIO_PINx - 8) * 4) );
        GPIOx->CRH |= tmp << ( (GPIO_PINx - 8) * 4 );
    }

    /* enable pull up/down */
    GPIOx->BSRR |= pull_updown;
}


/**
 * @brief    De-initializes the selected pin to its default state
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_deInit(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx)
{
    if(GPIO_PINx < 8)
    {
        GPIOx->CRL &= ~( 0x0f << ( GPIO_PINx * 4) );
        GPIOx->CRL |= 0x04 << (GPIO_PINx * 4);
        GPIOx->BSRR |= ( 1UL << (GPIO_PINx + 16) ) ;
    }
    else
    {
        GPIOx->CRH &= ~( 0x0f << ( (GPIO_PINx - 8) * 4) );
        GPIOx->CRH |= 0x04 << ((GPIO_PINx - 8) * 4);
        GPIOx->BSRR |= ( 1UL << (GPIO_PINx + 16) ) ;
    }
}


/**
 * @brief    Reads the state of GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   state of the pin, either GPIO_SET (1) or GPIO_RESET (0)
 */
uint8_t gpio_bitReadOutput(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx)
{
    uint32_t state;
    state = (GPIOx->ODR & (uint32_t)(1UL << GPIO_PINx) );

    return (uint8_t)(state >> GPIO_PINx);
}


/**
 * @brief    Reads the state of GPIO input pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   state of the pin, either GPIO_SET (1) or GPIO_RESET (0)
 */
uint8_t gpio_bitReadInput(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx)
{
    uint32_t state;
    state = (GPIOx->IDR & (uint32_t)(1UL << GPIO_PINx) );

    return (uint8_t)(state >> GPIO_PINx);
}


/**
 * @brief    Set the selected GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_bitSet(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx)
{
    GPIOx->BSRR |= 1UL << GPIO_PINx;
}


/**
 * @brief    Clear the selected GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_bitClear(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx)
{
    GPIOx->BSRR |= (1UL << (GPIO_PINx + 16UL));
}


/**
 * @brief    Toggles the state of selected GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_bitToggle(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx)
{
    uint8_t state;
    state = gpio_bitReadOutput(GPIOx, GPIO_PINx);

    if(state)
    {
        GPIOx->BSRR |= 1UL << (GPIO_PINx + 16);
    }
    else
    {
        GPIOx->BSRR |= 1UL << GPIO_PINx;
    }
}


/**
 * @brief    Reads the state of GPIO output port
 * @param    GPIOx: where x can be A, B, C and D.
 * @retval   state of GPIOx output port
 */
uint32_t gpio_portReadOutput(GPIO_TypeDef* GPIOx)
{
    return GPIOx->ODR;
}


/**
 * @brief    Reads the state of GPIO input port
 * @param    GPIOx: where x can be A, B, C and D.
 * @retval   state of GPIOx input port
 */
uint32_t gpio_portReadInput(GPIO_TypeDef* GPIOx)
{
    return GPIOx->IDR;
}