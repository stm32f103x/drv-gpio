/**
  ******************************************************************************
  * @file    gpio.h
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

#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f10x.h"



/* GPIO Defines */


/* GPIO_MODE */
#define GPIO_MODE_INPUT          (uint32_t)0x00
#define GPIO_MODE_OUTPUT_10MHZ   (uint32_t)0x01
#define GPIO_MODE_OUTPUT_2MHZ    (uint32_t)0x02
#define GPIO_MODE_OUTPUT_50MHZ   (uint32_t)0x03

/* GPIO_CONFIG - OUTPUT */
#define GPIO_CONF_GEN_OUT_PP     (uint32_t)0x00
#define GPIO_CONF_GEN_OUT_OD     (uint32_t)0x01
#define GPIO_CONF_ALT_FUN_PP     (uint32_t)0x02
#define GPIO_CONF_ALT_FUN_OD     (uint32_t)0x03

/* GPIO_CONFIG - INPUT */
#define GPIO_CONF_IN_ANALOG      (uint32_t)0x00
#define GPIO_CONF_IN_FLOATING    (uint32_t)0x01
#define GPIO_CONF_IN_PULLUP      (uint32_t)0x02
#define GPIO_CONF_IN_PULLDOWN    (uint32_t)0x03

/* PINx */
#define GPIO_PIN0           (uint8_t)( 0 )
#define GPIO_PIN1           (uint8_t)( 1 )
#define GPIO_PIN2           (uint8_t)( 2 )
#define GPIO_PIN3           (uint8_t)( 3 )
#define GPIO_PIN4           (uint8_t)( 4 )
#define GPIO_PIN5           (uint8_t)( 5 )
#define GPIO_PIN6           (uint8_t)( 6 )
#define GPIO_PIN7           (uint8_t)( 7 )
#define GPIO_PIN8           (uint8_t)( 8 )
#define GPIO_PIN9           (uint8_t)( 9 )
#define GPIO_PIN10          (uint8_t)( 10 )
#define GPIO_PIN11          (uint8_t)( 11 )
#define GPIO_PIN12          (uint8_t)( 12 )
#define GPIO_PIN13          (uint8_t)( 13 )
#define GPIO_PIN14          (uint8_t)( 14 )
#define GPIO_PIN15          (uint8_t)( 15 )


/**
 * @brief    Structure to set the desired pin functionality
 */
typedef struct
{
    uint32_t MODE;          /* Specifies the GPIO pin mode */
    uint32_t CONFIG;        /* Specifies the GPIO pin configuration */
} GPIO_Init_t;

/**
 * @brief    GPIO state type
 */
typedef enum { GPIO_RESET = (uint8_t)0, GPIO_SET = (uint8_t)!GPIO_RESET} GPIO_State_t;



/* GPIO function APIs */

/**
 * @brief    Initializes a GPIO
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @param    gpio_init: pointer to GPIO_Init_t structure where desired GPIO
 *                      function was initialized.
 * @retval   none
 */
void gpio_init(GPIO_TypeDef* GPIOx, uint8_t pin, GPIO_Init_t* gpio_init);


/**
 * @brief    De-initializes the selected pin to its default state
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_deInit(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx);


/**
 * @brief    Reads the state of GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   state of the pin, either GPIO_SET (1) or GPIO_RESET (0)
 */
uint8_t gpio_bitReadOutput(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx);


/**
 * @brief    Reads the state of GPIO input pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   state of the pin, either GPIO_SET (1) or GPIO_RESET (0)
 */
uint8_t gpio_bitReadInput(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx);


/**
 * @brief    Set the selected GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_bitSet(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx);


/**
 * @brief    Clear the selected GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_bitClear(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx);


/**
 * @brief    Toggles the state of selected GPIO output pin
 * @param    GPIOx: where x can be A, B, C and D.
 * @param    GPIO_PINx: where x can be any number from 0..15
 * @retval   none
 */
void gpio_bitToggle(GPIO_TypeDef* GPIOx, uint8_t GPIO_PINx);


/**
 * @brief    Reads the state of GPIO output port
 * @param    GPIOx: where x can be A, B, C and D.
 * @retval   state of GPIOx output port
 */
uint32_t gpio_portReadOutput(GPIO_TypeDef* GPIOx);


/**
 * @brief    Reads the state of GPIO input port
 * @param    GPIOx: where x can be A, B, C and D.
 * @retval   state of GPIOx input port
 */
uint32_t gpio_portReadInput(GPIO_TypeDef* GPIOx);


#endif /* __GPIO_H */