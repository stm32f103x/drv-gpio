/**
  ******************************************************************************
  * @file    main.c
  * @author  Marco, Roldan L.
  * @version v1.0
  * @date    September 07, 2021
  * @brief   GPIO driver
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

#include "stm32f10x.h"
#include "gpio.h"

void delay(uint32_t ms);

int main()
{
	GPIO_Init_t gpio;
	gpio.MODE = GPIO_OUTPUT_50MHZ;
	gpio.CONFIG = GPIO_GEN_OUT_PP;
	gpio_init(GPIOC, GPIO_PIN13, &gpio);

	while(1)
	{
		gpio_bitToggle(GPIOC, GPIO_PIN13);
		delay(1000);
	}
}

void delay(uint32_t ms)
{
	ms *= 1000;
	for(uint32_t i = 0; i < ms; i++);
}