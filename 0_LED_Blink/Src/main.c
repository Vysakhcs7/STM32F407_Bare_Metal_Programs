/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include<stdint.h>
/*------------------------------------------------------------------------------------------*/

void delay(void);
/*------------------------------------------------------------------------------------------*/

//RCC AHB1ENR base address
#define RCC_BASE_ADDR				(0x40023800U)
#define RCC_AHB1ENR_OFFSET			(0x30U)
#define RCC_AHB1ENR_ADDR			(uint32_t *)(RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET)
//GPIOD base address
#define AHB1_BASE_ADDR 				(0x40020000U)
#define GPIOD_BASE_OFFSET 			(0x0C00U)
#define GPIOD_BASE_ADDR 			(AHB1_BASE_ADDR + GPIOD_BASE_OFFSET)
//GPIO port mode register base address
#define GPIOD_MODER_OFFSET			(0x00U)
#define GPIOD_MODER_BASE_ADDR		(uint32_t *) (GPIOD_BASE_ADDR + GPIOD_MODER_OFFSET)
//GPIO port output data register base address
#define GPIOD_ODR_OFFSET 			(0x14U)
#define GPIOD_ODR_BASE_ADDR  		(uint32_t *) (GPIOD_BASE_ADDR +  GPIOD_ODR_OFFSET)
int main(void)
{
	//Bit 3 GPIODEN: IO port D clock enable FROM RCC_AHB1ENR register
	uint32_t *pRCC = RCC_AHB1ENR_ADDR;
	*pRCC =  0x00000008;

	//GPIO port mode register
	//01: General purpose output mode.Pin 12 = MODER12[1:0](Bit number 25 = 0, 24 = 1)
	uint32_t *pGPIOD_MODER = GPIOD_MODER_BASE_ADDR;
	*pGPIOD_MODER = 0x01000000;

	uint32_t *pGPIOD_ODR = GPIOD_ODR_BASE_ADDR;

	while(1)
	{
		*pGPIOD_ODR = 0x1000;
		delay();
		*pGPIOD_ODR = 0x0000;
		delay();

	}
	return 0;
}
/*------------------------------------------------------------------------------------------*/

void delay(void)
{
	for(volatile int i =0; i<1000000; i++);
}
/*------------------------------------------------------------------------------------------*/
