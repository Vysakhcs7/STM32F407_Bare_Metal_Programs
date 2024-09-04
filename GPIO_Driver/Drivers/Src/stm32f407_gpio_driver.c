/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Apr 6, 2024
 *      Author: Vysakh C S
 */

#include "STM32F407_gpio_driver.h"



/*
 * Function: GPIO_PeriClockControl
 */

/****************************************************************************
 * @fn             		- GPIO_PeriClockControl
 *
 * @brief          		- Enable or disable peripheral clock for a GPIO port
 *
 * @param[in] pGPIOx 	- Pointer to the GPIO port (GPIO_RegDef_t type)
 * @param[in] EnorDi 	- Enable or disable macro (ENABLE or DISABLE)
 *
 * @return         		- None
 *
 * @Note           		- None
****************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}
		else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}
		else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}
		else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}
		else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}
		else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}
		else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN();
			}

	}else
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
			else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
			else if(pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}
			else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
			else if(pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}
			else if(pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
			else if(pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}

	}

}



/*
 * Function: GPIO_Init
 */

/****************************************************************************
 * @fn             			- GPIO_Init
 *
 * @brief          			- Initialize GPIO pin
 *
 * @param[in] pGPIOHandle 	- Pointer to GPIO handle structure (GPIO_Handle_t type)
 *
 * @return         			- None
 *
 * @Note           			- None
****************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//configure the mode of the pin
	uint32_t temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{

	}
	temp = 0;
	//configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//configure the pupd
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR  &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER  &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	//temp = 0;

	//configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}



}


/*
 * Function: GPIO_DeInit
 */

/****************************************************************************
 * @fn             		- GPIO_DeInit
 *
 * @brief          		- Deinitialize GPIO pin
 *
 * @param[in] pGPIOx 	- Pointer to the GPIO port (GPIO_RegDef_t type)
 *
 * @return         		- None
 *
 * @Note           		- None
****************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
				{
				GPIOC_REG_RESET();
				}
			else if(pGPIOx == GPIOD)
				{
				GPIOD_REG_RESET();
				}
			else if(pGPIOx == GPIOE)
				{
				GPIOE_REG_RESET();
				}
			else if(pGPIOx == GPIOF)
				{
				GPIOF_REG_RESET();
				}
			else if(pGPIOx == GPIOG)
				{
				GPIOG_REG_RESET();
				}
			else if(pGPIOx == GPIOH)
				{
				GPIOH_REG_RESET();
				}
			else if(pGPIOx == GPIOI)
				{
				GPIOI_REG_RESET();
				}

}


/*
 * Function: GPIO_ReadFromInputPin
 */

/****************************************************************************
 * @fn             		- GPIO_ReadFromInputPin
 *
 * @brief          		- Read from a specific input pin
 *
 * @param[in] pGPIOx 	- Pointer to the GPIO port (GPIO_RegDef_t type)
 * @param[in] PinNumber 	- GPIO pin number
 *
 * @return         		- State of the input pin (0 or 1)
 *
 * @Note           		- None
****************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*
 * Function: GPIO_ReadFromInputPort
 */

/****************************************************************************
 * @fn             		- GPIO_ReadFromInputPort
 *
 * @brief          		- Read from an entire input port
 *
 * @param[in] pGPIOx 	- Pointer to the GPIO port (GPIO_RegDef_t type)
 *
 * @return         		- State of the input port (16-bit value)
 *
 * @Note           		- None
****************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}


/*
 * Function: GPIO_WriteToOutputPin
 */

/****************************************************************************
 * @fn             			- GPIO_WriteToOutputPin
 *
 * @brief          			- Write to a specific output pin
 *
 * @param[in] pGPIOx 		- Pointer to the GPIO port (GPIO_RegDef_t type)
 * @param[in] PinNumber 	- GPIO pin number
 * @param[in] Value 		- Value to be written (0 or 1)
 *
 * @return         			- None
 *
 * @Note           			- None
****************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);

	}else
	{
		pGPIOx->ODR |= ~(1 << PinNumber);
	}


}


/*
 * Function: GPIO_WriteToOutputPort
 */

/****************************************************************************
 * @fn             		- GPIO_WriteToOutputPort
 *
 * @brief          		- Write to an entire output port
 *
 * @param[in] pGPIOx 	- Pointer to the GPIO port (GPIO_RegDef_t type)
 * @param[in] Value 		- 16-bit value to be written to the port
 *
 * @return         		- None
 *
 * @Note           		- None
****************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;

}


/*
 * Function: GPIO_ToggleOutputPin
 */

/****************************************************************************
 * @fn             		- GPIO_ToggleOutputPin
 *
 * @brief          		- Toggle the state of a specific output pin
 *
 * @param[in] pGPIOx 	- Pointer to the GPIO port (GPIO_RegDef_t type)
 * @param[in] PinNumber 	- GPIO pin number
 *
 * @return         		- None
 *
 * @Note           		- None
****************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);

}


/*
 * Function: GPIO_IRQConfig
 */

/****************************************************************************
 * @fn             			- GPIO_IRQConfig
 *
 * @brief          			- Configure GPIO interrupt
 *
 * @param[in] IRQNumber 	- Interrupt request number
 * @param[in] IRQPriority 	- Interrupt priority
 * @param[in] EnorDi 		- Enable or disable macro (ENABLE or DISABLE)
 *
 * @return         			- None
 *
 * @Note           			- None
****************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}


/*
 * Function: GPIO_IRQHandling
 */

/****************************************************************************
 * @fn             			- GPIO_IRQHandling
 *
 * @brief          			- Handle GPIO interrupt
 *
 * @param[in] PinNumber 	- GPIO pin number
 *
 * @return         			- None
 *
 * @Note           			- None
****************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

