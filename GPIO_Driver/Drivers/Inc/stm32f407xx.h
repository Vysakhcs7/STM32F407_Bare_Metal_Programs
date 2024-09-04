/*
 * stm32f407xx.h
 *
 *  Created on: Apr 5, 2024
 *      Author: Vysakh C S
 *       Board: STM32F407VG DISCOVERY
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile


/*
 * Memory Map Definitions
 */

#define FLASH_BASE 				(0x08000000U)  /*! Flash memory base address */
#define SRAM1_BASE 				(0x20000000U)  /*! SRAM1 base address */
#define SRAM2_BASE 				(0x2001C000U)  /*! SRAM2 base address */
#define ROM 					(0x1FFF0000U)   /*! ROM base address */
#define SRAM 					(SRAM1_BASEADR) /*! SRAM base address (default to SRAM1) */






/*
 * Peripheral Base Addresses
 */

#define PERIPH_BASE				(0x40000000U)   /*! Peripheral base address */
#define APB1PERIPH_BASE 		(PERIPH_BASE)   /*! APB1 peripheral base address */
#define APB2PERIPH_BASE 		(APB1PERIPH_BASE + 0x10000U) /*! APB2 peripheral base address */
#define AHB1PERIPH_BASE 		(APB2PERIPH_BASE + 0x10000U) /*! AHB1 peripheral base address */
#define AHB2PERIPH_BASE 		(AHB1PERIPH_BASE + FFE0000U) /*! AHB2 peripheral base address */
#define AHB3PERIPH_BASE			(AHB2PERIPH_BASE + 50000000U) /*! AHB3 peripheral base address */


/*
 * RCC Base Addresses
 */

#define RCC_BASEADR  			(AHB1PERIPH_BASE + 0X3800U)


/*
 * GPIO Base Address
 */

#define GPIOA_BASEADR 			(AHB1PERIPH_BASE + 0X0000U) /*! GPIOA base address */
#define GPIOB_BASEADR 			(AHB1PERIPH_BASE + 0X0400U) /*! GPIOB base address */
#define GPIOC_BASEADR 			(AHB1PERIPH_BASE + 0X0800U) /*! GPIOC base address */
#define GPIOD_BASEADR 			(AHB1PERIPH_BASE + 0X0C00U) /*! GPIOD base address */
#define GPIOE_BASEADR 			(AHB1PERIPH_BASE + 0X1000U) /*! GPIOE base address */
#define GPIOF_BASEADR 			(AHB1PERIPH_BASE + 0X1400U) /*! GPIOF base address */
#define GPIOG_BASEADR 			(AHB1PERIPH_BASE + 0X1800U) /*! GPIOG base address */
#define GPIOH_BASEADR 			(AHB1PERIPH_BASE + 0X1C00U) /*! GPIOH base address */
#define GPIOI_BASEADR 			(AHB1PERIPH_BASE + 0X2000U) /*! GPIOI base address */


/*
 * Define a structure using typedef for GPIO registers
 */
typedef struct
{
    __vo uint32_t MODER;    // GPIO port mode register
    __vo uint32_t OTYPER;   // GPIO port output type register
    __vo uint32_t OSPEEDR;  // GPIO port output speed register
    __vo uint32_t PUPDR;    // GPIO port pull-up/pull-down register
    __vo uint32_t IDR;      // GPIO port input data register
    __vo uint32_t ODR;      // GPIO port output data register
    __vo uint32_t BSRR;     // GPIO port bit set/reset register
    __vo uint32_t LCKR;     // GPIO port configuration lock register
    __vo uint32_t AFR[2];   // GPIO alternate function register
} GPIO_RegDef_t;


typedef struct
{
    __vo uint32_t RCC_CR;
    __vo uint32_t RCC_PLLCFGR;
    __vo uint32_t RCC_CFGR;
    __vo uint32_t RCC_CIR;
    __vo uint32_t RCC_AHB1RSTR;
    __vo uint32_t RCC_AHB2RSTR;
    __vo uint32_t RCC_AHB3RSTR;
    uint32_t      RESERVED0;
    __vo uint32_t RCC_APB1RSTR;
    __vo uint32_t RCC_APB2RSTR;
    uint32_t      RESERVED1[2];
    __vo uint32_t RCC_AHB1ENR;
    __vo uint32_t RCC_AHB2ENR;
    __vo uint32_t RCC_AHB3ENR;
    __vo uint32_t RESERVED2;
    __vo uint32_t RCC_APB1ENR;
    __vo uint32_t RCC_APB2ENR;
    uint32_t      RESERVED3[2];
    __vo uint32_t RCC_AHB1LPENR;
    __vo uint32_t RCC_AHB2LPENR;
    __vo uint32_t RCC_AHB3LPENR;
    uint32_t      RESERVED4;
    __vo uint32_t RCC_APB1LPENR;
    __vo uint32_t RCC_APB2LPENR;
    uint32_t      RESERVED5[2];
    __vo uint32_t RCC_BDCR;
    __vo uint32_t RCC_CSR;
    uint32_t      RESERVED6[2];
    __vo uint32_t RCC_SSCGR;
    __vo uint32_t RCC_PLLI2SCFGR;
} RCC_RegDef_t;

/*
 *
 */
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEADR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASEADR)



#define RCC 				((RCC_RegDef_t*)RCC_BASEADR)

/*
 *
 */
#define GPIOA_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN() 	(RCC->RCC_AHB1ENR |= (1 << 8) )


/*
 *
 */
#define GPIOA_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~(1 << 8) )


#define GPIOA_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()      do { (RCC->RCC_AHB1RSTR |= (1 << 8)); (RCC->RCC_AHB1RSTR &= ~(1 << 8)); } while(0)





#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET

//
//GPIO_RegDef_t *pGPIOA= GPIOA;
//GPIO_RegDef_t *pGPIOB= GPIOB;
//GPIO_RegDef_t *pGPIOC= GPIOC;
//GPIO_RegDef_t *pGPIOD= GPIOD;
//GPIO_RegDef_t *pGPIOE= GPIOE;
//GPIO_RegDef_t *pGPIOF= GPIOF;
//GPIO_RegDef_t *pGPIOG= GPIOG;
//GPIO_RegDef_t *pGPIOH= GPIOH;
//GPIO_RegDef_t *pGPIOI= GPIOI;
















#include "STM32F407_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
