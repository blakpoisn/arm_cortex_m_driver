/***************************************************************************************************
 * @file      stm32f407xx_driver.h
 * @brief     API Header for Driver header file.
 *            
 * @author    Shubhankar Chaudhury
 * @date      28 Feb 2021
 **************************************************************************************************/

#ifndef STM32F407XX_DRIVER_H_
#define STM32F407XX_DRIVER_H_

// Includes ---------------------------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>

// Defines ----------------------------------------------------------------------------------------

//Common
#define DISABLE     0x00
#define ENABLE      0x01
#define OFF         DISABLE
#define ON          ENABLE
#define FALSE       DISABLE
#define TRUE        ENABLE


//AHB1
#define MA_RCC_BEG          0x40023800U         /*!< Reset and Clock Control [AHB1] */
#define MA_CRC_BEG          0x40023000U         /*!< Cyclic Redundancy Check [AHB1] */
#define MA_GPIOA_BEG        0x40020000U         /*!< General Purpose IO - Port A [AHB1] */
#define MA_GPIOB_BEG        0x40020400U         /*!< General Purpose IO - Port B [AHB1] */
#define MA_GPIOC_BEG        0x40020800U         /*!< General Purpose IO - Port C [AHB1] */
#define MA_GPIOD_BEG        0x40020C00U         /*!< General Purpose IO - Port D [AHB1] */
#define MA_GPIOE_BEG        0x40021000U         /*!< General Purpose IO - Port E [AHB1] */
#define MA_GPIOF_BEG        0x40021400U         /*!< General Purpose IO - Port F [AHB1] */
#define MA_GPIOG_BEG        0x40021800U         /*!< General Purpose IO - Port G [AHB1] */
#define MA_GPIOH_BEG        0x40021C00U         /*!< General Purpose IO - Port H [AHB1] */
#define MA_GPIOI_BEG        0x40022000U         /*!< General Purpose IO - Port I [AHB1] */

//APB2
#define MA_EXTI_BEG         0x40013C00U         /*!< External interrupt/event controller */
#define MA_SYSCFG_BEG       0x40013800U         /*!< System configuration */

//Core
// #define MA_NVIC_BEG         0xE000E100U

// Structures --------------------------------------------------------------------------------------
/**
 *  @brief Register deffinition for GPIO memory map.
 */
typedef struct {
  volatile uint32_t MODER;      /*!< Offset: 0x00 | GPIO port mode register */
  volatile uint32_t OTYPER;     /*!< Offset: 0x04 | GPIO port output type register */
  volatile uint32_t OSPEEDR;    /*!< Offset: 0x08 | GPIO output speed register */
  volatile uint32_t PUPDR;      /*!< Offset: 0x0C | GPIO pull-up/pull-down register */
  volatile uint32_t IDR;        /*!< Offset: 0x10 | GPIO input data register */
  volatile uint32_t ODR;        /*!< Offset: 0x14 | GPIO output data register */
  volatile uint32_t BSRR;       /*!< Offset: 0x18 | GPIO port bit set/reset register */
  volatile uint32_t LCKR;       /*!< Offset: 0x1C | GPIO port configuration lock register */
  volatile uint32_t AFR[2];     /*!< Offset: 0x20 | GPIO port bit alternate function register */
} gpio_regdef_t;

/**
 *  @brief Register deffinition for Reset & Clock Control register memory map.
 */
typedef struct {
  volatile uint32_t CR;         /*!< Offset: 0x00 | RCC clock control register. */
  volatile uint32_t PLLCFGR;    /*!< Offset: 0x04 | RCC PLL configuration register. */
  volatile uint32_t CFGR;       /*!< Offset: 0x08 | RCC clock configuration register. */
  volatile uint32_t CIR;        /*!< Offset: 0x0C | RCC clock interrupt register. */
  volatile uint32_t AHB1RSTR;   /*!< Offset: 0x10 | RCC AHB1 peripheral reset register. */
  volatile uint32_t AHB2RSTR;   /*!< Offset: 0x14 | RCC AHB2 peripheral reset register. */
  volatile uint32_t AHB3RSTR;   /*!< Offset: 0x18 | RCC AHB3 peripheral reset register. */
  volatile uint32_t RES0;       /*!< Offset: 0x1C | Reserved. */
  volatile uint32_t APB1RSTR;   /*!< Offset: 0x20 | RCC APB1 peripheral reset register. */
  volatile uint32_t APB2RSTR;   /*!< Offset: 0x24 | RCC APB2 peripheral reset register. */
  volatile uint32_t RES1;       /*!< Offset: 0x28 | Reserved. */
  volatile uint32_t RES2;       /*!< Offset: 0x2C | Reserved. */
  volatile uint32_t AHB1ENR;    /*!< Offset: 0x30 | RCC AHB1 peripheral clock enable register. */
  volatile uint32_t AHB2ENR;    /*!< Offset: 0x34 | RCC AHB2 peripheral clock enable register. */
  volatile uint32_t AHB3ENR;    /*!< Offset: 0x38 | RCC AHB3 peripheral clock enable register. */
  volatile uint32_t RES3;       /*!< Offset: 0x3C | Reserved. */
  volatile uint32_t APB1ENR;    /*!< Offset: 0x40 | RCC APB1 peripheral clock enable register. */
  volatile uint32_t APB2ENR;    /*!< Offset: 0x44 | RCC APB2 peripheral clock enable register. */
  volatile uint32_t RES4;       /*!< Offset: 0x48 | Reserved. */
  volatile uint32_t RES5;       /*!< Offset: 0x4C | Reserved. */
  volatile uint32_t AHB1LPENR;  /*!< Offset: 0x50 | RCC AHB1 peripheral low power clock enable register. */
  volatile uint32_t AHB2LPENR;  /*!< Offset: 0x54 | RCC AHB2 peripheral low power clock enable register. */
  volatile uint32_t AHB3LPENR;  /*!< Offset: 0x58 | RCC AHB3 peripheral low power clock enable register. */
  volatile uint32_t RES6;       /*!< Offset: 0x5C | Reserved. */
  volatile uint32_t APB1LPENR;  /*!< Offset: 0x60 | RCC APB1 peripheral low power clock enable reg. */
  volatile uint32_t APB2LPENR;  /*!< Offset: 0x64 | RCC APB2 peripheral low power clock enable reg. */
  volatile uint32_t RES7;       /*!< Offset: 0x68 | Reserved. */
  volatile uint32_t RES8;       /*!< Offset: 0x6C | Reserved. */
  volatile uint32_t BDCR;       /*!< Offset: 0x70 | RCC backup domain control register. */
  volatile uint32_t CSR;        /*!< Offset: 0x74 | RCC clock control & status register. */
  volatile uint32_t RES9;       /*!< Offset: 0x78 | Reserved. */
  volatile uint32_t RES10;      /*!< Offset: 0x7C | Reserved. */
  volatile uint32_t SSCGR;      /*!< Offset: 0x80 | RCC spread spectrum clock generation register. */
  volatile uint32_t PLLI2SCFGR; /*!< Offset: 0x84 | RCC PLLI2S configuration register. */
  volatile uint32_t PLLSAICFGR; /*!< Offset: 0x88 | RCC PLLSAI configuration register. */
  volatile uint32_t DCKCFGR;    /*!< Offset: 0x8C | RCC Dedicated Clock Configuration register. */
} rcc_regdef_t;

/**
 *  @brief Register deffinition for EXTI registers memory map.
 */
typedef struct {
  volatile uint32_t IMR;        /*!< Offset: 0x00 | Interrupt mask register. */
  volatile uint32_t EMR;        /*!< Offset: 0x04 | Event mask register. */
  volatile uint32_t RTSR;       /*!< Offset: 0x08 | Rising trigger selection register. */
  volatile uint32_t FTSR;       /*!< Offset: 0x0C | Falling trigger selection register. */
  volatile uint32_t SWIER;      /*!< Offset: 0x10 | Software interrupt event register. */
  volatile uint32_t PR;         /*!< Offset: 0x14 | Pending Register. */
} exti_regdef_t;

/**
 *  @brief Register deffinition for System Configuration registers memory map.
 */
typedef struct {
  volatile uint32_t MEMRMP;     /*!< Offset: 0x00 | Memory remap register. */
  volatile uint32_t PMC;        /*!< Offset: 0x04 | Peripheral mode configuration register. */
  volatile uint32_t EXTICR[4];  /*!< Offset: 0x08 | EXTI configuration register. */
  volatile uint32_t RES0;       /*!< Offset: 0x18 | Reserved. */
  volatile uint32_t RES1;       /*!< Offset: 0x1C | Reserved. */
  volatile uint32_t PR;         /*!< Offset: 0x20 | Compensation cell control register. */
} syscfg_regdef_t;

// Peripheral Definitions --------------------------------------------------------------------------
#define RCC_MEM_MAP     ((rcc_regdef_t *)MA_RCC_BEG)        /*!< RCC register memory map access pointer. */
#define EXTI_MEM_MAP    ((exti_regdef_t *)MA_EXTI_BEG)      /*!< EXTI register memory map access pointer. */
#define SYSCFG_MEM_MAP  ((syscfg_regdef_t *)MA_SYSCFG_BEG)  /*!< System Configuration register memory map access pointer. */

#endif /* STM32F407XX_DRIVER_H_ */