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

// Enumerations ------------------------------------------------------------------------------------
/**
 *  @brief Enumeration for device specific (stm32f407xx) IRQ numbers.
 */
typedef enum {
  IRQ_WWDG = 0,                /*!< IRQ 0 - Window Watchdog Interrupt */
  IRQ_PVD,                     /*!< IRQ 1 - PVD through EXTI line detection interrupt */
  IRQ_TAMP_STAMP,              /*!< IRQ 2 - Tamper and Timestamp interrupts through EXTI line */
  IRQ_RTC_WKUP,                /*!< IRQ 3 - RTC wakeup interrupt through EXTI line */
  IRQ_FLASH,                   /*!< IRQ 4 - Flash global interrupt */
  IRQ_RCC,                     /*!< IRQ 5 - RCC global interrupt */
  IRQ_EXTI0,                   /*!< IRQ 6 - EXTI line 0 interrupt */
  IRQ_EXTI1,                   /*!< IRQ 7 - EXTI line 1 interrupt */
  IRQ_EXTI2,                   /*!< IRQ 8 - EXTI line 2 interrupt */
  IRQ_EXTI3,                   /*!< IRQ 9 - EXTI line 3 interrupt */
  IRQ_EXTI4,                   /*!< IRQ 10 - EXTI line 4 interrupt */
  IRQ_DMA1_STREAM0,            /*!< IRQ 11 - DMA1 Stream 0 global interrupt */
  IRQ_DMA1_STREAM1,            /*!< IRQ 12 - DMA1 Stream 1 global interrupt */
  IRQ_DMA1_STREAM2,            /*!< IRQ 13 - DMA1 Stream 2 global interrupt */
  IRQ_DMA1_STREAM3,            /*!< IRQ 14 - DMA1 Stream 3 global interrupt */
  IRQ_DMA1_STREAM4,            /*!< IRQ 15 - DMA1 Stream 4 global interrupt */
  IRQ_DMA1_STREAM5,            /*!< IRQ 16 - DMA1 Stream 5 global interrupt */
  IRQ_DMA1_STREAM6,            /*!< IRQ 17 - DMA1 Stream 6 global interrupt */
  IRQ_ADC,                     /*!< IRQ 18 - ADC1, ADC2 and ADC3 global interrupt */
  IRQ_CAN1_TX,                 /*!< IRQ 19 - CAN1 TX interrupt */
  IRQ_CAN1_RX0,                /*!< IRQ 20 - CAN1 RX0 interrupt */
  IRQ_CAN1_RX1,                /*!< IRQ 21 - CAN1 RX1 interrupt */
  IRQ_CAN1_SCE,                /*!< IRQ 22 - CAN1 SCE interrupt */
  IRQ_EXTI9_5,                 /*!< IRQ 23 - EXTI Line 5 to 9 interrupt */
  IRQ_TIM1_BRK_TIM9,           /*!< IRQ 24 - TIM1 Break interrupt and TIM9 global interrupt */
  IRQ_TIM1_UP_TIM10,           /*!< IRQ 25 - TIM1 Update interrupt and TIM10 global interrupt */
  IRQ_TIM1_TRG_COM_TIM11,      /*!< IRQ 26 - TIM1 Trigger and Commutation interrupt and TIM11 global interrupt */
  IRQ_TIM1_CC,                 /*!< IRQ 27 - TIM1 Capture Compare interrupt */
  IRQ_TIM2,                    /*!< IRQ 28 - TIM2 global interrupt */
  IRQ_TIM3,                    /*!< IRQ 29 - TIM3 global interrupt */
  IRQ_TIM4,                    /*!< IRQ 30 - TIM4 global interrupt */
  IRQ_I2C1_EV,                 /*!< IRQ 31 - I2C1 event interrupt */
  IRQ_I2C1_ER,                 /*!< IRQ 32 - I2C1 error interrupt */
  IRQ_I2C2_EV,                 /*!< IRQ 33 - I2C2 event interrupt */
  IRQ_I2C2_ER,                 /*!< IRQ 34 - I2C2 error interrupt */
  IRQ_SPI1,                    /*!< IRQ 35 - SPI1 global interrupt */
  IRQ_SPI2,                    /*!< IRQ 36 - SPI2 global interrupt */
  IRQ_USART1,                  /*!< IRQ 37 - USART1 global interrupt */
  IRQ_USART2,                  /*!< IRQ 38 - USART2 global interrupt */
  IRQ_USART3,                  /*!< IRQ 39 - USART3 global interrupt */
  IRQ_EXTI10_15,               /*!< IRQ 40 - EXTI Line 10 to 15 interrupts */
  IRQ_RTC_ALARM,               /*!< IRQ 41 - RTC Alarm (A and B) through EXTI line interrupt */
  IRQ_OTG_FS_WKUP,             /*!< IRQ 42 - USB On-The-Go FS wakeup through EXTI line interrupt */
  IRQ_TIM8_BRK_TIM12,          /*!< IRQ 43 - TIM8 Break interrupt and TIM12 global interrupt */
  IRQ_TIM8_UP_TIM13,           /*!< IRQ 44 - TIM8 Update interrupt and TIM13 global interrupt */
  IRQ_TIM8_TRG_COM_TIM14,      /*!< IRQ 45 - TIM8 Trigger and Commutation interrupt and TIM14 global interrupt */
  IRQ_TIM8_CC,                 /*!< IRQ 46 - TIM8 Captur Compare interrupt */
  IRQ_DMA1_STREAM7,            /*!< IRQ 47 - DMA1 Stream 7 global interrupt */
  IRQ_FSMC,                    /*!< IRQ 48 - FSMC global interrupt */
  IRQ_SDIO,                    /*!< IRQ 49 - SDIO global interrupt */
  IRQ_TIM5,                    /*!< IRQ 50 - TIM5 global interrupt */
  IRQ_SPI3,                    /*!< IRQ 51 - SPI3 global interrupt */
  IRQ_USART4,                  /*!< IRQ 52 - USART4 global interrupt */
  IRQ_USART5,                  /*!< IRQ 53 - USART5 global interrupt */
  IRQ_TIM6_DAC,                /*!< IRQ 54 - TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts */
  IRQ_TIM7,                    /*!< IRQ 55 - TIM7 global interrupt */
  IRQ_DMA2_STREAM0,            /*!< IRQ 56 - DMA2 Stream 0 global interrupt */
  IRQ_DMA2_STREAM1,            /*!< IRQ 57 - DMA2 Stream 1 global interrupt */
  IRQ_DMA2_STREAM2,            /*!< IRQ 58 - DMA2 Stream 2 global interrupt */
  IRQ_DMA2_STREAM3,            /*!< IRQ 59 - DMA2 Stream 3 global interrupt */
  IRQ_DMA2_STREAM4,            /*!< IRQ 60 - DMA2 Stream 4 global interrupt */
  IRQ_ETH,                     /*!< IRQ 61 - Ethernet global interrupt */
  IRQ_ETH_WKUP,                /*!< IRQ 62 - Ethernet wakeup through EXTI line interrupt */
  IRQ_CAN2_TX,                 /*!< IRQ 63 - CAN2 TX interrupt */
  IRQ_CAN2_RX0,                /*!< IRQ 64 - CAN2 RX0 interrupt */
  IRQ_CAN2_RX1,                /*!< IRQ 65 - CAN2 RX1 interrupt */
  IRQ_CAN2_SCE,                /*!< IRQ 66 - CAN2 SCE interrupt */
  IRQ_OTG_FS,                  /*!< IRQ 67 - USB On-The-Go FS global interrupt */
  IRQ_DMA2_STREAM5,            /*!< IRQ 68 - DMA2 Stream 5 global interrupt */
  IRQ_DMA2_STREAM6,            /*!< IRQ 69 - DMA2 Stream 6 global interrupt */
  IRQ_DMA2_STREAM7,            /*!< IRQ 70 - DMA2 Stream 7 global interrupt */
  IRQ_USART6,                  /*!< IRQ 71 - USART6 global interrupt */
  IRQ_I2C3_EV,                 /*!< IRQ 72 - I2C3 event interrupt */
  IRQ_I2C3_ER,                 /*!< IRQ 73 - I2C3 error interrupt */
  IRQ_OTG_HS_EP1_OUT,          /*!< IRQ 74 - USB On-The-Go HS End Point 1 Out global interrupt */
  IRQ_OTG_HS_EP1_IN,           /*!< IRQ 75 - USB On-The-Go HS End Point 1 In global interrupt */
  IRQ_OTG_HS_WKUP,             /*!< IRQ 76 - USB On-The-Go HS Wakeup through EXTI interrupt */
  IRQ_OTG_HS,                  /*!< IRQ 77 - USB On-The-Go HS global interrupt*/
  IRQ_DCMI,                    /*!< IRQ 78 - DCMI global interrupt */
  IRQ_CRYP,                    /*!< IRQ 79 - Crypto global interrupt */
  IRQ_HASH_RNG,                /*!< IRQ 80 - Hash and Rng global interrupt */
  IRQ_FPU                      /*!< IRQ 81 - FPU global interrupt */
} irq_num_t;

// Structures --------------------------------------------------------------------------------------
/**
 *  @brief Register definition for GPIO memory map.
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
 *  @brief Register definition for Reset & Clock Control register memory map.
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
 *  @brief Register definition for EXTI registers memory map.
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
 *  @brief Register definition for System Configuration registers memory map.
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