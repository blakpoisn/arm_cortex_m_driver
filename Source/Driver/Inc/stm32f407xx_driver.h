/**
 * @file      : stm32f407xx_driver.h
 * @brief     : API Header for Driver implementation
 *
 * @author    : Shubhankar Chaudhury
 */

#ifndef STM32F407XX_DRIVER_H_
#define STM32F407XX_DRIVER_H_

/**
 *  Includes
 */

#include <stdint.h>
#include <stddef.h>
#include <stm32f407xx_gpio.h>

/**
 *  Defines
 */

//Commons
#define DISABLE     0x00
#define ENABLE      0x01
#define OFF         DISABLE
#define ON          ENABLE
#define ADDR_OFFSET(x)      (x/4)               // Offset for 32 bit pointers

//RCC
#define MA_RCC_BEG          0x40023800U         // Reset and Clock Control [AHB1]
#define MA_RCC_END          0x40023BFFU

#define MO_RCC_CR           0x00                // RCC clock control register
#define MO_RCC_PLLCFGR      0x04                // RCC PLL configuration register
#define MO_RCC_CFGR         0x08                // RCC clock configuration register
#define MO_RCC_CIR          0x0C                // RCC clock interrupt register
#define MO_RCC_AHB1RSTR     0x10                // RCC AHB1 peripheral reset register
#define MO_RCC_AHB2RSTR     0x14                // RCC AHB2 peripheral reset register
#define MO_RCC_AHB3RSTR     0x18                // RCC AHB3 peripheral reset register
#define MO_RCC_APB1RSTR     0x20                // RCC APB1 peripheral reset register
#define MO_RCC_APB2RSTR     0x24                // RCC APB2 peripheral reset register
#define MO_RCC_AHB1ENR      0x30                // RCC AHB1 peripheral clock enable register
#define MO_RCC_AHB2ENR      0x34                // RCC AHB2 peripheral clock enable register
#define MO_RCC_AHB3ENR      0x38                // RCC AHB3 peripheral clock enable register
#define MO_RCC_APB1ENR      0x40                // RCC APB1 peripheral clock enable register
#define MO_RCC_APB2ENR      0x44                // RCC APB2 peripheral clock enable register
#define MO_RCC_AHB1LPENR    0x50                // RCC AHB1 peripheral low power clock enable reg
#define MO_RCC_AHB2LPENR    0x54                // RCC AHB2 peripheral low power clock enable reg
#define MO_RCC_AHB3LPENR    0x58                // RCC AHB3 peripheral low power clock enable reg
#define MO_RCC_APB1LPENR    0x60                // RCC APB1 peripheral low power clock enable reg
#define MO_RCC_APB2LPENR    0x64                // RCC APB2 peripheral low power clock enable reg
#define MO_RCC_BDCR         0x70                // RCC backup domain control register
#define MO_RCC_CSR          0x74                // RCC clock control & status register
#define MO_RCC_SSCGR        0x80                // RCC spread spectrum clock generation register
#define MO_RCC_PLLI2SCFGR   0x84                // RCC PLLI2S configuration register

//CRC
#define MA_CRC_BEG          0x40023000U         // Cyclic Redundancy Check [AHB1]
#define MA_CRC_END          0x400233FFU

//GPIO
#define MA_GPIOA_BEG        0x40020000U         // General Purpose IO - Port A [AHB1]
#define MA_GPIOA_END        0x400203FFU
#define MA_GPIOB_BEG        0x40020400U         // General Purpose IO - Port B [AHB1]
#define MA_GPIOB_END        0x400207FFU
#define MA_GPIOC_BEG        0x40020800U         // General Purpose IO - Port C [AHB1]
#define MA_GPIOC_END        0x40020BFFU
#define MA_GPIOD_BEG        0x40020C00U         // General Purpose IO - Port D [AHB1]
#define MA_GPIOD_END        0x40020FFFU
#define MA_GPIOE_BEG        0x40021000U         // General Purpose IO - Port E [AHB1]
#define MA_GPIOE_END        0x400213FFU
#define MA_GPIOF_BEG        0x40021400U         // General Purpose IO - Port F [AHB1]
#define MA_GPIOF_END        0x400217FFU
#define MA_GPIOG_BEG        0x40021800U         // General Purpose IO - Port G [AHB1]
#define MA_GPIOG_END        0x40021BFFU
#define MA_GPIOH_BEG        0x40021C00U         // General Purpose IO - Port H [AHB1]
#define MA_GPIOH_END        0x40021FFFU
#define MA_GPIOI_BEG        0x40022000U         // General Purpose IO - Port I [AHB1]
#define MA_GPIOI_END        0x400223FFU

#define MO_GPIOx_MODER      0X00                // GPIO port mode register
#define MO_GPIOx_OTYPER     0x04                // GPIO port output type register
#define MO_GPIOx_OSPEEDR    0x08                // GPIO output speed register
#define MO_GPIOx_PUPDR      0x0C                // GPIO pull-up/pull-down register
#define MO_GPIOx_IDR        0x10                // GPIO input data register
#define MO_GPIOx_ODR        0x14                // GPIO output data register
#define MO_GPIOx_BSRR       0x18                // GPIO port bit set/reset register
#define MO_GPIOx_LCKR       0x1C                // GPIO port configuration lock register
#define MO_GPIOx_AFRL       0x20                // GPIO port bit alternate function low register (7:0)
#define MO_GPIOx_AFRH       0x24                // GPIO port bit alternate function high register (15:8)


#endif /* STM32F407XX_DRIVER_H_ */
