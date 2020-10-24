/*
 * stm32f407xx_driver.h
 *
 *  Created on: Oct 16, 2020
 *      Author: Shubhankar Chaudhury
 *      
 *
 */

#ifndef STM32F407XX_DRIVER_H_
#define STM32F407XX_DRIVER_H_

//~~~~~~~~~~~~~~~~~~~~
// Includes
//~~~~~~~~~~~~~~~~~~~~
#include <stm32f407xx_gpio.h>

//~~~~~~~~~~~~~~~~~~~~
// Defines
//~~~~~~~~~~~~~~~~~~~~

//RCC
#define MA_RCC_BEG          0x40023800U         // Reset and Clock Control [AHB1]
#define MA_RCC_END          0x40023BFFU

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
#define MA_GPIOJ_BEG        0x40022400U         // General Purpose IO - Port J [AHB1]
#define MA_GPIOJ_END        0x400227FFU
#define MA_GPIOK_BEG        0x40022800U         // General Purpose IO - Port K [AHB1]
#define MA_GPIOK_END        0x40022BFFU

#define MAO_GPIOx_MODER     0X00            // GPIO port mode register
#define MAO_GPIOx_OTYPER    0x04            // GPIO port output type register
#define MAO_GPIOx_OSPEEDR   0x08            // GPIO output speed register
#define MAO_GPIOx_PUPDR     0x0C            // GPIO pull-up/pull-down register
#define MAO_GPIOx_IDR       0x10            // GPIO input data register
#define MAO_GPIOx_ODR       0x14            // GPIO output data register
#define MAO_GPIOx_BSRR      0x18            // GPIO port bit set/reset register
#define MAO_GPIOx_LCKR      0x1C            // GPIO port configuration lock register
#define MAO_GPIOx_AFRL      0x20            // GPIO port bit alternate function low register (7:0)
#define MAO_GPIOx_AFRH      0x24            // GPIO port bit alternate function high register (15:8)


#endif /* STM32F407XX_DRIVER_H_ */
