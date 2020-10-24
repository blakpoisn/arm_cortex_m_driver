/*
 * stm32f407xx_gpio.h
 *
 *  Created on: 24-Oct-2020
 *      Author: Shubhankar Chaudhury
 */

#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

/* 
 *  Includes
 */
#include <stm32f407xx_driver.h>

/* 
 *  Defines
 */

//PIN MODE [2 bit]
#define OP_GPIO_PINMODE_IN          0x00            // pin Input mode
#define OP_GPIO_PINMODE_OUT         0x01            // pin General output mode
#define OP_GPIO_PINMODE_ALT         0x02            // pin Alternate function mode
#define OP_GPIO_PINMODE_ANALOG      0x03            // pin analog mode

//OUTPUT TYPE [1 bit]
#define OP_GPIO_OTYPE_PUSHPULL      0x00            // pin output type push-pull
#define OP_GPIO_OTYPE_OPDRAIN       0x01            // pin output type open drain

//OUTPUT SPEED [2 bit]
#define OP_GPIO_OSPEED_LOW          0x00            // pin output speed low
#define OP_GPIO_OSPEED_MED          0x01            // pin output speed medium
#define OP_GPIO_OSPEED_HI           0x02            // pin output speed high
#define OP_GPIO_OSPEED_VHI          0x03            // pin output speed very high

//PIN PULL UP PULL DOWN [2 bit]
#define OP_GPIO_PUPD_NOPUPD         0x00            // pin no pull up or pulled down
#define OP_GPIO_PUPD_PU             0x00            // pin pulled up
#define OP_GPIO_PUPD_PD             0x00            // pin pulled down

/* 
 *  Function Prototypes
 */



#endif /* STM32F407XX_GPIO_H_ */
