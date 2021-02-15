/***************************************************************************************************
 * @file      : stm32f407xx_gpio.h
 * @brief     : API Header for GPIO implementation
 * 
 * @author    : Shubhankar Chaudhury
 * @date      : 00 Xxx 20xx
 **************************************************************************************************/

#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

/// Includes ---------------------------------------------------------------------------------------

#include <stm32f407xx_driver.h>

/// Defines ----------------------------------------------------------------------------------------

//GPIO NAME
#define OP_GPIO_PORT_A              0               //PortA
#define OP_GPIO_PORT_B              1               //PortB
#define OP_GPIO_PORT_C              2               //PortC
#define OP_GPIO_PORT_D              3               //PortD
#define OP_GPIO_PORT_E              4               //PortE
#define OP_GPIO_PORT_F              5               //PortF
#define OP_GPIO_PORT_G              6               //PortG
#define OP_GPIO_PORT_H              7               //PortH
#define OP_GPIO_PORT_I              8               //PortI

//GPIO PIN number
#define OP_GPIO_PIN_0               0               //Pin 0
#define OP_GPIO_PIN_1               1               //Pin 1
#define OP_GPIO_PIN_2               2               //Pin 2
#define OP_GPIO_PIN_3               3               //Pin 3
#define OP_GPIO_PIN_4               4               //Pin 4
#define OP_GPIO_PIN_5               5               //Pin 5
#define OP_GPIO_PIN_6               6               //Pin 6
#define OP_GPIO_PIN_7               7               //Pin 7
#define OP_GPIO_PIN_8               8               //Pin 8
#define OP_GPIO_PIN_9               9               //Pin 9
#define OP_GPIO_PIN_10              10              //Pin 10
#define OP_GPIO_PIN_11              11              //Pin 11
#define OP_GPIO_PIN_12              12              //Pin 12
#define OP_GPIO_PIN_13              13              //Pin 13
#define OP_GPIO_PIN_14              14              //Pin 14
#define OP_GPIO_PIN_15              15              //Pin 15

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
#define OP_GPIO_PUPD_PU             0x01            // pin pulled up
#define OP_GPIO_PUPD_PD             0x02            // pin pulled down

//GPIO Alternate Function
#define OP_GPIO_ALTFUNC_AF0         0x00            // pin alternate function type 0 
#define OP_GPIO_ALTFUNC_AF1         0x01            // pin alternate function type 1 
#define OP_GPIO_ALTFUNC_AF2         0x02            // pin alternate function type 2 
#define OP_GPIO_ALTFUNC_AF3         0x03            // pin alternate function type 3 
#define OP_GPIO_ALTFUNC_AF4         0x04            // pin alternate function type 4 
#define OP_GPIO_ALTFUNC_AF5         0x05            // pin alternate function type 5 
#define OP_GPIO_ALTFUNC_AF6         0x06            // pin alternate function type 6 
#define OP_GPIO_ALTFUNC_AF7         0x07            // pin alternate function type 7 
#define OP_GPIO_ALTFUNC_AF8         0x08            // pin alternate function type 8 
#define OP_GPIO_ALTFUNC_AF9         0x09            // pin alternate function type 9 
#define OP_GPIO_ALTFUNC_AF10        0x0A            // pin alternate function type 10
#define OP_GPIO_ALTFUNC_AF11        0x0B            // pin alternate function type 11
#define OP_GPIO_ALTFUNC_AF12        0x0C            // pin alternate function type 12
#define OP_GPIO_ALTFUNC_AF13        0x0D            // pin alternate function type 13
#define OP_GPIO_ALTFUNC_AF14        0x0E            // pin alternate function type 14
#define OP_GPIO_ALTFUNC_AF15        0x0F            // pin alternate function type 15

//trigger options
#define OP_GPIO_INTR_FALL           0x00            // Falling edge interrupt trigger
#define OP_GPIO_INTR_RISE           0x01            // Rising edge interrupt trigger
#define OP_GPIO_INTR_BOTH           0x02            // Both edge interrupt trigger
#define OP_GPIO_INTR_NONE           0x03            // Disable interrupt trigger

#define OP_GPIO_EVENT_FALL          0x00            // Falling edge interrupt trigger
#define OP_GPIO_EVENT_RISE          0x01            // Rising edge interrupt trigger
#define OP_GPIO_EVENT_BOTH          0x02            // Both edge interrupt trigger
#define OP_GPIO_EVENT_NONE          0x03            // Disable interrupt trigger

/// Structures -------------------------------------------------------------------------------------

typedef struct {
    uint8_t GPIO_port;
    uint8_t GPIO_pin;
    uint8_t GPIO_pinmode;
    uint8_t GPIO_otype;
    uint8_t GPIO_ospeed;
    uint8_t GPIO_pushpull;
    uint8_t GPIO_atlFunc;
} gpio_handle_t;

/// Function Prototypes ----------------------------------------------------------------------------

gpio_handle_t gpio_handle_init(uint8_t  port, uint8_t pin);
uint8_t gpio_pin_init(gpio_handle_t *gpio_handle);
void gpio_port_enable(uint8_t gpio_port, uint8_t state);
void gpio_pin_write(gpio_handle_t *gpio_handle, uint8_t state);
uint8_t gpio_pin_read(gpio_handle_t *gpio_handle);
void gpio_stage_intr(gpio_handle_t *gpio_handle, uint8_t opt);
//void gpio_stage_event(gpio_handle_t *gpio_handle, uint8_t opt);
//void gpio_trig_swintr(gpio_handle_t *gpio_handle);
//void gpio_handle_irq(gpio_handle_t *gpio_handle)

#endif /* STM32F407XX_GPIO_H_ */
