/***************************************************************************************************
 * @file      stm32f407xx_gpio.h
 * @brief     API Header for GPIO implementation.
 * 
 * @author    Shubhankar Chaudhury
 * @date      02 Mar 2021
 **************************************************************************************************/

#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

// Includes ----------------------------------------------------------------------------------------
#include <stm32f407xx_driver.h>

// Enumerations ------------------------------------------------------------------------------------
/**
 *  @brief Enumeration for GPIO Port selection options.
 */
typedef enum {
  GPIO_PORT_A,           /*!< GPIO Port A Selection. */
  GPIO_PORT_B,           /*!< GPIO Port B Selection. */
  GPIO_PORT_C,           /*!< GPIO Port C Selection. */
  GPIO_PORT_D,           /*!< GPIO Port D Selection. */
  GPIO_PORT_E,           /*!< GPIO Port E Selection. */
  GPIO_PORT_F,           /*!< GPIO Port F Selection. */
  GPIO_PORT_G,           /*!< GPIO Port G Selection. */
  GPIO_PORT_H,           /*!< GPIO Port H Selection. */
  GPIO_PORT_I            /*!< GPIO Port I Selection. */
} gpio_port_t;

/**
 *  @brief Enumeration for GPIO Port Pin selection options.
 */
typedef enum {
  GPIO_PIN_0 ,         /*!< GPIO Port Pin 0  Selection. */
  GPIO_PIN_1 ,         /*!< GPIO Port Pin 1  Selection. */
  GPIO_PIN_2 ,         /*!< GPIO Port Pin 2  Selection. */
  GPIO_PIN_3 ,         /*!< GPIO Port Pin 3  Selection. */
  GPIO_PIN_4 ,         /*!< GPIO Port Pin 4  Selection. */
  GPIO_PIN_5 ,         /*!< GPIO Port Pin 5  Selection. */
  GPIO_PIN_6 ,         /*!< GPIO Port Pin 6  Selection. */
  GPIO_PIN_7 ,         /*!< GPIO Port Pin 7  Selection. */
  GPIO_PIN_8 ,         /*!< GPIO Port Pin 8  Selection. */
  GPIO_PIN_9 ,         /*!< GPIO Port Pin 9  Selection. */
  GPIO_PIN_10,         /*!< GPIO Port Pin 10 Selection. */
  GPIO_PIN_11,         /*!< GPIO Port Pin 11 Selection. */
  GPIO_PIN_12,         /*!< GPIO Port Pin 12 Selection. */
  GPIO_PIN_13,         /*!< GPIO Port Pin 13 Selection. */
  GPIO_PIN_14,         /*!< GPIO Port Pin 14 Selection. */
  GPIO_PIN_15          /*!< GPIO Port Pin 15 Selection. */
} gpio_pin_t;

/**
 *  @brief Enumeration for GPIO Pin Mode selection options.
 */
typedef enum {
  GPIO_PINMODE_IN,      /*!< GPIO Pin Mode Input Selection. */
  GPIO_PINMODE_OUT,     /*!< GPIO Pin Mode Output Selection. */
  GPIO_PINMODE_ALT,     /*!< GPIO Pin Mode Alternate Function Selection. */
  GPIO_PINMODE_ANALOG   /*!< GPIO Pin Mode Analog Selection. */
} gpio_pinmode_t;

/**
 *  @brief Enumeration for GPIO pin output type.
 */
typedef enum {
  GPIO_OTYPE_PUSHPULL,    /*!< GPIO Pin output type push-pull selection. */
  GPIO_OTYPE_OPDRAIN      /*!< GPIO Pin output type open drain selection. */
} gpio_otype_t;
 
/**
 *  @brief Enumeration for GPIO Pin output speed options.
 */
typedef enum {
  GPIO_OSPEED_LOW,      /*!< GPIO Pin output speed low selection. */
  GPIO_OSPEED_MED,      /*!< GPIO Pin output speed medium selection. */
  GPIO_OSPEED_HI,       /*!< GPIO Pin output speed high selection. */
  GPIO_OSPEED_VHI       /*!< GPIO Pin output speed very high selection. */
} gpio_ospeed_t;

/**
 *  @brief Enumeration for GPIO Pin pull-up or pull-down options.
 */
typedef enum {
  GPIO_PUPD_NOPUPD,    /*!< GPIO Pin no pull-up niether pull-down selected. */
  GPIO_PUPD_PU,        /*!< GPIO Pin pull-up selected. */
  GPIO_PUPD_PD         /*!< GPIO Pin pull-down selected. */
} gpio_pupd_t;

/**
 *  @brief Enumeration for GPIO Pin alternate function options.
 */
typedef enum {
  GPIO_ALTFUNC_0,   /*!< GPIO Pin alternate funcion number 0  selected. */
  GPIO_ALTFUNC_1,   /*!< GPIO Pin alternate funcion number 1  selected. */
  GPIO_ALTFUNC_2,   /*!< GPIO Pin alternate funcion number 2  selected. */
  GPIO_ALTFUNC_3,   /*!< GPIO Pin alternate funcion number 3  selected. */
  GPIO_ALTFUNC_4,   /*!< GPIO Pin alternate funcion number 4  selected. */
  GPIO_ALTFUNC_5,   /*!< GPIO Pin alternate funcion number 5  selected. */
  GPIO_ALTFUNC_6,   /*!< GPIO Pin alternate funcion number 6  selected. */
  GPIO_ALTFUNC_7,   /*!< GPIO Pin alternate funcion number 7  selected. */
  GPIO_ALTFUNC_8,   /*!< GPIO Pin alternate funcion number 8  selected. */
  GPIO_ALTFUNC_9,   /*!< GPIO Pin alternate funcion number 9  selected. */
  GPIO_ALTFUNC_10,  /*!< GPIO Pin alternate funcion number 10 selected. */
  GPIO_ALTFUNC_11,  /*!< GPIO Pin alternate funcion number 11 selected. */
  GPIO_ALTFUNC_12,  /*!< GPIO Pin alternate funcion number 12 selected. */
  GPIO_ALTFUNC_13,  /*!< GPIO Pin alternate funcion number 13 selected. */
  GPIO_ALTFUNC_14,  /*!< GPIO Pin alternate funcion number 14 selected. */
  GPIO_ALTFUNC_15   /*!< GPIO Pin alternate funcion number 15 selected. */
} gpio_altFunc_t;

/**
 *  @brief Enumeration for GPIO Pin interrupt trigger options.
 */
typedef enum {
  GPIO_INTR_FALL,    /*!< GPIO Pin interrupt at falling edge selection. */
  GPIO_INTR_RISE,    /*!< GPIO Pin interrupt at rising edge selection. */
  GPIO_INTR_BOTH,    /*!< GPIO Pin interrupt at both edge selection. */
  GPIO_INTR_NONE     /*!< GPIO Pin interrupt at none of the edges selection. */
} gpio_intr_t;

/**
 *  @brief Enumeration for GPIO Pin event trigger options.
 */
typedef enum {
  GPIO_EVENT_FALL,    /*!< GPIO Pin event at falling edge selection. */
  GPIO_EVENT_RISE,    /*!< GPIO Pin event at rising edge selection. */
  GPIO_EVENT_BOTH,    /*!< GPIO Pin event at both edge selection. */
  GPIO_EVENT_NONE     /*!< GPIO Pin event at none of the edges selection. */
} gpio_event_t;

// Structures --------------------------------------------------------------------------------------
/**
 *  @brief   GPIO handle objet for handling GPIO requests.
 */
typedef struct {
    gpio_port_t    port;        /*!< Assigned Port. */
    gpio_pin_t     pin;         /*!< Assigned Pin. */
    gpio_pinmode_t pinmode;     /*!< Assigned Pin Mode. */
    gpio_otype_t   otype;       /*!< Assigned Pin output type. */
    gpio_ospeed_t  ospeed;      /*!< Assigned Pin output speed. */
    gpio_pupd_t    pushpull;    /*!< Assigned Pin push/pull state. */
    gpio_altFunc_t atlFunc;     /*!< Assigned Pin alternalte function choice. */
} gpio_handle_t;

// Function Prototypes -----------------------------------------------------------------------------

gpio_handle_t gpio_handle_init(gpio_port_t  port, gpio_pin_t pin);
void gpio_pin_init(gpio_handle_t *gpio_handle);
void gpio_port_switch(gpio_port_t gpio_port, uint8_t state);
void gpio_port_reset(gpio_port_t gpio_port, uint8_t state);

void gpio_pin_write(gpio_handle_t *gpio_handle, uint8_t state);
void gpio_port_write(gpio_port_t gpio_port, uint16_t data);
void gpio_pin_toggle(gpio_handle_t *gpio_handle);
uint8_t gpio_pin_outputStatus(gpio_handle_t *gpio_handle);
uint16_t gpio_port_outputStatus(gpio_port_t gpio_port);

uint8_t gpio_pin_read(gpio_handle_t *gpio_handle);
uint16_t gpio_port_read(gpio_port_t gpio_port);

//void gpio_stage_intr(gpio_handle_t *gpio_handle, uint8_t opt);
//void gpio_stage_event(gpio_handle_t *gpio_handle, uint8_t opt);
//void gpio_trig_swintr(gpio_handle_t *gpio_handle);
//void gpio_handle_irq(gpio_handle_t *gpio_handle)

#endif /* STM32F407XX_GPIO_H_ */
