/**
 * @file      : stm32f407xx_gpio.c
 * @brief     : Driver API implementation for GPIO peripheral.
 * 
 * @author    : Shubhankar Chaudhury
 */

#include <stm32f407xx_gpio.h>

/**************************************************************************************************
 * @fn      : gpio_pin_init                                                                       *
 * @brief   : Initializes the pin. (only if not initialized already)                              *
 *                                                                                                *
 * @param   : gpio_handle - handle pointer to the port_pin object.                                *
 * @return  : initialization successful indication with 1 (true).                                 *
 **************************************************************************************************
 */
uint8_t gpio_pin_init(gpio_handle_t *gpio_handle){
    
    //  
    //  Check for RCC enable state for the port and enable if not
}