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
uint8_t gpio_pin_init(gpio_handle_t *gpio_handle)
{
    uint32_t *base_ptr, *offset_ptr;
    uint32_t temp;

    switch (gpio_handle->GPIO_port)
    {
        case OP_GPIO_PORT_A:
            base_ptr = (uint32_t *) MA_GPIOA_BEG;
            break;
        case OP_GPIO_PORT_B:
            base_ptr = (uint32_t *) MA_GPIOB_BEG;
            break;
        case OP_GPIO_PORT_C:
            base_ptr = (uint32_t *) MA_GPIOC_BEG;
            break;
        case OP_GPIO_PORT_D:
            base_ptr = (uint32_t *) MA_GPIOD_BEG;
            break;
        case OP_GPIO_PORT_E:
            base_ptr = (uint32_t *) MA_GPIOE_BEG;
            break;
        case OP_GPIO_PORT_F:
            base_ptr = (uint32_t *) MA_GPIOF_BEG;
            break;
        case OP_GPIO_PORT_G:
            base_ptr = (uint32_t *) MA_GPIOG_BEG;
            break;
        case OP_GPIO_PORT_H:
            base_ptr = (uint32_t *) MA_GPIOH_BEG;
            break;
        case OP_GPIO_PORT_I:
            base_ptr = (uint32_t *) MA_GPIOI_BEG;
            break;
        default:
            base_ptr = NULL;
    }
    if (base_ptr == NULL){
        return 0;
    }

    //enable clock

    // mode cofig
    offset_ptr = base_ptr + MO_GPIOx_MODER;
    *offset_ptr &= ~(0x03 << (2*gpio_handle->GPIO_pin));
    *offset_ptr |= (gpio_handle->GPIO_pinmode << (2*gpio_handle->GPIO_pin));

    return 1;
}
