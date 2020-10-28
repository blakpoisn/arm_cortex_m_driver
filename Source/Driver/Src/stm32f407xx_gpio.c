/**
 * @file      : stm32f407xx_gpio.c
 * @brief     : Driver API implementation for GPIO peripheral.
 *
 * @author    : Shubhankar Chaudhury
 */

#include <stm32f407xx_gpio.h>

/**************************************************************************************************
 * @fn      : gpio_handle_init                                                                    *
 * @brief   : Initializes the gpio_handle_t object.                                               *
 *                                                                                                *
 * @param   : port - the port to be initialized with.                                             *
 * @param   : pin - the pin to be initialized with.                                               *
 * @return  : gpio_handle_t object.                                                               *
 **************************************************************************************************
 */
gpio_handle_t gpio_handle_init(uint8_t  port, uint8_t pin)
{
    gpio_handle_t obj;
    obj.GPIO_port = port;
    obj.GPIO_pin = pin;
    obj.GPIO_pinmode = OP_GPIO_PINMODE_IN;
    obj.GPIO_otype = OP_GPIO_OTYPE_PUSHPULL;
    obj.GPIO_ospeed = OP_GPIO_OSPEED_LOW;
    obj.GPIO_pushpull = OP_GPIO_PUPD_NOPUPD;
    obj.GPIO_atlFunc = OP_GPIO_ALTFUNC_AF0;

    return obj;
}

/**************************************************************************************************
 * @fn      : gpio_pin_init                                                                       *
 * @brief   : Initializes the pin.                                                                *
 *                                                                                                *
 * @param   : gpio_handle - handle pointer to the port_pin object.                                *
 * @return  : initialization successful indication with 1 (true).                                 *
 **************************************************************************************************
 */
uint8_t gpio_pin_init(gpio_handle_t *gpio_handle)
{
    uint32_t *base_ptr, *offset_ptr;
    uint32_t *rcc_ahb1enr_ptr = (uint32_t *) (MA_RCC_BEG + MO_RCC_AHB1ENR);

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
    if ((base_ptr == NULL) || (gpio_handle->GPIO_pin > 0x0F))
    {
        return 0;
    }

    //enable clock
    *rcc_ahb1enr_ptr |= (ENABLE << gpio_handle->GPIO_port);

    // mode cofig
    offset_ptr = base_ptr + MO_GPIOx_MODER;
    *offset_ptr &= ~(0x03 << (2*gpio_handle->GPIO_pin));
    *offset_ptr |= (gpio_handle->GPIO_pinmode << (2*gpio_handle->GPIO_pin));

    switch (gpio_handle->GPIO_pinmode)
    {
        case OP_GPIO_PINMODE_IN:
            //TODO
            break;
        case OP_GPIO_PINMODE_OUT:
            offset_ptr = base_ptr + MO_GPIOx_OTYPER;
            *offset_ptr &= ~(0x01 << gpio_handle->GPIO_pin);
            *offset_ptr |= (gpio_handle->GPIO_otype << gpio_handle->GPIO_pin);

            offset_ptr = base_ptr + MO_GPIOx_OSPEEDR;
            *offset_ptr &= ~(0x03 << (2*gpio_handle->GPIO_pin));
            *offset_ptr |= (gpio_handle->GPIO_ospeed << (2*gpio_handle->GPIO_pin));
            break;
        case OP_GPIO_PINMODE_ALT:
            //TODO
            break;
        case OP_GPIO_PINMODE_ANALOG:
            //TODO
            break;
    }

    return 1;
}
