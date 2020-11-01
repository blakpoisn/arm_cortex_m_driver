/**
 * @file      : stm32f407xx_gpio.c
 * @brief     : Driver API implementation for GPIO peripheral.
 *
 * @author    : Shubhankar Chaudhury
 */

#include <stm32f407xx_gpio.h>

/*
 * Static Functions
 */
static uint32_t * port_address(uint8_t port);

/*
 * API Implementations
 */

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
    uint32_t *base_ptr = port_address(gpio_handle->GPIO_port);
    uint32_t *offset_ptr;
    uint32_t *rcc_ahb1enr_ptr = (uint32_t *) (MA_RCC_BEG + MO_RCC_AHB1ENR);

    if ((base_ptr == NULL) || (gpio_handle->GPIO_pin > 0x0F))
    {
        return 0;
    }

    //enable clock
    *rcc_ahb1enr_ptr |= (ENABLE << gpio_handle->GPIO_port);

    // mode cofig
    offset_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_MODER);
    *offset_ptr &= ~(0x03 << (2*gpio_handle->GPIO_pin));
    *offset_ptr |= (gpio_handle->GPIO_pinmode << (2*gpio_handle->GPIO_pin));

    switch (gpio_handle->GPIO_pinmode)
    {
        case OP_GPIO_PINMODE_IN:
            offset_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_PUPDR);
            *offset_ptr &= ~(0x03 << (2*gpio_handle->GPIO_pin));
            *offset_ptr |= (gpio_handle->GPIO_pushpull << (2*gpio_handle->GPIO_pin));
            break;
        case OP_GPIO_PINMODE_OUT:
            offset_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_OTYPER);
            *offset_ptr &= ~(0x01 << gpio_handle->GPIO_pin);
            *offset_ptr |= (gpio_handle->GPIO_otype << gpio_handle->GPIO_pin);

            offset_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_OSPEEDR);
            *offset_ptr &= ~(0x03 << (2*gpio_handle->GPIO_pin));
            *offset_ptr |= (gpio_handle->GPIO_ospeed << (2*gpio_handle->GPIO_pin));

            offset_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_PUPDR);
            *offset_ptr &= ~(0x03 << (2*gpio_handle->GPIO_pin));
            *offset_ptr |= (gpio_handle->GPIO_pushpull << (2*gpio_handle->GPIO_pin));
            break;
        case OP_GPIO_PINMODE_ALT:
            offset_ptr = base_ptr + (ADDR_OFFSET(MO_GPIOx_AFRL) + (gpio_handle->GPIO_pin * 4) / 32);
            *offset_ptr &= ~(0x0F << (4 * (gpio_handle->GPIO_pin % 8)));
            *offset_ptr |= (gpio_handle->GPIO_atlFunc << (4 * (gpio_handle->GPIO_pin % 8)));
            break;
    }

    return 1;
}

/**************************************************************************************************
 * @fn      : gpio_pin_write                                                                      *
 * @brief   : set / reset pin.                                                                    *
 *                                                                                                *
 * @param   : gpio_handle - handle pointer to the port_pin object.                                *
 * @param   : state - ON or OFF state.                                                            *
 * @return  : void                                                                                *
 **************************************************************************************************
 */
void gpio_pin_write(gpio_handle_t *gpio_handle, uint8_t state)
{
    uint32_t *base_ptr = port_address(gpio_handle->GPIO_port);
    uint32_t *bsrr_ptr;
    if ((base_ptr != NULL) && (gpio_handle->GPIO_pin < 16))
    {
        bsrr_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_BSRR);
        if(state)
        {
            *bsrr_ptr = ENABLE << (gpio_handle->GPIO_pin);
        }
        else
        {
            *bsrr_ptr = ENABLE << (gpio_handle->GPIO_pin + 16);
        }
    }
}

/**************************************************************************************************
 * @fn      : gpio_pin_write                                                                      *
 * @brief   : set / reset pin.                                                                    *
 *                                                                                                *
 * @param   : gpio_handle - handle pointer to the port_pin object.                                *
 * @param   : state - ON or OFF state.                                                            *
 * @return  : void                                                                                *
 **************************************************************************************************
 */
uint8_t gpio_pin_read(gpio_handle_t *gpio_handle)
{
    uint8_t state;
    uint32_t *base_ptr = port_address(gpio_handle->GPIO_port);
    uint32_t *idr_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_IDR);
    state = (uint8_t) (*idr_ptr >> gpio_handle->GPIO_pin) & ENABLE;
    return state;
}

//-Static-Fuction----------------------------------------------------------------------------------
static uint32_t * port_address(uint8_t port)
{
    uint32_t *base_ptr;
    switch (port)
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

    return base_ptr;
}
