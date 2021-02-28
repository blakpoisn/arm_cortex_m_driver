/***************************************************************************************************
 * @file      stm32f407xx_gpio.c
 * @brief     Driver API implementation for GPIO peripheral.
 *
 * @author    Shubhankar Chaudhury
 * @date      00 Xxx 20xx
 **************************************************************************************************/

// Includes ----------------------------------------------------------------------------------------
#include <stm32f407xx_gpio.h>

// Static Deffinition ------------------------------------------------------------------------------
static uint32_t * port_address(uint8_t port);

// Functions ---------------------------------------------------------------------------------------

/***************************************************************************************************
 * @fn      gpio_handle_t gpio_handle_init(uint8_t  port, uint8_t pin)
 * @brief   Initializes the gpio_handle_t object.
 *
 * @param   port the port to be initialized with.
 * @param   pin the pin to be initialized with.
 * @return  gpio_handle_t object.
 **************************************************************************************************/
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

/***************************************************************************************************
 * @fn      gpio_pin_init(gpio_handle_t *gpio_handle)
 * @brief   Initializes the pin.
 *
 * @param   gpio_handle handle pointer to the port_pin object.
 * @return  Initialization successful indication with 1 (true).
 **************************************************************************************************/
uint8_t gpio_pin_init(gpio_handle_t *gpio_handle)
{
  uint32_t *base_ptr = port_address(gpio_handle->GPIO_port);
  uint32_t *offset_ptr;
  uint32_t *rcc_ahb1enr_ptr = (uint32_t *) ADDR_WITH_OFFSET(MA_RCC_BEG,MO_RCC_AHB1ENR);

  if ((base_ptr == NULL) || (gpio_handle->GPIO_pin > 0x0F))
  {
    return 0;
  }

  //enable clock
  *rcc_ahb1enr_ptr |= (ENABLE << gpio_handle->GPIO_port);

  // mode config
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

/***************************************************************************************************
 * @fn      void gpio_port_switch(uint8_t gpio_port, uint8_t state)
 * @brief   Enable/Disable the port.
 *
 * @param   gpio_port OP_GPIO_PORT_x, port to be enabled/disabled
 * @param   state ENABLE / DISABLE 
 * @return  void
 **************************************************************************************************/
void gpio_port_switch(uint8_t gpio_port, uint8_t state)
{
  uint32_t *ahb1enr = (uint32_t *) ADDR_WITH_OFFSET(MA_RCC_BEG,MO_RCC_AHB1ENR);
  if (state == ENABLE)
  {
    *ahb1enr |= (ENABLE << gpio_port);
  }
  else
  {
    *ahb1enr &= ~(ENABLE << gpio_port);
  }
}

/***************************************************************************************************
 * @fn      void gpio_port_reset(uint8_t gpio_port)
 * @brief   Enable/Disable the port.
 *
 * @param   gpio_port OP_GPIO_PORT_x, port to be reset
 * @return  void
 **************************************************************************************************/
void gpio_port_reset(uint8_t gpio_port)
{
  uint32_t *ahb1rstr = (uint32_t *) ADDR_WITH_OFFSET(MA_RCC_BEG,MO_RCC_AHB1RSTR);
  *ahb1rstr |= (ENABLE << gpio_port);
  // *ahb1rstr &= ~(ENABLE << gpio_port);
}

/***************************************************************************************************
 * @fn      void gpio_pin_write(gpio_handle_t *gpio_handle, uint8_t state)
 * @brief   set / reset pin.
 *
 * @param   gpio_handle handle pointer to the port_pin object.
 * @param   state ON or OFF state.
 * @return  void
 **************************************************************************************************/
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

/***************************************************************************************************
 * @fn      void gpio_pin_toggle(gpio_handle_t *gpio_handle)
 * @brief   Toggles the pin output.
 *
 * @param   gpio_handle - handle pointer to the port_pin object.
 * @return  void
 * @todo    Implementation code pending.
 **************************************************************************************************/
void gpio_pin_toggle(gpio_handle_t *gpio_handle)
{
  
}

/***************************************************************************************************
 * @fn      uint8_t gpio_pin_read(gpio_handle_t *gpio_handle)
 * @brief   Set / reset pin.
 *
 * @param   gpio_handle Handle pointer to the port_pin object.
 * @param   state ON or OFF state.
 * @return  void
 **************************************************************************************************/
uint8_t gpio_pin_read(gpio_handle_t *gpio_handle)
{
  uint8_t state;
  uint32_t *base_ptr = port_address(gpio_handle->GPIO_port);
  uint32_t *idr_ptr = base_ptr + ADDR_OFFSET(MO_GPIOx_IDR);
  state = (uint8_t) (*idr_ptr >> gpio_handle->GPIO_pin) & ENABLE;
  return state;
}

/***************************************************************************************************
 * @fn      void gpio_stage_intr(gpio_handle_t *gpio_handle, uint8_t opt)
 * @brief   Stages interrupt with given option (rising/falling/both edge).
 * @todo    Need to do NVIC implementation and them this Interrupt parts. 
 *
 * @param   gpio_handle Handle pointer to the port_pin object.
 * @param   opt Option for interrupt falling/rising/disable trigger.
 * @return  void
 **************************************************************************************************/
void gpio_stage_intr(gpio_handle_t *gpio_handle, uint8_t opt)
{
  uint32_t *exti_imr = (uint32_t *) ADDR_WITH_OFFSET(MA_EXTI_BEG,MO_EXTI_IMR);
  uint32_t *exti_rtsr = (uint32_t *) ADDR_WITH_OFFSET(MA_EXTI_BEG,MO_EXTI_RTSR);
  uint32_t *exti_ftsr = (uint32_t *) ADDR_WITH_OFFSET(MA_EXTI_BEG,MO_EXTI_FTSR);
  uint32_t *syscfg_exticrx = (uint32_t *) ADDR_WITH_OFFSET(MA_SYSCFG_BEG,MO_SYSCFG_EXTICR1);
  
  if (opt != OP_GPIO_INTR_NONE)
  {
    syscfg_exticrx += (gpio_handle->GPIO_pin / 4);
    *syscfg_exticrx &= ~(0x0F << (4 * (gpio_handle->GPIO_pin % 4)));
    *syscfg_exticrx |= (gpio_handle->GPIO_port << (4 * (gpio_handle->GPIO_pin % 4)));
    *exti_imr |= (ENABLE << gpio_handle->GPIO_pin);
    if ((opt == OP_GPIO_INTR_RISE) || (opt = OP_GPIO_INTR_BOTH))
    {
      *exti_rtsr |= (ENABLE << gpio_handle->GPIO_pin);
    }
    else
    {
      *exti_rtsr &= ~(ENABLE << gpio_handle->GPIO_pin);
    }
    
    if ((opt == OP_GPIO_INTR_FALL) || (opt = OP_GPIO_INTR_BOTH))
    {
      *exti_ftsr |= (ENABLE << gpio_handle->GPIO_pin);
    }
    else
    {
      *exti_ftsr &= ~(ENABLE << gpio_handle->GPIO_pin);
    }
  }
  else
  {
    *exti_imr &= ~(ENABLE << gpio_handle->GPIO_pin);
  }
}

// Static Functions --------------------------------------------------------------------------------

/***************************************************************************************************
 * @fn      static uint32_t * port_address(uint8_t port)
 * @brief   Returns the port's base address
 *
 * @param   port Port name enumeration (OP_GPIO_PORT_x)
 * @return  base_ptr Pointer to the base address of the given port
 **************************************************************************************************/
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
