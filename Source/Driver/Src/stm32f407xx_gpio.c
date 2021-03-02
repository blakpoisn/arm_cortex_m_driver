/***************************************************************************************************
 * @file      stm32f407xx_gpio.c
 * @brief     Driver API implementation for GPIO peripheral.
 *
 * @author    Shubhankar Chaudhury
 * @date      02 Mar 2021
 **************************************************************************************************/

// Includes ----------------------------------------------------------------------------------------
#include <stm32f407xx_gpio.h>

// Static Definition -------------------------------------------------------------------------------
static gpio_regdef_t * port_address(uint8_t port);

// Functions ---------------------------------------------------------------------------------------

/***************************************************************************************************
 * @fn      gpio_handle_t gpio_handle_init(gpio_port_t  port, gpio_pin_t pin)
 * @brief   Initializes the gpio_handle_t object with defaults.
 *
 * @param   port Port to be initialized with.
 * @param   pin Pin to be initialized with.
 * @return  gpio_handle_t object.
 **************************************************************************************************/
gpio_handle_t gpio_handle_init(gpio_port_t  port, gpio_pin_t pin)
{
  gpio_handle_t obj = {
    port, 
    pin, 
    GPIO_PINMODE_IN,
    GPIO_OTYPE_PUSHPULL,
    GPIO_OSPEED_LOW,
    GPIO_PUPD_NOPUPD,
    GPIO_ALTFUNC_0
  };
  return obj;
}

/***************************************************************************************************
 * @fn      void gpio_pin_init(gpio_handle_t *gpio_handle)
 * @brief   Initializes the pin.
 *
 * @param   gpio_handle handle pointer to the port_pin object.
 **************************************************************************************************/
void gpio_pin_init(gpio_handle_t *gpio_handle)
{
  gpio_regdef_t *base_ptr = port_address(gpio_handle->port);

  //enable clock
  RCC_MEM_MAP->AHB1ENR |= (ENABLE << gpio_handle->port);

  // mode config
  base_ptr->MODER &= ~(0x03 << (2*gpio_handle->pin));
  base_ptr->MODER |= (gpio_handle->pinmode << (2*gpio_handle->pin));

  switch (gpio_handle->pinmode)
  {
    case GPIO_PINMODE_IN:
      base_ptr->PUPDR &= ~(0x03 << (2*gpio_handle->pin));
      base_ptr->PUPDR |= (gpio_handle->pushpull << (2*gpio_handle->pin));
      break;
    case GPIO_PINMODE_OUT:
      base_ptr->OTYPER &= ~(0x01 << gpio_handle->pin);
      base_ptr->OTYPER |= (gpio_handle->otype << gpio_handle->pin);
      
      base_ptr->OSPEEDR &= ~(0x03 << (2*gpio_handle->pin));
      base_ptr->OSPEEDR |= (gpio_handle->ospeed << (2*gpio_handle->pin));
      
      base_ptr->PUPDR &= ~(0x03 << (2*gpio_handle->pin));
      base_ptr->PUPDR |= (gpio_handle->pushpull << (2*gpio_handle->pin));
      break;
    case GPIO_PINMODE_ALT:
      base_ptr->AFR[(gpio_handle->pin / 8)] &= ~(0x0F << (4 * (gpio_handle->pin % 8)));
      base_ptr->AFR[(gpio_handle->pin / 8)] |= (gpio_handle->atlFunc << (4 * (gpio_handle->pin % 8)));

      base_ptr->OTYPER &= ~(0x01 << gpio_handle->pin);
      base_ptr->OTYPER |= (gpio_handle->otype << gpio_handle->pin);
      
      base_ptr->OSPEEDR &= ~(0x03 << (2*gpio_handle->pin));
      base_ptr->OSPEEDR |= (gpio_handle->ospeed << (2*gpio_handle->pin));
      
      base_ptr->PUPDR &= ~(0x03 << (2*gpio_handle->pin));
      base_ptr->PUPDR |= (gpio_handle->pushpull << (2*gpio_handle->pin));
      break;
    case GPIO_PINMODE_ANALOG:
      base_ptr->PUPDR &= ~(0x03 << (2*gpio_handle->pin));
      break;
  }
}

/***************************************************************************************************
 * @fn      void gpio_port_switch(gpio_port_t gpio_port, uint8_t state)
 * @brief   Enable/Disable the port.
 *
 * @param   gpio_port Port to be enabled/disabled
 * @param   state ENABLE / DISABLE 
 * @return  void
 **************************************************************************************************/
void gpio_port_switch(gpio_port_t gpio_port, uint8_t state)
{
  if (state == ENABLE) {
    RCC_MEM_MAP->AHB1ENR |= (ENABLE << gpio_port);
  }
  else {
    RCC_MEM_MAP->AHB1ENR &= ~(ENABLE << gpio_port);
  }
}

/***************************************************************************************************
 * @fn      void gpio_port_reset(gpio_port_t gpio_port)
 * @brief   Enable/Disable the port.
 *
 * @param   gpio_port Port to be reset.
 * @param   state Reset state = ENABLE or DISABLE.
 **************************************************************************************************/
void gpio_port_reset(gpio_port_t gpio_port, uint8_t state)
{
  if (state == ENABLE) {
    RCC_MEM_MAP->AHB1RSTR |= (ENABLE << gpio_port);
  }
  else {
    RCC_MEM_MAP->AHB1RSTR &= ~(ENABLE << gpio_port);
  }
}

/***************************************************************************************************
 * @fn      void gpio_pin_write(gpio_handle_t *gpio_handle, uint8_t state)
 * @brief   Write to the pin.
 *
 * @param   gpio_handle Pointer to the port_pin object.
 * @param   state ON or OFF state.
 **************************************************************************************************/
void gpio_pin_write(gpio_handle_t *gpio_handle, uint8_t state)
{
  gpio_regdef_t *base_ptr = port_address(gpio_handle->port);
  if(state == ENABLE)
  {
    base_ptr->BSRR = ENABLE << (gpio_handle->pin);
  }
  else
  {
    base_ptr->BSRR = ENABLE << (gpio_handle->pin + 16);
  }
}

/***************************************************************************************************
 * @fn      void gpio_port_write(gpio_port_t gpio_port, uint16_t data)
 * @brief   Writes to the port with provided data.
 * 
 * @param   gpio_port GPIO port to which data is to be written.
 * @param   data The data to be written.
 **************************************************************************************************/
void gpio_port_write(gpio_port_t gpio_port, uint16_t data)
{
  gpio_regdef_t *base_ptr = port_address(gpio_port);
  base_ptr->ODR = (uint32_t) data;
}

/***************************************************************************************************
 * @fn      void gpio_pin_toggle(gpio_handle_t *gpio_handle)
 * @brief   Toggles the pin output.
 *
 * @param   gpio_handle Handle to the port_pin object.
 **************************************************************************************************/
void gpio_pin_toggle(gpio_handle_t *gpio_handle)
{
  gpio_regdef_t *base_ptr = port_address(gpio_handle->port);
  base_ptr->ODR ^= ENABLE << (gpio_handle->pin);
}

/***************************************************************************************************
 * @fn      uint8_t gpio_pin_outputStatus(gpio_handle_t *gpio_handle)
 * @brief   Reads the output pin status.
 *
 * @param   gpio_handle Handle to the gpio pin object.
 * @return  Output staus of the pin.
 **************************************************************************************************/
uint8_t gpio_pin_outputStatus(gpio_handle_t *gpio_handle)
{
  gpio_regdef_t *base_ptr = port_address(gpio_handle->port);
  uint8_t status = (uint8_t) (base_ptr->ODR >> gpio_handle->pin) & ENABLE;
  return status;
}

/***************************************************************************************************
 * @fn      uint16_t gpio_port_outputStatus(gpio_port_t gpio_port)
 * @brief   Reads the output port status.
 *
 * @param   gpio_port The gpio port.
 * @return  Output staus of the port.
 **************************************************************************************************/
uint16_t gpio_port_outputStatus(gpio_port_t gpio_port)
{
  gpio_regdef_t *base_ptr = port_address(gpio_port);
  uint16_t status = (uint16_t) base_ptr->ODR & 0x0000FFFFU;
  return status;
}

/***************************************************************************************************
 * @fn      uint8_t gpio_pin_read(gpio_handle_t *gpio_handle)
 * @brief   Read the pin state.
 *
 * @param   gpio_handle Handle pointer to the port_pin object.
 * @return  Pin state.
 **************************************************************************************************/
uint8_t gpio_pin_read(gpio_handle_t *gpio_handle)
{
  gpio_regdef_t *base_ptr = port_address(gpio_handle->port);
  uint8_t state = (uint8_t) (base_ptr->IDR >> gpio_handle->pin) & ENABLE;
  return state;
}

/***************************************************************************************************
 * @fn      uint16_t gpio_port_read(gpio_port_t gpio_port)
 * @brief   Reads the port data.
 *
 * @param   gpio_port GPIO port to be read.
 * @return  Data read from the given GPIO port.
 **************************************************************************************************/
uint16_t gpio_port_read(gpio_port_t gpio_port)
{
  gpio_regdef_t *base_ptr = port_address(gpio_port);
  uint16_t data = (uint16_t) base_ptr->IDR & 0x0000FFFFU;
  return data;
}

// Static Functions --------------------------------------------------------------------------------

/***************************************************************************************************
 * @fn      static uint32_t * port_address(gpio_port_t port)
 * @brief   Returns the port's base address
 *
 * @param   port Port name enumeration.
 * @return  base_ptr Pointer to the base address of the given port
 **************************************************************************************************/
static gpio_regdef_t * port_address(gpio_port_t port)
{
  gpio_regdef_t *base_ptr;
  switch (port)
  {
    case GPIO_PORT_A:
      base_ptr = (gpio_regdef_t *) MA_GPIOA_BEG;
      break;
    case GPIO_PORT_B:
      base_ptr = (gpio_regdef_t *) MA_GPIOB_BEG;
      break;
    case GPIO_PORT_C:
      base_ptr = (gpio_regdef_t *) MA_GPIOC_BEG;
      break;
    case GPIO_PORT_D:
      base_ptr = (gpio_regdef_t *) MA_GPIOD_BEG;
      break;
    case GPIO_PORT_E:
      base_ptr = (gpio_regdef_t *) MA_GPIOE_BEG;
      break;
    case GPIO_PORT_F:
      base_ptr = (gpio_regdef_t *) MA_GPIOF_BEG;
      break;
    case GPIO_PORT_G:
      base_ptr = (gpio_regdef_t *) MA_GPIOG_BEG;
      break;
    case GPIO_PORT_H:
      base_ptr = (gpio_regdef_t *) MA_GPIOH_BEG;
      break;
    case GPIO_PORT_I:
      base_ptr = (gpio_regdef_t *) MA_GPIOI_BEG;
      break;
    default:
      base_ptr = NULL;
  }

  return base_ptr;
}
