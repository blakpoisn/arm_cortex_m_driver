/***************************************************************************************************
 * @file      cortex_m4_driver.h
 * @brief     Cortex M4 CPU core driver header file.
 *            
 * @author    Shubhankar Chaudhury
 * @date      29 Mar 2021
 **************************************************************************************************/
 
 #ifndef CORTEX_M4_DRIVER_H_
 #define CORTEX_M4_DRIVER_H_
 
 
// Includes ---------------------------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>

// Defines ----------------------------------------------------------------------------------------

//Common
#define FALSE     0x00
#define TRUE      0x01

//Core peripherals
#define MA_NVIC_BEG       0xE000E100U     /*!< Nested Vector Interrupt Controller Memory Region. */
#define MA_NVIC_STIR      0xE000EF00U     /*!< NVIC Software trigger interrupt register. */

// Structures --------------------------------------------------------------------------------------
/**
 *  @brief Register definition for NVIC CPU core memory map.
 */
typedef struct {
  volatile uint32_t ISER[8];       /*!< Interrupt Set Enable Register. */
  volatile uint32_t RESa[24];      /*!< Reserved. */
  volatile uint32_t ICER[8];       /*!< Interrupt Clear Enable Register. */
  volatile uint32_t RESb[24];      /*!< Reserved. */
  volatile uint32_t ISPR[8];       /*!< Interrupt Set Pending Register. */
  volatile uint32_t RESc[24];      /*!< Reserved. */
  volatile uint32_t ICPR[8];       /*!< Interrupt Clear Pending Register. */
  volatile uint32_t RESd[24];      /*!< Reserved. */
  volatile uint32_t IABR[8];       /*!< Interrupt Active Bit Register. */
  volatile uint32_t RESe[56];      /*!< Reserved. */
  volatile uint32_t IPR[60];       /*!< Interrupt Priority Register. */
} nvic_regdef_t;

// Peripheral Definitions --------------------------------------------------------------------------
#define NVIC_MEM_MAP     ((nvic_regdef_t *)MA_NVIC_BEG)        /*!< NVIC register memory map access pointer. */


#endif /* CORTEX_M4_DRIVER_H_ */