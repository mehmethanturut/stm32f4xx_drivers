/**
 * @file stm32f401xx_gpio_driver.h
 * @brief Header file for the GPIO driver for STM32F401xx microcontrollers.
 * 
 * This file contains the configuration structures, macros, and function 
 * prototypes to interface with the GPIO peripheral on STM32F401xx devices.
 * 
 * Created on: Oct 26, 2024
 * Author: Mehmethan Türüt
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

/**
 * @brief GPIO pin configuration structure.
 * 
 * This structure defines the configuration parameters for a GPIO pin.
 */
typedef struct 
{
    uint8_t GPIO_PinNumber;        /**< GPIO pin number (0-15). */
    uint8_t GPIO_PinMode;          /**< GPIO pin mode (@ref GPIO_PIN_MODES). */
    uint8_t GPIO_PinSpeed;         /**< GPIO pin speed (@ref GPIO_PIN_OUTPUT_SPEEDS). */
    uint8_t GPIO_PinPuPdControl;   /**< GPIO pull-up/pull-down control (@ref GPIO_PIN_PUPD). */
    uint8_t GPIO_PinOPType;        /**< GPIO output type (@ref GPIO_PIN_OUTPUT_TYPES). */
    uint8_t GPIO_PinAltFunMode;    /**< GPIO alternate function mode (@ref GPIO_PIN_MODES). */
} GPIO_PinConfig_t;

/**
 * @brief GPIO handle structure.
 * 
 * This structure defines the handle for a GPIO pin, which includes the 
 * base address of the GPIO port and the pin configuration structure.
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;         /**< Base address of the GPIO port. */
    GPIO_PinConfig_t GPIO_PinConfig; /**< GPIO pin configuration. */
} GPIO_Handle_t;

/** @defgroup GPIO_PIN_NUMBERS GPIO possible PIN numbers
 * @{
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15
/** @} */

/** @defgroup GPIO_PIN_MODES GPIO possible modes
 * @{
 */
#define GPIO_MODE_IN        0   /**< Input mode. */
#define GPIO_MODE_OUT       1   /**< Output mode. */
#define GPIO_MODE_ALTFN     2   /**< Alternate function mode. */
#define GPIO_MODE_ANALOG    3   /**< Analog mode. */
#define GPIO_MODE_IT_FT     4   /**< Interrupt mode with falling edge trigger. */
#define GPIO_MODE_IT_RT     5   /**< Interrupt mode with rising edge trigger. */
#define GPIO_MODE_IT_RFT    6   /**< Interrupt mode with rising and falling edge trigger. */
/** @} */

/** @defgroup GPIO_PIN_OUTPUT_TYPES GPIO possible output types
 * @{
 */
#define GPIO_OP_TYPE_PP     0   /**< Push-pull output. */
#define GPIO_OP_TYPE_OD     1   /**< Open-drain output. */
/** @} */

/** @defgroup GPIO_PIN_OUTPUT_SPEEDS GPIO possible output speeds
 * @{
 */
#define GPIO_SPEED_LOW      0   /**< Low speed. */
#define GPIO_SPEED_MED      1   /**< Medium speed. */
#define GPIO_SPEED_FAST     2   /**< Fast speed. */
#define GPIO_SPEED_HIGH     3   /**< High speed. */
/** @} */

/** @defgroup GPIO_PIN_PUPD GPIO pin pull-up/pull-down configuration
 * @{
 */
#define GPIO_NO_PUPD        0   /**< No pull-up, pull-down. */
#define GPIO_PIN_PU         1   /**< Pull-up enabled. */
#define GPIO_PIN_PD         2   /**< Pull-down enabled. */
/** @} */

/** 
 * @brief Enables or disables peripheral clock for a GPIO port.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 * @param[in] EnorDi Enable (1) or Disable (0) the peripheral clock.
 */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * @brief Initializes the GPIO pin.
 * 
 * @param[in] pGPIOHandle Pointer to the GPIO handle structure.
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief Deinitializes the GPIO port, resetting it to its default state.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Reads the value of a specific GPIO pin.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 * @param[in] PinNumber Pin number to be read.
 * 
 * @return The pin state (0 or 1).
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @brief Reads the value of an entire GPIO port.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 * 
 * @return 16-bit port value.
 */
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Writes a value to a specific GPIO pin.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 * @param[in] PinNumber Pin number to be written to.
 * @param[in] value Value to be written (0 or 1).
 */
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);

/**
 * @brief Writes a value to the entire GPIO port.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 * @param[in] value 16-bit value to be written to the port.
 */
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);

/**
 * @brief Toggles the output state of a specific GPIO pin.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 * @param[in] PinNumber Pin number to toggle.
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @brief Configures the interrupt for a specific IRQ number.
 * 
 * @param[in] IRQNumber IRQ number to configure.
 * @param[in] EnorDi Enable (1) or Disable (0) the IRQ.
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Configures the priority for a specific IRQ.
 * 
 * @param[in] IRQNumber IRQ number for which priority is to be set.
 * @param[in] IRQPriority Priority level for the IRQ.
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief ISR handler for a specific GPIO pin.
 * 
 * @param[in] PinNumber Pin number for which the ISR is to be handled.
 */
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
