/**
 * @file stm32f401xx_i2c_driver.h
 * @brief Header file for the I2C driver for STM32F401xx microcontrollers.
 * 
 * This file contains the configuration structures, macros, and function 
 * prototypes to interface with the I2C peripheral on STM32F401xx devices.
 * 
 * Created on: Oct 31, 2024
 * Author: Mehmethan Türüt
 */


#ifndef INC_STM32F401XX_I2C_DRIVER_H_
#define INC_STM32F401XX_I2C_DRIVER_H_

#include "stm32f401xx.h"

/**
 * @brief I2C configuration structure.
 * 
 * This structure holds the configuration parameters for I2C communication.
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;      /**< I2C clock speed (Standard mode, Fast mode, etc.). */
    uint8_t I2C_DeviceAddress;  /**< I2C device address (7-bit). */
    uint8_t I2C_ACKControl;     /**< ACK control (enable or disable). */
    uint8_t I2C_FMDutyCycle;    /**< Fast mode duty cycle (2 or 16/9). */
} I2C_Config_t;


/**
 * @brief I2C handle structure.
 * 
 * This structure contains the handle information for an I2C peripheral.
 */
typedef struct
{
    I2C_RegDef_t *pI2Cx;       /**< Pointer to the I2C peripheral base address. */
    I2C_Config_t I2C_Config;    /**< I2C configuration settings. */
} I2C_Handle_t;


/** @defgroup I2C_SCL_SPEED
 *  I2C clock speed settings.
 * @{
 */
#define I2C_SCL_SPEED_SM      100000   /**< Standard mode speed (100 kHz). */
#define I2C_SCL_SPEED_FM2K    200000   /**< Fast mode speed (200 kHz). */
#define I2C_SCL_SPEED_FM4K    400000   /**< Fast mode speed (400 kHz). */
/** @} */

/** @defgroup I2C_ACKControl
 *  I2C acknowledge control settings.
 * @{
 */
#define I2C_ACK_ENABLE        1        /**< Acknowledge enable. */
#define I2C_ACK_DISABLE       0        /**< Acknowledge disable. */
/** @} */

/** @defgroup I2C_FM_DUTY
 *  I2C fast mode duty cycle settings.
 * @{
 */
#define I2C_FM_DUTY_2         0        /**< Fast mode duty cycle 2. */
#define I2C_FM_DUTY_16_9      1        /**< Fast mode duty cycle 16/9. */
/** @} */


/**
 * @brief Enables or disables the peripheral clock for the given I2C port.
 * 
 * @param[in] pI2Cx Pointer to the I2C port base address.
 * @param[in] EnorDi Enable (1) or disable (0) the peripheral clock.
 */
void I2C_PeriClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * @brief Initializes the I2C peripheral according to the specified parameters.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);

/**
 * @brief Deinitializes the I2C peripheral, resetting it to its default state.
 * 
 * @param[in] pI2Cx Pointer to the I2C port base address.
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/**
 * @brief Configures the interrupt for the specified IRQ number.
 * 
 * @param[in] IRQNumber IRQ number to configure.
 * @param[in] EnorDi Enable (1) or disable (0) the IRQ.
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Sets the priority of the specified IRQ.
 * 
 * @param[in] IRQNumber IRQ number to configure priority for.
 * @param[in] IRQPriority Priority level to set.
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/**
 * @brief Application callback function for I2C events.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @param[in] Event Event type that occurred.
 */
void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t Event);

#endif /* INC_STM32F401XX_I2C_DRIVER_H_ */
