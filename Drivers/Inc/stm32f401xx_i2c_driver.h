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
    I2C_Config_t I2C_Config;   /**< I2C configuration settings. */
    uint8_t      *pTxBuffer;   /**< Pointer to the transmit buffer. */
    uint8_t      *pRxBuffer;   /**< Pointer to the receive buffer. */
    uint32_t     TxLen;        /**< Length of the transmission data. */
    uint32_t     RxLen;        /**< Length of the reception data. */
    uint8_t      TxRxState;    /**< State of the I2C communication (e.g., busy in transmission or reception). */
    uint8_t      DevAddr;      /**< Address of the target device on the I2C bus. */
    uint32_t     RxSize;       /**< Size of the data to be received. */
    uint8_t      Sr;           /**< Repeated start condition control flag. */

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
 * @brief  Sends data from the I2C master to a slave device.
 * 
 * This function transmits a specified amount of data from the master I2C device
 * to a target slave device. It manages the I2C communication process, including
 * addressing the slave and handling data transfer.
 * 
 * @param[in]  pI2CHandle  Pointer to the I2C handle structure containing the 
 *                         configuration and state information for the I2C peripheral.
 * @param[in]  pTxbuffer   Pointer to the buffer containing the data to be sent.
 * @param[in]  Len         Length of the data to be transmitted (in bytes).
 * @param[in]  SlaveAddr   7-bit address of the target slave device.
 * 
 * @note  This function is blocking and waits for the transmission process to complete.
 *        Ensure the I2C peripheral is properly configured and the slave device is
 *        ready to receive data.
 * 
 * @pre    The I2C peripheral must be initialized and configured in master mode.
 * @pre    The slave device must acknowledge its address.
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/**
 * @brief  Receives data from an I2C slave in master mode.
 *
 * This function is used to receive a block of data from an I2C slave device
 * when the I2C peripheral is configured in master mode.
 *
 * @param[in]   pI2CHandle  Pointer to the I2C handle structure containing the base address of the I2C peripheral
 *                          and the configuration settings for the I2C device.
 * @param[out]  pRxbuffer   Pointer to the buffer where the received data will be stored.
 * @param[in]   Len         Length of the data to be received in bytes.
 * @param[in]   SlaveAddr   Address of the I2C slave device from which data is to be received.
 *
 * @note This function initiates the communication sequence, sends the slave address in read mode,
 *       and receives the requested data. Ensure the I2C peripheral is properly initialized and configured
 *       before calling this function.
 *
 * @pre The I2C peripheral must be initialized using I2C_Init() before using this function.
 * @post After the function returns, the pRxbuffer will contain the received data.
 *
 * @return None
 *
 * @attention This function blocks until all the data is received, so ensure it is used in applications where
 *            blocking behavior is acceptable.
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);


/**
 * @brief Sends data from the I2C master in interrupt mode with a specified repeated start condition.
 * 
 * This function initiates the transmission of data from the master device to the slave device
 * in interrupt mode. It allows specifying whether a repeated start condition should be generated.
 * 
 * @param[in]  pI2CHandle  Pointer to the I2C handle structure containing the I2C configuration.
 * @param[in]  pTxbuffer   Pointer to the data buffer that holds the data to be sent.
 * @param[in]  Len         Length of the data to be transmitted.
 * @param[in]  SlaveAddr   Address of the slave device to communicate with.
 * @param[in]  Sr          Repeated start condition control (1: enable, 0: disable).
 * 
 * @return uint8_t         Returns the state of the I2C bus (busy or idle).
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

/**
 * @brief Receives data from the I2C master in interrupt mode with a specified repeated start condition.
 * 
 * This function initiates the reception of data by the master device from a slave device
 * in interrupt mode. It allows specifying whether a repeated start condition should be generated.
 * 
 * @param[in]  pI2CHandle  Pointer to the I2C handle structure containing the I2C configuration.
 * @param[out] pRxbuffer   Pointer to the data buffer to store the received data.
 * @param[in]  Len         Length of the data to be received.
 * @param[in]  SlaveAddr   Address of the slave device to communicate with.
 * @param[in]  Sr          Repeated start condition control (1: enable, 0: disable).
 * 
 * @return uint8_t         Returns the state of the I2C bus (busy or idle).
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);



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
 * @brief Handles the I2C event interrupt.
 * 
 * This function is triggered by the event interrupt generated by the I2C peripheral. It processes
 * different I2C events such as start bit generation, address matching, byte transfer completion,
 * stop detection, and buffer-related events for transmission and reception.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration and
 *                       state information for the I2C peripheral.
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * @brief Handles the I2C error interrupt.
 * 
 * This function is triggered by the error interrupt generated by the I2C peripheral. It processes
 * various error conditions such as acknowledgment failure, bus errors, arbitration loss, and overrun
 * or underrun errors.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration and
 *                       state information for the I2C peripheral.
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/**
 * @brief Closes the I2C receive data process.
 * 
 * This function disables the relevant interrupt control bits for buffer and event handling, resets
 * the receive buffer and state variables, and restores the ACK control setting if it was enabled.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration and
 *                       state information for the I2C peripheral.
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/**
 * @brief Closes the I2C send data process.
 * 
 * This function disables the relevant interrupt control bits for buffer and event handling, resets
 * the transmission state and buffer variables, and restores the ACK control setting if it was enabled.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration and
 *                       state information for the I2C peripheral.
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

/**
 * @brief Application callback function for I2C events.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure.
 * @param[in] Event Event type that occurred.
 */
void I2C_AppEventCallback(I2C_Handle_t *pI2CHandle, uint8_t Event);


uint32_t RCC_GetAPB1_CLK(void);

#endif /* INC_STM32F401XX_I2C_DRIVER_H_ */
