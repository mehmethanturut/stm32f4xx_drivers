/**
 * @file stm32f401xx_spi_driver.h
 * @brief Header file for the SPI driver for STM32F401xx microcontrollers.
 * 
 * This file contains the configuration structures, macros, and function 
 * prototypes to interface with the SPI peripheral on STM32F401xx devices.
 * 
 * Created on: Oct 31, 2024
 * Author: Mehmethan Türüt
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

/**
 * @brief Configuration structure for SPI peripheral.
 */
typedef struct
{
    uint8_t SPI_DeviceMode; /**< Specifies the device mode (Master/Slave) */
    uint8_t SPI_BusConfig;  /**< Specifies the bus configuration (Full-duplex/Half-duplex/Simplex) */
    uint8_t SPI_SclkSpeed;  /**< Specifies the SCLK speed */
    uint8_t SPI_DFF;        /**< Specifies the data frame format (8-bit/16-bit) */
    uint8_t SPI_CPOL;       /**< Specifies the clock polarity */
    uint8_t SPI_CPHA;       /**< Specifies the clock phase */
    uint8_t SPI_SSM;        /**< Software slave management enable/disable */
    uint8_t SPI_SSI;        /**< Internal slave select */
} SPI_Config_t;

/**
 * @brief Handle structure for SPI peripheral.
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;      /**< Pointer to the SPI base address */
    SPI_Config_t SPI_Config;  /**< SPI configuration settings */
    uint8_t *pTxBuffer;       /**< Pointer to the TX buffer */
    uint8_t *pRxBuffer;       /**< Pointer to the RX buffer */
    uint32_t TxLen;           /**< Length of TX data */
    uint32_t RxLen;           /**< Length of RX data */
    uint8_t TxState;          /**< Transmission state */
    uint8_t RxState;          /**< Reception state */
} SPI_Handle_t;

/** @defgroup SPI_DeviceMode SPI Device Mode */
#define SPI_DEVICE_MODE_SLAVE  0 /**< SPI Slave mode */
#define SPI_DEVICE_MODE_MASTER 1 /**< SPI Master mode */

/** @defgroup SPI_EVENTS SPI Events */
#define SPI_EVENT_TX_COMPLETE  1 /**< Transmission complete */
#define SPI_EVENT_RX_COMPLETE  2 /**< Reception complete */
#define SPI_EVENT_OVF_ERR      3 /**< Overflow error */

/** @defgroup SPI_BusConfig SPI Bus Configuration */
#define SPI_BUS_CONFIG_FD      1 /**< Full duplex */
#define SPI_BUS_CONFIG_HD      2 /**< Half duplex */
#define SPI_BUS_CONFIG_SX_RX   3 /**< Simplex RX only */

/** @defgroup SPI_SclkSpeed SPI SCLK Speed */
#define SPI_SCLK_DIV2          0 /**< SCLK speed DIV2 */
#define SPI_SCLK_DIV4          1 /**< SCLK speed DIV4 */
#define SPI_SCLK_DIV8          2 /**< SCLK speed DIV8 */
#define SPI_SCLK_DIV16         3 /**< SCLK speed DIV16 */
#define SPI_SCLK_DIV32         4 /**< SCLK speed DIV32 */
#define SPI_SCLK_DIV64         5 /**< SCLK speed DIV64 */
#define SPI_SCLK_DIV128        6 /**< SCLK speed DIV128 */
#define SPI_SCLK_DIV256        7 /**< SCLK speed DIV256 */

/** @defgroup SPI_DFF SPI Data Frame Format */
#define SPI_DFF_8              0 /**< 8-bit data frame format */
#define SPI_DFF_16             1 /**< 16-bit data frame format */

/** @defgroup SPI_CPOL SPI Clock Polarity */
#define SPI_CPOL_HIGH          1 /**< Clock polarity high */
#define SPI_CPOL_LOW           0 /**< Clock polarity low */

/** @defgroup SPI_CPHA SPI Clock Phase */
#define SPI_CPHA_HIGH          1 /**< Clock phase high */
#define SPI_CPHA_LOW           0 /**< Clock phase low */

/** @defgroup SPI_SSM SPI Software Slave Management */
#define SPI_SSM_EN             1 /**< Software slave management enable */
#define SPI_SSM_DI             0 /**< Software slave management disable */

/** @defgroup SPI_SSI SPI Slave Select */
#define SPI_SSI_SLAVE          0 /**< Slave select for slave mode */
#define SPI_SSI_MASTER         1 /**< Slave select for master mode */

/**
 * @brief Enables or disables the peripheral clock for the given SPI port.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 * @param[in] EnorDi Enable (1) or disable (0) the peripheral clock.
 */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief Initializes the SPI peripheral according to the specified parameters.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @brief Deinitializes the SPI peripheral, resetting it to its default state.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief Sends data over SPI in blocking mode.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 * @param[in] pTxBuffer Pointer to the transmission buffer.
 * @param[in] Len Length of data to transmit.
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/**
 * @brief Receives data over SPI in blocking mode.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 * @param[in] pRxBuffer Pointer to the reception buffer.
 * @param[in] Len Length of data to receive.
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief Sends data over SPI in interrupt mode.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] pTxBuffer Pointer to the transmission buffer.
 * @param[in] Len Length of data to transmit.
 * 
 * @return Status of the transmission process.
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);

/**
 * @brief Receives data over SPI in interrupt mode.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] pRxBuffer Pointer to the reception buffer.
 * @param[in] Len Length of data to receive.
 * 
 * @return Status of the reception process.
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief Configures the interrupt for the specified IRQ number.
 * 
 * @param[in] IRQNumber IRQ number to configure.
 * @param[in] EnorDi Enable (1) or disable (0) the IRQ.
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Sets the priority of the specified IRQ.
 * 
 * @param[in] IRQNumber IRQ number to configure priority for.
 * @param[in] IRQPriority Priority level to set.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Handles the interrupt request for the SPI peripheral.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/**
 * @brief Clears the overrun flag for the SPI peripheral.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

/**
 * @brief Closes the SPI transmission.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

/**
 * @brief Closes the SPI reception.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/**
 * @brief Application callback function for SPI events.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] Event Event type that occurred.
 */
void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event);

#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
