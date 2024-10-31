/*
 * Stm32f401xx_spi_driver.h
 *
 *  Created on: Oct 30, 2024
 *      Author: mehme
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"


/*
***** Configuration structure for SPI
*/


typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;


/*
***** handle structure for SPI
*/

typedef struct
{
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPI_Config;
}SPI_Handle_t;


/*******************************************************************************************
                            APIs supported by this driver 
********************************************************************************************/


/** 
 * @brief Enables or disables peripheral clock for a SPI port.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 * @param[in] EnorDi Enable (1) or Disable (0) the peripheral clock.
 */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief Initializes the SPI port.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @brief Deinitializes the SPI port, resetting it to its default state.
 * 
 * @param[in] pGPIOx Pointer to the SPI port base address.
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief Reads the value of a specific GPIO pin.
 * 
 * @param[in] pGPIOx Pointer to the GPIO port base address.
 * @param[in] PinNumber Pin number to be read.
 * 
 * @return The pin state (0 or 1).
 */
void SPI_SendData(GPIO_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(GPIO_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


/**
 * @brief Configures the interrupt for a specific IRQ number.
 * 
 * @param[in] IRQNumber IRQ number to configure.
 * @param[in] EnorDi Enable (1) or Disable (0) the IRQ.
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Configures the priority for a specific IRQ.
 * 
 * @param[in] IRQNumber IRQ number for which priority is to be set.
 * @param[in] IRQPriority Priority level for the IRQ.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief ISR handler for a SPI port.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
