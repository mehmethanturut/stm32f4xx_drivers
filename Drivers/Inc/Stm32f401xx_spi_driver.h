/**
 * @file stm32f401xx_spi_driver.h
 * @brief Header file for the GPIO driver for STM32F401xx microcontrollers
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


/*
@SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_SLAVE    0
#define SPI_DEVICE_MODE_MASTER   1

/*
@SPI_BusConfig
*/
#define SPI_BUS_CONFIG_FD           1   //full duplex
#define SPI_BUS_CONFIG_HD           2   //half duplex
#define SPI_BUS_CONFIG_SX_RX        3   //simplex rxonly

/*
@SPI_SclkSpeed
*/
#define SPI_SCLK_DIV2        0
#define SPI_SCLK_DIV4        1
#define SPI_SCLK_DIV8        2
#define SPI_SCLK_DIV16       3
#define SPI_SCLK_DIV32       4
#define SPI_SCLK_DIV64       5
#define SPI_SCLK_DIV128      6
#define SPI_SCLK_DIV256      7

/*
@SPI_DFF
*/
#define SPI_DFF_8           0
#define SPI_DFF_16          1

/*
@SPI_CPOL
*/
#define SPI_CPOL_HIGH       1
#define SPI_CPOL_LOW        0

/*
@SPI_CPHA
*/
#define SPI_CPHA_HIGH       1
#define SPI_CPHA_LOW        0

/*
@SPI_SSM
*/
#define SPI_SSM_EN          1   //software slave management enable
#define SPI_SSM_DI          0   //software slave management disable
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
