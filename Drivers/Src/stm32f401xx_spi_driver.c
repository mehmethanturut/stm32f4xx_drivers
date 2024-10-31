/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Oct 31, 2024
 *      Author: Mehmethan Türüt
 */

#include "stm32f401xx.h"




/*******************************************************************************************
                            APIs supported by this driver 
********************************************************************************************/


/** 
 * @brief Enables or disables peripheral clock for a SPI port.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 * @param[in] EnorDi Enable (1) or Disable (0) the peripheral clock.
 */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    if(EnorDi){
        if (pSPIx==SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx==SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx==SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx==SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else{
        if (pSPIx==SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx==SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx==SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx==SPI4)
        {
            SPI4_PCLK_DI();
        }  
    }
}

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