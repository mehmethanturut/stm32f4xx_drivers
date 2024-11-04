/**
 * @file stm32f401xx_spi_driver.c
 * @brief Source file for the SPI driver for STM32F401xx microcontrollers.
 * 
 * This file provides function definitions for configuring and controlling the SPI peripheral 
 * on STM32F401xx devices, including initialization, data transmission, and interrupt handling.
 * 
 * Created on: Oct 31, 2024
 * Author: Mehmethan Türüt
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
void SPI_Init(SPI_Handle_t *pSPIHandle){

    SPI_PeriClkCtrl(pSPIHandle->pSPIx,ENABLE);

    uint32_t *pTemp;
    pTemp =(uint32_t*) &(pSPIHandle->pSPIx->SPI_CR1_t);
    *pTemp = 0;

    //0.    configure ssm
    pSPIHandle->pSPIx->SPI_CR1_t.SSM=pSPIHandle->SPI_Config.SPI_SSM;

    //1. configure the SPI device mode
    pSPIHandle->pSPIx->SPI_CR1_t.MSTR=(pSPIHandle->SPI_Config.SPI_DeviceMode);


    //2. configure the SPI bus config
    if (pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_FD)
    {
        //bidi mode should be cleared 
        pSPIHandle->pSPIx->SPI_CR1_t.BIDIMODE=0;
    }
    else if (pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_HD)
    {
        //bidi mode should be set 
        pSPIHandle->pSPIx->SPI_CR1_t.BIDIMODE=1;
    }
    else if (pSPIHandle->SPI_Config.SPI_BusConfig==SPI_BUS_CONFIG_SX_RX)
    {
        //bidi mode should be cleared
        //RXONLY should be set
        pSPIHandle->pSPIx->SPI_CR1_t.BIDIMODE=0;
        pSPIHandle->pSPIx->SPI_CR1_t.RXONLY=1;
    }
    
    //3. configure the SPI clock speed
    pSPIHandle->pSPIx->SPI_CR1_t.BR=pSPIHandle->SPI_Config.SPI_SclkSpeed;

    //4. configure the Data Rate Format
    pSPIHandle->pSPIx->SPI_CR1_t.DFF=pSPIHandle->SPI_Config.SPI_DFF;

    //5. configure the CPOL
    pSPIHandle->pSPIx->SPI_CR1_t.CPOL=pSPIHandle->SPI_Config.SPI_CPOL;

    //6. configure the CPHA
    pSPIHandle->pSPIx->SPI_CR1_t.CPHA=pSPIHandle->SPI_Config.SPI_CPHA;

    //7. configure the ssi
    if(pSPIHandle->SPI_Config.SPI_SSM){
        pSPIHandle->pSPIx->SPI_CR1_t.SSI=pSPIHandle->SPI_Config.SPI_SSI;
    }
    
    
}


/**
 * @brief Deinitializes the SPI port, resetting it to its default state.
 * 
 * @param[in] pGPIOx Pointer to the SPI port base address.
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){
        if (pSPIx==SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (pSPIx==SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (pSPIx==SPI3)
    {
        SPI3_REG_RESET();
    }
    else if (pSPIx==SPI4)
    {
        SPI4_REG_RESET();
    }
}

/**
 * @brief Sends data over SPI.
 * 
 * This function transmits data through the specified SPI peripheral. The data to be sent 
 * is provided via a buffer, and the length of the data is specified by the caller.
 * 
 * @param[in] pSPIx Pointer to the SPI peripheral base address.
 * @param[in] pTxBuffer Pointer to the buffer holding the data to be transmitted.
 * @param[in] Len Length of the data to be transmitted.
 * 
 * @return None
 *  *
 * @Note  This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

    while(Len){
        while(1){
            if (!(pSPIx->SPI_SR_t.TXE))
            {
                break;
            }        
        }
        if (!(pSPIx->SPI_CR1_t.DFF))
        {
            pSPIx->SPI_DR_t.DR=*pTxBuffer;
            Len--;
            pTxBuffer++;
        }
        else if(pSPIx->SPI_CR1_t.DFF){
            pSPIx->SPI_DR_t.DR=*(uint16_t*)pTxBuffer;
            Len-=2;
            (uint16_t*)pTxBuffer++;
        }
    }
}

/**
 * @brief Receives data over SPI.
 * 
 * This function receives data through the specified SPI peripheral. The data is received
 * into a buffer provided by the caller, and the length of data to be received is specified.
 * 
 * @param[in] pSPIx Pointer to the SPI peripheral base address.
 * @param[out] pRxBuffer Pointer to the buffer where the received data will be stored.
 * @param[in] Len Length of the data to be received.
 * 
 * @return None
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


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