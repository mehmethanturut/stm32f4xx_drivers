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


static void SPI_TXE_IT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_IT_HANDLE(SPI_Handle_t *pSPIHandle);
static void SPI_OVF_IT_HANDLE(SPI_Handle_t *pSPIHandle);


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
            if ((pSPIx->SPI_SR_t.TXE))
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

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
        while(Len){
        while(1){
            if ((pSPIx->SPI_SR_t.RXNE))
                {
                break;
                } 
        }
        if (!(pSPIx->SPI_CR1_t.DFF))
        {
            *pRxBuffer= (uint8_t)(pSPIx->SPI_DR_t.DR);
            Len--;
            pRxBuffer++;
        }
        else if(pSPIx->SPI_CR1_t.DFF){
            *(uint16_t*)pRxBuffer= pSPIx->SPI_DR_t.DR;
            Len-=2;
            (uint16_t*)pRxBuffer++;
        }
    }
}


/**
 * @brief Configures the interrupt for a specific IRQ number.
 * 
 * @param[in] IRQNumber IRQ number to configure.
 * @param[in] EnorDi Enable (1) or Disable (0) the IRQ.
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
    if(EnorDi){
        if (IRQNumber<=31)
        {
            //ISER0 register
            *NVIC_ISER0 |=  (1<<IRQNumber);
        }
        
        
        else if (IRQNumber<=64  && IRQNumber>31)
        {
            //ISER1 register
            *NVIC_ISER1 |=  (1<<(IRQNumber%32));
        }

        else if (IRQNumber<=96  && IRQNumber>64)
        {
            //ISER2 register
            *NVIC_ISER2 |=  (1<<(IRQNumber%32));
        }
    }
    else{
        if (IRQNumber<=31)
        {
            //ICER0 register
            *NVIC_ICER0 |=  (1<<IRQNumber);
        }
        
        
        else if (IRQNumber<=64  && IRQNumber>31)
        {
            //ICER1 register
            *NVIC_ICER1 |=  (1<<(IRQNumber%32));
        }

        else if (IRQNumber<=96  && IRQNumber>64)
        {
            //ICER2 register
            *NVIC_ICER2 |=  (1<<(IRQNumber%32));
        }

    }
}

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
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
    //check for txe
    if(!(pSPIHandle->pSPIx->SPI_CR2_t.ERRIE)){
        if (pSPIHandle->pSPIx->SPI_SR_t.TXE && pSPIHandle->pSPIx->SPI_CR2_t.TXEIE)
        {
            //handle txe
            SPI_TXE_IT_HANDLE(pSPIHandle);
        }
        
        if (pSPIHandle->pSPIx->SPI_SR_t.RXNE && pSPIHandle->pSPIx->SPI_CR2_t.RXNEIE)
        {
            //handle rxe 
            SPI_RXNE_IT_HANDLE(pSPIHandle);
        }
    }
    else if(pSPIHandle->pSPIx->SPI_CR2_t.ERRIE){

        if (pSPIHandle->pSPIx->SPI_SR_t.OVF)
        {
            SPI_OVF_IT_HANDLE(pSPIHandle);
        }
    
    }
}



uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

    uint8_t state=pSPIHandle->TxState;
    if (state!=SPI_BSY_IN_TX)
    {
        pSPIHandle->pTxBuffer= pTxBuffer;
        pSPIHandle->TxLen = Len;

        pSPIHandle->TxState =SPI_BSY_IN_TX;
        pSPIHandle->pSPIx->SPI_CR2_t.TXEIE=1;
    }

    return state;



}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
    
    uint8_t state=pSPIHandle->RxState;
    if (state!=SPI_BSY_IN_RX)
    {
        pSPIHandle->pRxBuffer= pRxBuffer;
        pSPIHandle->RxLen = Len;

        pSPIHandle->RxState =SPI_BSY_IN_RX;
        pSPIHandle->pSPIx->SPI_CR2_t.RXNEIE=1;
    }

    return state;
}

//helper functions
static void SPI_TXE_IT_HANDLE(SPI_Handle_t *pSPIHandle){
        if (!(pSPIHandle->pSPIx->SPI_CR1_t.DFF))
        {
            pSPIHandle->pSPIx->SPI_DR_t.DR = *(pSPIHandle->pTxBuffer);
            pSPIHandle->TxLen--;
            pSPIHandle->pTxBuffer++;

        }
        else if(pSPIHandle->pSPIx->SPI_CR1_t.DFF){
            pSPIHandle->pSPIx->SPI_DR_t.DR=*(uint16_t*)pSPIHandle->pTxBuffer;
            pSPIHandle->TxLen-=2;
            (uint16_t*)pSPIHandle->pTxBuffer++;;
        }

        if (!(pSPIHandle->TxLen))
        {
            SPI_CloseTransmission(pSPIHandle);
            SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
        }
        
}
static void SPI_RXNE_IT_HANDLE(SPI_Handle_t *pSPIHandle){
    if (!(pSPIHandle->pSPIx->SPI_CR1_t.DFF))
    {
        *(pSPIHandle->pRxBuffer) = (uint8_t)(pSPIHandle->pSPIx->SPI_DR_t.DR);
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }
    else if(pSPIHandle->pSPIx->SPI_CR1_t.DFF){
        *(uint16_t*)pSPIHandle->pRxBuffer= pSPIHandle->pSPIx->SPI_DR_t.DR;
        pSPIHandle->RxLen=-2;
        (uint16_t*)pSPIHandle->pRxBuffer++;
    }
    
    if (!(pSPIHandle->RxLen))
    {
        SPI_CloseReception(pSPIHandle);
        SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
    }
}
static void SPI_OVF_IT_HANDLE(SPI_Handle_t *pSPIHandle){
    uint8_t temp;
    //clear ovr flag
    if(pSPIHandle->TxState !=SPI_BSY_IN_TX){
        temp = pSPIHandle->pSPIx->SPI_DR_t.DR;
        temp = pSPIHandle->pSPIx->SPI_SR_t.OVF;
    }
    (void) temp;

    SPI_AppEventCallback(pSPIHandle, SPI_EVENT_OVF_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
    uint8_t temp;
    temp = pSPIx->SPI_DR_t.DR;
    temp = pSPIx->SPI_SR_t.OVF;
    (void) temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
    pSPIHandle->pSPIx->SPI_CR2_t.TXEIE=0;
    pSPIHandle->pTxBuffer=NULL;
    pSPIHandle->TxLen=0;
    pSPIHandle->TxState=SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
    pSPIHandle->pSPIx->SPI_CR2_t.RXNEIE=0;
    pSPIHandle->pRxBuffer=NULL;
    pSPIHandle->RxLen=0;
        pSPIHandle->RxState=SPI_READY;
}


__attribute__((weak)) void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event){
    //weak

}