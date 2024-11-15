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

#include "../Inc/stm32f401xx.h"

/**
 * @brief Handles SPI TXE (Transmit buffer empty) interrupt.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
static void SPI_TXE_IT_HANDLE(SPI_Handle_t *pSPIHandle);

/**
 * @brief Handles SPI RXNE (Receive buffer not empty) interrupt.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
static void SPI_RXNE_IT_HANDLE(SPI_Handle_t *pSPIHandle);

/**
 * @brief Handles SPI overflow error interrupt.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
static void SPI_OVF_IT_HANDLE(SPI_Handle_t *pSPIHandle);

/**
 * @brief Enables or disables the peripheral clock for a SPI port.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 * @param[in] EnorDi Enable (1) or Disable (0) the peripheral clock.
 */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if(EnorDi) {
        // Enable the clock for the specified SPI peripheral
        if (pSPIx == SPI1) {
            SPI1_PCLK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_EN();
        } else if (pSPIx == SPI4) {
            SPI4_PCLK_EN();
        }
    } else {
        // Disable the clock for the specified SPI peripheral
        if (pSPIx == SPI1) {
            SPI1_PCLK_DI();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_DI();
        } else if (pSPIx == SPI4) {
            SPI4_PCLK_DI();
        }  
    }
}

/**
 * @brief Initializes the SPI peripheral.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) {
    // Enable the peripheral clock for the SPI module
    SPI_PeriClkCtrl(pSPIHandle->pSPIx, ENABLE);

    // Configure SPI Control Register 1 (CR1) to reset the SPI settings
    uint32_t *pTemp = (uint32_t*) &(pSPIHandle->pSPIx->SPI_CR1_t);
    *pTemp = 0;

    // Configure Software Slave Management (SSM)
    pSPIHandle->pSPIx->SPI_CR1_t.SSM = pSPIHandle->SPI_Config.SPI_SSM;

    // Set the SPI device mode (Master/Slave)
    pSPIHandle->pSPIx->SPI_CR1_t.MSTR = pSPIHandle->SPI_Config.SPI_DeviceMode;

    // Configure the SPI bus mode (Full-duplex, Half-duplex, or Simplex RX-only)
    if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
        pSPIHandle->pSPIx->SPI_CR1_t.BIDIMODE = 0; // Full-duplex
    } else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
        pSPIHandle->pSPIx->SPI_CR1_t.BIDIMODE = 1; // Half-duplex
    } else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SX_RX) {
        pSPIHandle->pSPIx->SPI_CR1_t.BIDIMODE = 0;
        pSPIHandle->pSPIx->SPI_CR1_t.RXONLY = 1; // Simplex RX-only
    }

    // Set the SPI clock speed by configuring the baud rate
    pSPIHandle->pSPIx->SPI_CR1_t.BR = pSPIHandle->SPI_Config.SPI_SclkSpeed;

    // Configure data format (8-bit/16-bit data frame format)
    pSPIHandle->pSPIx->SPI_CR1_t.DFF = pSPIHandle->SPI_Config.SPI_DFF;

    // Configure clock polarity (CPOL) and clock phase (CPHA)
    pSPIHandle->pSPIx->SPI_CR1_t.CPOL = pSPIHandle->SPI_Config.SPI_CPOL;
    pSPIHandle->pSPIx->SPI_CR1_t.CPHA = pSPIHandle->SPI_Config.SPI_CPHA;

    // Configure SSI bit based on the SSM configuration
    if(pSPIHandle->SPI_Config.SPI_SSM) {
        pSPIHandle->pSPIx->SPI_CR1_t.SSI = pSPIHandle->SPI_Config.SPI_SSI;
    }
}

/**
 * @brief Deinitializes the SPI peripheral, resetting it to its default state.
 * 
 * @param[in] pSPIx Pointer to the SPI port base address.
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    // Reset the SPI peripheral
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    } else if (pSPIx == SPI4) {
        SPI4_REG_RESET();
    }
}

/**
 * @brief Sends data over SPI in blocking mode.
 * 
 * @param[in] pSPIx Pointer to the SPI peripheral base address.
 * @param[in] pTxBuffer Pointer to the buffer holding data to be transmitted.
 * @param[in] Len Length of the data to be transmitted.
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
    while(Len) {
        // Wait until the TXE (Transmit buffer empty) flag is set
        while(!(pSPIx->SPI_SR_t.TXE));

        // Check data frame format (DFF) to send data in 8-bit or 16-bit chunks
        if (!(pSPIx->SPI_CR1_t.DFF)) {
            pSPIx->SPI_DR_t.DR = *pTxBuffer;  // Send 8-bit data
            Len--;
            pTxBuffer++;
        } else {
            pSPIx->SPI_DR_t.DR = *(uint16_t*)pTxBuffer;  // Send 16-bit data
            Len -= 2;
            (uint16_t*)pTxBuffer++;
        }
    }
}

/**
 * @brief Receives data over SPI in blocking mode.
 * 
 * @param[in] pSPIx Pointer to the SPI peripheral base address.
 * @param[out] pRxBuffer Pointer to the buffer where received data will be stored.
 * @param[in] Len Length of the data to be received.
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
    while(Len) {
        // Wait until the RXNE (Receive buffer not empty) flag is set
        while(!(pSPIx->SPI_SR_t.RXNE));

        // Check data frame format (DFF) to receive data in 8-bit or 16-bit chunks
        if (!(pSPIx->SPI_CR1_t.DFF)) {
            *pRxBuffer = (uint8_t)(pSPIx->SPI_DR_t.DR);  // Receive 8-bit data
            Len--;
            pRxBuffer++;
        } else {
            *(uint16_t*)pRxBuffer = pSPIx->SPI_DR_t.DR;  // Receive 16-bit data
            Len -= 2;
            (uint16_t*)pRxBuffer++;
        }
    }
}


/**
 * @brief Configures the interrupt for a specific IRQ number.
 * 
 * This function enables or disables the interrupt for the specified IRQ number by setting
 * or clearing the respective bit in the NVIC ISER (Interrupt Set-Enable Register) or 
 * ICER (Interrupt Clear-Enable Register), based on the range of the IRQ number.
 * 
 * @param[in] IRQNumber IRQ number to configure.
 *                      - If 0 <= IRQNumber <= 31, ISER0/ICER0 is used.
 *                      - If 32 <= IRQNumber <= 63, ISER1/ICER1 is used.
 *                      - If 64 <= IRQNumber <= 95, ISER2/ICER2 is used.
 * @param[in] EnorDi Enable (1) or Disable (0) the IRQ.
 * 
 * @note This function directly accesses NVIC registers to manage interrupts.
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    if(EnorDi) {
        // Enable IRQ based on IRQ number range
        if (IRQNumber <= 31) {
            // Set interrupt enable for IRQ numbers 0-31 in ISER0
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if (IRQNumber <= 64 && IRQNumber > 31) {
            // Set interrupt enable for IRQ numbers 32-63 in ISER1
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber <= 96 && IRQNumber > 64) {
            // Set interrupt enable for IRQ numbers 64-95 in ISER2
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        }
    } else {
        // Disable IRQ based on IRQ number range
        if (IRQNumber <= 31) {
            // Clear interrupt enable for IRQ numbers 0-31 in ICER0
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if (IRQNumber <= 64 && IRQNumber > 31) {
            // Clear interrupt enable for IRQ numbers 32-63 in ICER1
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber <= 96 && IRQNumber > 64) {
            // Clear interrupt enable for IRQ numbers 64-95 in ICER2
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }
}


/**
 * @brief Configures the priority for a specific IRQ.
 * 
 * @param[in] IRQNumber IRQ number for which priority is to be set.
 * @param[in] IRQPriority Priority level for the IRQ.
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
    //1. IPR register
    uint8_t iprx = IRQNumber/4;
    uint8_t iprx_section = IRQNumber%4;

    *(NVIC_PR_BASEADDR  +  (iprx))  |=   ((IRQPriority<<(8*iprx_section))<<4); //the first 4 bits of each section is non-implemented
}


/**
 * @brief Handles the interrupt request for the SPI peripheral.
 * 
 * This function checks for TXE, RXNE, and error flags and calls respective handlers.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
    // Check for TXE interrupt
    if(!(pSPIHandle->pSPIx->SPI_CR2_t.ERRIE)) {
        if (pSPIHandle->pSPIx->SPI_SR_t.TXE && pSPIHandle->pSPIx->SPI_CR2_t.TXEIE) {
            // Handle TXE
            SPI_TXE_IT_HANDLE(pSPIHandle);
        }

        // Check for RXNE interrupt
        if (pSPIHandle->pSPIx->SPI_SR_t.RXNE && pSPIHandle->pSPIx->SPI_CR2_t.RXNEIE) {
            // Handle RXNE
            SPI_RXNE_IT_HANDLE(pSPIHandle);
        }
    }
    // Check for overflow error interrupt
    else if(pSPIHandle->pSPIx->SPI_CR2_t.ERRIE) {
        if (pSPIHandle->pSPIx->SPI_SR_t.OVF) {
            SPI_OVF_IT_HANDLE(pSPIHandle);
        }
    }
}

/**
 * @brief Sends data over SPI in interrupt mode.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] pTxBuffer Pointer to the buffer holding data to be transmitted.
 * @param[in] Len Length of the data to be transmitted.
 * 
 * @return Transmission state.
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BSY_IN_TX) {
        // Store the Tx buffer address and length, set the Tx state, and enable TXEIE
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        pSPIHandle->TxState = SPI_BSY_IN_TX;
        pSPIHandle->pSPIx->SPI_CR2_t.TXEIE = 1;
    }

    return state;
}

/**
 * @brief Receives data over SPI in interrupt mode.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] pRxBuffer Pointer to the buffer where received data will be stored.
 * @param[in] Len Length of the data to be received.
 * 
 * @return Reception state.
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BSY_IN_RX) {
        // Store the Rx buffer address and length, set the Rx state, and enable RXNEIE
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;
        pSPIHandle->RxState = SPI_BSY_IN_RX;
        pSPIHandle->pSPIx->SPI_CR2_t.RXNEIE = 1;
    }

    return state;
}

/**
 * @brief Handles SPI Transmit Buffer Empty (TXE) interrupt.
 *
 * This function checks if the SPI is in 8-bit or 16-bit data format mode. If in
 * 8-bit mode, it loads one byte of data from the transmit buffer to the SPI data
 * register. If in 16-bit mode, it loads two bytes of data from the transmit buffer
 * to the SPI data register. After each data transmission, the transmit buffer pointer
 * is incremented, and the transmission length is decremented.
 *
 * When the transmission length reaches zero, the transmission is closed, and
 * the user callback for transmission completion (`SPI_EVENT_TX_COMPLETE`) is invoked.
 *
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
static void SPI_TXE_IT_HANDLE(SPI_Handle_t *pSPIHandle)
{
    // Check if data frame format is 8-bit
    if (!(pSPIHandle->pSPIx->SPI_CR1_t.DFF))
    {
        // Load 1 byte of data to the SPI data register
        pSPIHandle->pSPIx->SPI_DR_t.DR = *(pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen--;           // Decrement transmission length
        pSPIHandle->pTxBuffer++;       // Increment transmit buffer pointer
    }
    // Check if data frame format is 16-bit
    else if (pSPIHandle->pSPIx->SPI_CR1_t.DFF)
    {
        // Load 2 bytes of data to the SPI data register
        pSPIHandle->pSPIx->SPI_DR_t.DR = *(uint16_t*)pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen -= 2;        // Decrement transmission length by 2
        (uint16_t*)pSPIHandle->pTxBuffer++; // Increment transmit buffer pointer
    }

    // Check if all data has been transmitted
    if (!(pSPIHandle->TxLen))
    {
        SPI_CloseTransmission(pSPIHandle); // Close transmission
        SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE); // Notify completion
    }
}

/**
 * @brief Handles SPI Receive Buffer Not Empty (RXNE) interrupt.
 *
 * This function checks if the SPI is in 8-bit or 16-bit data format mode. If in
 * 8-bit mode, it reads one byte of data from the SPI data register into the receive
 * buffer. If in 16-bit mode, it reads two bytes of data from the SPI data register
 * into the receive buffer. After each data reception, the receive buffer pointer
 * is incremented, and the reception length is decremented.
 *
 * When the reception length reaches zero, the reception is closed, and
 * the user callback for reception completion (`SPI_EVENT_RX_COMPLETE`) is invoked.
 *
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
static void SPI_RXNE_IT_HANDLE(SPI_Handle_t *pSPIHandle)
{
    // Check if data frame format is 8-bit
    if (!(pSPIHandle->pSPIx->SPI_CR1_t.DFF))
    {
        // Read 1 byte of data from the SPI data register
        *(pSPIHandle->pRxBuffer) = (uint8_t)(pSPIHandle->pSPIx->SPI_DR_t.DR);
        pSPIHandle->RxLen--;           // Decrement reception length
        pSPIHandle->pRxBuffer++;       // Increment receive buffer pointer
    }
    // Check if data frame format is 16-bit
    else if (pSPIHandle->pSPIx->SPI_CR1_t.DFF)
    {
        // Read 2 bytes of data from the SPI data register
        *(uint16_t*)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->SPI_DR_t.DR;
        pSPIHandle->RxLen -= 2;        // Decrement reception length by 2
        (uint16_t*)pSPIHandle->pRxBuffer++; // Increment receive buffer pointer
    }

    // Check if all data has been received
    if (!(pSPIHandle->RxLen))
    {
        SPI_CloseReception(pSPIHandle); // Close reception
        SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE); // Notify completion
    }
}

/**
 * @brief Handles SPI Overflow (OVF) interrupt.
 *
 * This function clears the overflow flag if the SPI is not currently in a 
 * transmission state. It does so by reading the data register (DR) and the 
 * overflow flag (OVF) from the status register (SR). This prevents further
 * overflow errors by resetting the internal overflow condition.
 *
 * Once the overflow flag is cleared, it calls the user callback for overflow
 * error handling (`SPI_EVENT_OVF_ERR`).
 *
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
static void SPI_OVF_IT_HANDLE(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    
    // Clear overflow flag if SPI is not busy transmitting
    if (pSPIHandle->TxState != SPI_BSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->SPI_DR_t.DR;  // Read data register to clear flag
        temp = pSPIHandle->pSPIx->SPI_SR_t.OVF; // Read overflow flag from status register
    }

    // Suppress unused variable warning
    (void)temp;

    // Notify application about the overflow error
    SPI_AppEventCallback(pSPIHandle, SPI_EVENT_OVF_ERR);
}


/**
 * @brief Clears the overrun flag for the SPI peripheral.
 * 
 * @param[in] pSPIx Pointer to the SPI peripheral base address.
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
    // Clear the OVR flag by reading the DR and SR registers
    uint8_t temp;
    temp = pSPIx->SPI_DR_t.DR;
    temp = pSPIx->SPI_SR_t.OVF;
    (void)temp;
}

/**
 * @brief Closes the SPI transmission and resets transmission parameters.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
    // Disable TXEIE, reset Tx buffer and length, and set Tx state to ready
    pSPIHandle->pSPIx->SPI_CR2_t.TXEIE = 0;
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

/**
 * @brief Closes the SPI reception and resets reception parameters.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
    // Disable RXNEIE, reset Rx buffer and length, and set Rx state to ready
    pSPIHandle->pSPIx->SPI_CR2_t.RXNEIE = 0;
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

/**
 * @brief Weak function to handle SPI application event callbacks.
 * 
 * This function is meant to be overridden by the application to handle SPI events.
 * 
 * @param[in] pSPIHandle Pointer to the SPI handle structure.
 * @param[in] Event Event type that occurred.
 */
__attribute__((weak)) void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event) {
    // Weak implementation; application can override to handle specific SPI events
}