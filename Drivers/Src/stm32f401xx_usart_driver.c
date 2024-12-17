/**
 * @file stm32f401xx_i2c_driver.c
 * @brief Source file for the USART driver for STM32F401xx microcontrollers.
 * 
 * This file provides function definitions for configuring and controlling the USART peripheral 
 * on STM32F401x
 *
 *  Created on: Dec 8, 2024
 *      Author: Mehmethan Türüt
 */

#include "../Inc/stm32f401xx.h"


/**
 * @brief Enables or disables the peripheral clock for the given I2C port.
 * 
 * @param[in] pUSARTx Pointer to the USART port base address.
 * @param[in] EnorDi Enable (1) or disable (0) the peripheral clock.
 */
void USART_PeriClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
    if(EnorDi) {
        // Enable the clock for the specified USART peripheral
        if (pUSARTx == USART1) {
            USART1_PCLK_EN();
        } else if (pUSARTx == USART2) {
            USART2_PCLK_EN();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_EN();
        }
    } else {
        // Disable the clock for the specified USART peripheral
        if (pUSARTx == USART1) {
            USART1_PCLK_DI();
        } else if (pUSARTx == USART2) {
            USART2_PCLK_DI();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_DI();
        }  
    }
}

/**
 * @brief  Controls the clock for the specified USART peripheral.
 * 
 * @param  pUSARTx   Pointer to the USART peripheral (USART1, USART2, or USART6).
 * @param  EnorDi    Enable (1) or Disable (0) the clock.
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi) {
    if (EnorDi) {
        // Enable the clock for the specified USART peripheral
        if (pUSARTx == USART1) {
            USART1_ENABLE();
        } else if (pUSARTx == USART2) {
            USART2_ENABLE();
        } else if (pUSARTx == USART6) {
            USART6_ENABLE();
        }
    } else {
        // Disable the clock for the specified USART peripheral
        if (pUSARTx == USART1) {
            USART1_DISABLE();
        } else if (pUSARTx == USART2) {
            USART1_DISABLE();
        } else if (pUSARTx == USART6) {
            USART6_DISABLE();
        }  
    }
}


/**
 * @brief Initializes the USART peripheral according to the specified parameters.
 * 
 * This function configures the USART peripheral based on the settings in the USART handle.
 * It enables the peripheral clock and sets the mode (RX, TX, or both).
 * 
 * @param[in] pUSARTHandle Pointer to the USART handle structure containing the configuration information.
 * 
 * @note This function must be called before using the USART peripheral.
 */
void USART_Init(USART_Handle_t *pUSARTHandle){
    USART_PeriClkCtrl(pUSARTHandle->pUSARTx, ENABLE);

    switch (pUSARTHandle->USART_config.USART_Mode)
    {
    case USART_MODE_ONLY_RX:
        pUSARTHandle->pUSARTx->USART_CR1_t.RE=1;
        break;
    case USART_MODE_ONLY_TX:
        pUSARTHandle->pUSARTx->USART_CR1_t.TE=1;
        break;
    case USART_MODE_TXRX:
        pUSARTHandle->pUSARTx->USART_CR1_t.TE=1;
        pUSARTHandle->pUSARTx->USART_CR1_t.RE=1;
        break;
    default:
        pUSARTHandle->pUSARTx->USART_CR1_t.TE=1;
        pUSARTHandle->pUSARTx->USART_CR1_t.RE=1;
        break;
    }

    pUSARTHandle->pUSARTx->USART_CR1_t.M= pUSARTHandle->USART_config.USART_WordLen;

    switch (pUSARTHandle->USART_config.USART_ParityControl)
    {
    case USART_PARITY_DISABLE:
        pUSARTHandle->pUSARTx->USART_CR1_t.PCE=0;
        break;
    case USART_PARITY_EN_EVEN:
        pUSARTHandle->pUSARTx->USART_CR1_t.PCE=1;
        pUSARTHandle->pUSARTx->USART_CR1_t.PS=0;
        break;
    case USART_PARITY_EN_ODD:
        pUSARTHandle->pUSARTx->USART_CR1_t.PCE=1;
        pUSARTHandle->pUSARTx->USART_CR1_t.PS=1;
        break;
    default:
            pUSARTHandle->pUSARTx->USART_CR1_t.PCE=0;
        break;
    }

    switch (pUSARTHandle->USART_config.USART_StopLen)
    {
    case USART_STOPBITS_1:
        pUSARTHandle->pUSARTx->USART_CR2_t.STOP=0;
        break;
    case USART_STOPBITS_0_5:
        pUSARTHandle->pUSARTx->USART_CR2_t.STOP=1;
        break;
    case USART_STOPBITS_2:
        pUSARTHandle->pUSARTx->USART_CR2_t.STOP=2;
        break;
    case USART_STOPBITS_1_5:
        pUSARTHandle->pUSARTx->USART_CR2_t.STOP=3;
        break;
    default:
        pUSARTHandle->pUSARTx->USART_CR2_t.STOP=0;
        break;
    }

    switch (pUSARTHandle->USART_config.USART_HWFlowControl)
    {
    case USART_HW_FLOW_CTRL_NONE:
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSE=0;
        break;
    case USART_HW_FLOW_CTRL_RTS:
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSE=0;
        pUSARTHandle->pUSARTx->USART_CR3_t.RTSE=1;
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSIE=0;
        break;
    case USART_HW_FLOW_CTRL_CTS:
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSE=1;
        pUSARTHandle->pUSARTx->USART_CR3_t.RTSE=0;
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSIE=1;
        break;
    case USART_HW_FLOW_CTRL_CTS_RTS:
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSE=1;
        pUSARTHandle->pUSARTx->USART_CR3_t.RTSE=1;
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSIE=1;
        break;
    default:
        pUSARTHandle->pUSARTx->USART_CR3_t.CTSE=0;
        break;
    }

    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_config.USART_Baud);
}

/**
 * @brief Deinitializes the given USART peripheral.
 * 
 * @param[in] pUSARTx Pointer to the USART peripheral.
 */
void USART_DeInit(USART_RegDef_t *pUSARTx){
    if (pUSARTx == USART1) {
        USART1_REG_RESET();
    } else if (pUSARTx == USART2) {
        USART2_REG_RESET();
    } else if (pUSARTx == USART6) {
        USART6_REG_RESET();
    }
}

/**
 * @brief Sends data through the USART peripheral.
 * 
 * This function transmits data through the USART peripheral by writing to the data register.
 * It waits for the transmit data register to be empty before sending each byte.
 * 
 * @param[in] pUSARTHandle Pointer to the USART handle structure containing the configuration information.
 * @param[in] pTxBuffer Pointer to the buffer containing the data to be transmitted.
 * @param[in] Len Length of the data to be transmitted.
 * 
 * @note This function blocks until all data is transmitted.
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){
    for (uint32_t i = 0; i <Len; i++)
    {
        while(!(pUSARTHandle->pUSARTx->USART_SR_t.TXE));
        
        if (pUSARTHandle->USART_config.USART_WordLen)
        {
            pUSARTHandle->pUSARTx->USART_DR_t.DR=(*((uint16_t*)pTxBuffer) & 0x01ffU);

            if (pUSARTHandle->USART_config.USART_ParityControl== USART_PARITY_DISABLE)
            {
                pTxBuffer++;
                pTxBuffer++;
            }else{
                pTxBuffer++;
            }  
        }else{
            pUSARTHandle->pUSARTx->USART_DR_t.DR=*pTxBuffer;
            pTxBuffer++;
        }  
    }  

    while(!(pUSARTHandle->pUSARTx->USART_SR_t.TC));
}



/**
 * @brief Receives data through the USART peripheral.
 * 
 * This function receives data through the USART peripheral by reading from the data register.
 * It waits for the receive data register to be full before reading each byte.
 * 
 * @param[in] pUSARTHandle Pointer to the USART handle structure containing the configuration information.
 * @param[in] pRxBuffer Pointer to the buffer to store the received data.
 * @param[in] Len Length of the data to be received.
 * 
 * @note This function blocks until all data is received.
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
    for (uint32_t i = 0; i < Len; i++)
    {
        while(!(pUSARTHandle->pUSARTx->USART_SR_t.RXNE));

        if (pUSARTHandle->USART_config.USART_WordLen){
            if (pUSARTHandle->USART_config.USART_ParityControl== USART_PARITY_DISABLE)
            {
                *pRxBuffer = pUSARTHandle->pUSARTx->USART_DR_t.DR & 0xff;
                pRxBuffer++;
                pRxBuffer++;           
            }else{
                *pRxBuffer = pUSARTHandle->pUSARTx->USART_DR_t.DR & 0x7f;
                pRxBuffer++;
            }
        }else{
            if(pUSARTHandle->USART_config.USART_ParityControl== USART_PARITY_DISABLE)
            {
                *pRxBuffer= (uint8_t)pUSARTHandle->pUSARTx->USART_DR_t.DR;
            }else{
                *pRxBuffer= (uint8_t)(pUSARTHandle->pUSARTx->USART_DR_t.DR & 0x7f);
            }
            pRxBuffer++;
        }  
    }
}


/**
 * @brief Sends data through the USART peripheral in interrupt mode.
 * 
 * This function sends data through the USART peripheral in interrupt mode. It sets the
 * transmit buffer and length in the USART handle structure and enables the TXEIE and TCIE
 * bits in the USART control register to generate interrupts.
 * 
 * @param[in] pUSARTHandle Pointer to the USART handle structure containing the configuration information.
 * @param[in] pTxBuffer Pointer to the buffer containing the data to be transmitted.
 * @param[in] Len Length of the data to be transmitted.
 * 
 * @return uint8_t State of the USART transmission (busy or ready).
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
    
    uint8_t txstate = pUSARTHandle->TxBusyState;

    if (pUSARTHandle->TxBusyState != USART_BUSY_IN_TX)
    {
        pUSARTHandle->TxLen = Len;
        pUSARTHandle->pTxBuffer = pTxBuffer;
        pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

        pUSARTHandle->pUSARTx->USART_CR1_t.TXEIE = 1;
        pUSARTHandle->pUSARTx->USART_CR1_t.TCIE = 1;
    }
    return txstate;
}

/**
 * @brief Receives data through the USART peripheral in interrupt mode.
 * 
 * This function receives data through the USART peripheral in interrupt mode. It sets the
 * receive buffer and length in the USART handle structure and enables the RXNEIE bit in the
 * USART control register to generate interrupts.
 * 
 * @param[in] pUSARTHandle Pointer to the USART handle structure containing the configuration information.
 * @param[in] pRxBuffer Pointer to the buffer to store the received data.
 * @param[in] Len Length of the data to be received.
 * 
 * @return uint8_t State of the USART reception (busy or ready).
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len){
    uint8_t rxstate = pUSARTHandle->RxBusyState;

    if (pUSARTHandle->RxBusyState != USART_BUSY_IN_RX)
    {
        pUSARTHandle->RxLen = Len;
        pUSARTHandle->pRxBuffer = pRxBuffer;
        pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        pUSARTHandle->pUSARTx->USART_CR1_t.RXNEIE = 1;
    }
    return rxstate;
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
void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
        //1. IPR register
    uint8_t iprx = IRQNumber/4;
    uint8_t iprx_section = IRQNumber%4;

    *(NVIC_PR_BASEADDR  +  (iprx))  |=   ((IRQPriority<<(8*iprx_section))<<4); //the first 4 bits of each section is non-implemented
}



/**
 * @brief Configures the baud rate for the USART peripheral.
 * 
 * This function calculates and sets the baud rate for the specified USART peripheral
 * based on the provided baud rate value and the peripheral clock frequency.
 * 
 * @param[in] pUSARTx Pointer to the USART peripheral base address.
 * @param[in] BaudRate Desired baud rate for the USART communication.
 * 
 * @note This function must be called after the USART peripheral is initialized.
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {
    //Variable to hold the APB clock
    uint32_t PCLKx;

    uint32_t usartdiv;

    //variables to hold Mantissa and Fraction values
    uint32_t M_part, F_part;

    if (pUSARTx == USART1 || pUSARTx == USART6) {
        PCLKx = RCC_GetAPB2_CLK();
    } else {
        PCLKx = RCC_GetAPB1_CLK();
    }

    //Check for OVER8 configuration bit
    if (pUSARTx->USART_CR1_t.OVER8) {
        //OVER8 = 1 , over sampling by 8
        usartdiv = ((25 * PCLKx) / (2 * BaudRate));
    } else {
        //over sampling by 16
        usartdiv = ((25 * PCLKx) / (4 * BaudRate));
    }

    //Calculate the Mantissa part
    M_part = usartdiv / 100;

    //Extract the fraction part
    F_part = (usartdiv - (M_part * 100));

    //Calculate the final fractional
    if (pUSARTx->USART_CR1_t.OVER8) {
        //OVER8 = 1 , over sampling by 8
        F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);
    } else {
        //over sampling by 16
        F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
    }

    pUSARTx->USART_BRR_t.DIV_Fraction = F_part;
    pUSARTx->USART_BRR_t.DIV_Mantissa = M_part;
}