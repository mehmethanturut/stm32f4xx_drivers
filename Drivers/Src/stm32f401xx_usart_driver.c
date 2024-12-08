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
            USART2_DISABLE();
        } else if (pUSARTx == USART6) {
            USART6_DISABLE();
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

