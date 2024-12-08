/**
 * @file stm32f401xx_i2c_driver.h
 * @brief Header file for the USART driver for STM32F401xx microcontrollers.
 * 
 * This file contains the configuration structures, macros, and function 
 * prototypes to interface with the USART peripheral on STM32F401xx devices.
 * 
 *  Created on: Dec 8, 2024
 *      Author: Mehmethan Türüt
 */

#ifndef INC_STM32F401XX_USART_DRIVER_H_
#define INC_STM32F401XX_USART_DRIVER_H_

#include "stm32f401xx.h"

typedef struct
{
    uint32_t USART_Baud;
    uint8_t  USART_Mode;
    uint8_t  USART_StopLen;
    uint8_t  USART_WordLen;
    uint8_t  USART_ParityControl;
    uint8_t  USART_HWFlowControl;
} USART_Config_t;

typedef struct
{
    USART_RegDef_t *pUSARTx;
    USART_Config_t USART_config;
} USART_Handle_t;

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


/**
 * @brief Enables or disables the peripheral clock for the given I2C port.
 * 
 * @param[in] pUSARTx Pointer to the USART port base address.
 * @param[in] EnorDi Enable (1) or disable (0) the peripheral clock.
 */
void USART_PeriClkCtrl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/**
 * @brief  Controls the clock for the specified USART peripheral.
 * 
 * @param  pUSARTx   Pointer to the USART peripheral (USART1, USART2, or USART6).
 * @param  EnorDi    Enable (1) or Disable (0) the clock.
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);


void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusflagName);
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
void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Configures the priority for a specific IRQ.
 * 
 * @param[in] IRQNumber IRQ number for which priority is to be set.
 * @param[in] IRQPriority Priority level for the IRQ.
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
#endif /* INC_STM32F401XX_USART_DRIVER_H_ */
