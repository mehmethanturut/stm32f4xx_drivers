/**
 * @file stm32f401xx_i2c_driver.c
 * @brief Source file for the I2C driver for STM32F401xx microcontrollers.
 * 
 * This file provides function definitions for configuring and controlling the I2C peripheral 
 * on STM32F401xx devices, including initialization, data transmission, and interrupt handling.
 * 
 * Created on: Oct 31, 2024
 * Author: Mehmethan Türüt
 */

/**
 * @brief Enables or disables the peripheral clock for the given I2C peripheral
 * 
 * @param pI2Cx   Base address of the I2C peripheral (I2C1, I2C2, or I2C3)
 * @param EnorDi  ENABLE or DISABLE macros to enable or disable the clock
 */
void I2C_PeriClkCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if(EnorDi) {
        // Enable the clock for the specified I2C peripheral
        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        // Disable the clock for the specified I2C peripheral
        if (pI2Cx == I2C1) {
            I2C1_PCLK_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_DI();
        }
    }
}

/**
 * @brief Initializes the given I2C peripheral
 * 
 * @param pI2C   Base address of the I2C peripheral to initialize
 */
void I2C_Init(I2C_Regdef_t *pI2C){

    
}

/**
 * @brief Deinitializes (resets) the specified I2C peripheral
 * 
 * @param pI2Cx   Base address of the I2C peripheral (I2C1, I2C2, or I2C3)
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
    // Reset the I2C peripheral
    if (pI2Cx == I2C1) {
        I2C1_REG_RESET();
    } else if (pI2Cx == I2C2) {
        I2C2_REG_RESET();
    } else if (pI2Cx == I2C3) {
        I2C3_REG_RESET();
    }
}

/**
 * @brief Configures the interrupt for the specified IRQ number
 * 
 * @param IRQNumber   IRQ number to configure
 * @param EnorDi      ENABLE or DISABLE macros to enable or disable the IRQ
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi) {
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
 * @brief Configures the priority of the specified IRQ number
 * 
 * @param IRQNumber    IRQ number to configure
 * @param IRQPriority  Priority level to set for the IRQ
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
    //1. IPR register
    uint8_t iprx = IRQNumber/4;
    uint8_t iprx_section = IRQNumber%4;

    *(NVIC_PR_BASEADDR  +  (iprx))  |=   ((IRQPriority<<(8*iprx_section))<<4); //the first 4 bits of each section is non-implemented
}
