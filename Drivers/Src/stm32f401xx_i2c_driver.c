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
#include "stm32f401xx.h"


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
 * @brief Initializes the specified I2C peripheral based on the provided configuration.
 * 
 * This function sets up an I2C peripheral by enabling the peripheral clock, configuring 
 * its control registers, and setting parameters such as ACK behavior, frequency, own address, 
 * clock control register, and rise time. The function supports both standard and fast modes.
 * 
 * @param[in] pI2CHandle  Pointer to the `I2C_Handle_t` structure containing the base address 
 *                        of the I2C peripheral and its configuration settings.
 * 
 * @note 
 * - The `I2C_OAR1_t.res14` bit (14th bit of the `I2C_OAR1` register) must always be set to 1 
 *   to comply with the I2C specification.
 * - Ensure that the `RCC_GetAPB1_CLK()` function returns the correct clock frequency, as it is 
 *   used to calculate the required values for frequency, CCR, and rise time.
 * 
 * @pre 
 * - The I2C handle (`pI2CHandle`) must be initialized with valid configuration values.
 * - The APB1 clock for the I2C peripheral must be configured before calling this function.
 * 
 * @attention 
 * - The clock speed for standard mode (up to 100 kHz) and fast mode (up to 400 kHz) is calculated 
 *   based on the APB1 clock frequency. Incorrect configurations might result in communication issues.
 * - Fast mode configuration considers the duty cycle based on the `I2C_FMDutyCycle` parameter.
 * - Ensure the peripheral is enabled before starting I2C communication.
 * 
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){

    // enable peripheral clock
    I2C_PeriClkCtrl(pI2CHandle->pI2Cx, ENABLE);

    // configure ack bit
    pI2CHandle->pI2Cx->I2C_CR1_t.ACK=1;

    //configure FREQ 
    pI2CHandle->pI2Cx->I2C_CR2_t.FREQ= (RCC_GetAPB1_CLK()/1000000U);

    // I2C_OAR1 registers 14th bit(reserved) must be kept 1 by software
    pI2CHandle->pI2Cx->I2C_OAR1_t.res14=1;

    //program the devices own adress
    pI2CHandle->pI2Cx->I2C_OAR1_t.ADD= pI2CHandle->I2C_Config.I2C_DeviceAddress;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed<= I2C_SCL_SPEED_SM)
    {
        // standart mode
        pI2CHandle->pI2Cx->I2C_CCR_t.FS=0;
        pI2CHandle->pI2Cx->I2C_CCR_t.CCR= (RCC_GetAPB1_CLK() / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed));// i aint explaning this shit
    }
    else{
        // fast mode
        pI2CHandle->pI2Cx->I2C_CCR_t.FS=1;
        if (!pI2CHandle->I2C_Config.I2C_FMDutyCycle)
        {
            pI2CHandle->pI2Cx->I2C_CCR_t.CCR= (RCC_GetAPB1_CLK() / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        else{
            pI2CHandle->pI2Cx->I2C_CCR_t.CCR= (RCC_GetAPB1_CLK() / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        
            
    }
    
    
    
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed<= I2C_SCL_SPEED_SM)
    {
        // standart mode
        pI2CHandle->pI2Cx->I2C_TRISE_t.TRISE = (RCC_GetAPB1_CLK() /1000000U)+1;
    }
    else{
        // fast mode
        pI2CHandle->pI2Cx->I2C_TRISE_t.TRISE = ((RCC_GetAPB1_CLK() * 300 )/1000000000U)+1;
        
            
    }

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
 * @brief Sends data from the I2C master to a specified slave device.
 * 
 * This function initiates a communication sequence from the I2C master to a target 
 * slave device and sends the specified number of bytes from the given transmit buffer.
 * It handles the start condition, address transmission, and data transfer, and concludes 
 * the operation by generating a stop condition.
 * 
 * @param[in] pI2CHandle  Pointer to the I2C handle structure, which contains the base 
 *                        address of the I2C peripheral and its configuration.
 * @param[in] pTxbuffer   Pointer to the buffer containing the data to be transmitted.
 * @param[in] Len         Number of bytes to be transmitted from the buffer.
 * @param[in] SlaveAddr   7-bit address of the target slave device.
 * 
 * @note
 * - This function assumes the I2C peripheral is already configured and enabled.
 * - The slave address is shifted left to include the write bit.
 * - Ensure that the slave device is ready to receive data before calling this function.
 * - The function is blocking and will wait until each stage of the I2C communication 
 *   process is complete.
 * 
 * @pre 
 * - The I2C peripheral must be initialized and configured in master mode.
 * - The target slave device should acknowledge the address.
 * 
 * @attention
 * - The `ADDR` flag is cleared by reading `I2C_SR1` and `I2C_SR2` registers as per the 
 *   I2C specification.
 * - This function does not handle errors such as NACK received or bus arbitration loss. 
 *   Error handling must be implemented externally if needed.
 * - Ensure the APB1 clock and I2C timing registers are configured appropriately for 
 *   reliable communication.
 * 
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr) {
    uint8_t dummy;
    pI2CHandle->pI2Cx->I2C_CR1_t.START = 1;

    // Wait until start condition is generated
    while (!(pI2CHandle->pI2Cx->I2C_SR1_t.SB));

    // Add write bit to slave address and send
    SlaveAddr = (SlaveAddr << 1);
    SlaveAddr &= ~(1); // Write operation
    pI2CHandle->pI2Cx->I2C_DR_t.DR = SlaveAddr;

    // Wait for address acknowledgment
    while (!(pI2CHandle->pI2Cx->I2C_SR1_t.ADDR));

    // Clear address flag by reading SR1 and SR2
    dummy = pI2CHandle->pI2Cx->I2C_SR1_t.OVR;
    dummy = pI2CHandle->pI2Cx->I2C_SR2_t.MSL;
    (void)dummy;

    // Transmit data
    while (Len > 0) {
        while (!(pI2CHandle->pI2Cx->I2C_SR1_t.TxE));
        pI2CHandle->pI2Cx->I2C_DR_t.DR = *pTxbuffer;
        pTxbuffer++;
        Len--;
    }

    // Wait until transmission is complete
    while (!(pI2CHandle->pI2Cx->I2C_SR1_t.TxE & pI2CHandle->pI2Cx->I2C_SR1_t.BTF));

    // Generate stop condition
    pI2CHandle->pI2Cx->I2C_CR1_t.STOP = 1;
}


/**
 * @brief  Receives data from an I2C slave in master mode.
 *
 * This function handles the communication to receive a specified length of data from an I2C slave device.
 * The function generates the start condition, addresses the slave, receives data, and generates the stop condition.
 *
 * @param[in]   pI2CHandle  Pointer to the I2C handle structure containing the base address of the I2C peripheral
 *                          and configuration settings.
 * @param[out]  pRxbuffer   Pointer to the buffer where the received data will be stored.
 * @param[in]   Len         Number of bytes to receive.
 * @param[in]   SlaveAddr   Address of the I2C slave device.
 *
 * @note 
 * - This function manages both single-byte and multi-byte reads from the slave.
 * - Ensure that the ACK bit is correctly set or cleared depending on the read length.
 * - Blocking behavior: The function waits for flags to be set/reset during communication.
 *
 * @pre 
 * - The I2C peripheral should be properly initialized before calling this function.
 * - Slave device must be ready to communicate.
 *
 * @post 
 * - After the function completes, the received data will be available in the pRxbuffer.
 * - The STOP condition will be generated to release the I2C bus.
 *
 * @attention 
 * - Re-enabling the ACK bit after communication is critical for subsequent I2C transactions.
 * - Ensure the I2C lines are not stuck and the bus is idle before calling this function.
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr){
    uint8_t dummy;
    
    //1. generate the start condition
    pI2CHandle->pI2Cx->I2C_CR1_t.START=1;

    //2. confirm the that start generation  is complete
    while(!(pI2CHandle->pI2Cx->I2C_SR1_t.SB));

    //3. send the address of the slave
    SlaveAddr=(SlaveAddr<<1);
    SlaveAddr|=1;
    pI2CHandle->pI2Cx->I2C_DR_t.DR=SlaveAddr;

    //4. check the addr flag
    while(!(pI2CHandle->pI2Cx->I2C_SR1_t.ADDR));

    //read
    if(Len == 1){
        //disable acking
        pI2CHandle->pI2Cx->I2C_CR1_t.ACK=0;
        
        //clear the addr flag by reading the sr1 and sr2
        dummy= pI2CHandle->pI2Cx->I2C_SR1_t.ADDR;
        dummy= pI2CHandle->pI2Cx->I2C_SR2_t.BUSY;
        (void)dummy;

        //wait until rxne flag becomes 1
        while(!(pI2CHandle->pI2Cx->I2C_SR1_t.RxNE));

        //generate stop condition
        pI2CHandle->pI2Cx->I2C_CR1_t.STOP=1;

        *pRxbuffer= pI2CHandle->pI2Cx->I2C_DR_t.DR;

        return;
    }

    if(Len>1){
        dummy= pI2CHandle->pI2Cx->I2C_SR1_t.ADDR;
        dummy= pI2CHandle->pI2Cx->I2C_SR2_t.BUSY;
        (void)dummy;
        
        //read until len becomes zero
        while(Len){
            //wait until rxne becomes 1
            while(!(pI2CHandle->pI2Cx->I2C_SR1_t.RxNE));

            if(Len==2){
                //disable acking
                pI2CHandle->pI2Cx->I2C_CR1_t.ACK=0; 

                    //generate stop condition
                    pI2CHandle->pI2Cx->I2C_CR1_t.STOP=1;
            
            }

            pI2CHandle->pI2Cx->I2C_DR_t.DR=*pRxbuffer;
            pRxbuffer++;
            Len--;
        }
    }
    //re-enable acking
    pI2CHandle->pI2Cx->I2C_CR1_t.ACK=1;
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


uint32_t RCC_GetAPB1_CLK(void){
    switch (RCC->RCC_CFGR_t.SWS)
    {
    case 0:
        //HSI used as system clock
        return ((16000000/GET_AHB1_CLK_PRE)/GET_APB1_CLK_PRE);
        break;
    case 1:
        //HSE used as system clock
        break;
    case 2:
        //PLL used as system clock
        break;
    default:
        // NOT ALLOWED
        break;
    }
    return ((16000000/GET_AHB1_CLK_PRE)/GET_APB1_CLK_PRE);// To be deleted when other cases implemented
}
