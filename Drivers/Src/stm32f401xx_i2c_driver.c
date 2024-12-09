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
#include "../Inc/stm32f401xx.h"

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle );


/**
 * @brief Clears the ADDR flag in the I2C status registers.
 * 
 * This function checks the current mode of the I2C peripheral (master or slave) and clears the
 * ADDR flag by performing dummy reads of the SR1 and SR2 registers. In master mode, additional
 * conditions are handled depending on the transfer state and data size.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration and
 *                       state information for the I2C peripheral.
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_read;
    // Check for device mode
    if (pI2CHandle->pI2Cx->I2C_SR2_t.MSL)
    {
        // Device is in master mode
        if (pI2CHandle->TxRxState == I2C_BSY_IN_RX)
        {
            if (pI2CHandle->RxSize == 1)
            {
                // First disable the ACK
                pI2CHandle->pI2Cx->I2C_CR1_t.ACK = 0;;

                // Clear the ADDR flag (read SR1, then read SR2)
                dummy_read = pI2CHandle->pI2Cx->I2C_SR1_t.ADDR;
                dummy_read = pI2CHandle->pI2Cx->I2C_SR2_t.BUSY;
            }
        }
        else
        {
            // Clear the ADDR flag (read SR1, then read SR2)
            dummy_read = pI2CHandle->pI2Cx->I2C_SR1_t.ADDR;
            dummy_read = pI2CHandle->pI2Cx->I2C_SR2_t.BUSY;
        }
    }
    else
    {
        // Device is in slave mode
        // Clear the ADDR flag (read SR1, then read SR2)
        dummy_read = pI2CHandle->pI2Cx->I2C_SR1_t.ADDR;
        dummy_read = pI2CHandle->pI2Cx->I2C_SR2_t.BUSY;
    }
    (void)dummy_read;
}


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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr) {
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
    I2C_ClearADDRFlag(pI2CHandle);
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
    if (!(Sr))
    {
        pI2CHandle->pI2Cx->I2C_CR1_t.STOP = 1;
    }
    

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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
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
        I2C_ClearADDRFlag(pI2CHandle);

        //wait until rxne flag becomes 1
        while(!(pI2CHandle->pI2Cx->I2C_SR1_t.RxNE));

        //generate stop condition
        if (!(Sr))
        {
            pI2CHandle->pI2Cx->I2C_CR1_t.STOP = 1;
        }

        *pRxbuffer= pI2CHandle->pI2Cx->I2C_DR_t.DR;

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
                    if (!(Sr))
                    {
                        pI2CHandle->pI2Cx->I2C_CR1_t.STOP = 1;
                    }
            
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
 * @brief Sends data from the I2C master in interrupt mode with a specified repeated start condition.
 * 
 * This function initiates the transmission of data from the master device to the slave device
 * in interrupt mode. It allows specifying whether a repeated start condition should be generated.
 * 
 * @param[in]  pI2CHandle  Pointer to the I2C handle structure containing the I2C configuration.
 * @param[in]  pTxbuffer   Pointer to the data buffer that holds the data to be sent.
 * @param[in]  Len         Length of the data to be transmitted.
 * @param[in]  SlaveAddr   Address of the slave device to communicate with.
 * @param[in]  Sr          Repeated start condition control (1: enable, 0: disable).
 * 
 * @return uint8_t         Returns the state of the I2C bus (busy or idle).
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
    uint8_t busystate = pI2CHandle->TxRxState;
    
    if (!(pI2CHandle->TxRxState))
    {
        pI2CHandle->pTxBuffer=pTxbuffer;
        pI2CHandle->TxLen=Len;
        pI2CHandle->TxRxState= I2C_BSY_IN_TX;
        pI2CHandle->DevAddr=  SlaveAddr;
        pI2CHandle->Sr=Sr;

        //generate the start condition
        pI2CHandle->pI2Cx->I2C_CR1_t.START=1;

        //enable ITBUFEN
        pI2CHandle->pI2Cx->I2C_CR2_t.ITBUFEN=1;

        //enable ITEVTEN
        pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN=1;

        //enable ITERREN
        pI2CHandle->pI2Cx->I2C_CR2_t.ITERREN=1;
    }
    return busystate;
}


/**
 * @brief Receives data from the I2C master in interrupt mode with a specified repeated start condition.
 * 
 * This function initiates the reception of data by the master device from a slave device
 * in interrupt mode. It allows specifying whether a repeated start condition should be generated.
 * 
 * @param[in]  pI2CHandle  Pointer to the I2C handle structure containing the I2C configuration.
 * @param[out] pRxbuffer   Pointer to the data buffer to store the received data.
 * @param[in]  Len         Length of the data to be received.
 * @param[in]  SlaveAddr   Address of the slave device to communicate with.
 * @param[in]  Sr          Repeated start condition control (1: enable, 0: disable).
 * 
 * @return uint8_t         Returns the state of the I2C bus (busy or idle).
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
    uint8_t busystate = pI2CHandle->TxRxState;

    if (!(pI2CHandle->TxRxState))
    {
        pI2CHandle->pTxBuffer=pRxbuffer;
        pI2CHandle->RxLen=Len;
        pI2CHandle->TxRxState= I2C_BSY_IN_RX;
        pI2CHandle->DevAddr=  SlaveAddr;
        pI2CHandle->Sr=Sr;

        //generate the start condition
        pI2CHandle->pI2Cx->I2C_CR1_t.START=1;

        //enable ITBUFEN
        pI2CHandle->pI2Cx->I2C_CR2_t.ITBUFEN=1;

        //enable ITEVTEN    
        pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN=1;

        //enable ITERREN
        pI2CHandle->pI2Cx->I2C_CR2_t.ITERREN=1;
    }
    return busystate;
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

/**
 * @brief Handles the I2C event interrupt.
 * 
 * This function is called when an I2C event interrupt occurs. It handles various I2C events,
 * such as start bit (SB), address (ADDR), byte transfer finished (BTF), stop detection (STOPF),
 * transmit buffer empty (TxE), and receive buffer not empty (RxNE). Based on the event and
 * the current state, the function performs appropriate actions.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration
 *                       and state information for the I2C peripheral.
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
    //handle interrupt for SB
    if ((pI2CHandle->pI2Cx->I2C_SR1_t.SB)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN))
    {
        //start generation copmlete. move to address phase
        if (pI2CHandle->TxRxState==I2C_BSY_IN_TX)
        {
            pI2CHandle->pI2Cx->I2C_DR_t.DR= (pI2CHandle->DevAddr<<1);
        }
        else if (pI2CHandle->TxRxState==I2C_BSY_IN_RX)
        {
            pI2CHandle->pI2Cx->I2C_DR_t.DR=((pI2CHandle->DevAddr<<1)|1);
        }
        
    }
    
    //handle interrupt for ADDR
    if ((pI2CHandle->pI2Cx->I2C_SR1_t.ADDR)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN))
    {

        //clear addr flag
        I2C_ClearADDRFlag(pI2CHandle);

    }
    
    //handle interrupt for BTF
    if ((pI2CHandle->pI2Cx->I2C_SR1_t.BTF)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN))
    {
        if (pI2CHandle->TxRxState==I2C_BSY_IN_TX)
        {   
            if(!pI2CHandle->TxLen){
                if (pI2CHandle->pI2Cx->I2C_SR1_t.TxE)
                {
                    //BTF, TXE=1 close transmission

                    //generate stop condition
                    if(!pI2CHandle->Sr){
                        pI2CHandle->pI2Cx->I2C_CR1_t.STOP=1;
                    }
                    //reset handle structure
                    I2C_CloseSendData(pI2CHandle);
                    //notify the application
                    I2C_AppEventCallback(pI2CHandle, I2C_EV_TX_COMP);
                }
            } 
        }else if (pI2CHandle->TxRxState==I2C_BSY_IN_RX)
        {
            ;
        }
        
        
    }
    
    //handle interrupt for stopf
    if ((pI2CHandle->pI2Cx->I2C_SR1_t.STOPF)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN))
    {
        //clear the STOPF flag by reading the sr1 and writing into cr1
        uint8_t dummy1=pI2CHandle->pI2Cx->I2C_SR1_t.ADDR;
        (void)dummy1;
        pI2CHandle->pI2Cx->I2C_CR1_t.ACK|=0;
        I2C_AppEventCallback(pI2CHandle, I2C_EV_STOP);
    }
    

    //handle interrupt for txe
    if ((pI2CHandle->pI2Cx->I2C_SR1_t.TxE)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITBUFEN))
    {
        if(pI2CHandle->pI2Cx->I2C_SR2_t.MSL){
            //master
            if (pI2CHandle->TxRxState==I2C_BSY_IN_TX)
            {
                if(pI2CHandle->TxLen)
                {
                    pI2CHandle->pI2Cx->I2C_DR_t.DR= *(pI2CHandle->pTxBuffer);
                    
                    pI2CHandle->TxLen--;

                    pI2CHandle->pTxBuffer++;
                }
                
            }
        
        }else{
            //slave
            if(pI2CHandle->pI2Cx->I2C_SR2_t.TRA){
                I2C_AppEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }
    //handle interrupt for rxne

    if(pI2CHandle->pI2Cx->I2C_SR2_t.MSL){
        if ((pI2CHandle->pI2Cx->I2C_SR1_t.TxE)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN)&&(pI2CHandle->pI2Cx->I2C_CR2_t.ITBUFEN))
        {
            if (pI2CHandle->TxRxState== I2C_BSY_IN_RX)
            {
                if (pI2CHandle->RxSize==1)
                {
                    *pI2CHandle->pRxBuffer= pI2CHandle->pI2Cx->I2C_DR_t.DR;
                    pI2CHandle->RxLen--;
                }
                
                if (pI2CHandle->RxSize>1)
                {
                    if (pI2CHandle->RxLen==2)
                    {
                        pI2CHandle->pI2Cx->I2C_CR1_t.ACK=0;

                    }
                    *pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->I2C_DR_t.DR;
                    pI2CHandle->pRxBuffer++;
                    pI2CHandle->RxLen--;
                }
                if (!pI2CHandle->RxLen)
                {
                    //close reception notify application

                    //generate stop condition
                    pI2CHandle->pI2Cx->I2C_CR1_t.STOP=1;

                    //close I2C rx

                    I2C_CloseReceiveData(pI2CHandle);

                    //notify application
                    I2C_AppEventCallback(pI2CHandle, I2C_EV_RX_COMP);

                }
                
            }
            
        }
    }else{
        I2C_AppEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
    }
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){


/***********************Check for Bus error************************************/
	if(pI2CHandle->pI2Cx->I2C_CR2_t.ITERREN && pI2CHandle->pI2Cx->I2C_SR1_t.BERR )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1_t.BERR = 0;

		//Implement the code to notify the application about the error
	   I2C_AppEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	if(pI2CHandle->pI2Cx->I2C_CR2_t.ITERREN && pI2CHandle->pI2Cx->I2C_SR1_t.ARLO)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1_t.ARLO = 0;

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	if(pI2CHandle->pI2Cx->I2C_CR2_t.ITERREN && pI2CHandle->pI2Cx->I2C_SR1_t.AF)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1_t.AF = 0;

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	if(pI2CHandle->pI2Cx->I2C_CR2_t.ITERREN && pI2CHandle->pI2Cx->I2C_SR1_t.OVR)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1_t.OVR = 0;

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	if(pI2CHandle->pI2Cx->I2C_CR2_t.ITERREN && pI2CHandle->pI2Cx->I2C_SR1_t.TIMEOUT)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1_t.TIMEOUT = 0;

		//Implement the code to notify the application about the error
		I2C_AppEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}


/**
 * @brief Closes the I2C receive data process.
 * 
 * This function disables the relevant interrupt control bits for buffer and event handling, resets
 * the receive buffer and state variables, and restores the ACK control setting if enabled.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration
 *                       and state information for the I2C peripheral.
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    // Disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2_t.ITBUFEN = 0;

    // Disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN = 0;

    // Reset state and buffer variables
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    // Restore ACK control if enabled
    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        pI2CHandle->pI2Cx->I2C_CR1_t.ACK = 1;
    }
}


/**
 * @brief Closes the I2C send data process.
 * 
 * This function disables the relevant interrupt control bits for buffer and event handling,
 * resets the transmission state and buffer variables, and restores the ACK control setting
 * if enabled.
 * 
 * @param[in] pI2CHandle Pointer to the I2C handle structure that contains the configuration
 *                       and state information for the I2C peripheral.
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    // Disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2_t.ITBUFEN = 0;

    // Disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->I2C_CR2_t.ITEVTEN = 0;

    // Reset state and buffer variables
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    // Restore ACK control if enabled
    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        pI2CHandle->pI2Cx->I2C_CR1_t.ACK = 1;
    }
}


/**
 * @brief  Sends data from the I2C slave to the I2C data register.
 * @param  pI2Cx: Pointer to the I2C register definition structure.
 * @param  data:  Data to be sent.
 * @note   This function directly writes the data to the Data Register (DR).
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data){
    pI2Cx->I2C_DR_t.DR = data;
}

/**
 * @brief  Receives data from the I2C data register in slave mode.
 * @param  pI2Cx: Pointer to the I2C register definition structure.
 * @return uint8_t: Data received from the Data Register (DR).
 * @note   This function directly reads the data from the Data Register (DR).
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
    return (uint8_t)pI2Cx->I2C_DR_t.DR;
}




/**
 * @brief Gets the APB1 peripheral clock frequency.
 * 
 * This function calculates and returns the APB1 clock frequency based on the system clock source
 * and the configured AHB and APB1 prescalers. It currently supports only HSI as the system clock source.
 * 
 * @return uint32_t The APB1 clock frequency in Hz.
 */
uint32_t RCC_GetAPB1_CLK(void)
{
    switch (RCC->RCC_CFGR_t.SWS)
    {
    case 0:
        // HSI used as system clock
        return ((16000000 / GET_AHB1_CLK_PRE) / GET_APB1_CLK_PRE);
        break;
    case 1:
        // HSE used as system clock
        // Implementation pending
        break;
    case 2:
        // PLL used as system clock
        // Implementation pending
        break;
    default:
        // Invalid case
        break;
    }
    // Placeholder return for unimplemented cases
    return ((16000000 / GET_AHB1_CLK_PRE) / GET_APB1_CLK_PRE);
}
