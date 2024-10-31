/*
 * stm32f401xx_gpio.c
 *
 *  Created on: Oct 26, 2024
 *      Author: Mehmethan Türüt
 */




#include "../Inc/stm32f401xx_gpio_driver.h"



/*
                        peripheral    clock setup
*/

/********************************************************************************************
 * @fn      		  - GPIO_PeriClkCtrl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
    if(EnorDi){
        if (pGPIOx==GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx==GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx==GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx==GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx==GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx==GPIOH)
        {
            GPIOH_PCLK_EN();
        }
    }
    else{
        if (pGPIOx==GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx==GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx==GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx==GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx==GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx==GPIOH)
        {
            GPIOH_PCLK_DI();
        }
    }
}


/*
            init and de-init
*/

/********************************************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the chosen GPIO port
 *
 * @param[in]         - pointer to GPIO handle structure that contains the base address of GPIO peripherals and pin config
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
    uint32_t *pTemp;
    //1. configure mode 
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
        //non interrupt mode
        pTemp =(uint32_t*)&(pGPIOHandle->pGPIOx->GPIOx_MODER_t);
        *pTemp &=   ~(3)<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        *pTemp |=   pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

    }
    else{
        uint32_t *pTemp1;
        uint32_t *pTemp2;
        uint32_t *ptemp3;
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
            //1. configure FTSR
            pTemp1 =(uint32_t*) &(EXTI->EXTI_FTSR_t);
            pTemp2 =(uint32_t*) &(EXTI->EXTI_RTSR_t);
            *pTemp2 &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            *pTemp1 |=  (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){
            //1. configure RTSR
            pTemp1 =(uint32_t*) &(EXTI->EXTI_RTSR_t);
            pTemp2 =(uint32_t*) &(EXTI->EXTI_FTSR_t);
            *pTemp2 &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            *pTemp1 |=  (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){
            //1. configure both RTSR and FTSR
            pTemp1 =(uint32_t*) &(EXTI->EXTI_RTSR_t);
            pTemp2 =(uint32_t*) &(EXTI->EXTI_FTSR_t);
            *pTemp2 |=  (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            *pTemp1 |=  (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        }
        //2. configure the GPIO port selection in SYSCFG_EXTICR
        uint32_t *pTemp4;
        pTemp4 =(uint32_t*) (SYSCFG_BASEADDR+ (8+(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4));
        uint8_t  portcode =(uint8_t) GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        *pTemp4 = portcode<<    (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;





        //3. enable the EXTI interrupt delivery using
        ptemp3      =(uint32_t*)    & (EXTI->EXTI_IMR_t);
        *ptemp3     |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

    }

    //2. Configure the speed
    pTemp =(uint32_t*)&(pGPIOHandle->pGPIOx->GPIOx_OSPEEDR_t);
    *pTemp &=   ~(3)<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    *pTemp |=   pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    
    //3. Configure the pull up/down settings

    pTemp =(uint32_t*)&(pGPIOHandle->pGPIOx->GPIOx_PUPDR_t);
    *pTemp &=   ~(3)<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    *pTemp |=   pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    
    //4. Configure the Output type
    pTemp =(uint32_t*)&(pGPIOHandle->pGPIOx->GPIOx_PUPDR_t);
    *pTemp &=   ~(1)<<((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    *pTemp |=   pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    
    //5. configure alternate functionality mode

    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
    {
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber>7)
        {
            pTemp =(uint32_t*)&(pGPIOHandle->pGPIOx->GPIOx_AFRH_t);
            *pTemp &=   ~(0xf)<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
            *pTemp |=   pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

        }
        else{
            pTemp =(uint32_t*)&(pGPIOHandle->pGPIOx->GPIOx_AFRL_t);
            *pTemp &=   ~(0xf)<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
            *pTemp |=   pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        }
    }
    




    
}


/********************************************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function Deinitializes the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
    if (pGPIOx==GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx==GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx==GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx==GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx==GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx==GPIOH)
    {
        GPIOH_REG_RESET();
    }
}
/*
            data read and write
*/

/********************************************************************************************
 * @fn      		  - GPIO_ReadInputPin
 *
 * @brief             - This function reads the data from the given GPIO pin of GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]		  - GPIO pin number
 *
 * @return            -  Data from pin [0 or 1]
 *
 * @Note              -  none

 *********************************************************************************************/
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
    uint32_t *pTemp;
    
    pTemp = (uint32_t*)&(pGPIOx->GPIOx_IDR_t);
    uint8_t value=((*pTemp>>PinNumber)&1);
    return value;
}

/********************************************************************************************
 * @fn      		  - GPIO_ReadInputPort
 *
 * @brief             - This function reads the data from the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]		  - GPIO pin number
 *
 * @return            -  Data from port
 *
 * @Note              -  none

 *********************************************************************************************/

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx){
    uint32_t *pTemp;
    
    pTemp = (uint32_t*)&(pGPIOx->GPIOx_IDR_t);
    uint16_t value=(*pTemp);
    return value;
}

/********************************************************************************************
 * @fn      		  - GPIO_WriteOutputPin
 *
 * @brief             - This function writes the data to the given GPIO pin of GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]		  - GPIO pin number
 * @param[in]		  -	Data
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
    uint32_t *pTemp;
    pTemp =(uint32_t*)&(pGPIOx->GPIOx_ODR_t);
    *pTemp &=   ~(1)<<(PinNumber);
    *pTemp |=   (value)<<(PinNumber);
}

/********************************************************************************************
 * @fn      		  - GPIO_WriteOutputPort
 *
 * @brief             - This function writes the data to the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]		  -	Data
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value){
    uint32_t *pTemp;
    pTemp =(uint32_t*)&(pGPIOx->GPIOx_ODR_t);
    *pTemp=value;
}

/********************************************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the output of the given GPIO pin of GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]		  - GPIO pin number
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
    uint32_t *pTemp;
    pTemp =(uint32_t*)&(pGPIOx->GPIOx_ODR_t);
    *pTemp ^=(1<<PinNumber);
}

/*
            IRQ handling and ISR handling
*/
/********************************************************************************************
 * @fn      		  - GPIO_IRQITConfig
 *
 * @brief             - This function configures the IRQ from the user
 *
 * @param[in]         - IRQ Number
 * @param[in]		  -	ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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




/*
            IRQ priority
*/
/********************************************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - This function configures the IRQ Priority
 *
 * @param[in]         - IRQ Priority
 * @param[in]         - IRQ Number
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/


void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
    //1. IPR register
    uint8_t iprx = IRQNumber/4;
    uint8_t iprx_section = IRQNumber%4;

    *(NVIC_PR_BASEADDR  +  (iprx))  |=   ((IRQPriority<<(8*iprx_section))<<4); //the first 4 bits of each section is non-implemented







}
/********************************************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function handles the IRQ
 *
 * @param[in]         - GPIO pin Number
 *
 * @return            -  none
 *
 * @Note              -  none

 *********************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){
    //clear the EXTI PR register corresponding to the pin number
    uint32_t *pTemp = ((uint32_t*) &EXTI->EXTI_PR_t);
    if(*pTemp & (1<<PinNumber)){
        //clear
        *pTemp  |=  (1<<PinNumber);
    }
}

