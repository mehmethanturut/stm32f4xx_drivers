/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Oct 26, 2024
 *      Author: mehme
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"



/*
***** Configuration structure for gpio pin
*/

typedef struct 
{
    uint8_t GPIO_PinNumber; 
    uint8_t GPIO_PinMode;           //<values from  @GPIO_PIN_MODES>
    uint8_t GPIO_PinSpeed;          //<values from  @GPIO_PIN_OUTPUT_TYPES>
    uint8_t GPIO_PinPuPdControl;    //<values from  @GPIO_PIN_OUTPUT_SPEEDS>
    uint8_t GPIO_PinOPType;         //<values from  @GPIO_PIN_PUPD>
    uint8_t GPIO_PinAltFunMode;     //<values from  @GPIO_PIN_MODES>
}GPIO_PinConfig_t;


/*
***** handle structure for gpio pin
*/


typedef struct
{
    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*      @GPIO_PIN_NUMBERS
        GPIO possible PIN numbers
*/

#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15


/*      @GPIO_PIN_MODES
        GPIO possible modes
*/

#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG    3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6       


/*      @GPIO_PIN_OUTPUT_TYPES
        GPIO possible output types
*/

#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1


/*      @GPIO_PIN_OUTPUT_SPEEDS
        GPIO possible output speeds
*/

#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MED      1
#define GPIO_SPEED_FASR     3
#define GPIO_SPEED_HIGH     4


/*      @GPIO_PIN_PUPD
        GPIO  pin pull down/up  config 
*/


#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2


/*
******************************************************
*           APIs supported by this driver
*/
/*
            peripheral    clock setup
*/

void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);


/*
            init and de-init
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
            data read nd write
*/
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
            IRQ handling and ISR handling
*/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */