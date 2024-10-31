/*
 * Stm32f401xx_spi_driver.h
 *
 *  Created on: Oct 30, 2024
 *      Author: mehme
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"


/*
***** Configuration structure for SPI
*/


typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;


/*
***** handle structure for SPI
*/

typedef struct
{
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPI_Config;
}SPI_Handle_t;





#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
