/*
 * main.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Mehmethan Türüt
 */



#include "stm32f401xx.h"
#include <string.h>

//pA6 spi_miso 
//pB5 spi_mosi
//pB3 spi_sclk
//pA4  spi_nss
//af5

void SPI1_GPIOInits(void);
void SPI1_Inits(void);

int main(void)
{
    char user_data[] = "hello world"; 
    GPIOB_PCLK_EN();
    SPI1_PCLK_EN();

    SPI1_GPIOInits();
    SPI1_Inits();


    SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));




    
    return 0;
}





void SPI1_GPIOInits(void){
    GPIO_Handle_t   SPI_Pins;

    SPI_Pins.pGPIOx = GPIOB;

    SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
    SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //mosi
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
    GPIO_Init(&SPI_Pins);

    //miso
    //SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
    //GPIO_Init(&SPI_Pins);

    //clk
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_3;
    GPIO_Init(&SPI_Pins);

    //nss
    //SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
    //GPIO_Init(&SPI_Pins);

}



void SPI1_Inits(void){
    SPI_Handle_t SPI1Handle;

    SPI1Handle.pSPIx=SPI1;
    SPI1Handle.SPI_Config.SPI_SSM=SPI_SSM_EN;
    SPI1Handle.SPI_Config.SPI_SSI=SPI_SSI_MASTER;
    SPI1Handle.SPI_Config.SPI_BusConfig= SPI_BUS_CONFIG_FD;
    SPI1Handle.SPI_Config.SPI_CPHA=SPI_CPHA_LOW;
    SPI1Handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
    SPI1Handle.SPI_Config.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
    SPI1Handle.SPI_Config.SPI_SclkSpeed=SPI_SCLK_DIV2;
    SPI1Handle.SPI_Config.SPI_DFF=SPI_DFF_8;

    

    

    SPI_Init(&SPI1Handle);
}

