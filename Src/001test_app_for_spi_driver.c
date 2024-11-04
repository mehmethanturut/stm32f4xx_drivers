/*
 * main.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Mehmethan Türüt
 */



#include "stm32f401xx.h"
#include <string.h>


//pB15 spi_mosi
//pB14 spi_miso 
//pB13 spi_sclk
//pB12  spi_nss
//af5

void SPI3_GPIOInits(void);
void SPI3_Inits(void);

int main(void)
{
    char user_data[] = "hello world"; 
    GPIOB_PCLK_EN();
    SPI3_PCLK_EN();

    SPI3_GPIOInits();
    SPI3_Inits();


    SPI_SendData(SPI3, (uint8_t*)user_data, strlen(user_data));




    
    return 0;
}





void SPI3_GPIOInits(void){
    GPIO_Handle_t   SPI_Pins;

    SPI_Pins.pGPIOx = GPIOB;

    SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode=6;
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



void SPI3_Inits(void){
    SPI_Handle_t SPI3Handle;

    SPI3Handle.pSPIx=SPI3;
    SPI3Handle.SPI_Config.SPI_SSM=SPI_SSM_EN;
    SPI3Handle.SPI_Config.SPI_SSI=SPI_SSI_MASTER;
    SPI3Handle.SPI_Config.SPI_BusConfig= SPI_BUS_CONFIG_FD;
    SPI3Handle.SPI_Config.SPI_CPHA=SPI_CPHA_HIGH;
    SPI3Handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
    SPI3Handle.SPI_Config.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
    SPI3Handle.SPI_Config.SPI_SclkSpeed=SPI_SCLK_DIV128;
    SPI3Handle.SPI_Config.SPI_DFF=SPI_DFF_16;

    

    

    SPI_Init(&SPI3Handle);
}

