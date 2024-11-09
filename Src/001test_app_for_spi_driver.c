/*
 * 001test_app_for_spi.c
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

void SPI2_GPIOInits(void);
void SPI2_Inits(void);

int main(void)
{
    char user_data[] = "hello world"; 
    GPIOB_PCLK_EN();
    SPI2_PCLK_EN();

    SPI2_GPIOInits();
    SPI2_Inits();


    SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));




    
    return 0;
}





void SPI2_GPIOInits(void){
    GPIO_Handle_t   SPI_Pins;

    SPI_Pins.pGPIOx = GPIOB;

    SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
    SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    //mosi
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_15;
    GPIO_Init(&SPI_Pins);

    //miso
    //SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
    //GPIO_Init(&SPI_Pins);

    //clk
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
    GPIO_Init(&SPI_Pins);

    //nss
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
    GPIO_Init(&SPI_Pins);

}



void SPI2_Inits(void){
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx=SPI2;
    SPI2Handle.SPI_Config.SPI_SSM=SPI_SSM_EN;
    SPI2Handle.SPI_Config.SPI_SSI=SPI_SSI_MASTER;
    SPI2Handle.SPI_Config.SPI_BusConfig= SPI_BUS_CONFIG_FD;
    SPI2Handle.SPI_Config.SPI_CPHA=SPI_CPHA_LOW;
    SPI2Handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
    SPI2Handle.SPI_Config.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
    SPI2Handle.SPI_Config.SPI_SclkSpeed=SPI_SCLK_DIV2;
    SPI2Handle.SPI_Config.SPI_DFF=SPI_DFF_8;

    

    

    SPI_Init(&SPI2Handle);
}

