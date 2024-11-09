/*
 * 002Testapp_SPI_Sl_arduino.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Mehmethan Türüt
 */



#include "stm32f401xx.h"
#include <string.h>


//pB5 spi_mosi
//pB4 spi_miso
//pB3 spi_sclk
//pa4  spi_nss 
//af6

void delay(uint32_t iteration);
void SPI3_GPIOInits(void);
void SPI3_Inits(void);
void button_init(void);

int main(void)
{
    char user_data[] = "hello world";

    button_init();
    SPI3_GPIOInits();
    SPI3_Inits();

    SPI3_SSOE_HIGH();

    while(1){
        while (GPIO_ReadInputPin(GPIOC,GPIO_PIN_NO_13));

        SPI3_ENABLE();


        uint8_t data_len = strlen(user_data);
        SPI_SendData(SPI3,&data_len, 1);

        SPI_SendData(SPI3, (uint8_t*)user_data, strlen(user_data));   
        delay(400);  
        while(SPI3_BUSY);
        SPI3_DISABLE();   
    } 




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

    SPI_Pins.pGPIOx= GPIOA;
    //nss
    SPI_Pins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_4;
    GPIO_Init(&SPI_Pins);

}



void SPI3_Inits(void){
    SPI_Handle_t SPI3Handle;

    SPI3Handle.pSPIx=SPI3;
    SPI3Handle.SPI_Config.SPI_SSM=SPI_SSM_DI;
    SPI3Handle.SPI_Config.SPI_BusConfig= SPI_BUS_CONFIG_FD;
    SPI3Handle.SPI_Config.SPI_CPHA=SPI_CPHA_LOW;
    SPI3Handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
    SPI3Handle.SPI_Config.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
    SPI3Handle.SPI_Config.SPI_SclkSpeed=SPI_SCLK_DIV8;
    SPI3Handle.SPI_Config.SPI_DFF=SPI_DFF_8;



    SPI_Init(&SPI3Handle);
}


void button_init(void){

    GPIO_Handle_t GPIOButton;

    GPIOButton.pGPIOx= GPIOC;

    GPIOButton.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_IN;
    GPIOButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
    GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;


    GPIO_Init(&GPIOButton);


}

void delay(uint32_t iteration){

    uint32_t temp_count=3200*(iteration);
    while (temp_count--);
}


