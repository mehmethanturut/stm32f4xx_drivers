/**
 * @file stm32f401xx.h
 * @brief Header file defining hardware registers and base addresses for STM32F401xx microcontroller.
 * 
 * This file provides definitions and macros for base addresses, peripheral structures,
 * and clock configurations for STM32F401xx devices.
 * 
 * Created on: Oct 25, 2024
 * Author: Mehmethan Türüt
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

/*
*****************************processor specific details**********************
*/

/** 
 * @defgroup NVIC_ISERx ARM Cortex Mx NVIC ISERx register addresses
 * @{
 */
#define NVIC_ISER0      ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1      ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2      ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3      ((volatile uint32_t*)0xE000E10C)
/** @} */

/** 
 * @defgroup NVIC_ICERx ARM Cortex Mx NVIC ICERx register addresses
 * @{
 */
#define NVIC_ICER0      ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1      ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2      ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3      ((volatile uint32_t*)0xE000E18C)
/** @} */

/**
 * @def NVIC_PR_BASEADDR 
 * @brief ARM Cortex Mx Priority register base address.
 */
#define NVIC_PR_BASEADDR        ((volatile uint32_t*)0xE000E400)

/** 
 * @defgroup MEMORY_BASEADDR Base addresses of Flash, SRAM, and ROM
 * @{
 */
#define FLASH_BASEADDR                 0x08000000U /**< Base address of Flash memory */
#define SRAM_BASEADDR                  0x20000000U /**< Base address of SRAM memory */
#define ROM_BASEADDR                   0x1FFF0000U /**< Base address of ROM memory */
/** @} */

/**
 * @defgroup BUS_BASEADDR Base addresses for APB and AHB buses
 * @{
 */
#define PERIPH_BASEADDR                0x40000000U /**< Base address of peripheral memory */
#define APB1PERIPH_BASEADDR            PERIPH_BASEADDR /**< Base address of APB1 peripheral memory */
#define APB2PERIPH_BASEADDR            0x40010000U /**< Base address of APB2 peripheral memory */
#define AHB1PERIPH_BASEADDR            0x40020000U /**< Base address of AHB1 peripheral memory */
#define AHB2PERIPH_BASEADDR            0x50000000U /**< Base address of AHB2 peripheral memory */
/** @} */

/**
 * @defgroup INDIV_PERIPH_BASEADDR Individual peripheral base addresses
 * @{
 */
#define TIM2_BASEADDR                  APB1PERIPH_BASEADDR /**< Base address of TIM2 */
#define TIM3_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X400) /**< Base address of TIM3 */
#define TIM4_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X800) /**< Base address of TIM4 */
#define TIM5_BASEADDR                  (APB1PERIPH_BASEADDR+ 0XC00) /**< Base address of TIM5 */
#define RTC_BKP_BASEADDR               (APB1PERIPH_BASEADDR+ 0X2800) /**< Base address of RTC and backup registers */
#define WWDG_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X2C00) /**< Base address of window watchdog */
#define IWDG_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X3000) /**< Base address of independent watchdog */
#define I2S2ext_BASEADDR               (APB1PERIPH_BASEADDR+ 0X3400) /**< Base address of I2S2ext */
#define SPI2_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X3800) /**< Base address of SPI2/I2S2 */
#define I2S2_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X3800) /**< Base address of SPI2/I2S2 */
#define SPI3_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X3C00) /**< Base address of SPI3/I2S3 */
#define I2S3_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X3C00) /**< Base address of SPI3/I2S3 */
#define I2S3ext_BASEADDR               (APB1PERIPH_BASEADDR+ 0X4000) /**< Base address of I2S3ext */
#define USART2_BASEADDR                (APB1PERIPH_BASEADDR+ 0X4400) /**< Base address of USART2 */
#define I2C1_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X5400) /**< Base address of I2C1 */
#define I2C2_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X5800) /**< Base address of I2C2 */
#define I2C3_BASEADDR                  (APB1PERIPH_BASEADDR+ 0X5C00) /**< Base address of I2C3 */
#define PWR_BASEADDR                   (APB1PERIPH_BASEADDR+ 0X7000) /**< Base address of power control */

#define TIM1_BASEADDR                  (APB2PERIPH_BASEADDR) /**< Base address of TIM1 */
#define USART1_BASEADDR                (APB2PERIPH_BASEADDR+ 0X1000) /**< Base address of USART1 */
#define USART6_BASEADDR                (APB2PERIPH_BASEADDR+ 0X1400) /**< Base address of USART6 */
#define ADC_BASEADDR                   (APB2PERIPH_BASEADDR+ 0X2000) /**< Base address of ADC */
#define SDIO_BASEADDR                  (APB2PERIPH_BASEADDR+ 0X2C00) /**< Base address of SDIO */
#define SPI1_BASEADDR                  (APB2PERIPH_BASEADDR+ 0X3000) /**< Base address of SPI1 */
#define SPI4_BASEADDR                  (APB2PERIPH_BASEADDR+ 0X3400) /**< Base address of SPI4 */
#define SYSCFG_BASEADDR                (APB2PERIPH_BASEADDR+ 0X3800) /**< Base address of SYSCFG */
#define EXTI_BASEADDR                  (APB2PERIPH_BASEADDR+ 0X3C00) /**< Base address of EXTI */
#define TIM9_BASEADDR                  (APB2PERIPH_BASEADDR+ 0X4000) /**< Base address of TIM9 */
#define TIM10_BASEADDR                 (APB2PERIPH_BASEADDR+ 0X4400) /**< Base address of TIM10 */
#define TIM11_BASEADDR                 (APB2PERIPH_BASEADDR+ 0X4800) /**< Base address of TIM11 */

#define GPIOA_BASEADDR                 (AHB1PERIPH_BASEADDR) /**< Base address of GPIOA */
#define GPIOB_BASEADDR                 (AHB1PERIPH_BASEADDR+0X400) /**< Base address of GPIOB */
#define GPIOC_BASEADDR                 (AHB1PERIPH_BASEADDR+0X800) /**< Base address of GPIOC */
#define GPIOD_BASEADDR                 (AHB1PERIPH_BASEADDR+0XC00) /**< Base address of GPIOD */
#define GPIOE_BASEADDR                 (AHB1PERIPH_BASEADDR+0X1000) /**< Base address of GPIOE */
#define GPIOH_BASEADDR                 (AHB1PERIPH_BASEADDR+0X1C00) /**< Base address of GPIOH */
#define CRC_BASEADDR                   (AHB1PERIPH_BASEADDR+0X3000) /**< Base address of CRC module */
#define RCC_BASEADDR                   (AHB1PERIPH_BASEADDR+0X3800) /**< Base address of RCC */
#define FLASH_INTERFACE_BASEADDR       (AHB1PERIPH_BASEADDR+0X3C00) /**< Base address of flash interface */
#define DMA1_BASEADDR                  (AHB1PERIPH_BASEADDR+0X6000) /**< Base address of DMA1 */
#define DMA2_BASEADDR                  (AHB1PERIPH_BASEADDR+0X6400) /**< Base address of DMA2 */

#define USB_OTG_FS_BASEADDR            (AHB2PERIPH_BASEADDR) /**< Base address of USB OTG FS */

/* 
******************GPIO register definition structures******************
*/


/** 
 * @brief Structure for GPIO port mode register.
 */
volatile typedef struct 
{
    struct{         //GPIO port mode register
        uint32_t MODER0       :2; /**< Mode for pin 0 */
        uint32_t MODER1       :2; /**< Mode for pin 1 */
        uint32_t MODER2       :2; /**< Mode for pin 2 */
        uint32_t MODER3       :2; /**< Mode for pin 3 */
        uint32_t MODER4       :2; /**< Mode for pin 4 */
        uint32_t MODER5       :2; /**< Mode for pin 5 */
        uint32_t MODER6       :2; /**< Mode for pin 6 */
        uint32_t MODER7       :2; /**< Mode for pin 7 */
        uint32_t MODER8       :2; /**< Mode for pin 8 */
        uint32_t MODER9       :2; /**< Mode for pin 9 */
        uint32_t MODER10      :2; /**< Mode for pin 10 */
        uint32_t MODER11      :2; /**< Mode for pin 11 */
        uint32_t MODER12      :2; /**< Mode for pin 12 */
        uint32_t MODER13      :2; /**< Mode for pin 13 */
        uint32_t MODER14      :2; /**< Mode for pin 14 */
        uint32_t MODER15      :2; /**< Mode for pin 15 */
    } GPIOx_MODER_t;

    struct{         //GPIO port output type register
        uint32_t OT0       :1; /**< Output type for pin 0 */
        uint32_t OT1       :1; /**< Output type for pin 1 */
        uint32_t OT2       :1; /**< Output type for pin 2 */
        uint32_t OT3       :1; /**< Output type for pin 3 */
        uint32_t OT4       :1; /**< Output type for pin 4 */
        uint32_t OT5       :1; /**< Output type for pin 5 */
        uint32_t OT6       :1; /**< Output type for pin 6 */
        uint32_t OT7       :1; /**< Output type for pin 7 */
        uint32_t OT8       :1; /**< Output type for pin 8 */
        uint32_t OT9       :1; /**< Output type for pin 9 */
        uint32_t OT10      :1; /**< Output type for pin 10 */
        uint32_t OT11      :1; /**< Output type for pin 11 */
        uint32_t OT12      :1; /**< Output type for pin 12 */
        uint32_t OT13      :1; /**< Output type for pin 13 */
        uint32_t OT14      :1; /**< Output type for pin 14 */
        uint32_t OT15      :1; /**< Output type for pin 15 */
        uint32_t reserved  :16; /**< Reserved bits */
    } GPIOx_OTYPER_t;

    struct{         //GPIO port output speed register
        uint32_t OSPEEDR0       :2; /**< Speed for pin 0 */
        uint32_t OSPEEDR1       :2; /**< Speed for pin 1 */
        uint32_t OSPEEDR2       :2; /**< Speed for pin 2 */
        uint32_t OSPEEDR3       :2; /**< Speed for pin 3 */
        uint32_t OSPEEDR4       :2; /**< Speed for pin 4 */
        uint32_t OSPEEDR5       :2; /**< Speed for pin 5 */
        uint32_t OSPEEDR6       :2; /**< Speed for pin 6 */
        uint32_t OSPEEDR7       :2; /**< Speed for pin 7 */
        uint32_t OSPEEDR8       :2; /**< Speed for pin 8 */
        uint32_t OSPEEDR9       :2; /**< Speed for pin 9 */
        uint32_t OSPEEDR10      :2; /**< Speed for pin 10 */
        uint32_t OSPEEDR11      :2; /**< Speed for pin 11 */
        uint32_t OSPEEDR12      :2; /**< Speed for pin 12 */
        uint32_t OSPEEDR13      :2; /**< Speed for pin 13 */
        uint32_t OSPEEDR14      :2; /**< Speed for pin 14 */
        uint32_t OSPEEDR15      :2; /**< Speed for pin 15 */
    } GPIOx_OSPEEDR_t;

    struct{         //GPIO port pull-up/pull-down register
        uint32_t PUPDR0       :2; /**< Pull-up/pull-down for pin 0 */
        uint32_t PUPDR1       :2; /**< Pull-up/pull-down for pin 1 */
        uint32_t PUPDR2       :2; /**< Pull-up/pull-down for pin 2 */
        uint32_t PUPDR3       :2; /**< Pull-up/pull-down for pin 3 */
        uint32_t PUPDR4       :2; /**< Pull-up/pull-down for pin 4 */
        uint32_t PUPDR5       :2; /**< Pull-up/pull-down for pin 5 */
        uint32_t PUPDR6       :2; /**< Pull-up/pull-down for pin 6 */
        uint32_t PUPDR7       :2; /**< Pull-up/pull-down for pin 7 */
        uint32_t PUPDR8       :2; /**< Pull-up/pull-down for pin 8 */
        uint32_t PUPDR9       :2; /**< Pull-up/pull-down for pin 9 */
        uint32_t PUPDR10      :2; /**< Pull-up/pull-down for pin 10 */
        uint32_t PUPDR11      :2; /**< Pull-up/pull-down for pin 11 */
        uint32_t PUPDR12      :2; /**< Pull-up/pull-down for pin 12 */
        uint32_t PUPDR13      :2; /**< Pull-up/pull-down for pin 13 */
        uint32_t PUPDR14      :2; /**< Pull-up/pull-down for pin 14 */
        uint32_t PUPDR15      :2; /**< Pull-up/pull-down for pin 15 */
    } GPIOx_PUPDR_t;

    const  struct{         //GPIO port input data register
        uint32_t IDR0       :1; /**< Input data for pin 0 */
        uint32_t IDR1       :1; /**< Input data for pin 1 */
        uint32_t IDR2       :1; /**< Input data for pin 2 */
        uint32_t IDR3       :1; /**< Input data for pin 3 */
        uint32_t IDR4       :1; /**< Input data for pin 4 */
        uint32_t IDR5       :1; /**< Input data for pin 5 */
        uint32_t IDR6       :1; /**< Input data for pin 6 */
        uint32_t IDR7       :1; /**< Input data for pin 7 */
        uint32_t IDR8       :1; /**< Input data for pin 8 */
        uint32_t IDR9       :1; /**< Input data for pin 9 */
        uint32_t IDR10      :1; /**< Input data for pin 10 */
        uint32_t IDR11      :1; /**< Input data for pin 11 */
        uint32_t IDR12      :1; /**< Input data for pin 12 */
        uint32_t IDR13      :1; /**< Input data for pin 13 */
        uint32_t IDR14      :1; /**< Input data for pin 14 */
        uint32_t IDR15      :1; /**< Input data for pin 15 */
        uint32_t reserved   :16; /**< Reserved bits */
    } GPIOx_IDR_t;

    struct{         //GPIO port output data register
        uint32_t ODR0       :1; /**< Output data for pin 0 */
        uint32_t ODR1       :1; /**< Output data for pin 1 */
        uint32_t ODR2       :1; /**< Output data for pin 2 */
        uint32_t ODR3       :1; /**< Output data for pin 3 */
        uint32_t ODR4       :1; /**< Output data for pin 4 */
        uint32_t ODR5       :1; /**< Output data for pin 5 */
        uint32_t ODR6       :1; /**< Output data for pin 6 */
        uint32_t ODR7       :1; /**< Output data for pin 7 */
        uint32_t ODR8       :1; /**< Output data for pin 8 */
        uint32_t ODR9       :1; /**< Output data for pin 9 */
        uint32_t ODR10      :1; /**< Output data for pin 10 */
        uint32_t ODR11      :1; /**< Output data for pin 11 */
        uint32_t ODR12      :1; /**< Output data for pin 12 */
        uint32_t ODR13      :1; /**< Output data for pin 13 */
        uint32_t ODR14      :1; /**< Output data for pin 14 */
        uint32_t ODR15      :1; /**< Output data for pin 15 */
        uint32_t reserved   :16; /**< Reserved bits */
    } GPIOx_ODR_t;




    struct{         //GPIO port bit set/reset register
        uint32_t BS0       :1; /**< Bit set for pin 0 */
        uint32_t BS1       :1; /**< Bit set for pin 1 */
        uint32_t BS2       :1; /**< Bit set for pin 2 */
        uint32_t BS3       :1; /**< Bit set for pin 3 */
        uint32_t BS4       :1; /**< Bit set for pin 4 */
        uint32_t BS5       :1; /**< Bit set for pin 5 */
        uint32_t BS6       :1; /**< Bit set for pin 6 */
        uint32_t BS7       :1; /**< Bit set for pin 7 */
        uint32_t BS8       :1; /**< Bit set for pin 8 */
        uint32_t BS9       :1; /**< Bit set for pin 9 */
        uint32_t BS10      :1; /**< Bit set for pin 10 */
        uint32_t BS11      :1; /**< Bit set for pin 11 */
        uint32_t BS12      :1; /**< Bit set for pin 12 */
        uint32_t BS13      :1; /**< Bit set for pin 13 */
        uint32_t BS14      :1; /**< Bit set for pin 14 */
        uint32_t BS15      :1; /**< Bit set for pin 15 */
        uint32_t BR0       :1; /**< Bit reset for pin 0 */
        uint32_t BR1       :1; /**< Bit reset for pin 1 */
        uint32_t BR2       :1; /**< Bit reset for pin 2 */
        uint32_t BR3       :1; /**< Bit reset for pin 3 */
        uint32_t BR4       :1; /**< Bit reset for pin 4 */
        uint32_t BR5       :1; /**< Bit reset for pin 5 */
        uint32_t BR6       :1; /**< Bit reset for pin 6 */
        uint32_t BR7       :1; /**< Bit reset for pin 7 */
        uint32_t BR8       :1; /**< Bit reset for pin 8 */
        uint32_t BR9       :1; /**< Bit reset for pin 9 */
        uint32_t BR10      :1; /**< Bit reset for pin 10 */
        uint32_t BR11      :1; /**< Bit reset for pin 11 */
        uint32_t BR12      :1; /**< Bit reset for pin 12 */
        uint32_t BR13      :1; /**< Bit reset for pin 13 */
        uint32_t BR14      :1; /**< Bit reset for pin 14 */
        uint32_t BR15      :1; /**< Bit reset for pin 15 */
    } GPIOx_BSRR_t;

    struct{         //GPIO port configuration lock register
        uint32_t LCK0       :1; /**< Lock configuration for pin 0 */
        uint32_t LCK1       :1; /**< Lock configuration for pin 1 */
        uint32_t LCK2       :1; /**< Lock configuration for pin 2 */
        uint32_t LCK3       :1; /**< Lock configuration for pin 3 */
        uint32_t LCK4       :1; /**< Lock configuration for pin 4 */
        uint32_t LCK5       :1; /**< Lock configuration for pin 5 */
        uint32_t LCK6       :1; /**< Lock configuration for pin 6 */
        uint32_t LCK7       :1; /**< Lock configuration for pin 7 */
        uint32_t LCK8       :1; /**< Lock configuration for pin 8 */
        uint32_t LCK9       :1; /**< Lock configuration for pin 9 */
        uint32_t LCK10      :1; /**< Lock configuration for pin 10 */
        uint32_t LCK11      :1; /**< Lock configuration for pin 11 */
        uint32_t LCK12      :1; /**< Lock configuration for pin 12 */
        uint32_t LCK13      :1; /**< Lock configuration for pin 13 */
        uint32_t LCK14      :1; /**< Lock configuration for pin 14 */
        uint32_t LCK15      :1; /**< Lock configuration for pin 15 */
        uint32_t LCKK       :1; /**< Lock key bit */
        uint32_t res   :16; /**< Reserved bits */
    } GPIOx_LCKR_t;

    struct{         //GPIO alternate function low register
        uint32_t ARFL0     :4; /**< Alternate function low for pin 0 */
        uint32_t ARFL1     :4; /**< Alternate function low for pin 1 */
        uint32_t ARFL2     :4; /**< Alternate function low for pin 2 */
        uint32_t ARFL3     :4; /**< Alternate function low for pin 3 */
        uint32_t ARFL4     :4; /**< Alternate function low for pin 4 */
        uint32_t ARFL5     :4; /**< Alternate function low for pin 5 */
        uint32_t ARFL6     :4; /**< Alternate function low for pin 6 */
        uint32_t ARFL7     :4; /**< Alternate function low for pin 7 */
    } GPIOx_AFRL_t;

    struct{         //GPIO alternate function high register
        uint32_t ARFH8      :4; /**< Alternate function high for pin 8 */
        uint32_t ARFH9      :4; /**< Alternate function high for pin 9 */
        uint32_t ARFH10     :4; /**< Alternate function high for pin 10 */
        uint32_t ARFH11     :4; /**< Alternate function high for pin 11 */
        uint32_t ARFH12     :4; /**< Alternate function high for pin 12 */
        uint32_t ARFH13     :4; /**< Alternate function high for pin 13 */
        uint32_t ARFH14     :4; /**< Alternate function high for pin 14 */
        uint32_t ARFH15     :4; /**< Alternate function high for pin 15 */
    } GPIOx_AFRH_t;
}GPIO_RegDef_t;





/* 
******************RCC register definition structures******************
*/
typedef struct
{
    struct{         //RCC clock control register
        uint32_t HSION           :1; /**< High-speed internal clock enable */
        uint32_t HSIRDY          :1; /**< HSI clock ready flag */
        uint32_t res             :1; /**< Reserved */
        uint32_t HSITRIM         :5; /**< HSI clock trimming */
        uint32_t HSICAL          :8; /**< HSI clock calibration */
        uint32_t HSEON           :1; /**< High-speed external clock enable */
        uint32_t HSERDY          :1; /**< HSE clock ready flag */
        uint32_t HSEBYP          :1; /**< HSE clock bypass */
        uint32_t CSSON           :1; /**< Clock security system enable */
        uint32_t res1            :4; /**< Reserved */
        uint32_t PLLON           :1; /**< PLL enable */
        uint32_t PLLRDY          :1; /**< PLL ready flag */
        uint32_t PLLI2S_ON       :1; /**< PLLI2S enable */
        uint32_t PLLI2S_RDY      :1; /**< PLLI2S ready flag */
        uint32_t res2            :4; /**< Reserved */
    } RCC_CR_t;

    struct{         //RCC PLL configuration register
        uint32_t PLLM0            :1; /**< PLL input clock divider bit 0 */
        uint32_t PLLM1            :1; /**< PLL input clock divider bit 1 */
        uint32_t PLLM2            :1; /**< PLL input clock divider bit 2 */
        uint32_t PLLM3            :1; /**< PLL input clock divider bit 3 */
        uint32_t PLLM4            :1; /**< PLL input clock divider bit 4 */
        uint32_t PLLM5            :1; /**< PLL input clock divider bit 5 */
        uint32_t PLLN             :9; /**< PLL multiplication factor */
        uint32_t res              :1; /**< Reserved */
        uint32_t PLLP0            :1; /**< PLL output clock divider bit 0 */
        uint32_t PLLP1            :1; /**< PLL output clock divider bit 1 */
        uint32_t res1             :4; /**< Reserved */
        uint32_t PLLSRC           :1; /**< PLL source */
        uint32_t res2             :1; /**< Reserved */
        uint32_t PLLQ0            :1; /**< PLL output clock division factor for USB, SDIO, and RNG bit 0 */
        uint32_t PLLQ1            :1; /**< PLL output clock division factor for USB, SDIO, and RNG bit 1 */
        uint32_t PLLQ2            :1; /**< PLL output clock division factor for USB, SDIO, and RNG bit 2 */
        uint32_t PLLQ3            :1; /**< PLL output clock division factor for USB, SDIO, and RNG bit 3 */
        uint32_t res3             :4; /**< Reserved */
    } RCC_PLLCFGR_t;

    struct{         //RCC clock configuration register
        uint32_t SW0              :1; /**< System clock switch bit 0 */
        uint32_t SW1              :1; /**< System clock switch bit 1 */
        uint32_t SWS0             :1; /**< System clock switch status bit 0 */
        uint32_t SWS1             :1; /**< System clock switch status bit 1 */
        uint32_t HPRE             :4; /**< AHB prescaler */
        uint32_t res              :2; /**< Reserved */
        uint32_t PPRE1            :3; /**< APB low-speed prescaler (APB1) */
        uint32_t PPRE2            :3; /**< APB high-speed prescaler (APB2) */
        uint32_t RTCPRE           :5; /**< HSE division factor for RTC clock */
        uint32_t MCO1             :2; /**< Microcontroller clock output 1 */
        uint32_t I2SSCR           :1; /**< I2S clock selection */
        uint32_t MCO1PRE          :3; /**< MCO1 prescaler */
        uint32_t MCO2PRE          :3; /**< MCO2 prescaler */
        uint32_t MCO2             :2; /**< Microcontroller clock output 2 */
    } RCC_CFGR_t;

    struct{         //RCC clock interrupt register
        uint32_t LSIRDYF         :1; /**< LSI ready interrupt flag */
        uint32_t LSERDYF         :1; /**< LSE ready interrupt flag */
        uint32_t HSIRDYF         :1; /**< HSI ready interrupt flag */
        uint32_t HSERDYF         :1; /**< HSE ready interrupt flag */
        uint32_t PLLRDYF         :1; /**< PLL ready interrupt flag */
        uint32_t PLLI2SRDYF      :1; /**< PLLI2S ready interrupt flag */
        uint32_t res             :1; /**< Reserved */
        uint32_t CSSF            :1; /**< Clock security system interrupt flag */
        uint32_t LSIRDYIE        :1; /**< LSI ready interrupt enable */
        uint32_t LSERDYIE        :1; /**< LSE ready interrupt enable */
        uint32_t HSIRDYIE        :1; /**< HSI ready interrupt enable */
        uint32_t HSERDYIE        :1; /**< HSE ready interrupt enable */
        uint32_t PLLRDYIE        :1; /**< PLL ready interrupt enable */
        uint32_t PLLI2SRDYIE     :1; /**< PLLI2S ready interrupt enable */
        uint32_t res1            :2; /**< Reserved */
        uint32_t LSIRDYC         :1; /**< LSI ready clear flag */
        uint32_t LSERDYC         :1; /**< LSE ready clear flag */
        uint32_t HSIRDYC         :1; /**< HSI ready clear flag */
        uint32_t HSERDYC         :1; /**< HSE ready clear flag */
        uint32_t PLLRDYC         :1; /**< PLL ready clear flag */
        uint32_t PLLI2SRDYC      :1; /**< PLLI2S ready clear flag */
        uint32_t res2            :1; /**< Reserved */
        uint32_t CSSC            :1; /**< Clock security system interrupt clear */
        uint32_t res3            :8; /**< Reserved */
    } RCC_CIR_t;

    struct{         //RCC AHB1 peripheral reset register
        uint32_t GPIOARST        :1; /**< Reset GPIOA */
        uint32_t GPIOBRST        :1; /**< Reset GPIOB */
        uint32_t GPIOCRST        :1; /**< Reset GPIOC */
        uint32_t GPIODRST        :1; /**< Reset GPIOD */
        uint32_t GPIOERST        :1; /**< Reset GPIOE */
        uint32_t res             :2; /**< Reserved */
        uint32_t GPIOHRST        :1; /**< Reset GPIOH */
        uint32_t res1            :4; /**< Reserved */
        uint32_t CRCR            :1; /**< Reset CRC module */
        uint32_t res2            :8; /**< Reserved */
        uint32_t DMA1RST         :1; /**< Reset DMA1 */
        uint32_t DMA2RST         :1; /**< Reset DMA2 */
        uint32_t res3            :9; /**< Reserved */
    } RCC_AHB1RSTR_t;



    struct 
    {
        uint32_t res;
    }RESERVED0_t;

    struct 
    {
        uint32_t res;
    }RESERVED1_t;




    struct{         //RCC AHB2 peripheral reset register
        uint32_t res              :7; /**< Reserved */
        uint32_t OTGFSRST        :1; /**< Reset OTGFS */
        uint32_t res1             :24; /**< Reserved */
    } RCC_AHB2RSTR_t;



       struct{                 //RCC APB1 peripheral reset register
        uint32_t TIM2RST         :1; /**< Reset TIM2 */
        uint32_t TIM3RST         :1; /**< Reset TIM3 */
        uint32_t TIM4RST         :1; /**< Reset TIM4 */
        uint32_t TIM5RST         :1; /**< Reset TIM5 */
        uint32_t res             :7; /**< Reserved */
        uint32_t WWDGRST         :1; /**< Reset WWDG */
        uint32_t res1            :2; /**< Reserved */
        uint32_t SPI2RST         :1; /**< Reset SPI2 */
        uint32_t SPI3RST         :1; /**< Reset SPI3 */
        uint32_t res2            :1; /**< Reserved */
        uint32_t USART2RST       :1; /**< Reset USART2 */
        uint32_t res3            :3; /**< Reserved */
        uint32_t I2C1RST         :1; /**< Reset I2C1 */
        uint32_t I2C2RST         :1; /**< Reset I2C2 */
        uint32_t I2C3RST         :1; /**< Reset I2C3 */
        uint32_t res4            :4; /**< Reserved */
        uint32_t PWRRST          :1; /**< Reset PWR */
        uint32_t res5            :3; /**< Reserved */
    } RCC_APB1RSTR_t;

    struct{                 //RCC APB2 peripheral reset register
        uint32_t TIM1RST         :1; /**< Reset TIM1 */
        uint32_t res             :3; /**< Reserved */
        uint32_t USART1RST       :1; /**< Reset USART1 */
        uint32_t USART6RST       :1; /**< Reset USART6 */
        uint32_t res1            :2; /**< Reserved */
        uint32_t ADC1RST         :1; /**< Reset ADC1 */
        uint32_t res2            :2; /**< Reserved */
        uint32_t SDIORST         :1; /**< Reset SDIO */
        uint32_t SPI1RST         :1; /**< Reset SPI1 */
        uint32_t SPI4RST         :1; /**< Reset SPI4 */
        uint32_t SYSCFGRST       :1; /**< Reset SYSCFG */
        uint32_t res3            :1; /**< Reserved */
        uint32_t TIM9RST         :1; /**< Reset TIM9 */
        uint32_t TIM10RST        :1; /**< Reset TIM10 */
        uint32_t TIM11RST        :1; /**< Reset TIM11 */
        uint32_t res4            :13; /**< Reserved */
    } RCC_APB2RSTR_t;


    struct 
    {
        uint32_t res;
    }RESERVED2_t;

    struct 
    {
        uint32_t res;
    }RESERVED3_t;

    struct{                 //RCC AHB1 peripheral clock enable register
        uint32_t GPIOAEN         :1; /**< Enable clock for GPIOA */
        uint32_t GPIOBEN         :1; /**< Enable clock for GPIOB */
        uint32_t GPIOCEN         :1; /**< Enable clock for GPIOC */
        uint32_t GPIODEN         :1; /**< Enable clock for GPIOD */
        uint32_t GPIOEEN         :1; /**< Enable clock for GPIOE */
        uint32_t res             :2; /**< Reserved */
        uint32_t GPIOHEN         :1; /**< Enable clock for GPIOH */
        uint32_t res1            :4; /**< Reserved */
        uint32_t CRCEN           :1; /**< Enable clock for CRC */
        uint32_t res2            :8; /**< Reserved */
        uint32_t DMA1EN          :1; /**< Enable clock for DMA1 */
        uint32_t DMA2EN          :1; /**< Enable clock for DMA2 */
        uint32_t res3            :9; /**< Reserved */
    } RCC_AHB1ENR_t;

    struct{                 //RCC AHB2 peripheral clock enable register
        uint32_t res              :7; /**< Reserved */
        uint32_t OTGFSEN         :1; /**< Enable clock for OTGFS */
        uint32_t res1             :24; /**< Reserved */
    } RCC_AHB2ENR_t;


    struct 
    {
        uint32_t res;
    }RESERVED4_t;

    struct 
    {
        uint32_t res;
    }RESERVED5_t;

    struct{                 //RCC APB1 peripheral clock enable register
        uint32_t TIM2EN          :1; /**< Enable clock for TIM2 */
        uint32_t TIM3EN          :1; /**< Enable clock for TIM3 */
        uint32_t TIM4EN          :1; /**< Enable clock for TIM4 */
        uint32_t TIM5EN          :1; /**< Enable clock for TIM5 */
        uint32_t res             :7; /**< Reserved */
        uint32_t WWDGEN          :1; /**< Enable clock for WWDG */
        uint32_t res1            :2; /**< Reserved */
        uint32_t SPI2EN          :1; /**< Enable clock for SPI2 */
        uint32_t SPI3EN          :1; /**< Enable clock for SPI3 */
        uint32_t res2            :1; /**< Reserved */
        uint32_t USART2EN        :1; /**< Enable clock for USART2 */
        uint32_t res3            :3; /**< Reserved */
        uint32_t I2C1EN          :1; /**< Enable clock for I2C1 */
        uint32_t I2C2EN          :1; /**< Enable clock for I2C2 */
        uint32_t I2C3EN          :1; /**< Enable clock for I2C3 */
        uint32_t res4            :4; /**< Reserved */
        uint32_t PWREN           :1; /**< Enable clock for PWR */
        uint32_t res5            :3; /**< Reserved */
    } RCC_APB1ENR_t;

    struct{                 //RCC APB2 peripheral clock enable register
        uint32_t TIM1EN          :1; /**< Enable clock for TIM1 */
        uint32_t res             :3; /**< Reserved */
        uint32_t USART1EN        :1; /**< Enable clock for USART1 */
        uint32_t USART6EN        :1; /**< Enable clock for USART6 */
        uint32_t res1            :2; /**< Reserved */
        uint32_t ADC1EN          :1; /**< Enable clock for ADC1 */
        uint32_t res2            :2; /**< Reserved */
        uint32_t SDIOEN          :1; /**< Enable clock for SDIO */
        uint32_t SPI1EN          :1; /**< Enable clock for SPI1 */
        uint32_t SPI4EN          :1; /**< Enable clock for SPI4 */
        uint32_t SYSCFGEN        :1; /**< Enable clock for SYSCFG */
        uint32_t res3            :1; /**< Reserved */
        uint32_t TIM9EN          :1; /**< Enable clock for TIM9 */
        uint32_t TIM10EN         :1; /**< Enable clock for TIM10 */
        uint32_t TIM11EN         :1; /**< Enable clock for TIM11 */
        uint32_t res4            :13; /**< Reserved */
    } RCC_APB2ENR_t;



    struct 
    {
        uint32_t res;
    }RESERVED6_t;


    struct 
    {
        uint32_t res;
    }RESERVED7_t;

    struct{                 //RCC AHB1 peripheral clock enable in low power mode register
        uint32_t GPIOALPEN       :1; /**< Low power mode clock enable for GPIOA */
        uint32_t GPIOBLPEN       :1; /**< Low power mode clock enable for GPIOB */
        uint32_t GPIOCLPEN       :1; /**< Low power mode clock enable for GPIOC */
        uint32_t GPIODLPEN       :1; /**< Low power mode clock enable for GPIOD */
        uint32_t GPIOELPEN       :1; /**< Low power mode clock enable for GPIOE */
        uint32_t res             :2; /**< Reserved */
        uint32_t GPIOHLPEN       :1; /**< Low power mode clock enable for GPIOH */
        uint32_t res1            :4; /**< Reserved */
        uint32_t CRCLPEN         :1; /**< Low power mode clock enable for CRC */
        uint32_t res2            :2; /**< Reserved */
        uint32_t FLITFLPEN       :1; /**< Low power mode clock enable for FLITF */
        uint32_t SRAM1LPEN       :1; /**< Low power mode clock enable for SRAM1 */
        uint32_t res3            :4; /**< Reserved */
        uint32_t DMA1LPEN        :1; /**< Low power mode clock enable for DMA1 */
        uint32_t DMA2LPEN        :1; /**< Low power mode clock enable for DMA2 */
        uint32_t res4            :9; /**< Reserved */
    } RCC_AHB1LPENR_t;

    struct{                 //RCC AHB2 peripheral clock enable in low power mode register
        uint32_t res             :7; /**< Reserved */
        uint32_t OTGFSEN         :1; /**< Low power mode clock enable for OTGFS */
        uint32_t res1            :24; /**< Reserved */
    } RCC_AHB2LPENR_t;


    struct 
    {
        uint32_t res;
    }RESERVED8_t;


    struct 
    {
        uint32_t res;
    }RESERVED9_t;

    struct{                 //RCC APB1 peripheral clock enable in low power mode register
        uint32_t TIM2LPEN        :1; /**< Low power mode clock enable for TIM2 */
        uint32_t TIM3LPEN        :1; /**< Low power mode clock enable for TIM3 */
        uint32_t TIM4LPEN        :1; /**< Low power mode clock enable for TIM4 */
        uint32_t TIM5LPEN        :1; /**< Low power mode clock enable for TIM5 */
        uint32_t res             :7; /**< Reserved */
        uint32_t WWDGLPEN        :1; /**< Low power mode clock enable for WWDG */
        uint32_t res1            :2; /**< Reserved */
        uint32_t SPI2LPEN        :1; /**< Low power mode clock enable for SPI2 */
        uint32_t SPI3LPEN        :1; /**< Low power mode clock enable for SPI3 */
        uint32_t res2            :1; /**< Reserved */
        uint32_t USART2LPEN      :1; /**< Low power mode clock enable for USART2 */
        uint32_t res3            :3; /**< Reserved */
        uint32_t I2C1LPEN        :1; /**< Low power mode clock enable for I2C1 */
        uint32_t I2C2LPEN        :1; /**< Low power mode clock enable for I2C2 */
        uint32_t I2C3LPEN        :1; /**< Low power mode clock enable for I2C3 */
        uint32_t res4            :4; /**< Reserved */
        uint32_t PWRLPEN         :1; /**< Low power mode clock enable for PWR */
        uint32_t res5            :3; /**< Reserved */
    } RCC_APB1LPENR_t;

    struct{                 // RCC APB2 peripheral clock enabled in low power mode register
        uint32_t TIM1LPEN        :1; /**< Low power mode clock enable for TIM1 */
        uint32_t res              :3; /**< Reserved */
        uint32_t USART1LPEN      :1; /**< Low power mode clock enable for USART1 */
        uint32_t USART6LPEN      :1; /**< Low power mode clock enable for USART6 */
        uint32_t res1             :2; /**< Reserved */
        uint32_t ADC1LPEN        :1; /**< Low power mode clock enable for ADC1 */
        uint32_t res2             :2; /**< Reserved */
        uint32_t SDIOLPEN        :1; /**< Low power mode clock enable for SDIO */
        uint32_t SPI1LPEN        :1; /**< Low power mode clock enable for SPI1 */
        uint32_t SPI4LPEN        :1; /**< Low power mode clock enable for SPI4 */
        uint32_t SYSCFGLPEN      :1; /**< Low power mode clock enable for SYSCFG */
        uint32_t res3             :1; /**< Reserved */
        uint32_t TIM9LPEN        :1; /**< Low power mode clock enable for TIM9 */
        uint32_t TIM10LPEN       :1; /**< Low power mode clock enable for TIM10 */
        uint32_t TIM11LPEN       :1; /**< Low power mode clock enable for TIM11 */
        uint32_t res4             :13; /**< Reserved */
    } RCC_APB2LPENR_t;


    struct 
    {
        uint32_t res;
    }RESERVED10_t;


    struct 
    {
        uint32_t res;
    }RESERVED11_t;


    struct{         //RCC Backup domain control register
        uint32_t LSEON            :1; /**< LSE oscillator enable */
        uint32_t LSERDY           :1; /**< LSE oscillator ready */
        uint32_t LSEBYP           :1; /**< LSE oscillator bypass */
        uint32_t res              :5; /**< Reserved */
        uint32_t RTCSEL           :2; /**< RTC clock source selection */
        uint32_t res1             :5; /**< Reserved */
        uint32_t RTCEN            :1; /**< RTC clock enable */
        uint32_t BDRST            :1; /**< Backup domain software reset */
        uint32_t res2             :15; /**< Reserved */
    } RCC_BDCR_t;

    struct{         //RCC clock control & status register
        uint32_t LSION            :1; /**< LSI oscillator enable */
        uint32_t LSIRDY           :1; /**< LSI oscillator ready */
        uint32_t res              :22; /**< Reserved */
        uint32_t RMVF             :1; /**< Remove reset flag */
        uint32_t BORRSTF          :1; /**< BOR reset flag */
        uint32_t PINRSTF          :1; /**< PIN reset flag */
        uint32_t PORRSTF          :1; /**< POR/PDR reset flag */
        uint32_t SFTRSTF          :1; /**< Software reset flag */
        uint32_t IWDGRSTF         :1; /**< Independent watchdog reset flag */
        uint32_t WWDGRSTF         :1; /**< Window watchdog reset flag */
        uint32_t LPWRRSTF         :1; /**< Low-power reset flag */
    } RCC_CSR_t;

    struct 
    {
        uint32_t res; /**< Reserved */
    } RESERVED12_t;

    struct 
    {
        uint32_t res; /**< Reserved */
    } RESERVED13_t;

    struct{         // RCC spread spectrum clock generation register
        uint32_t MODPER           :13; /**< Modulation period */
        uint32_t INCSTEP          :15; /**< Increment step */
        uint32_t res              :2; /**< Reserved */
        uint32_t SPREADSEL        :1; /**< Spread Select */
        uint32_t SSCGEN           :1; /**< Spread spectrum clock generator enable */
    } RCC_SSCGR_t;

    struct{         //RCC PLLI2S configuration register
        uint32_t res               :3; /**< Reserved */
        uint32_t PLL2SN0           :1; /**< PLLI2S multiplication factor bit 0 */
        uint32_t PLL2SN1           :1; /**< PLLI2S multiplication factor bit 1 */
        uint32_t PLL2SN2           :1; /**< PLLI2S multiplication factor bit 2 */
        uint32_t PLL2SN3           :1; /**< PLLI2S multiplication factor bit 3 */
        uint32_t PLL2SN4           :1; /**< PLLI2S multiplication factor bit 4 */
        uint32_t PLL2SN5           :1; /**< PLLI2S multiplication factor bit 5 */
        uint32_t PLL2SN6           :1; /**< PLLI2S multiplication factor bit 6 */
        uint32_t PLL2SN7           :1; /**< PLLI2S multiplication factor bit 7 */
        uint32_t PLL2SN8           :1; /**< PLLI2S multiplication factor bit 8 */
        uint32_t res1              :13; /**< Reserved */
        uint32_t PLL2SR0           :1; /**< PLLI2S division factor for R bit 0 */
        uint32_t PLL2SR1           :1; /**< PLLI2S division factor for R bit 1 */
        uint32_t PLL2SR2           :1; /**< PLLI2S division factor for R bit 2 */
        uint32_t res2              :1; /**< Reserved */
    } RCC_PLLI2SCFGR_t;

       struct 
    {
        uint32_t res; /**< Reserved */
    } RESERVED14_t;

    struct{         // RCC Dedicated Clocks Configuration Register
        uint32_t res               :24; /**< Reserved */
        uint32_t TIMPRE            :1; /**< Timer prescaler selection */
        uint32_t PLL2SN1           :7; /**< Reserved */
    } RCC_DCKCFGR_t;


} RCC_RegDef_t; /**< RCC Register Definition */


/* 
****************** EXTI register definition structures******************
*/

typedef struct{
    struct                     //Interrupt mask register
    {
        uint32_t MR0       :1; /**< Mode for pin 0 */
        uint32_t MR1       :1; /**< Mode for pin 1 */
        uint32_t MR2       :1; /**< Mode for pin 2 */
        uint32_t MR3       :1; /**< Mode for pin 3 */
        uint32_t MR4       :1; /**< Mode for pin 4 */
        uint32_t MR5       :1; /**< Mode for pin 5 */
        uint32_t MR6       :1; /**< Mode for pin 6 */
        uint32_t MR7       :1; /**< Mode for pin 7 */
        uint32_t MR8       :1; /**< Mode for pin 8 */
        uint32_t MR9       :1; /**< Mode for pin 9 */
        uint32_t MR10      :1; /**< Mode for pin 10 */
        uint32_t MR11      :1; /**< Mode for pin 11 */
        uint32_t MR12      :1; /**< Mode for pin 12 */
        uint32_t MR13      :1; /**< Mode for pin 13 */
        uint32_t MR14      :1; /**< Mode for pin 14 */
        uint32_t MR15      :1; /**< Mode for pin 15 */
        uint32_t MR16      :1; /**< Mode for pin 13 */
        uint32_t MR17      :1; /**< Mode for pin 14 */
        uint32_t MR18      :1; /**< Mode for pin 15 */
        uint32_t res       :2;
        uint32_t MR21      :1; /**< Mode for pin 14 */
        uint32_t MR22      :1; /**< Mode for pin 15 */
        uint32_t res1      :9;
    }EXTI_IMR_t;

    struct                     //Event mask register
    {
        uint32_t MR0       :1; /**< Mode for pin 0 */
        uint32_t MR1       :1; /**< Mode for pin 1 */
        uint32_t MR2       :1; /**< Mode for pin 2 */
        uint32_t MR3       :1; /**< Mode for pin 3 */
        uint32_t MR4       :1; /**< Mode for pin 4 */
        uint32_t MR5       :1; /**< Mode for pin 5 */
        uint32_t MR6       :1; /**< Mode for pin 6 */
        uint32_t MR7       :1; /**< Mode for pin 7 */
        uint32_t MR8       :1; /**< Mode for pin 8 */
        uint32_t MR9       :1; /**< Mode for pin 9 */
        uint32_t MR10      :1; /**< Mode for pin 10 */
        uint32_t MR11      :1; /**< Mode for pin 11 */
        uint32_t MR12      :1; /**< Mode for pin 12 */
        uint32_t MR13      :1; /**< Mode for pin 13 */
        uint32_t MR14      :1; /**< Mode for pin 14 */
        uint32_t MR15      :1; /**< Mode for pin 15 */
        uint32_t MR16      :1; /**< Mode for pin 13 */
        uint32_t MR17      :1; /**< Mode for pin 14 */
        uint32_t MR18      :1; /**< Mode for pin 15 */
        uint32_t res       :2;
        uint32_t MR21      :1; /**< Mode for pin 14 */
        uint32_t MR22      :1; /**< Mode for pin 15 */
        uint32_t res1      :9;
    }EXTI_EMR_t;

    struct                     //Rising trigger selection register
    {
        uint32_t TR0       :1; /**< Mode for pin 0 */
        uint32_t TR1       :1; /**< Mode for pin 1 */
        uint32_t TR2       :1; /**< Mode for pin 2 */
        uint32_t TR3       :1; /**< Mode for pin 3 */
        uint32_t TR4       :1; /**< Mode for pin 4 */
        uint32_t TR5       :1; /**< Mode for pin 5 */
        uint32_t TR6       :1; /**< Mode for pin 6 */
        uint32_t TR7       :1; /**< Mode for pin 7 */
        uint32_t TR8       :1; /**< Mode for pin 8 */
        uint32_t TR9       :1; /**< Mode for pin 9 */
        uint32_t TR10      :1; /**< Mode for pin 10 */
        uint32_t TR11      :1; /**< Mode for pin 11 */
        uint32_t TR12      :1; /**< Mode for pin 12 */
        uint32_t TR13      :1; /**< Mode for pin 13 */
        uint32_t TR14      :1; /**< Mode for pin 14 */
        uint32_t TR15      :1; /**< Mode for pin 15 */
        uint32_t TR16      :1; /**< Mode for pin 13 */
        uint32_t TR17      :1; /**< Mode for pin 14 */
        uint32_t TR18      :1; /**< Mode for pin 15 */
        uint32_t res       :2;
        uint32_t TR21      :1; /**< Mode for pin 14 */
        uint32_t TR22      :1; /**< Mode for pin 15 */
        uint32_t res1      :9;
    }EXTI_RTSR_t;



    struct                     //Falling trigger selection register
    {
        uint32_t TR0       :1; /**< Mode for pin 0 */
        uint32_t TR1       :1; /**< Mode for pin 1 */
        uint32_t TR2       :1; /**< Mode for pin 2 */
        uint32_t TR3       :1; /**< Mode for pin 3 */
        uint32_t TR4       :1; /**< Mode for pin 4 */
        uint32_t TR5       :1; /**< Mode for pin 5 */
        uint32_t TR6       :1; /**< Mode for pin 6 */
        uint32_t TR7       :1; /**< Mode for pin 7 */
        uint32_t TR8       :1; /**< Mode for pin 8 */
        uint32_t TR9       :1; /**< Mode for pin 9 */
        uint32_t TR10      :1; /**< Mode for pin 10 */
        uint32_t TR11      :1; /**< Mode for pin 11 */
        uint32_t TR12      :1; /**< Mode for pin 12 */
        uint32_t TR13      :1; /**< Mode for pin 13 */
        uint32_t TR14      :1; /**< Mode for pin 14 */
        uint32_t TR15      :1; /**< Mode for pin 15 */
        uint32_t TR16      :1; /**< Mode for pin 13 */
        uint32_t TR17      :1; /**< Mode for pin 14 */
        uint32_t TR18      :1; /**< Mode for pin 15 */
        uint32_t res       :2;
        uint32_t TR21      :1; /**< Mode for pin 14 */
        uint32_t TR22      :1; /**< Mode for pin 15 */
        uint32_t res1      :9;
    }EXTI_FTSR_t;

    struct                 //Software interrupt event register
    {
        uint32_t SWIER0       :1; /**< Mode for pin 0 */
        uint32_t SWIER1       :1; /**< Mode for pin 1 */
        uint32_t SWIER2       :1; /**< Mode for pin 2 */
        uint32_t SWIER3       :1; /**< Mode for pin 3 */
        uint32_t SWIER4       :1; /**< Mode for pin 4 */
        uint32_t SWIER5       :1; /**< Mode for pin 5 */
        uint32_t SWIER6       :1; /**< Mode for pin 6 */
        uint32_t SWIER7       :1; /**< Mode for pin 7 */
        uint32_t SWIER8       :1; /**< Mode for pin 8 */
        uint32_t SWIER9       :1; /**< Mode for pin 9 */
        uint32_t SWIER10      :1; /**< Mode for pin 10 */
        uint32_t SWIER11      :1; /**< Mode for pin 11 */
        uint32_t SWIER12      :1; /**< Mode for pin 12 */
        uint32_t SWIER13      :1; /**< Mode for pin 13 */
        uint32_t SWIER14      :1; /**< Mode for pin 14 */
        uint32_t SWIER15      :1; /**< Mode for pin 15 */
        uint32_t SWIER16      :1; /**< Mode for pin 13 */
        uint32_t SWIER17      :1; /**< Mode for pin 14 */
        uint32_t SWIER18      :1; /**< Mode for pin 15 */
        uint32_t res       :2;
        uint32_t SWIER21      :1; /**< Mode for pin 14 */
        uint32_t SWIER22      :1; /**< Mode for pin 15 */
        uint32_t res1      :9;
    }EXTI_SWIER_t;

    struct                 //Pending register
    {
        uint32_t PR0       :1; /**< Mode for pin 0 */
        uint32_t PR1       :1; /**< Mode for pin 1 */
        uint32_t PR2       :1; /**< Mode for pin 2 */
        uint32_t PR3       :1; /**< Mode for pin 3 */
        uint32_t PR4       :1; /**< Mode for pin 4 */
        uint32_t PR5       :1; /**< Mode for pin 5 */
        uint32_t PR6       :1; /**< Mode for pin 6 */
        uint32_t PR7       :1; /**< Mode for pin 7 */
        uint32_t PR8       :1; /**< Mode for pin 8 */
        uint32_t PR9       :1; /**< Mode for pin 9 */
        uint32_t PR10      :1; /**< Mode for pin 10 */
        uint32_t PR11      :1; /**< Mode for pin 11 */
        uint32_t PR12      :1; /**< Mode for pin 12 */
        uint32_t PR13      :1; /**< Mode for pin 13 */
        uint32_t PR14      :1; /**< Mode for pin 14 */
        uint32_t PR15      :1; /**< Mode for pin 15 */
        uint32_t PR16      :1; /**< Mode for pin 13 */
        uint32_t PR17      :1; /**< Mode for pin 14 */
        uint32_t PR18      :1; /**< Mode for pin 15 */
        uint32_t res       :2;
        uint32_t PR21      :1; /**< Mode for pin 14 */
        uint32_t PR22      :1; /**< Mode for pin 15 */
        uint32_t res1      :9;
    }EXTI_PR_t;
}EXTI_RegDef_t;



/* 
****************** SYSCFG register definition structures******************
*/

typedef struct
{
    struct      //SYSCFG memory remap register
    {
        uint32_t MEM_MODE           :2;
        uint32_t res               :30;
    }SYSCFG_MEMRMP_t;


    struct      //SYSCFG peripheral mode configuration register
    {
        uint32_t res               :16;
        uint32_t ADC1DC2           :1;
        uint32_t res1              :15;
    }SYSCFG_PMC_t;


    struct      //SYSCFG external interrupt configuration register 1
    {
        uint32_t EXTI0              :4;
        uint32_t EXTI1              :4;
        uint32_t EXTI2              :4;
        uint32_t EXTI3              :4;
        uint32_t res               :16;
    }SYSCFG_EXTICR1_t;


    struct      //SYSCFG external interrupt configuration register 2
    {
        uint32_t EXTI4              :4;
        uint32_t EXTI5              :4;
        uint32_t EXTI6              :4;
        uint32_t EXTI7              :4;
        uint32_t res               :16;
    }SYSCFG_EXTICR2_t;



    struct      //SYSCFG external interrupt configuration register 3
    {
        uint32_t EXTI8              :4;
        uint32_t EXTI9              :4;
        uint32_t EXTI10             :4;
        uint32_t EXTI11             :4;
        uint32_t res               :16;
    }SYSCFG_EXTICR3_t;



    struct      //SYSCFG external interrupt configuration register 4
    {
        uint32_t EXTI12             :4;
        uint32_t EXTI13             :4;
        uint32_t EXTI14             :4;
        uint32_t EXTI15             :4;
        uint32_t res               :16;
    }SYSCFG_EXTICR4_t;

    struct      // Compensation cell control register
    {
        uint32_t CMP_PD            :1;
        uint32_t res               :7;
        uint32_t READY             :1;
        uint32_t res1              :23;
    }SYSCFG_CMPCR_t;
}SYSCFG_RegDef_t;



/* 
****************** SPI register definition structures******************
*/

typedef struct{
    struct       //SPI control register 1
    {
        uint32_t CPHA               :1;
        uint32_t CPOL               :1;
        uint32_t MSTR               :1;
        uint32_t BR                 :3;
        uint32_t SPE                :1;
        uint32_t LSBFIRST           :1;
        uint32_t SSI                :1;
        uint32_t SSM                :1;
        uint32_t RXONLY             :1;
        uint32_t DFF                :1;
        uint32_t CRCNEXT            :1;
        uint32_t CRCEN              :1;
        uint32_t BIDIOE             :1;
        uint32_t BIDIMODE           :1;
        uint32_t res1               :16;
    }SPI_CR1_t;



    struct       //SPI control register 2
    {
        uint32_t RXDMAEN            :1;
        uint32_t TXDMAEN            :1;
        uint32_t SSOE               :1;
        uint32_t res                :1;
        uint32_t FRF                :1;
        uint32_t ERRIE              :1;
        uint32_t RXNEIE             :1;
        uint32_t TXEIE              :1;
        uint32_t res1               :24;
    }SPI_CR2_t;



    struct       //SPI status register
    {
        uint32_t RXNE               :1;
        uint32_t TXE                :1;
        uint32_t CHSIDE             :1;
        uint32_t UDR                :1;
        uint32_t CRCERR             :1;
        uint32_t MODF               :1;
        uint32_t OVF                :1;
        uint32_t BSY                :1;
        uint32_t res                :23;
    }SPI_SR_t;



    struct       //SPI data register
    {
        uint32_t DR                 :16;
        uint32_t res                :16;
    }SPI_DR_t;
    


    struct       //SPI CRC polynomial register
    {
        uint32_t CRCPOLY            :16;
        uint32_t res                :16;
    }SPI_CRCPR_t;


    struct       //SPI RX CRC register
    {
        uint32_t RXCRC              :16;
        uint32_t res                :16;
    }SPI_RXCRCR_t;


    struct       //SPI TX CRC register
    {
        uint32_t TXCRC              :16;
        uint32_t res                :16;
    }SPI_TXCRCR_t;



    struct       //SPI_I2S configuration register
    {
        uint32_t CHLEN              :1;
        uint32_t DATLEN             :2;
        uint32_t CKPOL              :1;
        uint32_t I2SSTD             :2;
        uint32_t res                :1;
        uint32_t PCMSYNC            :1;
        uint32_t I2SCFG             :2;
        uint32_t I2SE               :1;
        uint32_t I2SMOD             :1;
        uint32_t res1               :20;
    }SPI_I2SCFGR_t;



    struct       //SPI_I2S prescaler register
    {
        uint32_t I2SDIV             :8;
        uint32_t ODD                :1;
        uint32_t MCKOE              :1;
        uint32_t res                :22 ;
    }SPI_I2SPR_t;
}SPI_RegDef_t;





/*
**************** Peripheral definitions (peripheral addresses typecasted to structures)
*/




/** 
 * @brief GPIO peripheral definitions for GPIOA, GPIOB, etc.
 */
#define GPIOA                 ((GPIO_RegDef_t*)  GPIOA_BASEADDR)
#define GPIOB                 ((GPIO_RegDef_t*)  GPIOB_BASEADDR)
#define GPIOC                 ((GPIO_RegDef_t*)  GPIOC_BASEADDR)
#define GPIOD                 ((GPIO_RegDef_t*)  GPIOD_BASEADDR)
#define GPIOE                 ((GPIO_RegDef_t*)  GPIOE_BASEADDR)
#define GPIOH                 ((GPIO_RegDef_t*)  GPIOH_BASEADDR)

/** 
 * @brief RCC peripheral definition.
 */
#define RCC                   ((RCC_RegDef_t*)    RCC_BASEADDR)


#define EXTI	( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG	( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

#define SPI1    ( (SPI_RegDef_t*) SPI1_BASEADDR )
#define SPI2    ( (SPI_RegDef_t*) SPI2_BASEADDR ) 
#define SPI3    ( (SPI_RegDef_t*) SPI3_BASEADDR )   
#define SPI4    ( (SPI_RegDef_t*) SPI4_BASEADDR )

#define I2C1    ( (I2C_RegDef_t*) I2C1_BASEADDR )
#define I2C2    ( (I2C_RegDef_t*) I2C2_BASEADDR )
#define I2C3    ( (I2C_RegDef_t*) I2C3_BASEADDR )

#define USART1  ( (USART_RegDef_t*)USART1_BASEADDR )
#define USART6  ( (USART_RegDef_t*)USART6_BASEADDR )


/** 
 * @defgroup CLOCK_ENABLE_MACROS Clock enable macros for GPIO, I2C, SPI, etc.
 * @{
 */
 
#define GPIOA_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOAEN=1)
#define GPIOB_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOBEN=1)
#define GPIOC_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOCEN=1)
#define GPIOD_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIODEN=1)
#define GPIOE_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOEEN=1)
#define GPIOH_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOHEN=1)
  
 
/*
************clock enable macros for I2Cx peripherals*************
*/
 
#define I2C1_PCLK_EN()                   (RCC->RCC_APB1ENR_t.I2C1EN=1)
#define I2C2_PCLK_EN()                   (RCC->RCC_APB1ENR_t.I2C2EN=1)
#define I2C3_PCLK_EN()                   (RCC->RCC_APB1ENR_t.I2C3EN=1)

/*
************clock enable macros for SPIx peripherals*************
*/

#define SPI1_PCLK_EN()                   (RCC->RCC_APB2ENR_t.SPI1EN=1)
#define SPI2_PCLK_EN()                   (RCC->RCC_APB1ENR_t.SPI2EN=1)
#define SPI3_PCLK_EN()                   (RCC->RCC_APB1ENR_t.SPI3EN=1)
#define SPI4_PCLK_EN()                   (RCC->RCC_APB2ENR_t.SPI4EN=1)



/*
************clock enable macros for USARTx peripherals*************
*/

#define USART1_PCLK_EN()                   (RCC->RCC_APB2ENR_t.USART1EN=1)
#define USART6_PCLK_EN()                   (RCC->RCC_APB2ENR_t.USART6EN=1)


/*
************clock enable macros for SYSCFG peripherals*************
*/

#define SYSCFG_PCLK_EN()                   (RCC->RCC_APB2ENR_t.SYSCFGEN=1)



/*
************clock disable macros for GPIO peripherals*************
*/

#define GPIOA_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOAEN=0)
#define GPIOB_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOBEN=0)
#define GPIOC_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOCEN=0)
#define GPIOD_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIODEN=0)
#define GPIOE_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOEEN=0)
#define GPIOH_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOHEN=0)



/*
************clock disable macros for I2Cx peripherals*************
*/


#define I2C1_PCLK_DI()                   (RCC->RCC_APB1ENR_t.I2C1EN=0)
#define I2C2_PCLK_DI()                   (RCC->RCC_APB1ENR_t.I2C2EN=0)
#define I2C3_PCLK_DI()                   (RCC->RCC_APB1ENR_t.I2C3EN=0)



/*
************clock disable macros for SPIx peripherals*************
*/

#define SPI1_PCLK_DI()                   (RCC->RCC_APB2ENR_t.SPI1EN=0)
#define SPI2_PCLK_DI()                   (RCC->RCC_APB1ENR_t.SPI2EN=0)
#define SPI3_PCLK_DI()                   (RCC->RCC_APB1ENR_t.SPI3EN=0)
#define SPI4_PCLK_DI()                   (RCC->RCC_APB2ENR_t.SPI4EN=0)



/*
************clock disable macros for USARTx peripherals*************
*/

#define USART1_PCLK_DI()                   (RCC->RCC_APB2ENR_t.USART1EN=0)
#define USART6_PCLK_DI()                   (RCC->RCC_APB2ENR_t.USART6EN=0)



/*
************clock disable macros for SYSCFG peripherals*************
*/

#define SYSCFG_PCLK_DI()                   (RCC->RCC_APB2ENR_t.SYSCFGEN=0)

/*
    GPIO peripherals reset macros
*/

#define GPIOA_REG_RESET()                 do{ (RCC->RCC_AHB1RSTR_t.GPIOARST=1);    (RCC->RCC_AHB1RSTR_t.GPIOARST=0);}while(0)
#define GPIOB_REG_RESET()                 do{ (RCC->RCC_AHB1RSTR_t.GPIOBRST=1);    (RCC->RCC_AHB1RSTR_t.GPIOBRST=0);}while(0)
#define GPIOC_REG_RESET()                 do{ (RCC->RCC_AHB1RSTR_t.GPIOCRST=1);    (RCC->RCC_AHB1RSTR_t.GPIOCRST=0);}while(0)
#define GPIOD_REG_RESET()                 do{ (RCC->RCC_AHB1RSTR_t.GPIODRST=1);    (RCC->RCC_AHB1RSTR_t.GPIODRST=0);}while(0)
#define GPIOE_REG_RESET()                 do{ (RCC->RCC_AHB1RSTR_t.GPIOERST=1);    (RCC->RCC_AHB1RSTR_t.GPIOERST=0);}while(0)
#define GPIOH_REG_RESET()                 do{ (RCC->RCC_AHB1RSTR_t.GPIOHRST=1);    (RCC->RCC_AHB1RSTR_t.GPIOHRST=0);}while(0)


/*
    SPI peripherals reset macros
*/

#define SPI1_REG_RESET()                  do{ (RCC->RCC_APB2RSTR_t.SPI1RST=1);     (RCC->RCC_APB2RSTR_t.SPI1RST=0);}while(0)
#define SPI2_REG_RESET()                  do{ (RCC->RCC_APB1RSTR_t.SPI2RST=1);     (RCC->RCC_APB1RSTR_t.SPI2RST=0);}while(0)
#define SPI3_REG_RESET()                  do{ (RCC->RCC_APB1RSTR_t.SPI3RST=1);     (RCC->RCC_APB1RSTR_t.SPI3RST=0);}while(0)
#define SPI4_REG_RESET()                  do{ (RCC->RCC_APB2RSTR_t.SPI4RST=1);     (RCC->RCC_APB2RSTR_t.SPI4RST=0);}while(0)

/*
    IRQ numbers for stm32f401x
*/

#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40



/*
    IRQ priority macros for stm32f401x
*/

#define NVIC_IRQ_PRIO0          0
#define NVIC_IRQ_PRIO1          1
#define NVIC_IRQ_PRIO2          2
#define NVIC_IRQ_PRIO3          3
#define NVIC_IRQ_PRIO4          4
#define NVIC_IRQ_PRIO5          5
#define NVIC_IRQ_PRIO6          6
#define NVIC_IRQ_PRIO7          7
#define NVIC_IRQ_PRIO8          8
#define NVIC_IRQ_PRIO9          9
#define NVIC_IRQ_PRIO10         10
#define NVIC_IRQ_PRIO11         11
#define NVIC_IRQ_PRIO12         12
#define NVIC_IRQ_PRIO13         13
#define NVIC_IRQ_PRIO14         14
#define NVIC_IRQ_PRIO15         15


/*
        returns port code
*/

#define GPIO_BASEADDR_TO_CODE(x)          ( (x  ==   GPIOA)  ? 0:\
                                            (x  ==   GPIOB)  ? 1:\
                                            (x  ==   GPIOC)  ? 2:\
                                            (x  ==   GPIOD)  ? 3:\
                                            (x  ==   GPIOE)  ? 4:\
                                            (x  ==   GPIOH)  ? 7:0 )

/**
 * @brief Generic macros.
 */
  
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

  
  
#include "stm32f401xx_gpio_driver.h"

#include "Stm32f401xx_spi_driver.h"

#endif /* INC_STM32F401XX_H_ */