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
 * @defgroup PERIPH_BASEADDR Base addresses for APB and AHB buses
 * @{
 */
#define PERIPH_BASEADDR                0x40000000U /**< Base address of peripheral memory */
#define APB1PERIPH_BASEADDR            PERIPH_BASEADDR /**< Base address of APB1 peripheral memory */
#define APB2PERIPH_BASEADDR            0x40010000U /**< Base address of APB2 peripheral memory */
#define AHB1PERIPH_BASEADDR            0x40020000U /**< Base address of AHB1 peripheral memory */
#define AHB2PERIPH_BASEADDR            0x50000000U /**< Base address of AHB2 peripheral memory */
/** @} */

/**
 * @defgroup PERIPH_BASEADDR Individual peripheral base addresses
 * @{
 */
#define TIM2_BASEADDR                  APB1PERIPH_BASEADDR /**< Base address of TIM2 */
#define TIM3_BASEADDR                  (APB1PERIPH_BASEADDR+0x400) /**< Base address of TIM3 */
#define TIM4_BASEADDR                  (APB1PERIPH_BASEADDR+0x800) /**< Base address of TIM4 */
#define TIM5_BASEADDR                  (APB1PERIPH_BASEADDR+0xC00) /**< Base address of TIM5 */
#define USART2_BASEADDR                (APB1PERIPH_BASEADDR+0x4400) /**< Base address of USART2 */
#define GPIOA_BASEADDR                 (AHB1PERIPH_BASEADDR) /**< Base address of GPIOA */
#define GPIOB_BASEADDR                 (AHB1PERIPH_BASEADDR+0x400) /**< Base address of GPIOB */
#define RCC_BASEADDR                   (AHB1PERIPH_BASEADDR+0x3800) /**< Base address of RCC */
/** @} */

/** 
 * @brief Structure for GPIO port mode register.
 */
volatile typedef struct 
{
    struct {
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
    
    // Similar documentation is repeated for GPIOx_OTYPER_t, GPIOx_OSPEEDR_t,
    // and other GPIO register structures...
} GPIO_RegDef_t;

/**
 * @brief Structure for RCC clock control register.
 */
typedef struct
{
    struct {
        uint32_t HSI_ON           :1; /**< High-speed internal clock enable */
        uint32_t HSI_RDY          :1; /**< HSI clock ready flag */
        uint32_t res              :1; /**< Reserved */
        uint32_t HSI_TRIM         :5; /**< HSI clock trimming */
        uint32_t HSI_CAL          :8; /**< HSI clock calibration */
        uint32_t HSE_ON           :1; /**< High-speed external clock enable */
        uint32_t HSE_RDY          :1; /**< HSE clock ready flag */
        uint32_t HSE_BYP          :1; /**< HSE clock bypass */
        uint32_t CSS_ON           :1; /**< Clock security system enable */
        uint32_t res1             :4; /**< Reserved */
        uint32_t PLL_ON           :1; /**< PLL enable */
        uint32_t PLL_RDY          :1; /**< PLL ready flag */
        uint32_t PLL_I2S_ON       :1; /**< PLLI2S enable */
        uint32_t PLL_I2S_RDY      :1; /**< PLLI2S ready flag */
        uint32_t res2             :4; /**< Reserved */
    } RCC_CR_t;
    
    // Similar documentation is repeated for other RCC register structures...
} RCC_RegDef_t;

/** 
 * @brief GPIO peripheral definitions for GPIOA, GPIOB, etc.
 */
#define GPIOA                 ((GPIO_RegDef_t*)  GPIOA_BASEADDR)
#define GPIOB                 ((GPIO_RegDef_t*)  GPIOB_BASEADDR)

/** 
 * @brief RCC peripheral definition.
 */
#define RCC                   ((RCC_RegDef_t*)    RCC_BASEADDR)

/** 
 * @defgroup CLOCK_ENABLE_MACROS Clock enable macros for GPIO, I2C, SPI, etc.
 * @{
 */
#define GPIOA_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOA_EN=1)
#define I2C1_PCLK_EN()                   (RCC->RCC_APB1ENR_t.I2C1_EN=1)
#define SPI1_PCLK_EN()                   (RCC->RCC_APB2ENR_t.SPI1_EN=1)
#define USART1_PCLK_EN()                 (RCC->RCC_APB2ENR_t.USART1_EN=1)
/** @} */

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

#endif /* INC_STM32F401XX_H_ */
