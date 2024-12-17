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
#include <stddef.h>

/*
 * ************************ Processor-specific details **********************
 */

/** 
 * @defgroup NVIC_ISERx ARM Cortex Mx NVIC ISERx register addresses
 * @brief Interrupt Set-Enable Registers (ISER) for enabling interrupts in the NVIC.
 * @{
 */
#define NVIC_ISER0      ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1      ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2      ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3      ((volatile uint32_t*)0xE000E10C)
/** @} */

/** 
 * @defgroup NVIC_ICERx ARM Cortex Mx NVIC ICERx register addresses
 * @brief Interrupt Clear-Enable Registers (ICER) for disabling interrupts in the NVIC.
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
 * @brief Memory layout for Flash, SRAM, and ROM on STM32F401xx.
 * @{
 */
#define FLASH_BASEADDR                 0x08000000U /**< Base address of Flash memory */
#define SRAM_BASEADDR                  0x20000000U /**< Base address of SRAM memory */
#define ROM_BASEADDR                   0x1FFF0000U /**< Base address of ROM memory */
/** @} */

/**
 * @defgroup BUS_BASEADDR Base addresses for APB and AHB buses
 * @brief Base addresses of the Advanced Peripheral Bus (APB) and Advanced High-performance Bus (AHB).
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
 * @brief Base addresses for various individual peripherals on STM32F401xx.
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
******************RCC register definition structures******************
*/

/**
 * @brief RCC Register Definition structure.
 * 
 * This structure contains all the register configurations for the 
 * Reset and Clock Control (RCC) peripheral on the STM32F401xx.
 */
typedef struct
{

    /**
     * @brief RCC clock control register (RCC_CR).
     * 
     * Controls the clock configuration and status for HSI, HSE, and PLL.
     */
    struct{         
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


    /**
     * @brief RCC PLL configuration register (RCC_PLLCFGR).
     * 
     * Configures the main PLL (PLL) multiplication and division factors.
     */
    struct{         
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


    /**
     * @brief RCC clock configuration register (RCC_CFGR).
     * 
     * Controls the system clock, AHB, APB1, APB2 prescalers, and other clock outputs.
     */
    struct{         
        uint32_t SW0              :1; /**< System clock switch bit 0 */
        uint32_t SW1              :1; /**< System clock switch bit 1 */
        uint32_t SWS              :2; /**< System clock switch status bits */
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
  
    /**
     * @brief RCC clock interrupt register (RCC_CIR).
     * 
     * Manages clock interrupt flags and enables for LSI, LSE, HSI, HSE, and PLL.
     */
    struct{         
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

    /**
     * @brief RCC AHB1 peripheral reset register (RCC_AHB1RSTR).
     * 
     * Controls the reset of peripherals on the AHB1 bus.
     */
    struct{         
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


   /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED0_t;
  
   /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED1_t;



    /**
     * @brief RCC AHB2 peripheral reset register (RCC_AHB2RSTR).
     * 
     * Controls the reset state for AHB2 peripherals.
     */
    struct{         
        uint32_t res              :7; /**< Reserved */
        uint32_t OTGFSRST         :1; /**< Reset OTGFS */
        uint32_t res1             :24; /**< Reserved */
    } RCC_AHB2RSTR_t;


    /**
     * @brief RCC APB1 peripheral reset register (RCC_APB1RSTR).
     * 
     * Controls the reset state for APB1 peripherals.
     */
    struct{                 
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

    /**
     * @brief RCC APB2 peripheral reset register (RCC_APB2RSTR).
     * 
     * Controls the reset state for APB2 peripherals.
     */
    struct{                
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

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED2_t;
   
    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED3_t;

    /**
     * @brief RCC AHB1 peripheral clock enable register (RCC_AHB1ENR).
     * 
     * Controls the clock enable for AHB1 peripherals.
     */
    struct{                 
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

    /**
     * @brief RCC AHB2 peripheral clock enable register (RCC_AHB2ENR).
     * 
     * Controls the clock enable for AHB2 peripherals.
     */
    struct{                 
        uint32_t res              :7; /**< Reserved */
        uint32_t OTGFSEN          :1; /**< Enable clock for OTGFS */
        uint32_t res1             :24; /**< Reserved */
    } RCC_AHB2ENR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED4_t;
   
    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED5_t;

    /**
     * @brief RCC APB1 peripheral clock enable register (RCC_APB1ENR).
     * 
     * Controls the clock enable for APB1 peripherals.
     */
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

    /**
     * @brief RCC APB2 peripheral clock enable register (RCC_APB2ENR).
     * 
     * Controls the clock enable for APB2 peripherals.
     */
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


    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED6_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED7_t;

    /**
     * @brief RCC AHB1 peripheral clock low power enable register (RCC_AHB1LPENR).
     * 
     * Controls the clock enable during low power mode for AHB1 peripherals.
     */
    struct{                 
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

    /**
     * @brief RCC AHB2 peripheral clock low power enable register (RCC_AHB2LPENR).
     * 
     * Controls the clock enable during low power mode for AHB2 peripherals.
     */
    struct{                 
        uint32_t res             :7; /**< Reserved */
        uint32_t OTGFSEN         :1; /**< Low power mode clock enable for OTGFS */
        uint32_t res1            :24; /**< Reserved */
    } RCC_AHB2LPENR_t;


    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED8_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED9_t;

    /**
     * @brief RCC APB1 peripheral clock low power enable register (RCC_APB1LPENR).
     * 
     * Controls the clock enable during low power mode for APB1 peripherals.
     */
    struct{                 
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


    /**
     * @brief RCC APB2 peripheral clock low power enable register (RCC_APB2LPENR).
     * 
     * Controls the clock enable during low power mode for APB2 peripherals.
     */
    struct{                 
        uint32_t TIM1LPEN        :1; /**< Low power mode clock enable for TIM1 */
        uint32_t res             :3; /**< Reserved */
        uint32_t USART1LPEN      :1; /**< Low power mode clock enable for USART1 */
        uint32_t USART6LPEN      :1; /**< Low power mode clock enable for USART6 */
        uint32_t res1            :2; /**< Reserved */
        uint32_t ADC1LPEN        :1; /**< Low power mode clock enable for ADC1 */
        uint32_t res2            :2; /**< Reserved */
        uint32_t SDIOLPEN        :1; /**< Low power mode clock enable for SDIO */
        uint32_t SPI1LPEN        :1; /**< Low power mode clock enable for SPI1 */
        uint32_t SPI4LPEN        :1; /**< Low power mode clock enable for SPI4 */
        uint32_t SYSCFGLPEN      :1; /**< Low power mode clock enable for SYSCFG */
        uint32_t res3            :1; /**< Reserved */
        uint32_t TIM9LPEN        :1; /**< Low power mode clock enable for TIM9 */
        uint32_t TIM10LPEN       :1; /**< Low power mode clock enable for TIM10 */
        uint32_t TIM11LPEN       :1; /**< Low power mode clock enable for TIM11 */
        uint32_t res4            :13; /**< Reserved */
    } RCC_APB2LPENR_t;


    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED10_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res;
    }RESERVED11_t;

    /**
     * @brief RCC Backup domain control register (RCC_BDCR).
     * 
     * Controls the settings for the backup domain, including LSE oscillator and RTC.
     */
    struct{        
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


    /**
     * @brief RCC clock control and status register (RCC_CSR).
     * 
     * Manages the settings for the low-speed internal oscillator and reset flags.
     */
    struct{         
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


    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res; /**< Reserved */
    } RESERVED12_t;


    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint32_t res; /**< Reserved */
    } RESERVED13_t;


    /**
     * @brief RCC spread spectrum clock generation register (RCC_SSCGR).
     * 
     * Configures spread spectrum modulation settings.
     */
    struct{         
        uint32_t MODPER           :13; /**< Modulation period */
        uint32_t INCSTEP          :15; /**< Increment step */
        uint32_t res              :2; /**< Reserved */
        uint32_t SPREADSEL        :1; /**< Spread Select */
        uint32_t SSCGEN           :1; /**< Spread spectrum clock generator enable */
    } RCC_SSCGR_t;


    /**
     * @brief RCC PLLI2S configuration register (RCC_PLLI2SCFGR).
     * 
     * Configures the multiplication and division factors for the PLLI2S.
     */
    struct{         
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


    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
       struct 
    {
        uint32_t res; /**< Reserved */
    } RESERVED14_t;


    /**
     * @brief RCC dedicated clocks configuration register (RCC_DCKCFGR).
     * 
     * Configures specific peripheral clocks.
     */
    struct{         
        uint32_t res               :24; /**< Reserved */
        uint32_t TIMPRE            :1; /**< Timer prescaler selection */
        uint32_t PLL2SN1           :7; /**< Reserved */
    } RCC_DCKCFGR_t;


} RCC_RegDef_t; /**< RCC Register Definition */


/* 
****************** SYSCFG register definition structures******************
*/

/**
 * @brief SYSCFG Register Definition structure.
 * 
 * This structure represents the configuration registers for the System Configuration (SYSCFG)
 * peripheral, including memory remap, peripheral mode, external interrupt configuration,
 * and compensation cell control.
 */
typedef struct
{
    /**
     * @brief SYSCFG memory remap register (SYSCFG_MEMRMP).
     * 
     * Configures memory remap settings for boot configuration.
     */
    struct      
    {
        uint32_t MEM_MODE           :2; /**< Memory Mapping Selection. */
        uint32_t res                :30; /**< Reserved. */
    } SYSCFG_MEMRMP_t;

    /**
     * @brief SYSCFG peripheral mode configuration register (SYSCFG_PMC).
     * 
     * Controls peripheral mode settings, including ADC configuration.
     */
    struct      
    {
        uint32_t res                :16; /**< Reserved. */
        uint32_t ADC1DC2            :1; /**< ADC1 DMA Channel 2 Configuration. */
        uint32_t res1               :15; /**< Reserved. */
    } SYSCFG_PMC_t;

    /**
     * @brief SYSCFG external interrupt configuration register 1 (SYSCFG_EXTICR1).
     * 
     * Configures the source input for external interrupt lines 0 to 3.
     */
    struct      
    {
        uint32_t EXTI0              :4; /**< External Interrupt 0 Configuration. */
        uint32_t EXTI1              :4; /**< External Interrupt 1 Configuration. */
        uint32_t EXTI2              :4; /**< External Interrupt 2 Configuration. */
        uint32_t EXTI3              :4; /**< External Interrupt 3 Configuration. */
        uint32_t res                :16; /**< Reserved. */
    } SYSCFG_EXTICR1_t;

    /**
     * @brief SYSCFG external interrupt configuration register 2 (SYSCFG_EXTICR2).
     * 
     * Configures the source input for external interrupt lines 4 to 7.
     */
    struct      
    {
        uint32_t EXTI4              :4; /**< External Interrupt 4 Configuration. */
        uint32_t EXTI5              :4; /**< External Interrupt 5 Configuration. */
        uint32_t EXTI6              :4; /**< External Interrupt 6 Configuration. */
        uint32_t EXTI7              :4; /**< External Interrupt 7 Configuration. */
        uint32_t res                :16; /**< Reserved. */
    } SYSCFG_EXTICR2_t;

    /**
     * @brief SYSCFG external interrupt configuration register 3 (SYSCFG_EXTICR3).
     * 
     * Configures the source input for external interrupt lines 8 to 11.
     */
    struct      
    {
        uint32_t EXTI8              :4; /**< External Interrupt 8 Configuration. */
        uint32_t EXTI9              :4; /**< External Interrupt 9 Configuration. */
        uint32_t EXTI10             :4; /**< External Interrupt 10 Configuration. */
        uint32_t EXTI11             :4; /**< External Interrupt 11 Configuration. */
        uint32_t res                :16; /**< Reserved. */
    } SYSCFG_EXTICR3_t;

    /**
     * @brief SYSCFG external interrupt configuration register 4 (SYSCFG_EXTICR4).
     * 
     * Configures the source input for external interrupt lines 12 to 15.
     */
    struct      
    {
        uint32_t EXTI12             :4; /**< External Interrupt 12 Configuration. */
        uint32_t EXTI13             :4; /**< External Interrupt 13 Configuration. */
        uint32_t EXTI14             :4; /**< External Interrupt 14 Configuration. */
        uint32_t EXTI15             :4; /**< External Interrupt 15 Configuration. */
        uint32_t res                :16; /**< Reserved. */
    } SYSCFG_EXTICR4_t;

    /**
     * @brief SYSCFG compensation cell control register (SYSCFG_CMPCR).
     * 
     * Controls the compensation cell for optimizing I/O speed performance.
     */
    struct      
    {
        uint32_t CMP_PD             :1; /**< Compensation Cell Power-Down. */
        uint32_t res                :7; /**< Reserved. */
        uint32_t READY              :1; /**< Compensation Cell Ready Flag. */
        uint32_t res1               :23; /**< Reserved. */
    } SYSCFG_CMPCR_t;

} SYSCFG_RegDef_t;




/* 
******************GPIO register definition structures******************
*/


/** 
 * @brief GPIO register definition structure.
 * 
 * This structure defines the registers for a GPIO port on the STM32F401xx.
 */
volatile typedef struct 
{
    /** 
     * @brief Structure for GPIO port mode register.
     * 
     * Defines the mode configuration for each pin in the GPIO port.
     */
    struct{  
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



    /** 
     * @brief Structure for GPIO port output type register.
     * 
     * Defines the output type for each pin in the GPIO port.
     */
    struct{         
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
        uint32_t res       :16; /**< Reserved bits */
    } GPIOx_OTYPER_t;


    /**
     * @brief GPIO port output speed register.
     * 
     * Configures the output speed for each pin in the GPIO port.
     */
    struct{         
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
  
  
    /**
     * @brief GPIO port pull-up/pull-down register.
     * 
     * Configures the pull-up or pull-down resistors for each pin in the GPIO port.
     */
    struct{         
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


    /**
     * @brief GPIO port input data register.
     * 
     * Reads the input data for each pin in the GPIO port.
     */
    const  struct{         
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
   
   
    /**
     * @brief GPIO port output data register.
     * 
     * Controls the output data for each pin in the GPIO port.
     */
    struct{         
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


    /**
     * @brief GPIO port bit set/reset register.
     * 
     * Used to set or reset specific bits in the GPIO port.
     */

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

    /**
     * @brief GPIO port configuration lock register.
     * 
     * Locks the configuration of the GPIO port until the next reset.
     */
    struct{        
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
        uint32_t res        :15; /**< Reserved bits */
    } GPIOx_LCKR_t;

    /**
     * @brief GPIO alternate function low register.
     * 
     * Configures the alternate functions for pins 0 to 7 in the GPIO port.
     */
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
   
    /**
     * @brief GPIO alternate function high register.
     * 
     * Configures the alternate functions for pins 8 to 15 in the GPIO port.
     */
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


/**
 * @brief DMA Register Definition Structure
 * 
 * This structure represents the DMA Low Interrupt Status Register (LISR) and 
 * DMA High Interrupt Status Register (HISR) of a DMA peripheral.
 */
volatile typedef struct {

    /**
     * @brief DMA Low Interrupt Status Register (DMA_LISR_t)
     * 
     * This register contains interrupt flags for the lower half of the DMA streams.
     */
    struct {
        uint32_t FEIF0               :1; /**< FIFO Error Interrupt Flag for stream 0 */
        uint32_t res                 :1; /**< Reserved bit */
        uint32_t DMEIF0              :1; /**< Direct Mode Error Interrupt Flag for stream 0 */
        uint32_t TEIF0               :1; /**< Transfer Error Interrupt Flag for stream 0 */
        uint32_t HTIF0               :1; /**< Half Transfer Interrupt Flag for stream 0 */
        uint32_t TCIF0               :1; /**< Transfer Complete Interrupt Flag for stream 0 */
        uint32_t FEIF1               :1; /**< FIFO Error Interrupt Flag for stream 1 */
        uint32_t res1                :1; /**< Reserved bit */
        uint32_t DMEIF1              :1; /**< Direct Mode Error Interrupt Flag for stream 1 */
        uint32_t TEIF1               :1; /**< Transfer Error Interrupt Flag for stream 1 */
        uint32_t HTIF1               :1; /**< Half Transfer Interrupt Flag for stream 1 */
        uint32_t TCIF1               :1; /**< Transfer Complete Interrupt Flag for stream 1 */
        uint32_t res2                :4; /**< Reserved bits */
        uint32_t FEIF2               :1; /**< FIFO Error Interrupt Flag for stream 2 */
        uint32_t res3                :1; /**< Reserved bit */
        uint32_t DMEIF2              :1; /**< Direct Mode Error Interrupt Flag for stream 2 */
        uint32_t TEIF2               :1; /**< Transfer Error Interrupt Flag for stream 2 */
        uint32_t HTIF2               :1; /**< Half Transfer Interrupt Flag for stream 2 */
        uint32_t TCIF2               :1; /**< Transfer Complete Interrupt Flag for stream 2 */
        uint32_t FEIF3               :1; /**< FIFO Error Interrupt Flag for stream 3 */
        uint32_t res4                :1; /**< Reserved bit */
        uint32_t DMEIF3              :1; /**< Direct Mode Error Interrupt Flag for stream 3 */
        uint32_t TEIF3               :1; /**< Transfer Error Interrupt Flag for stream 3 */
        uint32_t HTIF3               :1; /**< Half Transfer Interrupt Flag for stream 3 */
        uint32_t TCIF3               :1; /**< Transfer Complete Interrupt Flag for stream 3 */
        uint32_t res5                :4; /**< Reserved bits */
    } DMA_LISR_t;

    /**
     * @brief DMA High Interrupt Status Register (DMA_HISR_t)
     * 
     * This register contains interrupt flags for the higher half of the DMA streams.
     */
    struct {
        uint32_t FEIF4               :1; /**< FIFO Error Interrupt Flag for stream 4 */
        uint32_t res                 :1; /**< Reserved bit */
        uint32_t DMEIF4              :1; /**< Direct Mode Error Interrupt Flag for stream 4 */
        uint32_t TEIF4               :1; /**< Transfer Error Interrupt Flag for stream 4 */
        uint32_t HTIF4               :1; /**< Half Transfer Interrupt Flag for stream 4 */
        uint32_t TCIF4               :1; /**< Transfer Complete Interrupt Flag for stream 4 */
        uint32_t FEIF5               :1; /**< FIFO Error Interrupt Flag for stream 5 */
        uint32_t res1                :1; /**< Reserved bit */
        uint32_t DMEIF5              :1; /**< Direct Mode Error Interrupt Flag for stream 5 */
        uint32_t TEIF5               :1; /**< Transfer Error Interrupt Flag for stream 5 */
        uint32_t HTIF5               :1; /**< Half Transfer Interrupt Flag for stream 5 */
        uint32_t TCIF5               :1; /**< Transfer Complete Interrupt Flag for stream 5 */
        uint32_t res2                :4; /**< Reserved bits */
        uint32_t FEIF6               :1; /**< FIFO Error Interrupt Flag for stream 6 */
        uint32_t res3                :1; /**< Reserved bit */
        uint32_t DMEIF6              :1; /**< Direct Mode Error Interrupt Flag for stream 6 */
        uint32_t TEIF6               :1; /**< Transfer Error Interrupt Flag for stream 6 */
        uint32_t HTIF6               :1; /**< Half Transfer Interrupt Flag for stream 6 */
        uint32_t TCIF6               :1; /**< Transfer Complete Interrupt Flag for stream 6 */
        uint32_t FEIF7               :1; /**< FIFO Error Interrupt Flag for stream 7 */
        uint32_t res4                :1; /**< Reserved bit */
        uint32_t DMEIF7              :1; /**< Direct Mode Error Interrupt Flag for stream 7 */
        uint32_t TEIF7               :1; /**< Transfer Error Interrupt Flag for stream 7 */
        uint32_t HTIF7               :1; /**< Half Transfer Interrupt Flag for stream 7 */
        uint32_t TCIF7               :1; /**< Transfer Complete Interrupt Flag for stream 7 */
        uint32_t res5                :4; /**< Reserved bits */
    } DMA_HISR_t;


    /**
     * @brief DMA Low Interrupt Flag Clear Register (DMA_LIFCR_t)
     * 
     * This register is used to clear the interrupt flags for the lower half of the DMA streams.
     */
    struct {
        uint32_t CFEIF0               :1; /**< Clear FIFO Error Interrupt Flag for stream 0 */
        uint32_t res                  :1; /**< Reserved bit */
        uint32_t CDMEIF0              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 0 */
        uint32_t CTEIF0               :1; /**< Clear Transfer Error Interrupt Flag for stream 0 */
        uint32_t CHTIF0               :1; /**< Clear Half Transfer Interrupt Flag for stream 0 */
        uint32_t CTCIF0               :1; /**< Clear Transfer Complete Interrupt Flag for stream 0 */
        uint32_t CFEIF1               :1; /**< Clear FIFO Error Interrupt Flag for stream 1 */
        uint32_t res1                 :1; /**< Reserved bit */
        uint32_t CDMEIF1              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 1 */
        uint32_t CTEIF1               :1; /**< Clear Transfer Error Interrupt Flag for stream 1 */
        uint32_t CHTIF1               :1; /**< Clear Half Transfer Interrupt Flag for stream 1 */
        uint32_t CTCIF1               :1; /**< Clear Transfer Complete Interrupt Flag for stream 1 */
        uint32_t res2                 :4; /**< Reserved bits */
        uint32_t CFEIF2               :1; /**< Clear FIFO Error Interrupt Flag for stream 2 */
        uint32_t res3                 :1; /**< Reserved bit */
        uint32_t CDMEIF2              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 2 */
        uint32_t CTEIF2               :1; /**< Clear Transfer Error Interrupt Flag for stream 2 */
        uint32_t CHTIF2               :1; /**< Clear Half Transfer Interrupt Flag for stream 2 */
        uint32_t CTCIF2               :1; /**< Clear Transfer Complete Interrupt Flag for stream 2 */
        uint32_t CFEIF3               :1; /**< Clear FIFO Error Interrupt Flag for stream 3 */
        uint32_t res4                 :1; /**< Reserved bit */
        uint32_t CDMEIF3              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 3 */
        uint32_t CTEIF3               :1; /**< Clear Transfer Error Interrupt Flag for stream 3 */
        uint32_t CHTIF3               :1; /**< Clear Half Transfer Interrupt Flag for stream 3 */
        uint32_t CTCIF3               :1; /**< Clear Transfer Complete Interrupt Flag for stream 3 */
        uint32_t res5                 :4; /**< Reserved bits */
    } DMA_LIFCR_t;

    /**
     * @brief DMA High Interrupt Flag Clear Register (DMA_HIFCR_t)
     * 
     * This register is used to clear the interrupt flags for the higher half of the DMA streams.
     */
    struct {
        uint32_t CFEIF4               :1; /**< Clear FIFO Error Interrupt Flag for stream 4 */
        uint32_t res                  :1; /**< Reserved bit */
        uint32_t CDMEIF4              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 4 */
        uint32_t CTEIF4               :1; /**< Clear Transfer Error Interrupt Flag for stream 4 */
        uint32_t CHTIF4               :1; /**< Clear Half Transfer Interrupt Flag for stream 4 */
        uint32_t CTCIF4               :1; /**< Clear Transfer Complete Interrupt Flag for stream 4 */
        uint32_t CFEIF5               :1; /**< Clear FIFO Error Interrupt Flag for stream 5 */
        uint32_t res1                 :1; /**< Reserved bit */
        uint32_t CDMEIF5              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 5 */
        uint32_t CTEIF5               :1; /**< Clear Transfer Error Interrupt Flag for stream 5 */
        uint32_t CHTIF5               :1; /**< Clear Half Transfer Interrupt Flag for stream 5 */
        uint32_t CTCIF5               :1; /**< Clear Transfer Complete Interrupt Flag for stream 5 */
        uint32_t res2                 :4; /**< Reserved bits */
        uint32_t CFEIF6               :1; /**< Clear FIFO Error Interrupt Flag for stream 6 */
        uint32_t res3                 :1; /**< Reserved bit */
        uint32_t CDMEIF6              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 6 */
        uint32_t CTEIF6               :1; /**< Clear Transfer Error Interrupt Flag for stream 6 */
        uint32_t CHTIF6               :1; /**< Clear Half Transfer Interrupt Flag for stream 6 */
        uint32_t CTCIF6               :1; /**< Clear Transfer Complete Interrupt Flag for stream 6 */
        uint32_t CFEIF7               :1; /**< Clear FIFO Error Interrupt Flag for stream 7 */
        uint32_t res4                 :1; /**< Reserved bit */
        uint32_t CDMEIF7              :1; /**< Clear Direct Mode Error Interrupt Flag for stream 7 */
        uint32_t CTEIF7               :1; /**< Clear Transfer Error Interrupt Flag for stream 7 */
        uint32_t CHTIF7               :1; /**< Clear Half Transfer Interrupt Flag for stream 7 */
        uint32_t CTCIF7               :1; /**< Clear Transfer Complete Interrupt Flag for stream 7 */
        uint32_t res5                 :4; /**< Reserved bits */
    } DMA_HIFCR_t;


    /**
     * @brief DMA Stream Configuration Register (DMA_SxCR_t)
     * 
     * This register is used to configure the settings for a specific DMA stream.
     */
    struct {
        uint32_t EN                   :1; /**< Stream enable. */
        uint32_t DMEIE                :1; /**< Direct Mode Error Interrupt Enable. */
        uint32_t TEIE                 :1; /**< Transfer Error Interrupt Enable. */
        uint32_t HTIE                 :1; /**< Half Transfer Interrupt Enable. */
        uint32_t TCIE                 :1; /**< Transfer Complete Interrupt Enable. */
        uint32_t PFCTRL               :1; /**< Peripheral Flow Controller. */
        uint32_t DIR                  :2; /**< Data Transfer Direction. */
        uint32_t CIRC                 :1; /**< Circular Mode. */
        uint32_t PINC                 :1; /**< Peripheral Increment Mode. */
        uint32_t MINC                 :1; /**< Memory Increment Mode. */
        uint32_t PSIZE                :2; /**< Peripheral Data Size. */
        uint32_t MSIZE                :2; /**< Memory Data Size. */
        uint32_t PINCOS               :1; /**< Peripheral Increment Offset Size. */
        uint32_t PL                   :2; /**< Priority Level. */
        uint32_t DMB                  :1; /**< Double Buffer Mode. */
        uint32_t CT                   :1; /**< Current Target (for double-buffer mode). */
        uint32_t res                  :1; /**< Reserved bit. */
        uint32_t PBURST               :2; /**< Peripheral Burst Transfer Configuration. */
        uint32_t MBURST               :2; /**< Memory Burst Transfer Configuration. */
        uint32_t CHSEL                :3; /**< Channel Selection. */
        uint32_t res1                 :4; /**< Reserved bits. */
    } DMA_SxCR_t;

    /**
     * @brief DMA Stream Number of Data Register (DMA_SxNDTR_t)
     * 
     * This register specifies the number of data items to be transferred for the DMA stream.
     */
    struct {
        uint32_t NDT                  :16; /**< Number of Data items to Transfer. */
        uint32_t res                  :16; /**< Reserved bits. */
    } DMA_SxNDTR_t;

    /**
     * @brief DMA Stream Peripheral Address Register (DMA_SxPAR_t)
     * 
     * This register holds the address of the peripheral data register for the DMA stream.
     */
    struct {
        uint32_t PAR                  :32; /**< Peripheral Address. */
    } DMA_SxPAR_t;

    /**
     * @brief DMA Stream Memory 0 Address Register (DMA_SxM0AR_t)
     * 
     * This register holds the address of the memory buffer for the DMA stream.
     */
    struct {
        uint32_t M0A                  :32; /**< Memory 0 Address. */
    } DMA_SxM0AR_t;

    /**
     * @brief DMA Stream Memory 1 Address Register (DMA_SxM1AR_t)
     * 
     * This register holds the address of the second memory buffer for the DMA stream in double-buffer mode.
     */
    struct {
        uint32_t M1A                  :32; /**< Memory 1 Address. */
    } DMA_SxM1AR_t;

    /**
     * @brief DMA Stream FIFO Control Register (DMA_SxFCR_t)
     * 
     * This register controls the FIFO threshold, direct mode, and other FIFO-related settings for the DMA stream.
     */
    struct {
        uint32_t FTH                  :2; /**< FIFO Threshold Selection. */
        uint32_t DMDIS                :1; /**< Direct Mode Disable. */
        uint32_t FS                   :3; /**< FIFO Status. */
        uint32_t res                  :1; /**< Reserved bit. */
        uint32_t FEIE                 :1; /**< FIFO Error Interrupt Enable. */
        uint32_t res1                 :24; /**< Reserved bits. */
    } DMA_SxFCR_t;

} DMA_RegDef_t;







/* 
****************** EXTI register definition structures******************
*/

/**
 * @brief EXTI Register Definition structure.
 * 
 * Defines the structure for External Interrupt (EXTI) registers, which handle
 * the configuration of external interrupt and event masking.
 */
typedef struct{

      /**
     * @brief Interrupt mask register (EXTI_IMR).
     * 
     * Enables or disables interrupt requests from each pin.
     */
    struct                     
    {
        uint32_t MR0       :1; /**< Interrupt mask for pin 0 */
        uint32_t MR1       :1; /**< Interrupt mask for pin 1 */
        uint32_t MR2       :1; /**< Interrupt mask for pin 2 */
        uint32_t MR3       :1; /**< Interrupt mask for pin 3 */
        uint32_t MR4       :1; /**< Interrupt mask for pin 4 */
        uint32_t MR5       :1; /**< Interrupt mask for pin 5 */
        uint32_t MR6       :1; /**< Interrupt mask for pin 6 */
        uint32_t MR7       :1; /**< Interrupt mask for pin 7 */
        uint32_t MR8       :1; /**< Interrupt mask for pin 8 */
        uint32_t MR9       :1; /**< Interrupt mask for pin 9 */
        uint32_t MR10      :1; /**< Interrupt mask for pin 10 */
        uint32_t MR11      :1; /**< Interrupt mask for pin 11 */
        uint32_t MR12      :1; /**< Interrupt mask for pin 12 */
        uint32_t MR13      :1; /**< Interrupt mask for pin 13 */
        uint32_t MR14      :1; /**< Interrupt mask for pin 14 */
        uint32_t MR15      :1; /**< Interrupt mask for pin 15 */
        uint32_t MR16      :1; /**< Interrupt mask for pin 16 */
        uint32_t MR17      :1; /**< Interrupt mask for pin 17 */
        uint32_t MR18      :1; /**< Interrupt mask for pin 18 */
        uint32_t res       :2; /**< Reserved bits */
        uint32_t MR21      :1; /**< Interrupt mask pin 21 */
        uint32_t MR22      :1; /**< Interrupt mask pin 22 */
        uint32_t res1      :9; /**< Reserved bits */
    }EXTI_IMR_t;


    /**
     * @brief Event mask register (EXTI_EMR).
     * 
     * Enables or disables event requests from each pin.
     */
    struct                     
    {
        uint32_t MR0       :1; /**< Event mask for pin 0 */
        uint32_t MR1       :1; /**< Event mask for pin 1 */
        uint32_t MR2       :1; /**< Event mask for pin 2 */
        uint32_t MR3       :1; /**< Event mask for pin 3 */
        uint32_t MR4       :1; /**< Event mask for pin 4 */
        uint32_t MR5       :1; /**< Event mask for pin 5 */
        uint32_t MR6       :1; /**< Event mask for pin 6 */
        uint32_t MR7       :1; /**< Event mask for pin 7 */
        uint32_t MR8       :1; /**< Event mask for pin 8 */
        uint32_t MR9       :1; /**< Event mask for pin 9 */
        uint32_t MR10      :1; /**< Event mask for pin 10 */
        uint32_t MR11      :1; /**< Event mask for pin 11 */
        uint32_t MR12      :1; /**< Event mask for pin 12 */
        uint32_t MR13      :1; /**< Event mask for pin 13 */
        uint32_t MR14      :1; /**< Event mask for pin 14 */
        uint32_t MR15      :1; /**< Event mask for pin 15 */
        uint32_t MR16      :1; /**< Event mask for pin 16 */
        uint32_t MR17      :1; /**< Event mask for pin 17 */
        uint32_t MR18      :1; /**< Event mask for pin 18 */
        uint32_t res       :2; /**< Reserved bits */
        uint32_t MR21      :1; /**< Event mask pin 21 */
        uint32_t MR22      :1; /**< Event mask pin 22 */
        uint32_t res1      :9; /**< Reserved bits */
    }EXTI_EMR_t;


    /**
     * @brief Rising trigger selection register (EXTI_RTSR).
     * 
     * Configures rising-edge trigger for each pin.
     */
    struct                     
    {
       uint32_t TR0       :1; /**< Rising trigger for pin 0 */
        uint32_t TR1       :1; /**< Rising trigger for pin 1 */
        uint32_t TR2       :1; /**< Rising trigger for pin 2 */
        uint32_t TR3       :1; /**< Rising trigger for pin 3 */
        uint32_t TR4       :1; /**< Rising trigger for pin 4 */
        uint32_t TR5       :1; /**< Rising trigger for pin 5 */
        uint32_t TR6       :1; /**< Rising trigger for pin 6 */
        uint32_t TR7       :1; /**< Rising trigger for pin 7 */
        uint32_t TR8       :1; /**< Rising trigger for pin 8 */
        uint32_t TR9       :1; /**< Rising trigger for pin 9 */
        uint32_t TR10      :1; /**< Rising trigger for pin 10 */
        uint32_t TR11      :1; /**< Rising trigger for pin 11 */
        uint32_t TR12      :1; /**< Rising trigger for pin 12 */
        uint32_t TR13      :1; /**< Rising trigger for pin 13 */
        uint32_t TR14      :1; /**< Rising trigger for pin 14 */
        uint32_t TR15      :1; /**< Rising trigger for pin 15 */
        uint32_t TR16      :1; /**< Rising trigger for pin 16 */
        uint32_t TR17      :1; /**< Rising trigger for pin 17 */
        uint32_t TR18      :1; /**< Rising trigger for pin 18 */
        uint32_t res       :2; /**< Reserved bits */
        uint32_t TR21      :1; /**< Rising trigger pin 21 */
        uint32_t TR22      :1; /**< Rising trigger pin 22 */
        uint32_t res1      :9; /**< Reserved bits */
    }EXTI_RTSR_t;


    /**
     * @brief Falling trigger selection register (EXTI_FTSR).
     * 
     * Configures falling-edge trigger for each pin.
     */
    struct                     
    {
        uint32_t TR0       :1; /**< Falling trigger for pin 0 */
        uint32_t TR1       :1; /**< Falling trigger for pin 1 */
        uint32_t TR2       :1; /**< Falling trigger for pin 2 */
        uint32_t TR3       :1; /**< Falling trigger for pin 3 */
        uint32_t TR4       :1; /**< Falling trigger for pin 4 */
        uint32_t TR5       :1; /**< Falling trigger for pin 5 */
        uint32_t TR6       :1; /**< Falling trigger for pin 6 */
        uint32_t TR7       :1; /**< Falling trigger for pin 7 */
        uint32_t TR8       :1; /**< Falling trigger for pin 8 */
        uint32_t TR9       :1; /**< Falling trigger for pin 9 */
        uint32_t TR10      :1; /**< Falling trigger for pin 10 */
        uint32_t TR11      :1; /**< Falling trigger for pin 11 */
        uint32_t TR12      :1; /**< Falling trigger for pin 12 */
        uint32_t TR13      :1; /**< Falling trigger for pin 13 */
        uint32_t TR14      :1; /**< Falling trigger for pin 14 */
        uint32_t TR15      :1; /**< Falling trigger for pin 15 */
        uint32_t TR16      :1; /**< Falling trigger for pin 16 */
        uint32_t TR17      :1; /**< Falling trigger for pin 17 */
        uint32_t TR18      :1; /**< Falling trigger for pin 18 */
        uint32_t res       :2; /**< Reserved bits */
        uint32_t TR21      :1; /**< Falling trigger for pin 21 */
        uint32_t TR22      :1; /**< Falling trigger for pin 22 */
        uint32_t res1      :9; /**< Reserved bits */
    }EXTI_FTSR_t;


    /**
     * @brief Software interrupt event register (EXTI_SWIER).
     * 
     * Allows software to trigger interrupt events on each pin.
     */
    struct                
    {
        uint32_t SWIER0    :1; /**< Software interrupt event for pin 0 */
        uint32_t SWIER1    :1; /**< Software interrupt event for pin 1 */
        uint32_t SWIER2    :1; /**< Software interrupt event for pin 2 */
        uint32_t SWIER3    :1; /**< Software interrupt event for pin 3 */
        uint32_t SWIER4    :1; /**< Software interrupt event for pin 4 */
        uint32_t SWIER5    :1; /**< Software interrupt event for pin 5 */
        uint32_t SWIER6    :1; /**< Software interrupt event for pin 6 */
        uint32_t SWIER7    :1; /**< Software interrupt event for pin 7 */
        uint32_t SWIER8    :1; /**< Software interrupt event for pin 8 */
        uint32_t SWIER9    :1; /**< Software interrupt event for pin 9 */
        uint32_t SWIER10   :1; /**< Software interrupt event for pin 10 */
        uint32_t SWIER11   :1; /**< Software interrupt event for pin 11 */
        uint32_t SWIER12   :1; /**< Software interrupt event for pin 12 */
        uint32_t SWIER13   :1; /**< Software interrupt event for pin 13 */
        uint32_t SWIER14   :1; /**< Software interrupt event for pin 14 */
        uint32_t SWIER15   :1; /**< Software interrupt event for pin 15 */
        uint32_t SWIER16   :1; /**< Software interrupt event for pin 16 */
        uint32_t SWIER17   :1; /**< Software interrupt event for pin 17 */
        uint32_t SWIER18   :1; /**< Software interrupt event for pin 18 */
        uint32_t res       :2; /**< Reserved bits */
        uint32_t SWIER21   :1; /**< Software interrupt event for pin 21 */
        uint32_t SWIER22   :1; /**< Software interrupt event for pin 22 */
        uint32_t res1      :9; /**< Reserved bits */
    }EXTI_SWIER_t;

    /**
     * @brief Pending register (EXTI_PR).
     * 
     * Indicates pending interrupt requests for each pin.
     */
    struct                
    {
        uint32_t PR0       :1; /**< Pending interrupt for pin 0 */
        uint32_t PR1       :1; /**< Pending interrupt for pin 1 */
        uint32_t PR2       :1; /**< Pending interrupt for pin 2 */
        uint32_t PR3       :1; /**< Pending interrupt for pin 3 */
        uint32_t PR4       :1; /**< Pending interrupt for pin 4 */
        uint32_t PR5       :1; /**< Pending interrupt for pin 5 */
        uint32_t PR6       :1; /**< Pending interrupt for pin 6 */
        uint32_t PR7       :1; /**< Pending interrupt for pin 7 */
        uint32_t PR8       :1; /**< Pending interrupt for pin 8 */
        uint32_t PR9       :1; /**< Pending interrupt for pin 9 */
        uint32_t PR10      :1; /**< Pending interrupt for pin 10 */
        uint32_t PR11      :1; /**< Pending interrupt for pin 11 */
        uint32_t PR12      :1; /**< Pending interrupt for pin 12 */
        uint32_t PR13      :1; /**< Pending interrupt for pin 13 */
        uint32_t PR14      :1; /**< Pending interrupt for pin 14 */
        uint32_t PR15      :1; /**< Pending interrupt for pin 15 */
        uint32_t PR16      :1; /**< Pending interrupt for pin 16 */
        uint32_t PR17      :1; /**< Pending interrupt for pin 17 */
        uint32_t PR18      :1; /**< Pending interrupt for pin 18 */
        uint32_t res       :2;
        uint32_t PR21      :1; /**< Pending interrupt for pin 21 */
        uint32_t PR22      :1; /**< Pending interrupt for pin 22 */
        uint32_t res1      :9;
    }EXTI_PR_t;
}EXTI_RegDef_t;







/* 
****************** SPI register definition structures******************
*/

/**
 * @brief SPI Register Definition structure.
 * 
 * Contains register configurations for the Serial Peripheral Interface (SPI).
 */
volatile typedef struct{

    /**
     * @brief SPI control register 1 (SPI_CR1).
     * 
     * Configures the SPI mode, clock polarity, phase, master/slave selection, 
     * baud rate, data format, and CRC settings.
     */
    struct       
    {
        uint16_t CPHA               :1; /**< Clock Phase. */
        uint16_t CPOL               :1; /**< Clock Polarity. */
        uint16_t MSTR               :1; /**< Master Selection. */
        uint16_t BR                 :3; /**< Baud Rate Control. */
        uint16_t SPE                :1; /**< SPI Enable. */
        uint16_t LSBFIRST           :1; /**< Frame Format. */
        uint16_t SSI                :1; /**< Internal Slave Select. */
        uint16_t SSM                :1; /**< Software Slave Management. */
        uint16_t RXONLY             :1; /**< Receive Only Mode. */
        uint16_t DFF                :1; /**< Data Frame Format. */
        uint16_t CRCNEXT            :1; /**< Transmit CRC Next. */
        uint16_t CRCEN              :1; /**< Hardware CRC Calculation Enable. */
        uint16_t BIDIOE             :1; /**< Output Enable in Bidirectional Mode. */
        uint16_t BIDIMODE           :1; /**< Bidirectional Data Mode Enable. */
    } SPI_CR1_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED0_t;

    /**
     * @brief SPI control register 2 (SPI_CR2).
     * 
     * Configures DMA, frame format, and interrupt options for SPI.
     */
    struct       
    {
        uint16_t RXDMAEN            :1; /**< Rx Buffer DMA Enable. */
        uint16_t TXDMAEN            :1; /**< Tx Buffer DMA Enable. */
        uint16_t SSOE               :1; /**< SS Output Enable. */
        uint16_t res                :1; /**< Reserved. */
        uint16_t FRF                :1; /**< Frame Format. */
        uint16_t ERRIE              :1; /**< Error Interrupt Enable. */
        uint16_t RXNEIE             :1; /**< RX Buffer Not Empty Interrupt Enable. */
        uint16_t TXEIE              :1; /**< TX Buffer Empty Interrupt Enable. */
        uint16_t res1               :8; /**< Reserved. */
    } SPI_CR2_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED1_t;

    /**
     * @brief SPI status register (SPI_SR).
     * 
     * Provides status flags for SPI, including RX, TX, and error indicators.
     */
    struct          
    {
        uint16_t RXNE               :1; /**< Receive Buffer Not Empty. */
        uint16_t TXE                :1; /**< Transmit Buffer Empty. */
        uint16_t CHSIDE             :1; /**< Channel Side. */
        uint16_t UDR                :1; /**< Underrun Flag. */
        uint16_t CRCERR             :1; /**< CRC Error Flag. */
        uint16_t MODF               :1; /**< Mode Fault. */
        uint16_t OVF                :1; /**< Overrun Flag. */
        uint16_t BSY                :1; /**< Busy Flag. */
        uint16_t res                :7; /**< Reserved. */
    } SPI_SR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED2_t;

    /**
     * @brief SPI data register (SPI_DR).
     * 
     * Holds the data to be transmitted or received via SPI.
     */
    struct       
    {
        uint16_t DR                 :16; /**< Data Register. */
    } SPI_DR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED3_t;

    /**
     * @brief SPI CRC polynomial register (SPI_CRCPR).
     * 
     * Holds the polynomial for the CRC calculation used in SPI.
     */
    struct       
    {
        uint16_t CRCPOLY            :16; /**< CRC Polynomial Register. */
    } SPI_CRCPR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED4_t;

    /**
     * @brief SPI RX CRC register (SPI_RXCRCR).
     * 
     * Holds the calculated CRC value for received data.
     */
    struct       
    {
        uint16_t RXCRC              :16; /**< Receive CRC Register. */
    } SPI_RXCRCR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED5_t;

    /**
     * @brief SPI TX CRC register (SPI_TXCRCR).
     * 
     * Holds the calculated CRC value for transmitted data.
     */
    struct       
    {
        uint16_t TXCRC              :16; /**< Transmit CRC Register. */
    } SPI_TXCRCR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED6_t;



    /**
     * @brief SPI_I2S configuration register (SPI_I2SCFGR).
     * 
     * Configures the I2S mode settings when I2S is used instead of SPI, including channel length,
     * data length, clock polarity, and I2S standard selection.
     */
    struct       
    {
        uint16_t CHLEN              :1; /**< Channel Length (0: 16-bit, 1: 32-bit). */
        uint16_t DATLEN             :2; /**< Data Length (0: 16-bit, 1: 24-bit, 2: 32-bit). */
        uint16_t CKPOL              :1; /**< Clock Polarity. */
        uint16_t I2SSTD             :2; /**< I2S Standard Selection. */
        uint16_t res                :1; /**< Reserved. */
        uint16_t PCMSYNC            :1; /**< PCM Frame Synchronization. */
        uint16_t I2SCFG             :2; /**< I2S Configuration Mode. */
        uint16_t I2SE               :1; /**< I2S Enable. */
        uint16_t I2SMOD             :1; /**< I2S Mode Selection (0: SPI, 1: I2S). */
        uint16_t res1               :4; /**< Reserved. */
    } SPI_I2SCFGR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED7_t;

    /**
     * @brief SPI_I2S prescaler register (SPI_I2SPR).
     * 
     * Configures the prescaler and clock odd setting for the I2S clock.
     */
    struct       
    {
        uint16_t I2SDIV             :8; /**< I2S Linear Prescaler. */
        uint16_t ODD                :1; /**< Odd Factor for the Prescaler. */
        uint16_t MCKOE              :1; /**< Master Clock Output Enable. */
        uint16_t res                :6; /**< Reserved. */
    } SPI_I2SPR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED8_t;
}SPI_RegDef_t;




/* 
****************** I2C register definition structures******************
*/

/**
 * @brief I2C Register Definition Structure.
 * 
 * This structure defines the configuration and control registers for the I2C peripheral.
 */
volatile typedef struct {

    /**
     * @brief I2C control register 1 (I2C_CR1).
     * 
     * This register configures the I2C mode, enables SMBus, ACK, and enables various 
     * options like software reset and general call recognition.
     */
    struct       
    {
        uint16_t PE                 :1; /**< Peripheral Enable. */
        uint16_t SMBUS              :1; /**< SMBus Mode. */
        uint16_t res                :1; /**< Reserved. */
        uint16_t SMBTYPE            :1; /**< SMBus Type. */
        uint16_t ENARP              :1; /**< ARP Enable. */
        uint16_t ENPEC              :1; /**< PEC Enable. */
        uint16_t ENGC               :1; /**< General Call Enable. */
        uint16_t NOSTRECTH          :1; /**< Clock Stretching Disable. */
        uint16_t START              :1; /**< Start Generation. */
        uint16_t STOP               :1; /**< Stop Generation. */
        uint16_t ACK                :1; /**< Acknowledge Enable. */
        uint16_t POS                :1; /**< Acknowledge/PEC Position. */
        uint16_t PEC                :1; /**< Packet Error Checking. */
        uint16_t ALERT              :1; /**< SMBus Alert. */
        uint16_t res1               :1; /**< Reserved. */
        uint16_t SWRST              :1; /**< Software Reset. */
    } I2C_CR1_t;
   
    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED0_t;
   
    /**
     * @brief I2C control register 2 (I2C_CR2).
     * 
     * This register configures frequency, error, event, and buffer interrupts, as well as 
     * DMA and LAST data handling.
     */
    struct       
    {
        uint16_t FREQ               :6; /**< Peripheral Clock Frequency. */
        uint16_t res                :2; /**< Reserved. */
        uint16_t ITERREN            :1; /**< Error Interrupt Enable. */
        uint16_t ITEVTEN            :1; /**< Event Interrupt Enable. */
        uint16_t ITBUFEN            :1; /**< Buffer Interrupt Enable. */
        uint16_t DMAEN              :1; /**< DMA Requests Enable. */
        uint16_t LAST               :1; /**< DMA Last Transfer. */
        uint16_t res1               :3; /**< Reserved. */
    } I2C_CR2_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED1_t;


    /**
     * @brief I2C own address register 1 (I2C_OAR1).
     * 
     * This register configures the 7-bit or 10-bit addressing mode and stores the device's own address.
     */
    struct       
    {
        uint16_t ADD0               :1; /**< Interface Address Bit 0. */
        uint16_t ADD                :7; /**< Interface Address Bits 7:1 (7-bit mode) or Bits 9:3 (10-bit mode). */
        uint16_t ADD10              :2; /**< Interface Address Bits 9:8 in 10-bit addressing mode. */
        uint16_t res                :4; /**< Reserved. */
        uint16_t res14              :1; /**< Reserved. Should always be kept at 1 by software. */
        uint16_t ADDMODE            :1; /**< Addressing Mode (0: 7-bit, 1: 10-bit). */
    } I2C_OAR1_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED2_t;

    /**
     * @brief I2C own address register 2 (I2C_OAR2).
     * 
     * This register configures the second own address for dual-address mode.
     */
    struct       
    {
        uint16_t ENDUAL             :1; /**< Dual Addressing Mode Enable. */
        uint16_t ADD2               :7; /**< Interface Second Address in 7-bit mode. */
        uint16_t res                :8; /**< Reserved. */
    } I2C_OAR2_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED3_t;

    /**
     * @brief I2C data register (I2C_DR).
     * 
     * This register holds the data byte to be transmitted or received.
     */
    struct       
    {
        uint16_t DR                 :8; /**< Data Register. */
        uint16_t res                :8; /**< Reserved. */
    } I2C_DR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED4_t;

    /**
     * @brief I2C status register 1 (I2C_SR1).
     * 
     * This register provides status flags for the communication and error handling.
     */
    struct       
    {
        uint16_t SB                 :1; /**< Start Bit (Master Mode). */
        uint16_t ADDR               :1; /**< Address Sent (Master) / Matched (Slave). */
        uint16_t BTF                :1; /**< Byte Transfer Finished. */
        uint16_t ADD10              :1; /**< 10-bit Header Sent (Master Mode). */
        uint16_t STOPF              :1; /**< Stop Detection (Slave Mode). */
        uint16_t res                :1; /**< Reserved. */
        uint16_t RxNE               :1; /**< Data Register Not Empty (Receive). */
        uint16_t TxE                :1; /**< Data Register Empty (Transmit). */
        uint16_t BERR               :1; /**< Bus Error. */
        uint16_t ARLO               :1; /**< Arbitration Lost (Master Mode). */
        uint16_t AF                 :1; /**< Acknowledge Failure. */
        uint16_t OVR                :1; /**< Overrun/Underrun. */
        uint16_t PECERR             :1; /**< PEC Error in Reception. */
        uint16_t res1               :1; /**< Reserved. */
        uint16_t TIMEOUT            :1; /**< Timeout or Tlow Error. */
        uint16_t SMBALERT           :1; /**< SMBus Alert. */
    } I2C_SR1_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED5_t;


    /**
     * @brief I2C status register 2 (I2C_SR2).
     * 
     * This register provides additional status flags, including the master/slave mode, busy flag, 
     * and general call status.
     */
    struct       
    {
        uint16_t MSL                :1; /**< Master/Slave Mode. */
        uint16_t BUSY               :1; /**< Bus Busy. */
        uint16_t TRA                :1; /**< Transmitter/Receiver Mode. */
        uint16_t res                :1; /**< Reserved. */
        uint16_t GENCALL            :1; /**< General Call Address (Slave Mode). */
        uint16_t SMBDEFAULT         :1; /**< SMBus Device Default Address. */
        uint16_t SMBHOST            :1; /**< SMBus Host Header. */
        uint16_t DUALF              :1; /**< Dual Address Flag. */
        uint16_t PEC                :8; /**< Packet Error Checking Register. */
    } I2C_SR2_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED6_t;


    /**
     * @brief I2C clock control register (I2C_CCR).
     * 
     * This register configures the clock control and duty cycle settings for the I2C peripheral.
     */
    struct       
    {
        uint16_t CCR                :12; /**< Clock Control. */
        uint16_t res                :2; /**< Reserved. */
        uint16_t DUTY               :1; /**< Fast Mode Duty Cycle. */
        uint16_t FS                 :1; /**< I2C Master Mode Selection (0: Standard, 1: Fast). */
    } I2C_CCR_t;


    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED7_t;


    /**
     * @brief I2C TRISE register (I2C_TRISE).
     * 
     * This register configures the maximum rise time in Fast/Standard mode.
     */
    struct       
    {
        uint16_t TRISE              :6; /**< Maximum Rise Time. */
        uint16_t res                :10; /**< Reserved. */
    } I2C_TRISE_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED8_t;

    /**
     * @brief I2C filter register (I2C_FLTR).
     * 
     * This register configures the digital noise filter and analog noise filter.
     */
    struct       
    {
        uint16_t DNF                :4; /**< Digital Noise Filter. */
        uint16_t ANOFF              :1; /**< Analog Noise Filter OFF. */
        uint16_t res                :11; /**< Reserved. */
    } I2C_FLTR_t;

    /**
     * @brief RESERVED address.
     * 
     * This address is reserved and should not be used.
     */
    struct 
    {
        uint16_t res; /**< Reserved. */
    } RESERVED9_t;

} I2C_RegDef_t;








/**
 * @brief USART Register Definition Structure
 * 
 * This structure represents the various registers of a USART peripheral.
 * It includes the status register, data register, and control register 1,
 * each with their respective bit fields.
 */
volatile typedef struct {

    /**
     * @brief Status Register (USART_SR_t)
     * 
     * This register holds the status flags for the USART peripheral.
     */
    struct {
        uint32_t PE                 :1; /**< Parity Error. */
        uint32_t FE                 :1; /**< Framing Error. */
        uint32_t NF                 :1; /**< Noise Flag. */
        uint32_t ORE                :1; /**< Overrun Error. */
        uint32_t IDLE               :1; /**< Idle Line Detected. */
        uint32_t RXNE               :1; /**< Read Data Register Not Empty. */
        uint32_t TC                 :1; /**< Transmission Complete. */
        uint32_t TXE                :1; /**< Transmit Data Register Empty. */
        uint32_t LBD                :1; /**< LIN Break Detection. */
        uint32_t CTS                :1; /**< Clear to Send Flag. */
        uint32_t res                :22; /**< Reserved bits. */
    } USART_SR_t;

    /**
     * @brief Data Register (USART_DR_t)
     * 
     * This register is used to read or write data to the USART peripheral.
     */
    struct {
        uint32_t DR                 :9; /**< Data Register. */       
        uint32_t res                :23; /**< Reserved bits. */
    } USART_DR_t;

    struct{
        uint32_t DIV_Fraction       :4; /**< Fraction of USARTDIV. */
        uint32_t DIV_Mantissa       :12; /**< Mantissa of USARTDIV. */
        uint32_t res                :16; /**< Reserved bits. */
    }USART_BRR_t;
    

    /**
     * @brief Control Register 1 (USART_CR1_t)
     * 
     * This register contains control bits for configuring the USART peripheral.
     */
    struct {
        uint32_t SBK                :1; /**< Send Break. */
        uint32_t RWU                :1; /**< Receiver Wakeup. */
        uint32_t RE                 :1; /**< Receiver Enable. */
        uint32_t TE                 :1; /**< Transmitter Enable. */
        uint32_t IDLEIE             :1; /**< IDLE Interrupt Enable. */
        uint32_t RXNEIE             :1; /**< RXNE Interrupt Enable. */
        uint32_t TCIE               :1; /**< Transmission Complete Interrupt Enable. */
        uint32_t TXEIE              :1; /**< TXE Interrupt Enable. */
        uint32_t PEIE               :1; /**< PE Interrupt Enable. */
        uint32_t PS                 :1; /**< Parity Selection. */
        uint32_t PCE                :1; /**< Parity Control Enable. */
        uint32_t WAKE               :1; /**< Wakeup method. */
        uint32_t M                  :1; /**< Word Length. */
        uint32_t UE                 :1; /**< USART Enable. */
        uint32_t res                :1; /**< Reserved bit. */
        uint32_t OVER8              :1; /**< Oversampling mode. */
        uint32_t res1               :16; /**< Reserved bits. */
    } USART_CR1_t;


/**
 * @brief USART Control Register 2 (USART_CR2_t)
 * 
 * This register contains additional control bits for the USART peripheral.
 */
struct {
    uint32_t ADD                :4;  /**< Address of the USART node. */
    uint32_t res                :1;  /**< Reserved bit. */
    uint32_t LBDL               :1;  /**< LIN Break Detection Length. */
    uint32_t LBDIE              :1;  /**< LIN Break Detection Interrupt Enable. */
    uint32_t res1               :1;  /**< Reserved bit. */
    uint32_t LBCL               :1;  /**< Last Bit Clock pulse. */
    uint32_t CPHA               :1;  /**< Clock Phase. */
    uint32_t CPOL               :1;  /**< Clock Polarity. */
    uint32_t STOP               :2;  /**< STOP bits. */
    uint32_t LINEN              :1;  /**< LIN Mode Enable. */
    uint32_t res2               :17; /**< Reserved bits. */
} USART_CR2_t;

/**
 * @brief USART Control Register 3 (USART_CR3_t)
 * 
 * This register contains additional control bits for the USART peripheral.
 */
struct {
    uint32_t EIE                :1;  /**< Error Interrupt Enable. */
    uint32_t IREN               :1;  /**< IrDA Mode Enable. */
    uint32_t HDSEL              :1;  /**< Half-Duplex Selection. */
    uint32_t NACK               :1;  /**< Smartcard NACK Enable. */
    uint32_t SCEN               :1;  /**< Smartcard Mode Enable. */
    uint32_t DMAR               :1;  /**< DMA Enable Receiver. */
    uint32_t DMAT               :1;  /**< DMA Enable Transmitter. */
    uint32_t RTSE               :1;  /**< RTS Enable. */
    uint32_t CTSE               :1;  /**< CTS Enable. */
    uint32_t CTSIE              :1;  /**< CTS Interrupt Enable. */
    uint32_t ONEBIT             :1;  /**< One Sample Bit Method Enable. */
    uint32_t res2               :20; /**< Reserved bits. */
} USART_CR3_t;

/**
 * @brief USART Guard Time and Prescaler Register (USART_GTPR_t)
 * 
 * This register is used to set the prescaler value and guard time for the USART.
 */
struct {
    uint32_t PSC                :8;  /**< Prescaler value. */
    uint32_t GT                 :8;  /**< Guard time value. */
    uint32_t res2               :16; /**< Reserved bits. */
} USART_GTPR_t;

     
} USART_RegDef_t;





/*
**************** Peripheral definitions (peripheral addresses typecasted to structures)
*/



/**
 * @defgroup Peripheral_Base_Addresses Peripheral Base Addresses
 * @brief Base addresses of peripherals mapped in memory for STM32F401RE.
 * @{
 */

#define GPIOA               ((GPIO_RegDef_t*) GPIOA_BASEADDR)  /**< GPIO Port A base address */
#define GPIOB               ((GPIO_RegDef_t*) GPIOB_BASEADDR)  /**< GPIO Port B base address */
#define GPIOC               ((GPIO_RegDef_t*) GPIOC_BASEADDR)  /**< GPIO Port C base address */
#define GPIOD               ((GPIO_RegDef_t*) GPIOD_BASEADDR)  /**< GPIO Port D base address */
#define GPIOE               ((GPIO_RegDef_t*) GPIOE_BASEADDR)  /**< GPIO Port E base address */
#define GPIOH               ((GPIO_RegDef_t*) GPIOH_BASEADDR)  /**< GPIO Port H base address */

#define RCC                 ((RCC_RegDef_t*) RCC_BASEADDR)     /**< RCC base address */
#define EXTI                ((EXTI_RegDef_t*) EXTI_BASEADDR)   /**< EXTI base address */
#define SYSCFG              ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR) /**< SYSCFG base address */

#define SPI1                ((SPI_RegDef_t*) SPI1_BASEADDR)    /**< SPI1 base address */
#define SPI2                ((SPI_RegDef_t*) SPI2_BASEADDR)    /**< SPI2 base address */
#define SPI3                ((SPI_RegDef_t*) SPI3_BASEADDR)    /**< SPI3 base address */
#define SPI4                ((SPI_RegDef_t*) SPI4_BASEADDR)    /**< SPI4 base address */

#define USART1              ((USART_RegDef_t*) USART1_BASEADDR) /**< USART1 base address */
#define USART2              ((USART_RegDef_t*) USART2_BASEADDR) /**< USART2 base address */
#define USART6              ((USART_RegDef_t*) USART6_BASEADDR) /**< USART6 base address */

#define I2C1                ((I2C_RegDef_t*) I2C1_BASEADDR)    /**< I2C1 base address */
#define I2C2                ((I2C_RegDef_t*) I2C2_BASEADDR)    /**< I2C2 base address */
#define I2C3                ((I2C_RegDef_t*) I2C3_BASEADDR)    /**< I2C3 base address */

/** @} */

/**
 * @defgroup Peripheral_Clock_Enable Peripheral Clock Enable Macros
 * @brief Enables/disables the clock for the given peripheral.
 * @{
 */

/* Clock enable macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOAEN=1) /**< Enable GPIOA clock */
#define GPIOB_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOBEN=1) /**< Enable GPIOB clock */
#define GPIOC_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOCEN=1) /**< Enable GPIOC clock */
#define GPIOD_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIODEN=1) /**< Enable GPIOD clock */
#define GPIOE_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOEEN=1) /**< Enable GPIOE clock */
#define GPIOH_PCLK_EN()                  (RCC->RCC_AHB1ENR_t.GPIOHEN=1) /**< Enable GPIOH clock */

/* Clock enable macros for I2Cx peripherals */
#define I2C1_PCLK_EN()                   (RCC->RCC_APB1ENR_t.I2C1EN=1) /**< Enable I2C1 clock */
#define I2C2_PCLK_EN()                   (RCC->RCC_APB1ENR_t.I2C2EN=1) /**< Enable I2C2 clock */
#define I2C3_PCLK_EN()                   (RCC->RCC_APB1ENR_t.I2C3EN=1) /**< Enable I2C3 clock */

/* Clock enable macros for SPIx peripherals */
#define SPI1_PCLK_EN()                   (RCC->RCC_APB2ENR_t.SPI1EN=1) /**< Enable SPI1 clock */
#define SPI2_PCLK_EN()                   (RCC->RCC_APB1ENR_t.SPI2EN=1) /**< Enable SPI2 clock */
#define SPI3_PCLK_EN()                   (RCC->RCC_APB1ENR_t.SPI3EN=1) /**< Enable SPI3 clock */
#define SPI4_PCLK_EN()                   (RCC->RCC_APB2ENR_t.SPI4EN=1) /**< Enable SPI4 clock */

/* Clock enable macros for USARTx peripherals */
#define USART1_PCLK_EN()                 (RCC->RCC_APB2ENR_t.USART1EN=1) /**< Enable USART1 clock */
#define USART2_PCLK_EN()                 (RCC->RCC_APB1ENR_t.USART2EN=1) /**< Enable USART2 clock */
#define USART6_PCLK_EN()                 (RCC->RCC_APB2ENR_t.USART6EN=1) /**< Enable USART6 clock */

/* Clock enable macros for SYSCFG peripheral */
#define SYSCFG_PCLK_EN()                 (RCC->RCC_APB2ENR_t.SYSCFGEN=1) /**< Enable SYSCFG clock */

/** @} */

/**
 * @defgroup Peripheral_Clock_Disable Peripheral Clock Disable Macros
 * @brief Disables the clock for the given peripheral.
 * @{
 */

#define GPIOA_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOAEN=0) /**< Disable GPIOA clock */
#define GPIOB_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOBEN=0) /**< Disable GPIOB clock */
#define GPIOC_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOCEN=0) /**< Disable GPIOC clock */
#define GPIOD_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIODEN=0) /**< Disable GPIOD clock */
#define GPIOE_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOEEN=0) /**< Disable GPIOE clock */
#define GPIOH_PCLK_DI()                  (RCC->RCC_AHB1ENR_t.GPIOHEN=0) /**< Disable GPIOH clock */

#define I2C1_PCLK_DI()                   (RCC->RCC_APB1ENR_t.I2C1EN=0) /**< Disable I2C1 clock */
#define I2C2_PCLK_DI()                   (RCC->RCC_APB1ENR_t.I2C2EN=0) /**< Disable I2C2 clock */
#define I2C3_PCLK_DI()                   (RCC->RCC_APB1ENR_t.I2C3EN=0) /**< Disable I2C3 clock */

#define SPI1_PCLK_DI()                   (RCC->RCC_APB2ENR_t.SPI1EN=0) /**< Disable SPI1 clock */
#define SPI2_PCLK_DI()                   (RCC->RCC_APB1ENR_t.SPI2EN=0) /**< Disable SPI2 clock */
#define SPI3_PCLK_DI()                   (RCC->RCC_APB1ENR_t.SPI3EN=0) /**< Disable SPI3 clock */
#define SPI4_PCLK_DI()                   (RCC->RCC_APB2ENR_t.SPI4EN=0) /**< Disable SPI4 clock */

#define USART1_PCLK_DI()                 (RCC->RCC_APB2ENR_t.USART1EN=0) /**< Disable USART1 clock */
#define USART2_PCLK_DI()                 (RCC->RCC_APB1ENR_t.USART2EN=0) /**< Disable USART2 clock */
#define USART6_PCLK_DI()                 (RCC->RCC_APB2ENR_t.USART6EN=0) /**< Disable USART6 clock */

#define SYSCFG_PCLK_DI()                 (RCC->RCC_APB2ENR_t.SYSCFGEN=0) /**< Disable SYSCFG clock */

/** @} */

/**
 * @defgroup Peripheral_Reset Peripheral Reset Macros
 * @brief Resets the given peripheral.
 * @{
 */

#define GPIOA_REG_RESET()                do{ (RCC->RCC_AHB1RSTR_t.GPIOARST=1); (RCC->RCC_AHB1RSTR_t.GPIOARST=0);}while(0) /**< Reset GPIOA */
#define GPIOB_REG_RESET()                do{ (RCC->RCC_AHB1RSTR_t.GPIOBRST=1); (RCC->RCC_AHB1RSTR_t.GPIOBRST=0);}while(0) /**< Reset GPIOB */
#define GPIOC_REG_RESET()                do{ (RCC->RCC_AHB1RSTR_t.GPIOCRST=1); (RCC->RCC_AHB1RSTR_t.GPIOCRST=0);}while(0) /**< Reset GPIOC */
#define GPIOD_REG_RESET()                do{ (RCC->RCC_AHB1RSTR_t.GPIODRST=1); (RCC->RCC_AHB1RSTR_t.GPIODRST=0);}while(0) /**< Reset GPIOD */
#define GPIOE_REG_RESET()                do{ (RCC->RCC_AHB1RSTR_t.GPIOERST=1); (RCC->RCC_AHB1RSTR_t.GPIOERST=0);}while(0) /**< Reset GPIOE */
#define GPIOH_REG_RESET()                do{ (RCC->RCC_AHB1RSTR_t.GPIOHRST=1); (RCC->RCC_AHB1RSTR_t.GPIOHRST=0);}while(0) /**< Reset GPIOH */

#define SPI1_REG_RESET()                 do{ (RCC->RCC_APB2RSTR_t.SPI1RST=1); (RCC->RCC_APB2RSTR_t.SPI1RST=0);}while(0) /**< Reset SPI1 */
#define SPI2_REG_RESET()                 do{ (RCC->RCC_APB1RSTR_t.SPI2RST=1); (RCC->RCC_APB1RSTR_t.SPI2RST=0);}while(0) /**< Reset SPI2 */
#define SPI3_REG_RESET()                 do{ (RCC->RCC_APB1RSTR_t.SPI3RST=1); (RCC->RCC_APB1RSTR_t.SPI3RST=0);}while(0) /**< Reset SPI3 */
#define SPI4_REG_RESET()                 do{ (RCC->RCC_APB2RSTR_t.SPI4RST=1); (RCC->RCC_APB2RSTR_t.SPI4RST=0);}while(0) /**< Reset SPI4 */

#define I2C1_REG_RESET()                 do{  (RCC->RCC_APB1RSTR_t.I2C1RST=1); (RCC->RCC_APB1RSTR_t.I2C1RST=0);}while(0) /**< Reset I2C1 */
#define I2C2_REG_RESET()                 do{  (RCC->RCC_APB1RSTR_t.I2C2RST=1); (RCC->RCC_APB1RSTR_t.I2C2RST=0);}while(0) /**< Reset I2C2 */
#define I2C3_REG_RESET()                 do{  (RCC->RCC_APB1RSTR_t.I2C3RST=1); (RCC->RCC_APB1RSTR_t.I2C3RST=0);}while(0) /**< Reset I2C3 */

#define USART1_REG_RESET()               do{  (RCC->RCC_APB2RSTR_t.USART1RST=1); (RCC->RCC_APB2RSTR_t.USART1RST=0);}while(0) /**< Reset USART1 */
#define USART2_REG_RESET()               do{  (RCC->RCC_APB1RSTR_t.USART2RST=1); (RCC->RCC_APB1RSTR_t.USART2RST=0);}while(0) /**< Reset USART2 */
#define USART6_REG_RESET()               do{  (RCC->RCC_APB2RSTR_t.USART6RST=1); (RCC->RCC_APB2RSTR_t.USART6RST=0);}while(0) /**< Reset USART6 */
/** @} */

/**
 * @defgroup IRQ_IRQ_Priorities IRQ and Priority Macros
 * @brief Defines IRQ numbers and priority levels for peripherals.
 * @{
 */

#define IRQ_NO_EXTI0            6   /**< EXTI0 IRQ number */
#define IRQ_NO_EXTI1            7   /**< EXTI1 IRQ number */
#define IRQ_NO_EXTI2            8   /**< EXTI2 IRQ number */
#define IRQ_NO_EXTI3            9   /**< EXTI3 IRQ number */
#define IRQ_NO_EXTI4            10  /**< EXTI4 IRQ number */
#define IRQ_NO_EXTI9_5          23  /**< EXTI9_5 IRQ number */
#define IRQ_NO_EXTI15_10        40  /**< EXTI15_10 IRQ number */

#define IRQ_NO_SPI1             35  /**< SPI1 IRQ number */
#define IRQ_NO_SPI2             36  /**< SPI2 IRQ number */
#define IRQ_NO_SPI3             51  /**< SPI3 IRQ number */
#define IRQ_NO_SPI4             84  /**< SPI4 IRQ number */

#define IRQ_NO_I2C1_EV          31
#define IRQ_NO_I2C1_ER          32             
#define IRQ_NO_I2C2_EV          33
#define IRQ_NO_I2C2_ER          34
#define IRQ_NO_I2C3_EV          72
#define IRQ_NO_I2C3_ER          73

/** @} */

/**
 * @defgroup SPI_Peripheral_Control SPI Peripheral Control Macros
 * @brief Enables, disables, and sets up SPI peripheral control bits.
 * @{
 */

#define SPI1_ENABLE()            (SPI1->SPI_CR1_t.SPE=1) /**< Enable SPI1 peripheral */
#define SPI2_ENABLE()            (SPI2->SPI_CR1_t.SPE=1) /**< Enable SPI2 peripheral */
#define SPI3_ENABLE()            (SPI3->SPI_CR1_t.SPE=1) /**< Enable SPI3 peripheral */
#define SPI4_ENABLE()            (SPI4->SPI_CR1_t.SPE=1) /**< Enable SPI4 peripheral */

#define SPI1_DISABLE()           (SPI1->SPI_CR1_t.SPE=0) /**< Disable SPI1 peripheral */
#define SPI2_DISABLE()           (SPI2->SPI_CR1_t.SPE=0) /**< Disable SPI2 peripheral */
#define SPI3_DISABLE()           (SPI3->SPI_CR1_t.SPE=0) /**< Disable SPI3 peripheral */
#define SPI4_DISABLE()           (SPI4->SPI_CR1_t.SPE=0) /**< Disable SPI4 peripheral */

#define SPI1_SSOE_HIGH()         (SPI1->SPI_CR2_t.SSOE=1) /**< Set SPI1 SSOE high */
#define SPI2_SSOE_HIGH()         (SPI2->SPI_CR2_t.SSOE=1) /**< Set SPI2 SSOE high */
#define SPI3_SSOE_HIGH()         (SPI3->SPI_CR2_t.SSOE=1) /**< Set SPI3 SSOE high */
#define SPI4_SSOE_HIGH()         (SPI4->SPI_CR2_t.SSOE=1) /**< Set SPI4 SSOE high */

#define SPI1_SSOE_LOW()          (SPI1->SPI_CR2_t.SSOE=0) /**< Set SPI1 SSOE low */
#define SPI2_SSOE_LOW()          (SPI2->SPI_CR2_t.SSOE=0) /**< Set SPI2 SSOE low */
#define SPI3_SSOE_LOW()          (SPI3->SPI_CR2_t.SSOE=0) /**< Set SPI3 SSOE low */
#define SPI4_SSOE_LOW()          (SPI4->SPI_CR2_t.SSOE=0) /**< Set SPI4 SSOE low */

#define SPI1_BUSY                (SPI1->SPI_SR_t.BSY) /**< Check if SPI1 is busy */
#define SPI2_BUSY                (SPI2->SPI_SR_t.BSY) /**< Check if SPI2 is busy */
#define SPI3_BUSY                (SPI3->SPI_SR_t.BSY) /**< Check if SPI3 is busy */
#define SPI4_BUSY                (SPI4->SPI_SR_t.BSY) /**< Check if SPI4 is busy */



#define USART1_ENABLE()         (USART1->USART_CR1_t.UE=1) /**< Enable USART1 peripheral */
#define USART2_ENABLE()         (USART2->USART_CR1_t.UE=1) /**< Enable USART2 peripheral */
#define USART6_ENABLE()         (USART6->USART_CR1_t.UE=1) /**< Enable USART6 peripheral */

#define USART1_DISABLE()         (USART1->USART_CR1_t.UE=0) /**< Enable USART1 peripheral */
#define USART2_DISABLE()         (USART2->USART_CR1_t.UE=0) /**< Enable USART2 peripheral */
#define USART6_DISABLE()         (USART6->USART_CR1_t.UE=0) /**< Enable USART6 peripheral */
/**
 * @brief Converts a GPIO base address to its corresponding port code.
 * 
 * This macro takes a GPIO base address and returns an integer code that corresponds to the specific GPIO port.
 * The codes are used in various register configurations and operations where a specific port code is required.
 * 
 * @param x The base address of the GPIO port (e.g., GPIOA, GPIOB, GPIOC, etc.).
 * @return The integer port code corresponding to the given GPIO base address:
 *         - 0 for GPIOA
 *         - 1 for GPIOB
 *         - 2 for GPIOC
 *         - 3 for GPIOD
 *         - 4 for GPIOE
 *         - 7 for GPIOH
 *         - 0 if the base address does not match any defined port
 */
#define GPIO_BASEADDR_TO_CODE(x)          ( (x  ==   GPIOA)  ? 0: \
                                            (x  ==   GPIOB)  ? 1: \
                                            (x  ==   GPIOC)  ? 2: \
                                            (x  ==   GPIOD)  ? 3: \
                                            (x  ==   GPIOE)  ? 4: \
                                            (x  ==   GPIOH)  ? 7 : 0 )





/** @} */



/** 
 * @brief Generic macros for use in various peripheral setups.
 */
#define ENABLE          1   /**< Enable flag */
#define DISABLE         0   /**< Disable flag */
#define SET             1   /**< Set flag */
#define RESET           0   /**< Reset flag */
#define GPIO_PIN_SET    1   /**< GPIO pin set */
#define GPIO_PIN_RESET  0   /**< GPIO pin reset */


#define GET_AHB1_CLK_PRE                (       (RCC->RCC_CFGR_t.HPRE==8)   ? 2:\
                                                (RCC->RCC_CFGR_t.HPRE==9)   ? 4:\
                                                (RCC->RCC_CFGR_t.HPRE==10)  ? 8:\
                                                (RCC->RCC_CFGR_t.HPRE==11)  ? 16:\
                                                (RCC->RCC_CFGR_t.HPRE==12)  ? 64:\
                                                (RCC->RCC_CFGR_t.HPRE==13)  ? 128:\
                                                (RCC->RCC_CFGR_t.HPRE==14)  ? 256:\
                                                (RCC->RCC_CFGR_t.HPRE==15)  ? 512:1)



#define GET_APB1_CLK_PRE                (       (RCC->RCC_CFGR_t.PPRE1==4)   ? 2:\
                                                (RCC->RCC_CFGR_t.PPRE1==5)   ? 4:\
                                                (RCC->RCC_CFGR_t.PPRE1==6)   ? 8:\
                                                (RCC->RCC_CFGR_t.PPRE1==7)   ? 16:1)       

#define GET_APB2_CLK_PRE                (       (RCC->RCC_CFGR_t.PPRE2==4)   ? 2:\
                                                (RCC->RCC_CFGR_t.PPRE2==5)   ? 4:\
                                                (RCC->RCC_CFGR_t.PPRE2==6)   ? 8:\
                                                (RCC->RCC_CFGR_t.PPRE2==7)   ? 16:1)     


#include "stm32f401xx_gpio_driver.h"
#include "Stm32f401xx_spi_driver.h"
#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_usart_driver.h"

#endif /* INC_STM32F401XX_H_ */