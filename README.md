# STM32F401xx GPIO Driver

This project provides a GPIO driver library for the STM32F401xx microcontroller. The library includes functions and macros to configure and control the GPIO peripherals on STM32F401xx-based boards, offering a reusable and easy-to-understand interface for developers.

## Project Structure

- **Inc/**
  - `stm32f401xx.h`: Core hardware definitions, base addresses, and register structures for STM32F401xx peripherals.
  - `stm32f401xx_gpio_driver.h`: Header file containing GPIO configurations, structures, macros, and function prototypes.
- **Src/**
  - `stm32f401xx_gpio_driver.c`: Source file implementing the GPIO functions defined in `stm32f401xx_gpio_driver.h`.

## Features

- Initialize and deinitialize GPIO pins
- Enable/disable GPIO peripheral clocks
- Read/write data from/to GPIO pins or ports
- Configure pin modes, output types, speeds, and pull-up/pull-down settings
- Toggle GPIO pin output states
- Configure and handle GPIO interrupts and priorities

## Getting Started

### Prerequisites

- **STM32CubeIDE** or another IDE supporting STM32 development
- **STM32F401xx** board (e.g., STM32F4 Discovery)

### Installation

1. Clone this repository.
2. Open the project in your IDE.
3. Link the provided `Inc` and `Src` directories to your project.

### Usage

1. Include `stm32f401xx_gpio_driver.h` in your source files where GPIO operations are needed:
   
   ```c
   #include "stm32f401xx_gpio_driver.h"
   ```
2. Initialize GPIO pins using `GPIO_Init()`, configure their mode, speed, and other settings as needed.

### Example

```c
#include "stm32f401xx_gpio_driver.h"

int main(void) {
    GPIO_Handle_t gpioLED;

    gpioLED.pGPIOx = GPIOA;
    gpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    gpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClkCtrl(GPIOA, ENABLE);
    GPIO_Init(&gpioLED);

    while (1) {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        for (volatile int i = 0; i < 500000; i++); // Simple delay
    }
}
```

## Documentation

Each function in this project is documented using [Doxygen](https://www.doxygen.nl/). To generate documentation:

1. Ensure Doxygen is installed.
2. Run the following command in the project directory:
   
   ```bash
   doxygen Doxyfile
   ```

### Key Files

#### stm32f401xx.h

The `stm32f401xx.h` file provides core definitions, register mappings, and clock control configurations for the STM32F401xx microcontroller.

- Defines NVIC, APB, AHB register addresses, and base addresses for Flash, SRAM, and peripherals.
- Structures for configuring peripheral registers.
- Includes clock control macros for enabling and disabling peripheral clocks.

**File Content Highlights**:

- **NVIC Registers**:
  
  ```c
  #define NVIC_ISER0 ((volatile uint32_t*)0xE000E100)
  #define NVIC_ICER0 ((volatile uint32_t*)0xE000E180)
  ```
- **Base Addresses**:
  
  ```c
  #define FLASH_BASEADDR 0x08000000U
  #define SRAM_BASEADDR  0x20000000U
  ```
- **Peripheral Structure Example**:
  
  ```c
  typedef struct {
      struct {
          uint32_t MODER0 :2;
          // ...
      } GPIOx_MODER_t;
      // Other GPIO configuration registers
  } GPIO_RegDef_t;
  ```

#### stm32f401xx_gpio_driver.h

The `stm32f401xx_gpio_driver.h` file defines the GPIO driver interface, including configuration structures, function prototypes, and macros.

- **GPIO Pin Configuration Structure**: Configures each pin's mode, speed, output type, pull-up/pull-down settings, etc.
- **GPIO Handle Structure**: Associates a GPIO port with its pin configuration.
- **GPIO Macros**: Defines possible modes, output types, speeds, and pull-up/pull-down options.

**File Content Highlights**:

- **Pin Modes**:
  
  ```c
  #define GPIO_MODE_IN 0
  #define GPIO_MODE_OUT 1
  ```
- **Functions**:
  
  ```c
  void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
  void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
  uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
  ```

#### stm32f401xx_gpio_driver.c

The `stm32f401xx_gpio_driver.c` file implements the GPIO driver functions declared in `stm32f401xx_gpio_driver.h`.

- **Clock Control**: Enables or disables the clock for GPIO ports.
- **GPIO Initialization**: Sets up the GPIO pin configuration based on the structure provided.
- **Data Read/Write**: Reads input data from pins and writes output data to pins.
- **Interrupt Configuration**: Configures interrupts for GPIO pins and sets interrupt priorities.

**File Content Highlights**:

- **GPIO Initialization Example**:
  
  ```c
  void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
      // Function to initialize GPIO based on configuration
  }
  ```
- **GPIO Write Example**:
  
  ```c
  void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {
      // Write logic for the specified pin
  }
  ```

## License

This project is open-source and available under the MIT License.

## Author

Developed by Mehmethan Türüt.
