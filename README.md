
# STM32F401xx Peripheral Driver Library

This repository provides custom peripheral drivers for STM32F401xx microcontrollers, specifically for GPIO and SPI functionalities. The drivers are written in C and are designed for flexibility, making them suitable for embedded systems development. This project is intended for educational purposes to demonstrate low-level programming and peripheral interfacing with STM32 MCUs.

## Repository Structure

- **stm32f401xx.h**  
  The main header file that includes other necessary driver headers (`stm32f401xx_gpio_driver.h`, `stm32f401xx_spi_driver.h`) for easy integration. Users only need to include `stm32f401xx.h` in their projects.

- **stm32f401xx_gpio_driver.h / .c**  
  GPIO driver files implementing functionality to configure, initialize, and control GPIO pins on the STM32F401xx MCU.

- **stm32f401xx_spi_driver.h / .c**  
  SPI driver files implementing the initialization and data handling functions for SPI communication.

- **001test_app_for_spi_driver.c**  
  A test application demonstrating the usage of the SPI driver by setting up basic SPI communication.

- **002Testapp_SPI_Sl_arduino.c**  
  Another test application that interfaces the STM32 SPI as a slave with an Arduino, demonstrating SPI slave communication.

- **generate_documentation.bat**  
  A script file to generate documentation for the project using Doxygen.

## Features

- **Unified Header Import**: Only `stm32f401xx.h` needs to be included in the user's project to access all drivers.
- **GPIO Driver**: Configurable GPIO pins with options for mode, speed, pull-up/pull-down settings, and output type.
- **SPI Driver**: Support for initializing SPI in master/slave modes with options for data frame format, clock polarity, and clock phase.
- **Doxygen Documentation**: Automatically generate code documentation using Doxygen.
- **Test Applications**: Example programs to test and demonstrate SPI communication, including an Arduino interface.

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/mehmethanturut/stm32f4xx_drivers.git
   cd stm32f4xx_drivers
   ```

2. **Set up your development environment**:  
   Install the STM32CubeIDE or any compatible IDE supporting STM32 development.

3. **Add required files**:  
   Include the provided `.h` driver files in your STM32 project.

## Usage

1. **Including Drivers**:  
   Only include the main header file in your source files:
   ```c
   #include "stm32f401xx.h"
   ```

2. **Generating Documentation**:  
   Run the `generate_documentation.bat` file to create the documentation. This will use Doxygen to generate HTML and/or LaTeX documentation files in a `docs` folder.
   ```bash
   ./generate_documentation.bat
   ```

3. **Example Usage**:  
   Use the `001test_app_for_spi_driver.c` to initialize SPI and test data transfer. The `002Testapp_SPI_Sl_arduino.c` provides an example of using the STM32 as an SPI slave in conjunction with an Arduino.

4. **Configuring Parameters**:  
   Modify GPIO or SPI configurations in the respective driver files as per your project requirements. Parameters for speed, mode, pull-up/pull-down, and frame format can be adjusted within the driver setup functions.

## Configuration

- **GPIO Configuration**: Set pin modes, speed, and pull settings in `stm32f401xx_gpio_driver.c`.
- **SPI Configuration**: Configure data format, clock phase, and polarity in `stm32f401xx_spi_driver.c`.

## Contributing

Contributions are welcome! Please fork the repository, create a feature branch, and submit a pull request. Ensure that your code adheres to project coding standards and includes necessary documentation.

## License

This project is licensed under the MIT License.
