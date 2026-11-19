cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Keep the CubeMX target name for existing VS Code debug configurations.
set(CMAKE_PROJECT_NAME LAB04_I2C)
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
option(FIND_HOME_SENSOR_TEST "Run GNSS/compass discovery instead of IMU/matrix test" ON)
add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# SPI2 initialization belongs to imu_test.c rather than generated CubeMX code.
target_compile_definitions(stm32cubemx INTERFACE HAL_SPI_MODULE_ENABLED)
if(FIND_HOME_SENSOR_TEST)
    target_compile_definitions(${PROJECT_NAME} PRIVATE FIND_HOME_SENSOR_TEST)
    target_sources(${PROJECT_NAME} PRIVATE Core/Src/sensor_test.c)
else()
    target_sources(${PROJECT_NAME} PRIVATE Core/Src/imu_test.c)
endif()
target_sources(${PROJECT_NAME} PRIVATE
    Core/Src/imu.c
    Core/Src/matrix.c
    Core/Src/heading.c
    Core/Src/gnss.c
    Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c
    Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c
)
target_link_libraries(${PROJECT_NAME} stm32cubemx)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
