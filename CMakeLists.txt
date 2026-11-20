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
set(FIND_HOME_MODE "NAVIGATOR" CACHE STRING "Firmware: NAVIGATOR, NORTH or IMU")
set_property(CACHE FIND_HOME_MODE PROPERTY STRINGS NAVIGATOR NORTH IMU)
set(FIND_HOME_DECLINATION_DEG "0.0" CACHE STRING "Magnetic declination, degrees east-positive; 0 leaves correction off")
if(NOT FIND_HOME_MODE MATCHES "^(NAVIGATOR|NORTH|IMU)$")
    message(FATAL_ERROR "FIND_HOME_MODE must be NAVIGATOR, NORTH or IMU")
endif()
if(NOT FIND_HOME_DECLINATION_DEG MATCHES "^-?[0-9]+([.][0-9]+)?$")
    message(FATAL_ERROR "FIND_HOME_DECLINATION_DEG must be a decimal number")
endif()
add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# SPI2 initialization belongs to imu_port.c rather than generated CubeMX code.
target_compile_definitions(stm32cubemx INTERFACE HAL_SPI_MODULE_ENABLED)
if(NOT FIND_HOME_MODE STREQUAL "IMU")
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    set(CALIBRATION_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/north_calibration.h)
    add_custom_command(OUTPUT ${CALIBRATION_HEADER}
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/export_calibration.py
            ${CMAKE_CURRENT_SOURCE_DIR}/calibration/imu_accel.json
            ${CMAKE_CURRENT_SOURCE_DIR}/calibration/mag.json ${CALIBRATION_HEADER}
        DEPENDS tools/export_calibration.py calibration/imu_accel.json calibration/mag.json
        VERBATIM)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
    target_sources(${PROJECT_NAME} PRIVATE Core/Src/app.c Core/Src/north.c ${CALIBRATION_HEADER})
    if(FIND_HOME_MODE STREQUAL "NAVIGATOR")
        target_compile_definitions(${PROJECT_NAME} PRIVATE FIND_HOME_NAVIGATOR
            FIND_HOME_DECLINATION_DEG=${FIND_HOME_DECLINATION_DEG})
        target_sources(${PROJECT_NAME} PRIVATE Core/Src/navigator.c Core/Src/power.c)
    endif()
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE FIND_HOME_IMU_TEST)
    target_sources(${PROJECT_NAME} PRIVATE Core/Src/imu_test.c)
endif()
target_sources(${PROJECT_NAME} PRIVATE
    Core/Src/imu_port.c
    Core/Src/imu.c
    Core/Src/matrix.c
    Core/Src/heading.c
    Core/Src/gnss.c
    Core/Src/mag.c
    Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c
    Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c
)
target_link_libraries(${PROJECT_NAME} stm32cubemx)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
