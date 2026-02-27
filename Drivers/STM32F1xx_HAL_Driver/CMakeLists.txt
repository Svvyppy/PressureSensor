cmake_minimum_required(VERSION 3.10)
set(LIBRARY_NAME HAL)

add_library(${LIBRARY_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_gpio_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f1xx_hal_uart.c
)

target_include_directories(${LIBRARY_NAME}  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_link_libraries(${LIBRARY_NAME} INTERFACE CMSIS)
target_link_libraries(${LIBRARY_NAME} PUBLIC Periph)