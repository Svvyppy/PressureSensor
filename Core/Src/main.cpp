/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "hydrolib_bus_application_master.hpp"
#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_uart.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

#ifdef __cplusplus
}
#endif
extern "C"
{
    /* Private function prototypes
     * -----------------------------------------------*/
    void SystemClock_Config(void);
}

#define BUFFER_LENGTH 5

class Memory
{
public:
    hydrolib::ReturnCode Read(void *buffer, unsigned address, unsigned length)
    {
        if (length + address > BUFFER_LENGTH)
        {
            return hydrolib::ReturnCode::FAIL;
        }
        memcpy(buffer, buffer_ + address, length);
        return hydrolib::ReturnCode::OK;
    }

    hydrolib::ReturnCode Write(const void *buffer, unsigned address, unsigned length)
    {
        if (address + length > BUFFER_LENGTH)
        {
            return hydrolib::ReturnCode::FAIL;
        }
        memcpy(buffer_ + address, buffer, length);
        return hydrolib::ReturnCode::OK;
    }

    uint32_t Size() { return BUFFER_LENGTH; }

private:
    uint8_t buffer_[BUFFER_LENGTH] = {};
};
static constexpr hydrv::UART::UARTLow::UARTPreset USART1_115200_LOW{
    USART1_BASE, 7, RCC_APB2ENR_USART1EN, RCC_BASE + offsetof(RCC_TypeDef, APB1ENR), USART1_IRQn, 17, 6};

constinit hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 10, hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 9, hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255> uart1(USART1_115200_LOW, rx_pin1, tx_pin1, 7);

constinit hydrv::GPIO::GPIOLow rx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 11, hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 10, hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255> uart3(hydrv::UART::UARTLow::USART3_115200_LOW, rx_pin3, tx_pin3, 7);

constinit hydrolib::logger::LogDistributor<decltype(uart3)> distributor("[%s] [%l] %m\n\r", uart3);
// hydrolib::logger::LogDistributor distributor("%m", uart3);
constinit hydrolib::logger::Logger<decltype(distributor)> logger("SerialProtocol", 1, distributor);

hydrolib::bus::datalink::StreamManager manager(1, uart1, logger);
hydrolib::bus::datalink::Stream stream(manager, 2);

Memory memory;

hydrolib::bus::application::Slave slave(stream, memory, logger);
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();

    uart1.Init();
    uart3.Init();

    while (1)
    {
        manager.Process();
        slave.Process();
    }
}

extern "C"
{

    void USART3_IRQHandler(void) { uart3.IRQCallback(); }
    void USART1_IRQHandler(void) { uart1.IRQCallback(); }
    /**
     * @brief System Clock Configuration
     * @retval None
     */
    void SystemClock_Config(void)
    {
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
        RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

        /** Initializes the RCC Oscillators according to the specified
         * parameters in the RCC_OscInitTypeDef structure.
         */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.HSEState = RCC_HSE_ON;
        RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            Error_Handler();
        }

        /** Initializes the CPU, AHB and APB buses clocks
         */
        RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

        if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        {
            Error_Handler();
        }
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
        PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* USER CODE BEGIN 4 */

    /* USER CODE END 4 */

    /**
     * @brief  This function is executed in case of error occurrence.
     * @retval None
     */
    void Error_Handler(void)
    {
        /* USER CODE BEGIN Error_Handler_Debug */
        /* User can add his own implementation to report the HAL error return
         * state */
        __disable_irq();
        while (1)
        {
        }
        /* USER CODE END Error_Handler_Debug */
    }
}
#ifdef USE_FULL_ASSERT
extern "C"
{
    /**
     * @brief  Reports the name of the source file and the source line number
     *         where the assert_param error has occurred.
     * @param  file: pointer to the source file name
     * @param  line: assert_param error line source number
     * @retval None
     */
    void assert_failed(uint8_t *file, uint32_t line)
    {
        /* USER CODE BEGIN 6 */
        /* User can add his own implementation to report the file name and line
           number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
           file, line) */
        /* USER CODE END 6 */
    }
}
#endif /* USE_FULL_ASSERT */
