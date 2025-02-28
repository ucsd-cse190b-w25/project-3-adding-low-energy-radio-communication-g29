/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l4xx_hal.h"

#include "leds.h"
#include "timer.h"
#include "i2c.h"
#include "lsm6dsl.h"
#include "ble.h"

SPI_HandleTypeDef hspi3;
int dataAvailable = 0;  // Flag set by BLE interrupt (updated in EXTI ISR)

#define MOVEMENT_THRESHOLD    1200   // Max allowed change in any axis for static detection
#define LOST_MODE_THRESHOLD   600    // 200 consecutive static samples (~30 seconds)
#define MESSAGE_INTERVAL_COUNT 100   // 100 iterations (~10 seconds per message)

uint32_t static_count = 0;
volatile uint8_t lost_mode = 0;
uint32_t message_timer = 0;
uint32_t lost_seconds = 0;


int16_t ax, ay, az;
int16_t prev_ax, prev_ay, prev_az;


#define BLE_MAX_PACKET_SIZE 20  


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);


void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        // leds_set(lost_mode ? 1 : 0);
    }
}


int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}

// BLE message sending function
void sendBleMessage(uint32_t secondsMissing) {
    char fullMessage[64];
    snprintf(fullMessage, sizeof(fullMessage), "PrivTag DS12345 lost for %lu sec", secondsMissing);

    int messageLength = strlen(fullMessage);
    int bytesSent = 0;

    printf("[BLE] Sending message: %s\n", fullMessage);

    while (bytesSent < messageLength) {
        int chunkSize = (messageLength - bytesSent) > BLE_MAX_PACKET_SIZE ? BLE_MAX_PACKET_SIZE : (messageLength - bytesSent);

        printf("[BLE] Sending chunk: %.*s\n", chunkSize, &fullMessage[bytesSent]);

        updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, chunkSize, (uint8_t *)&fullMessage[bytesSent]);

        bytesSent += chunkSize;
        HAL_Delay(50);  // Prevent BLE flooding
    }
}

int main(void)
{
    // HAL initialization and system clock configuration 
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI3_Init();

    // Initialize peripherals
    leds_init();
    timer_init(TIM2);
    timer_set_ms(TIM2, 50);
    i2c_init();
    lsm6dsl_init();

    // Get initial accelerometer reading
    lsm6dsl_read_xyz(&prev_ax, &prev_ay, &prev_az);

    printf("DS12345 PrivTag started. Monitoring movement...\n");

    while (1)
    {
        lsm6dsl_read_xyz(&ax, &ay, &az);

        int diff_x = abs(ax - prev_ax);
        int diff_y = abs(ay - prev_ay);
        int diff_z = abs(az - prev_az);

        // Static detection
        if (diff_x < MOVEMENT_THRESHOLD && diff_y < MOVEMENT_THRESHOLD && diff_z < MOVEMENT_THRESHOLD)
        {
            static_count++;
        }
        else
        {
            // Movement detected: Exit lost mode
            if (lost_mode)
            {
                printf("[LOST MODE] Movement detected: Exiting lost mode.\n");

                lost_mode = 0;
                static_count = 0;
                message_timer = 0;
                lost_seconds = 0;

                // Force BLE to Disconnect
                printf("[BLE] Disconnecting and making non-discoverable...\n");
                disconnectBLE();
                HAL_Delay(100);
                setDiscoverability(0);  // Stop BLE advertisements
                HAL_Delay(100);
                HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
                HAL_Delay(500);
                HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);
                HAL_Delay(500);

                printf("[BLE] BLE module completely shut down.\n");
            }
        }

        // Enter Lost Mode
        if (static_count >= LOST_MODE_THRESHOLD)
        {
            if (!lost_mode)
            {
                lost_mode = 1;

                // Initialize BLE in Lost Mode
                printf("[BLE] Initializing BLE (Entering Lost Mode)...\n");

                // Reset BLE before enabling it
                HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
                HAL_Delay(500);
                HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);
                HAL_Delay(500);

                ble_init();
                HAL_Delay(100);
                setDiscoverability(1);
                HAL_Delay(100);
                setConnectable();

                printf("[LOST MODE] BLE is now connectable.\n");
            }
        }

        // Send BLE message every 10 seconds
        if (lost_mode)
        {
            message_timer++;
            if (message_timer >= MESSAGE_INTERVAL_COUNT)
            {
                lost_seconds += 10;
                sendBleMessage(lost_seconds);
                message_timer = 0;
            }
        }
        prev_ax = ax;
        prev_ay = ay;
        prev_az = az;

        printf("[STATUS] Accel: X=%d, Y=%d, Z=%d, static_count=%lu, lost_mode=%d, lost_seconds=%lu\n",
               ax, ay, az, static_count, lost_mode, lost_seconds);

        HAL_Delay(50);  // Reduced delay for responsiveness
    }

    return 0;
}


// System Clock Configuration
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

// SPI3 Initialization
static void MX_SPI3_Init(void)
{
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
}

// GPIO Initialization
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure BLE_INT_Pin (for BLE interrupt)
    GPIO_InitStruct.Pin = BLE_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

    // Configure GPIO_LED1_Pin and BLE_RESET_Pin as outputs
    GPIO_InitStruct.Pin = GPIO_LED1_Pin | BLE_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure BLE_CS_Pin as output 
    GPIO_InitStruct.Pin = BLE_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

    // Set initial levels for BLE_CS and BLE_RESET
    HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

    // Configure and enable EXTI interrupt for BLE_INT
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

// Error Handler
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
