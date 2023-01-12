/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    S0, /* Empty/Initial */
    S1, /* Short */
    S2, /* Pressed */
    S3, /* Long */
} keyState_enum;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint32_t sysTime = 0;

uint32_t keyUpdate_TS[keyNum];
keyState_enum keyState[keyNum];

uint8_t ledBuffer = 0;

char uartRxBuffer[uartBufferSize];
uint8_t uartRxBufferIdx = 0;
uint8_t uartRxOKFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void keyUpdate(void);
void keyResp(void);
void ledUpdate(void);
void msDelay(uint32_t t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init*/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

    /** NONJTRST: Full SWJ (JTAG-DP + SW-DP) but without NJTRST
     */
    LL_GPIO_AF_Remap_SWJ_NONJTRST();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    LL_SYSTICK_EnableIT();
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        keyUpdate();
        keyResp();
        ledUpdate();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
    {
    }
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
    {
    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }
    LL_Init1msTick(72000000);
    LL_SetSystemCoreClock(72000000);
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/* USER CODE BEGIN 4 */

void msDelay(uint32_t t)
{
    uint32_t msDelay_TS = sysTime, delayT = t;
    if (delayT < 0xffffffff)
        delayT++;
    while (sysTime - msDelay_TS < delayT) /* wait */
        ;
}

void keyUpdate(void)
{
    uint8_t i = keyNum, keyInfo = 0xff;

    keyInfo ^= ((LL_GPIO_IsInputPinSet(B1_GPIO_Port, B1_Pin)) << 0);
    keyInfo ^= ((LL_GPIO_IsInputPinSet(B2_GPIO_Port, B2_Pin)) << 1);
    keyInfo ^= ((LL_GPIO_IsInputPinSet(B3_GPIO_Port, B3_Pin)) << 2);
    keyInfo ^= ((LL_GPIO_IsInputPinSet(B4_GPIO_Port, B4_Pin)) << 3);

    while (i--)
    {
        /* Pressed */
        if ((keyInfo & (1 << i)) == 1)
        {
            switch (keyState[i])
            {
            case S0:
                keyState[i] = S2;          // switch state
                keyUpdate_TS[i] = sysTime; // update timestamp
                break;

            default:
                break;
            }
        }
        /* Not Pressed */
        if ((keyInfo & (1 << i)) == 0)
        {
            switch (keyState[i])
            {
            case S2:
                if (sysTime - keyUpdate_TS[i] >= keyLongPressTime) // S3 detection
                    keyState[i] = S3;
                else if (sysTime - keyUpdate_TS[i] >= keyShortPressTime) // S1 detection
                    keyState[i] = S1;
                else
                    keyState[i] = S0; // reset state
                break;

            default:
                keyState[i] = S0; // reset state
                break;
            }
        }
    }
}

void keyResp(void)
{
    /* B1 */
    switch (keyState[0])
    {
    case S1: // Short

        ledBuffer <<= 1;
        if (ledBuffer == 0)
            ledBuffer = 1;

        keyState[0] = S0; // reset state
        break;

    case S3: // Long

        keyState[0] = S0; // reset state
        break;

    default:
        break;
    }

    /* B2 */
    switch (keyState[1])
    {
    case S1: // Short
        ledBuffer >>= 1;
        if (ledBuffer == 0)
            ledBuffer = 1;

        keyState[1] = S0; // reset state
        break;

    case S3: // Long

        keyState[1] = S0; // reset state
        break;

    default:
        break;
    }

    /* B3 */
    switch (keyState[2])
    {
    case S1: // Short

        keyState[2] = S0; // reset state
        break;

    case S3: // Long

        keyState[2] = S0; // reset state
        break;

    default:
        break;
    }

    /* B4 */
    switch (keyState[3])
    {
    case S1: // Short

        keyState[3] = S0; // reset state
        break;

    case S3: // Long

        keyState[3] = S0; // reset state
        break;

    default:
        break;
    }
}

void ledUpdate(void)
{
    LL_GPIO_WriteOutputPort(LD1_GPIO_Port, ~ledBuffer << 8);
    LL_GPIO_SetOutputPin(LE_GPIO_Port, LE_Pin);
    LL_GPIO_ResetOutputPin(LE_GPIO_Port, LE_Pin);
}

void uart_ReceiveIRQ(void)
{
    uint8_t ch = LL_USART_ReceiveData8(USART1);

    if (ch == (uint8_t)('\r'))
    {
        uartRxBufferIdx = 0;
        uartRxOKFlag = 1;
    }
    else
        uartRxBuffer[uartRxBufferIdx++] = ch;
}

int fputc(int ch, FILE *f)
{
    while (!LL_USART_IsActiveFlag_TXE(USART1))
        ;
    LL_USART_TransmitData8(USART1, (uint8_t)ch);
    return ch;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
