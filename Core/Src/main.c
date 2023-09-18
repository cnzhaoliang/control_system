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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sys.h"
#include "delay.h"
#include "protocol.h" //仅在调试串口通信的时候需要
#include "stmflash.h"
#include "gimbal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void SendMsgToSerial(cJSON *root);
void ProcessCmdFromSerial(void);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t extiEvent;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  int i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  delay_init(168);

  Laser_Gimbal_Init();
  Gimbal.SetTask(&Gimbal, GIMBAL_SLAVE_TASK);
  Gimbal.Start(&Gimbal);
  printf("slave mode\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (extiEvent == KEY0_EVENT)
    {
      i++;
      if (i % 2)
      {
        Gimbal.SetTask(&Gimbal, GIMBAL_NO_TASK);
        extiEvent = 0;
      }
      else
      {
        Gimbal.SetTask(&Gimbal, GIMBAL_SLAVE_TASK);
      }
      extiEvent = NO_EVENT;
    }
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    ProcessCmdFromSerial();
    delay_ms(100);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

void removeWhitespace(char *str)
{
  char *p = str;
  char *q = str;
  while (*p != '\0')
  {
    if (!isspace((unsigned char)*p))
    {
      *q = *p;
      q++;
    }
    p++;
  }
  *q = '\0';
}

void SendMsgToSerial(cJSON *root)
{
  char *json_str = cJSON_Print(root);
  removeWhitespace(json_str);
  printf("%s\r\n", json_str);
  free(json_str);
}

void ProcessCmd(const char *json_str)
{
  // 解析 JSON
  cJSON *root = cJSON_Parse(json_str);
  if (root == NULL)
  {
    printf("Failed to parse JSON: %s\n", cJSON_GetErrorPtr());
    return;
  }
  // 处理 JSON 数据
  cJSON *Kp = cJSON_GetObjectItemCaseSensitive(root, "Kp");
  cJSON *Ki = cJSON_GetObjectItemCaseSensitive(root, "Ki");
  cJSON *Kd = cJSON_GetObjectItemCaseSensitive(root, "Kd");
  if (!cJSON_IsNumber(Kp) && !cJSON_IsNumber(Ki) && !cJSON_IsNumber(Kd))
  {
    printf("JSON format error\n");
  }
  else
  {
    Gimbal.Stop(&Gimbal);
    if (cJSON_IsNumber(Kp))
    {
      Gimbal.yawController.Kp = Kp->valuedouble;
      Gimbal.pitchController.Kp = Kp->valuedouble;
    }
    if (cJSON_IsNumber(Ki))
    {
      Gimbal.yawController.Ki = Ki->valuedouble;
      Gimbal.pitchController.Ki = Ki->valuedouble;
    }
    if (cJSON_IsNumber(Kd))
    {
      Gimbal.yawController.Kd = Kd->valuedouble;
      Gimbal.pitchController.Kd = Kd->valuedouble;
    }
    // 执行操作
    printf("Command received!\r\n");
    delay_ms(2000);
    Gimbal.Start(&Gimbal);
  }
  // 释放资源
  cJSON_Delete(root);
}

void ProcessCmdFromSerial(void)
{
  char jsonStrBuffer[DATA_MAX_LEN];
  if (UART_Frame.COM1_STA == NOT_EMPTY)
  {
    memcpy(jsonStrBuffer, UART_Frame.COM1_BUF, DATA_MAX_LEN);
    ProcessCmd((const char *)jsonStrBuffer);
    UART_Frame.COM1_STA = EMPTY;
  }
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
