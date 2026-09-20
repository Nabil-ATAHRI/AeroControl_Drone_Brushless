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
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

uint32_t AD_RES = 0;
uint16_t ESC_Pulse = 1000;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t mpu_ok = 0;
uint8_t motor_running = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);

uint16_t ADC_To_ESC(uint32_t adc_value)
{
uint16_t pulse;

if (adc_value > 4095U)
{
    adc_value = 4095U;
}

pulse = (uint16_t)(1000U +
         ((adc_value * 1000U) / 4095U));

return pulse;

}

void ESC_SetPulse(uint16_t pulse_us)
{
if (pulse_us < 1000U)
{
pulse_us = 1000U;
}

if (pulse_us > 2000U)
{
    pulse_us = 2000U;
}

__HAL_TIM_SET_COMPARE(&htim1,
                      TIM_CHANNEL_1,
                      pulse_us);

}

void UART_Send_Status(uint32_t adc,
uint16_t pulse)
{
char buffer[100];

sprintf(buffer,
        "ADC = %lu | ESC PWM = %u us\r\n",
        adc,
        pulse);

HAL_UART_Transmit(&huart2,
                  (uint8_t *)buffer,
                  strlen(buffer),
                  HAL_MAX_DELAY);

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SetMotorStatusLeds(uint8_t running)
{
    /* On this board: PD13 = green, PD12 = red, both active-low */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, running ? GPIO_PIN_RESET : GPIO_PIN_SET); /* green */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, running ? GPIO_PIN_SET : GPIO_PIN_RESET); /* red */
}


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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  /* ==========================================================
  UART START
  ========================================================== */

  char start_msg[] =
  "\r\n"
  "========================================\r\n"
  " STM32F407VGT6 - ADC + ESC TEST\r\n"
  " MPU6050 DISABLED\r\n"
  "========================================\r\n";

  HAL_UART_Transmit(&huart2,
  (uint8_t *)start_msg,
  strlen(start_msg),
  HAL_MAX_DELAY);

  /* ==========================================================
  ESC INITIALIZATION
  ========================================================== */

  char esc_msg[] =
  "ESC: PWM initialization...\r\n";

  HAL_UART_Transmit(&huart2,
  (uint8_t *)esc_msg,
  strlen(esc_msg),
  HAL_MAX_DELAY);

  /*

  Minimum throttle
  */

  ESC_SetPulse(1000);

  /*

  Start TIM1 CH1 PWM
  */

  if (HAL_TIM_PWM_Start(&htim1,
  TIM_CHANNEL_1) != HAL_OK)
  {
  char error_msg[] =
  "ERROR: TIM1 PWM START FAILED\r\n";

  HAL_UART_Transmit(&huart2,
                    (uint8_t *)error_msg,
                    strlen(error_msg),
                    HAL_MAX_DELAY);

  Error_Handler();

  }

  /*

  Keep ESC at minimum throttle
  for 5 seconds.
  */

  char wait_msg[] =
  "ESC: 1000 us\r\n"
  "ESC: waiting 5 seconds...\r\n";

  HAL_UART_Transmit(&huart2,
  (uint8_t *)wait_msg,
  strlen(wait_msg),
  HAL_MAX_DELAY);

  HAL_Delay(5000);

  /*

  ESC ready
  */

  char ready_msg[] =
  "ESC: READY\r\n"
  "ADC control enabled\r\n";

  HAL_UART_Transmit(&huart2,
  (uint8_t *)ready_msg,
  strlen(ready_msg),
  HAL_MAX_DELAY);


  void UART_Send_Status(uint32_t adc, uint16_t pulse)
  {
  char buffer[100];
  sprintf(buffer,
          "ADC = %lu | ESC PWM = %u us\r\n",
          adc,
          pulse);

  HAL_UART_Transmit(&huart2,
                    (uint8_t *)buffer,
                    strlen(buffer),
                    HAL_MAX_DELAY);

  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	      uint32_t adc_value;
	      uint16_t esc_pulse;
	      char buffer[100];

	      /* ==========================================
	         1. Start ADC
	         ========================================== */
	      if (HAL_ADC_Start(&hadc1) == HAL_OK)
	      {
	          /* ==========================================
	             2. Wait for ADC conversion
	             ========================================== */
	          if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	          {
	              /* ==========================================
	                 3. Read ADC value
	                 ADC = 0 ... 4095
	                 ========================================== */
	              adc_value = HAL_ADC_GetValue(&hadc1);

	              /* ==========================================
	                 4. Convert ADC to ESC PWM

	                 ADC = 0     -> 1000 us
	                 ADC = 4095  -> 2000 us
	                 ========================================== */
	              esc_pulse = 1000U +
	                          (uint16_t)((adc_value * 1000U) / 4095U);

	              /* Safety limits */
	              if (esc_pulse < 1000U)
	              {
	                  esc_pulse = 1000U;
	              }

	              if (esc_pulse > 2000U)
	              {
	                  esc_pulse = 2000U;
	              }

	              /* ==========================================
	                 5. Send PWM to ESC
	                 ========================================== */
	              __HAL_TIM_SET_COMPARE(&htim1,
	                                    TIM_CHANNEL_1,
	                                    esc_pulse);

	              /* ==========================================
	                 6. Display ADC and PWM on UART
	                 ========================================== */
	              sprintf(buffer,
	                      "ADC = %lu | ESC PWM = %u us\r\n",
	                      adc_value,
	                      esc_pulse);

	              HAL_UART_Transmit(&huart2,
	                                (uint8_t *)buffer,
	                                strlen(buffer),
	                                HAL_MAX_DELAY);
	          }

	          /* ==========================================
	             7. Stop ADC
	             ========================================== */
	          HAL_ADC_Stop(&hadc1);
	      }

	      /* ==========================================
	         8. Small delay
	         ========================================== */
	      HAL_Delay(100);
	  }


	  /* Start TIM1 PWM */
	  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
	  {
	      Error_Handler();
	  }

	  /* Minimum throttle */
	  __HAL_TIM_SET_COMPARE(&htim1,
	                        TIM_CHANNEL_1,
	                        1000);

	  /* Wait for ESC arming */
	  HAL_Delay(5000);


    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
