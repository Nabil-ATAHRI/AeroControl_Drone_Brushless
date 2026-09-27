/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F407VGT6 - ADC + ESC + MPU6050 + UART
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
#include "mpu6050.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>


/* Private variables ---------------------------------------------------------*/

MPU6050_t MPU6050;

uint32_t AD_RES = 0;

uint32_t ESC_Pulse = 1000;

uint8_t mpu_ok = 0;

uint8_t mpu_error = 0;

uint8_t motor_running = 0;


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

void Error_Handler(void);

static void UART_Send_Text(const char *text);

static uint32_t ADC_Read(void);

static uint32_t ADC_To_ESC(uint32_t adc_value);

static void ESC_SetPulse(uint32_t pulse_us);

static void ESC_Arm(void);

static void UART_Send_Status(void);


/* ========================================================================== */
/* UART SEND TEXT                                                             */
/* ========================================================================== */

static void UART_Send_Text(const char *text)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        HAL_MAX_DELAY
    );
}


/* ========================================================================== */
/* ADC READ                                                                   */
/* ========================================================================== */

static uint32_t ADC_Read(void)
{
    uint32_t value = 0;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        value = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    return value;
}


/* ========================================================================== */
/* ADC -> ESC                                                                 */
/* ========================================================================== */

/*
 * ADC = 0       -> 1000 us
 * ADC = 4095    -> 1500 us
 *
 * Pour les premiers tests :
 *
 * 1000 us = minimum
 * 1500 us = maximum de test
 */

static uint32_t ADC_To_ESC(uint32_t adc_value)
{
    uint32_t pulse;

    if (adc_value > 4095U)
    {
        adc_value = 4095U;
    }

    pulse =
        1000U +
        ((adc_value * 500U) / 4095U);

    if (pulse < 1000U)
    {
        pulse = 1000U;
    }

    if (pulse > 1500U)
    {
        pulse = 1500U;
    }

    return pulse;
}


/* ========================================================================== */
/* ESC SET PULSE                                                              */
/* ========================================================================== */

static void ESC_SetPulse(uint32_t pulse_us)
{
    /*
     * Sécurité minimum
     */

    if (pulse_us < 1000U)
    {
        pulse_us = 1000U;
    }


    /*
     * Limite pour les premiers tests
     */

    if (pulse_us > 1500U)
    {
        pulse_us = 1500U;
    }


    /*
     * TIM1 CH1
     *
     * TIM1 = 168 MHz
     * Prescaler = 167
     *
     * Timer frequency = 1 MHz
     *
     * 1 tick = 1 us
     */

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        pulse_us
    );


    ESC_Pulse = pulse_us;


    /*
     * Etat du moteur
     */

    if (pulse_us > 1050U)
    {
        motor_running = 1;
    }
    else
    {
        motor_running = 0;
    }
}


/* ========================================================================== */
/* ESC ARM                                                                    */
/* ========================================================================== */

static void ESC_Arm(void)
{
    UART_Send_Text(
        "\r\n"
        "========================================\r\n"
        "             ESC ARMING                \r\n"
        "========================================\r\n"
    );


    /*
     * Minimum throttle
     */

    ESC_SetPulse(1000U);


    UART_Send_Text(
        "ESC PWM = 1000 us\r\n"
    );


    UART_Send_Text(
        "Waiting 5 seconds...\r\n"
    );


    /*
     * Attendre l'initialisation de l'ESC
     */

    HAL_Delay(5000);


    UART_Send_Text(
        "ESC READY\r\n"
    );
}


/* ========================================================================== */
/* UART STATUS                                                                */
/* ========================================================================== */

static void UART_Send_Status(void)
{
    char message[200];


    if (mpu_ok == 1)
    {
        snprintf(
            message,
            sizeof(message),

            "ADC=%lu | "
            "Roll=%.2f | "
            "Pitch=%.2f | "
            "ESC=%lu us | "
            "MOTOR=%u\r\n",

            (unsigned long)AD_RES,

            MPU6050.KalmanAngleX,

            MPU6050.KalmanAngleY,

            (unsigned long)ESC_Pulse,

            motor_running
        );
    }
    else
    {
        snprintf(
            message,
            sizeof(message),

            "ADC=%lu | "
            "MPU ERROR=%u | "
            "ESC=%lu us | "
            "MOTOR=%u\r\n",

            (unsigned long)AD_RES,

            mpu_error,

            (unsigned long)ESC_Pulse,

            motor_running
        );
    }


    UART_Send_Text(message);
}


/* ========================================================================== */
/* MAIN                                                                       */
/* ========================================================================== */

int main(void)
{
    uint32_t loop_counter = 0;


    /* ====================================================================== */
    /* HAL INITIALIZATION                                                     */
    /* ====================================================================== */

    HAL_Init();


    /* ====================================================================== */
    /* SYSTEM CLOCK                                                            */
    /* ====================================================================== */

    SystemClock_Config();


    /* ====================================================================== */
    /* GPIO                                                                    */
    /* ====================================================================== */

    MX_GPIO_Init();


    /* ====================================================================== */
    /* I2C1                                                                    */
    /* ====================================================================== */

    MX_I2C1_Init();


    /* ====================================================================== */
    /* TIM1                                                                    */
    /* ====================================================================== */

    MX_TIM1_Init();


    /* ====================================================================== */
    /* USART2                                                                  */
    /* ====================================================================== */

    MX_USART2_UART_Init();


    /* ====================================================================== */
    /* ADC1                                                                    */
    /* ====================================================================== */

    MX_ADC1_Init();


    /* ====================================================================== */
    /* WAIT                                                                    */
    /* ====================================================================== */

    HAL_Delay(500);


    /* ====================================================================== */
    /* START MESSAGE                                                           */
    /* ====================================================================== */

    UART_Send_Text(
        "\r\n"
        "\r\n"
        "========================================\r\n"
        "        STM32F407VGT6                  \r\n"
        "        ADC + ESC + MPU6050            \r\n"
        "========================================\r\n"
        "\r\n"
    );


    /* ====================================================================== */
    /* START TIM1 PWM                                                          */
    /* ====================================================================== */

    UART_Send_Text(
        "Starting TIM1 PWM...\r\n"
    );


    if (HAL_TIM_PWM_Start(
            &htim1,
            TIM_CHANNEL_1
        ) != HAL_OK)
    {
        UART_Send_Text(
            "ERROR: TIM1 PWM START FAILED\r\n"
        );

        Error_Handler();
    }


    UART_Send_Text(
        "TIM1 PWM OK\r\n"
    );


    /* ====================================================================== */
    /* INITIAL ESC VALUE                                                       */
    /* ====================================================================== */

    ESC_SetPulse(1000U);


    /* ====================================================================== */
    /* MPU6050 INITIALIZATION                                                  */
    /* ====================================================================== */

    UART_Send_Text(
        "\r\n"
        "MPU6050 initialization...\r\n"
    );


    mpu_error =
        MPU6050_Init(&hi2c1);


    if (mpu_error == 0)
    {
        mpu_ok = 1;

        UART_Send_Text(
            "MPU6050 OK\r\n"
        );
    }
    else
    {
        mpu_ok = 0;

        char error_message[120];


        snprintf(
            error_message,
            sizeof(error_message),

            "MPU6050 ERROR - CODE=%u\r\n",

            mpu_error
        );


        UART_Send_Text(
            error_message
        );


        UART_Send_Text(
            "WARNING: MPU6050 unavailable.\r\n"
        );


        UART_Send_Text(
            "ADC can still control the motor.\r\n"
        );
    }


    /* ====================================================================== */
    /* ESC ARM                                                                 */
    /* ====================================================================== */

    ESC_Arm();


    /* ====================================================================== */
    /* APPLICATION START                                                       */
    /* ====================================================================== */

    UART_Send_Text(
        "\r\n"
        "========================================\r\n"
        "        APPLICATION START               \r\n"
        "========================================\r\n"
    );


    UART_Send_Text(
        "ADC -> ESC -> MOTOR\r\n"
    );


    UART_Send_Text(
        "ADC = 0    -> ESC = 1000 us\r\n"
    );


    UART_Send_Text(
        "ADC = 4095 -> ESC = 1500 us\r\n"
    );


    UART_Send_Text(
        "MPU6050 = diagnostic sensor\r\n"
    );


    /* ====================================================================== */
    /* MAIN LOOP                                                               */
    /* ====================================================================== */

    while (1)
    {
        /* ================================================================== */
        /* READ ADC                                                            */
        /* ================================================================== */

        AD_RES = ADC_Read();


        /* ================================================================== */
        /* READ MPU6050                                                        */
        /* ================================================================== */

        if (mpu_ok == 1)
        {
            MPU6050_Read_All(
                &hi2c1,
                &MPU6050
            );
        }


        /* ================================================================== */
        /* ADC -> ESC                                                           */
        /* ================================================================== */

        ESC_Pulse =
            ADC_To_ESC(
                AD_RES
            );


        ESC_SetPulse(
            ESC_Pulse
        );


        /* ================================================================== */
        /* LED MOTOR STATUS                                                    */
        /* ================================================================== */

        if (motor_running == 1)
        {
            HAL_GPIO_WritePin(
                GPIOD,
                GPIO_PIN_12,
                GPIO_PIN_SET
            );
        }
        else
        {
            HAL_GPIO_WritePin(
                GPIOD,
                GPIO_PIN_12,
                GPIO_PIN_RESET
            );
        }


        /* ================================================================== */
        /* UART DEBUG                                                           */
        /* ================================================================== */

        if ((loop_counter % 10U) == 0U)
        {
            UART_Send_Status();
        }


        /* ================================================================== */
        /* LOOP COUNTER                                                        */
        /* ================================================================== */

        loop_counter++;


        /* ================================================================== */
        /* 20 ms CONTROL PERIOD                                                */
        /* ================================================================== */

        HAL_Delay(20);
    }
}


/* ========================================================================== */
/* SYSTEM CLOCK CONFIGURATION                                                 */
/* ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    /* ====================================================================== */
    /* POWER                                                                   */
    /* ====================================================================== */

    __HAL_RCC_PWR_CLK_ENABLE();


    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    /* ====================================================================== */
    /* HSE + PLL                                                               */
    /* ====================================================================== */

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;


    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;


    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;


    RCC_OscInitStruct.PLL.PLLM =
        4;


    RCC_OscInitStruct.PLL.PLLN =
        168;


    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;


    RCC_OscInitStruct.PLL.PLLQ =
        4;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK)
    {
        Error_Handler();
    }


    /* ====================================================================== */
    /* CLOCK CONFIGURATION                                                     */
    /* ====================================================================== */

    RCC_ClkInitStruct.ClockType =
          RCC_CLOCKTYPE_HCLK
        | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1
        | RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;


    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;


    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV4;


    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_5
        ) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* ERROR HANDLER                                                              */
/* ========================================================================== */

void Error_Handler(void)
{
    __disable_irq();


    /*
     * Sécurité ESC :
     * minimum throttle
     */

    if (htim1.Instance != NULL)
    {
        __HAL_TIM_SET_COMPARE(
            &htim1,
            TIM_CHANNEL_1,
            1000U
        );
    }


    while (1)
    {
        HAL_GPIO_TogglePin(
            GPIOD,
            GPIO_PIN_13
        );

        HAL_Delay(200);
    }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
