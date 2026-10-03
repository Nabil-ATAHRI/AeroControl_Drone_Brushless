/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ADC + ESC + MPU6050 + RGB LED
  *
  *                    STM32F407VGT6
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

#include "mpu6050.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* RGB LED
 *
 * RED   = PA8
 * GREEN = PC8
 * BLUE  = PC6
 *
 * Common cathode:
 * HIGH = ON
 * LOW  = OFF
 */

/* ADC thresholds */
#define ADC_BLUE_MAX       999U
#define ADC_GREEN_MAX      3000U

/* ESC limits */
#define ESC_MIN_US         1000U
#define ESC_MAX_US         1500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

MPU6050_t MPU6050;

uint32_t AD_RES = 0;

uint32_t ESC_Pulse = ESC_MIN_US;

uint8_t mpu_ok = 0;

uint8_t mpu_error = 0;

uint8_t motor_running = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

static void UART_Send_Text(const char *text);

static uint32_t ADC_Read(void);

static uint32_t ADC_To_ESC(uint32_t adc);

static void ESC_SetPulse(uint32_t pulse_us);

static void ESC_Arm(void);

static void RGB_SetColor(
    uint8_t red,
    uint8_t green,
    uint8_t blue
);

static void RGB_UpdateFromADC(uint32_t adc);

static void MPU_Diagnostic_Read(void);

static void Send_Status(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ========================================================================== */
/* UART                                                                       */
/* ========================================================================== */

static void UART_Send_Text(const char *text)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)text,
        strlen(text),
        100
    );
}


/* ========================================================================== */
/* ADC                                                                        */
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

static uint32_t ADC_To_ESC(uint32_t adc)
{
    uint32_t pulse;

    if (adc > 4095U)
    {
        adc = 4095U;
    }

    /*
     * ADC = 0
     *     -> 1000 us
     *
     * ADC = 4095
     *     -> 1500 us
     */

    pulse =
        ESC_MIN_US +
        ((adc * (ESC_MAX_US - ESC_MIN_US)) / 4095U);

    return pulse;
}


/* ========================================================================== */
/* ESC PWM                                                                    */
/* ========================================================================== */

static void ESC_SetPulse(uint32_t pulse_us)
{
    if (pulse_us < ESC_MIN_US)
    {
        pulse_us = ESC_MIN_US;
    }

    if (pulse_us > ESC_MAX_US)
    {
        pulse_us = ESC_MAX_US;
    }

    ESC_Pulse = pulse_us;

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        pulse_us
    );

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
/* ESC ARMING                                                                 */
/* ========================================================================== */

static void ESC_Arm(void)
{
    UART_Send_Text(
        "\r\n"
        "========================================\r\n"
        "              ESC ARMING\r\n"
        "========================================\r\n"
    );

    ESC_SetPulse(ESC_MIN_US);

    UART_Send_Text(
        "ESC = 1000 us\r\n"
    );

    HAL_Delay(5000);

    UART_Send_Text(
        "ESC ARMING DONE\r\n"
    );

    UART_Send_Text(
        "========================================\r\n"
    );
}


/* ========================================================================== */
/* RGB LED                                                                    */
/* ========================================================================== */

static void RGB_SetColor(
    uint8_t red,
    uint8_t green,
    uint8_t blue
)
{
    /* RED = PA8 */

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_8,
        red ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    /* GREEN = PC8 */

    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_8,
        green ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    /* BLUE = PC6 */

    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_6,
        blue ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}


/* ========================================================================== */
/* RGB FROM ADC                                                               */
/* ========================================================================== */

static void RGB_UpdateFromADC(uint32_t adc)
{
    /*
     * ADC < 1000
     *     -> BLUE
     *
     * ADC 1000 ... 3000
     *     -> GREEN
     *
     * ADC > 3000
     *     -> RED
     */

    if (adc < 1000U)
    {
        RGB_SetColor(
            0,
            0,
            1
        );
    }
    else if (adc <= 3000U)
    {
        RGB_SetColor(
            0,
            1,
            0
        );
    }
    else
    {
        RGB_SetColor(
            1,
            0,
            0
        );
    }
}


/* ========================================================================== */
/* MPU6050 DIAGNOSTIC                                                         */
/* ========================================================================== */

static void MPU_Diagnostic_Read(void)
{
    uint8_t who_am_i = 0;
    uint8_t pwr = 0;
    uint8_t sample = 0;
    uint8_t config = 0;
    uint8_t gyro_config = 0;
    uint8_t accel_config = 0;

    char msg[300];

    /* WHO_AM_I */

    if (HAL_I2C_Mem_Read(
            &hi2c1,
            MPU6050_ADDR,
            WHO_AM_I_REG,
            I2C_MEMADD_SIZE_8BIT,
            &who_am_i,
            1,
            100
        ) != HAL_OK)
    {
        UART_Send_Text(
            "WHO_AM_I READ ERROR\r\n"
        );

        return;
    }

    /* PWR_MGMT_1 */

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        PWR_MGMT_1_REG,
        I2C_MEMADD_SIZE_8BIT,
        &pwr,
        1,
        100
    );

    /* SMPLRT_DIV */

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        SMPLRT_DIV_REG,
        I2C_MEMADD_SIZE_8BIT,
        &sample,
        1,
        100
    );

    /* CONFIG */

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &config,
        1,
        100
    );

    /* GYRO_CONFIG */

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        GYRO_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &gyro_config,
        1,
        100
    );

    /* ACCEL_CONFIG */

    HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        ACCEL_CONFIG_REG,
        I2C_MEMADD_SIZE_8BIT,
        &accel_config,
        1,
        100
    );

    snprintf(
        msg,
        sizeof(msg),

        "\r\n"
        "========== MPU6050 DIAGNOSTIC ==========\r\n"
        "WHO_AM_I     = 0x%02X\r\n"
        "PWR_MGMT_1   = 0x%02X\r\n"
        "SMPLRT_DIV   = 0x%02X\r\n"
        "CONFIG       = 0x%02X\r\n"
        "GYRO_CONFIG  = 0x%02X\r\n"
        "ACCEL_CONFIG = 0x%02X\r\n"
        "=========================================\r\n",

        who_am_i,
        pwr,
        sample,
        config,
        gyro_config,
        accel_config
    );

    UART_Send_Text(msg);
}


/* ========================================================================== */
/* UART STATUS                                                                */
/* ========================================================================== */

static void Send_Status(void)
{
    char msg[500];

    if (mpu_ok)
    {
        /*
         * Ax, Ay, Az sont en g.
         * Conversion en mg :
         *
         * 1 g = 1000 mg
         */

        double Ax_mg = MPU6050.Ax * 1000.0;
        double Ay_mg = MPU6050.Ay * 1000.0;
        double Az_mg = MPU6050.Az * 1000.0;

        snprintf(
            msg,
            sizeof(msg),

            "ADC=%lu | "
            "ESC=%lu us | "
            "MOTOR=%d | "
            "RGB=%s\r\n"

            "ACC: "
            "X=%.0f mg | "
            "Y=%.0f mg | "
            "Z=%.0f mg\r\n"

            "GYRO: "
            "X=%.2f deg/s | "
            "Y=%.2f deg/s | "
            "Z=%.2f deg/s\r\n"

            "TEMP=%.2f C | "
            "ROLL=%.2f deg | "
            "PITCH=%.2f deg\r\n"

            "RAW ACC: "
            "X=%d | "
            "Y=%d | "
            "Z=%d\r\n"

            "----------------------------------------\r\n",

            (unsigned long)AD_RES,

            (unsigned long)ESC_Pulse,

            motor_running,

            (AD_RES < 1000U)
                ? "BLUE"
                : ((AD_RES <= 3000U)
                    ? "GREEN"
                    : "RED"),

            Ax_mg,
            Ay_mg,
            Az_mg,

            MPU6050.Gx,
            MPU6050.Gy,
            MPU6050.Gz,

            MPU6050.Temperature,

            MPU6050.KalmanAngleX,
            MPU6050.KalmanAngleY,

            MPU6050.Accel_X_RAW,
            MPU6050.Accel_Y_RAW,
            MPU6050.Accel_Z_RAW
        );
    }
    else
    {
        snprintf(
            msg,
            sizeof(msg),

            "ADC=%lu | "
            "MPU ERROR=%u | "
            "ESC=%lu us | "
            "MOTOR=%d | "
            "RGB=%s\r\n",

            (unsigned long)AD_RES,

            mpu_error,

            (unsigned long)ESC_Pulse,

            motor_running,

            (AD_RES < 1000U)
                ? "BLUE"
                : ((AD_RES <= 3000U)
                    ? "GREEN"
                    : "RED")
        );
    }

    UART_Send_Text(msg);
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

    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */

    MX_GPIO_Init();

    MX_ADC1_Init();

    MX_I2C1_Init();

    MX_TIM1_Init();

    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    HAL_Delay(500);

    UART_Send_Text(
        "\r\n"
        "========================================\r\n"
        "          STM32F407VGT6\r\n"
        "========================================\r\n"
        "      ADC + ESC + MPU6050 + RGB\r\n"
        "========================================\r\n"
    );

    /* RGB startup = BLUE */

    RGB_SetColor(
        0,
        0,
        1
    );

    UART_Send_Text(
        "RGB = BLUE\r\n"
    );

    /* Start TIM1 PWM */

    if (HAL_TIM_PWM_Start(
            &htim1,
            TIM_CHANNEL_1
        ) != HAL_OK)
    {
        UART_Send_Text(
            "ERROR: TIM1 PWM START\r\n"
        );

        Error_Handler();
    }

    UART_Send_Text(
        "TIM1 PWM OK\r\n"
    );

    /* ESC arming */

    ESC_Arm();

    /* MPU6050 */

    UART_Send_Text(
        "Initializing MPU6050...\r\n"
    );

    mpu_error = MPU6050_Init(&hi2c1);

    if (mpu_error == 0)
    {
        mpu_ok = 1;

        UART_Send_Text(
            "MPU6050 READY\r\n"
        );

        UART_Send_Text(
            "ACCEL = mg\r\n"
        );
    }
    else
    {
        mpu_ok = 0;

        UART_Send_Text(
            "MPU6050 ERROR\r\n"
        );

        MPU_Diagnostic_Read();
    }

    /* ESC initial value */

    ESC_SetPulse(
        ESC_MIN_US
    );

    /* USER CODE END 2 */

    /* Infinite loop */

    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /* Read ADC */

        AD_RES = ADC_Read();

        /* Update RGB */

        RGB_UpdateFromADC(
            AD_RES
        );

        /* ADC -> ESC */

        ESC_Pulse = ADC_To_ESC(
            AD_RES
        );

        ESC_SetPulse(
            ESC_Pulse
        );

        /* Read MPU6050 */

        if (mpu_ok)
        {
            MPU6050_Read_All(
                &hi2c1,
                &MPU6050
            );
        }

        /* UART */

        Send_Status();

        HAL_Delay(100);

        /* USER CODE END 3 */
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM =
        8;

    RCC_OscInitStruct.PLL.PLLN =
        336;

    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;

    RCC_OscInitStruct.PLL.PLLQ =
        7;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

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


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        ESC_MIN_US
    );

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
  */
void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
    /* USER CODE BEGIN 6 */

    (void)file;
    (void)line;

    /* USER CODE END 6 */
}

#endif
