#include "mpu6050.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define MPU6050_I2C_ADDR       (0x68U << 1)

#define MPU6050_WHO_AM_I       0x75U
#define MPU6050_PWR_MGMT_1     0x6BU
#define MPU6050_SMPLRT_DIV     0x19U
#define MPU6050_CONFIG         0x1AU
#define MPU6050_GYRO_CONFIG    0x1BU
#define MPU6050_ACCEL_CONFIG   0x1CU
#define MPU6050_ACCEL_XOUT_H   0x3BU
#define MPU6050_TEMP_OUT_H     0x41U
#define MPU6050_GYRO_XOUT_H    0x43U

#define I2C_TIMEOUT             100U

/*
 * MPU6050 configuration:
 *
 * Accelerometer = +/-2g
 * Sensitivity   = 16384 LSB/g
 *
 * Gyroscope = +/-250 deg/s
 * Sensitivity = 131 LSB/(deg/s)
 */

#define ACCEL_SENSITIVITY       16384.0
#define GYRO_SENSITIVITY        131.0

#define RAD_TO_DEG              57.29577951308232


/* ========================================================================== */
/*                         KALMAN FILTER                                     */
/* ========================================================================== */

static Kalman_t KalmanX =
{
    .Q_angle = 0.001,
    .Q_bias = 0.003,
    .R_measure = 0.03,

    .angle = 0.0,
    .bias = 0.0,

    .P =
    {
        {0.0, 0.0},
        {0.0, 0.0}
    }
};


static Kalman_t KalmanY =
{
    .Q_angle = 0.001,
    .Q_bias = 0.003,
    .R_measure = 0.03,

    .angle = 0.0,
    .bias = 0.0,

    .P =
    {
        {0.0, 0.0},
        {0.0, 0.0}
    }
};


/* ========================================================================== */
/*                            UART                                            */
/* ========================================================================== */

static void MPU_UART_Send(const char *text)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)text,
        strlen(text),
        100
    );
}


static void MPU_UART_SendHex(
    const char *name,
    uint8_t value
)
{
    char buffer[64];

    snprintf(
        buffer,
        sizeof(buffer),
        "%s = 0x%02X\r\n",
        name,
        value
    );

    MPU_UART_Send(buffer);
}


static void MPU_UART_SendError(
    const char *name,
    HAL_StatusTypeDef status
)
{
    char buffer[100];

    snprintf(
        buffer,
        sizeof(buffer),
        "MPU6050 ERROR: %s STATUS=%d\r\n",
        name,
        (int)status
    );

    MPU_UART_Send(buffer);
}


/* ========================================================================== */
/*                         MPU6050 INIT                                      */
/* ========================================================================== */

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t who_am_i = 0;
    uint8_t value = 0;

    HAL_StatusTypeDef status;


    MPU_UART_Send("\r\n");
    MPU_UART_Send("========================================\r\n");
    MPU_UART_Send("        MPU6050 DIAGNOSTIC\r\n");
    MPU_UART_Send("========================================\r\n");


    /* ---------------------------------------------------------------------- */
    /* STEP 1 - I2C DEVICE                                                    */
    /* ---------------------------------------------------------------------- */

    MPU_UART_Send("Checking I2C device 0x68...\r\n");

    status = HAL_I2C_IsDeviceReady(
        I2Cx,
        MPU6050_I2C_ADDR,
        3,
        I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        MPU_UART_Send("I2C DEVICE NOT FOUND\r\n");

        MPU_UART_SendError(
            "HAL_I2C_IsDeviceReady",
            status
        );

        return 1;
    }

    MPU_UART_Send("I2C DEVICE 0x68 FOUND\r\n");


    /* ---------------------------------------------------------------------- */
    /* STEP 2 - WHO AM I                                                      */
    /* ---------------------------------------------------------------------- */

    status = HAL_I2C_Mem_Read(
        I2Cx,
        MPU6050_I2C_ADDR,
        MPU6050_WHO_AM_I,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i,
        1,
        I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        MPU_UART_SendError(
            "WHO_AM_I READ",
            status
        );

        return 2;
    }

    MPU_UART_SendHex(
        "WHO_AM_I",
        who_am_i
    );


    if (who_am_i != 0x68)
    {
        MPU_UART_Send(
            "WARNING: WHO_AM_I is not 0x68\r\n"
        );

        return 2;
    }

    MPU_UART_Send(
        "WHO_AM_I OK\r\n"
    );


    /* ---------------------------------------------------------------------- */
    /* STEP 3 - WAKE UP                                                       */
    /* ---------------------------------------------------------------------- */

    value = 0x00;

    status = HAL_I2C_Mem_Write(
        I2Cx,
        MPU6050_I2C_ADDR,
        MPU6050_PWR_MGMT_1,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        MPU_UART_SendError(
            "PWR_MGMT_1 WRITE",
            status
        );

        return 3;
    }

    HAL_Delay(100);


    /* ---------------------------------------------------------------------- */
    /* STEP 4 - SAMPLE RATE                                                   */
    /* ---------------------------------------------------------------------- */

    /*
     * 1 kHz / (1 + 7) = 125 Hz
     */

    value = 0x07;

    status = HAL_I2C_Mem_Write(
        I2Cx,
        MPU6050_I2C_ADDR,
        MPU6050_SMPLRT_DIV,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        MPU_UART_SendError(
            "SMPLRT_DIV WRITE",
            status
        );

        return 4;
    }


    /* ---------------------------------------------------------------------- */
    /* STEP 5 - DLPF                                                          */
    /* ---------------------------------------------------------------------- */

    value = 0x03;

    status = HAL_I2C_Mem_Write(
        I2Cx,
        MPU6050_I2C_ADDR,
        MPU6050_CONFIG,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        MPU_UART_SendError(
            "CONFIG WRITE",
            status
        );

        return 5;
    }


    /* ---------------------------------------------------------------------- */
    /* STEP 6 - GYRO +/-250 deg/s                                             */
    /* ---------------------------------------------------------------------- */

    value = 0x00;

    status = HAL_I2C_Mem_Write(
        I2Cx,
        MPU6050_I2C_ADDR,
        MPU6050_GYRO_CONFIG,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        MPU_UART_SendError(
            "GYRO_CONFIG WRITE",
            status
        );

        return 6;
    }


    /* ---------------------------------------------------------------------- */
    /* STEP 7 - ACCEL +/-2g                                                   */
    /* ---------------------------------------------------------------------- */

    value = 0x00;

    status = HAL_I2C_Mem_Write(
        I2Cx,
        MPU6050_I2C_ADDR,
        MPU6050_ACCEL_CONFIG,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        I2C_TIMEOUT
    );

    if (status != HAL_OK)
    {
        MPU_UART_SendError(
            "ACCEL_CONFIG WRITE",
            status
        );

        return 7;
    }


    MPU_UART_Send(
        "MPU6050 CONFIGURATION OK\r\n"
    );

    MPU_UART_Send(
        "Accelerometer: +/-2g = 16384 LSB/g\r\n"
    );

    MPU_UART_Send(
        "Gyroscope: +/-250 deg/s = 131 LSB/(deg/s)\r\n"
    );

    MPU_UART_Send(
        "========================================\r\n"
    );


    return 0;
}


/* ========================================================================== */
/*                       READ ACCELEROMETER                                  */
/* ========================================================================== */

void MPU6050_Read_Accel(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t buffer[6];

    if (HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_I2C_ADDR,
            MPU6050_ACCEL_XOUT_H,
            I2C_MEMADD_SIZE_8BIT,
            buffer,
            6,
            I2C_TIMEOUT
        ) != HAL_OK)
    {
        return;
    }


    DataStruct->Accel_X_RAW =
        (int16_t)((buffer[0] << 8) | buffer[1]);

    DataStruct->Accel_Y_RAW =
        (int16_t)((buffer[2] << 8) | buffer[3]);

    DataStruct->Accel_Z_RAW =
        (int16_t)((buffer[4] << 8) | buffer[5]);


    /*
     * Conversion RAW -> g
     */

    DataStruct->Ax =
        (double)DataStruct->Accel_X_RAW /
        ACCEL_SENSITIVITY;

    DataStruct->Ay =
        (double)DataStruct->Accel_Y_RAW /
        ACCEL_SENSITIVITY;

    DataStruct->Az =
        (double)DataStruct->Accel_Z_RAW /
        ACCEL_SENSITIVITY;
}


/* ========================================================================== */
/*                          READ GYRO                                        */
/* ========================================================================== */

void MPU6050_Read_Gyro(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t buffer[6];

    if (HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_I2C_ADDR,
            MPU6050_GYRO_XOUT_H,
            I2C_MEMADD_SIZE_8BIT,
            buffer,
            6,
            I2C_TIMEOUT
        ) != HAL_OK)
    {
        return;
    }


    DataStruct->Gyro_X_RAW =
        (int16_t)((buffer[0] << 8) | buffer[1]);

    DataStruct->Gyro_Y_RAW =
        (int16_t)((buffer[2] << 8) | buffer[3]);

    DataStruct->Gyro_Z_RAW =
        (int16_t)((buffer[4] << 8) | buffer[5]);


    /*
     * Conversion RAW -> deg/s
     */

    DataStruct->Gx =
        (double)DataStruct->Gyro_X_RAW /
        GYRO_SENSITIVITY;

    DataStruct->Gy =
        (double)DataStruct->Gyro_Y_RAW /
        GYRO_SENSITIVITY;

    DataStruct->Gz =
        (double)DataStruct->Gyro_Z_RAW /
        GYRO_SENSITIVITY;
}


/* ========================================================================== */
/*                           READ TEMPERATURE                                */
/* ========================================================================== */

void MPU6050_Read_Temp(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t buffer[2];
    int16_t temp_raw;


    if (HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_I2C_ADDR,
            MPU6050_TEMP_OUT_H,
            I2C_MEMADD_SIZE_8BIT,
            buffer,
            2,
            I2C_TIMEOUT
        ) != HAL_OK)
    {
        return;
    }


    temp_raw =
        (int16_t)((buffer[0] << 8) | buffer[1]);


    DataStruct->Temperature =
        ((float)temp_raw / 340.0f) + 36.53f;
}


/* ========================================================================== */
/*                         READ ALL                                          */
/* ========================================================================== */

void MPU6050_Read_All(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t buffer[14];

    double Accel_Angle_X;
    double Accel_Angle_Y;

    double Gyro_Rate_X;
    double Gyro_Rate_Y;


    if (HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_I2C_ADDR,
            MPU6050_ACCEL_XOUT_H,
            I2C_MEMADD_SIZE_8BIT,
            buffer,
            14,
            I2C_TIMEOUT
        ) != HAL_OK)
    {
        return;
    }


    /* ---------------------------------------------------------------------- */
    /* ACCEL RAW                                                              */
    /* ---------------------------------------------------------------------- */

    DataStruct->Accel_X_RAW =
        (int16_t)((buffer[0] << 8) | buffer[1]);

    DataStruct->Accel_Y_RAW =
        (int16_t)((buffer[2] << 8) | buffer[3]);

    DataStruct->Accel_Z_RAW =
        (int16_t)((buffer[4] << 8) | buffer[5]);


    /* ---------------------------------------------------------------------- */
    /* TEMPERATURE                                                            */
    /* ---------------------------------------------------------------------- */

    DataStruct->Temperature =
        ((float)(
            (int16_t)((buffer[6] << 8) | buffer[7])
        ) / 340.0f) + 36.53f;


    /* ---------------------------------------------------------------------- */
    /* GYRO RAW                                                               */
    /* ---------------------------------------------------------------------- */

    DataStruct->Gyro_X_RAW =
        (int16_t)((buffer[8] << 8) | buffer[9]);

    DataStruct->Gyro_Y_RAW =
        (int16_t)((buffer[10] << 8) | buffer[11]);

    DataStruct->Gyro_Z_RAW =
        (int16_t)((buffer[12] << 8) | buffer[13]);


    /* ---------------------------------------------------------------------- */
    /* ACCEL -> g                                                              */
    /* ---------------------------------------------------------------------- */

    DataStruct->Ax =
        (double)DataStruct->Accel_X_RAW /
        ACCEL_SENSITIVITY;

    DataStruct->Ay =
        (double)DataStruct->Accel_Y_RAW /
        ACCEL_SENSITIVITY;

    DataStruct->Az =
        (double)DataStruct->Accel_Z_RAW /
        ACCEL_SENSITIVITY;


    /* ---------------------------------------------------------------------- */
    /* GYRO -> deg/s                                                          */
    /* ---------------------------------------------------------------------- */

    DataStruct->Gx =
        (double)DataStruct->Gyro_X_RAW /
        GYRO_SENSITIVITY;

    DataStruct->Gy =
        (double)DataStruct->Gyro_Y_RAW /
        GYRO_SENSITIVITY;

    DataStruct->Gz =
        (double)DataStruct->Gyro_Z_RAW /
        GYRO_SENSITIVITY;


    /* ---------------------------------------------------------------------- */
    /* ACCELEROMETER ANGLES                                                   */
    /* ---------------------------------------------------------------------- */

    Accel_Angle_X =
        atan2(
            DataStruct->Ay,
            sqrt(
                DataStruct->Ax * DataStruct->Ax +
                DataStruct->Az * DataStruct->Az
            )
        ) * RAD_TO_DEG;


    Accel_Angle_Y =
        atan2(
            -DataStruct->Ax,
            sqrt(
                DataStruct->Ay * DataStruct->Ay +
                DataStruct->Az * DataStruct->Az
            )
        ) * RAD_TO_DEG;


    Gyro_Rate_X = DataStruct->Gx;
    Gyro_Rate_Y = DataStruct->Gy;


    /* ---------------------------------------------------------------------- */
    /* KALMAN                                                                  */
    /* ---------------------------------------------------------------------- */

    DataStruct->KalmanAngleX =
        Kalman_getAngle(
            &KalmanX,
            Accel_Angle_X,
            Gyro_Rate_X,
            0.01
        );


    DataStruct->KalmanAngleY =
        Kalman_getAngle(
            &KalmanY,
            Accel_Angle_Y,
            Gyro_Rate_Y,
            0.01
        );
}


/* ========================================================================== */
/*                         KALMAN FILTER                                     */
/* ========================================================================== */

double Kalman_getAngle(
    Kalman_t *Kalman,
    double newAngle,
    double newRate,
    double dt
)
{
    double rate;

    double S;

    double K[2];

    double y;

    double P00_temp;
    double P01_temp;


    /* Prediction */

    rate =
        newRate -
        Kalman->bias;


    Kalman->angle +=
        dt * rate;


    Kalman->P[0][0] +=
        dt * (
            dt * Kalman->P[1][1]
            - Kalman->P[0][1]
            - Kalman->P[1][0]
            + Kalman->Q_angle
        );


    Kalman->P[0][1] -=
        dt * Kalman->P[1][1];


    Kalman->P[1][0] -=
        dt * Kalman->P[1][1];


    Kalman->P[1][1] +=
        Kalman->Q_bias * dt;


    /* Measurement update */

    S =
        Kalman->P[0][0] +
        Kalman->R_measure;


    K[0] =
        Kalman->P[0][0] / S;


    K[1] =
        Kalman->P[1][0] / S;


    y =
        newAngle -
        Kalman->angle;


    Kalman->angle +=
        K[0] * y;


    Kalman->bias +=
        K[1] * y;


    P00_temp =
        Kalman->P[0][0];


    P01_temp =
        Kalman->P[0][1];


    Kalman->P[0][0] -=
        K[0] * P00_temp;


    Kalman->P[0][1] -=
        K[0] * P01_temp;


    Kalman->P[1][0] -=
        K[1] * P00_temp;


    Kalman->P[1][1] -=
        K[1] * P01_temp;


    return Kalman->angle;
}
