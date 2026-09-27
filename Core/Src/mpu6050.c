#include "mpu6050.h"

#include <math.h>


/* ============================================================
 * CONFIGURATION
 * ============================================================ */

#define I2C_TIMEOUT             100U

#define ACCEL_SENSITIVITY       16384.0
#define GYRO_SENSITIVITY        131.0

#define RAD_TO_DEG              57.29577951308232


/* ============================================================
 * KALMAN FILTER OBJECTS
 * ============================================================ */

Kalman_t KalmanX =
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


Kalman_t KalmanY =
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


/* ============================================================
 * KALMAN FILTER
 * ============================================================ */

double Kalman_getAngle(
    Kalman_t *Kalman,
    double newAngle,
    double newRate,
    double dt
)
{
    double rate;
    double S;
    double K0;
    double K1;
    double y;

    double P00_temp;
    double P01_temp;


    /* --------------------------------------------------------
     * Prediction
     * -------------------------------------------------------- */

    rate =
        newRate -
        Kalman->bias;


    Kalman->angle +=
        dt *
        rate;


    Kalman->P[0][0] +=
        dt *
        (
            dt *
            Kalman->P[1][1]
            -
            Kalman->P[0][1]
            -
            Kalman->P[1][0]
            +
            Kalman->Q_angle
        );


    Kalman->P[0][1] -=
        dt *
        Kalman->P[1][1];


    Kalman->P[1][0] -=
        dt *
        Kalman->P[1][1];


    Kalman->P[1][1] +=
        Kalman->Q_bias *
        dt;


    /* --------------------------------------------------------
     * Measurement update
     * -------------------------------------------------------- */

    S =
        Kalman->P[0][0] +
        Kalman->R_measure;


    K0 =
        Kalman->P[0][0] /
        S;


    K1 =
        Kalman->P[1][0] /
        S;


    y =
        newAngle -
        Kalman->angle;


    Kalman->angle +=
        K0 *
        y;


    Kalman->bias +=
        K1 *
        y;


    P00_temp =
        Kalman->P[0][0];


    P01_temp =
        Kalman->P[0][1];


    Kalman->P[0][0] -=
        K0 *
        P00_temp;


    Kalman->P[0][1] -=
        K0 *
        P01_temp;


    Kalman->P[1][0] -=
        K1 *
        P00_temp;


    Kalman->P[1][1] -=
        K1 *
        P01_temp;


    return Kalman->angle;
}


/* ============================================================
 * MPU6050 INITIALIZATION
 * ============================================================ */

uint8_t MPU6050_Init(
    I2C_HandleTypeDef *I2Cx
)
{
    uint8_t check;
    uint8_t data;


    /* ========================================================
     * CHECK MPU6050
     * ======================================================== */

    if (
        HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_ADDR,
            WHO_AM_I_REG,
            I2C_MEMADD_SIZE_8BIT,
            &check,
            1,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return 1;
    }


    /* ========================================================
     * EXPECTED VALUE
     *
     * Original MPU6050:
     *
     * WHO_AM_I = 0x68
     * ======================================================== */

    if (check != 0x68U)
    {
        return 2;
    }


    /* ========================================================
     * WAKE UP MPU6050
     * ======================================================== */

    data = 0x00U;


    if (
        HAL_I2C_Mem_Write(
            I2Cx,
            MPU6050_ADDR,
            PWR_MGMT_1_REG,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return 3;
    }


    HAL_Delay(100);


    /* ========================================================
     * SAMPLE RATE
     *
     * 1 kHz / (1 + 7) = 125 Hz
     * ======================================================== */

    data = 0x07U;


    if (
        HAL_I2C_Mem_Write(
            I2Cx,
            MPU6050_ADDR,
            SMPLRT_DIV_REG,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return 4;
    }


    /* ========================================================
     * DLPF CONFIGURATION
     * ======================================================== */

    data = 0x03U;


    if (
        HAL_I2C_Mem_Write(
            I2Cx,
            MPU6050_ADDR,
            CONFIG_REG,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return 5;
    }


    /* ========================================================
     * GYROSCOPE
     *
     * ±250 degrees/s
     * ======================================================== */

    data = 0x00U;


    if (
        HAL_I2C_Mem_Write(
            I2Cx,
            MPU6050_ADDR,
            GYRO_CONFIG_REG,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return 6;
    }


    /* ========================================================
     * ACCELEROMETER
     *
     * ±2g
     * ======================================================== */

    data = 0x00U;


    if (
        HAL_I2C_Mem_Write(
            I2Cx,
            MPU6050_ADDR,
            ACCEL_CONFIG_REG,
            I2C_MEMADD_SIZE_8BIT,
            &data,
            1,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return 7;
    }


    HAL_Delay(100);


    return 0;
}


/* ============================================================
 * READ ACCELEROMETER
 * ============================================================ */

void MPU6050_Read_Accel(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t Rec_Data[6];


    if (
        HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_ADDR,
            ACCEL_XOUT_H_REG,
            I2C_MEMADD_SIZE_8BIT,
            Rec_Data,
            6,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return;
    }


    DataStruct->Accel_X_RAW =
        (int16_t)
        (
            (Rec_Data[0] << 8) |
            Rec_Data[1]
        );


    DataStruct->Accel_Y_RAW =
        (int16_t)
        (
            (Rec_Data[2] << 8) |
            Rec_Data[3]
        );


    DataStruct->Accel_Z_RAW =
        (int16_t)
        (
            (Rec_Data[4] << 8) |
            Rec_Data[5]
        );


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


/* ============================================================
 * READ GYROSCOPE
 * ============================================================ */

void MPU6050_Read_Gyro(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t Rec_Data[6];


    if (
        HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_ADDR,
            GYRO_XOUT_H_REG,
            I2C_MEMADD_SIZE_8BIT,
            Rec_Data,
            6,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return;
    }


    DataStruct->Gyro_X_RAW =
        (int16_t)
        (
            (Rec_Data[0] << 8) |
            Rec_Data[1]
        );


    DataStruct->Gyro_Y_RAW =
        (int16_t)
        (
            (Rec_Data[2] << 8) |
            Rec_Data[3]
        );


    DataStruct->Gyro_Z_RAW =
        (int16_t)
        (
            (Rec_Data[4] << 8) |
            Rec_Data[5]
        );


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


/* ============================================================
 * READ TEMPERATURE
 * ============================================================ */

void MPU6050_Read_Temp(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t Rec_Data[2];

    int16_t temp_raw;


    if (
        HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_ADDR,
            TEMP_OUT_H_REG,
            I2C_MEMADD_SIZE_8BIT,
            Rec_Data,
            2,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return;
    }


    temp_raw =
        (int16_t)
        (
            (Rec_Data[0] << 8) |
            Rec_Data[1]
        );


    DataStruct->Temperature =
        ((float)temp_raw / 340.0f)
        + 36.53f;
}


/* ============================================================
 * READ ALL SENSOR DATA
 * ============================================================ */

void MPU6050_Read_All(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
)
{
    uint8_t Rec_Data[14];

    double dt = 0.02;

    double Accel_Angle_X;
    double Accel_Angle_Y;


    /* ========================================================
     * READ 14 BYTES
     *
     * ACCEL X/Y/Z
     * TEMP
     * GYRO X/Y/Z
     * ======================================================== */

    if (
        HAL_I2C_Mem_Read(
            I2Cx,
            MPU6050_ADDR,
            ACCEL_XOUT_H_REG,
            I2C_MEMADD_SIZE_8BIT,
            Rec_Data,
            14,
            I2C_TIMEOUT
        ) != HAL_OK
    )
    {
        return;
    }


    /* ========================================================
     * ACCELEROMETER
     * ======================================================== */

    DataStruct->Accel_X_RAW =
        (int16_t)
        (
            (Rec_Data[0] << 8) |
            Rec_Data[1]
        );


    DataStruct->Accel_Y_RAW =
        (int16_t)
        (
            (Rec_Data[2] << 8) |
            Rec_Data[3]
        );


    DataStruct->Accel_Z_RAW =
        (int16_t)
        (
            (Rec_Data[4] << 8) |
            Rec_Data[5]
        );


    /* ========================================================
     * TEMPERATURE
     * ======================================================== */

    int16_t temp_raw =
        (int16_t)
        (
            (Rec_Data[6] << 8) |
            Rec_Data[7]
        );


    DataStruct->Temperature =
        ((float)temp_raw / 340.0f)
        + 36.53f;


    /* ========================================================
     * GYROSCOPE
     * ======================================================== */

    DataStruct->Gyro_X_RAW =
        (int16_t)
        (
            (Rec_Data[8] << 8) |
            Rec_Data[9]
        );


    DataStruct->Gyro_Y_RAW =
        (int16_t)
        (
            (Rec_Data[10] << 8) |
            Rec_Data[11]
        );


    DataStruct->Gyro_Z_RAW =
        (int16_t)
        (
            (Rec_Data[12] << 8) |
            Rec_Data[13]
        );


    /* ========================================================
     * CONVERT ACCELEROMETER
     * ======================================================== */

    DataStruct->Ax =
        (double)DataStruct->Accel_X_RAW /
        ACCEL_SENSITIVITY;


    DataStruct->Ay =
        (double)DataStruct->Accel_Y_RAW /
        ACCEL_SENSITIVITY;


    DataStruct->Az =
        (double)DataStruct->Accel_Z_RAW /
        ACCEL_SENSITIVITY;


    /* ========================================================
     * CONVERT GYROSCOPE
     * ======================================================== */

    DataStruct->Gx =
        (double)DataStruct->Gyro_X_RAW /
        GYRO_SENSITIVITY;


    DataStruct->Gy =
        (double)DataStruct->Gyro_Y_RAW /
        GYRO_SENSITIVITY;


    DataStruct->Gz =
        (double)DataStruct->Gyro_Z_RAW /
        GYRO_SENSITIVITY;


    /* ========================================================
     * ACCELEROMETER ANGLES
     * ======================================================== */

    Accel_Angle_X =
        atan2(
            (double)DataStruct->Accel_Y_RAW,
            (double)DataStruct->Accel_Z_RAW
        )
        *
        RAD_TO_DEG;


    Accel_Angle_Y =
        atan2(
            -(double)DataStruct->Accel_X_RAW,
            sqrt(
                ((double)DataStruct->Accel_Y_RAW *
                 (double)DataStruct->Accel_Y_RAW)
                +
                ((double)DataStruct->Accel_Z_RAW *
                 (double)DataStruct->Accel_Z_RAW)
            )
        )
        *
        RAD_TO_DEG;


    /* ========================================================
     * KALMAN FILTER
     *
     * IMPORTANT:
     *
     * X uses Gx
     * Y uses Gy
     *
     * ======================================================== */

    DataStruct->KalmanAngleX =
        Kalman_getAngle(
            &KalmanX,
            Accel_Angle_X,
            DataStruct->Gx,
            dt
        );


    DataStruct->KalmanAngleY =
        Kalman_getAngle(
            &KalmanY,
            Accel_Angle_Y,
            DataStruct->Gy,
            dt
        );
}
