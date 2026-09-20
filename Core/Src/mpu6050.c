/*
 * mpu6050.c
 *
 * MPU6050 driver
 */

#include "mpu6050.h"

#include <math.h>


/* ============================================================
 * Constants
 * ============================================================ */

#define RAD_TO_DEG              57.2957795130823208768

#define WHO_AM_I_REG            0x75
#define PWR_MGMT_1_REG          0x6B
#define SMPLRT_DIV_REG          0x19
#define ACCEL_CONFIG_REG        0x1C
#define ACCEL_XOUT_H_REG        0x3B
#define TEMP_OUT_H_REG          0x41
#define GYRO_CONFIG_REG         0x1B
#define GYRO_XOUT_H_REG         0x43


/*
 * MPU6050 address
 *
 * AD0 = GND
 *
 * 7-bit address = 0x68
 * STM32 HAL address = 0x68 << 1 = 0xD0
 */
#define MPU6050_ADDR             (0x68 << 1)


#define I2C_TIMEOUT              100U


/*
 * Accelerometer Z calibration factor.
 *
 * Standard MPU6050 ±2g = 16384 LSB/g.
 *
 * 14418 is kept from your original project.
 */
#define ACCEL_Z_CORRECTOR        14418.0


/* ============================================================
 * Kalman variables
 * ============================================================ */

static uint32_t timer = 0;


Kalman_t KalmanX =
{
    .Q_angle   = 0.001f,
    .Q_bias    = 0.003f,
    .R_measure = 0.03f,

    .angle     = 0.0,
    .bias      = 0.0,

    .P         = {
                    {0.0, 0.0},
                    {0.0, 0.0}
                  }
};


Kalman_t KalmanY =
{
    .Q_angle   = 0.001f,
    .Q_bias    = 0.003f,
    .R_measure = 0.03f,

    .angle     = 0.0,
    .bias      = 0.0,

    .P         = {
                    {0.0, 0.0},
                    {0.0, 0.0}
                  }
};


/* ============================================================
 * MPU6050 Initialization
 * ============================================================ */

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;
    uint8_t Data = 0;

    HAL_StatusTypeDef status;


    /* --------------------------------------------------------
     * Read WHO_AM_I register
     * -------------------------------------------------------- */

    status = HAL_I2C_Mem_Read(I2Cx,
                              MPU6050_ADDR,
                              WHO_AM_I_REG,
                              I2C_MEMADD_SIZE_8BIT,
                              &check,
                              1,
                              I2C_TIMEOUT);


    /*
     * Check I2C communication
     * and MPU6050 device ID
     */

    if (status != HAL_OK)
    {
        return 1;
    }


    if (check != 0x68)
    {
        return 1;
    }


    /* --------------------------------------------------------
     * Wake up MPU6050
     * -------------------------------------------------------- */

    Data = 0x00;

    status = HAL_I2C_Mem_Write(I2Cx,
                               MPU6050_ADDR,
                               PWR_MGMT_1_REG,
                               I2C_MEMADD_SIZE_8BIT,
                               &Data,
                               1,
                               I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return 1;
    }


    HAL_Delay(100);


    /* --------------------------------------------------------
     * Sample rate
     *
     * Internal sample rate = 1 kHz
     * SMPLRT_DIV = 7
     *
     * 1000 / (1 + 7) = 125 Hz
     * -------------------------------------------------------- */

    Data = 0x07;

    status = HAL_I2C_Mem_Write(I2Cx,
                               MPU6050_ADDR,
                               SMPLRT_DIV_REG,
                               I2C_MEMADD_SIZE_8BIT,
                               &Data,
                               1,
                               I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return 1;
    }


    /* --------------------------------------------------------
     * Accelerometer configuration
     *
     * FS_SEL = 0
     * Full scale = ±2g
     * Sensitivity = 16384 LSB/g
     * -------------------------------------------------------- */

    Data = 0x00;

    status = HAL_I2C_Mem_Write(I2Cx,
                               MPU6050_ADDR,
                               ACCEL_CONFIG_REG,
                               I2C_MEMADD_SIZE_8BIT,
                               &Data,
                               1,
                               I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return 1;
    }


    /* --------------------------------------------------------
     * Gyroscope configuration
     *
     * FS_SEL = 0
     * Full scale = ±250 °/s
     * Sensitivity = 131 LSB/(°/s)
     * -------------------------------------------------------- */

    Data = 0x00;

    status = HAL_I2C_Mem_Write(I2Cx,
                               MPU6050_ADDR,
                               GYRO_CONFIG_REG,
                               I2C_MEMADD_SIZE_8BIT,
                               &Data,
                               1,
                               I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return 1;
    }


    /* Initialize Kalman timer */

    timer = HAL_GetTick();


    return 0;
}


/* ============================================================
 * Read Accelerometer
 * ============================================================ */

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx,
                        MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6] = {0};


    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(I2Cx,
                              MPU6050_ADDR,
                              ACCEL_XOUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT,
                              Rec_Data,
                              6,
                              I2C_TIMEOUT);


    if (status != HAL_OK)
    {
        return;
    }


    /* RAW values */

    DataStruct->Accel_X_RAW =
            (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);

    DataStruct->Accel_Y_RAW =
            (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);

    DataStruct->Accel_Z_RAW =
            (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);


    /* Convert to g */

    DataStruct->Ax =
            DataStruct->Accel_X_RAW / 16384.0;

    DataStruct->Ay =
            DataStruct->Accel_Y_RAW / 16384.0;

    DataStruct->Az =
            DataStruct->Accel_Z_RAW / ACCEL_Z_CORRECTOR;
}


/* ============================================================
 * Read Gyroscope
 * ============================================================ */

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx,
                       MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6] = {0};

    HAL_StatusTypeDef status;


    status = HAL_I2C_Mem_Read(I2Cx,
                              MPU6050_ADDR,
                              GYRO_XOUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT,
                              Rec_Data,
                              6,
                              I2C_TIMEOUT);


    if (status != HAL_OK)
    {
        return;
    }


    /* RAW values */

    DataStruct->Gyro_X_RAW =
            (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);

    DataStruct->Gyro_Y_RAW =
            (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);

    DataStruct->Gyro_Z_RAW =
            (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);


    /* Convert to °/s */

    DataStruct->Gx =
            DataStruct->Gyro_X_RAW / 131.0;

    DataStruct->Gy =
            DataStruct->Gyro_Y_RAW / 131.0;

    DataStruct->Gz =
            DataStruct->Gyro_Z_RAW / 131.0;
}


/* ============================================================
 * Read Temperature
 * ============================================================ */

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx,
                       MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2] = {0};

    int16_t temp;

    HAL_StatusTypeDef status;


    status = HAL_I2C_Mem_Read(I2Cx,
                              MPU6050_ADDR,
                              TEMP_OUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT,
                              Rec_Data,
                              2,
                              I2C_TIMEOUT);


    if (status != HAL_OK)
    {
        return;
    }


    temp = (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);


    /*
     * MPU6050 temperature:
     *
     * Temperature = RAW / 340 + 36.53
     */

    DataStruct->Temperature =
            ((float)temp / 340.0f) + 36.53f;
}


/* ============================================================
 * Read all MPU6050 data
 *
 * 14 bytes:
 *
 * ACCEL X  = bytes 0-1
 * ACCEL Y  = bytes 2-3
 * ACCEL Z  = bytes 4-5
 * TEMP     = bytes 6-7
 * GYRO X   = bytes 8-9
 * GYRO Y   = bytes 10-11
 * GYRO Z   = bytes 12-13
 * ============================================================ */

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx,
                      MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14] = {0};

    int16_t temp;

    HAL_StatusTypeDef status;


    /* --------------------------------------------------------
     * Read 14 bytes
     * -------------------------------------------------------- */

    status = HAL_I2C_Mem_Read(I2Cx,
                              MPU6050_ADDR,
                              ACCEL_XOUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT,
                              Rec_Data,
                              14,
                              I2C_TIMEOUT);


    /*
     * If I2C communication fails,
     * don't process invalid data.
     */

    if (status != HAL_OK)
    {
        return;
    }


    /* --------------------------------------------------------
     * Accelerometer RAW
     * -------------------------------------------------------- */

    DataStruct->Accel_X_RAW =
            (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]);

    DataStruct->Accel_Y_RAW =
            (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]);

    DataStruct->Accel_Z_RAW =
            (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]);


    /* --------------------------------------------------------
     * Temperature RAW
     * -------------------------------------------------------- */

    temp =
            (int16_t)((Rec_Data[6] << 8) | Rec_Data[7]);


    /* --------------------------------------------------------
     * Gyroscope RAW
     * -------------------------------------------------------- */

    DataStruct->Gyro_X_RAW =
            (int16_t)((Rec_Data[8] << 8) | Rec_Data[9]);

    DataStruct->Gyro_Y_RAW =
            (int16_t)((Rec_Data[10] << 8) | Rec_Data[11]);

    DataStruct->Gyro_Z_RAW =
            (int16_t)((Rec_Data[12] << 8) | Rec_Data[13]);


    /* --------------------------------------------------------
     * Convert accelerometer
     * -------------------------------------------------------- */

    DataStruct->Ax =
            DataStruct->Accel_X_RAW / 16384.0;

    DataStruct->Ay =
            DataStruct->Accel_Y_RAW / 16384.0;

    DataStruct->Az =
            DataStruct->Accel_Z_RAW / ACCEL_Z_CORRECTOR;


    /* --------------------------------------------------------
     * Convert temperature
     * -------------------------------------------------------- */

    DataStruct->Temperature =
            ((float)temp / 340.0f) + 36.53f;


    /* --------------------------------------------------------
     * Convert gyroscope
     * -------------------------------------------------------- */

    DataStruct->Gx =
            DataStruct->Gyro_X_RAW / 131.0;

    DataStruct->Gy =
            DataStruct->Gyro_Y_RAW / 131.0;

    DataStruct->Gz =
            DataStruct->Gyro_Z_RAW / 131.0;


    /* ========================================================
     * Kalman filter
     * ======================================================== */

    uint32_t now = HAL_GetTick();

    double dt;


    if (timer == 0)
    {
        dt = 0.01;
    }
    else
    {
        dt = (double)(now - timer) / 1000.0;
    }


    timer = now;


    /*
     * Protect Kalman filter against abnormal dt
     */

    if (dt <= 0.0 || dt > 1.0)
    {
        dt = 0.01;
    }


    /* --------------------------------------------------------
     * Calculate roll
     * -------------------------------------------------------- */

    double roll;

    double roll_sqrt =
            sqrt(
                (double)DataStruct->Accel_X_RAW *
                (double)DataStruct->Accel_X_RAW +

                (double)DataStruct->Accel_Z_RAW *
                (double)DataStruct->Accel_Z_RAW
            );


    if (roll_sqrt != 0.0)
    {
        roll =
                atan(
                    (double)DataStruct->Accel_Y_RAW /
                    roll_sqrt
                )
                * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }


    /* --------------------------------------------------------
     * Calculate pitch
     * -------------------------------------------------------- */

    double pitch =
            atan2(
                -(double)DataStruct->Accel_X_RAW,
                (double)DataStruct->Accel_Z_RAW
            )
            * RAD_TO_DEG;


    /* --------------------------------------------------------
     * Kalman Y
     * -------------------------------------------------------- */

    if ((pitch < -90.0 &&
         DataStruct->KalmanAngleY > 90.0) ||

        (pitch > 90.0 &&
         DataStruct->KalmanAngleY < -90.0))
    {
        KalmanY.angle = pitch;

        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY =
                Kalman_getAngle(
                    &KalmanY,
                    pitch,
                    DataStruct->Gy,
                    dt
                );
    }


    /* --------------------------------------------------------
     * Correct gyro X when crossing ±90°
     * -------------------------------------------------------- */

    if (fabs(DataStruct->KalmanAngleY) > 90.0)
    {
        DataStruct->Gx = -DataStruct->Gx;
    }


    /* --------------------------------------------------------
     * Kalman X
     * -------------------------------------------------------- */

    DataStruct->KalmanAngleX =
            Kalman_getAngle(
                &KalmanX,
                roll,
                DataStruct->Gy,
                dt
            );
}


/* ============================================================
 * Kalman filter
 * ============================================================ */

double Kalman_getAngle(Kalman_t *Kalman,
                       double newAngle,
                       double newRate,
                       double dt)
{
    double rate;

    double S;

    double K[2];

    double y;

    double P00_temp;

    double P01_temp;


    /* --------------------------------------------------------
     * Predict
     * -------------------------------------------------------- */

    rate = newRate - Kalman->bias;

    Kalman->angle += dt * rate;


    Kalman->P[0][0] +=
            dt *
            (
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


    /* --------------------------------------------------------
     * Measurement update
     * -------------------------------------------------------- */

    S = Kalman->P[0][0] +
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


    /* --------------------------------------------------------
     * Update covariance
     * -------------------------------------------------------- */

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
