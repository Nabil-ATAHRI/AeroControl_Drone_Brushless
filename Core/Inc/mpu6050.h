#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include "i2c.h"
#include <stdint.h>

#define MPU6050_ADDR        (0x68U << 1)

#define WHO_AM_I_REG        0x75U
#define PWR_MGMT_1_REG      0x6BU
#define SMPLRT_DIV_REG      0x19U
#define CONFIG_REG          0x1AU
#define GYRO_CONFIG_REG     0x1BU
#define ACCEL_CONFIG_REG    0x1CU
#define ACCEL_XOUT_H_REG    0x3BU
#define TEMP_OUT_H_REG      0x41U
#define GYRO_XOUT_H_REG     0x43U

typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    double Ax;
    double Ay;
    double Az;

    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

} MPU6050_t;


typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;

    double angle;
    double bias;

    double P[2][2];

} Kalman_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
);

void MPU6050_Read_Gyro(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
);

void MPU6050_Read_Temp(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
);

void MPU6050_Read_All(
    I2C_HandleTypeDef *I2Cx,
    MPU6050_t *DataStruct
);

double Kalman_getAngle(
    Kalman_t *Kalman,
    double newAngle,
    double newRate,
    double dt
);

#endif
