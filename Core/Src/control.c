#include "control.h"
#include "MPU6050.h"
#include "KF.h"
#include "tim.h"
// #include "stm32f1xx_hal_tim.h"
// #include "stm32f103xb.h"
#include "stdio.h"

#define PI 3.14159265

float gyro_balance;
float pitch, roll, yaw;
int encoder_left, encoder_right;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        get_angle();
        get_encoder(&encoder_left, &encoder_right);
        encoder_left = -encoder_left;
        encoder_right = -encoder_right;
        printf("Pitch, Roll: %f, %f\n", pitch, roll);
        set_PWM(4000, 4000);
        // printf("Encoder Left, Right: %d, %d\n", encoder_left, encoder_right);

    }
}

void get_angle()
{
    float gyro_x, gyro_y, accel_x, accel_y, accel_z;
    float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
    Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);  
    Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);  
    Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);  
    Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L);
    Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L);
    Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L);
    if (Gyro_X > 32768)
        Gyro_X -= 65536;
    if (Gyro_Y > 32768)
        Gyro_Y -= 65536;
    if (Gyro_Z > 32768)
        Gyro_Z -= 65536;
    if (Accel_X > 32768)
        Accel_X -= 65536;
    if (Accel_Y > 32768)
        Accel_Y -= 65536;
    if (Accel_Z > 32768)
        Accel_Z -= 65536;
    gyro_balance = -Gyro_X;
    accel_x = Accel_X / 1671.84;
    accel_y = Accel_Y / 1671.84;
    accel_z = Accel_Z / 1671.84;
    gyro_x = Gyro_X / 939.8;
    gyro_y = Gyro_Y / 939.8;
    pitch = KF_X(accel_y, accel_z, -gyro_x) / PI * 180;
    roll = KF_X(accel_x, accel_z, gyro_y) / PI * 180;
}

void get_encoder(int* encoder_left, int* encoder_right) {
    *encoder_left = (short) TIM3->CNT;
    *encoder_right = (short) TIM4->CNT;
    TIM3->CNT = 0;
    TIM4->CNT = 0;

    // *encoder_left = (short) __HAL_TIM_GetCounter(&htim3);
    // *encoder_right = (short) __HAL_TIM_GetCounter(&htim4);
    // __HAL_TIM_SetCounter(&htim3, 0);
    // __HAL_TIM_SetCounter(&htim4, 0);
}

void set_PWM(int PWM_left, int PWM_right) {
    if (PWM_left > 0) {
        AIN1_SET;
        AIN2_RESET;
    } else {
        AIN1_RESET;
        AIN2_SET;
    }
    if (PWM_right > 0) {
        BIN1_SET;
        BIN2_RESET;
    } else {
        BIN1_RESET;
        BIN2_SET;
    }
    PWM_left = my_abs(PWM_left);
    PWM_right = my_abs(PWM_right);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWM_left);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, PWM_right);
}

int my_abs(int x) {
    return x > 0 ? x : -x;
}

int PWM_limit(int IN, int max, int min) {
    int OUT = IN;
    if (IN > max) {
        OUT = max;
    } else if (IN < min) {
        OUT = min;
    }
    return OUT;
}