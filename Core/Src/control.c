#include "control.h"
#include "MPU6050.h"
#include "KF.h"
#include "tim.h"
#include "stdio.h"

#define PI 3.14159265

float gyro_balance, angle_balance, gyro_balance, gyro_turn;
int middle_angle;
float pitch, roll, yaw;
int encoder_left, encoder_right;
float acceleration_Z;
float motor_left, motor_right;
float balance_Kp = 20, balance_Kd = 1.35, velocity_Kp = 10, velocity_Ki = 0.8, turn_Kp = 42, turn_Kd = 0.6;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    int balance_PWM, Velocity_PWM, turn_PWM;
    if (htim == &htim2) {
        get_angle();
        get_encoder(&encoder_left, &encoder_right);
        encoder_left = -encoder_left;
        encoder_right = -encoder_right;
        balance_PWM = balance(angle_balance, gyro_balance);
        Velocity_PWM = velocity(encoder_left, encoder_right);
        turn_PWM = turn(gyro_turn);
        motor_left = balance_PWM + Velocity_PWM + turn_PWM;
        motor_right = balance_PWM + Velocity_PWM - turn_PWM;
        motor_left = PWM_limit(motor_left, 6900, -6900);
        motor_right = PWM_limit(motor_right, 6900, -6900);
        set_PWM(motor_left, motor_right);

        // printf("pitch, roll: %f, %f\n", pitch, roll);
        // printf("Encoder Left, Right: %d, %d\n", encoder_left, encoder_right);
        // set_PWM(4000, 4000);

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
    angle_balance = pitch;
    gyro_turn = Gyro_Z;
    acceleration_Z = Accel_Z;
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

int balance(float angle, float gyro) {
    float angle_bias, gyro_bias;
    int balance;
    angle_bias = middle_angle - angle;
    gyro_bias = 0 - gyro;
    balance = -balance_Kp * angle_bias - balance_Kd * gyro_bias;
    return balance;
}

int velocity(int encoder_left, int encoder_right) {
    static float velocity, encoder_least, encoder_bias, movement;
    static float encoder_integral, target_velocity;
    encoder_least = 0 - (encoder_left + encoder_right);
    encoder_bias *= 0.86;
    encoder_bias += encoder_least * 0.14;
    encoder_integral += encoder_bias;
    if (encoder_integral > 10000) encoder_integral = 10000;
    if (encoder_integral < -10000) encoder_integral = -10000;
    velocity = -encoder_bias * velocity_Kp - encoder_integral * velocity_Ki;
    return velocity;
}

int turn(float gyro) {
    static float turn_target, turn, turn_amplitude = 54;
    float Kp = turn_Kp, Kd;
    turn = turn_target * Kp + gyro * Kd;
    return turn;
}
