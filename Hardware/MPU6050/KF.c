#include "KF.h"

// Kalman Filter for X-axis orientation estimation
float KF_X(float acce_Y, float acce_Z, float gyro_X) 
{
    // State estimate
    static float x_hat[2][1] = {0};
    // Predicted state estimate
    static float x_hat_minus[2][1] = {0};
    // State covariance
    static float p_hat[2][2] = {{1, 0}, {0, 1}};
    // Predicted state covariance
    static float p_hat_minus[2][2] = {0};
    // Kalman gain
    static float K[2][1] = {0};
    // Sampling time
    const float Ts = 0.005; 
    // Identity matrix
    const float I[2][2] = {{1, 0}, {0, 1}};
    // Control input
    float u[1][1] = {{gyro_X}};
    // State transition matrix
    float A[2][2] = {{1, -Ts}, {0, 1}};
    // Control input matrix
    float B[2][1] = {{Ts}, {0}};
    // Output matrix
    float C[1][2] = {{1, 0}};
    // Process noise covariance
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}};
    // Measurement noise covariance
    float R[1][1] = {{1e-4}};
    // Transpose of A
    float A_T[2][2] = {{1, 0}, {-Ts, 1}};
    // Transpose of C
    float C_T[2][1] = {{1}, {0}};
    // Temporary variables for matrix operations
    float temp_1[2][1] = {0};
    float temp_2[2][1] = {0};
    float temp_3[2][2] = {0};
    float temp_4[2][2] = {0};
    float temp_5[1][2] = {0};
    float temp_6[1][1] = {0};
    // Measurement
    float y = atan2(-acce_Y, acce_Z);
    
    // Predict step
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    
    // Update step
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    
    return x_hat[0][0];
}

// Kalman Filter for Y-axis orientation estimation
float KF_Y(float acce_X, float acce_Z, float gyro_Y) 
{
    // State estimate
    static float x_hat[2][1] = {0};
    // Predicted state estimate
    static float x_hat_minus[2][1] = {0};
    // State covariance
    static float p_hat[2][2] = {{1, 0}, {0, 1}};
    // Predicted state covariance
    static float p_hat_minus[2][2] = {0};
    // Kalman gain
    static float K[2][1] = {0};
    // Sampling time
    const float Ts = 0.005; 
    // Identity matrix
    const float I[2][2] = {{1, 0}, {0, 1}};
    // Control input
    float u[1][1] = {{gyro_Y}};
    // State transition matrix
    float A[2][2] = {{1, -Ts}, {0, 1}};
    // Control input matrix
    float B[2][1] = {{Ts}, {0}};
    // Output matrix
    float C[1][2] = {{1, 0}};
    // Process noise covariance
    float Q[2][2] = {{1e-10, 0}, {0, 1e-10}};
    // Measurement noise covariance
    float R[1][1] = {{1e-4}};
    // Transpose of A
    float A_T[2][2] = {{1, 0}, {-Ts, 1}};
    // Transpose of C
    float C_T[2][1] = {{1}, {0}};
    // Temporary variables for matrix operations
    float temp_1[2][1] = {0};
    float temp_2[2][1] = {0};
    float temp_3[2][2] = {0};
    float temp_4[2][2] = {0};
    float temp_5[1][2] = {0};
    float temp_6[1][1] = {0};
    // Measurement
    float y = atan2(-acce_X, acce_Z);
    
    // Predict step
    mul(2, 2, 2, 1, A, x_hat, temp_1);
    mul(2, 1, 1, 1, B, u, temp_2);
    x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
    x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];
    
    mul(2, 2, 2, 2, A, p_hat, temp_3);
    mul(2, 2, 2, 2, temp_3, A_T, temp_4);
    p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
    p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
    p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
    p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];
    
    // Update step
    mul(1, 2, 2, 2, C, p_hat_minus, temp_5);
    mul(1, 2, 2, 1, temp_5, C_T, temp_6);
    temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
    mul(2, 2, 2, 1, p_hat_minus, C_T, temp_1);
    mul(2, 1, 1, 1, temp_1, temp_6, K);
    
    mul(1, 2, 2, 1, C, x_hat_minus, temp_6);
    temp_6[0][0] = y - temp_6[0][0];
    mul(2, 1, 1, 1, K, temp_6, temp_1);
    x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
    x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];
    
    mul(2, 1, 1, 2, K, C, temp_3);
    temp_3[0][0] = I[0][0] - temp_3[0][0];
    temp_3[0][1] = I[0][1] - temp_3[0][1];
    temp_3[1][0] = I[1][0] - temp_3[1][0];
    temp_3[1][1] = I[1][1] - temp_3[1][1];
    mul(2, 2, 2, 2, temp_3, p_hat_minus, p_hat);
    
    return x_hat[0][0];
}

// Matrix multiplication function
void mul(int A_row, int A_col, int B_row, int B_col, float A[][A_col], float B[][B_col], float C[][B_col])
{
    if (A_col == B_row)
    {
        for (int i = 0; i < A_row; i++)
        {
            for (int j = 0; j < B_col; j++)
            {
                C[i][j] = 0; // Initialize
                for (int k = 0; k < A_col; k++)
                {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
    }
    else
    {
        // Handle error if dimensions are not compatible
    }
}
