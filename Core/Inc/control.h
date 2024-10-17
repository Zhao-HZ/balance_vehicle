#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

// #include "stm32f1xx_hal_gpio.h"

#define AIN1_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define AIN1_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

#define AIN2_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define AIN2_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)

#define BIN1_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define BIN1_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

#define BIN2_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define BIN2_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

void get_angle();
void get_encoder(int* encoder_left, int* encoder_right);
int my_abs(int x);
void set_PWM(int PWM_left, int PWM_right);
int PWM_limit(int IN, int max, int min);

#endif /* INC_CONTROL_H_ */
