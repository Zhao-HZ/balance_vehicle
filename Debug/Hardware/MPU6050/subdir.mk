################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hardware/MPU6050/KF.c \
../Hardware/MPU6050/MPU6050.c \
../Hardware/MPU6050/inv_mpu.c \
../Hardware/MPU6050/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Hardware/MPU6050/KF.o \
./Hardware/MPU6050/MPU6050.o \
./Hardware/MPU6050/inv_mpu.o \
./Hardware/MPU6050/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Hardware/MPU6050/KF.d \
./Hardware/MPU6050/MPU6050.d \
./Hardware/MPU6050/inv_mpu.d \
./Hardware/MPU6050/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Hardware/MPU6050/%.o Hardware/MPU6050/%.su Hardware/MPU6050/%.cyclo: ../Hardware/MPU6050/%.c Hardware/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/18050/codedemo/stm32demo/balance_vehicle/Hardware/MPU6050" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Hardware-2f-MPU6050

clean-Hardware-2f-MPU6050:
	-$(RM) ./Hardware/MPU6050/KF.cyclo ./Hardware/MPU6050/KF.d ./Hardware/MPU6050/KF.o ./Hardware/MPU6050/KF.su ./Hardware/MPU6050/MPU6050.cyclo ./Hardware/MPU6050/MPU6050.d ./Hardware/MPU6050/MPU6050.o ./Hardware/MPU6050/MPU6050.su ./Hardware/MPU6050/inv_mpu.cyclo ./Hardware/MPU6050/inv_mpu.d ./Hardware/MPU6050/inv_mpu.o ./Hardware/MPU6050/inv_mpu.su ./Hardware/MPU6050/inv_mpu_dmp_motion_driver.cyclo ./Hardware/MPU6050/inv_mpu_dmp_motion_driver.d ./Hardware/MPU6050/inv_mpu_dmp_motion_driver.o ./Hardware/MPU6050/inv_mpu_dmp_motion_driver.su

.PHONY: clean-Hardware-2f-MPU6050

