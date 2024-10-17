/*
 * iic.h
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */

#ifndef INC_IIC_H_
#define INC_IIC_H_
#include "main.h"

typedef uint8_t  u8;
typedef uint32_t u32;
typedef uint16_t u16;
//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

//IO���������Ĵ����汾�����Ч��
//#define IIC_SCL_SET  	GPIOB->BSRR |= 1<<8
//#define IIC_SCL_RESET	GPIOB->BSRR |= 1<<24
//#define IIC_SDA_SET  	GPIOB->BSRR |= 1<<9
//#define IIC_SDA_RESET	GPIOB->BSRR |= 1<<25
//#define READ_SDA  	((GPIOB->IDR)&(1<<9))

//IO��������
#define IIC_SCL_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define IIC_SCL_RESET	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define IIC_SDA_SET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define IIC_SDA_RESET	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define READ_SDA  		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)

//IIC���в�������
void IIC_Init(void);           //��ʼ��IIC��IO��
int IIC_Start(void);					 //����IIC��ʼ�ź�
void IIC_Stop(void);	  			 //����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);		 //IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
int IIC_Wait_Ack(void); 			 //IIC�ȴ�ACK�ź�
void IIC_Ack(void);						 //IIC����ACK�ź�
void IIC_NAck(void);					 //IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif /* INC_IIC_H_ */
