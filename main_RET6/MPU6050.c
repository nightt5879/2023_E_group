#include "MPU6050_Reg.h"
#include "stm32f10x.h" // Device header

#define MPU6050_ADDRESS 0xD0

uint8_t ID; // the mpu-6050 ID

void MPU6050_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 100000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout--;
		if (Timeout == 0)
		{
			break;
		}
	}
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

	I2C_SendData(I2C2, RegAddress);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);

	I2C_SendData(I2C2, Data);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;

	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

	I2C_SendData(I2C2, RegAddress);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);

	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C2);

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	return Data;
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

uint8_t MPU6050_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 10000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C2, &I2C_InitStructure);

	I2C_Cmd(I2C2, ENABLE);

	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x08);	 // 复位MPU6050
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);	 // 唤醒MPU6050
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); // 陀螺仪传感器,±2000dps
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00); // 加速度传感器,±2g
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);	 // 设置采样率50Hz
	MPU6050_WriteReg(MPU_INT_EN_REG, 0X00);	   // 关闭所有中断
	MPU6050_WriteReg(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
	MPU6050_WriteReg(MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
	MPU6050_WriteReg(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
	ID = MPU6050_GetID();
	if (ID == 0x68)
	{
		MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0X01); // 设置CLKSEL,PLL X轴为参考
		MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0X00); // 加速度与陀螺仪都工作
		MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0X09); // 设置采样率为50Hz
	} else return 1;
	return 0;
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
					 int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;

	// DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	// DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	// *AccX = (DataH << 8) | DataL;
	*AccX = 0;

	// DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	// DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	// *AccY = (DataH << 8) | DataL;
	*AccY = 0;

	// DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	// DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	// *AccZ = (DataH << 8) | DataL;
	*AccZ = 0;

	// DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	// DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	// *GyroX = (DataH << 8) | DataL;
	*GyroX = 0;

	// DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	// DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	// *GyroY = (DataH << 8) | DataL;
	*GyroY = 0;

	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}
