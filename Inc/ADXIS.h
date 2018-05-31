/** 
* @brief        ADIS16470附属头文件
* @details  记得改第14行的头文件为自己的库文件
* @author      WMD
* @date     date 
* @version  
* @par Copyright (c):  
*       WMD 
* @par 日志
*/  
#ifndef _ADXIS_H
#define _ADXIS_H

#include "stm32f4xx_hal.h"
typedef struct
{
	int16_t DIAG_STAT;
	int16_t X_GYRO;
	int16_t Y_GYRO;
	int16_t Z_GYRO;
	int16_t X_ACCL;
	int16_t Y_ACCL;
	int16_t Z_ACCL;
	int16_t TEMP;
	int16_t DATA_CNTR;
	int16_t Checknum;
}ADX_t;
typedef struct
{
	float wx;//角速度
	float wy;
	float wz;
	float accx;//加速度
	float accy;
	float accz;
	float anglex;//角度
	float angley;
	float anglez;
}GyroData_t;
extern ADX_t imu;
extern GyroData_t GyroData;
int8_t ADX_BurstRead(void);
int8_t ADX_Read_Reg(uint8_t* addr_Reg,uint16_t* Rx_point,uint8_t Reg_num);
int8_t ADX_Write_Reg(uint8_t addr,uint8_t value);
void ADX_Init(void);//为保证spi数据的burst read 不在进行中 先大量读取
void ADX_Single_Handle(void);//采集一次数据
void Self_Calibration(void);
#endif
