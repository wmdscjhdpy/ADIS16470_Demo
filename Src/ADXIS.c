/** 
* @brief        ADIS16470简易功能库(基于STM32的HAL库)
* @details  为陀螺仪提供基本功能,可以通过本库获取陀螺仪的三轴角速度,加速度与当前姿态角 \n
*						STM32HAL库开发者使用: \n
*						在Cube中启用任意一个SPI设备为全双工,注意参数CPOL为HIGH,CHPA=2,如果需要使用BurstRead还要使波特率小于1Mb/s,否则使波特率小于2Mb/s \n
*						除了启用一个SPI外还需要启用一个GPIO_Output为SPI设备片选线,建议设置其label为SPI_NSS,方式为开漏(上拉)输出 \n
* 					针对stm32型号更改ADXIS.h内的头文件为工程对应的stm32头文件,在本文件中修改ADX_SPI为工程选用的SPI句柄 \n
*						工程需要先调用ADX_Init(void)后方可使用Burst_read或Single_Handle处理数据 \n
*						非STM32HAL库: \n
*						自行实现宏定义NSS_SET与NSS_RESET 片选线拉高/拉低函数 \n
*						自行实现 HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)方法,将check声明为unsigned char其他同上
* @author      WMD
* @date     2018年3月27日15:43:21
* @version  1.1
* @par Copyright (c):  
*       WMD 
* @par 日志
*/  
#include "ADXIS.h"
#include "spi.h"
#include "string.h"
#define NSS_SET HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_SET)
#define NSS_RESET HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_RESET)
#define ADX_SPI hspi1 //使用哪个spi
ADX_t imu;//导出的数据结构体
GyroData_t GyroData;
/** 
* @brief 该库使用默认模式,陀螺仪原始数据更新频率2000Hz
*/

static int16_t Send_Cmd=0x6800;//Burst Read指令
static void sb_delay(volatile uint32_t t)
{
	while(t--);
}
//单个数据的传送
void ADX_Init(void)//为保证spi数据的burst read 不在进行中 先大量读取
{
	ADX_Write_Reg(0x68,0x80);//重启指令
	ADX_Write_Reg(0x69,0x00);
	HAL_Delay(400);//等待重启完成
	
	ADX_Write_Reg(0x5C,0x03);//设置内置巴特沃斯滤波器等级
	ADX_Write_Reg(0x5D,0x00);

	ADX_Write_Reg(0x64,0x03);//设定均值滤波器值为4 此时角度差最终输出频率为2000/(3+1)=500Hz
	ADX_Write_Reg(0x65,0x00);
}
static uint16_t ADX_flame_TandR(uint16_t trans)
{
	NSS_RESET;
	uint16_t result;
	static HAL_StatusTypeDef state;
	state=HAL_SPI_TransmitReceive(&ADX_SPI,(uint8_t*)&trans,(uint8_t*)&result,1,0xff);
	if(state!=HAL_OK)
	{
		while(1);
	}
	NSS_SET;
	sb_delay(150);
	return result;
}
//读取寄存器的函数,因为SPI的方式 在连续读取上有效率优势,所以建议连续读取
/*
@parameter:
	addr_Reg 待读取的寄存器数组,Rx_point
	Rx_point 读取后的数据放的位置
	Reg_num	 欲读取的寄存器数量
@return:
	0 成功
	1 错误  //emmmm现在暂时还没有实现错误码
*/
int8_t ADX_Read_Reg(uint8_t* addr_Reg,uint16_t* Rx_point,uint8_t Reg_num)
{
	uint16_t Tx_tmp=0,Rx_tmp=0;
	
	//数据第一帧 只发不接
	Tx_tmp=addr_Reg[0]<<8;
	Rx_tmp=ADX_flame_TandR(Tx_tmp);
	for(uint8_t i=1;i<Reg_num;i++)//+1是因为spi有一帧延迟
	{
		Tx_tmp=addr_Reg[i]<<8;//准备发送帧的格式
		Rx_tmp=ADX_flame_TandR(Tx_tmp);
		Rx_point[i-1]=Rx_tmp;
	}
	//数据最后一帧 只接不发
	Tx_tmp=0;
	Rx_point[Reg_num-1]=ADX_flame_TandR(Tx_tmp);
	
	return 0;
}
/** 
* @brief  对ADIS16470寄存器内部写操作函数
* @param[in]   addr 写的地址
* @param[in]   value 写的值
* @retval  0  成功 
* @retval  -1   错误(未实现)
* @par 日志 
*
*/
int8_t ADX_Write_Reg(uint8_t addr,uint8_t value)
{
	addr|=0x80;//写数据的掩码
	uint16_t Tx_tmp=(addr<<8) | value;
	ADX_flame_TandR(Tx_tmp);
	return 0;
}
/** 
* @brief  采用BurstRead一次性获得三轴加速度与角速度,16位精度
* @param[in]   none
* @par 日志 
*
*/
int8_t ADX_BurstRead()
{
	NSS_RESET;
	static uint8_t* u8point=(uint8_t*)&imu;
	int16_t parity=0;
	HAL_StatusTypeDef check=HAL_OK;
	uint16_t tmpRx=0;
	static uint16_t tmpTx[10]={0};
	check|=HAL_SPI_TransmitReceive(&ADX_SPI,(uint8_t*)&Send_Cmd,(uint8_t*)&tmpRx,1,0xff);//发送指令,该16位接收无意义,故不保存数据
	//注意到数据手册中ADXIS16470的发送格式是低地址在前 高地址在后 符合stm32的小端模式 所以不需要做移位处理
	if(check!=HAL_OK)while(1);
	sb_delay(100);
	check|=HAL_SPI_TransmitReceive(&ADX_SPI,(uint8_t*)&tmpTx,u8point,10,0xff);//接收20字节的数据放入imu结构体中
	if(check!=HAL_OK)while(1);
	NSS_SET;
	sb_delay(150);
	//进行数据校验
	for(uint8_t i=0;i<9*2;i++)
	{
		parity+=u8point[i];
	}
	if(parity==imu.Checknum && !check)return 0;
	else 
	{
		memset(&imu,0x00,sizeof(imu));
		return -1;//校验失败
	}
}
/** 
* @brief  采用读寄存器的方式获得陀螺仪三轴加速度与三轴角速度与三轴姿态角,32位精度
* @param[in]   void
* @par 日志 
*
*/
void ADX_Single_Handle(void)
{
	//用于保存上一次的角度值 等下加回去
	float lastx=GyroData.anglex;
	float lasty=GyroData.angley;
	float lastz=GyroData.anglez;
	
	static uint8_t addr[]=
	{ 0x04,0x06,//Gyro_X
		0x08,0x0A,//Gyro_Y
		0x0C,0x0E,//Gyro_Z
		0x10,0x12,//Acc_X
		0x14,0x16,//Acc_Y
		0x18,0x1A,//Acc_Z
		0x24,0x26,//Delta_X 注意这里读到的是2ms内的角度差
		0x28,0x2A,//Delta_Y
		0x2C,0x2E,//Delta_Z
	};
	static uint16_t data[sizeof(addr)];
	int32_t* point32=(int32_t*)data;//便于将两个int16_t类型合成int32_t类型
	float* Gyro_float=(float*)&GyroData;//便于将int32_t类型转换成float类型
	ADX_Read_Reg(addr,data,sizeof(addr));
	//52428.8f
	uint8_t i;
	for(i=0;i<3;i++)//Gyro 0.1 °/s=2^16LSB
	{
		*(Gyro_float+i)=*(point32+i)/655360.0f;
	}
	for(;i<6;i++)//Acc 1.25 m/s^2=2^16LSB
	{
		*(Gyro_float+i)=*(point32+i)/52428.8f;
	}
	for(;i<9;i++)//Angle 2160 °=2^31LSB
	{
		*(Gyro_float+i)=*(point32+i)/994205.4f;
	}
	//加回角度值 
	GyroData.anglex+=lastx;
	GyroData.angley+=lasty;
	GyroData.anglez+=lastz;
}
/** 
* @brief  对陀螺仪进行零偏校准
* @note		该函数建议在开始时执行,执行时陀螺仪最好不要移动 因为陀螺仪可以保存校准值,所以校准不一定是每次启动时必须的
* @par 日志 
*
*/
void Self_Calibration(void)
{
	int32_t RawBiasData[6]={0};//存储二进制的当前误差值
	uint8_t addr=0x40;//XG_BIAS_LOW的地址 因为其后面的地址是连续的 所以取这个开始
	uint8_t* writedata=(uint8_t*)RawBiasData;//待写入内存的首地址 和addr一起递增
	for(uint8_t i=0;i<24;i++)//24是6个校准值*4 因为一个校准值是4个字节的 现在先执行一次进行清零
	{
		ADX_Write_Reg(addr+i,writedata[i]);
	}
	uint16_t count=0;
	uint32_t timestamp;
	GyroData_t GyroInt={0};//用于积分用的临时结构体
	while(count<5000)//校准10秒
	{
		ADX_Single_Handle();
		timestamp=HAL_GetTick();
		while(HAL_GetTick()-timestamp<2);
		count++;
	}
	GyroInt.anglex=GyroData.anglex;
	GyroInt.angley=GyroData.angley;
	GyroInt.anglez=GyroData.anglez;
	//执行到这里说明数据采集完了
	float* pointf=(float*)&GyroInt;//浮点地址 便于从Gyro_Data中获取数据
	uint8_t i=0;
	for(;i<3;i++)
	{
		RawBiasData[i]=-(*(pointf+i+6)) *131072.0f/2.0f;//将他们重新转换成int32_t 注意这时候校准值取的是角度值 5s的角度变化/5就等于角速度了 所以此处除以5
	}
	//执行到这里 所有校准数据准备完成 准备写入
	for(uint8_t i=0;i<12;i++)//12是3个校准值*4 因为一个校准值是4个字节的
	{
		ADX_Write_Reg(addr+i,writedata[i]);
	}
}