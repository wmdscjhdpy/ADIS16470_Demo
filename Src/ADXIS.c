//2018年3月27日15:43:21 WMD 
#include "ADXIS.h"
#include "spi.h"
#include "string.h"
#define NSS_SET HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_SET)
#define NSS_RESET HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_RESET)
//#define CHANGE(x) (((x)&0xff00)>>8) | (((x)&0x00ff)<<8) //大小端互转
#define ADX_SPI hspi1 //使用哪个spi
ADX_t imu;


static int16_t Send_Cmd=0x6800;//Burst Read指令
static void sb_delay(volatile uint32_t t)
{
	while(t--);
}
//单个数据的传送
void ADX_Init(void)//为保证spi数据的burst read 不在进行中 先大量读取
{
	ADX_Write_Reg(0x69,0x40);//重启指令
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
	
//	Rx_tmp=ADX_flame_TandR(Tx_tmp);
//	sb_delay(100000);
//	Rx_tmp=ADX_flame_TandR(Tx_tmp);
//	sb_delay(100000);
//	Rx_point[0]=Rx_tmp;
	return 0;
}
int8_t ADX_Write_Reg(uint8_t addr,uint8_t value)
{
	addr|=0x80;//写数据的掩码
	uint16_t Tx_tmp=(addr<<8) | value;
	ADX_flame_TandR(Tx_tmp);
	return 0;
}
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
		//memset(&imu,0x00,sizeof(imu));
		return -1;//校验失败
	}
}

