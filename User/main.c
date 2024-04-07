#include "stm32f10x.h"                  // Device header
#include "bsp_usart.h"
#include "stm32f10x_it.h"
#include "bsp_i2c.h"
#include "mpu6050.h"
#include <stdio.h>
#include "bsp_SysTick.h"
#include "bsp_delay.h"
#include "mpu6050_ruan.h"
#include "ii2_ruan.h"
int main(void)
{
	USART3_Config();
   i2c_GPIO_Config();
   RUAN_MPU6050_Init();
   if(RUAN_MPU6050ReadID() == 0 )
	{
		while(1)
		{
		printf("\r\n没有检测到MPU6050传感器\r\n");
		}	//检测不到MPU6050 会红灯亮然后卡死
   }
   SysTick_Init();
 SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //启动定时器
	while (1)
	{ 
		
		if( task_readdata_finish ) //task_readdata_finish = 1 表示读取MPU6050数据完成
   {
     
     printf("加速度:%8d%8d%8d",Acel[0],Acel[1],Acel[2]);
     
     printf("	陀螺仪:%8d%8d%8d",Gyro[0],Gyro[1],Gyro[2]);
     
     printf("	温度:%8.2f\r\n",Temp);
     
     task_readdata_finish = 0; // 清零标志位
     
   }
	}
}
