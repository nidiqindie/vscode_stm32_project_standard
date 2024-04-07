#include "bsp_led.h"   

 /**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
void LED_GPIO_Config(void)
{		

		GPIO_InitTypeDef GPIO_InitStructure;


		RCC_APB2PeriphClockCmd( LEDR_GPIO_CLK | LEDG_GPIO_CLK | LEDB_GPIO_CLK, ENABLE);

		GPIO_InitStructure.GPIO_Pin = LEDR_GPIO_PIN;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(LEDR_GPIO_PORT, &GPIO_InitStructure);	
	
		GPIO_InitStructure.GPIO_Pin = LEDG_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(LEDG_GPIO_PORT, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = LEDB_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(LEDB_GPIO_PORT, &GPIO_InitStructure);

		/* �ر�����led��	*/
		GPIO_SetBits(LEDR_GPIO_PORT, LEDR_GPIO_PIN);
		GPIO_SetBits(LEDG_GPIO_PORT, LEDG_GPIO_PIN);	 
		GPIO_SetBits(LEDB_GPIO_PORT, LEDB_GPIO_PIN);
}
/*********************************************END OF FILE**********************/
