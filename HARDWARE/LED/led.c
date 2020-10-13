#include "led.h"
//////////////////////////////////////////////////////////////////////////////////	 

//LED��������	  

//STM32H7����ģ��-HAL�⺯���汾
//DevEBox  ��Խ����
//�Ա����̣�mcudev.taobao.com
//�Ա����̣�shop389957290.taobao.com								  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��PA1,��ʹ������ڵ�ʱ��		

//LED I/O��ʼ��
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE();					          //����PAʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_1;			              //PA1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		    //�������
    GPIO_Initure.Pull=GPIO_PULLUP;         			    //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);     	     	//��ʼ��PA1
	
	
		GPIO_Initure.Pin=GPIO_PIN_0;			              //PA0
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		    //�������
    GPIO_Initure.Pull=GPIO_PULLUP;         			    //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);     	     	//��ʼ��PA0
	
		GPIO_Initure.Pin=GPIO_PIN_2;			              //PA2
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		    //�������
    GPIO_Initure.Pull=GPIO_PULLUP;         			    //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);     	     	//��ʼ��PA0
	   
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);	//PA1��1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);	//PA0��1
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);	//PA2��0 (Fire) 

}



//STM32H7����ģ��-HAL�⺯���汾
//DevEBox  ��Խ����
//�Ա����̣�mcudev.taobao.com
//�Ա����̣�shop389957290.taobao.com	










