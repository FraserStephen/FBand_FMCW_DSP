#ifndef _LED_H
#define _LED_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 

//LED��������	   

//STM32H7����ģ��-HAL�⺯���汾
//DevEBox  ��Խ����
//�Ա����̣�mcudev.taobao.com
//�Ա����̣�shop389957290.taobao.com								  
////////////////////////////////////////////////////////////////////////////////// 	

//LED�˿ڶ���

#define LED2(n)		   (n?HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET))
#define LED2_Toggle (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1))                                    //LED1�����ƽ��ת
#define PA0Toggle (HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9))                                    //PA0�����ƽ��ת
#define Fire_On  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)                            //PA10 Set
#define Fire_Clear HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)													//PA10 Reset

void LED_Init(void); //LED��ʼ������


#endif


//STM32H7����ģ��-HAL�⺯���汾
//DevEBox  ��Խ����
//�Ա����̣�mcudev.taobao.com
//�Ա����̣�shop389957290.taobao.com	






