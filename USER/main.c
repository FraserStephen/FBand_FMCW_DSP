#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h"
#include "key.h"


#include "LCD.h"
#include "Dis_Picture.h" 
#include "Text.h"	
#include "GBK_LibDrive.h"	

#include "adc.h"

#include "radar_dsp.h"



/*************************************************************/

// ADCģ��ת��ʵ��-HAL�⺯����
 
//STM32H7����ģ��-HAL�⺯���汾	
/*************************************************************/

	float temp;
	struct compx signal[FFT_N]; //size euqal to dsize/2 and FFT_N
	float freq[FFT_N];
	extern uint32_t DMA1Flag;
	extern uint16_t Fire;
	

	
int main(void)
{
	MPU_Config();
	Cache_Enable();               //��L1-Cache
	HAL_Init();				        		//��ʼ��HAL��
	Stm32_Clock_Init(160,5,2,4);  //����ʱ��,400Mhz 

	//uart_init(115200);						//���ڳ�ʼ��
	
	GPIO_Init(); 	
	LED_Init();


	PLL2_Init();
	DMA_Init();
	ADC_Init();					//��ʼ��ADC1ͨ��--PC0���ţ���ӦADC1��CH10ͨ��������������Ϣ����ο������ֲ�
	
  
	while(1)
	{
		Fire = 0;
		Get_DMAValue(signal);
		FFT(signal, freq);
		CFAR(freq);
		if(Fire){
			Fire_On;
		}
		else {
			Fire_Clear;
		}
		PA0Toggle;
		//delay_ms(10);
	}
}













