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

// ADC模数转换实验-HAL库函数版
 
//STM32H7工程模板-HAL库函数版本	
/*************************************************************/

	float temp;
	struct compx signal0[FFT_N]; //size euqal to dsize/2 and FFT_N
	struct compx signal1[FFT_N]; //size euqal to dsize/2 and FFT_N
	struct compx signal2[FFT_N]; //size euqal to dsize/2 and FFT_N
	struct compx signal3[FFT_N]; //size euqal to dsize/2 and FFT_N
	float freq0[FFT_N];
	float freq1[FFT_N];
	float freq2[FFT_N];
	float freq3[FFT_N];	
	extern uint32_t DMA1Flag;
	
	int Fire0; int Fire1; int Fire2; int Fire3;
	int Fire;

	
int main(void)
{
	MPU_Config();
	Cache_Enable();               //打开L1-Cache
	HAL_Init();				        		//初始化HAL库
	Stm32_Clock_Init(160,5,2,4);  //设置时钟,400Mhz 

	//uart_init(115200);						//串口初始化
	
	GPIO_Init(); 	
	LED_Init();


	PLL2_Init();
	DMA_Init();
	ADC_Init();					//初始化ADC1通道--PC0引脚，对应ADC1的CH10通2道，更多引脚信息，请参考数据手册
	
  
	while(1)
	{
		Get_DMAValue(signal0, signal1, signal2, signal3);
		FFT(signal0, freq0); FFT(signal1, freq1);
		FFT(signal2, freq2); FFT(signal3, freq3);
		Fire0 = CFAR(freq0);
		Fire1 = CFAR(freq1);
		Fire2 = CFAR(freq2);
		Fire3 = CFAR(freq3);
		Fire = Fire0 + Fire1 + Fire3 + Fire3;
		if(Fire > 2){
			Fire_On;
		}
		else {
			Fire_Clear;
		}
		PA0Toggle;
		//delay_ms(10);
	}
}













