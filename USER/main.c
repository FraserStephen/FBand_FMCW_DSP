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
		FFT(signal2, freq2); 
		CFAR(freq2);
		if(Fire >= 1){
			Fire_On;
		}
		else {
			Fire_Clear;
		}
		PA0Toggle;
		//delay_ms(10);
	}
}


//save C:\Users\WeiHaoLi\Desktop\F_DSP\DataSample\Py_ADCPro\data.txt 0x38000880, 0x38000C80










