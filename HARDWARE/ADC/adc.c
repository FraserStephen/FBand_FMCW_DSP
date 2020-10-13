#include "adc.h"
#include "radar_dsp.h"
#include "delay.h"
#include "stm32h7xx_hal.h"
#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//********************************************************************************/
//ADC驱动代码	

/******************************************************************************/							  
////////////////////////////////////////////////////////////////////////////////// 	

ADC_HandleTypeDef ADC1_Handler;//ADC句柄
ADC_ChannelConfTypeDef ADC1_ChanConf; //ADC Channel Handler
DMA_HandleTypeDef DmaHandle = {0}; //DMA Handler
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0}; //PLL Handler


uint32_t DMA1Flag = 0;

int32_t dsize = 512; //If you modify dsize, the same as line 19 , line 102 and line 112
//dsize must euqal to FFT_N*2
uint16_t ADCxValues0[FFT_N]; // Buffer0, must equal to dsize
uint16_t ADCxValues1[FFT_N]; // Buffer1


//********************************************************************************/
//Init GPIO
/******************************************************************************/	
void GPIO_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();  
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}



void MPU_Config(void){

		MPU_Region_InitTypeDef MPU_InitStruct;

    HAL_MPU_Disable();
 
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x24000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
 
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    
    
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x60000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;    
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;    
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x38000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;    
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
 
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
 

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}




//********************************************************************************/
//Init PLL2 to 72MHz

//PLL2 Clock / 2 = 36MHz (the fasten ADC clock)

/******************************************************************************/	


void PLL2_Init(void)
{
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 25;
    PeriphClkInitStruct.PLL2.PLL2N = 504;
    PeriphClkInitStruct.PLL2.PLL2P = 7;
    PeriphClkInitStruct.PLL2.PLL2Q = 7;
    PeriphClkInitStruct.PLL2.PLL2R = 7;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
		
		HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

}




//********************************************************************************/
//DMA
//********************************************************************************/
void DMA_Init(void)
{
	/*	
	__HAL_RCC_DMA1_CLK_ENABLE();
	DMA1_Stream1->PAR = (uint32_t)&(ADC1->DR);
	DMA1_Stream1->M0AR = (uint32_t)ADCxValues0;
	DMA1_Stream1->M1AR = (uint32_t)ADCxValues1;	
	DMA1_Stream1->NDTR = FFT_N;	
	
	DMA1_Stream1->CR = 0;
	
	DMA1_Stream1->CR |= 0<<6;
	DMA1_Stream1->CR |= 1<<8;
	DMA1_Stream1->CR |= 0<<9;
	DMA1_Stream1->CR |= 1<<10;
	DMA1_Stream1->CR |= 1<<11; //16bit for periph
	DMA1_Stream1->CR |= 1<<13; //16bit for memory
	DMA1_Stream1->CR |= 3<<16;
	
	DMA1_Stream1->CR |= 0<<21;
	DMA1_Stream1->CR |= 0<<23;
	
	DMA1_Stream1->CR |= 1<<18;  //Multi-Buff open
	DMA1_Stream1->CR &= ~(1<<19);	

	//DMA1_Stream1->CR |= (1<<0); //Enable DMA

	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);	
*/	

	
		__HAL_RCC_DMA1_CLK_ENABLE();
		DmaHandle.Instance = DMA1_Stream1;  //Select DMA stream 1
		DmaHandle.Init.Request = DMA_REQUEST_ADC1;  //Ask for ADC1
		DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY; 
		DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
		DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
		DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; //16bit
		DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	
		DmaHandle.Init.Mode = DMA_CIRCULAR;//Circular mode
		DmaHandle.Init.Priority = DMA_PRIORITY_VERY_HIGH ; 
		DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE; 
		DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
		
		HAL_DMA_Init(&DmaHandle);
		
		DMA1_Stream1->CR |= 1<<18;  //Multi-Buff open
		DMA1_Stream1->CR &= ~(1<<19);	
		DMA1_Stream1->PAR = (uint32_t)&(ADC1->DR);
		DMA1_Stream1->M0AR = (uint32_t)ADCxValues0;
		DMA1_Stream1->M1AR = (uint32_t)ADCxValues1;	
		DMA1_Stream1->NDTR = FFT_N;	
		
		//Open IRQ of DMA
		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);	
		
		__HAL_LINKDMA(&ADC1_Handler, DMA_Handle, DmaHandle); //bondle ADC and DMA 
	

}

void DMA1_Stream1_IRQHandler(void)
{
	//If DMA data trans finished
	if((DMA1->LISR & DMA_FLAG_TCIF1_5) != RESET)
	{     
				if((DMA1_Stream1->CR & (1<<19)) == 0){
					DMA1Flag = 1; //First Buff is available
				}
				else{
					DMA1Flag = 2; //Second Buff is available
				}
				DMA1->LIFCR = DMA_FLAG_TCIF1_5;//Clear IRQ	
	}
	//If DMA data trans half
	if((DMA1->LISR & DMA_FLAG_HTIF1_5) != RESET)
	{
        DMA1->LIFCR = DMA_FLAG_HTIF1_5;//Clear
				
	}
	
  //DMA Trans Error
  if((DMA1->LISR & DMA_FLAG_TEIF1_5) != RESET)
	{
				DMA1->LIFCR = DMA_FLAG_TEIF1_5;
  } 
  if((DMA1->LISR & DMA_FLAG_DMEIF1_5) != RESET)
  {
				DMA1->LIFCR = DMA_FLAG_DMEIF1_5;
  }
	
	LED2_Toggle;
}


//********************************************************************************/
//DMA END
//********************************************************************************/


//********************************************************************************/
//ADC
//********************************************************************************/


//********************************************************************************/
//初始化ADC
//ch: ADC_channels 
//通道值 0~16取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16

/******************************************************************************/	


void ADC_Cha_Config(void)
{
	ADC1_ChanConf.Channel = ADC_CHANNEL_10;
	ADC1_ChanConf.Rank = ADC_REGULAR_RANK_1;
	ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_64CYCLES_5; 
	ADC1_ChanConf.SingleDiff = ADC_SINGLE_ENDED;  //Single ended
	ADC1_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
	ADC1_ChanConf.Offset = 0;
	ADC1_ChanConf.OffsetRightShift = DISABLE;
	ADC1_ChanConf.OffsetSignedSaturation = DISABLE;
	
	HAL_ADC_ConfigChannel(&ADC1_Handler, &ADC1_ChanConf);
	
}


void ADC_Init(void)
{ 
	 __HAL_RCC_ADC12_CLK_ENABLE();
	ADC1_Handler.Instance=ADC1;
	//ADC1_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; 	//4分频，ADCCLK=PER_CK/4=200/4=50MHZ
	
	ADC1_Handler.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2; //PLL2 2 div (72MHz/2 = 36MHz)
	
	
	ADC1_Handler.Init.Resolution=ADC_RESOLUTION_16B;           	//16位模式
	ADC1_Handler.Init.ScanConvMode=ADC_SCAN_ENABLE;             //非扫描模式
	ADC1_Handler.Init.EOCSelection=ADC_EOC_SINGLE_CONV;       	//关闭EOC中断
	ADC1_Handler.Init.LowPowerAutoWait=DISABLE;					        //自动低功耗关闭				
	ADC1_Handler.Init.ContinuousConvMode=ENABLE;               //Open Continuous Convert
	ADC1_Handler.Init.NbrOfConversion=1;                        //1个转换在规则序列中 也就是只转换规则序列1 
	ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;            //禁止不连续采样模式
	ADC1_Handler.Init.NbrOfDiscConversion=0;                    //不连续采样通道数为0
	ADC1_Handler.Init.NbrOfConversion = 1;                    /* 1 Channle for sample*/
	ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;      //软件触发
	ADC1_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
	ADC1_Handler.Init.BoostMode=ENABLE;							                     
	ADC1_Handler.Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;			             //有新的数据的死后直接覆盖掉旧数据
	ADC1_Handler.Init.OversamplingMode=DISABLE;				                 	//过采样关闭
	ADC1_Handler.Init.ConversionDataManagement=ADC_CONVERSIONDATA_DMA_CIRCULAR;  //DMA circular mode recieve data


	HAL_ADC_Init(&ADC1_Handler);                                      //初始化 
	
	HAL_ADCEx_Calibration_Start(&ADC1_Handler,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); //ADC校准
	
	ADC_Cha_Config(); // ADC Channle config
	
	//__HAL_DMA_ENABLE(&DmaHandle);
	
	//HAL_ADC_Start_DMA(&ADC1_Handler, (uint32_t *)ADCxValues0, FFT_N*2); //Open DMA trans
	
//		__HAL_DMA_DISABLE_IT(&DmaHandle, DMA_IT_HT);
//		__HAL_DMA_ENABLE_IT(&DmaHandle, DMA_IT_TC);
	DMA1_Stream1->CR |= (1<<4); //Enabble DMA tranfer complete
	DMA1_Stream1->CR |= (1<<0); //Enable DMA
	
	HAL_ADC_Start(&ADC1_Handler);                               //开启ADC

	HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换
	
}


//********************************************************************************/
//ADC底层驱动，引脚配置，时钟使能
//此函数会被HAL_ADC_Init()调用
//hadc:ADC句柄
/******************************************************************************/	


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_ADC12_CLK_ENABLE();     //使能ADC1/2时钟
	__HAL_RCC_GPIOC_CLK_ENABLE();			//开启GPIOC时钟
	__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP); //ADC外设时钟选择

	GPIO_Initure.Pin=GPIO_PIN_0;            //PC0引脚，对应ADC1的CH10通道，更多引脚信息，请参考数据手册
	GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
	GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);
	
}



//********************************************************************************/
//ADC END
//********************************************************************************/

//********************************************************************************/
//Get DMA ADC result
/******************************************************************************/	
void Get_DMAValue(struct compx * signal)
{	
		int i = 0;
		//If first part of buff is acessable
	 if(DMA1Flag == 1)
	 {
			//DISABLE_INT();
		 for(i=0;i<FFT_N;i++){
				signal[i].real = ADCxValues0[i]*3.3/65536;
				signal[i].imag = 0;
		 }
      //ENABLE_INT();
	 }
	 else//If second part of buff is acessable
	 {
			//DISABLE_INT();
      for(i=0;i<FFT_N;i++){
				signal[i].real = ADCxValues1[i]*3.3/65536;
				signal[i].imag = 0;
			}
      //ENABLE_INT();	
	 }

}






//********************************************************************************/
//STM32H7工程模板-HAL库函数版本
/******************************************************************************/	
