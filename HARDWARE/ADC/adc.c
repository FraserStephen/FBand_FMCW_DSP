#include "adc.h"
#include "radar_dsp.h"
#include "delay.h"
#include "stm32h7xx_hal.h"
#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 
//********************************************************************************/
//ADC Driver code
//Set GPIO pin to ADC sample with DMA

//PC4 ---- ADC1 Channel 4 ---- DMA1_Stream0
//PC5 ---- ADC1 Channel 8 ---- DMA1_Stream0

//PB0 ---- ADC2 Channel 9 ---- DMA1_Stream1
//PB1 ---- ADC2 Channel 5 ---- DMA1_Stream1

/******************************************************************************/							  
////////////////////////////////////////////////////////////////////////////////// 	

ADC_HandleTypeDef ADC1_Handler;//ADC句柄
ADC_HandleTypeDef ADC2_Handler;//ADC句柄 
ADC_ChannelConfTypeDef ADC1_ChanConf; //ADC Channel Handler
ADC_ChannelConfTypeDef ADC2_ChanConf; //ADC Channel Handler
DMA_HandleTypeDef DmaHandle1 = {0}; //DMA Handler for ADC1
DMA_HandleTypeDef DmaHandle2 = {0}; //DMA Handler for ADC2
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0}; //PLL Handler


uint32_t DMAFlag1 = 0;
uint32_t DMAFlag2 = 0;

int32_t sample_size = FFT_N*2; //FFT_N * Number of Channel each ADC
//dsize must euqal to FFT_N*2

__align(16) uint16_t ADC1Values0[FFT_N*2] __attribute__((at(0x38000000))); // ADC1 Buffer0
__align(16) uint16_t ADC1Values1[FFT_N*2] __attribute__((at(0x38000440))); // ADC1 Buffer1
__align(16) uint16_t ADC2Values0[FFT_N*2] __attribute__((at(0x38000880))); // ADC2 Buffer0
__align(16) uint16_t ADC2Values1[FFT_N*2] __attribute__((at(0x38000FB0))); // ADC2 Buffer1


//********************************************************************************/
//Init GPIO
/******************************************************************************/	
void GPIO_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOC_CLK_ENABLE(); 
	
	
	//Set PC4 for ADC Channel 4
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	//Set PC5 for ADC Channel 8
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	//Set PB0 for ADC Channel 9
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//Set PB1 for ADC Channel 5
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	

/*	
	//Set PC0 for ADC Channel 10
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	//Set PC1 for ADC Channel 11
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	
	//Set PC1 for ADC Channel 18
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	//Set PC1 for ADC Channel 19
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
*/
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
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
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
	DMA1_Stream1->M0AR = (uint32_t)ADC1Values0;
	DMA1_Stream1->M1AR = (uint32_t)ADC1Values1;	
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
		
		//////////////////////// Open DMA1_Stream1 for ADC1   //////////////////////////////
		DmaHandle1.Instance = DMA1_Stream1;  //Select DMA stream 1 for ADC1
		DmaHandle1.Init.Request = DMA_REQUEST_ADC1;  //Ask for ADC1
		DmaHandle1.Init.Direction = DMA_PERIPH_TO_MEMORY; 
		DmaHandle1.Init.PeriphInc = DMA_PINC_DISABLE;
		DmaHandle1.Init.MemInc = DMA_MINC_ENABLE;
		DmaHandle1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; //16bit
		DmaHandle1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	
		DmaHandle1.Init.Mode = DMA_CIRCULAR;//Circular mode
		DmaHandle1.Init.Priority = DMA_PRIORITY_VERY_HIGH ; 
		DmaHandle1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		DmaHandle1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		DmaHandle1.Init.MemBurst = DMA_MBURST_SINGLE; 
		DmaHandle1.Init.PeriphBurst = DMA_PBURST_SINGLE;
		
		HAL_DMA_Init(&DmaHandle1);
		
		DMA1_Stream1->CR |= 1<<18;  //Multi-Buff open
		DMA1_Stream1->CR &= ~(1<<19);	
		DMA1_Stream1->PAR = (uint32_t)&(ADC1->DR);
		DMA1_Stream1->M0AR = (uint32_t)ADC1Values0;
		DMA1_Stream1->M1AR = (uint32_t)ADC1Values1;	
		DMA1_Stream1->NDTR = FFT_N*2;	
		
		//Open IRQ of DMA
		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);	
		
		__HAL_LINKDMA(&ADC1_Handler, DMA_Handle, DmaHandle1); //bondle ADC and DMA 
		
		
		//////////////////////// Open DMA1_Stream2 for ADC2   //////////////////////////////	
		DmaHandle2.Instance = DMA1_Stream2;  //Select DMA stream 1 for ADC2
		DmaHandle2.Init.Request = DMA_REQUEST_ADC2;  //Ask for ADC1
		DmaHandle2.Init.Direction = DMA_PERIPH_TO_MEMORY; 
		DmaHandle2.Init.PeriphInc = DMA_PINC_DISABLE;
		DmaHandle2.Init.MemInc = DMA_MINC_ENABLE;
		DmaHandle2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; //16bit
		DmaHandle2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	
		DmaHandle2.Init.Mode = DMA_CIRCULAR;//Circular mode
		DmaHandle2.Init.Priority = DMA_PRIORITY_VERY_HIGH ; 
		DmaHandle2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		DmaHandle2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		DmaHandle2.Init.MemBurst = DMA_MBURST_SINGLE; 
		DmaHandle2.Init.PeriphBurst = DMA_PBURST_SINGLE;
		
		HAL_DMA_Init(&DmaHandle2);
		
		DMA1_Stream2->CR |= 1<<18;  //Multi-Buff open
		DMA1_Stream2->CR &= ~(1<<19);	
		DMA1_Stream2->PAR = (uint32_t)&(ADC2->DR);
		DMA1_Stream2->M0AR = (uint32_t)ADC2Values0;
		DMA1_Stream2->M1AR = (uint32_t)ADC2Values1;	
		DMA1_Stream2->NDTR = FFT_N*2;	
		
		//Open IRQ of DMA
		HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);	
		
		__HAL_LINKDMA(&ADC2_Handler, DMA_Handle, DmaHandle2); //bondle ADC and DMA 
			
}

//IRQ of DMA1_Stream1 for ADC1
void DMA1_Stream1_IRQHandler(void)
{
	//If DMA data trans finished
	if((DMA1->LISR & DMA_FLAG_TCIF1_5) != RESET)
	{     
				if((DMA1_Stream1->CR & (1<<19)) == 0){
					DMAFlag1 = 1; //First Buff is available
				}
				else{
					DMAFlag1 = 2; //Second Buff is available
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


//IRQ of DMA1_Stream2 for ADC2 
void DMA1_Stream2_IRQHandler(void)
{
	//If DMA data trans finished
	if((DMA1->LISR & DMA_FLAG_TCIF2_6) != RESET)
	{     
				if((DMA1_Stream2->CR & (1<<19)) == 0){
					DMAFlag2 = 1; //First Buff is available
				}
				else{
					DMAFlag2 = 2; //Second Buff is available
				}
				DMA1->LIFCR = DMA_FLAG_TCIF2_6;//Clear IRQ	
	}
	//If DMA data trans half
	if((DMA1->LISR & DMA_FLAG_HTIF2_6) != RESET)
	{
        DMA1->LIFCR = DMA_FLAG_HTIF2_6;//Clear
				
	}
	
  //DMA Trans Error
  if((DMA1->LISR & DMA_FLAG_TEIF2_6) != RESET)
	{
				DMA1->LIFCR = DMA_FLAG_TEIF2_6;
  } 
  if((DMA1->LISR & DMA_FLAG_DMEIF2_6) != RESET)
  {
				DMA1->LIFCR = DMA_FLAG_DMEIF2_6;
  }
	
}



//********************************************************************************/
//DMA END
//********************************************************************************/


//********************************************************************************/
//ADC
//********************************************************************************/


//********************************************************************************/
//初始化ADC
//Must init DMA fire (run after function DMA_Init())
//ch: ADC_channels 
//通道值 0~16取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
/******************************************************************************/	


void ADC_Cha_Config(void)
{
	// Channel 10 for ADC1
	ADC1_ChanConf.Channel = ADC_CHANNEL_4; //PC4
	ADC1_ChanConf.Rank = ADC_REGULAR_RANK_1;
	ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;  //Sampling time 32.5 time cycles
	ADC1_ChanConf.SingleDiff = ADC_SINGLE_ENDED;  //Single ended
	ADC1_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
	ADC1_ChanConf.Offset = 0;
	ADC1_ChanConf.OffsetRightShift = DISABLE;
	ADC1_ChanConf.OffsetSignedSaturation = DISABLE;
	
	HAL_ADC_ConfigChannel(&ADC1_Handler, &ADC1_ChanConf);
	
	// Channel 11 for ADC1
	ADC1_ChanConf.Channel = ADC_CHANNEL_8; //PC5
	ADC1_ChanConf.Rank = ADC_REGULAR_RANK_2;
	ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;  //Sampling time 32.5 time cycles
	ADC1_ChanConf.SingleDiff = ADC_SINGLE_ENDED;  //Single ended
	ADC1_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
	ADC1_ChanConf.Offset = 0;
	ADC1_ChanConf.OffsetRightShift = DISABLE;
	ADC1_ChanConf.OffsetSignedSaturation = DISABLE;
	
	HAL_ADC_ConfigChannel(&ADC1_Handler, &ADC1_ChanConf);
	
	
	// Channel 18 for ADC2
	ADC2_ChanConf.Channel = ADC_CHANNEL_9; //PB0
	ADC2_ChanConf.Rank = ADC_REGULAR_RANK_1;
	ADC2_ChanConf.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;  //Sampling time 32.5 time cycles
	ADC2_ChanConf.SingleDiff = ADC_SINGLE_ENDED;  //Single ended
	ADC2_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
	ADC2_ChanConf.Offset = 0;
	ADC2_ChanConf.OffsetRightShift = DISABLE;
	ADC2_ChanConf.OffsetSignedSaturation = DISABLE;
	
	HAL_ADC_ConfigChannel(&ADC2_Handler, &ADC2_ChanConf);
	
	
	// Channel 19 for ADC2
	ADC2_ChanConf.Channel = ADC_CHANNEL_5; //PB1
	ADC2_ChanConf.Rank = ADC_REGULAR_RANK_2;
	ADC2_ChanConf.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;  //Sampling time 32.5 time cycles
	ADC2_ChanConf.SingleDiff = ADC_SINGLE_ENDED;  //Single ended
	ADC2_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
	ADC2_ChanConf.Offset = 0;
	ADC2_ChanConf.OffsetRightShift = DISABLE;
	ADC2_ChanConf.OffsetSignedSaturation = DISABLE;
	
	HAL_ADC_ConfigChannel(&ADC2_Handler, &ADC2_ChanConf);
}


void ADC_Init(void)
{ 
	 __HAL_RCC_ADC12_CLK_ENABLE();
	
	
	/////////////////////  ADC1 Init   /////////////////////  
	ADC1_Handler.Instance=ADC1;
	//ADC1_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; 	//4分频，ADCCLK=PER_CK/4=200/4=50MHZ
	
	ADC1_Handler.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2; //PLL2 2 div (72MHz/2 = 36MHz)
	
	
	ADC1_Handler.Init.Resolution=ADC_RESOLUTION_16B;           	//16位模式
	ADC1_Handler.Init.ScanConvMode=ADC_SCAN_ENABLE;             //Enable the scan model for multi channels
	ADC1_Handler.Init.EOCSelection=ADC_EOC_SINGLE_CONV;       	//关闭EOC中断
	ADC1_Handler.Init.LowPowerAutoWait=DISABLE;					        //自动低功耗关闭				
	ADC1_Handler.Init.ContinuousConvMode=ENABLE;               //Open Continuous Convert
	ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;            //禁止不连续采样模式
	ADC1_Handler.Init.NbrOfDiscConversion=1;                    //不连续采样通道数为0
	ADC1_Handler.Init.NbrOfConversion = 2;                    /* 2 Channle for sample*/
	ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;      //软件触发
	ADC1_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
	ADC1_Handler.Init.BoostMode=ENABLE;							                     //Clock faster than 20MHz
	ADC1_Handler.Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;			             //有新的数据的死后直接覆盖掉旧数据
	ADC1_Handler.Init.OversamplingMode=DISABLE;				                 	//过采样关闭
	ADC1_Handler.Init.ConversionDataManagement=ADC_CONVERSIONDATA_DMA_CIRCULAR;  //DMA circular mode recieve data

	HAL_ADC_Init(&ADC1_Handler);                                      //初始化 	
	HAL_ADCEx_Calibration_Start(&ADC1_Handler,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); //ADC校准
	

	
	
	
	
	/////////////////////  ADC2 Init   /////////////////////  
	ADC2_Handler.Instance=ADC2;
	//ADC1_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4; 	//4分频，ADCCLK=PER_CK/4=200/4=50MHZ
	
	ADC2_Handler.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2; //PLL2 2 div (72MHz/2 = 36MHz)
	
	
	ADC2_Handler.Init.Resolution=ADC_RESOLUTION_16B;           	//16位模式
	ADC2_Handler.Init.ScanConvMode=ADC_SCAN_ENABLE;             //Enable the scan model for multi channels
	ADC2_Handler.Init.EOCSelection=ADC_EOC_SINGLE_CONV;       	//关闭EOC中断
	ADC2_Handler.Init.LowPowerAutoWait=DISABLE;					        //自动低功耗关闭				
	ADC2_Handler.Init.ContinuousConvMode=ENABLE;               //Open Continuous Convert
	ADC2_Handler.Init.NbrOfConversion=1;                        //1个转换在规则序列中 也就是只转换规则序列1 
	ADC2_Handler.Init.DiscontinuousConvMode=DISABLE;            //禁止不连续采样模式
	ADC2_Handler.Init.NbrOfDiscConversion=0;                    //不连续采样通道数为0
	ADC2_Handler.Init.NbrOfConversion = 2;                    /* 2 Channle for sample*/
	ADC2_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;      //软件触发
	ADC2_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
	ADC2_Handler.Init.BoostMode=ENABLE;							                     
	ADC2_Handler.Init.Overrun=ADC_OVR_DATA_OVERWRITTEN;			             //有新的数据的死后直接覆盖掉旧数据
	ADC2_Handler.Init.OversamplingMode=DISABLE;				                 	//过采样关闭
	ADC2_Handler.Init.ConversionDataManagement=ADC_CONVERSIONDATA_DMA_CIRCULAR;  //DMA circular mode recieve data

	HAL_ADC_Init(&ADC2_Handler);                                      //初始化 	
	HAL_ADCEx_Calibration_Start(&ADC2_Handler,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED); //ADC校准
		
	
	ADC_Cha_Config(); // ADC Channle config
	
	HAL_ADC_Start(&ADC1_Handler);                               //开启ADC1
	HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换
	
	HAL_ADC_Start(&ADC2_Handler);                               //开启ADC2
	HAL_ADC_PollForConversion(&ADC2_Handler,10);                //轮询转换
	
	
	DMA1_Stream1->CR |= (1<<4); //Enabble DMA tranfer complete IRQ
	DMA1_Stream1->CR |= (1<<0); //Enable DMA Stream1
	DMA1_Stream2->CR |= (1<<4); //Enabble DMA tranfer complete IRQ
	DMA1_Stream2->CR |= (1<<0); //Enable DMA Stream2
	
	
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
	
	GPIO_Initure.Pin=GPIO_PIN_1;            //PC1引脚，对应ADC1的CH11通道，更多引脚信息，请参考数据手册
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
void Get_DMAValue(struct compx * signal0, struct compx * signal1, struct compx * signal2, struct compx * signal3)
{	
		int i = 0;
		//ADC1 data trans
		//If first part of buff is acessable
	 if(DMAFlag1 == 1)
	 {
			//DISABLE_INT();
		 for(i=0;i<FFT_N;i++){
				signal0[i].real = ADC1Values0[2*i]*3.3/65536;
				signal0[i].imag = 0;
			 
			 	signal1[i].real = ADC1Values0[2*i+1]*3.3/65536;
				signal1[i].imag = 0;
		 }
      //ENABLE_INT();
	 }
	 else//If second part of buff is acessable
	 {
			//DISABLE_INT();
      for(i=0;i<FFT_N;i++){
				signal0[i].real = ADC1Values1[2*i]*3.3/65536;
				signal0[i].imag = 0;
				
				signal1[i].real = ADC1Values1[2*i+1]*3.3/65536;
				signal1[i].imag = 0;
			}
      //ENABLE_INT();	
	 }
	 
	 //ADC2 data trans
	 //If first part of buff is acessable
	 if(DMAFlag2 == 1)
	 {
			//DISABLE_INT();
		 for(i=0;i<FFT_N;i++){
				signal2[i].real = ADC2Values0[2*i]*3.3/65536;
				signal2[i].imag = 0;
			 
			 	signal3[i].real = ADC2Values0[2*i+1]*3.3/65536;
				signal3[i].imag = 0;
		 }
      //ENABLE_INT();
	 }
	 else//If second part of buff is acessable
	 {
			//DISABLE_INT();
      for(i=0;i<FFT_N;i++){
				signal2[i].real = ADC2Values1[2*i]*3.3/65536;
				signal2[i].imag = 0;
				
				signal3[i].real = ADC2Values1[2*i+1]*3.3/65536;
				signal3[i].imag = 0;
			}
      //ENABLE_INT();	
	 }
	 
}

