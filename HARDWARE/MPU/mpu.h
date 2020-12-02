#ifndef __MPU_H
#define __MPU_H
#include "sys.h"
	

//u8 MPU_Set_Protection(u32 baseaddr,u32 size,u32 rnum,u32 ap);
u8 MPU_Set_Protection(u32 baseaddr,u32 size,u32 rnum,u8 ap,u8 sen,u8 cen,u8 ben);
void MPU_Memory_Protection(void);
#endif




























/************************************************/

//MPU驱动代码	  
 
//STM32H7工程模板-HAL库函数版本
//DevEBox  大越创新
//淘宝店铺：mcudev.taobao.com
//淘宝店铺：shop389957290.taobao.com		

/************************************************/	



