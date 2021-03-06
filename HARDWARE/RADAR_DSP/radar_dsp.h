#ifndef __DSP_H
#define __DSP_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
 
//********************************************************************************/
// DSP ����	  
/******************************************************************************/							  
////////////////////////////////////////////////////////////////////////////////// 	

#define  FFT_N  256 

//CFAR define

#define R                8      // R < FFT_N/2
#define Half_R           R/2
#define L_slipper        (R+1)
#define L_num            (FFT_N - L_slipper + 1) //L_num = floor((M-L_slipper)/L_move)+1;
#define T                5  //T = P_fa^(-1/R)-1  P_fa = 1e-4
#define Up_Range         85     //Up_Range < FFT_N - Half_R
#define Down_Range       75     //Down_Range > Half_R

#define Fire_Hold        5
#define Sample_Number    10



struct compx {float real, imag;};

struct compx EE(struct compx a, struct compx b);

void Get_DMAValue(struct compx * signal0, struct compx * signal1, struct compx * signal2, struct compx * signal3);

void FFT(struct compx * xin, float* freq); 				//
int CFAR(float * s);

#endif 




/********************************************************************************/
//ADC ʵ�� 
/******************************************************************************/



