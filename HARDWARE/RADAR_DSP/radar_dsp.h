#ifndef __DSP_H
#define __DSP_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
 
//********************************************************************************/
// DSP ด๚ย๋	  
/******************************************************************************/							  
////////////////////////////////////////////////////////////////////////////////// 	

#define  FFT_N  256 

//CFAR define

#define R                16      // R < FFT_N/2
#define Half_R           R/2
#define L_slipper        (R+1)
#define L_num            (FFT_N - L_slipper + 1) //L_num = floor((M-L_slipper)/L_move)+1;
#define T                2.6523  //T = P_fa^(-1/R)-1  P_fa = 1e-4
#define Up_Range         180     //Up_Range < FFT_N - Half_R
#define Down_Range       200     //Down_Range > Half_R


struct compx {float real, imag;};

struct compx EE(struct compx a, struct compx b);

void Get_DMAValue(struct compx * signal);

void FFT(struct compx * xin, float* freq); 				//
int CFAR(float * s);

#endif 




/********************************************************************************/
//ADC สตั้ 
/******************************************************************************/






