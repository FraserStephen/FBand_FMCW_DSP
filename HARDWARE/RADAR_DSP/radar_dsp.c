
#include "radar_dsp.h"
#include <math.h>


#define PI 3.14159


uint16_t Fire = 0;


struct compx EE(struct compx a, struct compx b)
{
	struct compx c;
	c.real = a.real*b.real - a.imag*b.imag;
	c.imag = a.real*b.imag + a.imag*b.real;
	return (c);
}



///////////FFT FUNCTIONS//////////////////
void FFT(struct compx *xin, float* freq){
		int f,m,nv2,nm1,i,k,l,j=0;
		struct compx u,w,t;
		nv2=FFT_N/2;                 
	  nm1=FFT_N-1;
	
		//Switch Address
	  for(i=0;i<nm1;i++)
	  {
	    if(i<j)                  
	     {
	      t=xin[j];
	      xin[j]=xin[i];
	      xin[i]=t;
	     }
	    k=nv2;                   
	    while(k<=j)               
	    {
	      j=j-k;               
	      k=k/2;                 
	    }
	   j=j+k;                
	  }
		
		//Butterfly
	  {
	  int le,lei,ip;                  
	  f=FFT_N;
	  for(l=1;(f=f/2)!=1;l++);
	  for(m=1;m<=l;m++)                     
	   {                                   
	    le=2<<(m-1);                     
	    lei=le/2;                             
	    u.real=1.0;                          
	    u.imag=0.0;
	    w.real=cos((float)PI/(float)lei);            
	    w.imag=-sin((float)PI/(float)lei);
	    for(j=0;j<=lei-1;j++)                 
	     {
	      for(i=j;i<=FFT_N-1;i=i+le)            
	       {
	        ip=i+lei;                        
	        t=EE(xin[ip],u);         
	        xin[ip].real=xin[i].real-t.real;
	        xin[ip].imag=xin[i].imag-t.imag;
	        xin[i].real=xin[i].real+t.real;
	        xin[i].imag=xin[i].imag+t.imag;
					 
					freq[ip] = xin[ip].real*xin[ip].real + xin[ip].imag*xin[ip].imag;
					freq[i] = xin[i].real*xin[i].real + xin[i].imag*xin[i].imag;
	       }
	      u=EE(u,w);               
	     }
	   }
	  }
}

int CFAR(float * s){
	
	float z[L_num];
	
	int i = 0; int j = 0;
	s[0] = 0;
	for(i=0;i<L_num;i++) z[i] = 0; //Clear buff
	for(i=0;i<L_num;i++){
		for (j=0;j<L_slipper;j++)  {z[i] = z[i] + s[i+j];} // sum slipper window
		z[i] = (z[i] - s[i+R/2])/R*T;
	}
	
	for(i=Down_Range;i<Up_Range;i++){
		if( s[i] > z[i-Half_R]) 
		{
			Fire = 1;
		}
	}
	
	return 1;
}
		
		
		
	



