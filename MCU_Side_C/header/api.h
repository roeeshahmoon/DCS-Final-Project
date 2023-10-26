#ifndef _api_H_
#define _api_H_

#include  "../header/halGPIO.h"     // private library - HAL layer

extern void Objects_Detector();
extern void Telemeter();
extern void Light_Detector();




//Scripts Mode function
extern void Inc_LCD(unsigned int X);
extern void Dec_LCD(unsigned int X);
extern void RRA_LCD(char X);
extern void Set_delay(unsigned int delay);
extern void Script_Mode();
extern void Light_and_Objects();



#endif

