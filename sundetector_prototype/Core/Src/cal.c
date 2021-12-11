#include "cal.h"

uint16_t Devide_by2(uint16_t x, uint16_t y){

	return (x+y)/2;
}

uint16_t Apsolute_Val(uint16_t x, uint16_t y){
	if(x>=y)
		return x-y;
	else
		return y-x;
}
uint16_t Sensor_cal1(uint16_t x){
	if(x>1300)
	{
		return x*1.1-409;
	}
	else
	{
		return x*0.8;
	}
}

uint16_t Sensor_cal2(uint16_t x){
	if(x>700)
	{
		return x*0.9+409.5;
	}
	else
	{
		return x*1.5;
	}
}

