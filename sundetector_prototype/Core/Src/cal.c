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
