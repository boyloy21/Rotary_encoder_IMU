/*
 * IIRFilter.c
 *
 *  Created on: Dec 23, 2023
 *      Author: veasnathahashi
 */


#include "IIRFilter.h"

void IIRFilter_Init(IIRFilter *filter, float alpha){
	filter->alpha = alpha;
	filter->out	  = 0.0f;
}

float IIRFilter_Update(IIRFilter *filter, float in){
	filter->out = filter->alpha * filter->out + ( 1.0f - filter->alpha ) * in;
	return filter->out;
}
