/*
 * IIRFilter.h
 *
 *  Created on: Dec 15, 2023
 *      Author: veasnathahashi
 */

#ifndef INC_IIRFILTER_H_
#define INC_IIRFILTER_H_
typedef struct {
	float alpha;
	float out;
}IIRFilter;


void IIRFilter_Init(IIRFilter *filter, float alpha);

float IIRFilter_Update(IIRFilter *filter, float in);



#endif /* INC_IIRFILTER_H_ */
