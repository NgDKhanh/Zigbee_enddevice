/*
 * ADC_lib.h
 *
 *  Created on: 12 Aug 2023
 *      Author: admin
 */

#ifndef ADC_LIB_H_
#define ADC_LIB_H_

void ADC_init(void);

void ADC_read(float *measured_uv);

void ADC_read_bat(float *measured_bat);

#endif /* ADC_LIB_H_ */
