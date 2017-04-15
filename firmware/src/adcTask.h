/*
 * adcTask.h
 *
 *  Created on: Feb 2, 2016
 *      Author: corviniol
 */

#ifndef SRC_ADCTASK_H_
#define SRC_ADCTASK_H_

#include <stdint.h>

typedef struct
{
    uint8_t adcChannelsMask;
    unsigned uSampleRate;
    unsigned uPrsChannel;
    unsigned uTimer;
} AdcTaskParams_t;

extern void vAdcTask(void *pParameters);
extern void vDacTask(void *pParameters);


//typedef struct
//{
//    uint8_t validChannelsMask;
//    uint8_t validChannelsCount;
//    int Vdd_mV;
//    ADC_InitScan_TypeDef* const conf;
//    int readings[8];
//} AdcScanResults_t;
//
//void* pvResultFormater(const AdcScanResults_t *results, void* dst);
#endif /* SRC_ADCTASK_H_ */
