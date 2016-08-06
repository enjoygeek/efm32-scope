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

#endif /* SRC_ADCTASK_H_ */
