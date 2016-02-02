/*
 * adcTask.c
 *
 *  Created on: Feb 2, 2016
 *      Author: corviniol
 */

#include "adcTask.h"

#include "em_cmu.h"
#include "em_adc.h"
#include "em_dac.h"
#include "em_prs.h"
#include "em_timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t semADC = NULL;
void ADC0_IRQHandler()
{
	ADC_IntClear(ADC0, ADC_IFC_SCAN);
	BaseType_t woken;
	xSemaphoreGiveFromISR(semADC, &woken);
	if (woken == pdTRUE)
	{
		taskYIELD();
	}
	BSP_LedToggle(0);
}

void vAdcTask(void *pParameters)
{
	semADC = xSemaphoreCreateBinary();

	CMU_ClockEnable(cmuClock_ADC0, true);
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	init.warmUpMode = adcWarmupKeepADCWarm;
	init.timebase = 0;
	init.prescale = ADC_PrescaleCalc(32000, 0);
	ADC_Init(ADC0, &init);

	NVIC_EnableIRQ(ADC0_IRQn);
	ADC_IntEnable(ADC0, ADC_IF_SCAN);


	ADC_InitSingle_TypeDef init_single =
	{
		.reference = adcRef1V25,
		.input = adcSingleInputCh5,
		.resolution = adcRes12Bit,
		.acqTime = adcAcqTime32,
		.rep = true,
		.leftAdjust = true
	};
//	ADC_InitSingle(ADC0, &init_single);

	ADC_InitScan_TypeDef init_scan =
	{
		.reference = adcRef1V25,
		.input = ADC_SCANCTRL_INPUTMASK_CH4 | ADC_SCANCTRL_INPUTMASK_CH5,
		.resolution = adcRes12Bit,
		.acqTime = adcAcqTime32,
		.rep = false,
		.leftAdjust = true,
		.prsEnable = true,
		.prsSel = adcPRSSELCh5
	};
	ADC_InitScan(ADC0, &init_scan);

	CMU_ClockEnable(cmuClock_TIMER3, true);
	TIMER_Init_TypeDef timer_init = TIMER_INIT_DEFAULT;
	timer_init.prescale = timerPrescale1024;
//	timer_init.enable = false;
	TIMER_TopSet(TIMER3, 48000);
	TIMER_Init(TIMER3, &timer_init);

	CMU_ClockEnable(cmuClock_PRS, true);
	PRS_LevelSet(0, 1 << (5 + _PRS_SWLEVEL_CH0LEVEL_SHIFT));
	PRS_SourceSignalSet(5, PRS_CH_CTRL_SOURCESEL_TIMER3, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgePos);
//	ADC_Start(ADC0, adcStartSingle);
//	for (;;)
//	{
//		while ((ADC0->STATUS & ADC_STATUS_SINGLEDV) == 0)
//			;
//		int res = ADC_DataSingleGet(ADC0);
//		printf("ADC VDD: %dmV\n", (res*1250) >> 16);
//	}
//	ADC_Start(ADC0, adcStartScan);
	for (;;)
	{
//		while ((ADC0->STATUS & ADC_STATUS_SCANDV) == 0)
//			;
		xSemaphoreTake(semADC, portMAX_DELAY);
		int n1 = (ADC0->STATUS & _ADC_STATUS_SCANDATASRC_MASK) >> _ADC_STATUS_SCANDATASRC_SHIFT;
		int res1 = ADC_DataScanGet(ADC0);

//		while ((ADC0->STATUS & ADC_STATUS_SCANDV) == 0)
//			;
		xSemaphoreTake(semADC, portMAX_DELAY);
		int n2 = (ADC0->STATUS & _ADC_STATUS_SCANDATASRC_MASK) >> _ADC_STATUS_SCANDATASRC_SHIFT;
		int res2 = ADC_DataScanGet(ADC0);
		printf("%d: %dmV , %d: %dmV\n", n1, (res1*1250) >> 16, n2, (res2*1250) >> 16);
	}
}

void vDacTask(void *pParameters)
{
	CMU_ClockEnable(cmuClock_DAC0, true);
	DAC_Init_TypeDef init = DAC_INIT_DEFAULT;
	DAC_InitChannel_TypeDef channel = DAC_INITCHANNEL_DEFAULT;

	init.reference = dacRefVDD;
	init.outMode = dacOutputPin;
	init.prescale = DAC_PrescaleCalc(10000, 0);

	DAC_Init(DAC0, &init);

	channel.refreshEnable = true;
	DAC_InitChannel(DAC0, &channel, 0);
	DAC_InitChannel(DAC0, &channel, 1);

	DAC_Enable(DAC0, 0, true);
	DAC_Enable(DAC0, 1, true);
	DAC_Channel0OutputSet(DAC0, 100);
	DAC_Channel1OutputSet(DAC0, 1000);
}


