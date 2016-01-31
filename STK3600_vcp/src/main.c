/**************************************************************************//**
 * @file main.c
 * @brief USB Composite Device example.
 * @version 3.20.12
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "em_chip.h"
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_dac.h"
#include "bsp.h"
#include "../Drivers/segmentlcd.h"
#include "bsp_trace.h"
#include "sleep.h"

#include "../Drivers/cdc.h"



#define STACK_SIZE_FOR_TASK    (configMINIMAL_STACK_SIZE + 10)
#define TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

/* Structure with parameters for LedBlink */
typedef struct
{
  /* Delay between blink of led */
  portTickType delay;
  /* Number of led */
  int          ledNo;
} LedTaskParams_t;



/**************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 *****************************************************************************/
static void LedTask(void *pParameters)
{
	vTaskDelay(1000 / portTICK_RATE_MS);

  LedTaskParams_t     * pData = (LedTaskParams_t*) pParameters;
  const portTickType delay = pData->delay;

  for (;;)
  {
    BSP_LedToggle(pData->ledNo);
    vTaskDelay(delay);
    printf("%s. FreeHeap: %d\n", pcTaskGetTaskName(NULL), xPortGetFreeHeapSize());
  }
}

static void AdcTask(void *pParameters)
{
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef init_single = ADC_INITSINGLE_DEFAULT;
	init.warmUpMode = adcWarmupKeepADCWarm;
	init.timebase = 0;
	init.prescale = ADC_PrescaleCalc(1000, 0);
	ADC_Init(ADC0, &init);

	init_single.reference = adcRef1V25;
//	init_single.input = adcSingleInputDACOut0;
	init_single.input = adcSingleInputVDDDiv3;
	init_single.rep = true;
	init_single.leftAdjust = true;
	ADC_InitSingle(ADC0, &init_single);

	ADC_Start(ADC0, adcStartSingle);
	for (;;)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
		while (ADC0->STATUS & ADC_STATUS_SINGLEDV == 0)
			;
		int res = ADC_DataSingleGet(ADC0);
		printf("ADC VDD: %dmV\n", (res*3*1250) >> 16);
	}
}

static void DacTask(void *pParameters)
{
	CMU_ClockEnable(cmuClock_DAC0, true);
	DAC_Init_TypeDef init = DAC_INIT_DEFAULT;
	DAC_InitChannel_TypeDef channel = DAC_INITCHANNEL_DEFAULT;

	init.reference = dacRefVDD;
	init.outMode = dacOutputPinADC;
	init.prescale = DAC_PrescaleCalc(1000, 0);

	DAC_Init(DAC0, &init);

	channel.refreshEnable = true;
	DAC_InitChannel(DAC0, &channel, 0);

	DAC_Channel0OutputSet(DAC0, 100);
	DAC_Enable(DAC0, 0, true);
}

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main( void )
{
	CHIP_Init();
  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
  CMU_OscillatorEnable(cmuOsc_LFXO, true, false);

  /* Initialize LCD driver */
  SegmentLCD_Init(false);
  SegmentLCD_Write("usbcomp");
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, true);

  /* Initialize LED driver */
  BSP_LedsInit();

   /* Initialize SLEEP driver, no calbacks are used */
   SLEEP_Init(NULL, NULL);
 #if (configSLEEP_MODE < 3)
   /* do not let to sleep deeper than define */
   SLEEP_SleepBlockBegin((SLEEP_EnergyMode_t)(configSLEEP_MODE+1));
 #endif

   /* Parameters value for taks*/
   static LedTaskParams_t parametersToTask1 = { 1000 / portTICK_RATE_MS, 0 };
   static LedTaskParams_t parametersToTask2 = { 500 / portTICK_RATE_MS, 1 };

   /*Create two task for blinking leds*/
   xTaskCreate( UsbCDCTask, "UsbCDC", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
//   xTaskCreate( LedTask, (const char *) "LedBlink1", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
//   xTaskCreate( LedTask, (const char *) "LedBlink2", STACK_SIZE_FOR_TASK, &parametersToTask2, TASK_PRIORITY, NULL);
   xTaskCreate( AdcTask, "ADC", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);
   xTaskCreate( DacTask, "DAC", STACK_SIZE_FOR_TASK, NULL, TASK_PRIORITY, NULL);

   NVIC_SetPriority(USB_IRQn, 7);
   vTaskStartScheduler();
}
