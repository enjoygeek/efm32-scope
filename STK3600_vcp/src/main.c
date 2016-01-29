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
#include "bsp.h"
#include "../Drivers/segmentlcd.h"
#include "bsp_trace.h"
#include "sleep.h"

//#include "em_usb.h"
//#include "msdd.h"
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


//typedef struct
//{
//	SemaphoreHandle_t semRecv;
//	SemaphoreHandle_t semTx;
//} CdcTaskParams_t;


/**************************************************************************//**
 *
 * This example shows how a Composite USB Device can be implemented.
 *
 *****************************************************************************/

//int SetupCmd(const USB_Setup_TypeDef *setup);
//void StateChangeEvent( USBD_State_TypeDef oldState,
//                       USBD_State_TypeDef newState );
//
//static const USBD_Callbacks_TypeDef callbacks =
//{
//  .usbReset        = NULL,
//  .usbStateChange  = StateChangeEvent,
//  .setupCmd        = SetupCmd,
//  .isSelfPowered   = NULL,
//  .sofInt          = NULL
//};
//
//const USBD_Init_TypeDef usbInitStruct =
//{
//  .deviceDescriptor    = &USBDESC_deviceDesc,
//  .configDescriptor    = USBDESC_configDesc,
//  .stringDescriptors   = USBDESC_strings,
//  .numberOfStrings     = sizeof(USBDESC_strings)/sizeof(void*),
//  .callbacks           = &callbacks,
//  .bufferingMultiplier = USBDESC_bufferingMultiplier,
//  .reserved            = 0
//};

//SemaphoreHandle_t semRecv;
//SemaphoreHandle_t semTx;

/**************************************************************************//**
 * @brief Simple task which is blinking led
 * @param *pParameters pointer to parameters passed to the function
 *****************************************************************************/
static void LedTask(void *pParameters)
{
	vTaskDelay(5000 / portTICK_RATE_MS);

  LedTaskParams_t     * pData = (LedTaskParams_t*) pParameters;
  const portTickType delay = pData->delay;

  for (;;)
  {
    BSP_LedToggle(pData->ledNo);
    vTaskDelay(delay);
    printf("!\n");
  }
}

//extern const uint8_t  *usbRxBuffer[  2 ];
//extern int            usbRxIndex, usbBytesReceived;
//
//static const char *txBuffer[256];
//static int txLen;
//static void UsbCDCTask(void *pParameters)
//{
//	txLen = 0;
//
//	CdcTaskParams_t *pData = (CdcTaskParams_t*) pParameters;
//	CDC_Init();                   /* Initialize the communication class device. */
//
//
//	/* Initialize and start USB device stack. */
//	int ret = USBD_Init(&usbInitStruct);
//
//	/*
//	* When using a debugger it is practical to uncomment the following three
//	* lines to force host to re-enumerate the device.
//	*/
//	USBD_Disconnect();
//	USBTIMER_DelayMs( 1000 );
//	USBD_Connect();
//	for(;;)
//	{
//		xSemaphoreTake(semTx, portMAX_DELAY);
//		if (txLen > 0)
//		{
//			USBD_Write(CDC_EP_DATA_IN, txBuffer, txLen, NULL);
//		}
//	}
//}


int RETARGET_ReadChar(void)
{
	return 0;
}

//int _write(int file, const char *ptr, int len)
//{
//	memcpy(txBuffer, ptr, len);
//	txLen = len;
//	xSemaphoreGive(semTx);
//	return len;
//}

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
extern void UsbCDCTask(void *pParameters);
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

   CdcTaskParams_t parametersToCdc;
   parametersToCdc.semRecv = xSemaphoreCreateCounting(1000, 0);
   parametersToCdc.semTx = xSemaphoreCreateCounting(1000, 0);


//   setvbuf(stdout, NULL, _IOLBF, 16);
   /*Create two task for blinking leds*/
//   xTaskCreate( LedTask, (const char *) "LedBlink1", STACK_SIZE_FOR_TASK, &parametersToTask1, TASK_PRIORITY, NULL);
   xTaskCreate( LedTask, (const char *) "LedBlink2", STACK_SIZE_FOR_TASK, &parametersToTask2, TASK_PRIORITY, NULL);
   xTaskCreate( UsbCDCTask, "UsbCDC", STACK_SIZE_FOR_TASK*10, &parametersToCdc, TASK_PRIORITY, NULL);


   NVIC_SetPriority(USB_IRQn, 7);
   vTaskStartScheduler();
}
