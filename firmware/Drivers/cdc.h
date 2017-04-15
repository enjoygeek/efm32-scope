/***************************************************************************//**
 * @file  cdc.h
 * @brief USB Communication Device Class (CDC) driver.
 * @version 4.1.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/
#ifndef __CDC_H
#define __CDC_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"

/***************************************************************************//**
 * @addtogroup Drivers
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Cdc
 * @{
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif


//typedef struct
//{
//  SemaphoreHandle_t semRecv;
//  SemaphoreHandle_t semTx;
//} CdcTaskParams_t;

typedef struct
{
    char buf[127] __attribute__((aligned(4)));
    int used_bytes;
} Buffer;

typedef Buffer* BufferPtr;

extern void UsbCDCTask(void *pParameters);

#ifdef __cplusplus
}
#endif

/** @} (end group Cdc) */
/** @} (end group Drivers) */

#endif /* __CDC_H */
