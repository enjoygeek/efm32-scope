/**************************************************************************//**
 * @file cdc.c
 * @brief USB Communication Device Class (CDC) driver.
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
#include "em_device.h"
#include "em_common.h"
#include "em_cmu.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_usb.h"
#include "core_cm3.h"
#include "bsp.h"
#include "dmactrl.h"
#include "cdc.h"
#include "descriptors.h"

#include "../src/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/**************************************************************************//**
 * @addtogroup Cdc
 * @{ Implements USB Communication Device Class (CDC).

@section cdc_intro CDC implementation.

   The source code of the CDC implementation resides in
   kits/common/drivers/cdc.c and cdc.h. This driver implements a basic
   USB to RS232 bridge.

@section cdc_config CDC device configuration options.

  This section contains a description of the configuration options for
  the driver. The options are @htmlonly #define's @endhtmlonly which are
  expected to be found in the application "usbconfig.h" header file.
  The values shown below are from the Giant Gecko DK3750 CDC example.

  @verbatim
// USB interface numbers. Interfaces are numbered from zero to one less than
// the number of concurrent interfaces supported by the configuration. A CDC
// device is by itself a composite device and has two interfaces.
// The interface numbers must be 0 and 1 for a standalone CDC device, for a
// composite device which includes a CDC interface it must not be in conflict
// with other device interfaces.
#define CDC_CTRL_INTERFACE_NO ( 0 )
#define CDC_DATA_INTERFACE_NO ( 1 )

// Endpoint address for CDC data reception.
#define CDC_EP_DATA_OUT ( 0x01 )

// Endpoint address for CDC data transmission.
#define CDC_EP_DATA_IN ( 0x81 )

// Endpoint address for the notification endpoint (not used).
#define CDC_EP_NOTIFY ( 0x82 )

// Timer id, see USBTIMER in the USB device stack documentation.
// The CDC driver has a Rx timeout functionality which require a timer.
#define CDC_TIMER_ID ( 0 )
  @endverbatim
 ** @} ***********************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/*** Typedef's and defines. ***/

#define CDC_BULK_EP_SIZE  (USB_FS_BULK_EP_MAXSIZE) /* This is the max. ep size.    */
#define CDC_USB_RX_BUF_SIZ  CDC_BULK_EP_SIZE /* Packet size when receiving on USB. */
#define CDC_USB_TX_BUF_SIZ  127    /* Packet size when transmitting on USB.  */

/* Calculate a timeout in ms corresponding to 5 char times on current     */
/* baudrate. Minimum timeout is set to 10 ms.                             */
#define CDC_RX_TIMEOUT    EFM32_MAX(10U, 50000 / (cdcLineCoding.dwDTERate))

/* The serial port LINE CODING data structure, used to carry information  */
/* about serial port baudrate, parity etc. between host and device.       */
EFM32_PACK_START(1)
typedef struct
{
  uint32_t dwDTERate;               /** Baudrate                            */
  uint8_t  bCharFormat;             /** Stop bits, 0=1 1=1.5 2=2            */
  uint8_t  bParityType;             /** 0=None 1=Odd 2=Even 3=Mark 4=Space  */
  uint8_t  bDataBits;               /** 5, 6, 7, 8 or 16                    */
  uint8_t  dummy;                   /** To ensure size is a multiple of 4 bytes */
} __attribute__ ((packed)) cdcLineCoding_TypeDef;
EFM32_PACK_END()


/*** Function prototypes. ***/

static int  UsbDataReceived(USB_Status_TypeDef status, uint32_t xferred,
                            uint32_t remaining);
static int  LineCodingReceived(USB_Status_TypeDef status,
                               uint32_t xferred,
                               uint32_t remaining);

/*** Variables ***/

/*
 * The LineCoding variable must be 4-byte aligned as it is used as USB
 * transmit and receive buffer.
 */
EFM32_ALIGN(4)
EFM32_PACK_START(1)
static cdcLineCoding_TypeDef __attribute__ ((aligned(4))) cdcLineCoding =
{
  115200, 0, 0, 8, 0
};
EFM32_PACK_END()

STATIC_UBUF(usbRxBuffer0,  CDC_USB_RX_BUF_SIZ);   /* USB receive buffers.   */
STATIC_UBUF(usbRxBuffer1,  CDC_USB_RX_BUF_SIZ);

const uint8_t  *usbRxBuffer[  2 ] = { usbRxBuffer0, usbRxBuffer1 };

static int            usbRxIndex = 0;

#define BUF_COUNT 6

static Buffer txBuffers[BUF_COUNT];
static int usbTxIndex = 0;
//static char usbTxBuffers[128] __attribute__((aligned(4)));
//static int txLen;

int SetupCmd(const USB_Setup_TypeDef *setup);
void StateChangeEvent( USBD_State_TypeDef oldState,
                       USBD_State_TypeDef newState );

static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,
  .usbStateChange  = StateChangeEvent,
  .setupCmd        = SetupCmd,
  .isSelfPowered   = NULL,
  .sofInt          = NULL
};

const USBD_Init_TypeDef usbInitStruct =
{
  .deviceDescriptor    = &USBDESC_deviceDesc,
  .configDescriptor    = USBDESC_configDesc,
  .stringDescriptors   = USBDESC_strings,
  .numberOfStrings     = sizeof(USBDESC_strings)/sizeof(void*),
  .callbacks           = &callbacks,
  .bufferingMultiplier = USBDESC_bufferingMultiplier,
  .reserved            = 0
};


static SemaphoreHandle_t semTx = NULL;
static SemaphoreHandle_t semRecv = NULL;

QueueHandle_t fullQueue = NULL;
QueueHandle_t emptyQueue = NULL;
/** @endcond */


int _read(int file, char *ptr, int len)
{
  int c, rxCount = 0;

  (void) file;

//  while (len--)
//  {
//    if ((c = RETARGET_ReadChar()) != -1)
//    {
//      *ptr++ = c;
//      rxCount++;
//    }
//    else
//    {
//      break;
//    }
//  }
//
//  if (rxCount <= 0)
//  {
//    return -1;                        /* Error exit */
//  }

  return rxCount;
}

SemaphoreHandle_t write_sem = NULL;
int _write(int file, const char *ptr, int len)
{
	BufferPtr pEmptyBuf;
	BaseType_t ret = xQueueReceive(emptyQueue, &pEmptyBuf, portMAX_DELAY);
	memcpy(pEmptyBuf->buf, ptr, len);
	pEmptyBuf->used_bytes = len;
	xQueueSendToBack(fullQueue, &pEmptyBuf, portMAX_DELAY);

	return len;
}

static int UsbDataReceived(USB_Status_TypeDef status,
                           uint32_t xferred,
                           uint32_t remaining);
static int UsbDataTransmitted(USB_Status_TypeDef status,
                              uint32_t xferred,
                              uint32_t remaining);

static bool portOpen = false;

void UsbCDCTask(void *pParameters)
{
//	setvbuf(stdout, txBuffers[0].buf, _IOLBF, sizeof(txBuffers[0].buf));

	fullQueue = xQueueCreate(BUF_COUNT, sizeof(BufferPtr));
	emptyQueue = xQueueCreate(BUF_COUNT, sizeof(BufferPtr));
	for (int i = 0; i < BUF_COUNT; i++)
	{
		BufferPtr buf_ptr = &txBuffers[i];
		xQueueSendToBack(emptyQueue, &buf_ptr, portMAX_DELAY);
	}

	/* Initialize and start USB device stack. */
	USBD_Init(&usbInitStruct);

	/*
	* When using a debugger it is practical to uncomment the following three
	* lines to force host to re-enumerate the device.
	*/
//	USBD_Disconnect();
//	USBTIMER_DelayMs( 1000 );
//	USBD_Connect();
	for(;;)
	{
		BufferPtr pFilledBuffer;
		xQueueReceive(fullQueue, &pFilledBuffer, portMAX_DELAY);
		if ((pFilledBuffer->used_bytes > 0) && portOpen)
		{
			while (USBD_EpIsBusy(CDC_EP_DATA_IN))
				;
			USBD_Write(CDC_EP_DATA_IN, pFilledBuffer->buf, pFilledBuffer->used_bytes, NULL);
		}
		xQueueSendToBack(emptyQueue, &pFilledBuffer, portMAX_DELAY);
	}
}


/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements CDC class specific commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/
int CDC_SetupCmd(const USB_Setup_TypeDef *setup)
{
  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ( ( setup->Type      == USB_SETUP_TYPE_CLASS          ) &&
       ( setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE )    )
  {
    switch (setup->bRequest)
    {
    case USB_CDC_GETLINECODING:
      /********************/
      if ( ( setup->wValue    == 0                     ) &&
           ( setup->wIndex    == CDC_CTRL_INTERFACE_NO ) && /* Interface no. */
           ( setup->wLength   == 7                     ) && /* Length of cdcLineCoding. */
           ( setup->Direction == USB_SETUP_DIR_IN      )    )
      {
        /* Send current settings to USB host. */
        USBD_Write(0, (void*) &cdcLineCoding, 7, NULL);
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_CDC_SETLINECODING:
      /********************/
      if ( ( setup->wValue    == 0                     ) &&
           ( setup->wIndex    == CDC_CTRL_INTERFACE_NO ) && /* Interface no. */
           ( setup->wLength   == 7                     ) && /* Length of cdcLineCoding. */
           ( setup->Direction != USB_SETUP_DIR_IN      )    )
      {
        /* Get new settings from USB host. */
        USBD_Read(0, (void*) &cdcLineCoding, 7, LineCodingReceived);
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_CDC_SETCTRLLINESTATE:
      /********************/
      if ( ( setup->wIndex  == CDC_CTRL_INTERFACE_NO ) &&   /* Interface no.  */
           ( setup->wLength == 0                     )    ) /* No data.       */
      {
        /* Do nothing ( Non compliant behaviour !! ) */
    	  portOpen = (setup->wValue != 0);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  return retVal;
}

/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts CDC operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
void CDC_StateChangeEvent( USBD_State_TypeDef oldState,
                           USBD_State_TypeDef newState)
{
  if (newState == USBD_STATE_CONFIGURED)
  {
    /* We have been configured, start CDC functionality ! */

    if (oldState == USBD_STATE_SUSPENDED)   /* Resume ?   */
    {
    }

    /* Start receiving data from USB host. */
    USBD_Read(CDC_EP_DATA_OUT, (void*) usbRxBuffer[ usbRxIndex ],
              CDC_USB_RX_BUF_SIZ, UsbDataReceived);
  }

  if (oldState == USBD_STATE_CONFIGURED)
  {
	  USBD_AbortTransfer(CDC_EP_DATA_OUT);
  }
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/**************************************************************************//**
 * @brief Callback function called whenever a new packet with data is received
 *        on USB.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/

static int UsbDataReceived(USB_Status_TypeDef status,
                           uint32_t xferred,
                           uint32_t remaining)
{
  (void) remaining;            /* Unused parameter. */

  if ((status == USB_STATUS_OK) && (xferred > 0))
  {
	xSemaphoreGiveFromISR(semRecv, NULL);

	usbRxIndex ^= 1;
	USBD_Read(CDC_EP_DATA_OUT, (void*) usbRxBuffer[ usbRxIndex ],
			  CDC_USB_RX_BUF_SIZ, UsbDataReceived);
  }
  return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief Callback function called whenever a packet with data has been
 *        transmitted on USB
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int UsbDataTransmitted(USB_Status_TypeDef status,
                              uint32_t xferred,
                              uint32_t remaining)
{
  (void) xferred;              /* Unused parameter. */
  (void) remaining;            /* Unused parameter. */

  if (status == USB_STATUS_OK)
  {
	  Buffer *buf;
	  BaseType_t woken;
	  bool need_yield = false;
	  xQueueReceiveFromISR(fullQueue, &buf, &woken);
	  need_yield |= woken == pdTRUE;

	  xQueueSendToBackFromISR(emptyQueue, &buf, &woken);
	  need_yield |= woken == pdTRUE;

	  if (need_yield)
	  {
		  taskYIELD();
	  }
  }
  return USB_STATUS_OK;
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a CDC_SET_LINECODING
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK if data accepted.
 *         USB_STATUS_REQ_ERR if data calls for modes we can not support.
 *****************************************************************************/
static int LineCodingReceived(USB_Status_TypeDef status,
                              uint32_t xferred,
                              uint32_t remaining)
{
  (void) remaining;

  /* We have received new serial port communication settings from USB host. */
  if ((status == USB_STATUS_OK) && (xferred == 7))
  {
    return USB_STATUS_OK;
  }
  return USB_STATUS_REQ_ERR;
}

/**************************************************************************//**
 * @brief
 *   Called whenever a USB setup command is received.
 *
 * @param[in] setup
 *   Pointer to an USB setup packet.
 *
 * @return
 *   An appropriate status/error code. See USB_Status_TypeDef.
 *****************************************************************************/
int SetupCmd(const USB_Setup_TypeDef *setup)
{
  int retVal;

  retVal = CDC_SetupCmd( setup );

  return retVal;
}

/**************************************************************************//**
 * @brief
 *   Called whenever the USB device has changed its device state.
 *
 * @param[in] oldState
 *   The device USB state just leaved. See USBD_State_TypeDef.
 *
 * @param[in] newState
 *   New (the current) USB device state. See USBD_State_TypeDef.
 *****************************************************************************/
void StateChangeEvent( USBD_State_TypeDef oldState,
                       USBD_State_TypeDef newState )
{
  /* Call device StateChange event handlers for all relevant functions within
     the composite device. */

  CDC_StateChangeEvent(  oldState, newState );
}

/** @endcond */
