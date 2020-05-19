/* Includes ------------------------------------------------------------------*/
#include <global_callbacks.h>
#include "usbd_cdc_if.h"

#include "main.h"
#include "cppmain.h"
#include "usbd_composite.h"
#include "constants.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  512
/* USER CODE END PRIVATE_DEFINES */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE] = {0};

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE] = {0};

/* USER CODE BEGIN PRIVATE_VARIABLES */
//uint8_t tempbuf[7] = {0,0,0,0,0,0,0};
USBD_CDC_LineCodingTypeDef line_coding;
uint8_t cdc_dtr = 0;


extern USBD_HandleTypeDef hUsbDeviceFS;


static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);


USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};


static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}


static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
    	memcpy(&line_coding,pbuf,sizeof(USBD_CDC_LineCodingTypeDef));
	break;

    case CDC_GET_LINE_CODING:
	    memcpy(pbuf,&line_coding,sizeof(USBD_CDC_LineCodingTypeDef));
	break;

    case CDC_SET_CONTROL_LINE_STATE:
    {
    	// DTR
    	USBD_SetupReqTypedef* req = (USBD_SetupReqTypedef*)pbuf;
    	cdc_dtr = (req->wValue & 0x0001);
    }
    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */

  //memcpy(Buf,UserRxBufferFS, length);
	//_write(0,(char*)UserRxBufferFS,*Len);
  CDC_Callback(Buf, Len);
  //CDC_Transmit_FS(UserRxBufferFS, *Len); //ECHO

	// Prepare next receive
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS); //next buffer
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);


  return (USBD_OK);
  /* USER CODE END 6 */
}
/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(const char* Buf, uint16_t Len)
{

  /* USER CODE BEGIN 7 */
  if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
	  return USBD_FAIL;
  uint8_t result = USBD_OK;
  uint32_t size = sizeof(uint8_t) * Len;
//  uint32_t additional_length = 0;

//  if(hcdc->TxState != 0)
//	  additional_length = hcdc->TxLength;

  size = size < APP_TX_DATA_SIZE ? size : APP_TX_DATA_SIZE;

  memcpy(UserTxBufferFS, Buf, size);


  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, size);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}
