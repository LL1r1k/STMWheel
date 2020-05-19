#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern uint8_t cdc_dtr;

uint8_t CDC_Transmit_FS(const char* Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
