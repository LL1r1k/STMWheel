/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CUSTOM_HID_IF_H__
#define __USBD_CUSTOM_HID_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_customhid.h"

/** CUSTOMHID Interface callback. */
extern USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS;

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CUSTOM_HID_IF_H__ */
