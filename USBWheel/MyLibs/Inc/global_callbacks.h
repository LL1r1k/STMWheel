/*
 * global_callbacks.h
 *
 *  Created on: 25 апр. 2020 г.
 *      Author: kks19
 */

#ifndef INC_GLOBAL_CALLBACKS_H_
#define INC_GLOBAL_CALLBACKS_H_

#include "main.h"

#pragma once
#ifdef __cplusplus

extern "C" {
#endif

void CDC_Callback(uint8_t* Buf, uint32_t *Len);
void USBD_OutEvent_HID(uint8_t* report);
void USB_SOF();
void USBD_GetEvent_HID(uint8_t id,uint16_t len,uint8_t** return_buf);


#ifdef __cplusplus
}
#endif

#endif /* INC_GLOBAL_CALLBACKS_H_ */
