/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CUSTOMHID_H
#define __USB_CUSTOMHID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"
#include "constants.h"

#define CUSTOM_HID_FS_BINTERVAL     0x01U


#define CUSTOM_HID_EPIN_ADDR                 0x83U //0x81U
#define CUSTOM_HID_EPIN_SIZE                 64 // was: 0x02U

#define CUSTOM_HID_EPOUT_ADDR                0x02U // 0x01U
#define CUSTOM_HID_EPOUT_SIZE                64

#define HID_INTERFACE 0x02 // Interface id (For composite)

#define USB_CUSTOM_HID_CONFIG_DESC_SIZ       41U
#define USB_CUSTOM_HID_DESC_SIZ              9U

#define CUSTOM_HID_DESCRIPTOR_TYPE           0x21U
#define CUSTOM_HID_REPORT_DESC               0x22U

#define CUSTOM_HID_REQ_SET_PROTOCOL          0x0BU
#define CUSTOM_HID_REQ_GET_PROTOCOL          0x03U

#define CUSTOM_HID_REQ_SET_IDLE              0x0AU
#define CUSTOM_HID_REQ_GET_IDLE              0x02U

#define CUSTOM_HID_REQ_SET_REPORT            0x09U
#define CUSTOM_HID_REQ_GET_REPORT            0x01U


#define USB_CUSTOM_HID_CONFIG_DESC_SIZE       41
#define USB_CUSTOM_HID_DESC_SIZE              9
#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE     512 //512

#define USBD_CUSTOM_HID_REPORT_DESC_SIZE     1267 //1267

typedef enum
{
  CUSTOM_HID_IDLE = 0U,
  CUSTOM_HID_BUSY,
}
CUSTOM_HID_StateTypeDef;

typedef struct _USBD_CUSTOM_HID_Itf
{
  uint8_t                  *pReport;
  int8_t (* Init)			(void);
  int8_t (* DeInit)			(void);
  int8_t (* OutEvent)      (uint8_t*); //modified
  int8_t (* GetEvent)      (USBD_SetupReqTypedef*,uint8_t** return_buf); //feature get

} USBD_CUSTOM_HID_ItfTypeDef;

typedef struct
{
  uint8_t              Report_buf[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
  uint32_t             Protocol;
  uint32_t             IdleState;
  uint32_t             AltSetting;
  uint32_t             IsReportAvailable;
  CUSTOM_HID_StateTypeDef     state;
}
USBD_CUSTOM_HID_HandleTypeDef;

extern USBD_ClassTypeDef  USBD_CUSTOM_HID;

#define USBD_CUSTOM_HID_CLASS    &USBD_CUSTOM_HID

uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev,
                                   uint8_t *report,
                                   uint16_t len);



uint8_t  USBD_CUSTOM_HID_RegisterInterface(USBD_HandleTypeDef   *pdev,
                                           USBD_CUSTOM_HID_ItfTypeDef *fops);

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CUSTOMHID_H */
