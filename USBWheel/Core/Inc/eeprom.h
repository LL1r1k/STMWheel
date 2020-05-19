#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "main.h"

//################################################################################################################
bool      EE_Format(void);
bool      EE_Read(uint16_t VirtualAddress, uint32_t* Data);
bool      EE_Write(uint16_t VirtualAddress, uint32_t Data);
bool      EE_Reads(uint16_t StartVirtualAddress, uint16_t HowManyToRead, uint32_t* Data);
bool      EE_Writes(uint16_t StartVirtualAddress, uint16_t HowManyToWrite, uint32_t* Data);
uint16_t  EE_GetSize(void);
uint16_t	EE_GetMaximumVirtualAddress(void);
//################################################################################################################

#ifdef __cplusplus
}
#endif

#endif
