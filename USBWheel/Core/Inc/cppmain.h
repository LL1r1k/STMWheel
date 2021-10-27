#ifndef CPPMAIN_H_
#define CPPMAIN_H_

#pragma once
#ifdef __cplusplus

	#include <vector>
	#include "stdlib.h"
	#include "stdio.h"

extern "C" {
#endif

#include "main.h"
/*
#include "main.h"
#include "usb_device.h"
#include "eeprom_addresses.h"*/

void cppmain();


#ifdef __cplusplus
}

template<class T,class C>
int8_t cliptest(T v, C l, C h)
{
	if(v > h){
		return 1;
	}else if(v < l){
		return -1;
	}else{
		return 0;
	}
}

template<class T,class C>
T clip(T v, C l, C h)
{
  return { v > h ? h : v < l ? l : v };
}


#endif

#endif /* CPPMAIN_H_ */
