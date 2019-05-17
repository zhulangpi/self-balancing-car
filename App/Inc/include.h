#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO口操作
#include  "MK60_uart.h"     //串口
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //低功耗定时器(延时)
#include  "MK60_pit.h"      //PIT
#include  "MK60_FLASH.h"    //FLASH
#include  "MK60_ftm.h"

#include  "VCAN_LED.H"          //LED
#include  "VCAN_KEY.H"          //KEY
#include  "MK60_adc.h"

//#define LQ
     
#ifndef LQ
#include  "MK60_i2c.h"
#endif    
     
#include  "OLED.h"
#include  "stand.h"
#include  "parameter.h"
#include  "VCAN_NRF24L0.h"
#include  "upper.h"
#include  "L3G4200D.h"
#include  "sensor.h"
#include  "8700_2100.h"
#include  "I2C.h"






#endif  //__INCLUDE_H__
