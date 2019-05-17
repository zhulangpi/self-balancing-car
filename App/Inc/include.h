#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO�ڲ���
#include  "MK60_uart.h"     //����
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //�͹��Ķ�ʱ��(��ʱ)
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
