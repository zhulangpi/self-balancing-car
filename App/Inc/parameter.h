#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include "include.h"

#define   SECTOR_NUM  (FLASH_SECTOR_NUM-5)         //尽量用最后面的扇区，确保安全

#define   STEP  6


extern uint8 ParameterRead(void);
extern void ParameterSave(void);
extern void KeyDeal(void);
extern void MenuDisp(void);
extern float mean( float* phead,uint16 length );

typedef struct
{
    uint8 KeyStateIndex;    //当前状态索引
    uint8 KeyA;             //按下A转向
    uint8 KeyB;             //按下B转向
    uint8 KeyC;             //按下C转向
    void (*CurrentOperate)(void);    //当前执行
            
}KbdTab_t;


typedef struct
{
    float* param;
    float step[STEP];
}ParamEdit_t;


//----------------------程序调试用-------------------------//
extern uint32 g_timecount;
extern float g_speed_direct;

#define  TIMECOUNTSTART()   lptmr_time_start_us()
#define  GETTIME()          lptmr_time_get_us() 

#endif