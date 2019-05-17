#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include "include.h"

#define   SECTOR_NUM  (FLASH_SECTOR_NUM-5)         //������������������ȷ����ȫ

#define   STEP  6


extern uint8 ParameterRead(void);
extern void ParameterSave(void);
extern void KeyDeal(void);
extern void MenuDisp(void);
extern float mean( float* phead,uint16 length );

typedef struct
{
    uint8 KeyStateIndex;    //��ǰ״̬����
    uint8 KeyA;             //����Aת��
    uint8 KeyB;             //����Bת��
    uint8 KeyC;             //����Cת��
    void (*CurrentOperate)(void);    //��ǰִ��
            
}KbdTab_t;


typedef struct
{
    float* param;
    float step[STEP];
}ParamEdit_t;


//----------------------���������-------------------------//
extern uint32 g_timecount;
extern float g_speed_direct;

#define  TIMECOUNTSTART()   lptmr_time_start_us()
#define  GETTIME()          lptmr_time_get_us() 

#endif