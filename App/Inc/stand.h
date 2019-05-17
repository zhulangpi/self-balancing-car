#ifndef  _STAND_H_
#define  _STAND_H_

#include "include.h"


typedef struct
{
    float Set;
    float Kp ;
    float Ki ;
    float Kd ;

}PID_t;

extern PID_t PID_Angle;
extern PID_t PID_Speed;
extern PID_t PID_Dir;


#define Kalman_Q 0.4f
#define Kalman_R 1.5f

#define EASYFILTER(n)\
static float n##bykalman=0;\
static float p##n=10,kg##n=0.5;\
p##n = p##n + Kalman_Q;\
kg##n = p##n / (p##n + Kalman_R);\
n##bykalman = n##bykalman + kg##n * (n - n##bykalman);\
p##n = (1 - kg##n) * p##n;\
n = n##bykalman;


//#undef  Kalman_Q
//#undef  Kalman_R



#define SPEED_Control_PERIOD   (100) //?毫秒调用一次    //1/100
#define DIR_Control_PERIOD   (5) //?毫秒调用一次    //1/10


extern float  g_fGravityAngle;    //加速度计测量的角度
extern float  g_fGyroscopeAngleSpeed_X;


extern float  g_fCarAngle;              
extern float  g_fAnglebyFilter;        //经过滤波融合得到的角度    

extern float  g_fGyroscopeAngleSpeed_Y;//陀螺仪Y轴角速度

extern float	g_fSpeedbyCoder;//左右电机脉冲测量值
extern float	g_fSpeedbyCoderL;
extern float	g_fSpeedbyCoderR;
extern uint16	g_nSpeedControlPeriod;



extern uint16	g_nDirectionControlPeriod;


//-------------------调试用----------------//

extern float  g_fGyroscopeAngleIntegral;
extern float  g_fGravityOut_Z; 
extern float  g_fSpeedControlOut;
extern float  g_fGyroOut_X;
extern float  g_fGyroOut_Y;
extern float  g_fAngleControlOut;
extern float  g_fSpeedControlOutL;
extern float  g_fDirectionControlOut;



/***********************************************

            内部函数声明

************************************************/

static void   Kalman_Filter( float angle_m, float gyro_m);
static void   Second_Order_Complementary_Filter( float angle_m, float gyro_m);
static void   First_Order_Complementary_Filter( float angle_m, float gyro_m);
static void   Tsinghua_Filter();




/****************角度控制*************************/

extern void   SampleFilter(void);
extern void   AngleCalculate(void);//角度合成
extern void   AngleControl(void);


/*************************速度控制************************/

extern void   FtmSpeedGet(void);
extern void   SpeedControl(void);
extern void   SpeedControlOutput(void);

extern float  g_fApha;


/*************************方向控制************************/
extern void    DirectionControl(void);
extern void    DirectionControlOutput(void);

/*************************电机控制********************************/

extern void	MotorOutput(void);
extern void	MotorPWMOut(void);
extern void	SetMoterVoltage(int16 a,int16 b);
extern void	StartStand(void);


#endif