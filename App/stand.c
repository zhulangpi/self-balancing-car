
#include  "include.h"


//--------------------选择--------------------//
#define AngleFusion2
//#define DOUBLELOOP


/*****************************以下参数不需要整定*************************/

//测量得到的数据
float  g_fGyroOut_X=0;   //陀螺仪X轴数据
float  g_fGyroOut_Y=0;
float  g_fGravityOut_Z=0;  //加速度计数据



//计算得到的数据
float  g_fGravityAngle=0;   //加速度计测量的角度
float  g_fAnglebyFilter;    //角度融合得到的角度
float  g_fGyroscopeAngleSpeed_X=0;//陀螺仪X轴角速度

//------------------角度控制--------------------//
float  g_fCarAngle = 0;
float  g_fAngleControlOut=0;
float  g_fGyroscopeAngleIntegral=0;

float  g_fGyroscopeDirectionIntegral=0;

//------------------速度控制-------------------//
float g_fSpeedbyCoder=0;//车速
float g_fSpeedbyCoderL=0;//车速
float g_fSpeedbyCoderR=0;//车速


float g_fApha = 0.5;  //积分分离PID阈值 


int32 nSpeedPulseSigmaL = 0;//100ms内编码器采集的脉冲数
int32 nSpeedPulseSigmaR = 0;

float g_fSpeedControlOutL=0;//电机速度控制输出值
float g_fSpeedControlOutNewL=0;//电机速度控制输出值
float g_fSpeedControlOutOldL=0;
float g_fSpeedControlOutR=0;//电机速度控制输出值
float g_fSpeedControlOutNewR=0;//电机速度控制输出值
float g_fSpeedControlOutOldR=0;
uint16 g_nSpeedControlPeriod=0;

//-----------------方向控制----------------------//

uint16  g_nDirectionControlPeriod =0;

float  g_fDirectionControlOutOld = 0;
float  g_fDirectionControlOutNew = 0;
float  g_fDirectionControlOut=0;
float  g_fGyroscopeAngleSpeed_Y=0;//陀螺仪Y轴角速度


//-------------------电机变量----------------//
float	fLeftVal, fRightVal;
float	fLeftMotorOut=0;
float	fRightMotorOut=0;



float probe0 =0 ,probe1=0,probe2 = 0;



/*****************************以下参数需要整定*************************/
#ifdef LQ
    #define  GRAVITY_OFFSET_Z       (-40.0f)        //加速度计Z轴偏置
    #define  GYROSCOPE_OFFSET_X     (60.5f)        //陀螺仪X轴偏置
    #define  GYROSCOPE_ANGLE_RATIO_X  (0.028f)//(0.088f)  //陀螺仪比例因子
#else
    //#define  GRAVITY_ANGLE_RATIO  (0.0225f)   //加速度计比例因子  -3940~4060     1800/(delta)
    #define  GRAVITY_OFFSET_Z       (-40.0f)       //加速度计Z轴偏置
    #define  GYROSCOPE_OFFSET_X     (5.8f)        //陀螺仪X轴偏置
    #define  GYROSCOPE_ANGLE_RATIO_X  (0.114f)//(0.088f)  //陀螺仪比例因子  
#endif

#define  GYROSCOPE_ANGLE_SIGMA_FREQUENCY  (0.005f)  //5ms

#define  CAR_ANGLESPEED_SET (0)


//---------------各种限幅---------------//

#define  ANGLE_OUT_MAX      ( 50000)
#define  ANGLE_OUT_MIN      (-50000)
#define  SPEED_OUT_MAXL     ( 50000)
#define  SPEED_OUT_MINL     (-50000)
#define  SPEED_OUT_MAXR     ( 50000)
#define  SPEED_OUT_MINR     (-50000)
#define  DIR_OUT_MAX        ( 50000)//5000
#define  DIR_OUT_MIN        (-50000)//-5000

//#define  ANGLE_OUT_MAX      ( 500000)
//#define  ANGLE_OUT_MIN      (-500000)
//#define  SPEED_OUT_MAXL     ( 500000)
//#define  SPEED_OUT_MINL     (-500000)
//#define  SPEED_OUT_MAXR     ( 500000)
//#define  SPEED_OUT_MINR     (-500000)
//#define  DIR_OUT_MAX        ( 500000)//5000
//#define  DIR_OUT_MIN        (-500000)//-5000


#define  MOTOR_OUT_MAX      ( 1000)
#define  MOTOR_OUT_MIN      ( -1000)


////初始化给个值，不然会出问题,在中断中修改，不懂看程序
PID_t PID_Angle = { 51,    70,     0,      1.5 };//25
PID_t PID_Speed = { 27,    65,     0.6,   0   };
PID_t PID_Dir =   { 0,      0,     0,      0 };

/*
PID_t PID_Angle = { 51,    70,     0,      1.5 };//25   顺/右转/积分漂移小  相反23.5  
PID_t PID_Speed = { 27,    65,     0.6,   0   };
PID_t PID_Dir =   { 0,      0,     0,      0 };



*/


/*************************************************
*@ func 采样陀螺仪加速度计数据并滤波

**************************************************/


void SampleFilter(void)
{

    float fGravityOut_X=0,fGravityOut_Y=0;
#ifdef LQ
//-------------8700-------------//

    fGravityOut_X = read_lq(SlaveAddress8700,'X');//X
    
    fGravityOut_Y = read_lq(SlaveAddress8700,'Z');//Z

//--------------2100------------//    
    g_fGyroOut_X = read_lq(SlaveAddress2100,'Y');//Y
     
    g_fGyroOut_Y = - read_lq(SlaveAddress2100,'Z');//Z
    
#else    
    
    fGravityOut_X = MMA8451_read_reg(1,'Y');      //Z垂直于表面   Y  车轴面-90°
    
    fGravityOut_Y = MMA8451_read_reg(1,'Z');
    
    g_fGyroOut_X = - L3G_read_reg(1,'X');           //X车轴面
    
    g_fGyroOut_Y = - L3G_read_reg(1,'Z');
    
#endif
    
    g_fGravityOut_Z = 100*atan2(fGravityOut_X,fGravityOut_Y);
    
    //EASYFILTER(g_fGravityOut_Z);
    //EASYFILTER(g_fGyroOut_X);

    //g_fGravityOut_Z = 0.001*low_pass_filter(g_fGravityOut_Z);




}




//***********************角度计算*********************//
//////////需调参数：  --第十届王明龙注释
//        R  --------------------互补滤波融合角度的比例因子
//        GRAVITY_ANGLE_RATIO  -----------加速度计的比例因子
//        GYROSCOPE_ANGLE_RATIO_X---------陀螺仪X轴的比例因子 
//        GYROSCOPE_ANGLE_RATIO_Y---------陀螺仪Y轴的比例因子 

////////固定参数：需要测定的参数
//      GYROSCOPE_OFFSET------陀螺仪的零偏值，，，，，ＸＹ轴零偏值是否相同？？？？？？？？？？？？？？？
//      GRAVITY_OFFSET--------加速度计的零偏值

void AngleCalculate(void)
{
    
//--------------角度-------------//
	
#ifdef LQ
    g_fGravityAngle = g_fGravityOut_Z - GRAVITY_OFFSET_Z;
#else
    g_fGravityAngle =  g_fGravityOut_Z - GRAVITY_OFFSET_Z ; //* GRAVITY_ANGLE_RATIO;
    
#endif
//--------------角速度-----------//
    g_fGyroscopeAngleSpeed_X  = (g_fGyroOut_X - GYROSCOPE_OFFSET_X) * GYROSCOPE_ANGLE_RATIO_X;//* 1.2 * ;// * GYROSCOPE_ANGLE_RATIO_X;  //陀螺仪测量的角速度
//  控制时间 period =0.005    5ms
    
    g_fGyroscopeAngleSpeed_Y = ( g_fGyroOut_Y - 0 ) * GYROSCOPE_ANGLE_RATIO_X;
    
    
//if(g_fSpeedbyCoder > 13)
//{
//    if(g_fGyroscopeAngleSpeed_X>100)
//        g_fGyroscopeAngleSpeed_X = 100;
////    if(g_fGyroscopeAngleSpeed_X<-100)
//  //      g_fGyroscopeAngleSpeed_X = -100;
//}
    
    //EASYFILTER(g_fGyroscopeAngleSpeed_Y);
    
    //probe2 = g_fGyroscopeAngleSpeed_Y;
    
    if( g_fGyroscopeAngleSpeed_Y<0 )//左转 逆时针 系数越大往上补偿 左转加速度为负值
        g_fGyroscopeAngleSpeed_X -= 0.01*g_fGyroscopeAngleSpeed_Y;//0.005
    else
        g_fGyroscopeAngleSpeed_X += 0.005*g_fGyroscopeAngleSpeed_Y;//0.0005
    
//-----------陀螺仪积分---------//
#ifndef AngleFusion4    
    //0.1
    g_fGyroscopeAngleIntegral += (g_fGyroscopeAngleSpeed_X + 0*fabs(g_fGyroscopeAngleSpeed_Y) )* GYROSCOPE_ANGLE_SIGMA_FREQUENCY;

#endif


#ifdef AngleFusion1    
    First_Order_Complementary_Filter(g_fGravityAngle,g_fGyroscopeAngleSpeed_X);
#elif defined AngleFusion2    
    Second_Order_Complementary_Filter(g_fGravityAngle,g_fGyroscopeAngleSpeed_X);
#elif defined AngleFusion3
    Kalman_Filter(g_fGravityAngle,g_fGyroscopeAngleSpeed_X);
#elif defined AngleFusion4
    Tsinghua_Filter();
#endif
    
    
    
    g_fCarAngle = g_fAnglebyFilter;
    
}


 ////////////////////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////直立控制函数////////////////////////////////////////////////////
 ///     功能：将'角度计算函数'获得的车模角度信号g_fCarAngle和g_fGyroscopeAngleSpeed_Y分别进行P、D控制，
 ///           产生车模直立控制信号g_fAngleControlOut
 ///     需调参数：ANGLE_Control_P--角度的P参数     ANGLE_Control_D--角加速度的D参数 
 ///     固定参数： CAR_ANGLE_SET---               CAR_ANGLE_SPEED_SET---
 ///                ANGLE_Control_OUT_MAX-- 上限       ANGLE_Control_OUT_MIN--下限
 ///     调用周期：
 ///----往届注释
 ////////////////////////////////////////////////////////////////////////////////////////////////////

void AngleControl(void)
{
  	float   fAngleDVal=0,fAnglePVal=0;    

    fAnglePVal = PID_Angle.Set - g_fCarAngle;//角度偏差

	fAngleDVal = CAR_ANGLESPEED_SET - g_fGyroscopeAngleSpeed_X;

	g_fAngleControlOut = PID_Angle.Kp * fAnglePVal + PID_Angle.Kd * fAngleDVal;

    //限幅处理
	if(g_fAngleControlOut>ANGLE_OUT_MAX)
		g_fAngleControlOut = ANGLE_OUT_MAX;

	else if(g_fAngleControlOut < ANGLE_OUT_MIN)
			g_fAngleControlOut = ANGLE_OUT_MIN;
    
    //probe2 = g_fAngleControlOut;
    
}

/*****************************************
    速度控制函数
    位置式增量式？
    双环单环？
    限幅 , 积分分离式PID

******************************************/

void SpeedControl(void)
{
    static float fSpeed_err_prevL =0;//error(k-2)
	static float fSpeed_err_lastL =0;//error(k-1)
	static float fSpeed_err_thisL =0;//error(k)
#ifdef   DOUBLELOOP  
    static float fSpeed_err_prevR =0;//error(k-2)
	static float fSpeed_err_lastR =0;//error(k-1)
	static float fSpeed_err_thisR =0;//error(k)
#endif
    uint8 apha = 1;
    //static float fSpeed_err_sum = 0;
    
	g_fSpeedbyCoder  = ( nSpeedPulseSigmaL + nSpeedPulseSigmaR ) *0.01f;//左右电机脉冲数平均，

	g_fSpeedbyCoderL = nSpeedPulseSigmaL * 0.02f;
	g_fSpeedbyCoderR = nSpeedPulseSigmaR * 0.02f;
	nSpeedPulseSigmaL=0;
	nSpeedPulseSigmaR=0;

    EASYFILTER(g_fSpeedbyCoder);


#ifdef DOUBLELOOP
    
	fSpeed_err_thisL = PID_Speed.Set - g_fSpeedbyCoderL;//左电机速度差
	fSpeed_err_thisR = PID_Speed.Set - g_fSpeedbyCoderR;//左电机速度差

//    if( g_fApha <= fSpeed_err_thisR && g_fSpeedbyCoder>18)
//        apha = 1;
//    else
//        apha = 0;
#else

    fSpeed_err_thisL = PID_Speed.Set - g_fSpeedbyCoder;//电机速度差

#endif

//    if( 1 <= fSpeed_err_thisL )
//        apha = 1;
//    else
//        apha = 0;
    probe0 = apha;

    
    g_fSpeedControlOutOldL = g_fSpeedControlOutNewL;
	

	g_fSpeedControlOutNewL += PID_Speed.Kp * (fSpeed_err_thisL- fSpeed_err_lastL) 
	  						+ PID_Speed.Ki * fSpeed_err_thisL; 
							//- PID_Speed.Kd * (fSpeed_err_thisL - 2*fSpeed_err_lastL + fSpeed_err_prevL);

    fSpeed_err_prevL = fSpeed_err_lastL;
    
    fSpeed_err_lastL = fSpeed_err_thisL;
    
    if( g_fSpeedControlOutNewL > SPEED_OUT_MAXL)
		g_fSpeedControlOutNewL = SPEED_OUT_MAXL;
	else if( g_fSpeedControlOutNewL < SPEED_OUT_MINL)
        g_fSpeedControlOutNewL = SPEED_OUT_MINL;
    
    
#ifdef    DOUBLELOOP
    g_fSpeedControlOutOldR = g_fSpeedControlOutNewR;
    
	g_fSpeedControlOutNewR += PID_Speed.Kp * (fSpeed_err_thisR- fSpeed_err_lastR) 
	  						+ PID_Speed.Ki * fSpeed_err_thisR 
							+ PID_Speed.Kd * (fSpeed_err_thisR - 2*fSpeed_err_lastR + fSpeed_err_prevR);
    
	fSpeed_err_prevR = fSpeed_err_lastR;
    
	fSpeed_err_lastR = fSpeed_err_thisR;

	if( g_fSpeedControlOutNewR > SPEED_OUT_MAXR)
		g_fSpeedControlOutNewR = SPEED_OUT_MAXR;
	else if( g_fSpeedControlOutNewR < SPEED_OUT_MINR)
            g_fSpeedControlOutNewR = SPEED_OUT_MINR;
#endif
    
    
}

/****************************************************

-速度控制输出

-该函数将速度输出变化量平均到100个控制周期内平滑输出

*****************************************************/
void SpeedControlOutput(void)
{
    float fSpeedValue=0;

    fSpeedValue = g_fSpeedControlOutNewL - g_fSpeedControlOutOldL ;

	g_fSpeedControlOutL = fSpeedValue *(g_nSpeedControlPeriod+1)/SPEED_Control_PERIOD + g_fSpeedControlOutOldL; 

#ifdef DOUBLELOOP

    fSpeedValue = g_fSpeedControlOutNewR - g_fSpeedControlOutOldR ;

	g_fSpeedControlOutR = fSpeedValue *(g_nSpeedControlPeriod+1)/SPEED_Control_PERIOD + g_fSpeedControlOutOldR;

#endif
	g_nSpeedControlPeriod++;
    
    
    //probe1 = g_fSpeedControlOutL;
}

extern float g_fecfbyKalman;
//方向控制
void DirectionControl(void)
{
    float fDirDVal = 0;
    
    fDirDVal = g_fGyroscopeAngleSpeed_Y;
    
//    fDirDVal = 0.9*g_fecfbyKalman+0.5*g_fGyroscopeAngleSpeed_Y;
//    fDirDVal *= 1.5;
    
    g_fDirectionControlOutOld = g_fDirectionControlOutNew;
    
    //PID_Dir.Kp = 2.0 * fabs(g_fDirPVal) + 50;
    if( g_fDirPVal <= 0 )//右转
    {
        if( fabs(g_fDirPVal) < 1)
        {
            PID_Dir.Kp = 80;
            PID_Dir.Kd = 1.4; 
        }
        else if( fabs(g_fDirPVal) < 3 )
        { 
            PID_Dir.Kp = 110;//100//90
            PID_Dir.Kd = 2.0;//2.0//1.6
            //g_fDirectionControlOutNew = g_fDirPVal * PID_Dir.Kp +  fDirDVal * PID_Dir.Kd;
        }
        else if( fabs(g_fDirPVal) < 6 )
        {
            PID_Dir.Kp = 140;//140//130
            PID_Dir.Kd = 2.5;//2.5//2.1
        }
        else
        {
            PID_Dir.Kp = 160;//160//150
            PID_Dir.Kd = 2.8;//2.8//2.4
        }
    }
    else
    {
        if( fabs(g_fDirPVal) < 1)
        {
            PID_Dir.Kp = 80;
            PID_Dir.Kd = 1.4; 
        }
        else if( fabs(g_fDirPVal) < 3 )
        { 
            PID_Dir.Kp = 160;//100//90
            PID_Dir.Kd = 2.5;//2.0//1.6

        }
        else if( fabs(g_fDirPVal) < 6 )
        {
            PID_Dir.Kp = 170;//140//130
            PID_Dir.Kd = 2.8;//2.5//2.1
        }
        else
        {
            PID_Dir.Kp = 180;//160//150
            PID_Dir.Kd = 2.9;//2.8//2.4
        }
        
        
    }
    
        
    
    
    
    
    
//    if( g_fDirPVal > 0 )
//    
//        g_fDirectionControlOutNew = g_fDirPVal * PID_Dir.Kp +  fDirDVal * PID_Dir.Kd;
//    
//    else
    
        g_fDirectionControlOutNew = g_fDirPVal * PID_Dir.Kp +  fDirDVal * PID_Dir.Kd;
    
    // 限幅
    if(	g_fDirectionControlOutNew < DIR_OUT_MIN)
		g_fDirectionControlOutNew = DIR_OUT_MIN;

	if(	g_fDirectionControlOutNew > DIR_OUT_MAX)
		g_fDirectionControlOutNew = DIR_OUT_MAX;
    
}


void DirectionControlOutput(void)
{
    float fDirVal = 0;

	fDirVal = g_fDirectionControlOutNew - g_fDirectionControlOutOld;
	
	g_fDirectionControlOut = fDirVal * (g_nDirectionControlPeriod+1) / DIR_Control_PERIOD + g_fDirectionControlOutOld; 
	
	g_nDirectionControlPeriod++;
    
    //probe0 = g_fDirectionControlOut;
    
}



//将三路ＰＩＤ合成作用到电机上
void MotorOutput(void)
{

    fLeftMotorOut = g_fAngleControlOut - g_fSpeedControlOutL - g_fDirectionControlOut;

#ifdef DOUBLELOOP

    fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOutR + g_fDirectionControlOut;

#else

    fRightMotorOut = g_fAngleControlOut- g_fSpeedControlOutL + g_fDirectionControlOut;

#endif

    MotorPWMOut();
}

// 对左右两个电机的输出量增加死区常量和输出饱和处理-------往届注释
// 增加死区常量是为了提高车模在静态下的稳定性，输出饱和处理是为了保证输出量不会超过PWM的满量程范围

extern float ADCMax ;
extern uint8 SpeedIncCnt;
void MotorPWMOut(void)
{
    fLeftVal  = fLeftMotorOut;
    fRightVal = fRightMotorOut;

    
//    if(fLeftVal > 0)                      //增加死区电压
//        fLeftVal +=(0);//85//90           //关于这个死区需要根据车模测试，王明龙注释,已经测试过了2015-4-20-频率=20KHz
//    else if(fLeftVal <0)
//        fLeftVal -=(0);//85//90        //注意 PWM占空比的分母改变时需要调整此项值，王明龙注释
//
//    if(fRightVal > 0)
//        fRightVal +=(0);//85//95
//    else if(fRightVal <0)
//        fRightVal -=(0);//85//95
    
    
    
    
    
    
    
    if(fLeftVal > MOTOR_OUT_MAX)        //限幅，输出饱和处理
        fLeftVal = MOTOR_OUT_MAX;       //注意 PWM占空比的分母改变时需要调整此项值，王明龙注释
    else if(fLeftVal < MOTOR_OUT_MIN)
        fLeftVal = MOTOR_OUT_MIN;

    if(fRightVal > MOTOR_OUT_MAX)
        fRightVal = MOTOR_OUT_MAX;
    else if(fRightVal < MOTOR_OUT_MIN)
        fRightVal = MOTOR_OUT_MIN;

	if( /*g_fSpeedbyCoder > (PID_Speed.Set + 20) || */(SpeedIncCnt>20 && ADCMax < 20 ) )//|| g_fCarAngle > 80 || g_fCarAngle < -15 ) //这里的角度值需要根据车模调整，王明龙注释
	{
		fLeftVal=0;
		fRightVal=0;
		disable_irq(PIT0_IRQn);
	}

//    probe0 =fLeftVal;
//    probe1 =fRightVal;
    
	SetMoterVoltage( (int16) fLeftVal, (int16) fRightVal);
	
}

//设置电机占空比
void SetMoterVoltage(int16 a,int16 b)
{
    if(b>=0)
    {
        ftm_pwm_duty(FTM0, FTM_CH0,0);     //设置占空比 为 duty/1000
        ftm_pwm_duty(FTM0, FTM_CH1,b);//	在这个函数里面设置里最大占空比为30%
    }
    else
    {
        ftm_pwm_duty(FTM0, FTM_CH1,0);
        systick_delay_us(100);
        ftm_pwm_duty(FTM0, FTM_CH0,-b);     //设置占空比 为 duty/1000
        ftm_pwm_duty(FTM0, FTM_CH1,0);
    }
    if(a>=0)
    {
        ftm_pwm_duty(FTM0, FTM_CH2,0);
        ftm_pwm_duty(FTM0, FTM_CH3,a);
    }
    else
    {
        ftm_pwm_duty(FTM0, FTM_CH3,0);
        systick_delay_us(100);
        ftm_pwm_duty(FTM0, FTM_CH2,-a);
        ftm_pwm_duty(FTM0, FTM_CH3,0);
    }
}

////////////////////////////////////////////////
///     FTM正交解码测速
///     Motor_Speed_L  左电机脉冲数
///     Motor_Speed_R 右电机脉冲数
///     移植龙邱K60编码器程序------王明龙
///////////////////////////////////////////////

void FtmSpeedGet(void)
{

  	int16	FtmSpeedL=0;//每次一中断速度case中从编码器读取左右电机脉冲测量值
	int16	FtmSpeedR=0;

    FtmSpeedL  =  FTM2_CNT;
    FTM2_CNT = 0;
    FtmSpeedR  =  FTM1_CNT;
    FTM1_CNT = 0;
	FtmSpeedL  = -FtmSpeedL;
    
    nSpeedPulseSigmaL += FtmSpeedL;//每一次PIT中断的速度case都读取一次编码器的脉冲数，
    nSpeedPulseSigmaR += FtmSpeedR;//在这里累加到100ms的总数

}











/*************************************

        角度融合

*************************************/

#ifdef AngleFusion1
#define  R1  (0.01f)        //一阶互补  加速度计
//------------一阶互补----------------//  
static void First_Order_Complementary_Filter( float g_fGravityAngle, float g_fGyroscopeAngleSpeed_X)
{
    g_fAnglebyFilter =  (1-R1)*( g_fAnglebyFilter + g_fGyroscopeAngleSpeed_X * GYROSCOPE_ANGLE_SIGMA_FREQUENCY )+ R1 * g_fGravityAngle ;    
}
#elif defined AngleFusion2
#define  R2  (0.03f)        //二阶互补  加速度计
//-------------二阶互补滤波-----------//
static void Second_Order_Complementary_Filter( float angle_m, float gyro_m)//采集后计算的角度和角加速度
{
    static float x1=0,x2=0,y1 =0;
 //GYROSCOPE_ANGLE_SIGMA_FREQUENCY 注意：dt的取值为滤波器采样时间
  	x1 = ( angle_m - g_fAnglebyFilter ) * R2 * R2;
    y1 += x1 * GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
    x2 = y1 + 2 * R2 * ( angle_m - g_fAnglebyFilter ) + gyro_m;
    g_fAnglebyFilter += x2*GYROSCOPE_ANGLE_SIGMA_FREQUENCY;

}
#elif defined AngleFusion3
//----------------卡尔曼----------------//
/************************
卡尔曼滤波变量////http://www.geek-workshop.com/thread-10172-1-1.html有参数说明
*******************************/

static void Kalman_Filter( float angle_m, float gyro_m) //gyro_m:gyro_measure 
{
	static float C_0 = 1;//1
	static float Q_angle = 0.001f;//0.001  //角度数据置信度,
	static float Q_gyro  = 0.01f; //0.01 //角速度数据置信度
	static float R_angle = 10.0f;       //10
	static float dt=0.005f;//dt的取值为kalman滤波器采样时间
	static float P[2][2] = {{ 1, 0 }, { 0, 1 } };
	static float Pdot[4] ={0,0,0,0};
	static float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;


	g_fAnglebyFilter+=(gyro_m-q_bias) * dt;//先验估计  
    angle_err = angle_m - g_fAnglebyFilter;//zk-先验估计    
    Pdot[0]=Q_angle - P[0][1] - P[1][0];// Pk-' 先验估计误差协方差的微分  
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;
    P[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差  
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;//后验估计误差协方差  
    P[0][1] -= K_0 * t_1;  
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    g_fAnglebyFilter += K_0 * angle_err;//后验估计  
    q_bias += K_1 * angle_err;//后验估计   
    g_fGyroscopeAngleSpeed_X = gyro_m-q_bias;  

}

#elif defined AngleFusion4
//----------------------清华滤波-----------------//
#define GRAVITY_ADJUST_TIME_CONSTANT (0.005)
static void Tsinghua_Filter(void)
{

//清华滤波    
//    float fDeltaValue;
    
//    g_fAnglebyFilter = g_fGyroscopeAngleIntegral;
//    fDeltaValue = (g_fGravityAngle - g_fAnglebyFilter) / GRAVITY_ADJUST_TIME_CONSTANT;
//    g_fGyroscopeAngleIntegral += (g_fGyroscopeAngleSpeed_X + fDeltaValue) * GYROSCOPE_ANGLE_SIGMA_FREQUENCY;


//大连海事滤波    
//    float fAngleErr;
//
//    g_fGyroscopeAngleIntegral += g_fGyroscopeAngleSpeed_X * 0.005;
//    fAngleErr = g_fGravityAngle - g_fGyroscopeAngleIntegral;
//    g_fGyroscopeAngleIntegral += fAngleErr * 0.005;
//    g_fAnglebyFilter = g_fGyroscopeAngleIntegral;
    
    
    
}

#endif

