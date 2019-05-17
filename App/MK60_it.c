/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_it.c
 * @brief      山外K60 平台中断服务函数
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-26
 */

#include  "include.h"


uint8  TimerCnt1ms = 0;
volatile uint8  g_nSpeedControlCount=0;
float Speed_Transition=0;
uint16 SpeedIncCnt = 0;
volatile uint8  g_nDirControlCount = 0;
extern uint16 stopflag;
/***********************************

--@func pit0中断处理
--@调用间隔1ms

************************************/

void PIT0_Isr(void)
{
    //DisableInterrupts;
    //TIMECOUNTSTART();
    
    PIT_Flag_Clear(PIT0);       //清中断标志

    TimerCnt1ms = TimerCnt1ms%5 + 1;
    
    
    switch(TimerCnt1ms)
    {
        case 1:
            
            SampleFilter();
            AngleCalculate();
            AngleControl();
            //g_timecount = GETTIME();
            break;

        case 2:
            FtmSpeedGet();
            g_nSpeedControlCount++;
            if( 20 == g_nSpeedControlCount )    //  速度控制次数  SPEED_Control_PERIOD/5.0
			{
                
//---------------发车和起步-----------//   
                
                if(SpeedIncCnt<=40) SpeedIncCnt++;//  +1/100ms
                
                if(40 < SpeedIncCnt)
                    ;
                else if(SpeedIncCnt >= 30 )
                {    
                 
                    PID_Speed.Set = Speed_Transition * ((SpeedIncCnt-30)/10.0);
                }
                else //if(SpeedIncCnt <= 2000)
                    PID_Speed.Set = 0;
                
//--------------发车和起步end------------//  
                
                
//                if( SpeedIncCnt >= 20 && g_fSpeedbyCoder < 20 )
//                {
//                    PID_Angle.Set = 42;
//                }
//                else
//                {
//                    PID_Angle.Set = 51;                    
//                }
                
//---------------提速处理-----------------//

//                if(g_fSpeedbyCoder < 20)
//                {
//                    PID_Speed.Kp = 65;
//                    PID_Speed.Ki = 0.6;                    
//                    
//                }
//                else if(g_fSpeedbyCoder < 24)
//                {
//                    PID_Speed.Kp = 65;
//                    PID_Speed.Ki = 0.55;
//                }
//                else
//                {
//                    PID_Speed.Kp = 50;
//                    PID_Speed.Ki = 0.25;
//                    
//                }
                    PID_Speed.Kp = 50;
                    PID_Speed.Ki = 0.2;
                
                

//---------------提速处理end-------------//                

                SpeedControl();
				g_nSpeedControlCount=0;
				g_nSpeedControlPeriod=0;
            }
            //g_timecount = GETTIME();
            break;

        case 3:
            Sensor_Getfast_avg2();  
            Control_JudgeSite();
            
            //g_timecount = GETTIME();
            break;

        case 4:
            Control();
            

            //g_timecount = GETTIME();
            break;
            
        case 5:
        
            g_nDirControlCount++;          //DIR_Control_PERIOD
            if( 1 == g_nDirControlCount )
            {   

                DirectionControl();
                g_nDirControlCount = 0;
                g_nDirectionControlPeriod = 0;
                
            }
            //g_timecount = GETTIME();
            break;
        default:
            break;

    }

    
if( g_speed_direct >= 0 && g_speed_direct < 5 )
    SpeedControlOutput();
if( g_speed_direct >= 5 && g_speed_direct < 10 )
    DirectionControlOutput();
if( g_speed_direct >= 10 && g_speed_direct < 15 )
{

    SpeedControlOutput();
    DirectionControlOutput();

}    
    MotorOutput();
    //g_timecount = GETTIME();
    //EnableInterrupts;
}





/**************************************

 蓝牙串口中断处理

***************************************/
void UART4_IRQHandler(void)
{
    uint8 buf=0; 
    
//   if(UART_S1_REG(UARTN[UART4]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
   {
        buf = UART4_D;
        if( 0x48 == buf)
        {    
            disable_irq(PIT0_IRQn);
            SetMoterVoltage(0,0);
        }    
        else if(0xed == buf)
        {
            enable_irq(PIT0_IRQn);
        }
            //PID_Speed.Set = buf;
   }   

   UART4_S1 = UART4_S1;//读状态寄存器，清中断标志位
   
}





/***********************************

--@func pit1中断处理
--@调用间隔10ms

************************************/
void PIT1_Isr(void)
{
    PIT_Flag_Clear(PIT1);       //清中断标志

    key_IRQHandler();

}





/******************************* 
    
    NRF24L01P外部中断 

********************************/
void PORTE_IRQHandler()
{
    uint32 flag;

    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //清中断标志位

    if(flag & (1 << 27))                                 //PTE27触发中断
    {
        nrf_handler();
        nrf_tx_state();
    }
}

/*******************************

        干簧管

*******************************/
extern uint16 stopflag;
extern float probe0,probe1,probe2;
void PORTA_IRQHandler()
{
    uint32 flag;
    static uint8 i=0;
    i++;
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    if(flag & (1 << 26))
    {
        if( /*i>=2 && */stopflag >= 800 && g_fGyroscopeAngleSpeed_Y<20)//20ms*
        {
            disable_irq(PIT0_IRQn);
            SetMoterVoltage(0,0);
            led(LED_MAX,LED_ON);
            //probe0 = 10;
            
        }
    }


}

/*********************
    发车后关外设
**********************/


uint8 bisai =0 ;
void lptmr_hander()
{
    LPTMR_Flag_Clear();
//    gpio_init(PTC0,GPO,1);// R电机使能 
//    gpio_init(PTC5,GPO,1);// L电机使能 
    disable_irq(PIT1_IRQn);//10ms key
    disable_irq(PORTE_IRQn);//nrf
    bisai = 1;
    
}








