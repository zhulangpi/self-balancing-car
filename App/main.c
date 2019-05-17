#include "include.h"


void ALL_Init(void);
void SetPriority(void);
void TestOffset();
void Integral_Init_Val();

uint8 SchemeSelecte(void);

extern float Speed_Transition;  //�ٶȹ�����


/*!
 *  @brief      main����
 *  @since      v5.0
 *  @note      160722����

 */


uint16 stopflag=0;
extern uint8 bisai;
extern float probe0,probe1,probe2;
void main()
{

    DisableInterrupts;          //�����ж�
    
    ALL_Init();
    
    
    //SetMoterVoltage( 500, 500);//120,105

    systick_delay_ms(50);//wait for angle sensor
    
    Integral_Init_Val();
    
    //lptmr_timing_ms(1000);                                // LPTMR ��ʱ 
    //enable_irq(LPTMR_IRQn); 
    
    
    EnableInterrupts;         //�����ж�
    
    for(;;)
    {
        if(!bisai)
        {
        KeyDeal();
//ParameterSave();
        //msg_visualscope(g_fGyroscopeAngleIntegral*10,g_fCarAngle*10,g_fGravityAngle*10,g_fSpeedbyCoder);
        //msg_visualscope(probe0,probe1,g_fGravityOut_Z,g_fGyroOut_X);
        //MenuDisp();
        //TestOffset();
        //msg_datascope(g_fGravityAngle,g_fGyroscopeAngleSpeed_X,g_fGyroscopeAngleIntegral,g_fCarAngle,g_fGravityOut_Z,g_fGyroOut_X,g_fGyroOut_Y);
        //msg_datascope(g_fSpeedbyCoder,PID_Speed.Set,probe0/20,probe1/20,g_fAngleControlOut/20,g_fCarAngle,g_fDirectionControlOut/20 );
        //msg_datascope(g_fSpeedbyCoder,g_fCarAngle,g_fGravityAngle,g_fGyroscopeAngleIntegral,g_fGyroscopeAngleSpeed_X,probe0,g_fSite );
        //msg_datascope( SensorData_f0,SensorData_f1,SensorData_f2,SensorData_s0,SensorData_s1,g_fSpeedbyCoder,g_fSite);
        //msg_datascope( (SensorData_f0-SensorData_f2)/(SensorData_f0+SensorData_f2),(SensorData_f0-SensorData_f1)/(SensorData_f1+SensorData_f0),(SensorData_f1-SensorData_f2)/(SensorData_f1+SensorData_f2),SensorData_f0,SensorData_f1,SensorData_f2,probe0);
        msg_datascope(g_fSite,g_fSpeedbyCoder,g_fGyroscopeAngleIntegral,g_fGyroscopeAngleSpeed_Y,0,0,0);
        //msg_datascope(g_fDirectionControlOut,g_fSpeedControlOutL,g_fAngleControlOut,g_fSpeedbyCoder,g_fCarAngle,g_fSite,PID_Speed.Set);
        //msg_datascope(g_fSpeedbyCoderL,g_fSpeedbyCoderR,g_fSpeedbyCoderL-g_fSpeedbyCoderR,g_fSpeedbyCoder,PID_Speed.Kp,PID_Speed.Ki,probe0);
        //upper1();
        }
        systick_delay_ms(20);
        //msg_deal();
        //Dis_Float(0,0,g_fSpeedbyCoder,2);
        stopflag++;
        
    }
}

void ALL_Init(void)
{
    
//------------�ж����ȼ�����------------//
    SetPriority();
//------------���������ʼ��------------//
    led_init(LED_MAX);
    key_init(KEY_MAX);
    LCD_Init();
    MenuDisp();
//------------װ��flash�ڲ���-----------//
    flash_init();
    ParameterRead();
    //PID_Speed.Set = 31;
    Speed_Transition = PID_Speed.Set;
//------------��������ʼ��--------------//
#ifdef LQ
    I2C_Init(I2C0);
    Init8700();//���ٶȼ� 
    Init2100();//������
#else
    MMA8451_Init();
    L3G4200D_init();
#endif    
    Sensor_Init();          //���
//------------�����ʼ��----------------//

#define DIANJI 12*1000
    ftm_pwm_init(FTM0, FTM_CH0,DIANJI,0);
    ftm_pwm_init(FTM0, FTM_CH1,DIANJI,0);
    ftm_pwm_init(FTM0, FTM_CH2,DIANJI,0);
    ftm_pwm_init(FTM0, FTM_CH3,DIANJI,0);
#undef DIANJI
    
    gpio_init(PTC0,GPO,1);// R���ʹ�� 
    gpio_init(PTC5,GPO,1);// L���ʹ�� 
//------------��������ʼ��--------------//
    ftm_quad_init2();
//------------�������ڳ�ʼ��------------//    
    uart_init(UART4, 115200);
    uart_rx_irq_en(UART4);
//------------nrf��ʼ��-----------------//
    while(!nrf_init());
    enable_irq(PORTE_IRQn);
//------------�ɻɹ�ͣ��----------------//
    gpio_init(PTA26, GPI, LOW);
    port_init_NoALT (PTA26, IRQ_RISING | PF | PULLUP );
    //enable_irq(PORTA_IRQn);
//------------10ms��ʱ������------------//    
    pit_init_ms(PIT1,10);
    enable_irq(PIT1_IRQn);
//------------1ms�жϳ�ʼ��-------------//
    pit_init_ms(PIT0,1);
    enable_irq(PIT0_IRQn);
    
}

uint8 SchemeSelecte(void)
{
    return 1;
}

//-------------------------�ж����ȼ�-------------------------//
void SetPriority()
{
    NVIC_SetPriorityGrouping(4);//    XXXX   ��λ��ռʽ����ֵԽС���ȼ�Խ�� 

    NVIC_SetPriority(PIT0_IRQn,3);      //ֱ��
    
    NVIC_SetPriority(PORTA_IRQn,2);     //�ɻɹ�
    
    NVIC_SetPriority(PIT1_IRQn,12);      //����ɨ��
    
    NVIC_SetPriority(UART4_RX_TX_IRQn,1);   //�����ж�
    
    NVIC_SetPriority(PORTE_IRQn,10);    //NRF
    
}

void TestOffset()
{
    //X<=>Stand   Y<=>Direction
    float fGyroOut_X_Offset=0,fGyroOut_Y_Offset=0;
    uint16 i =0;

    for(i=0;i<200;i++)
    {
#ifdef LQ
        fGyroOut_X_Offset += read_lq(SlaveAddress2100,'Y');
        fGyroOut_Y_Offset += - read_lq(SlaveAddress2100,'Z');
#else        
        fGyroOut_X_Offset = - L3G_read_reg(1,'X'); 
        fGyroOut_Y_Offset = - L3G_read_reg(1,'Y'); 
#endif        
    }
    
    fGyroOut_X_Offset/=200;
    fGyroOut_Y_Offset/=200;
    
    msg_datascope( fGyroOut_X_Offset,fGyroOut_Y_Offset,0,0,0,0,0 );
    
}

void Integral_Init_Val()
{
    float fGravityOut_X,fGravityOut_Y;
    
#ifdef LQ    
    
#define  GRAVITY_OFFSET_Z       (-40.0f)
    
    fGravityOut_X = read_lq(SlaveAddress8700,'X');
    
    fGravityOut_Y = read_lq(SlaveAddress8700,'Z');

    g_fGravityOut_Z = 100*atan2(fGravityOut_X,fGravityOut_Y);

#else

#define  GRAVITY_OFFSET_Z       (-40.0f)       //���ٶȼ�Z��ƫ�� 
    
    fGravityOut_X = MMA8451_read_reg(1,'Y');      //Z��ֱ�ڱ���   Y  ������-90��
    
    fGravityOut_Y = MMA8451_read_reg(1,'Z');
    
    g_fGravityOut_Z = 100*atan2(fGravityOut_X,fGravityOut_Y);
    
#endif    
    
    g_fAnglebyFilter = g_fGravityOut_Z - GRAVITY_OFFSET_Z;
    
    g_fGyroscopeAngleIntegral = g_fAnglebyFilter;
    
}
