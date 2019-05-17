#ifndef _UPPER_H_
#define _UPPER_H_




//------------------------Serial_Digital_Scope V2---------------------//

extern float  OutData[4];

extern void OutPut_Data(void);

extern void upper1();

//--------------------------------Data_Scope---------------------------//

extern unsigned char DataScope_OutPut_Buffer[42];	   //待发送帧数据缓存区

extern void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区

extern unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
 
extern void upper2();


/*****************************************

                NRF部分

*****************************************/

/**************************************************
        类型定义
***************************************************/

typedef enum    //@brief 消息类型编码
{
    
    Communication_e = 0,
    
    Angle_Set_e = 0x10,              //PID参数
    Angle_Kp_e,           
    Angle_Ki_e,  
    Angle_Kd_e,
    Speed_Set_e,
    Speed_Kp_e,
    Speed_Ki_e,
    Speed_Kd_e,
    
    
    Startstop_e = 0x32,//一键停车/启动
    Backward_e = 0x34,//一键刹车
    
    //------------二维表------------//
    Paramf_e=0x53,
    Paramm_e=0x59,
    Params_e=0x5a,
    Paramb_e=0x5b,
    //-------------------------------//
    
    


    

    

    
} msg_type_e;


/**************************************************
        变量声明
***************************************************/

extern uint8 msg_buff_rx[DATA_PACKET];//用于消息缓存
extern uint8 msg_buff_tx[DATA_PACKET];

/************************************
    函数声明
*************************************/

void msg_datascope(float data0, float data1, float data2, float data3,float data4, float data5,float data6);

void msg_visualscope(float data0, float data1, float data2, float data3);

void msg_deal(void);


#endif
