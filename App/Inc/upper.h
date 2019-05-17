#ifndef _UPPER_H_
#define _UPPER_H_




//------------------------Serial_Digital_Scope V2---------------------//

extern float  OutData[4];

extern void OutPut_Data(void);

extern void upper1();

//--------------------------------Data_Scope---------------------------//

extern unsigned char DataScope_OutPut_Buffer[42];	   //������֡���ݻ�����

extern void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����

extern unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 
 
extern void upper2();


/*****************************************

                NRF����

*****************************************/

/**************************************************
        ���Ͷ���
***************************************************/

typedef enum    //@brief ��Ϣ���ͱ���
{
    
    Communication_e = 0,
    
    Angle_Set_e = 0x10,              //PID����
    Angle_Kp_e,           
    Angle_Ki_e,  
    Angle_Kd_e,
    Speed_Set_e,
    Speed_Kp_e,
    Speed_Ki_e,
    Speed_Kd_e,
    
    
    Startstop_e = 0x32,//һ��ͣ��/����
    Backward_e = 0x34,//һ��ɲ��
    
    //------------��ά��------------//
    Paramf_e=0x53,
    Paramm_e=0x59,
    Params_e=0x5a,
    Paramb_e=0x5b,
    //-------------------------------//
    
    


    

    

    
} msg_type_e;


/**************************************************
        ��������
***************************************************/

extern uint8 msg_buff_rx[DATA_PACKET];//������Ϣ����
extern uint8 msg_buff_tx[DATA_PACKET];

/************************************
    ��������
*************************************/

void msg_datascope(float data0, float data1, float data2, float data3,float data4, float data5,float data6);

void msg_visualscope(float data0, float data1, float data2, float data3);

void msg_deal(void);


#endif
