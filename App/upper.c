
#include "include.h"


float  OutData[4]={0};
static uint16 CRC_CHECK(uint8 *Buf, uint8 CRC_CNT);


//--------------------------------------------------------------------------------------//
//-----------------------------Serial_Digital_Scope V2----------------------------------//
//--------------------------------------------------------------------------------------//

void upper1(void)
{
    OutData[0]=g_fCarAngle;
    OutData[1]=g_fGravityAngle;
    OutData[2]=g_fGyroscopeAngleSpeed_X;
    OutData[3]=g_fGyroscopeAngleIntegral;
    OutPut_Data();
}

/*
*	����˵����SCI ʾ�������ͺ���
*	����˵����
OutData[]	��Ҫ���͵���ֵ���������
*	�������أ��޷��Ž��ֵ
*	�޸�ʱ�䣺2013-2-10
*/

void OutPut_Data(void)
{
    int16 temp[4] = {0};
    uint16 temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;

    for(i=0;i<4;i++)
    {
        temp[i]	 = (int16)OutData[i];
        temp1[i] = (uint16)temp[i];
    }
    for(i=0;i<4;i++)
    {
        databuf[i*2]	= (unsigned char)(temp1[i]%256);
        databuf[i*2+1]  = (unsigned char)(temp1[i]/256);
    }
    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;
    for(i=0;i<10;i++)
       uart_putchar (UART4,(char)databuf[i]);
}

//**************************************************************************
/*
*  ����˵����SCIʾ����CRCУ��
�ڲ����ú���
*  ����˵���� ��
*  �������أ��޷��Ž��ֵ
*  �޸�ʱ�䣺2013-2-10
*/
//**************************************************************************
static uint16 CRC_CHECK(uint8 *Buf, uint8 CRC_CNT)
{
    uint16 CRC_Temp;
    uint8 i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}


//--------------------------------------------------------------------------------------//
//-----------------------------------------Data_Scope-----------------------------------//
//--------------------------------------------------------------------------------------//


uint8 DataScope_OutPut_Buffer[42] = {0};	   //���ڷ��ͻ�����


//����˵�����������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ 
//����˵�����û�����ֱ�Ӳ����˺��� 
//target:Ŀ�굥��������
//buf:��д������
//beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
//�����޷��� 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}


//����˵������������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
//Data��ͨ������
//Channel��ѡ��ͨ����1-10��
//�����޷��� 
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
  
  if ( (Channel > 10) || (Channel == 0) ) return;  //ͨ����������10�����0��ֱ����������ִ�к���
  else
  {
     switch (Channel)
		{
            case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
            case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		    case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		    case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		    case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		    case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		    case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		    case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		    case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		    case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}


//����˵�������� DataScopeV1.0 ����ȷʶ���֡��ʽ
//Channel_Number����Ҫ���͵�ͨ������
//���ط��ͻ��������ݸ���
//����0��ʾ֡��ʽ����ʧ�� 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{

  if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //ͨ����������10�����0��ֱ����������ִ�к���
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';  //֡ͷ
		
	 switch(Channel_Number)   
    { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6; break;   
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10; break;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; break;
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18; break;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22; break; 
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26; break;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; break;
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; break;
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38; break;
         case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; break;
    }	 
  }
	return 0;
}

void upper2(void)
{

    DataScope_Get_Channel_Data(g_fCarAngle,1);
    DataScope_Get_Channel_Data(g_fSpeedbyCoder,  2);
    DataScope_Get_Channel_Data( PID_Speed.Set,  3);
//    DataScope_Get_Channel_Data( 0,  4);
//    DataScope_Get_Channel_Data( 0,  5);
//    DataScope_Get_Channel_Data( 0,  6);
//    DataScope_Get_Channel_Data( 0,  7);
//    DataScope_Get_Channel_Data( 0,  8);
//    DataScope_Get_Channel_Data( 0,  9);
//    DataScope_Get_Channel_Data( 0,  10);
    

    
    
    DataScope_Data_Generate(10);
    uart_putbuff( UART4,DataScope_OutPut_Buffer,DataScope_Data_Generate(10));

}



/************************************************

                    NRF����

************************************************/

uint8 msg_buff_rx[DATA_PACKET];//������Ϣ����
uint8 msg_buff_tx[DATA_PACKET];

/**************************** visualscope ************************************/
void msg_visualscope(float data0, float data1, float data2, float data3)
{

    unsigned char i;
    unsigned short CRC16 = 0;

    msg_buff_tx[0]=0x0a; // ��λ��Ҫ�õĸ�ʽ

    int16 temp[4] = {0};
    uint16 temp1[4] = {0};
    
    temp[0]	 = (int16)data0;
    temp1[0] = (uint16)temp[0];
    temp[1]	 = (int16)data1;
    temp1[1] = (uint16)temp[1];
    temp[2]	 = (int16)data2;
    temp1[2] = (uint16)temp[2];
    temp[3]	 = (int16)data3;
    temp1[3] = (uint16)temp[3];

    for(i=0;i<4;i++)
    {
        msg_buff_tx[i*2+1]	= (unsigned char)(temp1[i]%256);
        msg_buff_tx[i*2+2]  = (unsigned char)(temp1[i]/256);
    }
    CRC16 = CRC_CHECK(msg_buff_tx+1,8);
    msg_buff_tx[9] = CRC16%256;
    msg_buff_tx[10] = CRC16/256;
  
  if(nrf_tx(msg_buff_tx,DATA_PACKET) == 1 )          //����һ�����ݰ���buff����Ϊ32�ֽڣ�
  {
      while(nrf_tx_state() == NRF_TXING);         //�ȴ��������
  }
}

/**************************** datascope ************************************/
void msg_datascope(float data0, float data1, float data2, float data3,float data4, float data5,float data6)
{
  msg_buff_tx[0]=30; // ��λ��Ҫ�õĸ�ʽ
  msg_buff_tx[1]=0x24;
  
  *(float*)((uint32)msg_buff_tx+2) = data0;
  *(float*)((uint32)msg_buff_tx+6) = data1;
  *(float*)((uint32)msg_buff_tx+10) = data2;
  *(float*)((uint32)msg_buff_tx+14) = data3;
  *(float*)((uint32)msg_buff_tx+18) = data4;
  *(float*)((uint32)msg_buff_tx+22) = data5;
  *(float*)((uint32)msg_buff_tx+26) = data6;
  msg_buff_tx[30]=29;
  if(nrf_tx(msg_buff_tx,DATA_PACKET) == 1 )          //����һ�����ݰ���buff����Ϊ32�ֽڣ�
  {
      while(nrf_tx_state() == NRF_TXING);
  }
}





/**************************************************************************
*  ����˵��������NRF24L01������Ϣ
            �����ж��Ƿ�����Ϣ���ٴ���
*************************************************************************/
void msg_deal()
{
    uint16 val,relen;
    float   temp;
    if(nrf_rx_fifo_check(1,&val))
    {
        relen = nrf_rx(msg_buff_rx,DATA_PACKET);
        relen = msg_buff_rx[2]*256+msg_buff_rx[3];

        switch((msg_type_e)msg_buff_rx[1])
        {
            case Angle_Set_e:   PID_Angle.Set = ( (int16)(msg_buff_rx[2]*256+msg_buff_rx[3]) )/100.0;
                                break;
            case Angle_Kp_e:    PID_Angle.Kp =  (msg_buff_rx[2]*256+msg_buff_rx[3])/100.0;
                                break;
            case Angle_Ki_e:    PID_Angle.Ki =  (msg_buff_rx[2]*256+msg_buff_rx[3])/100.0;
                                break;
            case Angle_Kd_e:    PID_Angle.Kd =  (msg_buff_rx[2]*256+msg_buff_rx[3])/100.0;
                                break;
            case Speed_Set_e:   PID_Speed.Set = (msg_buff_rx[2]*256+msg_buff_rx[3])/100.0;
                                break;
            case Speed_Kp_e:    PID_Speed.Kp =  (msg_buff_rx[2]*256+msg_buff_rx[3])/100.0;
                                break;
            case Speed_Ki_e:    PID_Speed.Ki =  (msg_buff_rx[2]*256+msg_buff_rx[3])/100.0;
                                break;
            case Speed_Kd_e:    PID_Speed.Kd =  (msg_buff_rx[2]*256+msg_buff_rx[3])/100.0;
                                break;
            case Paramf_e:
                                if(msg_buff_rx[2] < DirPD_x && msg_buff_rx[3] < DirPD_y)
                                {
                                    DirDymaticPDf[msg_buff_rx[2]][msg_buff_rx[3]][0]=msg_buff_rx[4]*256+msg_buff_rx[5];
                                    DirDymaticPDf[msg_buff_rx[2]][msg_buff_rx[3]][1]=msg_buff_rx[6]*256+msg_buff_rx[7];

                                }
                                break;
            case Startstop_e:
                                break;
            case Backward_e:
                                break;
                                
                                
            default:    break;
        }
    }

}

