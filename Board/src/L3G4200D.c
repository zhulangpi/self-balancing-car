#include "include.h"
#include "L3G4200D.h"
/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��IIC.c
 * ����         �����ٶȼƺ����������ģ��IIC��������
 * ʵ��ƽ̨     ������ӡ�󿪷���
 * ��汾       ������Ұ���
 * Ƕ��ϵͳ     ��
 * ����         ��xuxu
**********************************************************************************/




/************************************************
*  �������ƣ�IIC_start
*  ����˵����IIC start
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void IIC_start()
{
    SCLout;
    SDAout;
    SCL_L_;
    asm("nop");
    SDA_H_;
    nop5();
    SCL_H_;
    nops();
    SDA_L_;
    nops();
    SCL_L_;
}



/************************************************
*  �������ƣ�IIC_stop
*  ����˵����IIC end//��ֹͣλ SDA=0->1
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void IIC_stop()
{
    SCLout;
    SDAout;
    SCL_L_;nop5();
    SDA_L_;nop5();
    SCL_H_;nops();
    SDA_H_;nops();
    SCL_L_;
}




/************************************************
*  �������ƣ�IIC_send_byte
*  ����˵����IIC end�ֽڷ��ͳ���
*  ����˵����cΪ�ֽ�
*  �������أ��ޣ������Ǵ�Ӧ��λ
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void send_byte(unsigned char c)
{
    unsigned char i;
    SCLout;
    SDAout;asm("nop");
    for(i=0;i<8;i++)
    {
        SCL_L_;
        if((c<<i) & 0x80)
            SDA_H_; //�жϷ���λ
        else
            SDA_L_;
        nop5();
        SCL_H_;
        nops();
        SCL_L_;
    }
    nops();
    SDA_H_; //������8bit���ͷ�����׼������Ӧ��λ
    nop5();
    SCL_H_;
    nops(); //sda�����ݼ��Ǵ�Ӧ��λ
    SCL_L_; //�����Ǵ�Ӧ��λ|��Ҫ���ƺ�ʱ��
}



/************************************************
*  �������ƣ�IIC_read_byte
*  ����˵�����ֽڽ��ճ���,������������������
*  ����˵������
*  �������أ�return: uchar��1�ֽ�
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
unsigned char read_byte(void)
{
    unsigned char i;
    unsigned char c;
    SDAin;
    SCLout;
    c=0;
    SCL_L_;
    nop5();
    for(i=0;i<8;i++)
    {
        nop5();
        SCL_L_; //��ʱ����Ϊ�ͣ�׼����������λ
        nops();
        SCL_H_; //��ʱ����Ϊ�ߣ�ʹ��������������Ч
        nop5();
        c<<=1;
        if(SDA_read)
            c+=1; //������λ�������յ����ݴ�c
    }
    SCL_L_;
    return c;
}




/************************************************
*  �������ƣ�IIC_Single_Write
*  ����˵����//д��Ĵ���
*  ����˵����SlaveAddress�豸ID���Ĵ�����ַaddress��thedataΪд������
*  �������أ�return: uchar��1�ֽ�
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void Single_Write(unsigned char SlaveAddress,unsigned char address, unsigned char thedata)
{
    IIC_start();		//����
    send_byte(SlaveAddress);	//д���豸ID��д�ź�
    send_byte(address);	//X��ַ
    send_byte(thedata);	//д���豸ID������
    IIC_stop();
}



/************************************************
*  �������ƣ�IIC_Single_Read
*  ����˵����//���Ĵ���
*  ����˵����SlaveAddress�豸ID���Ĵ�����ַaddress
*  �������أ�return1���ֽڣ�retΪ��������
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char address)
{
    unsigned char ret = 100;
    IIC_start();		//����
    send_byte(SlaveAddress);	//д���豸ID��д�ź�
    send_byte(address);	//X��ַ
    IIC_start();		//���·��Ϳ�ʼ
    send_byte(SlaveAddress+1);	//д���豸ID������
    ret = read_byte();	//��ȡһ�ֽ�
    IIC_stop();
    return ret;
}


/************************************************
*  �������ƣ�Init_L3G4200D
*  ����˵���������ǵĳ�ʼ������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
void L3G4200D_init(void)
{
    Single_Write(L3G4200_Addr,CTRL_REG1, 0x0f);
    Single_Write(L3G4200_Addr,CTRL_REG2, 0x00);
    Single_Write(L3G4200_Addr,CTRL_REG3, 0x08);
    Single_Write(L3G4200_Addr,CTRL_REG4, 0x30);	//0X0--250dps    2000--0X30   0x10--500
    Single_Write(L3G4200_Addr,CTRL_REG5, 0x00);
}


                                                                        
/************************************************
*  �������ƣ�Get_Gyro
*  ����˵������ȡL3G4200D����
*  ����˵����nΪ�����Σ�AxisΪ�ĸ���,'X''Y''Z'
*  �������أ����ٶȵ�ֵ
*  �޸�ʱ�䣺2014-1-14    �Ѿ�����
*************************************************/
long int L3G_read_reg(unsigned char n,unsigned char Axis) //nΪ������
{
    unsigned char h,l;
    int sum=0;
    short int single=0;
    unsigned char i;
    unsigned char MSB,LSB;
	switch(Axis)
	{
		case 'X': MSB=OUT_X_H; LSB=OUT_X_L;break;
		case 'Y': MSB=OUT_Y_H; LSB=OUT_Y_L;break;
		case 'Z': MSB=OUT_Z_H; LSB=OUT_Z_L;break;
		default :break;
	}
	
    for(i=0;i<n;i++)
    {
        h = Single_Read(L3G4200_Addr,MSB);
        l = Single_Read(L3G4200_Addr,LSB);

        single = (h<<8u) + l;
        sum += single;
    }
    return(sum/n);
}


/**************************************************************

    ���ּ��ٶȼ�MMA8451


***************************************************************/


//MMA8451��ʼ��
void MMA8451_Init()
{	

    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,MMACTRL_REG1,ASLP_RATE_20MS+DATA_RATE_5MS);
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,XYZ_DATA_CFG_REG, FULL_SCALE_2G); //2G
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,HP_FILTER_CUTOFF_REG, PULSE_LPF_EN_MASK ); //��ͨ�˲�
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,MMACTRL_REG1, ACTIVE_MASK);          //����״̬
    nops(); nops(); nops(); nops(); nops(); nops();
	
}



long int MMA8451_read_reg(unsigned char n,unsigned char Axis) //nΪ������
{
    unsigned char h,l;
    int sum=0;
    short int single=0;
    unsigned char i;
    unsigned char MSB,LSB;
	switch(Axis)
	{
		case 'X': MSB=MMA8451_REG_OUTX_MSB; LSB=MMA8451_REG_OUTX_LSB;break;
		case 'Y': MSB=MMA8451_REG_OUTY_MSB; LSB=MMA8451_REG_OUTY_LSB;break;
		case 'Z': MSB=MMA8451_REG_OUTZ_MSB; LSB=MMA8451_REG_OUTZ_LSB;break;
		default :break;
	}
	
    for(i=0;i<n;i++)
    {
        h = Single_Read(MMA845x_IIC_ADDRESS,MSB);
        l = Single_Read(MMA845x_IIC_ADDRESS,LSB);

        single = (h<<8u) + l;
        single = single >> 2;
        sum += single;
    }
    return(sum/n);
}


