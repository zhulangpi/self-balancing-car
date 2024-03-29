#include "include.h"
#include "L3G4200D.h"
/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：IIC.c
 * 描述         ：加速度计和陀螺仪软件模拟IIC驱动程序
 * 实验平台     ：凌立印象开发板
 * 库版本       ：基于野火库
 * 嵌入系统     ：
 * 作者         ：xuxu
**********************************************************************************/




/************************************************
*  函数名称：IIC_start
*  功能说明：IIC start
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
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
*  函数名称：IIC_stop
*  功能说明：IIC end//送停止位 SDA=0->1
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
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
*  函数名称：IIC_send_byte
*  功能说明：IIC end字节发送程序
*  参数说明：c为字节
*  函数返回：无，不考虑从应答位
*  修改时间：2014-1-14    已经测试
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
            SDA_H_; //判断发送位
        else
            SDA_L_;
        nop5();
        SCL_H_;
        nops();
        SCL_L_;
    }
    nops();
    SDA_H_; //发送完8bit，释放总线准备接收应答位
    nop5();
    SCL_H_;
    nops(); //sda上数据即是从应答位
    SCL_L_; //不考虑从应答位|但要控制好时序
}



/************************************************
*  函数名称：IIC_read_byte
*  功能说明：字节接收程序,接收器件传来的数据
*  参数说明：无
*  函数返回：return: uchar型1字节
*  修改时间：2014-1-14    已经测试
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
        SCL_L_; //置时钟线为低，准备接收数据位
        nops();
        SCL_H_; //置时钟线为高，使数据线上数据有效
        nop5();
        c<<=1;
        if(SDA_read)
            c+=1; //读数据位，将接收的数据存c
    }
    SCL_L_;
    return c;
}




/************************************************
*  函数名称：IIC_Single_Write
*  功能说明：//写入寄存器
*  参数说明：SlaveAddress设备ID，寄存器地址address，thedata为写入数据
*  函数返回：return: uchar型1字节
*  修改时间：2014-1-14    已经测试
*************************************************/
void Single_Write(unsigned char SlaveAddress,unsigned char address, unsigned char thedata)
{
    IIC_start();		//启动
    send_byte(SlaveAddress);	//写入设备ID及写信号
    send_byte(address);	//X地址
    send_byte(thedata);	//写入设备ID及读信
    IIC_stop();
}



/************************************************
*  函数名称：IIC_Single_Read
*  功能说明：//读寄存器
*  参数说明：SlaveAddress设备ID，寄存器地址address
*  函数返回：return1个字节，ret为读出数据
*  修改时间：2014-1-14    已经测试
*************************************************/
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char address)
{
    unsigned char ret = 100;
    IIC_start();		//启动
    send_byte(SlaveAddress);	//写入设备ID及写信号
    send_byte(address);	//X地址
    IIC_start();		//重新发送开始
    send_byte(SlaveAddress+1);	//写入设备ID及读信
    ret = read_byte();	//读取一字节
    IIC_stop();
    return ret;
}


/************************************************
*  函数名称：Init_L3G4200D
*  功能说明：陀螺仪的初始化函数
*  参数说明：无
*  函数返回：无
*  修改时间：2014-1-14    已经测试
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
*  函数名称：Get_Gyro
*  功能说明：读取L3G4200D数据
*  参数说明：n为读几次，Axis为哪个轴,'X''Y''Z'
*  函数返回：角速度的值
*  修改时间：2014-1-14    已经测试
*************************************************/
long int L3G_read_reg(unsigned char n,unsigned char Axis) //n为读几次
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

    数字加速度计MMA8451


***************************************************************/


//MMA8451初始化
void MMA8451_Init()
{	

    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,MMACTRL_REG1,ASLP_RATE_20MS+DATA_RATE_5MS);
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,XYZ_DATA_CFG_REG, FULL_SCALE_2G); //2G
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,HP_FILTER_CUTOFF_REG, PULSE_LPF_EN_MASK ); //低通滤波
    nops(); nops(); nops(); nops(); nops(); nops();
    Single_Write(MMA845x_IIC_ADDRESS,MMACTRL_REG1, ACTIVE_MASK);          //激活状态
    nops(); nops(); nops(); nops(); nops(); nops();
	
}



long int MMA8451_read_reg(unsigned char n,unsigned char Axis) //n为读几次
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


