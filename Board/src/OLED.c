/*
使用说明：
OLED电源使用3.3V。   
----------------
G    电源地
3.3V 接3.3V电源
D0   PORTA_PA14  
D1   PORTA_PA15
RST  PORTA_PA16 
DC   PORTA_PA17
CS   已接地，不用接
============================================
OLED电源使用5V。   
----------------
G    电源地
3.3V 接5V电源，电源跟模块之间串接100欧姆电阻，并加3.3V钳位二极管
D0   PORTA_PA14 单片机跟模块之间串接1k-3.3k电阻 
D1   PORTA_PA15 单片机跟模块之间串接1k-3.3k电阻 
RST  PORTA_PA16 单片机跟模块之间串接1k-3.3k电阻 
DC   PORTA_PA17 单片机跟模块之间串接1k-3.3k电阻 
CS   已接地，不用接     
============================================     
如果用户使用的是5V单片机，请看用户手册，切勿烧毁模块！  
============================================*/
#include "OLED.h"
#include "MK60_gpio.h"
#define X_WIDTH 128
#define Y_WIDTH 64
//======================================


void LCD_WrDat(uint8 data)
{
    uint8 i=8;
    //LCD_CS=0;;
    gpio_set (DC, 1);//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(17)); //IO口输出高电平
    //asm("nop"); 
    gpio_set (D0, 0);//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));//IO口输出低电平
    //asm("nop");    
    while(i--)
    {
        if(data&0x80){gpio_set(D1,1);}//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(15));}
        else{gpio_set(D1,0);}//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(15));}
        gpio_set(D0,1);// GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(14)); 
        asm("nop");
                //asm("nop");            
        gpio_set(D0,0);//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));    
        data<<=1;    
    }
	//LCD_CS=1;
}


void LCD_WrCmd(uint8 cmd)
{
    uint8 i=8;

    //LCD_CS=0;;
    gpio_set (DC, 0);// GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(17));
    gpio_init(RST,GPI,0);
    gpio_set (D0, 0);//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));
    //asm("nop");   
    while(i--)
    {
        if(cmd&0x80){gpio_set(D1,1);}//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(15));}
        else{gpio_set(D1,0);}//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(15));;}
        gpio_set(D0,1);//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(14));
        asm("nop");
        //asm("nop");             
        gpio_set(D0,0);// GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));    
        cmd<<=1;   
    } 	
	//LCD_CS=1;
}


//OLED 设置坐标
void LCD_Set_Pos(uint8 x, uint8 y)
{ 
    LCD_WrCmd(0xb0+y);
    LCD_WrCmd(((x&0xf0)>>4)|0x10);
    LCD_WrCmd((x&0x0f)|0x01); 
} 

//bmp_dat=0x00全屏灭，bmp_dat=0xff全屏亮  
void LCD_Fill(uint8 bmp_data)
{
    uint8 y,x;
    
    for(y=0;y<8;y++)
    {
        LCD_WrCmd(0xb0+y);
        LCD_WrCmd(0x01);
        LCD_WrCmd(0x10);
        for(x=0;x<X_WIDTH;x++)
            LCD_WrDat(bmp_data);
    }
}

void LCD_CLS(void)
{
    uint8 y,x;
	
    for(y=0;y<8;y++)
    {
        LCD_WrCmd(0xb0+y);
        LCD_WrCmd(0x01);
        LCD_WrCmd(0x10); 
        for(x=0;x<X_WIDTH;x++)
          LCD_WrDat(0);
    }
}

//////清除某一行，y对应于行号（我用6*8的字符，行号到第8（0-7）行截止）--王明龙
void LCD_ClrOneLine(uint8 y)
{
    uint8 x;
	
	if(y>=8) return;
	
    for(;y<8;y++)
    {
        LCD_WrCmd(0xb0+y);
        LCD_WrCmd(0x01);
        LCD_WrCmd(0x10); 
        for(x=0;x<X_WIDTH;x++)
          LCD_WrDat(0);
    }
}


//////清除多行(第y1行--y2行)，y1，y2（0-7）对应于行号（我用6*8的字符，行号到第8（0-7）行截止）--王明龙
void LCD_ClrLines(uint8 y1,uint8 y2)
{
    uint8 x;
	
	if(y1>=8|y2>=8) return;
	
    for(;y1<y2;y1++)
    {
        LCD_WrCmd(0xb0+y1);
        LCD_WrCmd(0x01);
        LCD_WrCmd(0x10); 
        for(x=0;x<X_WIDTH;x++)
          LCD_WrDat(0);
    }
}


void LCD_DLY_ms(uint16 ms)
{                         
    uint16 a;
    while(ms)
    {
        a=13350;
        while(a--);
        ms--;
    }
    return;
}

void LCD_Init(void)        
{
    //设置PORTA pin14,pin15为GPIO口 
    gpio_init(D0,GPO,1);//PORTA_PCR14=(0|PORT_PCR_MUX(1));
    gpio_init(D1,GPO,1);//PORTA_PCR15=(0|PORT_PCR_MUX(1)); 
    gpio_init(RST,GPO,1);//PORTA_PCR16=(0|PORT_PCR_MUX(1));
    gpio_init(DC,GPO,1);//PORTA_PCR17=(0|PORT_PCR_MUX(1)); 

    //设置PORTA pin14,pin15为输出方向;pin16,pin17为输入方向
    //GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(14)|GPIO_PIN(15)|GPIO_PIN(16)|GPIO_PIN(17));


    gpio_set(D0,1);//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(14));
    //LCD_CS=1;	//预制SLK和SS为高电平   	

    gpio_turn(RST);//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(16));//0
    LCD_DLY_ms(50);
    gpio_turn(RST);//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));//1
    gpio_init(RST,GPI,0);

    LCD_WrCmd(0xae);//--关闭OLED面板
    LCD_WrCmd(0x00);//---设定较低的列地址 
    LCD_WrCmd(0x10);//---设置高的列地址
    LCD_WrCmd(0x40);//--设置起始行地址  集映射内存显示起始行 (0x00~0x3F)
    LCD_WrCmd(0x81);//--设置对比度控制寄存器
    LCD_WrCmd(0xcf); // 赛格输出电流亮度设置
    LCD_WrCmd(0xa1);//--设置赛格/列映射     0xa0左右反置 0xa1正常
    LCD_WrCmd(0xc8);//设置COM /行扫描方向  0xc0上下反置 0xc8正常
    LCD_WrCmd(0xa6);//--正常显示
    LCD_WrCmd(0xa8);//---集复用率(1 to 64)
    LCD_WrCmd(0x3f);//--1/64 duty
    LCD_WrCmd(0xd3);//-设置显示偏移位移映射内存计数器 (0x00~0x3F)
    LCD_WrCmd(0x00);//-不偏移
    LCD_WrCmd(0xd5);//---设置显示时钟的分频比/振荡器频
    LCD_WrCmd(0x80);//--组分比，时钟设置为100帧
    LCD_WrCmd(0xd9);//--设置预充电周期
    LCD_WrCmd(0xf1);//设置预充电15钟放1钟
    LCD_WrCmd(0xda);//--COM引脚设置硬件配置
    LCD_WrCmd(0x12);
    LCD_WrCmd(0xdb);//--set vcomh
    LCD_WrCmd(0x40);//Set VCOM Deselect Level
    LCD_WrCmd(0x20);//-设置页面寻址方式 (0x00/0x01/0x02)
    LCD_WrCmd(0x02);//
    LCD_WrCmd(0x8d);//--充电泵启用/禁
    LCD_WrCmd(0x14);//--set(0x10) disable
    LCD_WrCmd(0xa4);// 使整个显示(0xa4/0xa5)
    LCD_WrCmd(0xa6);// 禁用反显示(0xa6/a7) 
    LCD_WrCmd(0xaf);//---打开面板 
    LCD_Fill(0x00);  //初始清屏
    LCD_Set_Pos(0,0);  
	
}


//==============================================================
//函数名： void LCD_PutPixel(uint8 x,uint8 y)
//功能描述：绘制一个点（x,y）
//参数：真实坐标值(x,y),x的范围0～127，y的范围0～64
//返回：无
//==============================================================
void LCD_PutPixel(uint8 x,uint8 y)
{
    uint8 data1;  //data1当前点的数据 
     
    LCD_Set_Pos(x,y); 
    data1 = 0x01<<(y%8); 	
    LCD_WrCmd(0xb0+(y>>3));
    LCD_WrCmd(((x&0xf0)>>4)|0x10);
    LCD_WrCmd((x&0x0f)|0x00);
    LCD_WrDat(data1); 	 	
}


//==============================================================
//函数名： void LCD_CCDPixel(uint8 x,uint8 y)
//功能描述：绘制CCD采集的曲线
//参数：真实坐标值(x,y),x的范围0～127，y的范围0～64
//返回：无
//==============================================================
void LCD_CCDPixel(uint8 *p)
{
	uint8 i=0;
	for(;i<128;i++)
	{
	  	LCD_PutPixel(i, 64-(*p)/4);
		p++;
	}

}




//==============================================================
//函数名： void LCD_Rectangle(uint8 x1,uint8 y1,
//                   uint8 x2,uint8 y2,uint8 color,uint8 gif)
//功能描述：绘制一个实心矩形
//参数：左上角坐标（x1,y1）,右下角坐标（x2，y2）
//      其中x1、x2的范围0～127，y1，y2的范围0～63，即真实坐标值
//返回：无
//==============================================================
void LCD_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif)
{
    uint8 n; 

    LCD_Set_Pos(x1,y1>>3);
    for(n=x1;n<=x2;n++)
    {
        LCD_WrDat(0x01<<(y1%8)); 			
        if(gif == 1) 	LCD_DLY_ms(50);
    }  
    LCD_Set_Pos(x1,y2>>3);
    for(n=x1;n<=x2;n++)
    {
          LCD_WrDat(0x01<<(y2%8)); 			
          if(gif == 1) 	LCD_DLY_ms(5);
    }
	
}  
//==============================================================
//函数名：LCD_P6x8Str(uint8 x,uint8 y,uint8 *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void LCD_P6x8Str(uint8 x,uint8 y,uint8 ch[])
{
    uint8 c=0,i=0,j=0;      
    while (ch[j]!='\0')
    {    
        c =ch[j]-32;
        if(x>126)  {x=0;y++;}
        LCD_Set_Pos(x,y);    
        for(i=0;i<6;i++)     
            LCD_WrDat(F6x8[c][i]);  
        x+=6;
        j++;
    }
}


//==============================================================
//函数名：LCD_P8x16Str(uint8 x,uint8 y,uint8 *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void LCD_P8x16Str(uint8 x,uint8 y,uint8 ch[])
{
    uint8 c=0,i=0,j=0;

    while (ch[j]!='\0')
    {    
        c =ch[j]-32;
        if(x>120){x=0;y++;}
        LCD_Set_Pos(x,y);    
        for(i=0;i<8;i++)     
            LCD_WrDat(F8X16[c*16+i]);
        LCD_Set_Pos(x,y+1);
        for(i=0;i<8;i++)
            LCD_WrDat(F8X16[c*16+i+8]);  
        x+=8;
        j++;
    }
}
//显示16*16点阵  显示的坐标（x,y），y为页范围0～7 
void LCD_P16x16Str(uint8 x,uint8 y,uint8 N) 
{  
    uint8 wm=0;  
    uint16 adder=32*N;  //        
    LCD_Set_Pos(x , y);  
    for(wm = 0;wm < 16;wm++)  //               
    {  
        LCD_WrDat(F16x16[adder]);   
        adder += 1;  
    }        
    LCD_Set_Pos(x,y + 1);   
    for(wm = 0;wm < 16;wm++) //           
    {  
        LCD_WrDat(F16x16[adder]);  
        adder += 1;  
    }           
}  
//输出汉字字符串
void LCD_P14x16Str(uint8 x,uint8 y,uint8 ch[])
{
    uint8 wm=0,ii = 0;
    uint16 adder=1; 

    while(ch[ii] != '\0')
    {
        wm = 0;
        adder = 1;
        while(F14x16_Idx[wm] > 127)
        {
            if(F14x16_Idx[wm] == ch[ii])
            {
                if(F14x16_Idx[wm + 1] == ch[ii + 1])
                {
                    adder = wm * 14;
                    break;
                }
            }
            wm += 2;			
        }
        if(x>118){x=0;y++;}
        LCD_Set_Pos(x , y); 
        if(adder != 1)// 显示汉字					
        {
            LCD_Set_Pos(x , y);
            for(wm = 0;wm < 14;wm++)               
            {
                LCD_WrDat(F14x16[adder]);	
                adder += 1;
            }      
            LCD_Set_Pos(x,y + 1); 
            for(wm = 0;wm < 14;wm++)          
            {
                LCD_WrDat(F14x16[adder]);
                adder += 1;
            }   		
        }
        else			  //显示空白字符			
        {
            ii += 1;
            LCD_Set_Pos(x,y);
            for(wm = 0;wm < 16;wm++)
                LCD_WrDat(0);
            LCD_Set_Pos(x,y + 1);
            for(wm = 0;wm < 16;wm++)
                LCD_WrDat(0);	
        }
        x += 14;
        ii += 2;
    }
}
//输出汉字和字符混合字符串
void LCD_Print(uint8 x, uint8 y, uint8 ch[])
{
    uint8 ch2[3];
    uint8 ii=0;        
    while(ch[ii] != '\0')
    {
        if(ch[ii] > 127)
        {
            ch2[0] = ch[ii];
            ch2[1] = ch[ii + 1];
            ch2[2] = '\0';			//汉字为两个字节
            LCD_P14x16Str(x , y, ch2);	//显示汉字
            x += 14;
            ii += 2;
        }
        else
        {
            ch2[0] = ch[ii];	
            ch2[1] = '\0';			//字母占一个字节
            LCD_P8x16Str(x , y , ch2);	//显示字母
            x += 8;
            ii+= 1;
        }
    }
} 

//==============================================================
//函数名： void Draw_BMP(uint8 x,uint8 y)
//功能描述：显示BMP图片128×64
//参数：起始点坐标(x,y),x的范围0～127，y为页的范围0～7
//返回：无
//==============================================================
void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[])
{ 	
    uint16 ii=0;
    uint8 x,y;

    if(y1%8==0) y=y1/8;      
    else y=y1/8+1;
    for(y=y0;y<=y1;y++)
    {
        LCD_Set_Pos(x0,y);				
        for(x=x0;x<x1;x++)
            LCD_WrDat(bmp[ii++]);	    	
    }
}
//清除n个16*16字  
void OLED_Cler_16x16(uint8 x,uint8 y, uint8 n)
{  
    uint8 i=0;  
    LCD_Set_Pos(x,y);  
    for(i=0;i<16*n;i++)  
        LCD_WrDat(0x00);  
    LCD_Set_Pos(x,y+1);  
    for(i=0;i<16*n;i++)  
        LCD_WrDat(0x00);  
}

//清除n个8*16字符  
void OLED_Cler_8x16(uint8 x,uint8 y,uint8 n) 
{  
    uint8 i;  
    if(x>120)  {x=0;y++;}  
    LCD_Set_Pos(x,y);  
    for(i=0;i<8*n;i++)  
        LCD_WrDat(0x00);  
    LCD_Set_Pos(x,y+1);  
    for(i=0;i<8*n;i++)  
        LCD_WrDat(0x00);      
}  
//清除N个6*8字符 
void OLED_Cler_6x8(uint8 x,uint8 y,uint8 n) 
{  
    uint16 i=0;  
    LCD_Set_Pos(x,y);  
    if(x>126)  
    {   x=0;    y++;   }  
    for(i=0;i<6*n;i++)  
        LCD_WrDat(0x00);  
}  
//显示数字
void Dis_Num(uint8 x, uint8 y, int16 num,uint8 N) 
{
    // uint8 line;
    uint8 j=0;
    uint8 n[6]={0};
    x=x*8;
    n[0]=(int)num/10000; 
    n[1]=(int)num/1000%10;
    n[2]=(int)(num/100)%10;
    n[3]=(int)(num/10)%10;
    n[4]=(int)num%10;
    n[5]='\0';
    for(j=0;j<5;j++) n[j]=n[j]+16+32;
        LCD_P8x16Str(x,y,&n[5-N]);//从ACSII码表中读取字节，然后写入液晶
}

//显示浮点型数据
void Dis_Float(uint8 X,uint8 Y,double real,uint8 N) 
{
   uint8   i_Count=1;
   uint8   n[12]={0};
   long   j=1;  
   uint32   real_int=0;
   double decimal=0;
   uint32   real_decimal=0;


   X=X*8;
   
   if(real<0) 
   {
		real=0-real;
        LCD_P8x16Str(X,Y,"*"); 
        X=X+8;
   }
   real_int=(int32)real;

   decimal=real-real_int;
   real_decimal=(uint32)(decimal*1e4);

   while(real_int/10/j!=0)
   {
      j=j*10;i_Count++;  
   } 
   n[0]=(real_int/10000)%10; 
   n[1]=(real_int/1000)%10; 
   n[2]=(real_int/100)%10;
   n[3]=(real_int/10)%10;
   n[4]=(real_int/1)%10;
   n[5]='.'; 
   n[6]=(real_decimal/1000)%10;
   n[7]=(real_decimal/100)%10;
   n[8]=(real_decimal/10)%10;
   n[9]=(real_decimal/1)%10;
   n[6+N]='\0';


   for(j=0;j<12;j++) n[j]=n[j]+'0';
   n[5]='.';
   n[6+N]='\0';
   
   LCD_P8x16Str(X,Y,&n[5-i_Count]); 
}

void Dis_Float2(uint8 X,uint8 Y,double real,uint8 N1,uint8 N2) 
{
   uint8   i_Count=1;
   uint8   n[12]={0};
   long   j=1;  
   uint16   real_int=0;
   double decimal=0;
   uint16   real_decimal=0;
   X=X*8;
   real_int=(int)real;
   //Dis_Num(2,0,real_int,5);
   decimal=real-(double)real_int;
   real_decimal=(uint32)(decimal*1e4);
   //Dis_Num(2,6,real_decimal,4);
   while(real_int/10/j!=0)
   {
      j=j*10;i_Count++;  
   } 
   n[0]=(real_int/10000)%10; 
   n[1]=(real_int/1000)%10;
   n[2]=(real_int/100)%10;
   n[3]=(real_int/10)%10;
   n[4]=(real_int/1)%10; 
   n[5]='.';
   n[6]=(real_decimal/1000)%10;
   n[7]=(real_decimal/100)%10;
   n[8]=(real_decimal/10)%10;
   n[9]=real_decimal%10;
   n[6+N2]='\0'; 
   for(j=0;j<10;j++) n[j]=n[j]+16+32;
   n[5]='.';
   n[6+N2]='\0';   
   LCD_P6x8Str(X,Y,&n[5-N1]); 
}

const uint8 F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines
};
//======================================================
// 128X64I液晶底层驱动[8X16]字体库
// 设计者: powerint
// 描  述: [8X16]西文字符的字模数据 (纵向取模,字节倒序)
// !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
//======================================================
const uint8 F8X16[]=
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//!1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//"2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//#3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//%5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//&6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//'7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//(8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//)9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//*10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//,12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//-13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//.14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,///15
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//016
  0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//117
  0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//218
  0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//319
  0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//420
  0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//521
  0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//622
  0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//723
  0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//824
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//925
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//:26
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//;27
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//<28
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//=29
  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//>30
  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//?31
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A33
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B34
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C35
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D36
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E37
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F38
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G39
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H40
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I41
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J42
  0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K43
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L44
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M45
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N46
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O47
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P48
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q49
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R50
  0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S51
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T52
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U53
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V54
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W55
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X56
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y57
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z58
  0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[59
  0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\60
  0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//]61
  0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^62
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_63
  0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//`64
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a65
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b66
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c67
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d68
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e69
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f70
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g71
  0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h72
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i73
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j74
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k75
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l76
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m77
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n78
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o79
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z90
  0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{91
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//|92
  0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//}93
  0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~94

};