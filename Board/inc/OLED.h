#ifndef _OLED_H
#define _OLED_H
#include "common.h"

#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1

//定义OLED引脚
#define D0   PTB16
#define D1   PTB10
#define RST  PTB9
#define DC   PTB8

extern const uint8 F6x8[][6];
extern const uint8 F14x16_Idx[];
extern const uint8 F14x16[];
extern const uint8 F8X16[];
extern uint8 F16x16[];

#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF 


 void LCD_Init(void);                   //oled初始化
 void LCD_CLS(void);                    //清屏
 
 void LCD_P6x8Str(uint8 x,uint8 y,uint8 ch[]); //写入一组标准ASCII字符串  ---太小看不清
 void LCD_P8x16Str(uint8 x,uint8 y,uint8 ch[]);//写入一组标准ASCII字符串
                                             //X*Y=列*行  
                                             //列0~119 行0~7
 
 void LCD_P14x16Str(uint8 x,uint8 y,uint8 ch[]);//输出汉字字符串
 void LCD_Print(uint8 x, uint8 y, uint8 ch[]);//输出汉字和字符混合字符串
 void LCD_PutPixel(uint8 x,uint8 y);        //绘制一个点（x,y）
 void LCD_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);//绘制一个实心矩形

 
 
 void Dis_Num(uint8 x, uint8 y, int16 num,uint8 N) ;              //显示数据
 void Dis_Float2(uint8 X,uint8 Y,double real,uint8 N1,uint8 N2) ;//显示浮点数
 void Dis_Float(uint8 X,uint8 Y,double real,uint8 N) ; //-----N=小数点后位数




 void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]); //显示BMP图片128×64
 void LCD_Fill(uint8 dat);//屏幕显示操作
    
#endif

