#ifndef _OLED_H
#define _OLED_H
#include "common.h"

#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,����λ��Ϊ0--31��Ч
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //�ѵ�ǰλ��1

//����OLED����
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


 void LCD_Init(void);                   //oled��ʼ��
 void LCD_CLS(void);                    //����
 
 void LCD_P6x8Str(uint8 x,uint8 y,uint8 ch[]); //д��һ���׼ASCII�ַ���  ---̫С������
 void LCD_P8x16Str(uint8 x,uint8 y,uint8 ch[]);//д��һ���׼ASCII�ַ���
                                             //X*Y=��*��  
                                             //��0~119 ��0~7
 
 void LCD_P14x16Str(uint8 x,uint8 y,uint8 ch[]);//��������ַ���
 void LCD_Print(uint8 x, uint8 y, uint8 ch[]);//������ֺ��ַ�����ַ���
 void LCD_PutPixel(uint8 x,uint8 y);        //����һ���㣨x,y��
 void LCD_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);//����һ��ʵ�ľ���

 
 
 void Dis_Num(uint8 x, uint8 y, int16 num,uint8 N) ;              //��ʾ����
 void Dis_Float2(uint8 X,uint8 Y,double real,uint8 N1,uint8 N2) ;//��ʾ������
 void Dis_Float(uint8 X,uint8 Y,double real,uint8 N) ; //-----N=С�����λ��




 void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]); //��ʾBMPͼƬ128��64
 void LCD_Fill(uint8 dat);//��Ļ��ʾ����
    
#endif

