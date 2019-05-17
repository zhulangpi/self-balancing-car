
#include "include.h"

uint32 g_timecount=0;
float g_speed_direct = 1;


//---------------------参数表--------------------//
ParamEdit_t ParamTbl[20]=
{
    { &(PID_Angle.Set), { 0.1,    1, -0.1,   -1 ,10,-10} },
    { &(PID_Angle.Kp),  { 0.1,    1, -0.1,   -1 ,10,-10} },
    { &(PID_Angle.Ki),  { 0.1,    1, -0.1,   -1 ,10,-10} },
    { &(PID_Angle.Kd),  { 0.01, 0.05,  -0.01,  -0.05 ,-0.2,0.2} },
    
    { &(PID_Speed.Set), {   1,    5,  -1,   -5 ,0.5,-0.5} },
    { &(PID_Speed.Kp),  { 0.1,    1, -0.1,   -1 ,5,-5} },
    { &(PID_Speed.Ki),  { 0.01, 0.1, -0.01,  -0.1 ,0.005,-0.005} },
    { &(PID_Speed.Kd),  { 0.1,    1, -0.1,   -1 ,10,-10} },
    
    { &(PID_Dir.Set), { 0.1,    1, -0.1,   -1 ,10,-10} },
    { &(PID_Dir.Kp),  { 0.1,    1, -0.1,   -1 ,10,-10} },
    { &(PID_Dir.Ki),  { 0.1,    1, -0.1,   -1 ,10,-10} },
    { &(PID_Dir.Kd),  { 0.01, 0.05,  -0.01,  -0.05 ,1,-1} },
    
    { &(g_speed_direct),       { 1,    -1, 2,  -2, 4,  -4 } }
    
    
};

//------------------菜单表-----------------//
KbdTab_t KeyTbl[50]=
{
//-index--A---B---C--  
//-当前-切换-确定-返回
    { 0,  1,  4,  0,  MenuDisp },    //param
    { 1,  2, 40,  0,  MenuDisp },    //setting
    { 2,  3, 44,  0,  MenuDisp },    //display
    { 3,  0,  0,  0,  MenuDisp },    //reserved3--预留

    { 4,  5,  8,  0,  MenuDisp },    //angle     
    { 5,  6, 12,  0,  MenuDisp },    //speed     
    { 6,  7, 16,  0,  MenuDisp },    //direct--预留     
    { 7,  4, 20,  0,  MenuDisp },    //other--预留

    { 8,  9, 24,  4,  MenuDisp },    //angleset
    { 9, 10, 25,  4,  MenuDisp },    //kp
    {10, 11, 26,  4,  MenuDisp },    //ki
    {11,  8, 27,  4,  MenuDisp },    //kd

    {12, 13, 28,  5,  MenuDisp },    //speedset     
    {13, 14, 29,  5,  MenuDisp },    //kp     
    {14, 15, 30,  5,  MenuDisp },    //ki     
    {15, 12, 31,  5,  MenuDisp },    //kd

    {16, 17, 32,  6,  MenuDisp },    //directset     
    {17, 18, 33,  6,  MenuDisp },    //kp    
    {18, 19, 34,  6,  MenuDisp },    //ki     
    {19, 16, 35,  6,  MenuDisp },    //kd 

    {20, 21, 36,  7,  MenuDisp },    //other1
    {21, 22, 37,  7,  MenuDisp },    //other2
    {22, 23, 38,  7,  MenuDisp },    //other3
    {23, 20, 39,  7,  MenuDisp },    //other4


//数据处理
//-index--A---B---C------// 
//-当前-无效-无效-返回---//    
    {24,  0,  0,  8,  MenuDisp },    //edit angleset
    {25,  0,  0,  8,  MenuDisp },    //edit kp
    {26,  0,  0,  8,  MenuDisp },    //edit ki
    {27,  0,  0,  8,  MenuDisp },    //edit kd  
    
    {28,  0,  0, 12,  MenuDisp },    //edit speedset     
    {29,  0,  0, 12,  MenuDisp },    //edit kp     
    {30,  0,  0, 12,  MenuDisp },    //edit ki     
    {31,  0,  0, 12,  MenuDisp },    //edit kd    

    {32,  0,  0, 16,  MenuDisp },    //edit directset
    {33,  0,  0, 16,  MenuDisp },    //edit kp
    {34,  0,  0, 16,  MenuDisp },    //edit ki
    {35,  0,  0, 16,  MenuDisp },    //edit kd
    
    {36,  0,  0, 20,  MenuDisp },    //edit other1
    {37,  0,  0, 20,  MenuDisp },    //edit other2
    {38,  0,  0, 20,  MenuDisp },    //edit other3
    {39,  0,  0, 20,  MenuDisp },    //edit other4
    
//-----------------setting--------------------//

    {40, 41,  0,  1,  MenuDisp },    //angleorspeed
    {41, 42,  0,  1,  MenuDisp },
    {42, 43,  0,  1,  MenuDisp },
    {43, 40,  0,  1,  MenuDisp },
    
    
//------------------display-------------------//

    {44, 45,  0,  2,  MenuDisp },    //
    {45, 46,  0,  2,  MenuDisp },
    {46, 47,  0,  2,  MenuDisp },
    {47, 44,  0,  2,  MenuDisp },
    

};



static uint8 KeyFuncIndex=4;    //当前菜单号
static uint8 stepnum = 0;   //步长号

void MenuDisp(void)
{
    if( 4 > KeyFuncIndex )
    {
        LCD_Fill(0x00);
        LCD_P8x16Str(0,0," Parameter");
        LCD_P8x16Str(0,2," Setting");
        LCD_P8x16Str(0,4," Display");
        LCD_P8x16Str(0,6," Preserved3");
        LCD_P8x16Str(0,KeyFuncIndex*2,"?");

    }
    else if( 4 <= KeyFuncIndex && 8 > KeyFuncIndex )
    {
        LCD_Fill(0x00);
        LCD_P8x16Str(0,0," Angle");
        LCD_P8x16Str(0,2," Speed");
        LCD_P8x16Str(0,4," Direction");
        LCD_P8x16Str(0,6," Other");
        LCD_P8x16Str(0,(KeyFuncIndex-4)*2,"?");
    }
    else if( 8<= KeyFuncIndex && 12 > KeyFuncIndex )
    {
        LCD_Fill(0x00);
        Dis_Float(1,0,PID_Angle.Set,3);
        Dis_Float(1,2,PID_Angle.Kp,3);
        Dis_Float(1,4,PID_Angle.Ki,3);
        Dis_Float(1,6,PID_Angle.Kd,3);
        LCD_P8x16Str(0,(KeyFuncIndex-8)*2,"?");

    }
    else if( 12<= KeyFuncIndex && 16 > KeyFuncIndex )
    {
        LCD_Fill(0x00);
        Dis_Float(1,0,PID_Speed.Set,3);
        Dis_Float(1,2,PID_Speed.Kp,3);
        Dis_Float(1,4,PID_Speed.Ki,3);
        Dis_Float(1,6,PID_Speed.Kd,3);
        LCD_P8x16Str(0,(KeyFuncIndex-12)*2,"?");
    }
    else if( 16<= KeyFuncIndex && 20 > KeyFuncIndex )
    {
        LCD_Fill(0x00);
        Dis_Float(1,0,PID_Dir.Set,3);
        Dis_Float(1,2,PID_Dir.Kp,3);
        Dis_Float(1,4,PID_Dir.Ki,3);
        Dis_Float(1,6,PID_Dir.Kd,3);
        LCD_P8x16Str(0,(KeyFuncIndex-16)*2,"?");

    }
    else if( 20<= KeyFuncIndex && 24 > KeyFuncIndex )
    {
        LCD_Fill(0x00);
        LCD_P8x16Str(0,0," g_fApha");


        LCD_P8x16Str(0,(KeyFuncIndex-20)*2,"?");        

    }
    //----------显示参数修改------------//
    else if( 24<= KeyFuncIndex && 40 > KeyFuncIndex )
    {
        LCD_Fill(0x00);
        Dis_Float(1, 0, *(ParamTbl[KeyFuncIndex-24].param),3);
        Dis_Float(1, 2, ParamTbl[KeyFuncIndex-24].step[stepnum],3);

    }
    //-----------------setting-------------//
    else if( 40<= KeyFuncIndex && 44 > KeyFuncIndex )
    {
        if( 40 == KeyFuncIndex )
        {
            LCD_Fill(0x00);
            Dis_Float(1, 0, *(ParamTbl[12].param),2);
            Dis_Float(1, 2, ParamTbl[12].step[stepnum],2);
            
        }
    }
    else if( 44<= KeyFuncIndex && 48 > KeyFuncIndex )
        if( 44 == KeyFuncIndex )
        {
            LCD_Fill(0x00);
            Dis_Float(0,0,SensorData_f0,2);
            Dis_Float(0,2,SensorData_f1,2);
            Dis_Float(0,4,SensorData_f2,2);
            Dis_Float(0,6,g_fCarAngle,2);
        }

}


void KeyDeal(void)
{

    KEY_MSG_t keymsg;
    if(get_key_msg(&keymsg) == 1)
    {
        if( KEY_DOWN == keymsg.status )
        {    
            switch(keymsg.key)
            {
                case KEY_A:
                //---------菜单切换------------//
                    if( 24 > KeyFuncIndex)
                        KeyFuncIndex = KeyTbl[KeyFuncIndex].KeyA;
                //---------参数步长修改------------//
                    else if( 24 <= KeyFuncIndex && 40 > KeyFuncIndex)
                        stepnum = (stepnum + 1) % STEP;
                //-----------------setting-------------//
                    else if( 40<= KeyFuncIndex && 44 > KeyFuncIndex )    
                        if( 40 == KeyFuncIndex )
                            stepnum = (stepnum + 1) % STEP;
                        
                    
                    break;

                case KEY_B:
                //----------菜单确定-----------//
                    if( 24 > KeyFuncIndex)
                        KeyFuncIndex = KeyTbl[KeyFuncIndex].KeyB;                    
                //---------参数修改执行------------//
                    else if( 24 <= KeyFuncIndex && 40 > KeyFuncIndex)
                        *(ParamTbl[KeyFuncIndex-24].param) += ParamTbl[KeyFuncIndex-24].step[stepnum]; 
                    else if( 40 == KeyFuncIndex )
                        *(ParamTbl[12].param) += ParamTbl[12].step[stepnum]; 
                    
                    break;

                case KEY_C:
                //----------菜单返回-----------//
                        KeyFuncIndex = KeyTbl[KeyFuncIndex].KeyC;                

                    break;
                case KEY_D:
                //--------------参数保存-----------------//    
                    ParameterSave();

                    break;
                case KEY_E:
                //---------------执行空操作刷新OLED防花屏-----------//
                    break;

                default:break;
            }

        }
//----------执行结构体对应函数-------------//        
        ( KeyTbl[KeyFuncIndex].CurrentOperate )();
        

    }
    
    
}

/*****************************
    读取flash内保存的参数

******************************/
uint8 ParameterRead(void)
{

    if( 0xffffffff == flash_read(SECTOR_NUM,  4, float) )
        return 0;
        
    * ParamTbl[0].param  = flash_read(SECTOR_NUM,  0, float);
    * ParamTbl[1].param  = flash_read(SECTOR_NUM,  4, float);
    * ParamTbl[2].param  = flash_read(SECTOR_NUM,  8, float);
    * ParamTbl[3].param  = flash_read(SECTOR_NUM, 12, float);

    * ParamTbl[4].param  = flash_read(SECTOR_NUM, 16, float);
    * ParamTbl[5].param  = flash_read(SECTOR_NUM, 20, float);
    * ParamTbl[6].param  = flash_read(SECTOR_NUM, 24, float);
    * ParamTbl[7].param  = flash_read(SECTOR_NUM, 28, float);
    
    * ParamTbl[8].param  = flash_read(SECTOR_NUM, 32, float);
    * ParamTbl[9].param  = flash_read(SECTOR_NUM, 36, float);
    * ParamTbl[10].param  = flash_read(SECTOR_NUM, 40, float);
    * ParamTbl[11].param  = flash_read(SECTOR_NUM, 44, float);  
    
    * ParamTbl[12].param  = flash_read(SECTOR_NUM, 48, float);
    
    return 1;
        
}

/***********************************
    保存参数flash
    flash_write保存整型数据
    flash_write_float保存浮点型数据

**********************************/
void ParameterSave(void)
{ 
    flash_erase_sector(SECTOR_NUM);
    
    flash_write_float(SECTOR_NUM,  0 , * ParamTbl[0].param   );      //251*2*1024+0
    flash_write_float(SECTOR_NUM,  4 , * ParamTbl[1].param   );
    flash_write_float(SECTOR_NUM,  8 , * ParamTbl[2].param   );
    flash_write_float(SECTOR_NUM, 12 , * ParamTbl[3].param   );
    
    flash_write_float(SECTOR_NUM, 16 , * ParamTbl[4].param   );
    flash_write_float(SECTOR_NUM, 20 , * ParamTbl[5].param   );
    flash_write_float(SECTOR_NUM, 24 , * ParamTbl[6].param   );
    flash_write_float(SECTOR_NUM, 28 , * ParamTbl[7].param   );

    flash_write_float(SECTOR_NUM, 32 , * ParamTbl[8].param   );
    flash_write_float(SECTOR_NUM, 36 , * ParamTbl[9].param   );
    flash_write_float(SECTOR_NUM, 40 , * ParamTbl[10].param   );
    flash_write_float(SECTOR_NUM, 44 , * ParamTbl[11].param   );

    flash_write_float(SECTOR_NUM, 48 , * ParamTbl[12].param   );
}


