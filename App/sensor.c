#include  "include.h"


float SensorData[6]={0};

const ADCn_Ch_e SensorCH[6]={SENSOR_ADC0, SENSOR_ADC1, SENSOR_ADC2, SENSOR_ADC3, SENSOR_ADC4, SENSOR_ADC5 };

float g_fSite = 0;
float g_fSitesole = 0;

float LostValue,LastValue;//丢线的区分度
float ADCMax = 0,ADCMin = 0;

RoadType_e RoadType = Straightaway_e;
LostFlag_e lostflag = normal_e;

int16 lostnum=0;                //连续丢线时间
uint16 RoadCount=0;
uint16 RampCount = 0;
float g_fDirPVal = 0;

uint8 Control_IsRamp(void);



/************************横电感拟合**************************/
void Sensor_Calc_Site(void)
{
    float left=SensorData_f0, mid=SensorData_f1, right=SensorData_f2;
    float site0=0,site1=0,site2=0,temp=0,k0=0,k1=0,k2=0;
    //static float lastsite = 0;
    if(((0==left) + (0==mid) + (0==right)) > 2)
        return;
  //*****site0*****
    if(left == right)
        site0 = 0;
    else
    {
        temp=(left-right)/(left+right);
        site0 = 77*powf(temp,5)-33.28*powf(temp,4)-5.171*powf(temp,3)+2.99*powf(temp,2)+25.01*temp+28.05;
    }

  //*****site1*****

    if(left == mid)
        site1 = 0;
    else
    {
        temp=(left-mid)/(mid+left);
        site1 = 998.4*powf(temp,5)+44.5*powf(temp,4)-64.77*powf(temp,3)+18.33*powf(temp,2)+48.36*temp+40.91;


    }

   //*****site2*****

    if(mid == right)
        site2 = 0;
    else
    {
        temp=(mid-right)/(right+mid);
        site2 = -163.3*powf(temp,5)+190.4*powf(temp,4)+83.42*powf(temp,3)-72.87*powf(temp,2)+53.3*temp+15.88;
            
    }
    
    
#define CENTER0 (28.05f) 
#define CENTER1 (40.91f) 
#define CENTER2 (15.88f)

    //*****计算权重*****
    //离两电感中心越远权重越低 理论上若调校过中心处temp==0
    k0 = fabs(site0 - CENTER0);
    k1 = fabs(site1 - CENTER1);
    k2 = fabs(site2 - CENTER2);
    
#undef  CENTER0
#undef  CENTER1
#undef  CENTER2
    
#define THRESHOLD   10
    // 1/(site-center) == +-0.1 => site == center +- 10.0
    if(k0<THRESHOLD)
        k0 = 1;
    else
        k0 = THRESHOLD/k0;
    if(k1<THRESHOLD)
        k1 = 1;
    else
        k1 = THRESHOLD/k1;
    if(k2<THRESHOLD)
        k2 = 1;
    else
        k2 = THRESHOLD/k2;

#undef  THRESHOLD
    
#define MIDSITE  (30.0f)
    
    g_fSite= (k0*site1 + k1*site0 + k2*site2)/(k0+k1+k2);
    
    
    if(g_fSite>45)
        g_fSite = 45;
    if(g_fSite<15)
        g_fSite = 15;
    
    g_fSite -=  MIDSITE;

#undef MIDSITE
 
}

float g_fecfbyKalman;
/******************************* Control_JudgeEc *****************************/
void Control_JudgeEc(void)
{
    //******************************计算ec,ecbyKalman**********
    static float lastsite = 0 ,ecf = 0;
    
    ecf = (g_fSite-lastsite);
    lastsite = g_fSite;

#define ecKalman_Q        0.15
#define ecKalman_R        0.4
    static float pf=10,kgf=0.5;
    
    
//    if(ecf==ecf)
//    {
//        pf = pf + ecKalman_Q;//对应于预估值的协方差
//        kgf = pf / (pf + ecKalman_R);//kalman gain
//        if( kgf*(ecf-g_fecfbyKalman)>50)
//            g_fecfbyKalman = (g_fecfbyKalman + 50);
//        else if( kgf*(ecf-g_fecfbyKalman)<-50)
//            g_fecfbyKalman = (g_fecfbyKalman - 50);
//        else
//            g_fecfbyKalman = (g_fecfbyKalman + kgf * (ecf - g_fecfbyKalman));
//        pf = (1 - kgf) * pf;
//    }
    
    
    EASYFILTER(g_fecfbyKalman);
    
    g_fecfbyKalman = 200*ecf;
    
    if(g_fecfbyKalman > 500) g_fecfbyKalman=500;
    if(g_fecfbyKalman < -500) g_fecfbyKalman=-500;
    
#undef  ecKalman_Q
#undef  ecKalman_R

}



/********************************判断位置********************************

*判断丢线位置/给lostflag赋值
*保存丢线前g_fSitesole
*计算Site

****/
void Control_JudgeSite(void)
{

    ADCMax = MAX(SensorData_f0,SensorData_f1);
    ADCMax = MAX(ADCMax,SensorData_f2);


    if(ADCMax < 700)
    {
        if(lostflag == atright_e) lostflag = lostright_e;
        else if(lostflag == atleft_e) lostflag = lostleft_e;
        LostValue = LastValue;
    }
    else
    {
        LostValue = 0;
        if(ADCMax > 800 && SensorData_f1 > 700)
        {
            lostflag = (g_fSitesole<0)?atright_e:atleft_e;
            LastValue = g_fSitesole;
        }
        else
        {
            if(lostflag == lostright_e) lostflag = atright_e;
            else if(lostflag == lostleft_e) lostflag = atleft_e;
        }
    }
  //  if(ADCMax > 600)
    {
        Sensor_Calc_Site();
        //Sensor_Calc_Sitesole();
    }

}

/************************判断弯道***********************
*只修改了外部变量RoadType

*/
void Control_JudgeRoad(void)
{
    static RoadType_e laststate = Straightaway_e, lastRoadType = Straightaway_e;
    static uint16 num = 0;

    if(ADCMax > 750 && fabs(g_fSite)<15 )//非弯道，非丢线--直道
    {
        if(laststate != Straightaway_e)
        {
            num = 0;
            laststate = Straightaway_e;
        }
        else
            num++;
    }
    else
    {
        if(laststate != LRoad_e)
        {
            num = 0;
            laststate = LRoad_e;
        }
        else
            num++;
    }
  
    if(num>20 && laststate == Straightaway_e)
    {
        RoadType = Straightaway_e;
        if(lastRoadType != Straightaway_e)
        {
            RoadCount = 0;
        }
        RoadCount ++ ;
        lastRoadType = Straightaway_e;
    }
    else
    {
        RoadType = LRoad_e;
        if(lastRoadType != LRoad_e)
        {
            RoadCount = 0;
        }
        RoadCount ++ ;
        lastRoadType = LRoad_e;
    }
}

/***********************丢线保护**********************
*丢线一定时间返回 1
*/
int8 Control_LostProtect()
{
    static uint16 lostcount=0;//丢线计数

//******************************保护措施*******************
    if( (ADCMax<250) && (SensorData_s0<250) && (SensorData_s1<250))
    {
        if(lostcount<2000)
            lostcount++;
    }
    else
        lostcount = 0;

    if(PID_Speed.Set != 0 && lostcount > 800)
    {
        //出跑道刹车

    }

    if(lostcount > 350)//舵机不打角
        return 1;
    else
        return 0;
}




/*************************************************
*
*   计算直线Kp,Kd
**/

float const DirPD_xTf[DirPD_x]={-200,-100,-50,50,100,200};//g_fSite
float const DirPD_yTf[DirPD_y]={-400,-200,-100,0,100,200,400};//g_fGyroscopeAngleSpeed_Y
float DirDymaticPDf[DirPD_x][DirPD_y][2]=
    {
{{ 18, 1.33}, { 18, 1.33} , {18, 1.33} , {18, 1.33} , {18, 1.33} , {18, 1.33} , {18, 1.33} },

{{ 16, 1.33}, { 16, 1.33} , {16, 1.33} , {16, 1.33} , {16, 1.33} , {16, 1.33} , {16, 1.33} },

{{ 12, 1.33}, { 12, 1.33} , {12, 1.33} , {12, 1.33} , {12, 1.33} , {12, 1.33} , {12, 1.33} },

{{ 12, 1.33},  { 12, 1.33} , {12, 1.33} , {12, 1.33} , {12, 1.33} , {12, 1.33} , {12, 1.33} },

{ {16, 1.33}, {16, 1.33} , {16, 1.33} , {16, 1.33} , {16, 1.33} , {16, 1.33} , {16, 1.33} },

{{ 18, 1.33}, { 18, 1.33} , {18, 1.33} , {18, 1.33} , {18, 1.33} , {18, 1.33} , {18, 1.33} }
};

void Dir_GetPDf(void)
{
    uint8 x = 0,y = 0;
    float ke = 0,kec = 0;
    ke = g_fSite;
    kec = g_fGyroscopeAngleSpeed_Y;

//查横纵坐标
    for (x=0;x<DirPD_x-1;x++)
    {
        if(ke <= DirPD_xTf[x])
        {
            break;
        }
    }
    for (y=0;y<DirPD_y-1;y++)
    {
        if(kec <= DirPD_yTf[y])
        {
            break;
        }
    }
    if(x>=1&&(DirPD_xTf[x]-ke)>(ke - DirPD_xTf[x-1]))
        x--;
    if(y>0&&(DirPD_yTf[y]-kec)>(kec - DirPD_yTf[y-1]))
        y--;
    
   // probe0 = x;
    //probe1 = y;
    PID_Dir.Kp = DirDymaticPDf[x][y][0];
    PID_Dir.Kd = DirDymaticPDf[x][y][1];
    
}




extern uint16 stopflag;
/***************************** Control ********************************/
void Control(void)
{

    //if( stopflag > 300 )        ///*g_fGyroscopeAngleSpeed_Y<1000 &&*/ g_fCarAngle < PID_Angle.Set + 50 && g_fCarAngle > PID_Angle.Set - 50)//坡道
     //   Control_IsRamp();
    //else
        //Control_JudgeRoad();
        g_fDirPVal = g_fSite - PID_Dir.Set;

    

}//Control end



extern float probe0,probe1,probe2;
/************************
    5ms调用一次
************************/
uint8 Control_IsRamp(void)
{
    static uint8 RampFlag = 0;
    static uint16 RampCount = 0;
    //static float temp =0;
//  if(Finflag == 0 || Speed_std == 0)
//    return 0;
    
    
    switch( RampFlag )
    {
        case 0://坡道?
            RampCount = 0;
            if(ADCMax > 2000)
            {
                RampFlag = 1;
            }
        
            break;
            
        case 1://前排入坡道
            if(RampCount > 10)
            {
                RampFlag = 2;
                RampCount = 0;
                //temp = PID_Angle.Set;
            }
            else if(ADCMax > 2000 )
                RampCount++;
            else
                RampFlag = 0;
            
            break;
            
        case 2://前排确实入坡道，处理
            if( RampCount > 50 )//60
            {    
                RampFlag = 3;
                RampCount = 0;

            }
            else//处理
            {
                RampCount++;
            }
            break;
        
        case 3://要下坡了
            if(RampCount > 60)//70
            {
                RampFlag = 4;
                RampCount = 0;
            }
            else //if(ADCMax < 800)
            {
                RampCount++;
            }

            break;
        
        case 4://下坡处理
            if( RampCount > 60 )//70
            {    
                RampFlag = 5;
                RampCount = 0;
            }
            else//处理
            {
                RampCount++;
                //if( RampCount > 30 && ADCMax >1100 )
                  //  RampFlag = 5;
            }
            break;
        
        case 5:
        
            RampFlag = 0;

            break;
        
        default:

            break;
    }
    

    //probe0 = RampFlag;
    if( RampFlag == 2 || RampFlag== 4 )//|| RampFlag== 3)
         g_fDirPVal = 0;
    else
         g_fDirPVal = g_fSite - PID_Dir.Set;
    
    
    return RampFlag;
}












/********************************** Sensor_Init ******************************/
void Sensor_Init(void)
{
  adc_init(SENSOR_ADC0); //PTB0
  adc_init(SENSOR_ADC1); //PTB1
  adc_init(SENSOR_ADC2); //PTB2
  adc_init(SENSOR_ADC3); //PTB4
  adc_init(SENSOR_ADC4); //PTB5
  adc_init(SENSOR_ADC5); //PTB6

  port_init_NoALT (PTB0, PF);
  port_init_NoALT (PTB1, PF);
  port_init_NoALT (PTB2, PF);
  port_init_NoALT (PTB4, PF);
  port_init_NoALT (PTB5, PF);
  port_init_NoALT (PTB6, PF);

  SensorClearData();
}

/****************************** 采集ADC，并进行平均滤波（流水操作） ***********/
void Sensor_Get_avg(void)
{ 
  uint16 temp=0;
  uint8 i;
  
  SensorClearData();

#define Times   2//采样次数  
  for(i=Times;i>0;i--)
  {
    SENSOR_GET(0);
    SENSOR_GET(1);
    SENSOR_GET(2);
    SENSOR_GET(3);
    SENSOR_GET(4);
    SENSOR_GET(5);
  }
  for(i=0;i<6;i++)
  {
    SensorData[i] = SensorData[i]/Times;
  }
#undef Times
}

/****************************** 采集ADC，并行操作 *****************************/
void Sensor_Getfast_avg(void)
{   
#define Times   5//采样次数  
    uint8 flag0=0,flag1=0,i=0;
    uint16 count0=0,count1=0;

    SensorClearData();
    //开始第1次采ADC0
    adc_start(SensorCH[0], ADC_12bit);
    //开始第1次采ADC1
    adc_start(SensorCH[3], ADC_12bit);

    while(0==flag0 || 0==flag1)
    {
        //*********通道0************
        if (0==flag0 && ( ADC_SC1_REG(ADC0_BASE_PTR, 0 ) & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK)//ADC0完成
        {
            SensorData[count0%3] += ADC_R_REG(ADC0_BASE_PTR, 0);
            ADC_SC1_REG(ADC0_BASE_PTR, 0) &= ~ADC_SC1_COCO_MASK;
            count0++;
            if(count0 >= 3*Times)
                flag0=1;
            else
            {
                adc_start(SensorCH[count0%3], ADC_12bit);
            }
        }
        //*********通道1************
        if (0==flag1 && ( ADC_SC1_REG(ADC1_BASE_PTR, 0 ) & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK)//ADC1完成
        {
            SensorData[(count1%3)+3] += ADC_R_REG(ADC1_BASE_PTR, 0);
            ADC_SC1_REG(ADC1_BASE_PTR, 0) &= ~ADC_SC1_COCO_MASK;
            count1++;
            if(count1 >= 3*Times)
                flag1=1;
            else
            {
                adc_start(SensorCH[(count1%3)+3], ADC_12bit);
            }
        }
    }
    
    for(i=0;i<6;i++)
    {
        SensorData[i] = SensorData[i]/Times;
        
    }

#undef Times

}



void Sensor_Getfast_avg2(void)
{   
#define Times   10//采样次数  
    uint8 flag0=0,flag1=0;
    uint16 count0=0,count1=0;

    SensorData[0] = 0;
    SensorData[3] = 0;
    SensorData[5] = 0;
    //开始第1次采ADC0
    adc_start(SensorCH[0], ADC_12bit);
    //开始第1次采ADC1
    adc_start(SensorCH[3], ADC_12bit);

    while(0==flag0 || 0==flag1)
    {
        //*********通道0************
        if (0==flag0 && ( ADC_SC1_REG(ADC0_BASE_PTR, 0 ) & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK)//ADC0完成
        {
            SensorData[0] += ADC_R_REG(ADC0_BASE_PTR, 0);
            ADC_SC1_REG(ADC0_BASE_PTR, 0) &= ~ADC_SC1_COCO_MASK;
            count0++;
            if(count0 >= Times)
                flag0=1;
            else
                adc_start(SensorCH[0], ADC_12bit);
        }
        //*********通道1************
        if (0==flag1 && ( ADC_SC1_REG(ADC1_BASE_PTR, 0 ) & ADC_SC1_COCO_MASK ) == ADC_SC1_COCO_MASK)//ADC1完成
        {
            if(count1%2)
                SensorData[3] += ADC_R_REG(ADC1_BASE_PTR, 0);
            else
                SensorData[5] += ADC_R_REG(ADC1_BASE_PTR, 0);
            
            ADC_SC1_REG(ADC1_BASE_PTR, 0) &= ~ADC_SC1_COCO_MASK;
            count1++;
            if(count1 >= 2*Times)
                flag1=1;
            else
            {
                if( count1%2 )
                    adc_start(SensorCH[3], ADC_12bit);
                else
                    adc_start(SensorCH[5], ADC_12bit);
            }
        }
    }
    
    SensorData[0] = SensorData[0]/Times;
    SensorData[3] = SensorData[3]/Times;
    SensorData[5] = SensorData[5]/Times;
    
#undef Times

}



#define DEPTH 30

float low_pass_filter(float IN)
{
    static float buff1[DEPTH]={0};//数组长度自己定义------------影响滤波器截止频率和滤波器增益
    static float buff2[DEPTH]={0};//长度自己定义
    static float buff3[DEPTH]={0};//长度自己定义
    static float y1=0, y2=0, y3=0;
    static int p=0;
    float OUT,x;

    x=IN ;

    y1 += x - buff1[p];
    buff1[p] = x;
    y2 += y1 - buff2[p];
    buff2[p] = y1;
    y3 += y2 - buff3[p];
    buff3[p] = y2;

    if(++p>9)
    p = 0;

    OUT = y3;
    return OUT;
}





