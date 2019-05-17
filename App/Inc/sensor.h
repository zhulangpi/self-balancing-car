#ifndef  _SENSOR_H_
#define  _SENSOR_H_

/**************************************************
        引脚、模块资源配置
***************************************************/
#define SENSOR_ADC0     ADC0_SE8    //PTB0
#define SENSOR_ADC1     ADC0_SE9    //PTB1
#define SENSOR_ADC2     ADC0_SE12   //PTB2
#define SENSOR_ADC3     ADC1_SE10   //PTB4
#define SENSOR_ADC4     ADC1_SE11   //PTB5
#define SENSOR_ADC5     ADC1_SE12   //PTB6



#define SensorData_f0   SensorData[5] //RW0 
#define SensorData_f1   SensorData[3] //RW2 
#define SensorData_s0   SensorData[2] //RW3 
#define SensorData_s1   SensorData[1] //RW4
#define SensorData_f2   SensorData[0] //RW5
//#define SensorData_f0   SensorData[5] 



#define DirPD_x 6
#define DirPD_y 7

extern float DirDymaticPDf[DirPD_x][DirPD_y][2];


/**************************************************
        类型定义
***************************************************/
typedef enum    
{             
  SENSOR_ADC0_e = 0,
  SENSOR_ADC1_e,
  SENSOR_ADC2_e,
  SENSOR_ADC3_e,
  SENSOR_ADC4_e,
  SENSOR_ADC5_e, 
}       SENSOR_ADC_e;


typedef enum
{
    normal_e = 0,
    atright_e,//在右弯
    atleft_e,
    lostright_e,//右弯丢线
    lostleft_e,
    
    
}   LostFlag_e;

typedef enum
{
    Straightaway_e = 0,
    LRoad_e,
    Ramp_e,
    
}   RoadType_e;




/*************************************************************************
        宏定义函数
*************************************************************************/
//**********************清除Sensor数据**********************
#define SensorClearData()\
  {\
    SensorData[0] = 0;SensorData[1] = 0;SensorData[2] = 0;\
    SensorData[3] = 0;SensorData[4] = 0;SensorData[5] = 0;\
  }
  
#define SensorClearSum()\
{\
  SensorSum[0] = 0;SensorSum[1] = 0;SensorSum[2] = 0;\
  SensorSum[3] = 0;SensorSum[4] = 0;SensorSum[5] = 0;\
}

#define SensorClearsnum()\
  {\
    snum[0]=0; snum[1]=0; snum[2]=0;\
    snum[3]=0; snum[4]=0; snum[5]=0;\
  }

//**********************采一次ADC**********************
#define SENSOR_GET(num)\
    {\
      temp = adc_once(SENSOR_ADC##num, ADC_12bit);\
      SensorData[SENSOR_ADC##num##_e] += temp;\
    }


//---------------------变量声明----------------------//   

extern float SensorData[6];
extern float g_fSite;
extern float g_fSitesole;

extern float g_fDirPVal;

extern RoadType_e RoadType;
extern LostFlag_e lostflag;

extern int16 lostnum;                //连续丢线时间
extern uint16 RoadCount;
extern uint16 RampCount;





//---------------------函数声明----------------------//

extern void Sensor_Init(void);
extern void Sensor_Get_avg(void);
extern void Sensor_Getfast_avg(void);
extern void Sensor_Getfast_avg2(void);
extern void Sensor_Calc_Site(void);
extern void Control_JudgeSite();
extern void Control();
extern void Calc_DirP(void);
extern float low_pass_filter(float);
extern void Control_JudgeSite(void);


#endif