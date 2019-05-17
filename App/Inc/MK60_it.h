/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       MK60_it.h
 * @brief      山外K60 平台中断服务重定向头文件
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-26
 */


#ifndef __MK60_IT_H__
#define __MK60_IT_H__

/*                          重新定义中断向量表
 *  先取消默认的中断向量元素宏定义        #undef  VECTOR_xxx
 *  在重新定义到自己编写的中断函数      #define VECTOR_xxx    xxx_IRQHandler
 *  例如：
 *       #undef  VECTOR_003                         先取消映射到中断向量表里的中断函数地址宏定义
 *       #define VECTOR_003    HardFault_Handler    重新定义硬件上访中断服务函数
 */

//---------重定义中断----------//
#undef  VECTOR_069 
#define VECTOR_069      UART4_IRQHandler
   
#undef  VECTOR_084
#define VECTOR_084      PIT0_Isr

#undef  VECTOR_085
#define VECTOR_085      PIT1_Isr

#undef  VECTOR_107
#define VECTOR_107      PORTE_IRQHandler

#undef  VECTOR_103
#define VECTOR_103      PORTA_IRQHandler

#undef  VECTOR_101      
#define VECTOR_101      lptmr_hander



extern void UART4_IRQHandler(void);
extern void PIT1_Isr(void);
extern void PIT0_Isr(void);
extern void PORTE_IRQHandler();
extern void PORTA_IRQHandler();
extern void lptmr_hander();
#endif  //__MK60_IT_H__

