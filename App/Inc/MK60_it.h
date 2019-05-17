/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_it.h
 * @brief      ɽ��K60 ƽ̨�жϷ����ض���ͷ�ļ�
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */


#ifndef __MK60_IT_H__
#define __MK60_IT_H__

/*                          ���¶����ж�������
 *  ��ȡ��Ĭ�ϵ��ж�����Ԫ�غ궨��        #undef  VECTOR_xxx
 *  �����¶��嵽�Լ���д���жϺ���      #define VECTOR_xxx    xxx_IRQHandler
 *  ���磺
 *       #undef  VECTOR_003                         ��ȡ��ӳ�䵽�ж�����������жϺ�����ַ�궨��
 *       #define VECTOR_003    HardFault_Handler    ���¶���Ӳ���Ϸ��жϷ�����
 */

//---------�ض����ж�----------//
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

