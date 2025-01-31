/**
 ****************************************************************************************************
 * @file        freertos_demo.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-01-11
 * @brief       lwIP+FreeRTOS����ϵͳ��ֲ ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */
 
#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H



#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "./MALLOC/malloc.h"
#include "./BSP/LED/led.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/ringBuffer/ringbuffer.h"
#include "wifi_app.h"
#include "lwip_demo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"



void freertos_demo(void);   /* ����lwIP�������� */

#endif
