#ifndef __LWIP_DEMO_H
#define __LWIP_DEMO_H

#include "wifi_app.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/ringBuffer/ringbuffer.h"
#include "./BSP/LED/led.h"

#include "lwipopts.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/apps/lwiperf.h"
#include "ethernetif.h" 
#include "lwip/inet_chksum.h"

#include "lwip/tcp.h"
#include "lwip/sockets.h"

#include "FreeRTOS.h"
#include "task.h"

#include "./MALLOC/malloc.h"

// 0:²âËÙ 1:¿Í»§¶Ë 2:»¥ping
#define LWIP_TASK_TYPE  2


void lwip_demo_init(uint8_t task_type);
void lwip_test_speed_task(void *pvParameters);
void test_tcp_connect_task(void *pvParameters);
void test_ping_task(void *pvParameters);
#endif
