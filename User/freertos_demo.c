/**
 ****************************************************************************************************
 * @file        freertos_demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-01-11
 * @brief       lwIP+FreeRTOS操作系统移植实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */
 
#include "freertos_demo.h"





/******************************************************************************************************/
/* START_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define START_TASK_PRIO         5           /* 任务优先级 */
#define START_STK_SIZE          128         /* 任务堆栈大小 */
TaskHandle_t StartTask_Handler;             /* 任务句柄 */
void start_task(void *pvParameters);        /* 任务函数 */

/* LWIP_DEMO 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */


/* LED_TASK 任务 配置
 * 包括: 任务句柄 任务优先级 堆栈大小 创建任务
 */
#define LED_TASK_PRIO           7         /* 任务优先级 */
#define LED_STK_SIZE            128         /* 任务堆栈大小 */
TaskHandle_t LEDTask_Handler;               /* 任务句柄 */
void led_task(void *pvParameters);          /* 任务函数 */


/**
 * @breif       freertos_demo
 * @param       无
 * @retval      无
 */
void freertos_demo(void)
{
    /* start_task任务 */
    xTaskCreate((TaskFunction_t )start_task,
                (const char *   )"start_task",
                (uint16_t       )START_STK_SIZE,
                (void *         )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t * )&StartTask_Handler);

    vTaskStartScheduler(); /* 开启任务调度 */
		printf("开始任务调度了\r\n");
}

/**
 * @brief       start_task
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void start_task(void *pvParameters)
{
    pvParameters = pvParameters;
	
    //想想该怎么写
	

    taskENTER_CRITICAL();          


    /* LED测试任务 */
    xTaskCreate((TaskFunction_t )led_task,
                (const char*    )"led_task",
                (uint16_t       )LED_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )LED_TASK_PRIO,
                (TaskHandle_t*  )&LEDTask_Handler);
		
    wifi_init();      
		lwip_demo_init(0);								
    vTaskDelete(StartTask_Handler); /* 删除开始任务 */
    taskEXIT_CRITICAL();            /* 退出临界区 */
    
}



/**
 * @brief       系统再运行
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void led_task(void *pvParameters)
{
    pvParameters = pvParameters;
    
    while (1)
    {
        LED1_TOGGLE();
				vTaskDelay(500);	
		}				
			
}



