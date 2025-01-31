#include "wifi_app.h"
// 观察wifi连接状态的语句

#define WIFI_TASK_PRIO     15          /* 任务优先级 */
#define WIFI_STK_SIZE      5*1024      /* 任务堆栈大小 */
static TaskHandle_t WIFI_Task_Handler;             /* 任务句柄 */


#define WIFI_INIT_TASK_PRIO     15          /* 任务优先级 */
#define WIFI_INIT_STK_SIZE      15*1024      /* 任务堆栈大小 */
static TaskHandle_t WIFI_INIT_Task_Handler;             /* 任务句柄 */


static volatile uint8_t wifi_conncet_status = 1;/* 0表示连接上了wifi热点 */
static volatile uint8_t wifi_init_status = 1;//0表示wifi固件初始化完毕可以进行连接了
const char *ssid = "Lighting";
const char *key = "lcs123456";


QueueHandle_t wifi_recvmsg_queue = NULL;     	/* 通过队列发送wifi底层相关信息*/
struct kfifo * recv_fifo = NULL;							// 接收88w8801的消息
struct kfifo * send_fifo = NULL;							// 接收来自lwip的消息


static void wifi_task(void *pvParameters);

void wifi_init_result(uint8_t status)
{
		wifi_init_status = status;
    printf("WIFI_ON %d\r\n",status);

}
void wifi_scan_result(uint8_t *ssid,uint8_t rssi,uint8_t channel,uint8_t *encryption_mode)
{
    printf("SSID: %s\n", ssid);
    printf("RSSI: %d\n", rssi);
    printf("Channel: %d\n", channel);
    printf("Encryption Mode: %s\n", encryption_mode);
}

void wifi_connect_result(uint8_t status)
{
	    wifi_conncet_status = status;
    printf("WIFI_CONNECT %d\r\n",status);
}

void wifi_start_ap_result(uint8_t status)
{
    printf("WIFI_START_AP %d\r\n",status);
}


void wifi_ap_connect_result(uint8_t *name,uint8_t *mac,uint8_t *ip)
{
    uint8_t mac_str[20] = {0};
    sprintf((char*)mac_str,"%02x:%02x:%02x:%02x:%02x:%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    printf("wifi_ap_connect_result\n");
    printf("name %s\n",name);
    printf("mac %s\n",mac_str);
    printf("ip %s\n",ip);

}

void wifi_ap_disconnect_result(uint8_t *mac)
{
    uint8_t mac_str[20] = {0};

    printf("wifi_ap_disconnect_result\n");
    printf("mac %s\n",mac_str);
    sprintf((char*)mac_str,"%02x:%02x:%02x:%02x:%02x:%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

// 说白了就是说明初始化状态的一个指针
static wifi_cb_t wifi_cb =
{
    wifi_init_result,
    wifi_scan_result,
    wifi_connect_result,
    wifi_start_ap_result,
    NULL,
    wifi_ap_connect_result,
    wifi_ap_disconnect_result,
};


static void wifi_init_task(void *pvParameters)
{
		wifi_recvmsg_queue = xQueueCreate(16,sizeof(wifi_msg)); 		// wifi线程用来处理消息 其他线程进行消息投递
		printf("wifi_recvmsg_queue 消息队列初始化完成\r\n");
		configASSERT(wifi_recvmsg_queue != NULL);	
		recv_fifo	 = kfifo_alloc(8*1024);													// 分配fifo 供lwip和tcp沟通使用
		send_fifo = kfifo_alloc(8* 1024);
		marvel88w8801_init(&wifi_cb);	
		uint8_t ret = 0;
		uint32_t start_time = 0;
    uint32_t timeout = 5000; // 超时时间设定为 5000ms（5秒）
	
		while(1){
			 if (WiFi_LowLevel_GetITStatus(1))
			 {
					printf("主循环检测到中断了\r\n");
					ret = mrvl88w8801_process_packet(); //现在处理失败了看看还会不会读到的
			 }
			 else if(!wifi_init_status)
			 {
					wifi_init_status = 1;
					wifi_connect_ap(ssid,8,key,9);// 如果收到了就进行连接
			 }
			 // 如果连接上就跳出
			 if(!wifi_conncet_status)
			 {
					break;
			 }
			 if(ret == 2) 
			 {
				  // 获取当前时间作为开始时间
            if (start_time == 0)
            {
                start_time = HAL_GetTick(); // HAL_GetTick 获取当前系统时间（以毫秒计）
                printf("开始超时计时...\r\n");
            }

            // 检查是否超时
            if ((HAL_GetTick() - start_time) > timeout)
            {
                printf("处理超时，执行超时处理逻辑...\r\n");
                start_time = 0; // 重置计时器
                mrvl88w8801_process_packet_timeout();        // 重置状态，避免重复触发超时
                continue;       // 跳过当前循环，执行超时后的处理逻辑
            }
			 }
			 else
        {
            // 如果 ret != 2，重置计时器
            start_time = 0;
        }
		};
		// 真正的和lwip相关的处理操作
		lwip_demo_init(LWIP_TASK_TYPE);		//初始化一个lwip任务用来测试
		xTaskCreate((TaskFunction_t )wifi_task,
                (const char*    )"wifi_task",
                (uint16_t       )WIFI_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )WIFI_TASK_PRIO,
                (TaskHandle_t*  )&WIFI_Task_Handler);	
				
		vTaskDelete(WIFI_INIT_Task_Handler); /* 删除自身任务 */
}
// 核心线程 用来处理wifi的连接情况和其他的消息
static void wifi_task(void *pvParameters)
{
		// 还没想好怎么写
		wifi_msg wifi_status;
		int32_t init_status = -1;
		uint8_t * buffer;
		sd_irq_exit_enable();
		extern TaskHandle_t LWIP_Task_Handler;
		xTaskNotifyGive(LWIP_Task_Handler);		//初始化一个lwip任务用来测试
		printf("启动外部中断,进入真正的数据处理任务\r\n");
		// 这里其实保险起见也可以加一个time_out
    while(1)
    {
        // 从队列中接收消息
				// 数字越小网速越快
        if(xQueueReceive(wifi_recvmsg_queue, &wifi_status, 20) == pdPASS)
        {
						// 不同的状态调用不同的处理函数
            switch(wifi_status.wifi_status)
						{
							case WIFI_IRQ:
								//printf("进行中断寄存器的处理\r\n");
								  
								mrvl88w8801_handle_interrupt(&init_status);		// 处理中断 主要是清除寄存器什么的
  	
								configASSERT(init_status != -1);
								if(init_status <= 3 && init_status > 0)		// 触发中断的 上行和下行 然后清空就行
								{
										//printf("读取到的中断的状态%d\r\n",init_status);
										wifi_status.wifi_status = WIFI_RECV;
										wifi_status.init_status = init_status;
										xQueueSendToFront(wifi_recvmsg_queue,&wifi_status,portMAX_DELAY);	//意思是要处理接受到的消息了
								}
//								else
//									printf("发生了读取错误 读到的int_status是%d\r\n",init_status);
								break;
							case WIFI_RECV:
//								printf("解析接受的数据 长度是%d\r\n",wifi_status.init_status);	 
					
									rtos_process_rxirq_packet(wifi_status.init_status);   //此时的init_status就是接收到的状态	
									// 最后有一个把数据写入fifo里面并通知lwip
									break;
							case WIFI_SEND:
									// lwip发过来的信息 这边是负责从send_fifo中读出来然后发送出去
									//printf("准备发送数据了 长度是%d\r\n",wifi_status.init_status);
									buffer = mrvl88w8801_get_send_data_buf();	//一点小小的准备工作
									kfifo_get(send_fifo,buffer,wifi_status.init_status); // 这里的init_status就是要发送的长度了
									
									mrvl88w8801_send_data(buffer,wifi_status.init_status);	// 把数据发出去
									break;
							default:
									break;
						}
        } else
				{
					// 这个东西咋说呢额 他内个中断寄存器中断触发机制很抽象
					// 比如ping命令发的包不大他可能就不会出发中断--好像是奥 所以就超时检查一下 此时越过中断寄存器
					 //printf("超时啦\r\n");
					mrvl88w8801_process_packet_timeout(); 
				}
		}
}

void wifi_init()
{
		xTaskCreate((TaskFunction_t )wifi_init_task,
                (const char*    )"wifi_init_task",
                (uint16_t       )WIFI_INIT_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )WIFI_INIT_TASK_PRIO,
                (TaskHandle_t*  )&WIFI_INIT_Task_Handler);	
		
}
void EXTI15_10_IRQHandler(void)
{
	
		wifi_msg wifi_status;
    // 检查是否是PC13引脚触发的中断
    if(__HAL_GPIO_EXTI_GET_IT(SD_IRQ_PIN) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(SD_IRQ_PIN); // 清除中断标志
				//printf("在RTOS EXTI15_10中读取到中断数据了\r\n");
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				// 发送消息过去看看是什么
				wifi_status.wifi_status = WIFI_IRQ;
				xQueueSendFromISR(wifi_recvmsg_queue, &wifi_status, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			   // 禁用中断--看看是不是重复触发的问题
        //HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
			}
}


