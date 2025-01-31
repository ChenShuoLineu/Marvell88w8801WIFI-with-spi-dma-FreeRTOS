#include "wifi_app.h"
// �۲�wifi����״̬�����

#define WIFI_TASK_PRIO     15          /* �������ȼ� */
#define WIFI_STK_SIZE      5*1024      /* �����ջ��С */
static TaskHandle_t WIFI_Task_Handler;             /* ������ */


#define WIFI_INIT_TASK_PRIO     15          /* �������ȼ� */
#define WIFI_INIT_STK_SIZE      15*1024      /* �����ջ��С */
static TaskHandle_t WIFI_INIT_Task_Handler;             /* ������ */


static volatile uint8_t wifi_conncet_status = 1;/* 0��ʾ��������wifi�ȵ� */
static volatile uint8_t wifi_init_status = 1;//0��ʾwifi�̼���ʼ����Ͽ��Խ���������
const char *ssid = "Lighting";
const char *key = "lcs123456";


QueueHandle_t wifi_recvmsg_queue = NULL;     	/* ͨ�����з���wifi�ײ������Ϣ*/
struct kfifo * recv_fifo = NULL;							// ����88w8801����Ϣ
struct kfifo * send_fifo = NULL;							// ��������lwip����Ϣ


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

// ˵���˾���˵����ʼ��״̬��һ��ָ��
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
		wifi_recvmsg_queue = xQueueCreate(16,sizeof(wifi_msg)); 		// wifi�߳�����������Ϣ �����߳̽�����ϢͶ��
		printf("wifi_recvmsg_queue ��Ϣ���г�ʼ�����\r\n");
		configASSERT(wifi_recvmsg_queue != NULL);	
		recv_fifo	 = kfifo_alloc(8*1024);													// ����fifo ��lwip��tcp��ͨʹ��
		send_fifo = kfifo_alloc(8* 1024);
		marvel88w8801_init(&wifi_cb);	
		uint8_t ret = 0;
		uint32_t start_time = 0;
    uint32_t timeout = 5000; // ��ʱʱ���趨Ϊ 5000ms��5�룩
	
		while(1){
			 if (WiFi_LowLevel_GetITStatus(1))
			 {
					printf("��ѭ����⵽�ж���\r\n");
					ret = mrvl88w8801_process_packet(); //���ڴ���ʧ���˿������᲻�������
			 }
			 else if(!wifi_init_status)
			 {
					wifi_init_status = 1;
					wifi_connect_ap(ssid,8,key,9);// ����յ��˾ͽ�������
			 }
			 // ��������Ͼ�����
			 if(!wifi_conncet_status)
			 {
					break;
			 }
			 if(ret == 2) 
			 {
				  // ��ȡ��ǰʱ����Ϊ��ʼʱ��
            if (start_time == 0)
            {
                start_time = HAL_GetTick(); // HAL_GetTick ��ȡ��ǰϵͳʱ�䣨�Ժ���ƣ�
                printf("��ʼ��ʱ��ʱ...\r\n");
            }

            // ����Ƿ�ʱ
            if ((HAL_GetTick() - start_time) > timeout)
            {
                printf("����ʱ��ִ�г�ʱ�����߼�...\r\n");
                start_time = 0; // ���ü�ʱ��
                mrvl88w8801_process_packet_timeout();        // ����״̬�������ظ�������ʱ
                continue;       // ������ǰѭ����ִ�г�ʱ��Ĵ����߼�
            }
			 }
			 else
        {
            // ��� ret != 2�����ü�ʱ��
            start_time = 0;
        }
		};
		// �����ĺ�lwip��صĴ������
		lwip_demo_init(LWIP_TASK_TYPE);		//��ʼ��һ��lwip������������
		xTaskCreate((TaskFunction_t )wifi_task,
                (const char*    )"wifi_task",
                (uint16_t       )WIFI_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )WIFI_TASK_PRIO,
                (TaskHandle_t*  )&WIFI_Task_Handler);	
				
		vTaskDelete(WIFI_INIT_Task_Handler); /* ɾ���������� */
}
// �����߳� ��������wifi�������������������Ϣ
static void wifi_task(void *pvParameters)
{
		// ��û�����ôд
		wifi_msg wifi_status;
		int32_t init_status = -1;
		uint8_t * buffer;
		sd_irq_exit_enable();
		extern TaskHandle_t LWIP_Task_Handler;
		xTaskNotifyGive(LWIP_Task_Handler);		//��ʼ��һ��lwip������������
		printf("�����ⲿ�ж�,�������������ݴ�������\r\n");
		// ������ʵ�������Ҳ���Լ�һ��time_out
    while(1)
    {
        // �Ӷ����н�����Ϣ
				// ����ԽС����Խ��
        if(xQueueReceive(wifi_recvmsg_queue, &wifi_status, 20) == pdPASS)
        {
						// ��ͬ��״̬���ò�ͬ�Ĵ�����
            switch(wifi_status.wifi_status)
						{
							case WIFI_IRQ:
								//printf("�����жϼĴ����Ĵ���\r\n");
								  
								mrvl88w8801_handle_interrupt(&init_status);		// �����ж� ��Ҫ������Ĵ���ʲô��
  	
								configASSERT(init_status != -1);
								if(init_status <= 3 && init_status > 0)		// �����жϵ� ���к����� Ȼ����վ���
								{
										//printf("��ȡ�����жϵ�״̬%d\r\n",init_status);
										wifi_status.wifi_status = WIFI_RECV;
										wifi_status.init_status = init_status;
										xQueueSendToFront(wifi_recvmsg_queue,&wifi_status,portMAX_DELAY);	//��˼��Ҫ������ܵ�����Ϣ��
								}
//								else
//									printf("�����˶�ȡ���� ������int_status��%d\r\n",init_status);
								break;
							case WIFI_RECV:
//								printf("�������ܵ����� ������%d\r\n",wifi_status.init_status);	 
					
									rtos_process_rxirq_packet(wifi_status.init_status);   //��ʱ��init_status���ǽ��յ���״̬	
									// �����һ��������д��fifo���沢֪ͨlwip
									break;
							case WIFI_SEND:
									// lwip����������Ϣ ����Ǹ����send_fifo�ж�����Ȼ���ͳ�ȥ
									//printf("׼������������ ������%d\r\n",wifi_status.init_status);
									buffer = mrvl88w8801_get_send_data_buf();	//һ��СС��׼������
									kfifo_get(send_fifo,buffer,wifi_status.init_status); // �����init_status����Ҫ���͵ĳ�����
									
									mrvl88w8801_send_data(buffer,wifi_status.init_status);	// �����ݷ���ȥ
									break;
							default:
									break;
						}
        } else
				{
					// �������զ˵�ض� ���ڸ��жϼĴ����жϴ������ƺܳ���
					// ����ping����İ����������ܾͲ�������ж�--�����ǰ� ���Ծͳ�ʱ���һ�� ��ʱԽ���жϼĴ���
					 //printf("��ʱ��\r\n");
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
    // ����Ƿ���PC13���Ŵ������ж�
    if(__HAL_GPIO_EXTI_GET_IT(SD_IRQ_PIN) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(SD_IRQ_PIN); // ����жϱ�־
				//printf("��RTOS EXTI15_10�ж�ȡ���ж�������\r\n");
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				// ������Ϣ��ȥ������ʲô
				wifi_status.wifi_status = WIFI_IRQ;
				xQueueSendFromISR(wifi_recvmsg_queue, &wifi_status, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			   // �����ж�--�����ǲ����ظ�����������
        //HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
			}
}


