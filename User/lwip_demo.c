#include "lwip_demo.h"


#define LWIP_DMEO_TASK_PRIO     8          			/* 任务优先级 */
#define LWIP_DMEO_STK_SIZE      5*1024      		/* 任务堆栈大小 */
#define LWIP_DEMO_TYPE					0								// 0:测速 1:本地客户端 2:互ping
TaskHandle_t LWIP_Task_Handler;             		/* 任务句柄 */


extern pmrvl88w8801_core_t pmrvl88w8801_core; 	//有大用的指针
extern __lwip_dev g_lwipdev;                   /* lwip控制结构体 */

void lwip_demo_init(uint8_t task_type)
{
		switch(task_type)
		{
			case 0:  //测速
				printf("初始化测速任务\r\n");
				xTaskCreate((TaskFunction_t )lwip_test_speed_task,
										(const char*    )"lwip_test_speed_task",
										(uint16_t       )LWIP_DMEO_STK_SIZE, 
										(void*          )NULL,
										(UBaseType_t    )LWIP_DMEO_TASK_PRIO,
										(TaskHandle_t*  )&LWIP_Task_Handler);
				break;
			case 1:
				printf("初始化客户端测试任务\r\n");
				xTaskCreate((TaskFunction_t )test_tcp_connect_task,
										(const char*    )"test_tcp_connect_task",
										(uint16_t       )LWIP_DMEO_STK_SIZE, 
										(void*          )NULL,
										(UBaseType_t    )LWIP_DMEO_TASK_PRIO,
										(TaskHandle_t*  )&LWIP_Task_Handler);
				break;
			case 2:
				printf("测试互ping任务\r\n");
				xTaskCreate((TaskFunction_t )test_ping_task,
										(const char*    )"test_ping_task",
										(uint16_t       )LWIP_DMEO_STK_SIZE, 
										(void*          )NULL,
										(UBaseType_t    )LWIP_DMEO_TASK_PRIO,
										(TaskHandle_t*  )&LWIP_Task_Handler);
				break;
			default:
				printf("Nothing Created\r\n");
				break;
		}
}
/* 报告状态 */
static const char *report_type_str[] = 
{
    "TCP_DONE_SERVER",             /* LWIPERF_TCP_DONE_SERVER,*/
    "TCP_DONE_CLIENT",             /* LWIPERF_TCP_DONE_CLIENT,*/
    "TCP_ABORTED_LOCAL",           /* LWIPERF_TCP_ABORTED_LOCAL, */
    "TCP_ABORTED_LOCAL_DATAERROR", /* LWIPERF_TCP_ABORTED_LOCAL_DATAERROR, */
    "TCP_ABORTED_LOCAL_TXERROR",   /* LWIPERF_TCP_ABORTED_LOCAL_TXERROR, */
    "TCP_ABORTED_REMOTE",          /* LWIPERF_TCP_ABORTED_REMOTE, */
    "UDP_STARTED",                 /* LWIPERF_UDP_STARTED, */
    "UDP_DONE",                    /* LWIPERF_UDP_DONE, */
    "UDP_ABORTED_LOCAL",           /* LWIPERF_UDP_ABORTED_LOCAL, */
    "UDP_ABORTED_REMOTE"           /* LWIPERF_UDP_ABORTED_REMOTE */
};

/* 当测试结束以后会调用此函数，此函数用来报告测试结果 */
/**
 * @brief       当测试结束以后会调用此函数，此函数用来报告测试结果
 * @param       无
 * @retval      无
 */
static void lwiperf_report(void *arg,
                           enum lwiperf_report_type report_type,
                           const ip_addr_t *local_addr,
                           u16_t local_port,
                           const ip_addr_t *remote_addr,
                           u16_t remote_port,
                           u32_t bytes_transferred,
                           u32_t ms_duration,
                           u32_t bandwidth_kbitpsec)
{
    printf("-------------------------------------------------\r\n");
  
    if (((int)report_type < (sizeof(report_type_str)/sizeof(report_type_str[0]))) && local_addr && remote_addr)
    {
        printf(" %s \r\n", report_type_str[report_type]);
        printf(" Local address : %u.%u.%u.%u ", ((u8_t *)local_addr)[0], ((u8_t *)local_addr)[1],
               ((u8_t *)local_addr)[2], ((u8_t *)local_addr)[3]);
        printf(" Port %d \r\n", local_port);
        printf(" Remote address : %u.%u.%u.%u ", ((u8_t *)remote_addr)[0], ((u8_t *)remote_addr)[1],
               ((u8_t *)remote_addr)[2], ((u8_t *)remote_addr)[3]);
        printf(" Port %d \r\n", remote_port);
        printf(" Bytes Transferred %d \r\n", (int)bytes_transferred);
        printf(" Duration (ms) %d \r\n", (int)ms_duration);
        printf(" Bandwidth (kbitpsec) %d \r\n", (int)bandwidth_kbitpsec);
        
   
    }
    else
    {
        printf(" IPERF Report error\r\n");
    }
}

// 用来测试网速的任务--此时是本地为服务器
void lwip_test_speed_task(void *pvParameters)
{
	
		uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		printf("准备开始初始化lwip测速任务了\r\n");
	   while (lwip_comm_init(pmrvl88w8801_core->mac_address) != 0)	//不extern也许更好
    {
				printf("初始化失败\r\n");
        delay_ms(500);
        LED1_TOGGLE();
			  delay_ms(500);
    }
	
    
		if(lwiperf_start_tcp_server_default(lwiperf_report,NULL))
    {
        printf("\r\n************************************************\r\n");
        printf(" IPERF Server example\r\n");
        printf("************************************************\r\n");
        printf(" IPv4 Address     : %u.%u.%u.%u\r\n", g_lwipdev.ip[0],g_lwipdev.ip[1],g_lwipdev.ip[2],g_lwipdev.ip[3]);
        printf(" IPv4 Subnet mask : %u.%u.%u.%u\r\n", g_lwipdev.netmask[0],g_lwipdev.netmask[1],g_lwipdev.netmask[2],g_lwipdev.netmask[3]);
        printf(" IPv4 Gateway     : %u.%u.%u.%u\r\n", g_lwipdev.gateway[0], g_lwipdev.gateway[1],g_lwipdev.gateway[2],g_lwipdev.gateway[3]);
        printf("************************************************\r\n");
    }
    else
    {
        printf("IPERF initialization failed!\r\n");
    }
    while(1)
    {
        vTaskDelay(5);
    }
}


// 以客户端连接远端服务器
void test_tcp_connect_task(void *pvParameters)
{
		uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		printf("准备开始初始化lwip任务了\r\n");
	   while (lwip_comm_init(pmrvl88w8801_core->mac_address) != 0)	//不extern用pvParamater更好
    {
				printf("初始化失败\r\n");
        delay_ms(500);
        LED1_TOGGLE();
			  delay_ms(500);
    }
		
		struct netconn *tcp_clientconn = NULL; /* TCP CLIENT网络连接结构体 */
    err_t err;
    ip4_addr_t server_ipaddr,local_ipaddr;
    static uint16_t server_port,local_port;
    server_port = LWIP_DEMO_PORT;
		uint8_t retry_conncet = 0;
		uint8_t*  tcp_send_buf = NULL;
		
		
    IP4_ADDR(&server_ipaddr,DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3);   /* 构造目的IP地址 */
    printf("准备进行创建和握手\r\n"); 
		while (retry_conncet  < 10) 
    {
				tcp_clientconn = netconn_new(NETCONN_TCP);  
				if (tcp_clientconn == NULL) {
							printf("无法创建新的连接\r\n");
							retry_conncet++;
							vTaskDelay(15000);
							continue;
				}		
				printf("创建新的连接成功\r\n");					
				// 确保连接在阻塞模式下工作
				err = netconn_connect(tcp_clientconn,&server_ipaddr,server_port); 
				if (err != ERR_OK){
						printf("接连失败\r\n");
						netconn_delete(tcp_clientconn);                                             /* 返回值不等于ERR_OK,删除tcp_clientconn连接 */
				}
				else if (err == ERR_OK) {                                                         /* 处理新连接的数据 */
				
						netconn_getaddr(tcp_clientconn,&local_ipaddr,&local_port,1);  
						tcp_nagle_disable(tcp_clientconn); //禁止把小的合并						/* 获取本地IP主机IP地址和端口号 */
						printf("连接上服务器%d.%d.%d.%d,本机端口号为:%d\r\n",DEST_IP_ADDR0,DEST_IP_ADDR1, DEST_IP_ADDR2,DEST_IP_ADDR3,local_port);
						retry_conncet = 20;
						break;
				}
				printf("当前重新尝试的次数为retry_conncet%d\r\n",retry_conncet+1);
				retry_conncet ++;	
				vTaskDelay(5000);		
		}
		if(retry_conncet == 20) { //表示连接上了
				printf("连接上了可以进行下一步了\r\n");
	
				tcp_send_buf = mymalloc(SRAMEX,5*1024); 			//分配用来发送的内存
				memset(tcp_send_buf,0x66,5*1024);
				uint8_t count = 0;
				//测试发送和接受
				while (1){
							memset(tcp_send_buf,0x66 + count,5*1024);
							err = netconn_write(tcp_clientconn, tcp_send_buf, 3000, NETCONN_NOCOPY);
							if (err != ERR_OK) {
									if(err == ERR_MEM)
										printf("内存不足\r\n");
									if (err == ERR_RST) {
										printf("Connection reset by peer.\n");
										netconn_close(tcp_clientconn);
										break;
									}				
							}
							else {		
									printf("发送 3000 字节成功%d\r\n",count);	
									struct netbuf *recvbuf; 
									// 阻塞式接受 可以用socket的非阻塞式接受
									if ((err = netconn_recv(tcp_clientconn,&recvbuf)) == ERR_OK){									
//											printf("消息可以收到\r\n");
											memset(tcp_send_buf,0,5*1024); /* 数据接收缓冲区清零 */
											uint16_t data_len = 0;
											//这里可能存在内存池给大了？
											for (struct pbuf *q = recvbuf->p;q != NULL;q = q->next) {        /* 遍历完整个pbuf链表 */     	 
													if (q->len > (5*1024 - data_len)) {					
														memcpy(tcp_send_buf + data_len,q->payload,(5*1024 - data_len));/* 拷贝数据 */ 
														data_len += 5*1024 - data_len;
														printf("达到接受能力上限了\r\n");
															break;
													}else
															memcpy(tcp_send_buf + data_len,q->payload,q->len);  
												data_len += q->len;
											}
											netbuf_delete(recvbuf); //少的还真是这个
											tcp_send_buf[data_len] = '\0';
											printf("接收到的数据是%s\r\n",tcp_send_buf);
										} else {
											if (err == ERR_RST) {
														printf("Connection reset by peer.\n");
														netconn_close(tcp_clientconn);
														break;
													}

										}
									count ++;
							}	
							vTaskDelay(500);
					}							
			}else
				printf("在指定次数下未连接TCP服务器\r\n");	
		if(tcp_send_buf)		// 存在内存泄漏--假设这任务被别的任务直接删了而不是while的break
		{	
				myfree(SRAMEX,tcp_send_buf);
				tcp_send_buf = NULL;
		}
}
void lwip_ping_call_back(struct netconn *conn, enum netconn_evt evt, u16_t len)
{
		//printf("ping的回调被调用了\r\n");
	    // 根据事件类型打印信息
    switch (evt) {
        case NETCONN_EVT_RCVPLUS:
            printf("Data received on netconn!\r\n");
            break;
        case NETCONN_EVT_SENDPLUS:
            printf("Data sent on netconn!\r\n");
            break;
        case NETCONN_EVT_ERROR:
            printf("Error occurred on netconn!\r\n");
            break;
        default:
            printf("Unknown event on netconn: %d\r\n", evt);
            break;
    }
}

// 是本机去ping其它人 而不是别人ping 我 别人ping我直接用其他的测试就行
// 如果你发现ping不同windows的话 可以搜一搜"windows主机不回复ping"
// 然后再windows defener 新建一个 ping 的规则就好啦
void test_ping_task(void *pvParameters)
{
		uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		printf("准备开始初始化lwip任务了\r\n");
	   while (lwip_comm_init(pmrvl88w8801_core->mac_address) != 0)	//不extern也许更好
    {
				printf("初始化失败\r\n");
        delay_ms(500);
        LED1_TOGGLE();
			  delay_ms(500);
    }
		
		struct netconn *conn = NULL;
    struct netbuf *buf = NULL;
    struct icmp_echo_hdr *icmp = NULL;
    ip_addr_t target_ip;
    err_t err;
		ipaddr_aton(DEST_IP, &target_ip);
		uint8_t ping_count = 4;
		uint8_t ping_data_size = 32;
		// 这里可以给一个call_back
		conn = netconn_new_with_proto_and_callback(NETCONN_RAW, IP_PROTO_ICMP,lwip_ping_call_back);		// 我们的目标不是TCP 所以是RAW格式
		configASSERT(conn);
		netconn_set_recvtimeout(conn, 2000); // 设置1秒超时
		printf("Target IP: %s\r\n", ipaddr_ntoa(&target_ip));
		netconn_bind(conn, IP_ADDR_ANY,0);
		
		for (int i = 0; i < ping_count; i++) {
        uint32_t start_time, end_time, rtt;
        
        // 创建 ICMP 请求包
        buf = netbuf_new();
        if (!buf) {
            printf("Ping: Failed to allocate netbuf\n");
            continue;
        }

        icmp = (struct icmp_echo_hdr *)netbuf_alloc(buf, sizeof(struct icmp_echo_hdr) + ping_data_size);
        if (!icmp) {
            printf("Ping: Failed to allocate buffer for ICMP\n");
            netbuf_delete(buf);
            continue;
        }

        // 填充 ICMP 头部
        ICMPH_TYPE_SET(icmp, ICMP_ECHO);
        ICMPH_CODE_SET(icmp, 0);
        icmp->id = htons(0x01);
        icmp->seqno = htons(i);  // ICMP 序号
			
				uint8_t * buffer = (uint8_t *)icmp + sizeof(struct icmp_echo_hdr);
				for (uint16_t i = 0; i < ping_data_size; i++) 
							buffer[i] = (uint8_t)(0x40 + i);
    
        icmp->chksum = 0;  // 先清零 后面好像会算的
				icmp->chksum = inet_chksum(icmp, sizeof(struct icmp_echo_hdr) + ping_data_size);
				printf("ICMP Checksum: 0x%04X\r\n", icmp->chksum);
        // 记录开始时间
        start_time = sys_now();
        // 发送 ICMP 请求
        err = netconn_sendto(conn, buf, &target_ip, IP_PROTO_ICMP); // 使用sendto，port参数设为0
        netbuf_delete(buf);  // 释放发送缓冲区

        if (err != ERR_OK) {
            printf("Ping: Send failed, error: %d\n", err);
            continue;
        }

        // 等待 ICMP 响应
        err = netconn_recv(conn, &buf);
        if (err == ERR_OK) {
            end_time = sys_now();  // 记录结束时间
            rtt = end_time - start_time;

            void *data;
            uint16_t len;
            
            netbuf_data(buf, &data, &len); // **正确读取数据**
            
            if (len >= (sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr))) {
                struct ip_hdr *iphdr = (struct ip_hdr *)data;
                uint8_t ttl = IPH_TTL(iphdr);  // **获取 TTL**

                printf("Reply from %s: bytes=%d time=%dms TTL=%d\r\n",
                       DEST_IP, ping_data_size, rtt, ttl);
            } else {
                printf("Ping: Received packet too small\r\n");
            }

            
        } else {
            printf("Request timed out.\r\n");
        }
				netbuf_delete(buf);
        vTaskDelay(500);  // 等待一段时间再发送下一次 Ping
    }

    printf("Ping finished.\r\n");
    netconn_delete(conn);
		while(1)
    {
        vTaskDelay(5);
    }
}

// 使用socket接口时(记得修改lwipopts.h)的非阻塞模式
/*uint8_t SetNonBlock(int fd)
{
    int res = 0;
    int flags = fcntl(fd, F_GETFL, 0);
   
    if(flags < 0)
    {
        printf("error code: %d, set non block, fcntl get failed\r\n", flags);
        return pdFALSE;
    }
    res = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    if(res < 0)
    {
        printf("error code: %d, set non block, fcntl set failed\r\n", res);
        return pdFALSE;
    }
    return pdTRUE;
}*/
