/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"
#include "netif/etharp.h"  
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/ethip6.h"
#include "lwip/etharp.h"
#include "netif/ppp/pppoe.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "./MALLOC/malloc.h"
#include "./BSP/ringBuffer/ringbuffer.h"
#include "wifi_app.h"


#include "task.h"
#include "lwip/sys.h"
#include "ethernetif.h" 
#include "string.h"

/* 定义一个信号量 */
QueueHandle_t g_rx_queue = NULL;
xSemaphoreHandle g_delete_semaphore = NULL; 		//收热点关闭消息后处理(没写打个样)

extern struct kfifo * recv_fifo;				// 接收到的数据会被放入这里
extern struct kfifo * send_fifo;				// 需要发送的数据会被放入这里
extern QueueHandle_t wifi_recvmsg_queue;


__lwip_dev g_lwipdev;                   /* lwip控制结构体 */
struct netif g_lwip_netif;              /* 定义一个全局的网络接口 */
struct kfifo g_lwip_kfifo;							/* 全局变量用于接收信息*/
/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

/* Forward declarations. */
void  ethernetif_input(void *pParams);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init(struct netif *netif)
{
    netif->hwaddr_len = ETHARP_HWADDR_LEN; /*设置MAC地址长度,为6个字节*/
    /*在这里初始化的MAC地址*/
    netif->hwaddr[0]=g_lwipdev.mac[0]; 
    netif->hwaddr[1]=g_lwipdev.mac[1]; 
    netif->hwaddr[2]=g_lwipdev.mac[2];
    netif->hwaddr[3]=g_lwipdev.mac[3];   
    netif->hwaddr[4]=g_lwipdev.mac[4];
    netif->hwaddr[5]=g_lwipdev.mac[5];
    
    netif->mtu=1500; /*最大允许传输单元,允许该网卡广播和ARP功能*/
   
    /* 创建一个信号量 */
		//printf("创建g_rx_semaphore了\r\n");
    g_rx_queue = xQueueCreate(16,sizeof(uint32_t));
		configASSERT(g_rx_queue != NULL);
		g_delete_semaphore = xSemaphoreCreateBinary();
    /* 创建处理ETH_MAC的任务 */
    sys_thread_new("eth_thread",
                   ethernetif_input,        /* 任务入口函数 */
                   netif,                   /* 任务入口函数参数 */
                   NETIF_IN_TASK_STACK_SIZE,/* 任务栈大小 */
                   NETIF_IN_TASK_PRIORITY); /* 任务的优先级 */
    
    /* 网卡状态信息标志位，是很重要的控制字段，它包括网卡功能使能、广播*/
    /* 使能、 ARP 使能等等重要控制位*/
		// 注意这个link_up 因为我们是初始化好才来的 所以这样没问题
    netif->flags = NETIF_FLAG_BROADCAST|NETIF_FLAG_ETHARP|NETIF_FLAG_LINK_UP;   /*广播 ARP协议 链接检测*/
    
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

//怎么样向外发送 已经绑定好了
static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
		//printf("进入low_level_output函数了\r\n");
		
    //struct ethernetif *ethernetif = netif->state;
    //struct pbuf *q;
		// 每次最多1514字节


#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

		//printf("最终到了 low_level_ouput里面\r\n");
		//尽最大可能copy过去
		// 这块怎么写了
		taskENTER_CRITICAL();  // 进入临界区
		uint16_t send_len = kfifo_put_pbuf(send_fifo, p, p->tot_len, 0);
		taskEXIT_CRITICAL();   // 退出临界区
		wifi_msg wifi_status;
	
//		printf("totlen: %d放入kfifo的量%d\r\n",p->tot_len,send_len);

		// 制造消息 然后把消息发送出去
		wifi_status.wifi_status = WIFI_SEND;
		wifi_status.init_status = send_len;
    xQueueSendToBack(wifi_recvmsg_queue,&wifi_status,portMAX_DELAY);
	
		//printf("low_level_output发送数据完毕\r\n")
    MIB2_STATS_NETIF_ADD(netif, ifoutoctets, p->tot_len);
    if (((u8_t*)p->payload)[0] & 1)
    {
        /* broadcast or multicast packet*/
        MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
    }
    else
    {
        /* unicast packet */
        MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
    }
    /* increase ifoutdiscards or ifouterrors on error */

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

    LINK_STATS_INC(link.xmit);
		//printf("退出low_level_output函数了\r\n");
    return ERR_OK;
}


/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void
ethernetif_input(void *pParams)
{
    struct netif *netif;
    struct pbuf *p = NULL;
    netif = (struct netif *) pParams;
		uint32_t length;
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));

    while (1)
    {
        if (xQueueReceive(g_rx_queue, &length, portMAX_DELAY) == pdPASS)
        {
            /* 将接收到的包移动到新的pbuf中 */
//						printf("进到这个里面了就说明获取到length%d\r\n",length);
            taskENTER_CRITICAL();
            /* 调用low_level_input函数接收数据 */
            p = kfifo_get_pbuf(recv_fifo,length);					// 感觉一直这样复制粘贴还挺浪费的
						configASSERT(p);
						//rx才用完
            taskEXIT_CRITICAL();

            /* 指向包有效负载，它从一个以太网报头开始 */
            if (p != NULL)
            {
                taskENTER_CRITICAL();

                /* 调用netif结构体中的input字段(一个函数)来处理数据包 */
                if (netif->input(p, netif) != ERR_OK)
                {
                    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                    pbuf_free(p);
                    p = NULL;
                }
                else
                {
//										printf("向上提交成功\r\n");
                }

                taskEXIT_CRITICAL();
            }
        }
    }

}  

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
ethernetif_init(struct netif *netif)
{

    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
    * Initialize the snmp variables and counters inside the struct netif.
    * The last argument should be replaced with your link speed, in units
    * of bits per second.
    */
    MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
    * You can instead declare your own function an call etharp_output()
    * from it if you have to do some checks before sending (e.g. if link
    * is available...) */
#if LWIP_IPV4
    netif->output = etharp_output;
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
		//假如说要向外了怎么传送
    netif->linkoutput = low_level_output;



    low_level_init(netif);

    return ERR_OK;
}

void lwip_comm_default_ip_set(__lwip_dev *lwipx,uint8_t *mac_address)
{
    /* 默认远端IP为:192.168.3.20 */
    lwipx->remoteip[0] = DEST_IP_ADDR0;
    lwipx->remoteip[1] = DEST_IP_ADDR1;
    lwipx->remoteip[2] = DEST_IP_ADDR2;
    lwipx->remoteip[3] = DEST_IP_ADDR3;
		/*printf("lwip_comm_default_ip_set%2x : %2x :%2x : %2x :%2x :%2x",mac_address[0],mac_address[1],mac_address[2],
		mac_address[3],mac_address[4],mac_address[5]);*/
	
	
    /* MAC地址设置 */
    lwipx->mac[0] = mac_address[0];
    lwipx->mac[1] = mac_address[1];
    lwipx->mac[2] = mac_address[2];
    lwipx->mac[3] = mac_address[3];
    lwipx->mac[4] = mac_address[4];
    lwipx->mac[5] = mac_address[5];
    
    /* 默认本地IP为:192.168.3.40 */
    lwipx->ip[0] = LOCAL_IP_ADDR0;
    lwipx->ip[1] = LOCAL_IP_ADDR1;
    lwipx->ip[2] = LOCAL_IP_ADDR2;
    lwipx->ip[3] = LOCAL_IP_ADDR3;
    /* 默认子网掩码:255.255.255.0 */
    lwipx->netmask[0] = 255;
    lwipx->netmask[1] = 255;
    lwipx->netmask[2] = 255;
    lwipx->netmask[3] = 0;
    
    /* 默认网关:192.168.3.1 */
    lwipx->gateway[0] = LIWP_GATEWAY0;
    lwipx->gateway[1] = LIWP_GATEWAY1;
    lwipx->gateway[2] = LIWP_GATEWAY2;
    lwipx->gateway[3] = LIWP_GATEWAY3;
    lwipx->dhcpstatus = 0; /* 没有DHCP */
}


// 这里有问题 因为前面是个循环 所以说吧这个每次都释放了一个信号量
// 可是这明显不对你也不知道触发了多久
void ethernet_lwip_process(uint8_t *rx,uint32_t rx_len)
{
		RxPD *rx_packet = (RxPD *)rx; //注意这个state在调用low_level_input之前被修改了
    rx_packet->payload = (u8_t *)((u8_t *)rx_packet + rx_packet->rx_pkt_offset + 4);
    uint16_t len = kfifo_put(recv_fifo,rx_packet->payload,rx_packet->rx_pkt_length); //把数据放进去
//		printf("放入完毕len is %d rx_len is %d \r\n",len,rx_len);	// 这俩不一样很正常 因为不是所有数据都有效
    xQueueSend(g_rx_queue, &len,portMAX_DELAY); //把消息发过去就行
}
/**
 * @breif       LWIP初始化(LWIP启动的时候使用)
 * @param       无
 * @retval      0,成功
 *              1,内存错误
 *              2,以太网芯片初始化失败
 *              3,网卡添加失败.
 */
// 这个函数应该在freertos中调用 得先开启任务调度
uint8_t lwip_comm_init(uint8_t *mac_address)
{
    struct netif *netif_init_flag;              /* 调用netif_add()函数时的返回值,用于判断网络初始化是否成功 */
    ip_addr_t ipaddr;                           /* ip地址 */
    ip_addr_t netmask;                          /* 子网掩码 */
    ip_addr_t gw;                               /* 默认网关 */
    //我草了tcpip_init一定得在前面不能在后面
		tcpip_init(NULL, NULL); //这里就会创建一个任务管理消息邮箱
		//printf("lwip_comm_init%2x : %2x :%2x : %2x :%2x :%2x",mac_address[0],mac_address[1],mac_address[2],
		//mac_address[3],mac_address[4],mac_address[5]);
    lwip_comm_default_ip_set(&g_lwipdev,mac_address);         /* 设置默认IP和mac地址等信息 */


#if LWIP_DHCP                                   /* 使用动态IP */
    ip_addr_set_zero_ip4(&ipaddr);              /* 对IP地址、子网掩码及网关清零 */
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);
#else   /* 使用静态IP */
    IP4_ADDR(&ipaddr, g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    IP4_ADDR(&netmask, g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    IP4_ADDR(&gw, g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
    printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n", g_lwipdev.mac[0], g_lwipdev.mac[1], g_lwipdev.mac[2], g_lwipdev.mac[3], g_lwipdev.mac[4], g_lwipdev.mac[5]);
    printf("静态IP地址........................%d.%d.%d.%d\r\n", g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    printf("子网掩码..........................%d.%d.%d.%d\r\n", g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    printf("默认网关..........................%d.%d.%d.%d\r\n", g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
    g_lwipdev.dhcpstatus = 0XFF;
		
#endif  /* 向网卡列表中添加一个网口 */
    netif_init_flag = netif_add(&g_lwip_netif, (const ip_addr_t *)&ipaddr, (const ip_addr_t *)&netmask, (const ip_addr_t *)&gw, NULL, &ethernetif_init, &tcpip_input);
    if (netif_init_flag == NULL)
    {
        return 4;                           /* 网卡添加失败 */
    }
//		printf("添加网卡完毕\r\n");
		netif_set_default(&g_lwip_netif); // 设为默认网卡
//		printf("准备使用该网卡\r\n");
    netif_set_up(&g_lwip_netif); // 允许lwip使用该网卡


/* 操作OK. */
    return 0;                               /* 操作OK. */
}

void set_static_arp_entry(ip4_addr_t server_ipaddr,uint8_t * remote_address) {
    struct eth_addr ethaddr;
    

    // 设置目标MAC地址：FF:FF:F0:FF:FF:FF
    ethaddr.addr[0] = remote_address[0];
    ethaddr.addr[1] = remote_address[1];
    ethaddr.addr[2] = remote_address[2];
    ethaddr.addr[3] = remote_address[3];
    ethaddr.addr[4] = remote_address[4];
    ethaddr.addr[5] = remote_address[5];

    // 添加静态ARP条目到指定的网络接口
    etharp_add_static_entry(&server_ipaddr, &ethaddr);

    // 更新 ARP 缓存中的信息到指定的网络接口
    etharp_request(&g_lwip_netif, &server_ipaddr);
}
void lwip_set_link_down(void)
{
		netif_set_link_down(&g_lwip_netif);
}
void ethernet_link_down()
{
		printf("释放二值信号量\r\n");
    xSemaphoreGive(g_delete_semaphore);/* 释放二值信号量 */
}
