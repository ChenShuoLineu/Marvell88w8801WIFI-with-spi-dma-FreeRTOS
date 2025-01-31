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

/* ����һ���ź��� */
QueueHandle_t g_rx_queue = NULL;
xSemaphoreHandle g_delete_semaphore = NULL; 		//���ȵ�ر���Ϣ����(ûд�����)

extern struct kfifo * recv_fifo;				// ���յ������ݻᱻ��������
extern struct kfifo * send_fifo;				// ��Ҫ���͵����ݻᱻ��������
extern QueueHandle_t wifi_recvmsg_queue;


__lwip_dev g_lwipdev;                   /* lwip���ƽṹ�� */
struct netif g_lwip_netif;              /* ����һ��ȫ�ֵ�����ӿ� */
struct kfifo g_lwip_kfifo;							/* ȫ�ֱ������ڽ�����Ϣ*/
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
    netif->hwaddr_len = ETHARP_HWADDR_LEN; /*����MAC��ַ����,Ϊ6���ֽ�*/
    /*�������ʼ����MAC��ַ*/
    netif->hwaddr[0]=g_lwipdev.mac[0]; 
    netif->hwaddr[1]=g_lwipdev.mac[1]; 
    netif->hwaddr[2]=g_lwipdev.mac[2];
    netif->hwaddr[3]=g_lwipdev.mac[3];   
    netif->hwaddr[4]=g_lwipdev.mac[4];
    netif->hwaddr[5]=g_lwipdev.mac[5];
    
    netif->mtu=1500; /*��������䵥Ԫ,����������㲥��ARP����*/
   
    /* ����һ���ź��� */
		//printf("����g_rx_semaphore��\r\n");
    g_rx_queue = xQueueCreate(16,sizeof(uint32_t));
		configASSERT(g_rx_queue != NULL);
		g_delete_semaphore = xSemaphoreCreateBinary();
    /* ��������ETH_MAC������ */
    sys_thread_new("eth_thread",
                   ethernetif_input,        /* ������ں��� */
                   netif,                   /* ������ں������� */
                   NETIF_IN_TASK_STACK_SIZE,/* ����ջ��С */
                   NETIF_IN_TASK_PRIORITY); /* ��������ȼ� */
    
    /* ����״̬��Ϣ��־λ���Ǻ���Ҫ�Ŀ����ֶΣ���������������ʹ�ܡ��㲥*/
    /* ʹ�ܡ� ARP ʹ�ܵȵ���Ҫ����λ*/
		// ע�����link_up ��Ϊ�����ǳ�ʼ���ò����� ��������û����
    netif->flags = NETIF_FLAG_BROADCAST|NETIF_FLAG_ETHARP|NETIF_FLAG_LINK_UP;   /*�㲥 ARPЭ�� ���Ӽ��*/
    
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

//��ô�����ⷢ�� �Ѿ��󶨺���
static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
		//printf("����low_level_output������\r\n");
		
    //struct ethernetif *ethernetif = netif->state;
    //struct pbuf *q;
		// ÿ�����1514�ֽ�


#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

		//printf("���յ��� low_level_ouput����\r\n");
		//��������copy��ȥ
		// �����ôд��
		taskENTER_CRITICAL();  // �����ٽ���
		uint16_t send_len = kfifo_put_pbuf(send_fifo, p, p->tot_len, 0);
		taskEXIT_CRITICAL();   // �˳��ٽ���
		wifi_msg wifi_status;
	
//		printf("totlen: %d����kfifo����%d\r\n",p->tot_len,send_len);

		// ������Ϣ Ȼ�����Ϣ���ͳ�ȥ
		wifi_status.wifi_status = WIFI_SEND;
		wifi_status.init_status = send_len;
    xQueueSendToBack(wifi_recvmsg_queue,&wifi_status,portMAX_DELAY);
	
		//printf("low_level_output�����������\r\n")
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
		//printf("�˳�low_level_output������\r\n");
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
            /* �����յ��İ��ƶ����µ�pbuf�� */
//						printf("������������˾�˵����ȡ��length%d\r\n",length);
            taskENTER_CRITICAL();
            /* ����low_level_input������������ */
            p = kfifo_get_pbuf(recv_fifo,length);					// �о�һֱ��������ճ����ͦ�˷ѵ�
						configASSERT(p);
						//rx������
            taskEXIT_CRITICAL();

            /* ָ�����Ч���أ�����һ����̫����ͷ��ʼ */
            if (p != NULL)
            {
                taskENTER_CRITICAL();

                /* ����netif�ṹ���е�input�ֶ�(һ������)���������ݰ� */
                if (netif->input(p, netif) != ERR_OK)
                {
                    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                    pbuf_free(p);
                    p = NULL;
                }
                else
                {
//										printf("�����ύ�ɹ�\r\n");
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
		//����˵Ҫ��������ô����
    netif->linkoutput = low_level_output;



    low_level_init(netif);

    return ERR_OK;
}

void lwip_comm_default_ip_set(__lwip_dev *lwipx,uint8_t *mac_address)
{
    /* Ĭ��Զ��IPΪ:192.168.3.20 */
    lwipx->remoteip[0] = DEST_IP_ADDR0;
    lwipx->remoteip[1] = DEST_IP_ADDR1;
    lwipx->remoteip[2] = DEST_IP_ADDR2;
    lwipx->remoteip[3] = DEST_IP_ADDR3;
		/*printf("lwip_comm_default_ip_set%2x : %2x :%2x : %2x :%2x :%2x",mac_address[0],mac_address[1],mac_address[2],
		mac_address[3],mac_address[4],mac_address[5]);*/
	
	
    /* MAC��ַ���� */
    lwipx->mac[0] = mac_address[0];
    lwipx->mac[1] = mac_address[1];
    lwipx->mac[2] = mac_address[2];
    lwipx->mac[3] = mac_address[3];
    lwipx->mac[4] = mac_address[4];
    lwipx->mac[5] = mac_address[5];
    
    /* Ĭ�ϱ���IPΪ:192.168.3.40 */
    lwipx->ip[0] = LOCAL_IP_ADDR0;
    lwipx->ip[1] = LOCAL_IP_ADDR1;
    lwipx->ip[2] = LOCAL_IP_ADDR2;
    lwipx->ip[3] = LOCAL_IP_ADDR3;
    /* Ĭ����������:255.255.255.0 */
    lwipx->netmask[0] = 255;
    lwipx->netmask[1] = 255;
    lwipx->netmask[2] = 255;
    lwipx->netmask[3] = 0;
    
    /* Ĭ������:192.168.3.1 */
    lwipx->gateway[0] = LIWP_GATEWAY0;
    lwipx->gateway[1] = LIWP_GATEWAY1;
    lwipx->gateway[2] = LIWP_GATEWAY2;
    lwipx->gateway[3] = LIWP_GATEWAY3;
    lwipx->dhcpstatus = 0; /* û��DHCP */
}


// ���������� ��Ϊǰ���Ǹ�ѭ�� ����˵�����ÿ�ζ��ͷ���һ���ź���
// ���������Բ�����Ҳ��֪�������˶��
void ethernet_lwip_process(uint8_t *rx,uint32_t rx_len)
{
		RxPD *rx_packet = (RxPD *)rx; //ע�����state�ڵ���low_level_input֮ǰ���޸���
    rx_packet->payload = (u8_t *)((u8_t *)rx_packet + rx_packet->rx_pkt_offset + 4);
    uint16_t len = kfifo_put(recv_fifo,rx_packet->payload,rx_packet->rx_pkt_length); //�����ݷŽ�ȥ
//		printf("�������len is %d rx_len is %d \r\n",len,rx_len);	// ������һ�������� ��Ϊ�����������ݶ���Ч
    xQueueSend(g_rx_queue, &len,portMAX_DELAY); //����Ϣ����ȥ����
}
/**
 * @breif       LWIP��ʼ��(LWIP������ʱ��ʹ��)
 * @param       ��
 * @retval      0,�ɹ�
 *              1,�ڴ����
 *              2,��̫��оƬ��ʼ��ʧ��
 *              3,�������ʧ��.
 */
// �������Ӧ����freertos�е��� ���ȿ����������
uint8_t lwip_comm_init(uint8_t *mac_address)
{
    struct netif *netif_init_flag;              /* ����netif_add()����ʱ�ķ���ֵ,�����ж������ʼ���Ƿ�ɹ� */
    ip_addr_t ipaddr;                           /* ip��ַ */
    ip_addr_t netmask;                          /* �������� */
    ip_addr_t gw;                               /* Ĭ������ */
    //�Ҳ���tcpip_initһ������ǰ�治���ں���
		tcpip_init(NULL, NULL); //����ͻᴴ��һ�����������Ϣ����
		//printf("lwip_comm_init%2x : %2x :%2x : %2x :%2x :%2x",mac_address[0],mac_address[1],mac_address[2],
		//mac_address[3],mac_address[4],mac_address[5]);
    lwip_comm_default_ip_set(&g_lwipdev,mac_address);         /* ����Ĭ��IP��mac��ַ����Ϣ */


#if LWIP_DHCP                                   /* ʹ�ö�̬IP */
    ip_addr_set_zero_ip4(&ipaddr);              /* ��IP��ַ���������뼰�������� */
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);
#else   /* ʹ�þ�̬IP */
    IP4_ADDR(&ipaddr, g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    IP4_ADDR(&netmask, g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    IP4_ADDR(&gw, g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
    printf("����en��MAC��ַΪ:................%d.%d.%d.%d.%d.%d\r\n", g_lwipdev.mac[0], g_lwipdev.mac[1], g_lwipdev.mac[2], g_lwipdev.mac[3], g_lwipdev.mac[4], g_lwipdev.mac[5]);
    printf("��̬IP��ַ........................%d.%d.%d.%d\r\n", g_lwipdev.ip[0], g_lwipdev.ip[1], g_lwipdev.ip[2], g_lwipdev.ip[3]);
    printf("��������..........................%d.%d.%d.%d\r\n", g_lwipdev.netmask[0], g_lwipdev.netmask[1], g_lwipdev.netmask[2], g_lwipdev.netmask[3]);
    printf("Ĭ������..........................%d.%d.%d.%d\r\n", g_lwipdev.gateway[0], g_lwipdev.gateway[1], g_lwipdev.gateway[2], g_lwipdev.gateway[3]);
    g_lwipdev.dhcpstatus = 0XFF;
		
#endif  /* �������б������һ������ */
    netif_init_flag = netif_add(&g_lwip_netif, (const ip_addr_t *)&ipaddr, (const ip_addr_t *)&netmask, (const ip_addr_t *)&gw, NULL, &ethernetif_init, &tcpip_input);
    if (netif_init_flag == NULL)
    {
        return 4;                           /* �������ʧ�� */
    }
//		printf("����������\r\n");
		netif_set_default(&g_lwip_netif); // ��ΪĬ������
//		printf("׼��ʹ�ø�����\r\n");
    netif_set_up(&g_lwip_netif); // ����lwipʹ�ø�����


/* ����OK. */
    return 0;                               /* ����OK. */
}

void set_static_arp_entry(ip4_addr_t server_ipaddr,uint8_t * remote_address) {
    struct eth_addr ethaddr;
    

    // ����Ŀ��MAC��ַ��FF:FF:F0:FF:FF:FF
    ethaddr.addr[0] = remote_address[0];
    ethaddr.addr[1] = remote_address[1];
    ethaddr.addr[2] = remote_address[2];
    ethaddr.addr[3] = remote_address[3];
    ethaddr.addr[4] = remote_address[4];
    ethaddr.addr[5] = remote_address[5];

    // ��Ӿ�̬ARP��Ŀ��ָ��������ӿ�
    etharp_add_static_entry(&server_ipaddr, &ethaddr);

    // ���� ARP �����е���Ϣ��ָ��������ӿ�
    etharp_request(&g_lwip_netif, &server_ipaddr);
}
void lwip_set_link_down(void)
{
		netif_set_link_down(&g_lwip_netif);
}
void ethernet_link_down()
{
		printf("�ͷŶ�ֵ�ź���\r\n");
    xSemaphoreGive(g_delete_semaphore);/* �ͷŶ�ֵ�ź��� */
}
