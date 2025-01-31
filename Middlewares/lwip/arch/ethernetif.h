/**
 ****************************************************************************************************
 * @file        ethernetif.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-02-14
 * @brief       ������������
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
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__
#include "lwip/err.h"
#include "lwip/netif.h"
#include "lwip/opt.h"
#include "netif/etharp.h"  
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/ethip6.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "netif/ppp/pppoe.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "lwip/sys.h"
#include "wifi.h"

typedef void (*display_fn)(uint8_t index);


#define NETIF_IN_TASK_STACK_SIZE    ( 2048 )
#define NETIF_IN_TASK_PRIORITY      ( 14 )

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'


// ����IP Զ��IP �� ��������
#define DEST_IP_ADDR0               192
#define DEST_IP_ADDR1               168
#define DEST_IP_ADDR2               29
#define DEST_IP_ADDR3               219

#define DEST_IP										"192.168.29.219"

#define LOCAL_IP_ADDR0							192
#define LOCAL_IP_ADDR1							168
#define LOCAL_IP_ADDR2							29
#define LOCAL_IP_ADDR3							40

#define LOCAL_IP									"192.168.29.40"

#define LIWP_GATEWAY0								192
#define LIWP_GATEWAY1								168
#define LIWP_GATEWAY2								29
#define LIWP_GATEWAY3								130

#define LWIP_DEMO_PORT               8080  /* ���ӵ�Զ�˷������Ķ˿ں� */

typedef struct  
{
    uint8_t mac[6];                 /* MAC��ַ */
    uint8_t remoteip[4];            /* Զ������IP��ַ */ 
    uint8_t ip[4];                  /* ����IP��ַ */
    uint8_t netmask[4];             /* �������� */
    uint8_t gateway[4];             /* Ĭ�����ص�IP��ַ */
    uint8_t dhcpstatus;             /* dhcp״̬ */
                                        /* 0, δ��ȡDHCP��ַ;*/
                                        /* 1, ����DHCP��ȡ״̬*/
                                        /* 2, �ɹ���ȡDHCP��ַ*/
                                        /* 0XFF,��ȡʧ�� */
    uint8_t link_status;                                /* ����״̬ */
    display_fn lwip_display_fn;                         /* ��ʾ����ָ�� */
}__lwip_dev;

 
void    lwip_comm_default_ip_set(__lwip_dev *lwipx,uint8_t *mac_address);    /* lwip Ĭ��IP���� */
uint8_t lwip_comm_init(uint8_t *mac_address);                           /* LWIP��ʼ��(LWIP������ʱ��ʹ��) */
void ethernet_lwip_process(uint8_t *rx,uint32_t rx_len);
err_t ethernetif_init(struct netif *netif);  /* ������ʼ������ */
void set_static_arp_entry(ip4_addr_t server_ipaddr,uint8_t * remote_address);
void lwip_set_link_down(void);
void ethernet_link_down(void);
#endif

