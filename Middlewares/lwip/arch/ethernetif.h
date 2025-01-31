/**
 ****************************************************************************************************
 * @file        ethernetif.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-02-14
 * @brief       网卡驱动代码
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
 * 修改说明
 * V1.0 20211014
 * 第一次发布
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


// 本地IP 远端IP 和 网关设置
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

#define LWIP_DEMO_PORT               8080  /* 连接的远端服务器的端口号 */

typedef struct  
{
    uint8_t mac[6];                 /* MAC地址 */
    uint8_t remoteip[4];            /* 远端主机IP地址 */ 
    uint8_t ip[4];                  /* 本机IP地址 */
    uint8_t netmask[4];             /* 子网掩码 */
    uint8_t gateway[4];             /* 默认网关的IP地址 */
    uint8_t dhcpstatus;             /* dhcp状态 */
                                        /* 0, 未获取DHCP地址;*/
                                        /* 1, 进入DHCP获取状态*/
                                        /* 2, 成功获取DHCP地址*/
                                        /* 0XFF,获取失败 */
    uint8_t link_status;                                /* 连接状态 */
    display_fn lwip_display_fn;                         /* 显示函数指针 */
}__lwip_dev;

 
void    lwip_comm_default_ip_set(__lwip_dev *lwipx,uint8_t *mac_address);    /* lwip 默认IP设置 */
uint8_t lwip_comm_init(uint8_t *mac_address);                           /* LWIP初始化(LWIP启动的时候使用) */
void ethernet_lwip_process(uint8_t *rx,uint32_t rx_len);
err_t ethernetif_init(struct netif *netif);  /* 网卡初始化函数 */
void set_static_arp_entry(ip4_addr_t server_ipaddr,uint8_t * remote_address);
void lwip_set_link_down(void);
void ethernet_link_down(void);
#endif

