#ifndef __SPISIDO_H
#define __SPISIDO_H

#include <string.h>
#include <stdlib.h>

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"

#include "FreeRTOS.h"
#include "semphr.h"

/* SPI总线速度设置 */
#define SPI_SPEED_2         0
#define SPI_SPEED_4         1
#define SPI_SPEED_8         2
#define SPI_SPEED_16        3
#define SPI_SPEED_32        4
#define SPI_SPEED_64        5
#define SPI_SPEED_128       6
#define SPI_SPEED_256       7

#define POLYNOMIAL_CRC7 0x89ul

/* 程序中用到的函数选项 */
#define WIFI_RWDATA_ADDRINCREMENT _BV(0)
#define WIFI_RWDATA_ALLOWMULTIBYTE _BV(1)



#define sd_spi_speed_low()              spi_set_speed(SPI_SPEED_256)       /* SD卡 SPI低速模式 */
#define sd_spi_speed_high()             spi_set_speed(SPI_SPEED_8)         /* SD卡 SPI高速模式 */

#define SD_DUMMY_BYTE   0xFF

#define _BV(bit) ((1u) << (bit))

//GPIO的配置
#define SD_SPI                           SPI2
#define SD_SPI_CLK()                     __HAL_RCC_SPI2_CLK_ENABLE()
//2024/8/16 还真得两倍长度？ 
#define SD_SPI_SCK_PIN                   GPIO_PIN_13               
#define SD_SPI_SCK_GPIO_PORT             GPIOB                       
#define SD_SPI_SCK_GPIO_CLK()             __HAL_RCC_GPIOB_CLK_ENABLE()


#define SD_CS_PIN                        GPIO_PIN_12                 
#define SD_CS_GPIO_PORT                  GPIOB                    
#define SD_CS_GPIO_CLK()                 __HAL_RCC_GPIOB_CLK_ENABLE()

/* NORFLASH 片选信号 */
#define SD_CS(x)      do{ x ? \
                            HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET); \
                         }while(0)

#define SD_SPI_MISO_PIN                  GPIO_PIN_2                
#define SD_SPI_MISO_GPIO_PORT            GPIOC                       
#define SD_SPI_MISO_GPIO_CLK()           __HAL_RCC_GPIOC_CLK_ENABLE()
	 
#define SD_SPI_MOSI_PIN                  GPIO_PIN_3                
#define SD_SPI_MOSI_GPIO_PORT            GPIOC                       
#define SD_SPI_MOSI_GPIO_CLK()            __HAL_RCC_GPIOC_CLK_ENABLE()
	 


// PC13 IRQ 这样的话当我打开中断后就可以查询这个位看看有没有命令需要处理
#define SD_IRQ_PIN                    GPIO_PIN_13                 
#define SD_IRQ_GPIO_PORT              GPIOC                       
#define SD_IRQ_GPIO_CLK()             __HAL_RCC_GPIOC_CLK_ENABLE() 
												 
#define INT (HAL_GPIO_ReadPin(SD_IRQ_GPIO_PORT, SD_IRQ_PIN) == GPIO_PIN_SET)		

												 
//PC0 PDN
#define SD_PDN_PIN                    GPIO_PIN_0                 
#define SD_PDN_GPIO_PORT              GPIOC                       
#define SD_PDN_GPIO_CLK()             __HAL_RCC_GPIOC_CLK_ENABLE()
												 
/* NORFLASH 片选信号 */
#define SD_PDN(x)      do{ x ? \
                            HAL_GPIO_WritePin(SD_PDN_GPIO_PORT, SD_PDN_PIN, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(SD_PDN_GPIO_PORT, SD_PDN_PIN, GPIO_PIN_RESET); \
                         }while(0)
/******************************************************************************************/
/* SD卡 返回值定义 */
#define SD_OK           0
#define SD_ERROR        1
#define SD_TIMEOUT			(-1)

/* SDIO的操作 */
#define SDIO_EXCU_READ	0
#define SDIO_EXCU_WRITE	1
												 

/* SD卡 命令定义 */
#define CMD0    (0)             /* GO_IDLE_STATE */
#define CMD1    (1)             /* SEND_OP_COND (MMC) */
#define CMD5     (5)
#define CMD8  	(8)
#define CMD12			(12)						
#define CMD52   (52)            /* MARVELL专用的*/
#define CMD53 	(53)
#define CMD55 	(55)



#define CMD52_WRITE _BV(31)
#define CMD52_READAFTERWRITE _BV(27)
#define CMD53_WRITE _BV(31)
#define CMD53_BLOCKMODE _BV(27)
#define CMD53_INCREMENTING _BV(26)
#define POLYNOMIAL_CRC16 0x11021ul


// 固件状态寄存器
#define WIFI_SCRATCH0_0 0x60 // 88W8801 firmware status
#define WIFI_SCRATCH0_1 0x61

#define WIFI_SCRATCH0_2 0x62
#define WIFI_SCRATCH0_3 0x63

/* 程序中用到的常量 */
#define WIFI_FIRMWARESTATUS_OK 0xfedc
#define WIFI_MACADDR_LEN 6
#define WIFI_MAX_SSIDLEN 32
#define WIFI_MAX_WEPKEYLEN 26
#define WIFI_MAX_WPAKEYLEN 64

//关于WIFI的一些												 
/** Host Control Registers : I/O port 0 */
#define IO_PORT_0_REG			0x78
/** Host Control Registers : I/O port 1 */
#define IO_PORT_1_REG			0x79
/** Host Control Registers : I/O port 2 */
#define IO_PORT_2_REG			0x7A

#define POLY 0x1021

//关于SDIO协议的
unsigned char sd_init(void);
void spi_set_speed(uint8_t speed);

// 给WIFI 用的 校验/发送/接收
uint8_t WiFi_LowLevel_CalcCRC7(const void *data, int len);
uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length);
// 读取参数
uint8_t WiFi_LowLevel_GetFunctionNum(void); //得到FUN NUM有几个
int WiFi_LowLevel_SetBlockSize(uint8_t func, uint32_t size);
int WiFi_LowLevel_GetITStatus(uint8_t clear);

// 对外的接口用来读写数据

uint8_t WiFi_LowLevel_ReadReg(uint8_t func, uint32_t addr);
uint8_t WiFi_LowLevel_WriteReg(uint8_t func, uint32_t addr, uint8_t value);

int WiFi_LowLevel_WriteData(uint8_t func, uint32_t addr,void *data, uint32_t size, 
	uint32_t bufsize, uint32_t flags); //写数据用这个
int WiFi_LowLevel_ReadData(uint8_t func, uint32_t addr, void *data, 
	uint32_t size, uint32_t bufsize, uint32_t flags);//读取数据




uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length);
#endif
