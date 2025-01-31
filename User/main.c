/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-5-04
 * @brief       lwIP+FreeRTOS操作系统移植 实验
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

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/SRAM/sram.h"
#include "./MALLOC/malloc.h"
#include "freertos_demo.h"
#include "./BSP/SPI_SDIO/spi_sdio.h"
#include "wifi.h"
#include "./BSP/OV2640/ov2640.h"
#include "./BSP/24CXX/24cxx.h"


uint8_t run_success_flag = 0xFE;
enum BootTable {
    APP_RUN_SUCCESS = 0,   // APP_RUN_SUCCESS为0x66表示成功运行 为0则表示需要切换掉备份程序
};

int main(void)
{
		//sys_nvic_set_vector_table(FLASH_BASE, 0x40000);
		//__set_PRIMASK(0);//解除中断屏蔽，打开中断
	
		//__enable_irq();
    HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7); /* 设置时钟,168Mhz */
    delay_init(168);                    /* 延时初始化 */
    usart_init(115200);                 /* 串口初始化为115200 */
    led_init();                         /* 初始化LED */
		at24cxx_init();											// I2C的初始化
		delay_ms(50);
		at24cxx_write_one_byte(APP_RUN_SUCCESS,run_success_flag);
		delay_ms(50);
		printf("读取到APP正常运行的flag是0x%x\r\n",at24cxx_read_one_byte(APP_RUN_SUCCESS));
    printf("程序正常\r\n");
		sram_init();   
		/* SRAM初始化 */
    my_mem_init(SRAMEX);                /* 初始化外部SRAM内存池 */
		// 连接上了就跳转到这里
		freertos_demo();                    /* 创建lwIP的任务函数 */
}
