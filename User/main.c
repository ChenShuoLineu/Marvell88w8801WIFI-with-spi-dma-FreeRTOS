/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-5-04
 * @brief       lwIP+FreeRTOS����ϵͳ��ֲ ʵ��
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
    APP_RUN_SUCCESS = 0,   // APP_RUN_SUCCESSΪ0x66��ʾ�ɹ����� Ϊ0���ʾ��Ҫ�л������ݳ���
};

int main(void)
{
		//sys_nvic_set_vector_table(FLASH_BASE, 0x40000);
		//__set_PRIMASK(0);//����ж����Σ����ж�
	
		//__enable_irq();
    HAL_Init();                         /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7); /* ����ʱ��,168Mhz */
    delay_init(168);                    /* ��ʱ��ʼ�� */
    usart_init(115200);                 /* ���ڳ�ʼ��Ϊ115200 */
    led_init();                         /* ��ʼ��LED */
		at24cxx_init();											// I2C�ĳ�ʼ��
		delay_ms(50);
		at24cxx_write_one_byte(APP_RUN_SUCCESS,run_success_flag);
		delay_ms(50);
		printf("��ȡ��APP�������е�flag��0x%x\r\n",at24cxx_read_one_byte(APP_RUN_SUCCESS));
    printf("��������\r\n");
		sram_init();   
		/* SRAM��ʼ�� */
    my_mem_init(SRAMEX);                /* ��ʼ���ⲿSRAM�ڴ�� */
		// �������˾���ת������
		freertos_demo();                    /* ����lwIP�������� */
}
