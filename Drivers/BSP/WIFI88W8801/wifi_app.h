#ifndef __WIFI_APP_H
#define __WIFI_APP_H
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"


#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"

#include "./BSP/ringBuffer/ringbuffer.h"
#include "wifi.h"
#include "./BSP/SPI_SDIO/spi_sdio.h"
#include "./MALLOC/malloc.h"
#include "lwip_demo.h"

typedef enum
{
		WIFI_NONE = 0,
		WIFI_IRQ = 1,
		WIFI_SEND,
		WIFI_RECV
}WIFI_APP_STATE;
typedef struct {
		int init_status;
		WIFI_APP_STATE wifi_status;
}wifi_msg;


void wifi_init(void);

#endif
