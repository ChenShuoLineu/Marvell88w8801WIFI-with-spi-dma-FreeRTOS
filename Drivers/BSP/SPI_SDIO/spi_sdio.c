#include "./BSP/SPI_SDIO/spi_sdio.h"


//marvell说白了用的底层还是SPI_SDIO 所以要先封装一下
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;
static SPI_HandleTypeDef g_spi_handler; 					/* 用到的SPI的句柄 */
static xSemaphoreHandle   dma_finish_semaphore;		//dma完成的标识

volatile uint8_t spi_dma_done = 0;

uint8_t sdio_func_num  = 0;							//分区
static uint16_t sdio_block_size[4]; 		// 每个区一次最多读多少字节

#define DUMMY_SIZE  (1024)
static uint8_t dummy_buf[DUMMY_SIZE];						// 大可放心 每次最多512byte

// 配置GPIO引脚初始化SPI
void spi_sdio_gpio_configuration()
{
	
	// 使能DMA1时钟
    __HAL_RCC_DMA1_CLK_ENABLE();
		hdma_tx.Instance = DMA1_Stream4; // 根据实际硬件选择合适的DMA流
    hdma_tx.Init.Channel = DMA_CHANNEL_0; // 根据硬件手册选择合适的通道
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;                   // 启用循环模式 DMA_CIRCULAR
		hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	
		HAL_DMA_Init(&hdma_tx);


    // 关联DMA句柄到SPI的TX
		__HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TC | DMA_IT_TE); // 启用传输完成中断
		__HAL_DMA_CLEAR_FLAG(&hdma_tx,DMA_IT_TC |DMA_IT_TE);	
    __HAL_LINKDMA(&g_spi_handler, hdmatx, hdma_tx);
	
		// TX（发送）DMA中断优先级更高
		HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 4, 0); // 主优先级 0，子优先级 0
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
		
		
		// 设置接收的
		
		// 配置 RX DMA
		hdma_rx.Instance = DMA1_Stream3; // 根据实际硬件选择合适的 DMA 流 (SPI2 RX 为 DMA1_Stream3)
		hdma_rx.Init.Channel = DMA_CHANNEL_0; // 根据硬件手册选择合适的通道
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY; // DMA 传输方向：外设到内存
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE; // 外设地址不递增
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE; // 内存地址递增
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // 外设数据对齐：字节对齐
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE; // 内存数据对齐：字节对齐
		hdma_rx.Init.Mode = DMA_NORMAL; // 启用普通模式，如果需要循环模式可以修改为 DMA_CIRCULAR
		hdma_rx.Init.Priority = DMA_PRIORITY_LOW; // 设置 DMA 的优先级（可以根据实际情况调整）
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE; // 禁用 FIFO 模式

		// 初始化 RX DMA
		HAL_DMA_Init(&hdma_rx);

		// 关联 DMA 句柄到 SPI2 的 RX
		__HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_TC | DMA_IT_TE); // 启用传输完成中断（TC）和传输错误中断（TE）
		__HAL_DMA_CLEAR_FLAG(&hdma_rx, DMA_IT_TC | DMA_IT_TE); // 清除中断标志
		__HAL_LINKDMA(&g_spi_handler, hdmarx, hdma_rx); // 关联 SPI2 的 RX DMA 句柄

		// 设置 RX（接收）DMA 中断优先级
		HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 4, 0); // 主优先级 0，子优先级 0（确保 TX 和 RX 中断优先级设置合理）
		HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn); // 启用 DMA1 Stream3 IRQ（RX DMA 中断）
		
		SD_SPI_CLK(); // 使能SPI2时钟
		SD_SPI_SCK_GPIO_CLK(); //使能CLK对应的时钟
		SD_SPI_MISO_GPIO_CLK();
		SD_SPI_MOSI_GPIO_CLK();
		SD_CS_GPIO_CLK();
	
		SD_IRQ_GPIO_CLK();
		SD_PDN_GPIO_CLK();
	
	    /* 配置CRC外设时钟 */
    __HAL_RCC_CRC_CLK_ENABLE();
    

	
    GPIO_InitTypeDef GPIO_InitStruct;
	
		//配置PDN--是否上电 输出
		GPIO_InitStruct.Pin = SD_PDN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_PDN_GPIO_PORT, &GPIO_InitStruct);
		SD_PDN(0);
		//配置SPI IRQ -- 使用该信号。
		//主机可以通过检测SPI_IRQ引脚的状态来知道Wi-Fi模块是否有数据需要处理
		GPIO_InitStruct.Pin = SD_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; //改成上拉试一试 不行就得是下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_IRQ_GPIO_PORT, &GPIO_InitStruct);
	
		// 配置SPI的NSS引脚
    GPIO_InitStruct.Pin = SD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);
		SD_CS(1);
		
    // 配置MISO引脚
    GPIO_InitStruct.Pin = SD_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(SD_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);
    /* MISO引脚模式设置(复用输出) */
    GPIO_InitStruct.Pin = SD_SPI_MOSI_PIN;
    HAL_GPIO_Init(SD_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
    /* SCK引脚模式设置(复用输出) */
    GPIO_InitStruct.Pin = SD_SPI_SCK_PIN;
		HAL_GPIO_Init(SD_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
		
	 __HAL_SPI_ENABLE(&g_spi_handler); /* 使能SPI */
    g_spi_handler.Instance = SD_SPI;                                	/* SPI2 */
    g_spi_handler.Init.Mode = SPI_MODE_MASTER;                        /* 设置SPI工作模式，设置为主模式 */
    g_spi_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* 设置SPI单向或者双向的数据模式:SPI设置为双线模式 */
    g_spi_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* 设置SPI的数据大小:SPI发送接收8位帧结构 */
    g_spi_handler.Init.CLKPolarity = SPI_POLARITY_HIGH;               /* 串行同步时钟的空闲状态为高电平 */
    g_spi_handler.Init.CLKPhase = SPI_PHASE_2EDGE;                    /* 串行同步时钟的第二个跳变沿（上升或下降）数据被采样 */
    g_spi_handler.Init.NSS = SPI_NSS_SOFT ;                            /* NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制 */
    g_spi_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* 定义波特率预分频的值:波特率预分频值为256 */
    g_spi_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始 */
    g_spi_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* 关闭TI模式 */
		g_spi_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; /* 启用CRC校验 */
		g_spi_handler.Init.CRCPolynomial = 0x1021;                   /* CRC-16-CCITT多项式 */
    HAL_SPI_Init(&g_spi_handler);                                     /* 初始化 */


}

/**
 * @brief       SPI1读写一个字节数据
 * @param       txdata  : 要发送的数据(1字节)
 * @retval      接收到的数据(1字节)
 */
uint8_t sd_spi_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0x00;
		spi_dma_done = 0;
	//	xQueueReset(dma_finish_semaphore);  // 重置信号量;
    HAL_SPI_TransmitReceive_DMA(&g_spi_handler, &txdata, &rxdata, 1);
		while(spi_dma_done == 0)
		{
		}
    xSemaphoreTake(dma_finish_semaphore, portMAX_DELAY);		// 等待finiish
    return rxdata; /* 返回收到的数据 */
}
//问题就在这里 假设有一个现在被选中了
// 然后正运行着被打断了 
// 又尝试选中就全乱了
static uint8_t sd_wait_ready(void)
{
    uint32_t t = 0;
    do
    {  
        if (sd_spi_read_write_byte(0XFF) == 0XFF)
        {
            return SD_OK;   /* OK */
        }

        t++;
    } while (t < 0XFFFF); /* 等待 */

    return SD_ERROR;
}


/**
 * @brief       等待SD卡回应
 * @param       response : 期待得到的回应值
 * @retval      等待结果
 *              SD_OK,      成功
 *              SD_ERROR,   失败
 */
static int sd_get_receive_response(uint8_t *resp, uint8_t resp_len)
{
		int i = 0;
		uint8_t temp;
		
		do
		{
			if (i == 200)
			{
				printf("命令的响应超时了\r\n");
				return SD_TIMEOUT;
			}
			i++;
			
			temp = sd_spi_read_write_byte(0xff);
		} while (temp & 0x80); //0x80就代表收到消息了 命令的响应
		
		resp[0] = temp;
		for (i = 1; i < resp_len; i++)
			resp[i] = sd_spi_read_write_byte(0xff);
		return SD_OK;
}
static void sd_deselect(void)
{
    SD_CS(1);                       /* 取消SD卡片选 */
    sd_spi_read_write_byte(0xff);   /* 提供额外的8个时钟 */
}

/**
 * @brief       SD卡 选中, 并等待卡准备OK
 * @param       无
 * @retval      选中结果
 *              SD_OK,      成功
 *              SD_ERROR,   失败
 */
static uint8_t sd_select(void)
{
    SD_CS(0);
		// 还真得有这个 没有就GG了
    if (sd_wait_ready() == 0)
    {
        return SD_OK;   // 等待成功 
    }
		printf("选中失败\r\n");
    sd_deselect();
    return SD_ERROR;    /* 等待失败 */
}

static int sd_send_cmd(uint8_t cmd, uint32_t arg,uint8_t *resp,uint8_t resp_len)
{
		//__disable_irq(); // 关闭总中断
    int res;
    uint8_t crc = 0X01;     /* 默认 CRC = 忽略CRC + 停止 */
		memset(resp,0x00,resp_len);

	
    sd_deselect();      /* 取消上次片选 */
    if (sd_select()){
						printf("在send cmd中发生了选中失败\r\n");
            return 0xFF;    /* 选中失败 */
     }

    // Prepare the data array
    uint8_t data[5];
    data[0] = cmd | 0x40;       // Command byte
    data[1] = (arg >> 24) & 0xFF;  // Argument byte 1
    data[2] = (arg >> 16) & 0xFF;  // Argument byte 2
    data[3] = (arg >> 8) & 0xFF;   // Argument byte 3
    data[4] = arg & 0xFF;          // Argument byte 4
		for(uint8_t i = 0; i < 5; i ++)
			sd_spi_read_write_byte(data[i]);

    crc = WiFi_LowLevel_CalcCRC7(data, 5); // Calculate CRC7 for the command and argument bytes
		crc = (crc << 1) | 1; //这就是数据
    // Send the CRC7 value
    if (cmd == CMD0) crc = 0X95;        /* CMD0 的CRC值固定为 0X95 */

    if (cmd == CMD8) crc = 0X87;        /* CMD8 的CRC值固定为 0X87 */

    sd_spi_read_write_byte(crc);

    if (cmd == CMD12)   /* cmd 等于 多块读结束命令(CMD12)时 */
    {
        sd_spi_read_write_byte(0xff);   /* CMD12 跳过一个字节 */
    }

			
		res = sd_get_receive_response(resp,resp_len); 	//接收到的消息有两种 要么超时-1 要么正常0
		configASSERT(res != SD_TIMEOUT);
		//__enable_irq(); 
		return res; /* 返回状态值 */
}


unsigned char sd_init()
{
    uint16_t retry;     /*  用来进行超时计数 */
    uint8_t ocr[10];
	
		spi_sdio_gpio_configuration();//初始化SPI的GPIO引脚
    sd_spi_speed_low(); /* 设置到低速模式 注：初始化时,时钟频率<400kHz */
		dma_finish_semaphore = xSemaphoreCreateBinary();  // 最大值2，初始值0
	
		UBaseType_t count = uxSemaphoreGetCount(dma_finish_semaphore);
    // 打印计数值
    printf("Current semaphore count: %u\n", (unsigned int)count);
	
    retry = 10;
		//一旦通信的时候发现CS = 0就知道SPI了
    do
    {
				SD_PDN(0);
				delay_ms(10);
				SD_PDN(1);
				delay_ms(100);
        /* 重置SD卡进入默认状态，如果返回值为0x01(R1响应的in idle state为1)，则表示SD卡复位成功 */
				sd_send_cmd(CMD0, 0,ocr,1);         //res要么是timeOut 要么就是收到的长度
    } while ((ocr[0] != 0X01) && retry--);
		printf("%c %c\r\n", ocr[0],ocr[1]);
		if(ocr[0] == 0x01)
		{
				//printf("找到了\r\n");
			    /* 发送cmd5 */
				sd_send_cmd(CMD5,0,ocr,sizeof(ocr));
				// 重新上电就可以 但是不重新上电就不行
				//PDN这条线的控制有问题 明明初始化好了插上就GG了
				// 不插上就得重新断电
				printf("CMD5, R1_%02X, RESP1_:%02x %02x %02x %02x\r\n", ocr[0], ocr[1], ocr[2], ocr[3], ocr[4]);

				/* ocr 3.2V~3.4V*/
				//查看电压支持不支持
				sd_send_cmd(CMD5,0x300000,ocr,sizeof(ocr));
				printf("CMD5_VOL, R1_%02X, RESP1_:%02x %02x %02x %02x\r\n", ocr[0], ocr[1], ocr[2], ocr[3], ocr[4]);
				// 没有找到R4是MAarvell88w8801定义的
			
				if (ocr[1] & _BV(7)) //最高位是不是1
				{
					// Card is ready to operate after initialization
					sdio_func_num = (ocr[1] >> 4) & 7; //高8位的低三位就是func_num
					printf("Number of I/O Functions: %d\r\n", sdio_func_num);
					printf("Memory Present: %d\r\n", (ocr[1] & _BV(3)) != 0);
					return sdio_func_num;
				}else
					printf("最高位不是1\r\n");
		
		}
		else
			printf("GG了超时也没找到\r\n");
		//SPI模式不需要CMD3 和CMD7 因为内是SD模式在选中卡 但是SPI靠的是CS引脚
		
		return 0;
}



// 设置速度
void spi_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed)); /* 判断有效性 */
    __HAL_SPI_DISABLE(&g_spi_handler);             /* 关闭SPI */
    g_spi_handler.Instance->CR1 &= 0XFFC7;         /* 位3-5清零，用来设置波特率 */
    g_spi_handler.Instance->CR1 |= speed << 3;     /* 设置SPI速度 */
    __HAL_SPI_ENABLE(&g_spi_handler);              /* 使能SPI */
}

int WiFi_LowLevel_GetITStatus(uint8_t clear)
{
		//有中断发生返回1 没有返回0
		return !INT;
}


static void sd_spi_write_bytes(uint8_t* txdata,uint16_t len)
{
		if(txdata == NULL || len == 0)
				return;
		//xQueueReset(dma_finish_semaphore);  // 重置信号量
		HAL_SPI_TransmitReceive_DMA(&g_spi_handler,txdata,dummy_buf,len);
//		uint32_t count = 0;
		xSemaphoreTake(dma_finish_semaphore, portMAX_DELAY);
//		while (HAL_SPI_GetState(&g_spi_handler) != HAL_SPI_STATE_READY)	 	//按理来说是瞬间完成的
//    {
//        // 等待SPI DMA传输完成
//				//count++;
//				count++;
//				if(count > 0xffff)
//				{
//						printf("发生溢出都没有准备好 写出问题了\r\n");
//						break;
//				}			
//    }
}


static void sd_spi_read_bytes(uint8_t* rxdata,uint16_t len)
{
		if(rxdata == NULL || len == 0)
				return;
		//xQueueReset(dma_finish_semaphore);  // 重置信号量
		HAL_SPI_TransmitReceive_DMA(&g_spi_handler,dummy_buf,rxdata,len);
		xSemaphoreTake(dma_finish_semaphore, portMAX_DELAY);
//		uint32_t count = 0;
//		while (HAL_SPI_GetState(&g_spi_handler) != HAL_SPI_STATE_READY)
//    {
//				count++;
//				if(count > 0xffff)
//				{
//						printf("发生溢出都没有准备好 读出问题了\r\n");
//						break;
//				}		
//    }

}




// 返回全局变量
uint8_t WiFi_LowLevel_GetFunctionNum(void)
{
  return sdio_func_num;
}


// marvell专属命令CMD52 read/write
// 强制的格式要求了
// CMD52的格式
void WiFi_LowLevel_SendCMD52(uint8_t func, uint32_t addr, uint8_t data,
	uint32_t flags, uint8_t *resp, uint8_t resp_len)
{
		// 31位 R/W FLAG:read 还是 write
		// 30 29 28 fun num:那一类function
		// 27 RAW flag  是否允许写入寄存器后直接读取寄存器装填 这关系到返回值
		// 不设置的话 写入后 的response 和你写入的一样（实际可能不一样）
		// 25-9 寄存器的地址
		// 7-0 如果是写入的话就是要写入的数据 要是读的话就随便乱写
		uint32_t arg = (func << 28) | (addr << 9) | data | flags;
		// 响应是16位长度的
		sd_send_cmd(CMD52, arg, resp, resp_len);
}

//CMD53 还没用到不知道要干嘛
//主要用于对某一个地址进行多数据写入或者多数据读出的CMD，
//此命令分为两种模式：
//block mode和byte mode，block mode必须是写入或者读出block size整数倍的数据，
//byte mode是可以写入或者读出任意大小的数据

static int WiFi_LowLevel_SendCMD53(uint8_t func, uint32_t addr, uint16_t count,
	uint32_t flags, uint8_t *resp, uint8_t resp_len)
{
	// 32位是需要我们设置的
	// 31位 R/W flag
	// 30 29 28：Fun Num
	//	27:Bolck Mode: 为1的话，此时读取写入以块而不是字节为基本单位
	// 块的大小 对于Fun1 - 7写入FBR中的IO块大小寄存器的
	//					对于Fun0 	写入CCCR中的FN0块大小寄存器
	// 设置之前先读取CCCR中的位判定是否支持
	// 26： OP Mode：
	//     	0代表只从一个寄存器地址传送数据 FIFO是
	//			1代表地址自增 当RAM等缓存区存在大量数据使用此命令 地址范围  [1FFFFh:0]
	// 25 - 9:address 地址
	// 8-0: count:如果不是块模式 此字段是要读取或者写入的字节数， 000h就是512字节
	//						如果是块模式 那就是数据块数 000h就是无限 此时只能通过CCCR中的
	//						I/O中止函数来做  
	
	//这里0x1ff是因为最大count就这么大了可不能再大了
  uint32_t arg = (func << 28) | (addr << 9) | (count & 0x1ff) | flags;
  // 只要是-1就是GG了 0就是正常
	int res = sd_send_cmd(53, arg, resp, resp_len);
  return res;
}


// 读取对应的内个IO寄存器
uint8_t WiFi_LowLevel_ReadReg(uint8_t func, uint32_t addr)
{
		uint8_t resp[2];
		//分析一下就是 读取 func 位置的addr函数 至于RAW随便无所谓
		WiFi_LowLevel_SendCMD52(func, addr, 0, 0, resp, sizeof(resp));
	//响应一共16位 前8位是各种flag 后8位就是read和right的数据了
		return resp[1];
}
// 写入内个IO寄存器
uint8_t WiFi_LowLevel_WriteReg(uint8_t func, uint32_t addr, uint8_t value)
{
		uint8_t resp[2];
		// 所以这里flag就好理解了 首先最高位得是读 CMD52_WRITE
		// 为了保证读的正确性就得有CMD52_READAFTERWRITE 响应的正确性
		WiFi_LowLevel_SendCMD52(func, addr, value, CMD52_WRITE | CMD52_READAFTERWRITE, resp, sizeof(resp));
		return resp[1];
}

// 设置块的大小
int WiFi_LowLevel_SetBlockSize(uint8_t func, uint32_t size)
{
  sdio_block_size[func] = size;
  WiFi_LowLevel_WriteReg(0, (func << 8) | 0x10, size & 0xff);
  WiFi_LowLevel_WriteReg(0, (func << 8) | 0x11, size >> 8);
  return 1;
}

//得到有多少块
static uint16_t WiFi_LowLevel_GetBlockNum(uint8_t func, uint32_t *psize, uint32_t flags)
{
		uint16_t block_num = 0;
		
		//看看flag是不是允许多块的
		if ((flags & WIFI_RWDATA_ALLOWMULTIBYTE) == 0 || *psize > 512) 
		{
			
			block_num = *psize / sdio_block_size[func];
			if (*psize % sdio_block_size[func] != 0)
				block_num++;
			// 把大小也改了
			*psize = block_num * sdio_block_size[func];
		}
		else
		{
			//说白了就是内存对齐到4的倍数
			*psize = (*psize + 3) & ~3; 
		}
		
		return block_num;
}
// 发送数据用的 和命令不一样
static void WiFi_LowLevel_Send(void *data, uint16_t len)
{
		sd_spi_write_bytes((uint8_t*)data,len);

}
//纯粹的接收
static void WiFi_LowLevel_Receive(void *data, uint16_t len)
{
		sd_spi_read_bytes((uint8_t*)data,len);
}






uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length) {
        uint16_t crc = 0x0000;
    uint8_t i;

    while (length--) {
        crc ^= *data++ << 8;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // CRC-16-CCITT-FALSE
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t WiFi_LowLevel_CalcCRC7(const void *data, int len)
{
		const uint8_t *p = data;
		int i, j;
		uint16_t temp = 0;

		if (len != 0)
			temp = p[0] << 8;

		for (i = 1; i <= len; i++)
		{
			if (i != len)
				temp |= p[i];
			for (j = 0; j < 8; j++)
			{
				if (temp & 0x8000)
					temp ^= POLYNOMIAL_CRC7 << 8;
				temp <<= 1;
			}
		}
		return temp >> 9;
}



int WiFi_LowLevel_WriteData(uint8_t func, uint32_t addr, 
	void *data, uint32_t size, 
	uint32_t bufsize, uint32_t flags)
{
		if(sd_select())
		{
			printf("在WiFi_LowLevel_WriteData选中失败\r\n");
			return 2;
		}
		uint8_t  *p = (uint8_t * )data;
		int ret;
		char err = 0;
		uint8_t resp[2];
		uint16_t block_num; 
		uint16_t crc = 0xffff;
		uint32_t curr;
		uint32_t cmd53_flags = CMD53_WRITE;

		if ((uintptr_t)data & 3)
		{
			printf("%s: data must be 4-byte aligned!\n", __FUNCTION__);
			sd_deselect();

			return 2; 
		}
		if (size == 0)
		{
			printf("%s: size cannot be 0!\n", __FUNCTION__);
			sd_deselect();

			return 2;
		}
		// 大小都不够用块模式就不用了 这里的话把size改了会不会有问题呢
		block_num = WiFi_LowLevel_GetBlockNum(func, &size, flags);
		if (bufsize != 0 && bufsize < size) 
			printf("%s: a buffer of at least %d bytes is required! bufsize=%d\r\n", __FUNCTION__, size, bufsize);
		
		if (flags & WIFI_RWDATA_ADDRINCREMENT)
		{
			//printf("地址需要自增\r\n");
			cmd53_flags |= CMD53_INCREMENTING; //地址要不要自增
		}
			//printf("地址不需要自增\r\n");
		// 启动块模式
		if (block_num)
			ret = WiFi_LowLevel_SendCMD53(func, addr, block_num, cmd53_flags | CMD53_BLOCKMODE, resp, sizeof(resp));
		// 按字节发送
		else
			ret = WiFi_LowLevel_SendCMD53(func, addr, size, cmd53_flags, resp, sizeof(resp));
		if (ret == SD_TIMEOUT)
		{
			sd_deselect();
			printf("返回值是超时寄寄了\r\n");

			return 0;
		}
		sd_spi_read_write_byte(0xff); // NWR 可能就是占位？
		while (size > 0 && err == 0)
		{
			
				//printf("当前是writedata循环的%d\r\n",count++);
				curr = (block_num) ? sdio_block_size[func] : size;
				//printf("在write_data中发送的curr是%d block_size:%d size:%d\r\n",curr,sdio_block_size[func],size);
				//块的开始标志
				sd_spi_read_write_byte(0xfc); // start block token
				WiFi_LowLevel_Send(p, curr);
				crc = CRC16_CCITT(p, curr); //这个CRC校验 在发TCP数据的时候可以
				sd_spi_read_write_byte((crc >> 8) & 0xFF); // 提取高8位
				sd_spi_read_write_byte(crc & 0xff);				//提取低8位
				resp[0] = sd_spi_read_write_byte(0xff); // start block token
				//实际上你会发现crc校验还是失败的
				//现在就是发送过去了
				// 用硬件SPI就会变得非常慢..
				//e5是正常 eb是crc校验错误 但是错误也能用？
			 if(resp[0] == 0xe5 || resp[0] == 0xeb)
			 {
					//printf("resp is %2x\r\n",resp[0]); //一直都校验错误
					p += curr;
					size -= curr;
			 }
			 else
			 {
					 printf("传输错误%2x 此时curr为%d\r\n",resp[0],curr);
						sd_deselect();

					 return 2;
			 }

		}
		sd_deselect();

		return err == 0;
}



// size是接受的大小
// buffer_size整个缓存区的大小
// flag意思就是用不用多块的方式
int WiFi_LowLevel_ReadData(uint8_t func, uint32_t addr, void *data, 
	uint32_t size, uint32_t bufsize, uint32_t flags)
{
		if(sd_select())
		{
			printf("在WiFi_LowLevel_ReadData选中失败\r\n");
//			__enable_irq(); 
			return 2;
		}
		int err = 0, i;
		uint8_t *p = data;
		uint8_t resp;
		uint16_t block_num; 
		uint16_t crc[2], curr;
		uint32_t cmd53_flags = 0;
	
		
		if ((uintptr_t)data & 3)
		{
			printf("%s: data must be 4-byte aligned!\n", __FUNCTION__);
			sd_deselect();
			return 2; 
		}
		if (size == 0)
		{
			printf("%s: size cannot be 0!\n", __FUNCTION__);
			sd_deselect();
			return 2;
		}
		// 这个flag主要意思是是按多块发送还是按单个的发送
		block_num = WiFi_LowLevel_GetBlockNum(func, &size, flags);
		if (bufsize != 0 && bufsize < size)
		{
			printf("%s: a buffer of at least %d bytes is required! bufsize=%d\n", __FUNCTION__, size, bufsize);
			sd_deselect();
			return 2;
		}
		
		if (flags & WIFI_RWDATA_ADDRINCREMENT)
			cmd53_flags |= CMD53_INCREMENTING;
		
		//看看数据是不是大到需要分块读
		if (block_num)
			WiFi_LowLevel_SendCMD53(func, addr, block_num, cmd53_flags | CMD53_BLOCKMODE, &resp, 1);
		else
			WiFi_LowLevel_SendCMD53(func, addr, size, cmd53_flags, &resp, 1);
		//printf("block num is%d,ret is %d\r\n",block_num,ret);
		//printf("resp is 0x%2x\r\n",resp);//0x08 说明resp是crc failed
		for (i = 0; size > 0; i++)
		{
				//为什么只有i不等于0的时候才会读呢
				if (i != 0)
				{
						resp = sd_spi_read_write_byte(0xff);
//						printf("当前是size%d\r\n,resp是%2x",size,resp);
						if (resp != 0xfe)
						{
							err = 1;
							printf("%s: invalid start of block!\n", __FUNCTION__);
							break;
						}
				}
				//到底要不要保存
				curr = (block_num) ? sdio_block_size[func] : size;
				if (bufsize != 0)
				{
					//把数据保存下来
					
					WiFi_LowLevel_Receive(p, curr);
					//printf("保存数据 %d\r\n",curr);
					/*
					if (i == 0)
						p[0] = realsize & 0xff;*/ //太奇怪了这一步看看接收到的是啥
				}
				else
					WiFi_LowLevel_Receive(NULL, curr);
				//两位CRC校验位接受一下
				WiFi_LowLevel_Receive(&crc[0], 2);
			
				if (bufsize != 0)
				{
					p += curr;
				}
			
				size -= curr;
		}
		
		sd_deselect();	//取消片选
		return err == 0;
}

// DMA发送完成中断处理函数
void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tx);
}

// DMA接收完成中断处理函数 当接收完成的时候就会被调用然后可能会产生任务的切换
void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_rx);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SD_SPI) {
        spi_dma_done = 1;  // 设置标志，表示DMA完成
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				xSemaphoreGiveFromISR(dma_finish_semaphore,&xHigherPriorityTaskWoken);/* 释放二值信号量 */
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
