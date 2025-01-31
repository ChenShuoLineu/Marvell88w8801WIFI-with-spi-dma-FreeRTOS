#include "./BSP/SPI_SDIO/spi_sdio.h"


//marvell˵�����õĵײ㻹��SPI_SDIO ����Ҫ�ȷ�װһ��
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;
static SPI_HandleTypeDef g_spi_handler; 					/* �õ���SPI�ľ�� */
static xSemaphoreHandle   dma_finish_semaphore;		//dma��ɵı�ʶ

volatile uint8_t spi_dma_done = 0;

uint8_t sdio_func_num  = 0;							//����
static uint16_t sdio_block_size[4]; 		// ÿ����һ�����������ֽ�

#define DUMMY_SIZE  (1024)
static uint8_t dummy_buf[DUMMY_SIZE];						// ��ɷ��� ÿ�����512byte

// ����GPIO���ų�ʼ��SPI
void spi_sdio_gpio_configuration()
{
	
	// ʹ��DMA1ʱ��
    __HAL_RCC_DMA1_CLK_ENABLE();
		hdma_tx.Instance = DMA1_Stream4; // ����ʵ��Ӳ��ѡ����ʵ�DMA��
    hdma_tx.Init.Channel = DMA_CHANNEL_0; // ����Ӳ���ֲ�ѡ����ʵ�ͨ��
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;                   // ����ѭ��ģʽ DMA_CIRCULAR
		hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	
		HAL_DMA_Init(&hdma_tx);


    // ����DMA�����SPI��TX
		__HAL_DMA_ENABLE_IT(&hdma_tx, DMA_IT_TC | DMA_IT_TE); // ���ô�������ж�
		__HAL_DMA_CLEAR_FLAG(&hdma_tx,DMA_IT_TC |DMA_IT_TE);	
    __HAL_LINKDMA(&g_spi_handler, hdmatx, hdma_tx);
	
		// TX�����ͣ�DMA�ж����ȼ�����
		HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 4, 0); // �����ȼ� 0�������ȼ� 0
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
		
		
		// ���ý��յ�
		
		// ���� RX DMA
		hdma_rx.Instance = DMA1_Stream3; // ����ʵ��Ӳ��ѡ����ʵ� DMA �� (SPI2 RX Ϊ DMA1_Stream3)
		hdma_rx.Init.Channel = DMA_CHANNEL_0; // ����Ӳ���ֲ�ѡ����ʵ�ͨ��
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY; // DMA ���䷽�����赽�ڴ�
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE; // �����ַ������
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE; // �ڴ��ַ����
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // �������ݶ��룺�ֽڶ���
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE; // �ڴ����ݶ��룺�ֽڶ���
		hdma_rx.Init.Mode = DMA_NORMAL; // ������ͨģʽ�������Ҫѭ��ģʽ�����޸�Ϊ DMA_CIRCULAR
		hdma_rx.Init.Priority = DMA_PRIORITY_LOW; // ���� DMA �����ȼ������Ը���ʵ�����������
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE; // ���� FIFO ģʽ

		// ��ʼ�� RX DMA
		HAL_DMA_Init(&hdma_rx);

		// ���� DMA ����� SPI2 �� RX
		__HAL_DMA_ENABLE_IT(&hdma_rx, DMA_IT_TC | DMA_IT_TE); // ���ô�������жϣ�TC���ʹ�������жϣ�TE��
		__HAL_DMA_CLEAR_FLAG(&hdma_rx, DMA_IT_TC | DMA_IT_TE); // ����жϱ�־
		__HAL_LINKDMA(&g_spi_handler, hdmarx, hdma_rx); // ���� SPI2 �� RX DMA ���

		// ���� RX�����գ�DMA �ж����ȼ�
		HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 4, 0); // �����ȼ� 0�������ȼ� 0��ȷ�� TX �� RX �ж����ȼ����ú���
		HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn); // ���� DMA1 Stream3 IRQ��RX DMA �жϣ�
		
		SD_SPI_CLK(); // ʹ��SPI2ʱ��
		SD_SPI_SCK_GPIO_CLK(); //ʹ��CLK��Ӧ��ʱ��
		SD_SPI_MISO_GPIO_CLK();
		SD_SPI_MOSI_GPIO_CLK();
		SD_CS_GPIO_CLK();
	
		SD_IRQ_GPIO_CLK();
		SD_PDN_GPIO_CLK();
	
	    /* ����CRC����ʱ�� */
    __HAL_RCC_CRC_CLK_ENABLE();
    

	
    GPIO_InitTypeDef GPIO_InitStruct;
	
		//����PDN--�Ƿ��ϵ� ���
		GPIO_InitStruct.Pin = SD_PDN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_PDN_GPIO_PORT, &GPIO_InitStruct);
		SD_PDN(0);
		//����SPI IRQ -- ʹ�ø��źš�
		//��������ͨ�����SPI_IRQ���ŵ�״̬��֪��Wi-Fiģ���Ƿ���������Ҫ����
		GPIO_InitStruct.Pin = SD_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; //�ĳ�������һ�� ���о͵�������
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_IRQ_GPIO_PORT, &GPIO_InitStruct);
	
		// ����SPI��NSS����
    GPIO_InitStruct.Pin = SD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);
		SD_CS(1);
		
    // ����MISO����
    GPIO_InitStruct.Pin = SD_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(SD_SPI_MISO_GPIO_PORT, &GPIO_InitStruct);
    /* MISO����ģʽ����(�������) */
    GPIO_InitStruct.Pin = SD_SPI_MOSI_PIN;
    HAL_GPIO_Init(SD_SPI_MOSI_GPIO_PORT, &GPIO_InitStruct);
    /* SCK����ģʽ����(�������) */
    GPIO_InitStruct.Pin = SD_SPI_SCK_PIN;
		HAL_GPIO_Init(SD_SPI_SCK_GPIO_PORT, &GPIO_InitStruct);
		
	 __HAL_SPI_ENABLE(&g_spi_handler); /* ʹ��SPI */
    g_spi_handler.Instance = SD_SPI;                                	/* SPI2 */
    g_spi_handler.Init.Mode = SPI_MODE_MASTER;                        /* ����SPI����ģʽ������Ϊ��ģʽ */
    g_spi_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* ����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ */
    g_spi_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* ����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ */
    g_spi_handler.Init.CLKPolarity = SPI_POLARITY_HIGH;               /* ����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ */
    g_spi_handler.Init.CLKPhase = SPI_PHASE_2EDGE;                    /* ����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ����� */
    g_spi_handler.Init.NSS = SPI_NSS_SOFT ;                            /* NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ���� */
    g_spi_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* ���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256 */
    g_spi_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ */
    g_spi_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* �ر�TIģʽ */
		g_spi_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; /* ����CRCУ�� */
		g_spi_handler.Init.CRCPolynomial = 0x1021;                   /* CRC-16-CCITT����ʽ */
    HAL_SPI_Init(&g_spi_handler);                                     /* ��ʼ�� */


}

/**
 * @brief       SPI1��дһ���ֽ�����
 * @param       txdata  : Ҫ���͵�����(1�ֽ�)
 * @retval      ���յ�������(1�ֽ�)
 */
uint8_t sd_spi_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0x00;
		spi_dma_done = 0;
	//	xQueueReset(dma_finish_semaphore);  // �����ź���;
    HAL_SPI_TransmitReceive_DMA(&g_spi_handler, &txdata, &rxdata, 1);
		while(spi_dma_done == 0)
		{
		}
    xSemaphoreTake(dma_finish_semaphore, portMAX_DELAY);		// �ȴ�finiish
    return rxdata; /* �����յ������� */
}
//����������� ������һ�����ڱ�ѡ����
// Ȼ���������ű������ 
// �ֳ���ѡ�о�ȫ����
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
    } while (t < 0XFFFF); /* �ȴ� */

    return SD_ERROR;
}


/**
 * @brief       �ȴ�SD����Ӧ
 * @param       response : �ڴ��õ��Ļ�Ӧֵ
 * @retval      �ȴ����
 *              SD_OK,      �ɹ�
 *              SD_ERROR,   ʧ��
 */
static int sd_get_receive_response(uint8_t *resp, uint8_t resp_len)
{
		int i = 0;
		uint8_t temp;
		
		do
		{
			if (i == 200)
			{
				printf("�������Ӧ��ʱ��\r\n");
				return SD_TIMEOUT;
			}
			i++;
			
			temp = sd_spi_read_write_byte(0xff);
		} while (temp & 0x80); //0x80�ʹ����յ���Ϣ�� �������Ӧ
		
		resp[0] = temp;
		for (i = 1; i < resp_len; i++)
			resp[i] = sd_spi_read_write_byte(0xff);
		return SD_OK;
}
static void sd_deselect(void)
{
    SD_CS(1);                       /* ȡ��SD��Ƭѡ */
    sd_spi_read_write_byte(0xff);   /* �ṩ�����8��ʱ�� */
}

/**
 * @brief       SD�� ѡ��, ���ȴ���׼��OK
 * @param       ��
 * @retval      ѡ�н��
 *              SD_OK,      �ɹ�
 *              SD_ERROR,   ʧ��
 */
static uint8_t sd_select(void)
{
    SD_CS(0);
		// ���������� û�о�GG��
    if (sd_wait_ready() == 0)
    {
        return SD_OK;   // �ȴ��ɹ� 
    }
		printf("ѡ��ʧ��\r\n");
    sd_deselect();
    return SD_ERROR;    /* �ȴ�ʧ�� */
}

static int sd_send_cmd(uint8_t cmd, uint32_t arg,uint8_t *resp,uint8_t resp_len)
{
		//__disable_irq(); // �ر����ж�
    int res;
    uint8_t crc = 0X01;     /* Ĭ�� CRC = ����CRC + ֹͣ */
		memset(resp,0x00,resp_len);

	
    sd_deselect();      /* ȡ���ϴ�Ƭѡ */
    if (sd_select()){
						printf("��send cmd�з�����ѡ��ʧ��\r\n");
            return 0xFF;    /* ѡ��ʧ�� */
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
		crc = (crc << 1) | 1; //���������
    // Send the CRC7 value
    if (cmd == CMD0) crc = 0X95;        /* CMD0 ��CRCֵ�̶�Ϊ 0X95 */

    if (cmd == CMD8) crc = 0X87;        /* CMD8 ��CRCֵ�̶�Ϊ 0X87 */

    sd_spi_read_write_byte(crc);

    if (cmd == CMD12)   /* cmd ���� ������������(CMD12)ʱ */
    {
        sd_spi_read_write_byte(0xff);   /* CMD12 ����һ���ֽ� */
    }

			
		res = sd_get_receive_response(resp,resp_len); 	//���յ�����Ϣ������ Ҫô��ʱ-1 Ҫô����0
		configASSERT(res != SD_TIMEOUT);
		//__enable_irq(); 
		return res; /* ����״ֵ̬ */
}


unsigned char sd_init()
{
    uint16_t retry;     /*  �������г�ʱ���� */
    uint8_t ocr[10];
	
		spi_sdio_gpio_configuration();//��ʼ��SPI��GPIO����
    sd_spi_speed_low(); /* ���õ�����ģʽ ע����ʼ��ʱ,ʱ��Ƶ��<400kHz */
		dma_finish_semaphore = xSemaphoreCreateBinary();  // ���ֵ2����ʼֵ0
	
		UBaseType_t count = uxSemaphoreGetCount(dma_finish_semaphore);
    // ��ӡ����ֵ
    printf("Current semaphore count: %u\n", (unsigned int)count);
	
    retry = 10;
		//һ��ͨ�ŵ�ʱ����CS = 0��֪��SPI��
    do
    {
				SD_PDN(0);
				delay_ms(10);
				SD_PDN(1);
				delay_ms(100);
        /* ����SD������Ĭ��״̬���������ֵΪ0x01(R1��Ӧ��in idle stateΪ1)�����ʾSD����λ�ɹ� */
				sd_send_cmd(CMD0, 0,ocr,1);         //resҪô��timeOut Ҫô�����յ��ĳ���
    } while ((ocr[0] != 0X01) && retry--);
		printf("%c %c\r\n", ocr[0],ocr[1]);
		if(ocr[0] == 0x01)
		{
				//printf("�ҵ���\r\n");
			    /* ����cmd5 */
				sd_send_cmd(CMD5,0,ocr,sizeof(ocr));
				// �����ϵ�Ϳ��� ���ǲ������ϵ�Ͳ���
				//PDN�����ߵĿ��������� ������ʼ�����˲��Ͼ�GG��
				// �����Ͼ͵����¶ϵ�
				printf("CMD5, R1_%02X, RESP1_:%02x %02x %02x %02x\r\n", ocr[0], ocr[1], ocr[2], ocr[3], ocr[4]);

				/* ocr 3.2V~3.4V*/
				//�鿴��ѹ֧�ֲ�֧��
				sd_send_cmd(CMD5,0x300000,ocr,sizeof(ocr));
				printf("CMD5_VOL, R1_%02X, RESP1_:%02x %02x %02x %02x\r\n", ocr[0], ocr[1], ocr[2], ocr[3], ocr[4]);
				// û���ҵ�R4��MAarvell88w8801�����
			
				if (ocr[1] & _BV(7)) //���λ�ǲ���1
				{
					// Card is ready to operate after initialization
					sdio_func_num = (ocr[1] >> 4) & 7; //��8λ�ĵ���λ����func_num
					printf("Number of I/O Functions: %d\r\n", sdio_func_num);
					printf("Memory Present: %d\r\n", (ocr[1] & _BV(3)) != 0);
					return sdio_func_num;
				}else
					printf("���λ����1\r\n");
		
		}
		else
			printf("GG�˳�ʱҲû�ҵ�\r\n");
		//SPIģʽ����ҪCMD3 ��CMD7 ��Ϊ����SDģʽ��ѡ�п� ����SPI������CS����
		
		return 0;
}



// �����ٶ�
void spi_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed)); /* �ж���Ч�� */
    __HAL_SPI_DISABLE(&g_spi_handler);             /* �ر�SPI */
    g_spi_handler.Instance->CR1 &= 0XFFC7;         /* λ3-5���㣬�������ò����� */
    g_spi_handler.Instance->CR1 |= speed << 3;     /* ����SPI�ٶ� */
    __HAL_SPI_ENABLE(&g_spi_handler);              /* ʹ��SPI */
}

int WiFi_LowLevel_GetITStatus(uint8_t clear)
{
		//���жϷ�������1 û�з���0
		return !INT;
}


static void sd_spi_write_bytes(uint8_t* txdata,uint16_t len)
{
		if(txdata == NULL || len == 0)
				return;
		//xQueueReset(dma_finish_semaphore);  // �����ź���
		HAL_SPI_TransmitReceive_DMA(&g_spi_handler,txdata,dummy_buf,len);
//		uint32_t count = 0;
		xSemaphoreTake(dma_finish_semaphore, portMAX_DELAY);
//		while (HAL_SPI_GetState(&g_spi_handler) != HAL_SPI_STATE_READY)	 	//������˵��˲����ɵ�
//    {
//        // �ȴ�SPI DMA�������
//				//count++;
//				count++;
//				if(count > 0xffff)
//				{
//						printf("���������û��׼���� д��������\r\n");
//						break;
//				}			
//    }
}


static void sd_spi_read_bytes(uint8_t* rxdata,uint16_t len)
{
		if(rxdata == NULL || len == 0)
				return;
		//xQueueReset(dma_finish_semaphore);  // �����ź���
		HAL_SPI_TransmitReceive_DMA(&g_spi_handler,dummy_buf,rxdata,len);
		xSemaphoreTake(dma_finish_semaphore, portMAX_DELAY);
//		uint32_t count = 0;
//		while (HAL_SPI_GetState(&g_spi_handler) != HAL_SPI_STATE_READY)
//    {
//				count++;
//				if(count > 0xffff)
//				{
//						printf("���������û��׼���� ����������\r\n");
//						break;
//				}		
//    }

}




// ����ȫ�ֱ���
uint8_t WiFi_LowLevel_GetFunctionNum(void)
{
  return sdio_func_num;
}


// marvellר������CMD52 read/write
// ǿ�Ƶĸ�ʽҪ����
// CMD52�ĸ�ʽ
void WiFi_LowLevel_SendCMD52(uint8_t func, uint32_t addr, uint8_t data,
	uint32_t flags, uint8_t *resp, uint8_t resp_len)
{
		// 31λ R/W FLAG:read ���� write
		// 30 29 28 fun num:��һ��function
		// 27 RAW flag  �Ƿ�����д��Ĵ�����ֱ�Ӷ�ȡ�Ĵ���װ�� ���ϵ������ֵ
		// �����õĻ� д��� ��response ����д���һ����ʵ�ʿ��ܲ�һ����
		// 25-9 �Ĵ����ĵ�ַ
		// 7-0 �����д��Ļ�����Ҫд������� Ҫ�Ƕ��Ļ��������д
		uint32_t arg = (func << 28) | (addr << 9) | data | flags;
		// ��Ӧ��16λ���ȵ�
		sd_send_cmd(CMD52, arg, resp, resp_len);
}

//CMD53 ��û�õ���֪��Ҫ����
//��Ҫ���ڶ�ĳһ����ַ���ж�����д����߶����ݶ�����CMD��
//�������Ϊ����ģʽ��
//block mode��byte mode��block mode������д����߶���block size�����������ݣ�
//byte mode�ǿ���д����߶��������С������

static int WiFi_LowLevel_SendCMD53(uint8_t func, uint32_t addr, uint16_t count,
	uint32_t flags, uint8_t *resp, uint8_t resp_len)
{
	// 32λ����Ҫ�������õ�
	// 31λ R/W flag
	// 30 29 28��Fun Num
	//	27:Bolck Mode: Ϊ1�Ļ�����ʱ��ȡд���Կ�������ֽ�Ϊ������λ
	// ��Ĵ�С ����Fun1 - 7д��FBR�е�IO���С�Ĵ�����
	//					����Fun0 	д��CCCR�е�FN0���С�Ĵ���
	// ����֮ǰ�ȶ�ȡCCCR�е�λ�ж��Ƿ�֧��
	// 26�� OP Mode��
	//     	0����ֻ��һ���Ĵ�����ַ�������� FIFO��
	//			1�����ַ���� ��RAM�Ȼ��������ڴ�������ʹ�ô����� ��ַ��Χ  [1FFFFh:0]
	// 25 - 9:address ��ַ
	// 8-0: count:������ǿ�ģʽ ���ֶ���Ҫ��ȡ����д����ֽ����� 000h����512�ֽ�
	//						����ǿ�ģʽ �Ǿ������ݿ��� 000h�������� ��ʱֻ��ͨ��CCCR�е�
	//						I/O��ֹ��������  
	
	//����0x1ff����Ϊ���count����ô���˿ɲ����ٴ���
  uint32_t arg = (func << 28) | (addr << 9) | (count & 0x1ff) | flags;
  // ֻҪ��-1����GG�� 0��������
	int res = sd_send_cmd(53, arg, resp, resp_len);
  return res;
}


// ��ȡ��Ӧ���ڸ�IO�Ĵ���
uint8_t WiFi_LowLevel_ReadReg(uint8_t func, uint32_t addr)
{
		uint8_t resp[2];
		//����һ�¾��� ��ȡ func λ�õ�addr���� ����RAW�������ν
		WiFi_LowLevel_SendCMD52(func, addr, 0, 0, resp, sizeof(resp));
	//��Ӧһ��16λ ǰ8λ�Ǹ���flag ��8λ����read��right��������
		return resp[1];
}
// д���ڸ�IO�Ĵ���
uint8_t WiFi_LowLevel_WriteReg(uint8_t func, uint32_t addr, uint8_t value)
{
		uint8_t resp[2];
		// ��������flag�ͺ������ �������λ���Ƕ� CMD52_WRITE
		// Ϊ�˱�֤������ȷ�Ծ͵���CMD52_READAFTERWRITE ��Ӧ����ȷ��
		WiFi_LowLevel_SendCMD52(func, addr, value, CMD52_WRITE | CMD52_READAFTERWRITE, resp, sizeof(resp));
		return resp[1];
}

// ���ÿ�Ĵ�С
int WiFi_LowLevel_SetBlockSize(uint8_t func, uint32_t size)
{
  sdio_block_size[func] = size;
  WiFi_LowLevel_WriteReg(0, (func << 8) | 0x10, size & 0xff);
  WiFi_LowLevel_WriteReg(0, (func << 8) | 0x11, size >> 8);
  return 1;
}

//�õ��ж��ٿ�
static uint16_t WiFi_LowLevel_GetBlockNum(uint8_t func, uint32_t *psize, uint32_t flags)
{
		uint16_t block_num = 0;
		
		//����flag�ǲ����������
		if ((flags & WIFI_RWDATA_ALLOWMULTIBYTE) == 0 || *psize > 512) 
		{
			
			block_num = *psize / sdio_block_size[func];
			if (*psize % sdio_block_size[func] != 0)
				block_num++;
			// �Ѵ�СҲ����
			*psize = block_num * sdio_block_size[func];
		}
		else
		{
			//˵���˾����ڴ���뵽4�ı���
			*psize = (*psize + 3) & ~3; 
		}
		
		return block_num;
}
// ���������õ� �����һ��
static void WiFi_LowLevel_Send(void *data, uint16_t len)
{
		sd_spi_write_bytes((uint8_t*)data,len);

}
//����Ľ���
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
			printf("��WiFi_LowLevel_WriteDataѡ��ʧ��\r\n");
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
		// ��С�������ÿ�ģʽ�Ͳ����� ����Ļ���size���˻᲻����������
		block_num = WiFi_LowLevel_GetBlockNum(func, &size, flags);
		if (bufsize != 0 && bufsize < size) 
			printf("%s: a buffer of at least %d bytes is required! bufsize=%d\r\n", __FUNCTION__, size, bufsize);
		
		if (flags & WIFI_RWDATA_ADDRINCREMENT)
		{
			//printf("��ַ��Ҫ����\r\n");
			cmd53_flags |= CMD53_INCREMENTING; //��ַҪ��Ҫ����
		}
			//printf("��ַ����Ҫ����\r\n");
		// ������ģʽ
		if (block_num)
			ret = WiFi_LowLevel_SendCMD53(func, addr, block_num, cmd53_flags | CMD53_BLOCKMODE, resp, sizeof(resp));
		// ���ֽڷ���
		else
			ret = WiFi_LowLevel_SendCMD53(func, addr, size, cmd53_flags, resp, sizeof(resp));
		if (ret == SD_TIMEOUT)
		{
			sd_deselect();
			printf("����ֵ�ǳ�ʱ�ļ���\r\n");

			return 0;
		}
		sd_spi_read_write_byte(0xff); // NWR ���ܾ���ռλ��
		while (size > 0 && err == 0)
		{
			
				//printf("��ǰ��writedataѭ����%d\r\n",count++);
				curr = (block_num) ? sdio_block_size[func] : size;
				//printf("��write_data�з��͵�curr��%d block_size:%d size:%d\r\n",curr,sdio_block_size[func],size);
				//��Ŀ�ʼ��־
				sd_spi_read_write_byte(0xfc); // start block token
				WiFi_LowLevel_Send(p, curr);
				crc = CRC16_CCITT(p, curr); //���CRCУ�� �ڷ�TCP���ݵ�ʱ�����
				sd_spi_read_write_byte((crc >> 8) & 0xFF); // ��ȡ��8λ
				sd_spi_read_write_byte(crc & 0xff);				//��ȡ��8λ
				resp[0] = sd_spi_read_write_byte(0xff); // start block token
				//ʵ������ᷢ��crcУ�黹��ʧ�ܵ�
				//���ھ��Ƿ��͹�ȥ��
				// ��Ӳ��SPI�ͻ��÷ǳ���..
				//e5������ eb��crcУ����� ���Ǵ���Ҳ���ã�
			 if(resp[0] == 0xe5 || resp[0] == 0xeb)
			 {
					//printf("resp is %2x\r\n",resp[0]); //һֱ��У�����
					p += curr;
					size -= curr;
			 }
			 else
			 {
					 printf("�������%2x ��ʱcurrΪ%d\r\n",resp[0],curr);
						sd_deselect();

					 return 2;
			 }

		}
		sd_deselect();

		return err == 0;
}



// size�ǽ��ܵĴ�С
// buffer_size�����������Ĵ�С
// flag��˼�����ò��ö��ķ�ʽ
int WiFi_LowLevel_ReadData(uint8_t func, uint32_t addr, void *data, 
	uint32_t size, uint32_t bufsize, uint32_t flags)
{
		if(sd_select())
		{
			printf("��WiFi_LowLevel_ReadDataѡ��ʧ��\r\n");
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
		// ���flag��Ҫ��˼���ǰ���鷢�ͻ��ǰ������ķ���
		block_num = WiFi_LowLevel_GetBlockNum(func, &size, flags);
		if (bufsize != 0 && bufsize < size)
		{
			printf("%s: a buffer of at least %d bytes is required! bufsize=%d\n", __FUNCTION__, size, bufsize);
			sd_deselect();
			return 2;
		}
		
		if (flags & WIFI_RWDATA_ADDRINCREMENT)
			cmd53_flags |= CMD53_INCREMENTING;
		
		//���������ǲ��Ǵ���Ҫ�ֿ��
		if (block_num)
			WiFi_LowLevel_SendCMD53(func, addr, block_num, cmd53_flags | CMD53_BLOCKMODE, &resp, 1);
		else
			WiFi_LowLevel_SendCMD53(func, addr, size, cmd53_flags, &resp, 1);
		//printf("block num is%d,ret is %d\r\n",block_num,ret);
		//printf("resp is 0x%2x\r\n",resp);//0x08 ˵��resp��crc failed
		for (i = 0; size > 0; i++)
		{
				//Ϊʲôֻ��i������0��ʱ��Ż����
				if (i != 0)
				{
						resp = sd_spi_read_write_byte(0xff);
//						printf("��ǰ��size%d\r\n,resp��%2x",size,resp);
						if (resp != 0xfe)
						{
							err = 1;
							printf("%s: invalid start of block!\n", __FUNCTION__);
							break;
						}
				}
				//����Ҫ��Ҫ����
				curr = (block_num) ? sdio_block_size[func] : size;
				if (bufsize != 0)
				{
					//�����ݱ�������
					
					WiFi_LowLevel_Receive(p, curr);
					//printf("�������� %d\r\n",curr);
					/*
					if (i == 0)
						p[0] = realsize & 0xff;*/ //̫�������һ���������յ�����ɶ
				}
				else
					WiFi_LowLevel_Receive(NULL, curr);
				//��λCRCУ��λ����һ��
				WiFi_LowLevel_Receive(&crc[0], 2);
			
				if (bufsize != 0)
				{
					p += curr;
				}
			
				size -= curr;
		}
		
		sd_deselect();	//ȡ��Ƭѡ
		return err == 0;
}

// DMA��������жϴ�����
void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tx);
}

// DMA��������жϴ����� ��������ɵ�ʱ��ͻᱻ����Ȼ����ܻ����������л�
void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_rx);
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SD_SPI) {
        spi_dma_done = 1;  // ���ñ�־����ʾDMA���
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				xSemaphoreGiveFromISR(dma_finish_semaphore,&xHigherPriorityTaskWoken);/* �ͷŶ�ֵ�ź��� */
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
