/**
 ****************************************************************************************************
 * @file        ov2640.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-11-08
 * @brief       OV2640 驱动代码
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
 * V1.0 20211108
 * 第一次发布
 *
 ****************************************************************************************************
 */


#include "./BSP/OV2640/ov2640.h"
#include "./BSP/OV2640/ov2640cfg.h"

//两个全局变量用来管理
uint32_t *g_jpeg_data_buf;       /* JPEG数据缓存buf */
//DMA缓存还是放在内存不是外部SRAM中把

__ALIGNED(4) uint32_t g_dcmi_line_buf[2][jpeg_line_size];   /* JPEG数据 DMA双缓存buf */


volatile uint32_t g_jpeg_data_len = 0;                      /* buf中的JPEG有效数据长度 */
volatile uint8_t g_jpeg_data_ok = 0;                        /* JPEG数据采集完成标志 */
SemaphoreHandle_t CaramaSemaphore;
const uint16_t jpeg_img_size_tbl[][2] =
{
    160, 120,       /* QQVGA */
    176, 144,       /* QCIF */
    320, 240,       /* QVGA */
    400,240,        /* WGVGA */
    352,288,        /* CIF */
    640, 480,       /* VGA */
    800, 600,       /* SVGA */
    1024, 768,      /* XGA */
    1280, 800,      /* WXGA */
    1280, 960,      /* XVGA */
    1440, 900,      /* WXGA+ */
    1280, 1024,     /* SXGA */
    1600, 1200,     /* UXGA */
};
/**
 * @brief       OV2640 读寄存器
 * @param       reg :  寄存器地址
 * @retval      读到的寄存器值
 */
uint8_t ov2640_read_reg(uint16_t reg)
{
    uint8_t data = 0;
    
    sccb_start();                       /* 启动SCCB传输 */
    sccb_send_byte(OV2640_ADDR);        /* 写器件ADDR */
    delay_us(100);
    sccb_send_byte(reg);                /* 写寄存器地址 */
    delay_us(100);
    sccb_stop();
    delay_us(100);
    
    /* 设置寄存器地址后，才是读 */
    sccb_start();
    sccb_send_byte(OV2640_ADDR | 0x01); /* 发送读命令 */
    delay_us(100);
    data = sccb_read_byte();            /* 读取数据 */
    sccb_nack();
    sccb_stop();
    
    return data;
}

/**
 * @brief       OV2640 写寄存器
 * @param       reg : 寄存器地址
 * @param       data: 要写入寄存器的值
 * @retval      0, 成功; 1, 失败;
 */
uint8_t ov2640_write_reg(uint16_t reg, uint8_t data)
{
    uint8_t res = 0;
    
    sccb_start();                               /* 启动SCCB传输 */
    delay_us(100);
    if (sccb_send_byte(OV2640_ADDR)) res = 1;   /* 写器件ID */
    delay_us(100);
    if (sccb_send_byte(reg)) res = 1;           /* 写寄存器地址 */
    delay_us(100);
    if (sccb_send_byte(data)) res = 1;          /* 写数据 */
    delay_us(100);
    sccb_stop();
    
    return res;
}

/**
 * @brief       初始化 OV2640
 * @param       无
 * @retval      0, 成功; 1, 失败;
 */
uint8_t ov2640_init(void)
{
    uint16_t i = 0;
    uint16_t reg;
    g_jpeg_data_buf = mymalloc(SRAMEX,30*1024); //分配内存
		if(g_jpeg_data_buf == NULL)
		{
			printf("给g_jpeg_data_buf分配内存失败\r\n");
			return 1;
		}
		CaramaSemaphore = xSemaphoreCreateBinary();
		if(CaramaSemaphore == NULL)
		{
				printf("给g_jpeg_data_buf创建信号量失败\r\n");
				return 1;
		}
    GPIO_InitTypeDef gpio_init_struct;
    
    OV_PWDN_GPIO_CLK_ENABLE();      /* 使能OV_PWDN脚时钟 */
    OV_RESET_GPIO_CLK_ENABLE();     /* 使能OV_RESET脚时钟 */

    gpio_init_struct.Pin = OV_PWDN_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;     /* 高速 */
    HAL_GPIO_Init(OV_PWDN_GPIO_PORT, &gpio_init_struct);    /* 初始化OV_PWDN引脚 */

    gpio_init_struct.Pin = OV_RESET_GPIO_PIN;
    HAL_GPIO_Init(OV_RESET_GPIO_PORT, &gpio_init_struct);   /* 初始化OV_RESET引脚 */

    OV2640_PWDN(0);     /* POWER ON */
    delay_ms(10);
    OV2640_RST(0);      /* 必须先拉低OV2640的RST脚,再上电 */
    delay_ms(20);
    OV2640_RST(1);      /* 结束复位 */
    delay_ms(20);
    
    sccb_init();        /* 初始化SCCB 的IO口 */
    delay_ms(5);
    ov2640_write_reg(OV2640_DSP_RA_DLMT, 0x01);     /* 操作sensor寄存器 */
    ov2640_write_reg(OV2640_SENSOR_COM7, 0x80);     /* 软复位OV2640 */
    delay_ms(50);
    reg = ov2640_read_reg(OV2640_SENSOR_MIDH);      /* 读取厂家ID 高八位 */
    reg <<= 8;
    reg |= ov2640_read_reg(OV2640_SENSOR_MIDL);     /* 读取厂家ID 低八位 */

    if (reg != OV2640_MID)      /* ID 是否正常 */
    {
        printf("MID:%d\r\n", reg);
        return 1;               /* 失败 */
    }
    
    reg = ov2640_read_reg(OV2640_SENSOR_PIDH);  /* 读取厂家ID 高八位 */
    reg <<= 8;
    reg |= ov2640_read_reg(OV2640_SENSOR_PIDL); /* 读取厂家ID 低八位 */

    if (reg != OV2640_PID)      /* ID是否正常 */
    {
        printf("HID:%d\r\n", reg);
        return 1;   /* 失败 */
    }

    /* 初始化 OV2640 */
		//这个是在设定帧率
    for (i = 0; i < sizeof(ov2640_uxga_init_reg_tbl) / 2; i++)
    {
        ov2640_write_reg(ov2640_uxga_init_reg_tbl[i][0], ov2640_uxga_init_reg_tbl[i][1]);
    }

    return 0;       /* OV2640初始化完成 */
}

/**
 * @brief       OV2640 切换为JPEG模式
 * @param       无
 * @retval      无
 */
void ov2640_jpeg_mode(void)
{
    uint16_t i = 0;

    /* 设置:YUV422格式 */
    for (i = 0; i < (sizeof(ov2640_yuv422_reg_tbl) / 2); i++)
    {
        ov2640_write_reg(ov2640_yuv422_reg_tbl[i][0], ov2640_yuv422_reg_tbl[i][1]); /* 发送配置数组 */
    }

    /* 设置:输出JPEG数据 */
    for (i = 0; i < (sizeof(ov2640_jpeg_reg_tbl) / 2); i++)
    {
        ov2640_write_reg(ov2640_jpeg_reg_tbl[i][0], ov2640_jpeg_reg_tbl[i][1]);     /* 发送配置数组 */
    }
}

/**
 * @brief       OV2640 切换为RGB565模式
 * @param       无
 * @retval      无
 */
void ov2640_rgb565_mode(void)
{
    uint16_t i = 0;

    /* 设置:RGB565输出 */
    for (i = 0; i < (sizeof(ov2640_rgb565_reg_tbl) / 4); i++)
    {
        ov2640_write_reg(ov2640_rgb565_reg_tbl[i][0], ov2640_rgb565_reg_tbl[i][1]); /* 发送配置数组 */
    }
}

/* 自动曝光设置参数表,支持5个等级 */
const static uint8_t OV2640_AUTOEXPOSURE_LEVEL[5][8]=
{
    {
        0xFF, 0x01,
        0x24, 0x20,
        0x25, 0x18,
        0x26, 0x60,
    },
    {
        0xFF, 0x01,
        0x24, 0x34,
        0x25, 0x1c,
        0x26, 0x00,
    },
    {
        0xFF, 0x01,
        0x24, 0x3e,
        0x25, 0x38,
        0x26, 0x81,
    },
    {
        0xFF, 0x01,
        0x24, 0x48,
        0x25, 0x40,
        0x26, 0x81,
    },
    {
        0xFF, 0x01,
        0x24, 0x58,
        0x25, 0x50,
        0x26, 0x92,
    },
};

/**
 * @brief       OV2640 EV曝光补偿
 * @param       level : 0~4
 * @retval      无
 */
void ov2640_auto_exposure(uint8_t level)
{
    uint8_t i;
    uint8_t *p = (uint8_t*)OV2640_AUTOEXPOSURE_LEVEL[level];
    
    for (i = 0; i < 4; i++)
    { 
        ov2640_write_reg(p[i * 2], p[i * 2 + 1]); 
    }
}

/**
 * @brief       OV2640 白平衡设置
 * @param       mode : 0~4, 白平衡模式
 *   @arg       0: 自动   auto
 *   @arg       1: 日光   sunny
 *   @arg       2: 办公室 office
 *   @arg       3: 阴天   cloudy
 *   @arg       4: 家里   home
 * @retval      无
 */
void ov2640_light_mode(uint8_t mode)
{
    uint8_t regccval = 0x5E;/* Sunny */
    uint8_t regcdval = 0x41;
    uint8_t regceval = 0x54;
    
    switch (mode)
    { 
        case 0:             /* auto */
            ov2640_write_reg(0xFF, 0x00);
            ov2640_write_reg(0xC7, 0x00);    /* AWB ON  */
            return;
        case 2:             /* cloudy */
            regccval = 0x65;
            regcdval = 0x41;
            regceval = 0x4F;
            break;
        case 3:             /* office */
            regccval = 0x52;
            regcdval = 0x41;
            regceval = 0x66;
            break;
        case 4:             /* home */
            regccval = 0x42;
            regcdval = 0x3F;
            regceval = 0x71;
            break;
        default : break;
    }
    
    ov2640_write_reg(0xFF, 0x00);
    ov2640_write_reg(0xC7, 0x40);            /* AWB OFF  */
    ov2640_write_reg(0xCC, regccval);
    ov2640_write_reg(0xCD, regcdval);
    ov2640_write_reg(0xCE, regceval);
}

/**
 * @brief       OV2640 色彩饱和度设置
 * @param       set : 0~4, 代表色彩饱和度 -2 ~ 2.
 * @retval      无
 */
void ov2640_color_saturation(uint8_t sat)
{
    uint8_t reg7dval = ((sat + 2) << 4) | 0x08;
    
    ov2640_write_reg(0xFF, 0x00);
    ov2640_write_reg(0x7C, 0x00);
    ov2640_write_reg(0x7D, 0x02);
    ov2640_write_reg(0x7C, 0x03);
    ov2640_write_reg(0x7D, reg7dval);
    ov2640_write_reg(0x7D, reg7dval);
}

/**
 * @brief       OV2640 亮度设置
 * @param       bright : 0~5, 代表亮度 -2 ~ 2.
 * @retval      无
 */
void ov2640_brightness(uint8_t bright)
{
    ov2640_write_reg(0xff, 0x00);
    ov2640_write_reg(0x7c, 0x00);
    ov2640_write_reg(0x7d, 0x04);
    ov2640_write_reg(0x7c, 0x09);
    ov2640_write_reg(0x7d, bright << 4); 
    ov2640_write_reg(0x7d, 0x00); 
}

/**
 * @brief       OV2640 对比度设置
 * @param       contrast : 0~4, 代表对比度 -2 ~ 2.
 * @retval      无
 */
void ov2640_contrast(uint8_t contrast)
{
    uint8_t reg7d0val = 0x20;       /* 默认为普通模式 */
    uint8_t reg7d1val = 0x20;
    
    switch (contrast)
    {
        case 0:
            reg7d0val = 0x18;       /* -2 */
            reg7d1val = 0x34;
            break;
        case 1:
            reg7d0val = 0x1C;       /* -1 */
            reg7d1val = 0x2A;
            break;
        case 3:
            reg7d0val = 0x24;       /* 1 */
            reg7d1val = 0x16;
            break;
        case 4:
            reg7d0val = 0x28;       /* 2 */
            reg7d1val = 0x0C;
            break;
        default : break;
    }
    
    ov2640_write_reg(0xff, 0x00);
    ov2640_write_reg(0x7c, 0x00);
    ov2640_write_reg(0x7d, 0x04);
    ov2640_write_reg(0x7c, 0x07);
    ov2640_write_reg(0x7d, 0x20);
    ov2640_write_reg(0x7d, reg7d0val);
    ov2640_write_reg(0x7d, reg7d1val);
    ov2640_write_reg(0x7d, 0x06);
}

/**
 * @brief       OV2640 特效设置
 * @param       eft : 0~6, 特效效果
 *   @arg       0: 正常
 *   @arg       1: 负片
 *   @arg       2: 黑白
 *   @arg       3: 偏红色
 *   @arg       4: 偏绿色
 *   @arg       5: 偏蓝色
 *   @arg       6: 复古
 * @retval      无
 */
void ov2640_special_effects(uint8_t eft)
{
    uint8_t reg7d0val = 0x00;   /* 默认为普通模式 */
    uint8_t reg7d1val = 0x80;
    uint8_t reg7d2val = 0x80; 
    
    switch(eft)
    {
        case 1:     /* 负片 */
            reg7d0val = 0x40; 
            break;
        case 2:     /* 黑白 */
            reg7d0val = 0x18; 
            break;
        case 3:     /* 偏红色 */
            reg7d0val = 0x18; 
            reg7d1val = 0x40;
            reg7d2val = 0xC0; 
            break;
        case 4:     /* 偏绿色 */
            reg7d0val = 0x18; 
            reg7d1val = 0x40;
            reg7d2val = 0x40; 
            break;
        case 5:     /* 偏蓝色 */
            reg7d0val = 0x18; 
            reg7d1val = 0xA0;
            reg7d2val = 0x40; 
            break;
        case 6:     /* 复古 */
            reg7d0val = 0x18; 
            reg7d1val = 0x40;
            reg7d2val = 0xA6; 
            break;
    }
    
    ov2640_write_reg(0xff, 0x00);
    ov2640_write_reg(0x7c, 0x00);
    ov2640_write_reg(0x7d, reg7d0val);
    ov2640_write_reg(0x7c, 0x05);
    ov2640_write_reg(0x7d, reg7d1val);
    ov2640_write_reg(0x7d, reg7d2val); 
}

/**
 * @brief       OV2640 彩条测试
 * @param       mode : 0~2
 *   @arg       0: 关闭 
 *   @arg       1: 彩条
 * @retval      无
 */
void ov2640_color_bar(uint8_t mode)
{
    uint8_t reg;
    
    ov2640_write_reg(0xFF, 0x01);
    reg = ov2640_read_reg(0x12);
    reg &= ~(1 << 1);
    if (mode)reg |=1 << 1;
    ov2640_write_reg(0x12, reg);
}

/**
 * @bref        设置传感器输出窗口
 * @param       sx    : 起始地址x
 * @param       sy    : 起始地址y
 * @param       width : 宽度(对应:horizontal)
 * @param       hight : 高度(对应:vertical)
 * @retval      无
 */
void ov2640_window_set(uint16_t sx, uint16_t sy, uint16_t width, uint16_t height)
{
    uint16_t endx;
    uint16_t endy;
    uint8_t temp; 
    
    endx = sx + width / 2;
    endy = sy + height / 2;

    ov2640_write_reg(0xFF, 0x01);    
    temp = ov2640_read_reg(0x03);       /* 读取Vref之前的值 */
    temp &= 0xF0;
    temp |= ((endy & 0x03) << 2) | (sy & 0x03);
    ov2640_write_reg(0x03, temp);       /* 设置Vref的start和end的最低2位 */ 
    ov2640_write_reg(0x19, sy >> 2);    /* 设置Vref的start高8位 */
    ov2640_write_reg(0x1A, endy >> 2);  /* 设置Vref的end的高8位 */

    temp = ov2640_read_reg(0x32);       /* 读取Href之前的值 */
    temp &= 0xC0;
    temp |= ((endx & 0x07) << 3) | (sx & 0x07);
    ov2640_write_reg(0x32, temp);       /* 设置Href的start和end的最低3位 */
    ov2640_write_reg(0x17, sx >> 3);    /* 设置Href的start高8位 */
    ov2640_write_reg(0x18, endx >> 3);  /* 设置Href的end的高8位 */
}

/** 
 * @bref        设置图像输出大小
 * @@param      width : 宽度(对应:horizontal)
 * @param       height: 高度(对应:vertical)
 * @note        OV2640输出图像的大小(分辨率),完全由该函数确定
 *              width和height必须是4的倍数
 * @retval      0     : 设置成功
 *              其他  : 设置失败
 */
uint8_t ov2640_outsize_set(uint16_t width, uint16_t height)
{
    uint16_t outh;
    uint16_t outw;
    uint8_t temp;

    if (width % 4) return 1;
    if (height % 4) return 2;

    outw = width / 4;
    outh = height/ 4;
    ov2640_write_reg(0xFF, 0x00);    
    ov2640_write_reg(0xE0, 0x04);
    ov2640_write_reg(0x5A, outw & 0xFF);    /* 设置OUTW的低八位 */
    ov2640_write_reg(0x5B, outh & 0xFF);    /* 设置OUTH的低八位 */

    temp = (outw >> 8) & 0x03;
    temp |= (outh >> 6) & 0x04;
    ov2640_write_reg(0x5C, temp);           /* 设置OUTH/OUTW的高位 */
    ov2640_write_reg(0xE0, 0x00);

    return 0;
}

/**
 * @brief       设置图像开窗大小
 *   @note      OV2640输出图像的大小(分辨率),完全由该函数确定
 *              由于开启了scale功能,用于输出的图像窗口为: xsize - 2 * offx, ysize - 2 * offy
 *              实际输出(width,height), 是在: xsize - 2 * offx, ysize - 2 * offy的基础上进行缩放处理.
 *              一般设置offx和offy的值为16和4,更小也是可以,不过默认是16和4
 * @param       offx,offy    : 输出图像在ov2640_image_window_set设定窗口(假设长宽为xsize和ysize)上的偏移
 * @param       width,height : 实际输出图像的宽度和高度,必须是4的倍数
 * @retval      0, 成功; 其他, 失败;
 */
uint8_t ov2640_image_win_set(uint16_t offx, uint16_t offy, uint16_t width, uint16_t height)
{
    uint16_t hsize;
    uint16_t vsize;
    uint8_t temp;

    if (width % 4) return 1;
    if (height % 4) return 2;
    hsize = width / 4;
    vsize = height / 4;
    ov2640_write_reg(0xFF, 0x00);
    ov2640_write_reg(0xE0, 0x04);
    ov2640_write_reg(0x51, hsize & 0xFF);           /* 设置H_SIZE的低八位 */
    ov2640_write_reg(0x52, vsize & 0xFF);           /* 设置V_SIZE的低八位 */
    ov2640_write_reg(0x53, offx & 0xFF);            /* 设置offx的低八位 */
    ov2640_write_reg(0x54, offy & 0xFF);            /* 设置offy的低八位 */
    temp = (vsize >> 1) & 0x80;
    temp |= (offy >> 4) & 0x70;
    temp |= (hsize>>5) & 0x08;
    temp |= (offx >> 8) & 0x07; 
    ov2640_write_reg(0x55, temp);                   /* 设置H_SIZE/V_SIZE/OFFX,OFFY的高位 */
    ov2640_write_reg(0x57, (hsize >> 2) & 0x80);    /* 设置H_SIZE/V_SIZE/OFFX,OFFY的高位 */
    ov2640_write_reg(0xE0, 0x00);

    return 0;
}

/**
 * @bref    该函数设置图像尺寸大小,也就是所选格式的输出分辨率
 * @note    UXGA:1600*1200,SVGA:800*600,CIF:352*288
 * @param   width  : 图像宽度
 * @param   height : 图像高度
 *
 * @retval  0      : 设置成功
 *          其他   : 设置失败
 */
uint8_t ov2640_imagesize_set(uint16_t width, uint16_t height)
{ 
    uint8_t temp; 
    
    ov2640_write_reg(0xFF, 0x00);
    ov2640_write_reg(0xE0, 0x04);
    ov2640_write_reg(0xC0, (width) >> 3 & 0xFF);    /* 设置HSIZE的10:3位 */
    ov2640_write_reg(0xC1, (height) >> 3 & 0xFF);   /* 设置VSIZE的10:3位 */

    temp = (width & 0x07) << 3;
    temp |= height & 0x07;
    temp |= (width >> 4) & 0x80;

    ov2640_write_reg(0x8C, temp);
    ov2640_write_reg(0xE0, 0x00);

    return 0;
}

/**
 * @bref        闪光灯外部信号控制
 * @param       sw : 0,关闭
 *                   1,打开
 * @retval      无
 */
void ov2640_flash_extctrl(uint8_t sw)
{
    if(sw) OV2640_FLASH(1);
    else OV2640_FLASH(0);
}

/**
 * @bref        闪光灯内部信号控制
 * @note        控制信号从OV2640的STROBE引脚自动输出
 * @retval      无
 */
void ov2640_flash_intctrl(void)
{
    ov2640_write_reg(0xFF, 0x01);
    ov2640_write_reg(0x4B, 0x81);
    ov2640_write_reg(0x4B, 0x01);
}


void jpeg_data_process(void)
{
    uint16_t i;
    uint16_t rlen;      /* 剩余数据长度 */
    uint32_t *pbuf;

		if (g_jpeg_data_ok == 0)                    /* jpeg数据还未采集完? */
		{
				__HAL_DMA_DISABLE(&g_dma_dcmi_handle);  /* 关闭DMA */
				while(DMA2_Stream1->CR & 0x01);         /* 等待DMA2_Stream1可配置 */

				rlen = jpeg_line_size - __HAL_DMA_GET_COUNTER(&g_dma_dcmi_handle);    /* 得到剩余长度 */
				pbuf = g_jpeg_data_buf + g_jpeg_data_len;   /* 偏移到有效数据末尾,继续添加 */
				//把数据从双缓存拿出来 因为是双缓存
				//这边操作这个的时候内边就也在往DMA中写入
			if (DMA2_Stream1->CR & (1 << 19))
				{
						for (i = 0; i < rlen; i++)
						{
								pbuf[i] = g_dcmi_line_buf[1][i];    /* 读取buf1里面的剩余数据 */
						}
				}
				else
				{
						for (i = 0; i < rlen; i++)
						{
								pbuf[i] = g_dcmi_line_buf[0][i];    /* 读取buf0里面的剩余数据 */
						}
				}

				g_jpeg_data_len += rlen;    /* 加上剩余长度 */
				//可以用一个消息队列传出去？
				g_jpeg_data_ok = 1;         /* 标记JPEG数据采集完成,等待其他函数处理 */
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				xSemaphoreGiveFromISR(CaramaSemaphore,&xHigherPriorityTaskWoken);/* 释放二值信号量 */
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		if (g_jpeg_data_ok == 2)        /* 上一次的jpeg数据已经被处理了 */
		{
				__HAL_DMA_SET_COUNTER(&g_dma_dcmi_handle, jpeg_line_size);   /* 传输长度为jpeg_buf_size*4字节 */
				__HAL_DMA_ENABLE(&g_dma_dcmi_handle);   /* 重新传输 */
				g_jpeg_data_ok = 0;         /* 标记数据未采集 */
				g_jpeg_data_len = 0;        /* 数据重新开始 */
		}
}



void jpeg_dcmi_rx_callback(void)
{
    uint16_t i;
    volatile uint32_t *pbuf;
		//总共就一个g_jpeg_data_buf既要发送又要接受
    pbuf = g_jpeg_data_buf + g_jpeg_data_len;   /* 偏移到有效数据末尾 */

    if (DMA2_Stream1->CR & (1 << 19))           /* buf0已满,正常处理buf1 */
    {
        for (i = 0; i < jpeg_line_size; i++)
        {
            pbuf[i] = g_dcmi_line_buf[0][i];    /* 读取buf0里面的数据 */
        }
        
        g_jpeg_data_len += jpeg_line_size;      /* 偏移 */
    }
    else    /* buf1已满,正常处理buf0 */
    {
        for (i = 0; i < jpeg_line_size; i++)
        {
            pbuf[i] = g_dcmi_line_buf[1][i];    /* 读取buf1里面的数据 */
        }
        
        g_jpeg_data_len += jpeg_line_size;      /* 偏移 */
    }
}
