# 项目介绍
1.  编写了marvell88w8801WIFI的驱动(核心代码:wifi_app.c/wifi.c/spi_sdio.c)
2.  STM32F4采用 spi + dma的方式与WIFI模块进行数据传输和命令收发 速度最快1.5Mbps
3.  成功移植到了 lwip + FreeRTOS的环境中
4.  实现测速/互ping/TCP连接等基本功能
5.  具体实现讲解欢迎关注本人CSDN账号:https://blog.csdn.net/qq_45731845?spm=1000.2115.3001.5343
# 项目环境
1. lwip----2.1.3
2. FreeRTOS---10.4.6
3. wifi每次都需要重新加载固件(wifi_fm.c)到wifi的SRAM中,这个功能自己写一个读写flash的代码即可
# 项目结构
├─Drivers
│  ├─BSP
│  │  ├─LED
|  |  |─SRAM
|  |  |─STMFLASH
│  │  ├─ringBuffer  ------ 环形缓冲区:  初始化/put/get
│  │  ├─SPI_SDIO    ------ SPI外设配置: 外设配置/SD初始化/命令发送/spi收发数据
│  │  └─WIFI88W8801 ------ WIFI的驱动:  WIFI的固件加载/WIFI初始化/WIFI数据收发...
│  ├─CMSIS
│  │  ├─Device
│  │  └─Include
│  ├─STM32F4xx_HAL_Driver
│  └─SYSTEM
│      ├─delay
│      ├─sys
│      └─usart      ------ 串口驱动: 调试用
├─Middlewares
│  ├─FreeRTOS
│  │  ├─include
│  │  └─portable
│  ├─lwip
│  │  ├─arch
|  |  |  |─ethernetif.c         ---- 移植lwip需要配置的文件 实现底层数据包的发送和接收
│  │  └─src
│  ├─MALLOC          ----- 外部SRAM内存的管理
│  └─USMART            
├─Output
├─Projects
│  └─MDK-ARM
└─User