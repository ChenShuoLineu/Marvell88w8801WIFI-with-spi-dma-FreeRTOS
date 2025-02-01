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
1. WIFI驱动: Drivers/BSP/WIFI88w8801 + Drivers/BSP/SPI_SDIO
    spi_sdio.c : SPI +  DMA等外设的配置 /  SD卡的初始化 / 读写WIFI时的底层函数
    wifi.c : WIFI模块的初始化 / 固件下载 / 发送数据包 / 接收数据包 / 解析数据包
    wifi_app.c : 和 lwip核心线程/中断去交互
2. ringBuffer: Drivers/BSP/ringBuffer
    ringBuffer.c : 初始化 / 解析pbuf放入 缓冲区 / 从缓冲区读出数据放入pbuf
3. ethernetif.c : Middlwares/lwip/arch
    使用wifi模块移植lwip时的配置
    将数据包提交到lwip 核心线程 / 和 wifi底层线程交互
4. lwip_demo.c : User
    互ping / 测速 / 做客户端
