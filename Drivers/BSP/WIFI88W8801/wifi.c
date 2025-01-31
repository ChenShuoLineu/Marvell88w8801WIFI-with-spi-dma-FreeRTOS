#include "wifi.h"



pmrvl88w8801_core_t pmrvl88w8801_core; //有大用的指针
// 就是所有的发送的时候都是把mrvl_tx_buffer做了一个强转
uint8_t *mrvl_tx_buffer; 			//发送和接受的buf
uint8_t *mrvl_rx_buffer;			
uint8_t *mp_reg_array;				// 用来读取寄存器的值
wifi_cb_t *mrvl_wifi_cb = NULL; //定义了一个指针后面可能用到


uint8_t wifi_start_scan(scan_condition_t *scan_condition)
{
    if(scan_condition->ssid_len)
    {
        mrvl88w8801_scan_ssid(scan_condition->ssid,scan_condition->ssid_len,scan_condition->scan_max_time);
    }
    else
    {
        mrvl88w8801_scan(scan_condition->channel,scan_condition->channel_num,scan_condition->scan_max_time);
    }
    return WIFI_ERR_OK;
}

uint8_t wifi_connect_ap(uint8_t *ssid,uint8_t ssid_len,uint8_t *pwd,uint8_t pwd_len)
{
    mrvl88w8801_connect(ssid,ssid_len,pwd,pwd_len);
    return WIFI_ERR_OK;
}
uint8_t wifi_disconnect_ap()
{
    mrvl88w8801_disconnect();
    return WIFI_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_core_init
 * 参数:  		NULL
 * 返回值: 	返回执行结果
 * 描述:		pmrvl88w8801_core结构体初始化
******************************************************************************/
static uint8_t mrvl88w8801_core_init()
{
    memset(pmrvl88w8801_core,0,sizeof(mrvl88w8801_core_t));
    pmrvl88w8801_core->bss_type = BSS_TYPE_STA;
    pmrvl88w8801_core->mp_regs = mp_reg_array;
    pmrvl88w8801_core->curr_wr_port = 1;
    pmrvl88w8801_core->curr_rd_port = 1;
    return COMP_ERR_OK;
}

void marvel88w8801_init(wifi_cb_t *cb)
{
		unsigned char fun_num = 0;
		//想想什么时候释放吧--这三个变量的作用周期已经跟freertos一样了
		mrvl_tx_buffer = mymalloc(SRAMEX,10*1024);
		mrvl_rx_buffer = mymalloc(SRAMEX,10*1024);
		mp_reg_array = mymalloc(SRAMEX,56);
		pmrvl88w8801_core = mymalloc(SRAMEX,sizeof(mrvl88w8801_core_t));
		mrvl_wifi_cb = cb;				//用来和main函数进行变量值的交换
		fun_num = sd_init(); 			//初始化对应的SDIO 并得到有多少个分区
		mrvl88w8801_core_init(); //初始化这个指针 后面有大用
		if(fun_num == 0)
			return;
		mrvl88w8801_get_control_io_port(); //得到对应的IO寄存器的值
		
		WiFi_LowLevel_SetBlockSize(0, 32);
		// 初始化Function 1
		WiFi_LowLevel_WriteReg(0, SDIO_CCCR_IOEN, SDIO_CCCR_IOEN_IOE1); // IOE1=1 (Enable Function)
		while ((WiFi_LowLevel_ReadReg(0, SDIO_CCCR_IORDY) & SDIO_CCCR_IORDY_IOR1) == 0); // 等待IOR1=1 (I/O Function Ready)
		// 开启中断 这样内个IRQ才有用了
		WiFi_LowLevel_WriteReg(0, SDIO_CCCR_INTEN, SDIO_CCCR_INTEN_IENM | SDIO_CCCR_INTEN_IEN1); 
		WiFi_LowLevel_WriteReg(1, WIFI_INTMASK, WIFI_INTMASK_HOSTINTMASK);
		sd_spi_speed_high();//最高支持到16分频的不能再高了
		WiFi_LowLevel_SetBlockSize(1, 512);
		WiFi_DownloadFirmware(); //把固件从flash里面读出来然后下载到wifi里面
		//此时查询一下状态是什么 假如此时是0 就能对上了 不是0就..
		printf("此时的IRQ状态%d\r\n",WiFi_LowLevel_GetITStatus(1));//对上了 也就是启动了
		//sd_spi_speed_high();//最高支持到16分频的不能再高了
		mrvl88w8801_init_fw(); //初始化固件
		//WiFi_LowLevel_SetBlockSize(1, 1024);
		delay_ms(255);
		
}

//得到io_port寄存器 后面有用
void mrvl88w8801_get_control_io_port()
{
		uint32_t control_io_port;
		//md/fw data都是往这个地址去写，cmd response是通过这个地址读回来
    control_io_port = WiFi_LowLevel_ReadReg(1, WIFI_IOPORT0) | (WiFi_LowLevel_ReadReg(1, WIFI_IOPORT1) << 8) | 
																						(WiFi_LowLevel_ReadReg(1, WIFI_IOPORT2) << 16);
		printf("control_io_port是%d\r\n",control_io_port);
		pmrvl88w8801_core->control_io_port = control_io_port;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_init_fw
 * 参数:  		NULL
 * 返回值: 	返回执行结果
 * 描述:		执行init fw cmd，整个流程是先发送一个命令，收到cmd response的
 				时候发送下一条CMD
******************************************************************************/
uint8_t mrvl88w8801_init_fw()
{
    mrvl88w8801_prepare_cmd(HostCmd_CMD_FUNC_INIT,HostCmd_ACT_GEN_SET,NULL,0);
    return COMP_ERR_OK;
}

//初始化硬件并且激活一些线程
// 准备开始工作
static uint8_t  mrvl88w8801_func_init(uint8_t* tx)
{
		//这里没有问题成功写入了数据且都对上了
		uint16_t tx_packet_len = CMD_HDR_SIZE;
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
	  cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_FUNC_INIT; //0X00A9 初始化命令
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;//这不就成0了吗
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;

		return COMP_ERR_OK;

}
/******************************************************************************
 *	函数名:	mrvl88w8801_mac_control
 * 参数:  	tx(IN)				-->tx buffer
 			data_buff(IN)		-->action行为指针
 * 返回值: 	返回执行结果
 * 描述:	组HostCmd_CMD_MAC_CONTROL command的封包
******************************************************************************/
static uint8_t mrvl88w8801_mac_control(uint8_t* tx,void *data_buff)
{
    uint16_t action = *((uint16_t *) data_buff);
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    HostCmd_DS_MAC_CONTROL *pmac = &cmd->params.mac_ctrl;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_MAC_CONTROL);

    cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_MAC_CONTROL;
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;
    pmac->action = comp_cpu_to_le16(action);

    return COMP_ERR_OK;
}

//
/******************************************************************************
 *	函数名:	mrvl88w8801_get_hw_spec
 * 参数:  	tx(IN)				-->tx buffer
 * 返回值: 	返回执行结果
 * 描述:	组HostCmd_CMD_GET_HW_SPEC command的封包
******************************************************************************/
static uint8_t mrvl88w8801_get_hw_spec(uint8_t* tx)
{
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_GET_HW_SPEC);
	
		// 这俩命令更像是一个封装的头部
		// 但为啥长度是12呢?CMD_HDR_SIZE
    cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
	
    cmd->command = HostCmd_CMD_GET_HW_SPEC;
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;

    return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_ret_get_hw_spec
 * 参数:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * 返回值: 	返回执行结果
 * 描述:		解析GET HW SPEC命令的响应，此部分我们只是获取write data port
 				的最大值
******************************************************************************/
static uint8_t mrvl88w8801_ret_get_hw_spec(uint8_t *rx_buffer,int len)
{
    HostCmd_DS_GET_HW_SPEC *hw_spec = (HostCmd_DS_GET_HW_SPEC *)rx_buffer;
		//得到端口号的最大值
    pmrvl88w8801_core->mp_end_port = hw_spec->mp_end_port;
    return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_mac_addr_prepare
 * 参数:  		tx(IN)				-->tx buffer
 				cmd_action(IN)	-->set/get
 				data_buff(IN)		-->mac address，此部分主要用于set
 				data_len(IN)		-->mac的长度
 * 返回值: 	返回执行结果
 * 描述:		组HostCmd_CMD_802_11_MAC_ADDRESS command的封包
******************************************************************************/
static uint8_t mrvl88w8801_mac_addr_prepare(uint8_t* tx,uint16_t cmd_action,void *data_buff,uint16_t data_len)
{
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    HostCmd_DS_802_11_MAC_ADDRESS *pmac_addr = &cmd->params.mac_addr;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_802_11_MAC_ADDRESS)+data_len;

    cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_802_11_MAC_ADDRESS;
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;
    if(cmd_action == HostCmd_ACT_GEN_GET)
    {
        pmac_addr->action = HostCmd_ACT_GEN_GET;
        memset(pmac_addr->mac_addr,0,MAC_ADDR_LENGTH);
    }
    else	/* set mac address */
    {
        pmac_addr->action = HostCmd_ACT_GEN_SET;
        memcpy(pmac_addr->mac_addr,data_buff,MAC_ADDR_LENGTH);
    }
    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_ret_mac_address
 * 参数:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * 返回值: 	返回执行结果
 * 描述:		此部分主要是处理mac cmd reponse，如果拿到mac，那么会做两个事情:
 				1.更新pmrvl88w8801_core结构体中断额mac_address
 				2.初始化lwip的mac
******************************************************************************/
static uint8_t mrvl88w8801_ret_mac_address(uint8_t *rx_buffer,int len)
{
    HostCmd_DS_802_11_MAC_ADDRESS *pconnect_rsp = (HostCmd_DS_802_11_MAC_ADDRESS *)rx_buffer;
    memcpy(pmrvl88w8801_core->mac_address,pconnect_rsp->mac_addr,MAC_ADDR_LENGTH);
    printf("mrvl88w8801_ret_mac_address mac dump mac_address is\n");
		for(unsigned char i = 0;i < MAC_ADDR_LENGTH;i++)
			printf("0x%2x:",pmrvl88w8801_core->mac_address[i]);
		printf("\r\n");

	
		//wificb是一堆函数指针
    if(mrvl_wifi_cb && mrvl_wifi_cb->wifi_init_result)
        mrvl_wifi_cb->wifi_init_result(COMP_ERR_OK);
    return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_process_packet
 * 参数:  	channel(IN)		-->channel数组,用于存储分别搜索什么通道
 			channel_num(IN)	-->搜索的通道个数
 			max_time(IN)		-->搜索的最大time
 * 返回值: 	返回执行结果
 * 描述:	执行普通搜索动作
******************************************************************************/
uint8_t mrvl88w8801_scan(uint8_t *channel,uint8_t channel_num,uint16_t max_time)
{
    uint8_t index = 0;
    uint16_t scan_para_len = sizeof(MrvlIEtypesHeader_t) + channel_num*sizeof(ChanScanParamSet_t);
    uint8_t scan_para[scan_para_len];
    MrvlIEtypes_ChanListParamSet_t *channel_list;
    ChanScanParamSet_t *channel_list_para[channel_num];

    printf("mrvl88w8801_scan\n");
    channel_list = (MrvlIEtypes_ChanListParamSet_t *)(scan_para);
    channel_list->header.type = TLV_TYPE_CHANLIST;
    channel_list->header.len = channel_num*sizeof(ChanScanParamSet_t);

    /* 组合channel list的参数 */
    for(index = 0; index < channel_num; index++)
    {
        channel_list_para[index] = (ChanScanParamSet_t *)(scan_para + sizeof(MrvlIEtypesHeader_t)+index*sizeof(ChanScanParamSet_t));
        channel_list_para[index]->radio_type = 0;
        channel_list_para[index]->chan_number = channel[index];
        channel_list_para[index]->chan_scan_mode = 0;
        channel_list_para[index]->min_scan_time = 0;
        channel_list_para[index]->max_scan_time = max_time;
    }

    mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_SCAN,HostCmd_ACT_GEN_GET,&scan_para,scan_para_len);
    return COMP_ERR_OK;

}

/******************************************************************************
 *	函数名:	mrvl88w8801_scan_ssid
 * 参数:  		ssid(IN)		-->特定搜索的AP名称
 				ssid_len(IN)	-->特定搜索AP名称的长度
 				max_time(IN)	-->搜索的最大time
 * 返回值: 	返回执行结果
 * 描述:		执行特定名称搜索动作
******************************************************************************/
uint8_t mrvl88w8801_scan_ssid(uint8_t *ssid,uint8_t ssid_len,uint16_t max_time)
{
    uint8_t index = 0;
    uint16_t scan_para_len = sizeof(MrvlIEtypesHeader_t) + ssid_len +sizeof(MrvlIEtypesHeader_t) + 14*sizeof(ChanScanParamSet_t);
    uint8_t scan_para[scan_para_len];
    MrvlIEtypes_ChanListParamSet_t *channel_list;
    ChanScanParamSet_t *channel_list_para[14];
    MrvlIEtypes_SSIDParamSet_t *ssid_tlv =  (MrvlIEtypes_SSIDParamSet_t *)scan_para;

    printf("mrvl88w8801_scan_ssid\n");
    /* 组合SSID tlv的参数 */
    ssid_tlv->header.type = TLV_TYPE_SSID;
    ssid_tlv->header.len = ssid_len;
    memcpy(ssid_tlv->ssid,ssid,ssid_len);

    /* 组合channel list tlv的参数 */
    channel_list = (MrvlIEtypes_ChanListParamSet_t *)(scan_para+sizeof(MrvlIEtypesHeader_t) + ssid_len);
    channel_list->header.type = TLV_TYPE_CHANLIST;
    channel_list->header.len = 14*sizeof(ChanScanParamSet_t);
    for(index = 0; index < 14; index++)
    {
        channel_list_para[index] = (ChanScanParamSet_t *)(scan_para + sizeof(MrvlIEtypesHeader_t)*2+ssid_len+index*sizeof(ChanScanParamSet_t));

        channel_list_para[index]->radio_type = 0;
        channel_list_para[index]->chan_number = index+1;
        channel_list_para[index]->chan_scan_mode = 0;
        channel_list_para[index]->min_scan_time = 0;
        channel_list_para[index]->max_scan_time = max_time;
    }

    mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_SCAN,HostCmd_ACT_GEN_GET,&scan_para,scan_para_len);
    return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_connect_prepare
 * 参数:  		tx(IN)				-->tx buffer
 				data_buff(IN)		-->搜索的cmd body，在这里就是channel list
 				data_len(IN)		-->搜索的cmd body len,在这里就是channel list长度
 * 返回值: 	返回执行结果
 * 描述:		组HostCmd_CMD_802_11_ASSOCIATE command的封包
******************************************************************************/
static uint8_t mrvl88w8801_connect_prepare(uint8_t* tx,void *data_buff,uint16_t data_len)
{
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    HostCmd_DS_802_11_ASSOCIATE *passociate = &cmd->params.associate;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_802_11_ASSOCIATE)+data_len-sizeof(passociate->tlv_buffer);

    cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_802_11_ASSOCIATE;
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;
    comp_memcpy(passociate->peer_sta_addr,pmrvl88w8801_core->con_info.mac_address,MAC_ADDR_LENGTH);
    comp_memcpy(pmrvl88w8801_core->remote_mac_address,pmrvl88w8801_core->con_info.mac_address,MAC_ADDR_LENGTH);
    passociate->cap_info = pmrvl88w8801_core->con_info.cap_info;
    passociate->listen_interval = 0xa;
    passociate->beacon_period = 0x40;
    passociate->dtim_period = 0x0;

    comp_memcpy(passociate->tlv_buffer,data_buff,data_len);

    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_ret_connect
 * 参数:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * 返回值: 	返回执行结果
 * 描述:		解析连接命令的cmd response
******************************************************************************/
static uint8_t mrvl88w8801_ret_connect(uint8_t *rx_buffer,int len)
{
    HostCmd_DS_802_11_ASSOCIATE_RSP *pconnect_rsp = (HostCmd_DS_802_11_ASSOCIATE_RSP *)rx_buffer;
    uint16_t cap = pconnect_rsp->assoc_rsp.Capability;
    switch (cap)
    {
			case 0xfffc:
				COMP_DEBUG("WIFI ERR:connect timeout\n");
			case 0xfffd:
				COMP_DEBUG("WIFI ERR:authencition refused\n");
			case 0xfffe:
				COMP_DEBUG("WIFI ERR:authencition unhandled message\n");
			case 0xffff:
				 COMP_DEBUG("WIFI ERR:internal error\n");

        if(mrvl_wifi_cb && mrvl_wifi_cb->wifi_connect_result)
            mrvl_wifi_cb->wifi_connect_result(1);
        break;
			default:
			{

					COMP_DEBUG("WIFI SUCCESS:connect success cap 0x%x\n",cap);
					break;
			}
    }

    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_connect
 * 参数:  		ssid(IN)		-->要连接的AP名称
 				ssid_len(IN)	-->要连接的AP名称长度
 				pwd(IN)		-->要连接的AP密码
 				pwd_len(IN)	-->要连接的AP密码长度
 * 返回值: 	返回执行结果
 * 描述:		连接ssid
******************************************************************************/
uint8_t mrvl88w8801_connect(uint8_t *ssid,uint8_t ssid_len,uint8_t *pwd,uint8_t pwd_len)
{
    memcpy(pmrvl88w8801_core->con_info.ssid,ssid,ssid_len);
    pmrvl88w8801_core->con_info.ssid_len = ssid_len;
    memcpy(pmrvl88w8801_core->con_info.pwd,pwd,pwd_len);
    pmrvl88w8801_core->con_info.pwd_len = pwd_len;

    /* 执行特定名称的搜索动作，如果搜索到，在搜索解析函数中，会根据con_status去连接 */
    mrvl88w8801_scan_ssid(ssid,ssid_len,200);
    pmrvl88w8801_core->con_info.con_status = CON_STA_CONNECTING;
    return COMP_ERR_OK;
}


/******************************************************************************
 *	函数名:	mrvl88w8801_connect_prepare
 * 参数:  	tx(IN)				-->tx buffer
 * 返回值: 	返回执行结果
 * 描述:	组HostCmd_CMD_802_11_DEAUTHENTICATE command的封包
******************************************************************************/
static uint8_t mrvl88w8801_disconnect_prepare(uint8_t* tx)
{
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    HostCmd_DS_802_11_DEAUTHENTICATE *pdeauth = &cmd->params.deauth;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_802_11_DEAUTHENTICATE);

    cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_802_11_DEAUTHENTICATE;
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;
    comp_memcpy(pdeauth->mac_addr,pmrvl88w8801_core->remote_mac_address,MAC_ADDR_LENGTH);
    pdeauth->reason_code = 36;

    return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_disconnect
 * 参数:  		NULL
 * 返回值: 	返回执行结果
 * 描述:		用于做为STA mode，主动断开于AP的连线
******************************************************************************/
uint8_t mrvl88w8801_disconnect()
{
    mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_DEAUTHENTICATE,HostCmd_ACT_GEN_GET,NULL,0);
    return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_scan_prepare
 * 参数:  	tx(IN)				-->tx buffer
 			data_buff(IN)		-->搜索的cmd body，在这里就是channel list
 			data_len(IN)		-->搜索的cmd body len,在这里就是channel list长度
 * 返回值: 	返回执行结果
 * 描述:	组HostCmd_CMD_802_11_SCAN command的封包
******************************************************************************/
static uint8_t mrvl88w8801_scan_prepare(uint8_t* tx,void *data_buff,uint16_t data_len)
{
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    HostCmd_DS_802_11_SCAN *pscan = &cmd->params.scan;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_802_11_SCAN)+data_len-sizeof(pscan->tlv_buffer);

    cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_802_11_SCAN;
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;
    pscan->bss_mode = HostCmd_BSS_MODE_ANY;
    memset(pscan->bssid,0,MAC_ADDR_LENGTH);
    memcpy(pscan->tlv_buffer,data_buff,data_len);

    return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_ass_supplicant_pmk_pkg
 * 参数:  		bssid(IN)				-->mac address
 * 返回值: 	返回执行结果
 * 描述:		组合HostCmd_CMD_SUPPLICANT_PMK cmd封包的cmd body部分，
 				这个命令的主要作用是:
 				如果遇到连接的动作，让芯片做4方握手的认证
******************************************************************************/
static uint8_t mrvl88w8801_ass_supplicant_pmk_pkg(uint8_t *bssid)
{
    uint16_t pmk_len = 0;
    uint8_t pmk_tlv[64];
    MrvlIEtypes_SSIDParamSet_t *pmk_ssid_tlv;
    MrvlIETypes_BSSIDList_t *pmk_bssid_tlv;
    MrvlIEtypes_PASSPhrase_t *pmk_phrase_tlv;

    pmk_ssid_tlv	=  (MrvlIEtypes_SSIDParamSet_t *)pmk_tlv;
    pmk_ssid_tlv->header.type = TLV_TYPE_SSID;
    pmk_ssid_tlv->header.len = pmrvl88w8801_core->con_info.ssid_len; //SSID长度是多少
		//把密码拷贝进去
    memcpy(pmk_ssid_tlv->ssid,pmrvl88w8801_core->con_info.ssid,pmrvl88w8801_core->con_info.ssid_len);
    pmk_len += sizeof(MrvlIEtypesHeader_t) + pmrvl88w8801_core->con_info.ssid_len;
		
    pmk_bssid_tlv = (MrvlIETypes_BSSIDList_t *)(pmk_tlv+pmk_len);
    pmk_bssid_tlv->header.type = TLV_TYPE_BSSID;
    pmk_bssid_tlv->header.len = TLV_PAYLOADLEN(*pmk_bssid_tlv);//这里的长度计算不一样
		printf(" now %d before is %d\r\n",TLV_PAYLOADLEN(*pmk_bssid_tlv),MAC_ADDR_LENGTH);
		//给的是mac地址
    memcpy(pmk_bssid_tlv->mac_address,bssid,MAC_ADDR_LENGTH);
    pmk_len += sizeof(MrvlIETypes_BSSIDList_t);

    pmk_phrase_tlv = (MrvlIEtypes_PASSPhrase_t *)(pmk_tlv+pmk_len);
    pmk_phrase_tlv->header.type = TLV_TYPE_PASSPHRASE;//提前算好
    pmk_phrase_tlv->header.len = pmrvl88w8801_core->con_info.pwd_len;
    memcpy(pmk_phrase_tlv->phrase,pmrvl88w8801_core->con_info.pwd,pmrvl88w8801_core->con_info.pwd_len);
    //printf("ssid len is %d key len is %d\r\n",pmrvl88w8801_core->con_info.ssid_len,pmrvl88w8801_core->con_info.pwd_len);
		pmk_len += sizeof(MrvlIEtypesHeader_t) + pmrvl88w8801_core->con_info.pwd_len;

		//发送的是set没毛病
    mrvl88w8801_prepare_cmd(HostCmd_CMD_SUPPLICANT_PMK,HostCmd_ACT_GEN_SET,&pmk_tlv,pmk_len);
		return COMP_ERR_OK;
}

/******************************************************************************
 *	函数名:	mrvl88w8801_supplicant_pmk_prepare
 * 参数:  		tx(IN)				-->tx buffer
 				cmd_action(IN)	-->set/get
 				data_buff(IN)		-->mac address，此部分主要用于set
 				data_len(IN)		-->mac的长度
 * 返回值: 	返回执行结果
 * 描述:		组HostCmd_CMD_SUPPLICANT_PMK command的封包
******************************************************************************/
static uint8_t mrvl88w8801_supplicant_pmk_prepare(uint8_t* tx,uint16_t cmd_action,void *data_buff,uint16_t data_len)
{
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    HostCmd_DS_802_11_SUPPLICANT_PMK *psupplicant_pmk = &cmd->params.esupplicant_psk;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_802_11_SUPPLICANT_PMK)+data_len-sizeof(psupplicant_pmk->tlv_buffer);

    cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_SUPPLICANT_PMK;
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;
    if(cmd_action == HostCmd_ACT_GEN_GET)
    {
        psupplicant_pmk->action = HostCmd_ACT_GEN_GET;
    }
    else	/* set mac address */
    {
        psupplicant_pmk->action = HostCmd_ACT_GEN_SET;

    }
    psupplicant_pmk->cache_result = 0;
    memcpy(psupplicant_pmk->tlv_buffer,data_buff,data_len);
    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_ret_scan
 * 参数:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * 返回值: 	返回执行结果
 * 描述:		解析搜索命令的cmd response,注意搜索的TLV是IEEE的TLV
 				不是Marvell的TLV
******************************************************************************/
static uint8_t mrvl88w8801_ret_scan(uint8_t *rx_buffer,int len)
{
   uint8_t j;
    uint8_t index = 0;
    uint16_t ie_size;
    WiFi_Vendor *vendor;
    IEEEType *rates;
    uint8_t *encryption_mode;
    uint8_t ssid[MAX_SSID_LENGTH+1], channel;

    uint8_t vendor_tlv_count = 0;
    IEEEType *vendor_data_ptr[8];
    IEEEType *rsn_data_ptr;

    IEEEType *ie_params;
    WiFi_SecurityType security;
    bss_desc_set_t*bss_desc_set;
    HostCmd_DS_802_11_SCAN_RSP *pscan_rsp = (HostCmd_DS_802_11_SCAN_RSP *)rx_buffer;

    COMP_DEBUG("bss_descript_size %d\n",pscan_rsp->bss_descript_size);
    COMP_DEBUG("number_of_sets %d\n",pscan_rsp->number_of_sets);

    if(pmrvl88w8801_core->con_info.con_status == CON_STA_CONNECTING && pscan_rsp->number_of_sets == 0)
    {
        COMP_DEBUG("WIFI ERR:can not find ap\n");
    }

    /* 判断搜索到的AP个数 */
    if (pscan_rsp->number_of_sets > 0)
    {
        bss_desc_set = (bss_desc_set_t *)(pscan_rsp->bss_desc_and_tlv_buffer);
        for (index = 0; index < pscan_rsp->number_of_sets; index++)
        {
            security = WIFI_SECURITYTYPE_WEP;
            rates = NULL;
            ie_params = &bss_desc_set->ie_parameters;
			if(bss_desc_set->ie_length > ((sizeof(bss_desc_set_t) + sizeof(bss_desc_set->ie_length) + sizeof(bss_desc_set->ie_parameters))))
            {
				ie_size = bss_desc_set->ie_length - (sizeof(bss_desc_set_t) - sizeof(bss_desc_set->ie_length) - sizeof(bss_desc_set->ie_parameters));
            }
			else
			{
				ie_size = 0;
			}
			while (ie_size > 0)
      {
				/* 判断各个TLV */
				switch (ie_params->header.type)
				{
				case TLV_TYPE_SSID:
						if(ie_params->header.length <= MAX_SSID_LENGTH)
						{
								comp_memcpy(ssid, ie_params->data, ie_params->header.length);
								ssid[ie_params->header.length] = '\0';
						}
						else
						{
								comp_memcpy(ssid, ie_params->data, MAX_SSID_LENGTH);
								ssid[MAX_SSID_LENGTH] = '\0';
						}
						break;
				case TLV_TYPE_RATES:
						rates = ie_params;
						break;
				case TLV_TYPE_PHY_DS:
						channel = ie_params->data[0];
						break;
				case TLV_TYPE_RSN_PARAMSET:
						/* 此部分我们认为收到RSN就是WPA2 */
						security = WIFI_SECURITYTYPE_WPA2;
						rsn_data_ptr = ie_params;
						break;
				case TLV_TYPE_VENDOR_SPECIFIC_IE:
						if (security != WIFI_SECURITYTYPE_WPA2)
						{
								vendor = (WiFi_Vendor *)ie_params->data;
								if (vendor->oui[0] == 0x00 && vendor->oui[1] == 0x50 && vendor->oui[2] == 0xf2 && vendor->oui_type == 0x01)
										security = WIFI_SECURITYTYPE_WPA;
						}
						if(vendor_tlv_count < 8)
						{
								vendor_data_ptr[vendor_tlv_count] = ie_params;
								vendor_tlv_count++;
						}
						break;
				}
				ie_size -= TLV_STRUCTLEN(*ie_params);
				ie_params = (IEEEType *)TLV_NEXT(ie_params);
     }
		if ((bss_desc_set->cap_info & WIFI_CAPABILITY_PRIVACY) == 0)
				security = WIFI_SECURITYTYPE_NONE;

		pmrvl88w8801_core->con_info.security = security;
		COMP_DEBUG("SSID '%s', ", ssid); /* SSID 名称 */
		COMP_DEBUG("MAC %02X:%02X:%02X:%02X:%02X:%02X, ", bss_desc_set->bssid[0], bss_desc_set->bssid[1], bss_desc_set->bssid[2], bss_desc_set->bssid[3], bss_desc_set->bssid[4], bss_desc_set->bssid[5]); /* MAC地址 */
		COMP_DEBUG("RSSI %d, Channel %d\n", bss_desc_set->rssi, channel); /* 信号强度和通道号 */
		COMP_DEBUG("  Capability: 0x%04x (Security: ", bss_desc_set->cap_info);
		switch (security)
		{
			case WIFI_SECURITYTYPE_NONE:
					encryption_mode = "OPEN";
					COMP_DEBUG("Unsecured");
					break;
			case WIFI_SECURITYTYPE_WEP:
					encryption_mode = "WEP";
					COMP_DEBUG("WEP");
					break;
			case WIFI_SECURITYTYPE_WPA:
					encryption_mode = "WPA";
					COMP_DEBUG("WPA");
					break;
			case WIFI_SECURITYTYPE_WPA2:
					encryption_mode = "WPA2";
					COMP_DEBUG("WPA2");
					break;
		}
		COMP_DEBUG(", Mode: ");
		if (bss_desc_set->cap_info & WIFI_CAPABILITY_IBSS)
				COMP_DEBUG("Ad-Hoc");
		else
				COMP_DEBUG("Infrastructure");
		COMP_DEBUG(")\n");

		if (rates != NULL)
		{
				COMP_DEBUG("  Rates:");
				for (j = 0; j < rates->header.length; j++)
						COMP_DEBUG(" %.1fMbps", (rates->data[j] & 0x7f) * 0.5);
				COMP_DEBUG("\n");
		}


		if(pmrvl88w8801_core->con_info.con_status == CON_STA_CONNECTING )
		{
				/* 做连接的动作 */
				uint8_t rate_tlv[] = {0x01,0x00,0x0c,0x00,0x82,0x84,0x8b,0x8c,0x12,0x96,0x98,0x24,0xb0,0x48,0x60,0x6c};
				uint16_t associate_para_len = 0;
				uint8_t associate_para[256];
				MrvlIEtypes_SSIDParamSet_t *ssid_tlv;
				MrvlIETypes_PhyParamDSSet_t *phy_tlv;
				MrvlIETypes_CfParamSet_t *cf_tlv;
				MrvlIETypes_AuthType_t *auth_tlv;
				MrvlIEtypes_ChanListParamSet_t *channel_list;
				ChanScanParamSet_t *channel_list_para;

				ssid_tlv	=  (MrvlIEtypes_SSIDParamSet_t *)associate_para;
				ssid_tlv->header.type = TLV_TYPE_SSID;
				ssid_tlv->header.len = pmrvl88w8801_core->con_info.ssid_len;
				comp_memcpy(ssid_tlv->ssid,ssid,pmrvl88w8801_core->con_info.ssid_len);
				associate_para_len += sizeof(MrvlIEtypesHeader_t) + pmrvl88w8801_core->con_info.ssid_len;

				phy_tlv = (MrvlIETypes_PhyParamDSSet_t *)(associate_para+associate_para_len);
				phy_tlv->header.type = TLV_TYPE_PHY_DS;
				phy_tlv->header.len = 1;
				phy_tlv->channel = channel;
				associate_para_len +=sizeof(MrvlIETypes_PhyParamDSSet_t);


				cf_tlv = (MrvlIETypes_CfParamSet_t *)(associate_para+associate_para_len);
				comp_memset(cf_tlv,0,sizeof(MrvlIETypes_CfParamSet_t));
				cf_tlv->header.type = TLV_TYPE_CF;
				cf_tlv->header.len = sizeof(MrvlIETypes_CfParamSet_t) - sizeof(MrvlIEtypesHeader_t);
				associate_para_len +=sizeof(MrvlIETypes_CfParamSet_t);

				if(security == WIFI_SECURITYTYPE_NONE)
				{
						auth_tlv = (MrvlIETypes_AuthType_t *)(associate_para+associate_para_len);
						auth_tlv->header.type = TLV_TYPE_AUTH_TYPE;
						auth_tlv->header.len = sizeof(MrvlIETypes_AuthType_t) - sizeof(MrvlIEtypesHeader_t);
						auth_tlv->auth_type = WIFI_AUTH_MODE_OPEN;
						associate_para_len +=sizeof(MrvlIETypes_AuthType_t);
				}

				channel_list = (MrvlIEtypes_ChanListParamSet_t *)(associate_para+associate_para_len);
				channel_list_para = (ChanScanParamSet_t *)(associate_para+associate_para_len + sizeof(MrvlIEtypesHeader_t));
				channel_list->header.type = TLV_TYPE_CHANLIST;
				channel_list->header.len = sizeof(ChanScanParamSet_t);
				channel_list_para->radio_type = 0;
				channel_list_para->chan_number = channel;
				channel_list_para->chan_scan_mode = 0;
				channel_list_para->min_scan_time = 0;
				channel_list_para->max_scan_time = 200;
				associate_para_len +=sizeof(MrvlIEtypes_ChanListParamSet_t);

				comp_memcpy(associate_para+associate_para_len,rate_tlv,sizeof(rate_tlv));
				associate_para_len += sizeof(rate_tlv);

				if(security >= WIFI_SECURITYTYPE_WPA)
				{
						MrvlIETypes_Vendor_t *vendor_tlv;
						MrvlIETypes_RSN_t *rsn_tlv;
						printf("\r\n");
						for(index = 0 ; index < pmrvl88w8801_core->con_info.ssid_len;index++)
							printf("0x%2x ",bss_desc_set->bssid[index]);
						printf("\r\n");
						mrvl88w8801_ass_supplicant_pmk_pkg(bss_desc_set->bssid);

						for(index = 0; index < vendor_tlv_count; index++)
						{
								vendor_tlv = (MrvlIETypes_Vendor_t *)(associate_para+associate_para_len);
								vendor_tlv->header.type = TLV_TYPE_VENDOR_SPECIFIC_IE;
								vendor_tlv->header.len = vendor_data_ptr[index]->header.length;
								comp_memcpy(vendor_tlv->vendor,vendor_data_ptr[index]->data,vendor_tlv->header.len);
								associate_para_len += sizeof(MrvlIEtypesHeader_t) + vendor_tlv->header.len;
						}

						if(security == WIFI_SECURITYTYPE_WPA2)
						{
								printf("看起来是WPA2的连接\r\n");
								//printf("???\r\n");
								rsn_tlv = (MrvlIETypes_RSN_t *)(associate_para+associate_para_len);
								rsn_tlv->header.type = TLV_TYPE_RSN_PARAMSET;
								rsn_tlv->header.len = rsn_data_ptr->header.length;
								//printf("cpoy 开始了\r\n");
								memcpy(rsn_tlv->rsn,rsn_data_ptr->data,rsn_tlv->header.len);
								//printf("cpoy 结束了");
								associate_para_len += sizeof(MrvlIEtypesHeader_t) + rsn_tlv->header.len;
						}
				}
				printf("before 发送MAC ASSOCIATE命令\r\n");
				//这里吧mac地址拷贝过来了后面就能直接用
				comp_memcpy(pmrvl88w8801_core->con_info.mac_address,bss_desc_set->bssid,MAC_ADDR_LENGTH);
				pmrvl88w8801_core->con_info.cap_info = bss_desc_set->cap_info;
				//hw_hex_dump((uint8_t *)&associate_para,associate_para_len);
				printf("发送MAC ASSOCIATE命令\r\n");
				mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_ASSOCIATE,HostCmd_ACT_GEN_GET,&associate_para,associate_para_len);

			}
			else
			{
					if(mrvl_wifi_cb && mrvl_wifi_cb->wifi_scan_result)
							mrvl_wifi_cb->wifi_scan_result(ssid,bss_desc_set->rssi,channel,encryption_mode);
			}
			/* 解析下一个热点 */
			bss_desc_set = (bss_desc_set_t *)((uint8_t *)bss_desc_set + sizeof(bss_desc_set->ie_length) + bss_desc_set->ie_length);

		}
	}
		
    return COMP_ERR_OK;
}
// 发送命令
static uint8_t mrvl88w8801_prepare_cmd(uint16_t cmd_id,uint16_t cmd_action,void *data_buff,uint16_t data_len)
{
    uint8_t *tx = mrvl_tx_buffer;
    uint16_t tx_len;
		uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
    /* 组包处理 */
    switch(cmd_id)
    {
			  case HostCmd_CMD_FUNC_INIT: //初始化固件的时候用 只有初始化了才能正常使用
				{
					/* FUNC INIT的命令 */
					mrvl88w8801_func_init(tx);
					printf("初始化完毕了\r\n");
					break;
				}
				case HostCmd_CMD_MAC_CONTROL: //MAC control 接着上一条发送的动作
				{
						mrvl88w8801_mac_control(tx,data_buff);
						break;
				}
				case HostCmd_CMD_GET_HW_SPEC:  //第三条需要的动作
				{
						mrvl88w8801_get_hw_spec(tx);
						break;
				}
				case HostCmd_CMD_802_11_MAC_ADDRESS:
				{
						mrvl88w8801_mac_addr_prepare(tx,cmd_action,data_buff,data_len);
						break;
				}
				case HostCmd_CMD_802_11_SCAN: //扫描能用的连接 在scan函数最后进行调用
				{
						mrvl88w8801_scan_prepare(tx,data_buff,data_len);
						//scan之前的准备工作
						break;
				}
				case HostCmd_CMD_SUPPLICANT_PMK: //做连接的时候用到的
				{
						printf("这是给WPA2连接准备的发送命令 自然MAC CONNCET cmd就没用了\r\n");
						mrvl88w8801_supplicant_pmk_prepare(tx,cmd_action,data_buff,data_len);
						break;
				}
				case HostCmd_CMD_802_11_ASSOCIATE: //准备连接了
				{
						printf("MAC CONNCET cmd\r\n");
						mrvl88w8801_connect_prepare(tx,data_buff,data_len);
						break;
				}
				 case HostCmd_CMD_802_11_DEAUTHENTICATE://准备断开连接了
				{
						mrvl88w8801_disconnect_prepare(tx);
						break;
				}

		}
		if(pmrvl88w8801_core->con_info.security >= WIFI_SECURITYTYPE_WPA && cmd_id == HostCmd_CMD_802_11_ASSOCIATE)
    {
				printf("进到这里很正常有个命令不改发送出去\r\n");
				delay_ms(100);
    }
		else
		{
			tx_len = tx[0] | (tx[1] << 8);
			// 地址不要自增--这里是0
			WiFi_LowLevel_WriteData(1,control_io_port +CTRL_PORT,tx,tx_len,
												TX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
			printf("发送完毕 %d\r\n",tx_len);
		}
		return COMP_ERR_OK;
}

// 我们发送了命令就会有响应 根据响应就要做一些其他的动作
// 有两种 相应收到直接发送下一条 或者处理相应后发送下一条
static uint8_t mrvl88w8801_process_cmdrsp(uint8_t *rx_buffer,int len)
{
		// 还是一个指针的强转
	  HostCmd_DS_COMMAND *resp = (HostCmd_DS_COMMAND *) rx_buffer;
		// 看看命令是不是需要回复的
    uint16_t cmdresp_no = resp->command & (~HostCmd_RET_BIT);
		// 命令是不是OK
    if(resp->result != HostCmd_RESULT_OK)
    {
        return WIFI_ERR_INVALID_RESPONSE;
    }
		 switch(cmdresp_no)
    {
				// 对于初始化的响应工作
		    case HostCmd_CMD_FUNC_INIT:
				{
					printf("step1 响应初始化\r\n");
					uint16_t mac_control_action = HostCmd_ACT_MAC_RX_ON | HostCmd_ACT_MAC_TX_ON |HostCmd_ACT_MAC_ETHERNETII_ENABLE;
					mrvl88w8801_prepare_cmd(HostCmd_CMD_MAC_CONTROL,HostCmd_ACT_GEN_SET,(void *)&mac_control_action,2);
					break;
				}
				// 对于MAC的响应
				 case HostCmd_CMD_MAC_CONTROL:
				{
						printf("step2 响应MAC初始化\r\n");
						mrvl88w8801_prepare_cmd(HostCmd_CMD_GET_HW_SPEC,HostCmd_ACT_GEN_GET,NULL,0);

						break;
				}
				case HostCmd_CMD_GET_HW_SPEC:
				{
						mrvl88w8801_ret_get_hw_spec(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_MAC_ADDRESS,HostCmd_ACT_GEN_GET,NULL,0);

						break;
				}
				case HostCmd_CMD_802_11_MAC_ADDRESS: //对这个命令的响应
				{
						mrvl88w8801_ret_mac_address(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						break;
				}
				case HostCmd_CMD_802_11_SCAN: //对于scan的响应在中断中调用
				{
						mrvl88w8801_ret_scan(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						break;
				}
				 case HostCmd_CMD_SUPPLICANT_PMK: //对内个握手命令的响应 好奇怪竟然是发送？
				{
						printf("接受到相应了 所以就可以发送?\r\n");
						uint16_t tx_len = mrvl_tx_buffer[0] | (mrvl_tx_buffer[1] << 8);
						WiFi_LowLevel_WriteData(1,pmrvl88w8801_core->control_io_port +CTRL_PORT,mrvl_tx_buffer,tx_len,TX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);

						break;
				}
				case HostCmd_CMD_802_11_ASSOCIATE: //对于conncet的响应
				{
						printf("ret conncet\r\n");
						mrvl88w8801_ret_connect(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						break;
				}
				 case HostCmd_CMD_802_11_DEAUTHENTICATE: //对于主动断开连接的响应
				{
						/* STA主动断线只会收到cmd response */
						break;
				}
				default:
					printf("error response %d\r\n",cmdresp_no);
					break;
		}
		return COMP_ERR_OK;
		
}

/******************************************************************************
 *	函数名:	mrvl88w8801_handle_interrupt
 * 参数:  		NULL
 * 返回值: 	返回执行结果
 * 描述:		发生download/upload interrupt做的处理
******************************************************************************/
void mrvl88w8801_handle_interrupt(int32_t *status)
{
    /* 读ctrl寄存器 */
		/* 获取中断寄存器的状态 */
		// 是的只要读取了寄存器 电平就会从低恢复位高了
		*status = WiFi_LowLevel_ReadReg(1, WIFI_INTSTATUS); // 获取需要处理的中断标志位
		if(*status <= 0)
		{
				//	printf("没有读取到\r\n");
				//WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~(0)); //清除保证
				return;
		}
		memset(pmrvl88w8801_core->mp_regs,0x00,256);
		WiFi_LowLevel_ReadData(1,REG_PORT,pmrvl88w8801_core->mp_regs,64,256,WIFI_RWDATA_ALLOWMULTIBYTE);
    pmrvl88w8801_core->read_bitmap = (uint16_t) pmrvl88w8801_core->mp_regs[RD_BITMAP_L];
    pmrvl88w8801_core->read_bitmap |=((uint16_t) pmrvl88w8801_core->mp_regs[RD_BITMAP_U]) << 8;
    pmrvl88w8801_core->write_bitmap = (uint16_t) pmrvl88w8801_core->mp_regs[WR_BITMAP_L];
    pmrvl88w8801_core->write_bitmap |=((uint16_t) pmrvl88w8801_core->mp_regs[WR_BITMAP_U]) << 8;
		//printf("写中断寄存器前%d\r\n",WiFi_LowLevel_GetITStatus(1));
		// 所以这里很尴尬如果我在写(清除)的时候 wifi模块也在往里写怎么办呢
		WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~(*status)); //清除保证能读到
		//printf("写中断寄存器后%d\r\n",WiFi_LowLevel_GetITStatus(1));
}

/******************************************************************************
 *	函数名:	mrvl88w8801_get_read_port
 * 参数:  		port(OUT)			-->返回read port
 * 返回值: 	返回执行结果
 * 描述:		用于获取read port
******************************************************************************/
static uint8_t mrvl88w8801_get_read_port(uint8_t *port)
{
		//printf("进入读取端口函数了\r\n");
    if (pmrvl88w8801_core->read_bitmap & CTRL_PORT_MASK)
    {
				//printf("进入到情形1了\r\n"); //是奇数就会进到这里
				//除了最后一一位别的保持不变 怕有多余的要读的位
        pmrvl88w8801_core->read_bitmap &= (uint16_t) (~CTRL_PORT_MASK);
        *port = CTRL_PORT;
    }
    else
    {
				//按位与了一下找到了下一个要读的端口1
				//对于一个0xFF 对它的每一位都按位与去找要读的端口 然后从端口取出来数据
        if (pmrvl88w8801_core->read_bitmap & (1 << pmrvl88w8801_core->curr_rd_port))
        {
            pmrvl88w8801_core->read_bitmap &=(uint16_t) (~(1 << pmrvl88w8801_core->curr_rd_port));
            *port = pmrvl88w8801_core->curr_rd_port;

            /* hw rx wraps round only after port (MAX_PORT-1) */
            if (++pmrvl88w8801_core->curr_rd_port == MAX_PORT)
                /* port 0 is reserved for cmd port */
                pmrvl88w8801_core->curr_rd_port = 1;
        }
        else
        {
            //printf("mrvl88w8801_get_read_port error\n");
            return WIFI_ERR_NO_MORE_READ_HANDLE;
        }
    }
    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_process_event
 * 参数:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * 返回值: 	返回执行结果
 * 描述:		处理收到的event
******************************************************************************/
static uint8_t mrvl88w8801_process_event(uint8_t *rx_buffer,int len)
{
    uint16_t event = *(uint16_t *)(rx_buffer + CMD_SDIO_HDR_SIZE);

    //COMP_DEBUG("WIFI EVENT ID 0x%x\n",event);
    switch(event)
    {
			case EVENT_PORT_RELEASE: //这里在创思里面是进行了使能操作我们不用
			{
					COMP_DEBUG("EVENT_PORT_RELEASE\n");
					COMP_DEBUG("WIFI:connected mac:\n");
					if(mrvl_wifi_cb && mrvl_wifi_cb->wifi_connect_result)
							mrvl_wifi_cb->wifi_connect_result(0);
					break;
			}
			case EVENT_WMM_STATUS_CHANGE:
			{
					COMP_DEBUG("EVENT_WMM_STATUS_CHANGE\n");
					break;
			}
			case EVENT_DISASSOCIATED:
			{
					/* AP把STA移除或者AP关闭，STA都会收到此消息 */
					COMP_DEBUG("EVENT_DISASSOCIATED\n");
					ethernet_link_down();
					break;
			}
			default:
				printf("unknown event type\r\n");
				break;
		}
		return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_get_write_port
 * 参数:  		port(OUT)			-->返回read port
 * 返回值: 	返回执行结果
 * 描述:		用于获取write port
******************************************************************************/
static uint8_t mrvl88w8801_get_write_port(uint8_t *port)
{
    if (pmrvl88w8801_core->write_bitmap & (1 << pmrvl88w8801_core->curr_wr_port))
    {
        pmrvl88w8801_core->write_bitmap &= (uint32_t) (~(1 << pmrvl88w8801_core->curr_wr_port));
        *port = pmrvl88w8801_core->curr_wr_port;
        if (++pmrvl88w8801_core->curr_wr_port == pmrvl88w8801_core->mp_end_port)
            pmrvl88w8801_core->curr_wr_port = 1;
    }
    else
    {
    }
    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_get_send_data_buf
 * 参数:  		NULL
 * 返回值: 	返回TX buffer的payload，也就是tcp/ip数据部分
 * 描述:		用于tcp/ip调用，直接写tcp/ip数据
******************************************************************************/
uint8_t *mrvl88w8801_get_send_data_buf()
{
		memset(mrvl_tx_buffer,0x00,3*1024);
    TxPD *tx_packet = (TxPD *)(mrvl_tx_buffer);
    pmrvl88w8801_core->tx_data_ptr = mrvl_tx_buffer;
    return (tx_packet->payload);//进行了有效移位
}
/******************************************************************************
 *	函数名:	mrvl88w8801_send_data
 * 参数:  		data(IN)			-->要发送的data
 				size(IN)			-->要发送的data size
 * 返回值: 	返回执行结果
 * 描述:		发送data
******************************************************************************/
uint8_t mrvl88w8801_send_data(uint8_t *data,uint16_t size)
{
    uint8_t wr_bitmap_l;
    uint8_t wr_bitmap_u;
    uint8_t port = 0;
			
    TxPD *tx_packet = (TxPD *)(pmrvl88w8801_core->tx_data_ptr);
    uint16_t tx_packet_len = sizeof(TxPD) - sizeof(tx_packet->payload) + size;
		//printf("进来之后算法tx_packet_len 是%d\r\n",tx_packet_len);
    tx_packet->pack_len = tx_packet_len;
    tx_packet->pack_type = TYPE_DATA;

    tx_packet->bss_type = pmrvl88w8801_core->bss_type;
    tx_packet->bss_num = 0;
    tx_packet->tx_pkt_length = size;
    tx_packet->tx_pkt_offset = tx_packet_len - size - 4;
    tx_packet->tx_pkt_type = 0;
    tx_packet->tx_control = 0;
    tx_packet->priority = 0;
    tx_packet->flags = 0;
    tx_packet->pkt_delay_2ms = 0;
    tx_packet->reserved1 = 0;
    wr_bitmap_l = WiFi_LowLevel_ReadReg(1,WR_BITMAP_L);
    wr_bitmap_u = WiFi_LowLevel_ReadReg(1,WR_BITMAP_U);
    pmrvl88w8801_core->write_bitmap = wr_bitmap_l;
    pmrvl88w8801_core->write_bitmap |=wr_bitmap_u << 8;
    /* 针对write port做处理 */
    mrvl88w8801_get_write_port(&port);
    WiFi_LowLevel_WriteData(1,pmrvl88w8801_core->control_io_port+port,
			(uint8_t*)tx_packet,tx_packet->pack_len,TX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
		
//		printf("作为lwip的一部分发送数据完毕%d\r\n",size);
    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_process_data
 * 参数:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * 返回值: 	返回执行结果
 * 描述:		处理收到的data(也就是tcp/ip data)
******************************************************************************/
static uint8_t mrvl88w8801_process_data(uint8_t *rx_buffer,int len)
{
		
    if(pmrvl88w8801_core->bss_type == BSS_TYPE_UAP)
    {
				//printf("进入到bss_type_uap中了\r\n");
        RxPD *rx_packet = (RxPD *)rx_buffer;
        uint16_t rx_payload_len = rx_packet->rx_pkt_length;
        uint8_t *payload = (uint8_t *)((uint8_t *)rx_packet + rx_packet->rx_pkt_offset + CMD_SDIO_HDR_SIZE);
        RxPacketHdr_t *rx_hdr = (RxPacketHdr_t *)payload;

#if 0
        COMP_DEBUG("Rx dest %02x:%02x:%02x:%02x:%02x:%02x\n",
                   rx_hdr->eth803_hdr.dest_addr[0], rx_hdr->eth803_hdr.dest_addr[1],
                   rx_hdr->eth803_hdr.dest_addr[2], rx_hdr->eth803_hdr.dest_addr[3],
                   rx_hdr->eth803_hdr.dest_addr[4], rx_hdr->eth803_hdr.dest_addr[5]);

        COMP_DEBUG("local mac %02x:%02x:%02x:%02x:%02x:%02x\n",
                   pmrvl88w8801_core->mac_address[0], pmrvl88w8801_core->mac_address[1],
                   pmrvl88w8801_core->mac_address[2], pmrvl88w8801_core->mac_address[3],
                   pmrvl88w8801_core->mac_address[4], pmrvl88w8801_core->mac_address[5]);
#endif
        if (rx_hdr->eth803_hdr.dest_addr[0] & 0x01)
        {
            /* 1.广播封包，转发 */
						//printf("广播封包\r\n");
            uint8_t *buffer = mrvl88w8801_get_send_data_buf();
            uint8_t *payload = (uint8_t *)((uint8_t *)rx_packet + rx_packet->rx_pkt_offset + 4);
            comp_memcpy(buffer,payload,rx_payload_len);
            mrvl88w8801_send_data(buffer,rx_payload_len);

            /* 2.自己AP处理广播封包 */
            ethernet_lwip_process(rx_buffer,len);
        }
        else
        {
            /* 单播封包 */
            if(memcmp(pmrvl88w8801_core->mac_address,rx_hdr->eth803_hdr.dest_addr,MAC_ADDR_LENGTH) == 0)
            {
								//printf("单播封包\r\n");
                ethernet_lwip_process(rx_buffer,len);

            }
            else
            {
                /* 判断是否是STA地址 ，转发 */
								//printf("STA转包\r\n");
                uint8_t *buffer = mrvl88w8801_get_send_data_buf();
                uint8_t *payload = (uint8_t *)((uint8_t *)rx_packet + rx_packet->rx_pkt_offset + 4);
                comp_memcpy(buffer,payload,rx_payload_len);
                mrvl88w8801_send_data(buffer,rx_payload_len);
            }
        }

    }

    if(pmrvl88w8801_core->bss_type == BSS_TYPE_STA)
    {
				//printf("进入到bss_type_sta中了 %d\r\n",len);
        ethernet_lwip_process(rx_buffer,len);
    }

    return COMP_ERR_OK;
}
/******************************************************************************
 *	函数名:	mrvl88w8801_parse_rx_packet
 * 参数:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * 返回值: 	返回执行结果
 * 描述:		解析从芯片收到的数据，再分别交由cmd response/event/data
 				解析函数做处理
******************************************************************************/
static uint8_t mrvl88w8801_parse_rx_packet(uint8_t *rx_buffer,int len)
{
    uint16_t rx_type = (uint16_t)rx_buffer[2];
    switch(rx_type)
    {
			case TYPE_CMD_CMDRSP: //现在只用到了这些
			{
//					printf("对数据进行解包\r\n");
					mrvl88w8801_process_cmdrsp(rx_buffer,len);
				
					break;
			}
			/*--------------后面可能用到的-----------------*/
			case TYPE_DATA:
			{
//						printf("处理data数据\r\n");
						mrvl88w8801_process_data(rx_buffer,len);
						break;
			}
			case TYPE_EVENT:
			{
//					printf("处理事件\r\n");
					mrvl88w8801_process_event(rx_buffer,len);
					break;
			}
			default:
			{
//					printf("ERR:unkown rx type %d\r\n",rx_type);
					break;
			}
    }

    return COMP_ERR_OK;
}
uint8_t mrvl88w8801_process_packet_timeout(void)
{
    uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
    uint8_t port;
    int32_t int_status;
    uint32_t len_reg_l, len_reg_u;
    uint16_t rx_len;
    uint8_t* rx_buffer = mrvl_rx_buffer;

		memset(pmrvl88w8801_core->mp_regs,0x00,256);
		WiFi_LowLevel_ReadData(1,REG_PORT,pmrvl88w8801_core->mp_regs,64,256,WIFI_RWDATA_ALLOWMULTIBYTE);
		pmrvl88w8801_core->read_bitmap = (uint16_t) pmrvl88w8801_core->mp_regs[RD_BITMAP_L];
    pmrvl88w8801_core->read_bitmap |=((uint16_t) pmrvl88w8801_core->mp_regs[RD_BITMAP_U]) << 8;
    pmrvl88w8801_core->write_bitmap = (uint16_t) pmrvl88w8801_core->mp_regs[WR_BITMAP_L];
    pmrvl88w8801_core->write_bitmap |=((uint16_t) pmrvl88w8801_core->mp_regs[WR_BITMAP_U]) << 8;
    /* 获取中断寄存器状态 */
    int_status = pmrvl88w8801_core->mp_regs[HOST_INT_STATUS_REG];
		if(int_status > 0 && int_status <= 3) // 这里把等于去了
		{
				// 明天试试不write行不行
				WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~(int_status)); //清除保证
				//if(int_status > 0)
						//printf("TM为啥啊有中断寄存器值却没触发中断 %d\r\n",int_status);
		}else if(int_status)
		{
				printf("读取发生了致命错误%d \r\n",int_status);
				return 0;
		}
		//直接看寄存器不管那么多 
    while(1)
    {
         /* 获得read port */
         if(mrvl88w8801_get_read_port(&port) != COMP_ERR_OK)
          {
               //COMP_DEBUG("no more rd_port to be handled\n");
              break;
           }

            /* 获取需要读取的封包长度 */
            len_reg_l = RD_LEN_P0_L + (port << 1);
            len_reg_u = RD_LEN_P0_U + (port << 1);
            rx_len = ((uint16_t) pmrvl88w8801_core->mp_regs[len_reg_u]) << 8;
            rx_len |= (uint16_t) pmrvl88w8801_core->mp_regs[len_reg_l];
            //COMP_DEBUG("@@@@@@@RX: port=%d rx_len=%u\n", port, rx_len);

            if(rx_buffer != NULL)
            {
                /* 使用CMD53读取收到的数据 */
									WiFi_LowLevel_ReadData(1,control_io_port+port,rx_buffer,rx_len,
																			RX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
                /* 解析数据 */
                mrvl88w8801_parse_rx_packet(rx_buffer,rx_len);
            }
     }
    return COMP_ERR_OK;
}
void rtos_process_rxirq_packet(int32_t int_status)
{
		uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
    uint8_t port;
    uint32_t len_reg_l, len_reg_u;
    uint16_t rx_len;
    uint8_t* rx_buffer = mrvl_rx_buffer;
		uint8_t read_count = 0;
		// 这里挂掉了
		
		//现在的问题是这里怎么对上 因为肯定是要快速清除的 清除完之后如果没有快速处理信息就完了
		if((int_status & UP_LD_HOST_INT_MASK) == UP_LD_HOST_INT_MASK)
    {
				read_count = 0;
					//有数据可读就去看看有哪些
					while(1) //问题在哪里了就是因为我们的程序用了while去处理然后比如下次中断来了 然后这次的可能还没处理结束
					{
            if(mrvl88w8801_get_read_port(&port) != COMP_ERR_OK)
            {
                //printf("no more rd_port to be handled\r\n");
                break;
            }
						read_count++;
						//printf("这是第%d次的端口 端口号是%d\r\n",read_count,port);
					//	printf("port is %d\r\n",port);
            /* 获取需要读取的封包长度 */
            len_reg_l = RD_LEN_P0_L + (port << 1); //length存放的位置在哪里
            len_reg_u = RD_LEN_P0_U + (port << 1);
            rx_len = ((uint16_t) pmrvl88w8801_core->mp_regs[len_reg_u]) << 8;
            rx_len |= (uint16_t) pmrvl88w8801_core->mp_regs[len_reg_l];
//            printf("@@@@@@@RX: port=%d rx_len=%u\r\n", port, rx_len);

            if(rx_buffer != NULL)
            {
								/* 使用CMD53读取收到的数据 */
								// 这里有个问题 读取多长的数据
								WiFi_LowLevel_ReadData(1,control_io_port+port,rx_buffer,rx_len,
																			RX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
//                printf("准备开始解包\r\n");
								mrvl88w8801_parse_rx_packet(rx_buffer,rx_len);
            }
					}
							
    }
		if((int_status & DN_LD_HOST_INT_MASK) == 1)
		{
				printf("有可以写入的端口了\r\n");
		}
}
// 在多线程环境下没有考虑过这个全局变量的共享性万一被两个线程都访问了就完蛋了

uint8_t mrvl88w8801_process_packet(void)
{
    uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
    uint8_t port;
    int32_t int_status;
    uint32_t len_reg_l, len_reg_u;
    uint16_t rx_len;
    uint8_t* rx_buffer = mrvl_rx_buffer;

    /* 处理中断 */
    mrvl88w8801_handle_interrupt(&int_status);
		//不管读到的状态如何 这个函数就保证了能在正常使用
		if(int_status == 0) //进来了但是没有中断发生,怎么样能让电平状态上去呢
		{
				printf("当int_status == 0时退出process_packet的状态是%d\r\n",WiFi_LowLevel_GetITStatus(1));
				return 2;
		}
		//printf("读取到的中断寄存器状态是%d\r\n",int_status);
		//芯片接受到数据后上报中断
		if((int_status & UP_LD_HOST_INT_MASK) == UP_LD_HOST_INT_MASK)
    {
					while(1) //问题在哪里了就是因为我们的程序用了while去处理然后比如下次中断来了 然后这次的可能还没处理结束
					{
            if(mrvl88w8801_get_read_port(&port) != COMP_ERR_OK)
            {
                //printf("no more rd_port to be handled\r\n");
                break;
            }
						printf("port is %d\r\n",port);
            /* 获取需要读取的封包长度 */
            len_reg_l = RD_LEN_P0_L + (port << 1);
            len_reg_u = RD_LEN_P0_U + (port << 1);
            rx_len = ((uint16_t) pmrvl88w8801_core->mp_regs[len_reg_u]) << 8;
            rx_len |= (uint16_t) pmrvl88w8801_core->mp_regs[len_reg_l];
            //printf("@@@@@@@RX: port=%d rx_len=%u\n", port, rx_len);
						//printf("len_reg_l is %d  len_reg_u is %d\r\n",len_reg_l,len_reg_u);

            if(rx_buffer != NULL)
            {
								/* 使用CMD53读取收到的数据 */
								// 这里有个问题 读取多长的数据
								WiFi_LowLevel_ReadData(1,control_io_port+port,rx_buffer,rx_len,
																			RX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
								//printf("读取数据完毕 rx_len %d\r\n",rx_len);
                mrvl88w8801_parse_rx_packet(rx_buffer,rx_len);
            }
					}
							
    }
		//是发送数据后可以接收到中断
		if((int_status & DN_LD_HOST_INT_MASK) == DN_LD_HOST_INT_MASK)
		{
				printf("数据下载完毕中断\r\n");
		}
		printf("正常情况退出process_packet的状态是%d\r\n",WiFi_LowLevel_GetITStatus(1));
    return COMP_ERR_OK;
}

/* 固件下载 */
// WiFi模块必须要有固件才能正常工作, 固件是Marvell公司 (WiFi模块的制造商)开发的
// 固件是保存在STM32单片机的Flash里面的, Flash里面保存的内容在断电时不会丢失
// 通电时 (或单片机复位后), 单片机把保存在Flash里面的固件通过SPI或SDIO接口发送给WiFi模块
// 保存到WiFi模块的SRAM (内存)里面运行, SRAM里面的内容一断电或复位就会丢失
// 所以每次断电或复位后都要重新下载固件
/* 获取固件状态 */
uint16_t WiFi_GetFirmwareStatus(void)
{
  return WiFi_LowLevel_ReadReg(1, WIFI_SCRATCH0_0) | (WiFi_LowLevel_ReadReg(1, WIFI_SCRATCH0_1) << 8);
}
/* 在规定的超时时间内, 等待指定的卡状态位置位, 并清除相应的中断标志位 */
// 成功时返回1
int WiFi_Wait(uint8_t status, uint32_t timeout)
{
		uint32_t diff, start;
		
		start = HAL_GetTick();
		while ((WiFi_LowLevel_ReadReg(1, WIFI_INTSTATUS) & status) != status)
		{
				diff = HAL_GetTick() - start;
				if (timeout != 0 && diff > timeout)
				{
						// 若超时时间已到
						printf("WiFi_Wait(0x%2x): timeout!\n", status);
						return 0;
				}
		}
		// 清除对应的中断标志位
		WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~status); 
		// 不能将SDIOIT位清除掉! 否则有可能导致该位永远不再置位
		return 1;
}

static void WiFi_DownloadFirmware(void)
{
	
		const uint8_t *data;
		int ret;
		uint16_t curr;
		uint8_t retry;
		uint32_t len = 255536;//固件的大小
		uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
		uint8_t *recv = mymalloc(SRAMEX,1500); //这个是不是会超过函数的栈内存呢如果直接定义局部变量
		printf("%p",recv);
		uint32_t fw_addr = ADDR_FLASH_SECTOR_10;
		if (WiFi_GetFirmwareStatus() == WIFI_FIRMWARESTATUS_OK)
		{
			// PDN引脚没有正确连接到单片机上, 单片机复位时WiFi模块就无法跟着复位
			// 所以这个时候不需要重新下载固件
			// WiFi模块原有的状态不变 (比如已连上的热点, 已发送但未接收回应的命令)
			// 这个时候继续运行下去, 很可能会出错
			printf("%s: Reset signal has no effect!\r\n", __FUNCTION__);
			printf("Please make sure PDN pin is connected to the MCU!\r\n");
			myfree(SRAMEX,recv);
			return;
		}
 
		while (len)
		{
				// 获取本次应下载的字节数
				// 每次可以发送n>=curr字节的数据, 只能用一个CMD53命令发送, WiFi模块只认前curr字节的数据
				// 88W8801不会出现curr=0的情况, 所以这里不需要while语句
				curr = WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR0) | (WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR1) << 8);
				//printf("curr is %d\r\n",curr);
				//下次允许你发送几个字节
				//这个curr一定能除以4的
				stmflash_read(fw_addr,(uint32_t *)recv,curr/4);
				fw_addr += curr; //进行一波自增
				//printf("读取完了准备发送\r\n");
				// 发送固件数据
				// res就是根据CMD53进行操作的
				ret = WiFi_LowLevel_WriteData(1, control_io_port, recv, curr, 0, WIFI_RWDATA_ALLOWMULTIBYTE);
				if (ret == 2)
						break; // 参数有误 没有处理
				ret = WiFi_Wait(WIFI_INTSTATUS_DNLD, 100); // 实际上这里超时了也得注意
				retry = WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR0);
				
				if (retry & 1)
				{
						ret = 0; // 如果寄存器的值为奇数, 说明模块收到的固件内容有误, 需要重传
						retry |= WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR1) << 8;
						printf("%s: Data were not recognized! curr=%d, retry=%d, remaining=%d\n", __FUNCTION__, curr, retry, len);
						break;//还没想好怎么处理
				}
				len -= curr;
				printf("剩余的长度是%d\r\n",len);
				data += curr;
				//读一下看看对不对
		}
  
		// 等待Firmware启动 --但是这里竟然可以成功就TM离谱
		// 注释掉前面的就GG了
		while (WiFi_GetFirmwareStatus() != WIFI_FIRMWARESTATUS_OK);
		printf("Firmware is successfully downloaded!\n");
		delay_ms(500);
}

// 使能下降沿中断 目的就是在RTOS里方便操作一点
void sd_irq_exit_enable(void)
{
			// 使能 GPIO 时钟
		SD_IRQ_GPIO_CLK();

		// 配置 GPIO 为输入模式，保留当前的配置不变
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = SD_IRQ_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // 选择下降沿触发中断
		GPIO_InitStruct.Pull = GPIO_NOPULL;          // 根据当前使用配置拉电阻
		HAL_GPIO_Init(SD_IRQ_GPIO_PORT, &GPIO_InitStruct);
		// 设置中断优先级 注意要受到RTOS的管理 要大于4
		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0); // 优先级根据你的应用调整
			// 使能中断
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

