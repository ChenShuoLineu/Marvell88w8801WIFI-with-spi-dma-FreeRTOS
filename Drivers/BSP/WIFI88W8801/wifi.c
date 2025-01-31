#include "wifi.h"



pmrvl88w8801_core_t pmrvl88w8801_core; //�д��õ�ָ��
// �������еķ��͵�ʱ���ǰ�mrvl_tx_buffer����һ��ǿת
uint8_t *mrvl_tx_buffer; 			//���ͺͽ��ܵ�buf
uint8_t *mrvl_rx_buffer;			
uint8_t *mp_reg_array;				// ������ȡ�Ĵ�����ֵ
wifi_cb_t *mrvl_wifi_cb = NULL; //������һ��ָ���������õ�


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
 *	������:	mrvl88w8801_core_init
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		pmrvl88w8801_core�ṹ���ʼ��
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
		//����ʲôʱ���ͷŰ�--���������������������Ѿ���freertosһ����
		mrvl_tx_buffer = mymalloc(SRAMEX,10*1024);
		mrvl_rx_buffer = mymalloc(SRAMEX,10*1024);
		mp_reg_array = mymalloc(SRAMEX,56);
		pmrvl88w8801_core = mymalloc(SRAMEX,sizeof(mrvl88w8801_core_t));
		mrvl_wifi_cb = cb;				//������main�������б���ֵ�Ľ���
		fun_num = sd_init(); 			//��ʼ����Ӧ��SDIO ���õ��ж��ٸ�����
		mrvl88w8801_core_init(); //��ʼ�����ָ�� �����д���
		if(fun_num == 0)
			return;
		mrvl88w8801_get_control_io_port(); //�õ���Ӧ��IO�Ĵ�����ֵ
		
		WiFi_LowLevel_SetBlockSize(0, 32);
		// ��ʼ��Function 1
		WiFi_LowLevel_WriteReg(0, SDIO_CCCR_IOEN, SDIO_CCCR_IOEN_IOE1); // IOE1=1 (Enable Function)
		while ((WiFi_LowLevel_ReadReg(0, SDIO_CCCR_IORDY) & SDIO_CCCR_IORDY_IOR1) == 0); // �ȴ�IOR1=1 (I/O Function Ready)
		// �����ж� �����ڸ�IRQ��������
		WiFi_LowLevel_WriteReg(0, SDIO_CCCR_INTEN, SDIO_CCCR_INTEN_IENM | SDIO_CCCR_INTEN_IEN1); 
		WiFi_LowLevel_WriteReg(1, WIFI_INTMASK, WIFI_INTMASK_HOSTINTMASK);
		sd_spi_speed_high();//���֧�ֵ�16��Ƶ�Ĳ����ٸ���
		WiFi_LowLevel_SetBlockSize(1, 512);
		WiFi_DownloadFirmware(); //�ѹ̼���flash���������Ȼ�����ص�wifi����
		//��ʱ��ѯһ��״̬��ʲô �����ʱ��0 ���ܶ����� ����0��..
		printf("��ʱ��IRQ״̬%d\r\n",WiFi_LowLevel_GetITStatus(1));//������ Ҳ����������
		//sd_spi_speed_high();//���֧�ֵ�16��Ƶ�Ĳ����ٸ���
		mrvl88w8801_init_fw(); //��ʼ���̼�
		//WiFi_LowLevel_SetBlockSize(1, 1024);
		delay_ms(255);
		
}

//�õ�io_port�Ĵ��� ��������
void mrvl88w8801_get_control_io_port()
{
		uint32_t control_io_port;
		//md/fw data�����������ַȥд��cmd response��ͨ�������ַ������
    control_io_port = WiFi_LowLevel_ReadReg(1, WIFI_IOPORT0) | (WiFi_LowLevel_ReadReg(1, WIFI_IOPORT1) << 8) | 
																						(WiFi_LowLevel_ReadReg(1, WIFI_IOPORT2) << 16);
		printf("control_io_port��%d\r\n",control_io_port);
		pmrvl88w8801_core->control_io_port = control_io_port;
}
/******************************************************************************
 *	������:	mrvl88w8801_init_fw
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		ִ��init fw cmd�������������ȷ���һ������յ�cmd response��
 				ʱ������һ��CMD
******************************************************************************/
uint8_t mrvl88w8801_init_fw()
{
    mrvl88w8801_prepare_cmd(HostCmd_CMD_FUNC_INIT,HostCmd_ACT_GEN_SET,NULL,0);
    return COMP_ERR_OK;
}

//��ʼ��Ӳ�����Ҽ���һЩ�߳�
// ׼����ʼ����
static uint8_t  mrvl88w8801_func_init(uint8_t* tx)
{
		//����û������ɹ�д���������Ҷ�������
		uint16_t tx_packet_len = CMD_HDR_SIZE;
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
	  cmd->pack_len = tx_packet_len;
    cmd->pack_type = TYPE_CMD_CMDRSP;
    cmd->command = HostCmd_CMD_FUNC_INIT; //0X00A9 ��ʼ������
    cmd->size = tx_packet_len - CMD_SDIO_HDR_SIZE;//�ⲻ�ͳ�0����
    cmd->seq_num = 0;
    cmd->bss = 0;
    cmd->result = 0;

		return COMP_ERR_OK;

}
/******************************************************************************
 *	������:	mrvl88w8801_mac_control
 * ����:  	tx(IN)				-->tx buffer
 			data_buff(IN)		-->action��Ϊָ��
 * ����ֵ: 	����ִ�н��
 * ����:	��HostCmd_CMD_MAC_CONTROL command�ķ��
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
 *	������:	mrvl88w8801_get_hw_spec
 * ����:  	tx(IN)				-->tx buffer
 * ����ֵ: 	����ִ�н��
 * ����:	��HostCmd_CMD_GET_HW_SPEC command�ķ��
******************************************************************************/
static uint8_t mrvl88w8801_get_hw_spec(uint8_t* tx)
{
    HostCmd_DS_COMMAND *cmd = (HostCmd_DS_COMMAND *)tx;
    uint16_t tx_packet_len = CMD_HDR_SIZE + sizeof(HostCmd_DS_GET_HW_SPEC);
	
		// �������������һ����װ��ͷ��
		// ��Ϊɶ������12��?CMD_HDR_SIZE
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
 *	������:	mrvl88w8801_ret_get_hw_spec
 * ����:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * ����ֵ: 	����ִ�н��
 * ����:		����GET HW SPEC�������Ӧ���˲�������ֻ�ǻ�ȡwrite data port
 				�����ֵ
******************************************************************************/
static uint8_t mrvl88w8801_ret_get_hw_spec(uint8_t *rx_buffer,int len)
{
    HostCmd_DS_GET_HW_SPEC *hw_spec = (HostCmd_DS_GET_HW_SPEC *)rx_buffer;
		//�õ��˿ںŵ����ֵ
    pmrvl88w8801_core->mp_end_port = hw_spec->mp_end_port;
    return COMP_ERR_OK;
}

/******************************************************************************
 *	������:	mrvl88w8801_mac_addr_prepare
 * ����:  		tx(IN)				-->tx buffer
 				cmd_action(IN)	-->set/get
 				data_buff(IN)		-->mac address���˲�����Ҫ����set
 				data_len(IN)		-->mac�ĳ���
 * ����ֵ: 	����ִ�н��
 * ����:		��HostCmd_CMD_802_11_MAC_ADDRESS command�ķ��
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
 *	������:	mrvl88w8801_ret_mac_address
 * ����:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * ����ֵ: 	����ִ�н��
 * ����:		�˲�����Ҫ�Ǵ���mac cmd reponse������õ�mac����ô������������:
 				1.����pmrvl88w8801_core�ṹ���ж϶�mac_address
 				2.��ʼ��lwip��mac
******************************************************************************/
static uint8_t mrvl88w8801_ret_mac_address(uint8_t *rx_buffer,int len)
{
    HostCmd_DS_802_11_MAC_ADDRESS *pconnect_rsp = (HostCmd_DS_802_11_MAC_ADDRESS *)rx_buffer;
    memcpy(pmrvl88w8801_core->mac_address,pconnect_rsp->mac_addr,MAC_ADDR_LENGTH);
    printf("mrvl88w8801_ret_mac_address mac dump mac_address is\n");
		for(unsigned char i = 0;i < MAC_ADDR_LENGTH;i++)
			printf("0x%2x:",pmrvl88w8801_core->mac_address[i]);
		printf("\r\n");

	
		//wificb��һ�Ѻ���ָ��
    if(mrvl_wifi_cb && mrvl_wifi_cb->wifi_init_result)
        mrvl_wifi_cb->wifi_init_result(COMP_ERR_OK);
    return COMP_ERR_OK;
}

/******************************************************************************
 *	������:	mrvl88w8801_process_packet
 * ����:  	channel(IN)		-->channel����,���ڴ洢�ֱ�����ʲôͨ��
 			channel_num(IN)	-->������ͨ������
 			max_time(IN)		-->���������time
 * ����ֵ: 	����ִ�н��
 * ����:	ִ����ͨ��������
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

    /* ���channel list�Ĳ��� */
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
 *	������:	mrvl88w8801_scan_ssid
 * ����:  		ssid(IN)		-->�ض�������AP����
 				ssid_len(IN)	-->�ض�����AP���Ƶĳ���
 				max_time(IN)	-->���������time
 * ����ֵ: 	����ִ�н��
 * ����:		ִ���ض�������������
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
    /* ���SSID tlv�Ĳ��� */
    ssid_tlv->header.type = TLV_TYPE_SSID;
    ssid_tlv->header.len = ssid_len;
    memcpy(ssid_tlv->ssid,ssid,ssid_len);

    /* ���channel list tlv�Ĳ��� */
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
 *	������:	mrvl88w8801_connect_prepare
 * ����:  		tx(IN)				-->tx buffer
 				data_buff(IN)		-->������cmd body�����������channel list
 				data_len(IN)		-->������cmd body len,���������channel list����
 * ����ֵ: 	����ִ�н��
 * ����:		��HostCmd_CMD_802_11_ASSOCIATE command�ķ��
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
 *	������:	mrvl88w8801_ret_connect
 * ����:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * ����ֵ: 	����ִ�н��
 * ����:		�������������cmd response
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
 *	������:	mrvl88w8801_connect
 * ����:  		ssid(IN)		-->Ҫ���ӵ�AP����
 				ssid_len(IN)	-->Ҫ���ӵ�AP���Ƴ���
 				pwd(IN)		-->Ҫ���ӵ�AP����
 				pwd_len(IN)	-->Ҫ���ӵ�AP���볤��
 * ����ֵ: 	����ִ�н��
 * ����:		����ssid
******************************************************************************/
uint8_t mrvl88w8801_connect(uint8_t *ssid,uint8_t ssid_len,uint8_t *pwd,uint8_t pwd_len)
{
    memcpy(pmrvl88w8801_core->con_info.ssid,ssid,ssid_len);
    pmrvl88w8801_core->con_info.ssid_len = ssid_len;
    memcpy(pmrvl88w8801_core->con_info.pwd,pwd,pwd_len);
    pmrvl88w8801_core->con_info.pwd_len = pwd_len;

    /* ִ���ض����Ƶ�������������������������������������У������con_statusȥ���� */
    mrvl88w8801_scan_ssid(ssid,ssid_len,200);
    pmrvl88w8801_core->con_info.con_status = CON_STA_CONNECTING;
    return COMP_ERR_OK;
}


/******************************************************************************
 *	������:	mrvl88w8801_connect_prepare
 * ����:  	tx(IN)				-->tx buffer
 * ����ֵ: 	����ִ�н��
 * ����:	��HostCmd_CMD_802_11_DEAUTHENTICATE command�ķ��
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
 *	������:	mrvl88w8801_disconnect
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		������ΪSTA mode�������Ͽ���AP������
******************************************************************************/
uint8_t mrvl88w8801_disconnect()
{
    mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_DEAUTHENTICATE,HostCmd_ACT_GEN_GET,NULL,0);
    return COMP_ERR_OK;
}

/******************************************************************************
 *	������:	mrvl88w8801_scan_prepare
 * ����:  	tx(IN)				-->tx buffer
 			data_buff(IN)		-->������cmd body�����������channel list
 			data_len(IN)		-->������cmd body len,���������channel list����
 * ����ֵ: 	����ִ�н��
 * ����:	��HostCmd_CMD_802_11_SCAN command�ķ��
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
 *	������:	mrvl88w8801_ass_supplicant_pmk_pkg
 * ����:  		bssid(IN)				-->mac address
 * ����ֵ: 	����ִ�н��
 * ����:		���HostCmd_CMD_SUPPLICANT_PMK cmd�����cmd body���֣�
 				����������Ҫ������:
 				����������ӵĶ�������оƬ��4�����ֵ���֤
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
    pmk_ssid_tlv->header.len = pmrvl88w8801_core->con_info.ssid_len; //SSID�����Ƕ���
		//�����뿽����ȥ
    memcpy(pmk_ssid_tlv->ssid,pmrvl88w8801_core->con_info.ssid,pmrvl88w8801_core->con_info.ssid_len);
    pmk_len += sizeof(MrvlIEtypesHeader_t) + pmrvl88w8801_core->con_info.ssid_len;
		
    pmk_bssid_tlv = (MrvlIETypes_BSSIDList_t *)(pmk_tlv+pmk_len);
    pmk_bssid_tlv->header.type = TLV_TYPE_BSSID;
    pmk_bssid_tlv->header.len = TLV_PAYLOADLEN(*pmk_bssid_tlv);//����ĳ��ȼ��㲻һ��
		printf(" now %d before is %d\r\n",TLV_PAYLOADLEN(*pmk_bssid_tlv),MAC_ADDR_LENGTH);
		//������mac��ַ
    memcpy(pmk_bssid_tlv->mac_address,bssid,MAC_ADDR_LENGTH);
    pmk_len += sizeof(MrvlIETypes_BSSIDList_t);

    pmk_phrase_tlv = (MrvlIEtypes_PASSPhrase_t *)(pmk_tlv+pmk_len);
    pmk_phrase_tlv->header.type = TLV_TYPE_PASSPHRASE;//��ǰ���
    pmk_phrase_tlv->header.len = pmrvl88w8801_core->con_info.pwd_len;
    memcpy(pmk_phrase_tlv->phrase,pmrvl88w8801_core->con_info.pwd,pmrvl88w8801_core->con_info.pwd_len);
    //printf("ssid len is %d key len is %d\r\n",pmrvl88w8801_core->con_info.ssid_len,pmrvl88w8801_core->con_info.pwd_len);
		pmk_len += sizeof(MrvlIEtypesHeader_t) + pmrvl88w8801_core->con_info.pwd_len;

		//���͵���setûë��
    mrvl88w8801_prepare_cmd(HostCmd_CMD_SUPPLICANT_PMK,HostCmd_ACT_GEN_SET,&pmk_tlv,pmk_len);
		return COMP_ERR_OK;
}

/******************************************************************************
 *	������:	mrvl88w8801_supplicant_pmk_prepare
 * ����:  		tx(IN)				-->tx buffer
 				cmd_action(IN)	-->set/get
 				data_buff(IN)		-->mac address���˲�����Ҫ����set
 				data_len(IN)		-->mac�ĳ���
 * ����ֵ: 	����ִ�н��
 * ����:		��HostCmd_CMD_SUPPLICANT_PMK command�ķ��
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
 *	������:	mrvl88w8801_ret_scan
 * ����:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * ����ֵ: 	����ִ�н��
 * ����:		�������������cmd response,ע��������TLV��IEEE��TLV
 				����Marvell��TLV
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

    /* �ж���������AP���� */
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
				/* �жϸ���TLV */
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
						/* �˲���������Ϊ�յ�RSN����WPA2 */
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
		COMP_DEBUG("SSID '%s', ", ssid); /* SSID ���� */
		COMP_DEBUG("MAC %02X:%02X:%02X:%02X:%02X:%02X, ", bss_desc_set->bssid[0], bss_desc_set->bssid[1], bss_desc_set->bssid[2], bss_desc_set->bssid[3], bss_desc_set->bssid[4], bss_desc_set->bssid[5]); /* MAC��ַ */
		COMP_DEBUG("RSSI %d, Channel %d\n", bss_desc_set->rssi, channel); /* �ź�ǿ�Ⱥ�ͨ���� */
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
				/* �����ӵĶ��� */
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
								printf("��������WPA2������\r\n");
								//printf("???\r\n");
								rsn_tlv = (MrvlIETypes_RSN_t *)(associate_para+associate_para_len);
								rsn_tlv->header.type = TLV_TYPE_RSN_PARAMSET;
								rsn_tlv->header.len = rsn_data_ptr->header.length;
								//printf("cpoy ��ʼ��\r\n");
								memcpy(rsn_tlv->rsn,rsn_data_ptr->data,rsn_tlv->header.len);
								//printf("cpoy ������");
								associate_para_len += sizeof(MrvlIEtypesHeader_t) + rsn_tlv->header.len;
						}
				}
				printf("before ����MAC ASSOCIATE����\r\n");
				//�����mac��ַ���������˺������ֱ����
				comp_memcpy(pmrvl88w8801_core->con_info.mac_address,bss_desc_set->bssid,MAC_ADDR_LENGTH);
				pmrvl88w8801_core->con_info.cap_info = bss_desc_set->cap_info;
				//hw_hex_dump((uint8_t *)&associate_para,associate_para_len);
				printf("����MAC ASSOCIATE����\r\n");
				mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_ASSOCIATE,HostCmd_ACT_GEN_GET,&associate_para,associate_para_len);

			}
			else
			{
					if(mrvl_wifi_cb && mrvl_wifi_cb->wifi_scan_result)
							mrvl_wifi_cb->wifi_scan_result(ssid,bss_desc_set->rssi,channel,encryption_mode);
			}
			/* ������һ���ȵ� */
			bss_desc_set = (bss_desc_set_t *)((uint8_t *)bss_desc_set + sizeof(bss_desc_set->ie_length) + bss_desc_set->ie_length);

		}
	}
		
    return COMP_ERR_OK;
}
// ��������
static uint8_t mrvl88w8801_prepare_cmd(uint16_t cmd_id,uint16_t cmd_action,void *data_buff,uint16_t data_len)
{
    uint8_t *tx = mrvl_tx_buffer;
    uint16_t tx_len;
		uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
    /* ������� */
    switch(cmd_id)
    {
			  case HostCmd_CMD_FUNC_INIT: //��ʼ���̼���ʱ���� ֻ�г�ʼ���˲�������ʹ��
				{
					/* FUNC INIT������ */
					mrvl88w8801_func_init(tx);
					printf("��ʼ�������\r\n");
					break;
				}
				case HostCmd_CMD_MAC_CONTROL: //MAC control ������һ�����͵Ķ���
				{
						mrvl88w8801_mac_control(tx,data_buff);
						break;
				}
				case HostCmd_CMD_GET_HW_SPEC:  //��������Ҫ�Ķ���
				{
						mrvl88w8801_get_hw_spec(tx);
						break;
				}
				case HostCmd_CMD_802_11_MAC_ADDRESS:
				{
						mrvl88w8801_mac_addr_prepare(tx,cmd_action,data_buff,data_len);
						break;
				}
				case HostCmd_CMD_802_11_SCAN: //ɨ�����õ����� ��scan���������е���
				{
						mrvl88w8801_scan_prepare(tx,data_buff,data_len);
						//scan֮ǰ��׼������
						break;
				}
				case HostCmd_CMD_SUPPLICANT_PMK: //�����ӵ�ʱ���õ���
				{
						printf("���Ǹ�WPA2����׼���ķ������� ��ȻMAC CONNCET cmd��û����\r\n");
						mrvl88w8801_supplicant_pmk_prepare(tx,cmd_action,data_buff,data_len);
						break;
				}
				case HostCmd_CMD_802_11_ASSOCIATE: //׼��������
				{
						printf("MAC CONNCET cmd\r\n");
						mrvl88w8801_connect_prepare(tx,data_buff,data_len);
						break;
				}
				 case HostCmd_CMD_802_11_DEAUTHENTICATE://׼���Ͽ�������
				{
						mrvl88w8801_disconnect_prepare(tx);
						break;
				}

		}
		if(pmrvl88w8801_core->con_info.security >= WIFI_SECURITYTYPE_WPA && cmd_id == HostCmd_CMD_802_11_ASSOCIATE)
    {
				printf("��������������и�����ķ��ͳ�ȥ\r\n");
				delay_ms(100);
    }
		else
		{
			tx_len = tx[0] | (tx[1] << 8);
			// ��ַ��Ҫ����--������0
			WiFi_LowLevel_WriteData(1,control_io_port +CTRL_PORT,tx,tx_len,
												TX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
			printf("������� %d\r\n",tx_len);
		}
		return COMP_ERR_OK;
}

// ���Ƿ���������ͻ�����Ӧ ������Ӧ��Ҫ��һЩ�����Ķ���
// ������ ��Ӧ�յ�ֱ�ӷ�����һ�� ���ߴ�����Ӧ������һ��
static uint8_t mrvl88w8801_process_cmdrsp(uint8_t *rx_buffer,int len)
{
		// ����һ��ָ���ǿת
	  HostCmd_DS_COMMAND *resp = (HostCmd_DS_COMMAND *) rx_buffer;
		// ���������ǲ�����Ҫ�ظ���
    uint16_t cmdresp_no = resp->command & (~HostCmd_RET_BIT);
		// �����ǲ���OK
    if(resp->result != HostCmd_RESULT_OK)
    {
        return WIFI_ERR_INVALID_RESPONSE;
    }
		 switch(cmdresp_no)
    {
				// ���ڳ�ʼ������Ӧ����
		    case HostCmd_CMD_FUNC_INIT:
				{
					printf("step1 ��Ӧ��ʼ��\r\n");
					uint16_t mac_control_action = HostCmd_ACT_MAC_RX_ON | HostCmd_ACT_MAC_TX_ON |HostCmd_ACT_MAC_ETHERNETII_ENABLE;
					mrvl88w8801_prepare_cmd(HostCmd_CMD_MAC_CONTROL,HostCmd_ACT_GEN_SET,(void *)&mac_control_action,2);
					break;
				}
				// ����MAC����Ӧ
				 case HostCmd_CMD_MAC_CONTROL:
				{
						printf("step2 ��ӦMAC��ʼ��\r\n");
						mrvl88w8801_prepare_cmd(HostCmd_CMD_GET_HW_SPEC,HostCmd_ACT_GEN_GET,NULL,0);

						break;
				}
				case HostCmd_CMD_GET_HW_SPEC:
				{
						mrvl88w8801_ret_get_hw_spec(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						mrvl88w8801_prepare_cmd(HostCmd_CMD_802_11_MAC_ADDRESS,HostCmd_ACT_GEN_GET,NULL,0);

						break;
				}
				case HostCmd_CMD_802_11_MAC_ADDRESS: //������������Ӧ
				{
						mrvl88w8801_ret_mac_address(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						break;
				}
				case HostCmd_CMD_802_11_SCAN: //����scan����Ӧ���ж��е���
				{
						mrvl88w8801_ret_scan(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						break;
				}
				 case HostCmd_CMD_SUPPLICANT_PMK: //���ڸ������������Ӧ ����־�Ȼ�Ƿ��ͣ�
				{
						printf("���ܵ���Ӧ�� ���ԾͿ��Է���?\r\n");
						uint16_t tx_len = mrvl_tx_buffer[0] | (mrvl_tx_buffer[1] << 8);
						WiFi_LowLevel_WriteData(1,pmrvl88w8801_core->control_io_port +CTRL_PORT,mrvl_tx_buffer,tx_len,TX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);

						break;
				}
				case HostCmd_CMD_802_11_ASSOCIATE: //����conncet����Ӧ
				{
						printf("ret conncet\r\n");
						mrvl88w8801_ret_connect(rx_buffer+CMD_HDR_SIZE,len-CMD_HDR_SIZE);
						break;
				}
				 case HostCmd_CMD_802_11_DEAUTHENTICATE: //���������Ͽ����ӵ���Ӧ
				{
						/* STA��������ֻ���յ�cmd response */
						break;
				}
				default:
					printf("error response %d\r\n",cmdresp_no);
					break;
		}
		return COMP_ERR_OK;
		
}

/******************************************************************************
 *	������:	mrvl88w8801_handle_interrupt
 * ����:  		NULL
 * ����ֵ: 	����ִ�н��
 * ����:		����download/upload interrupt���Ĵ���
******************************************************************************/
void mrvl88w8801_handle_interrupt(int32_t *status)
{
    /* ��ctrl�Ĵ��� */
		/* ��ȡ�жϼĴ�����״̬ */
		// �ǵ�ֻҪ��ȡ�˼Ĵ��� ��ƽ�ͻ�ӵͻָ�λ����
		*status = WiFi_LowLevel_ReadReg(1, WIFI_INTSTATUS); // ��ȡ��Ҫ������жϱ�־λ
		if(*status <= 0)
		{
				//	printf("û�ж�ȡ��\r\n");
				//WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~(0)); //�����֤
				return;
		}
		memset(pmrvl88w8801_core->mp_regs,0x00,256);
		WiFi_LowLevel_ReadData(1,REG_PORT,pmrvl88w8801_core->mp_regs,64,256,WIFI_RWDATA_ALLOWMULTIBYTE);
    pmrvl88w8801_core->read_bitmap = (uint16_t) pmrvl88w8801_core->mp_regs[RD_BITMAP_L];
    pmrvl88w8801_core->read_bitmap |=((uint16_t) pmrvl88w8801_core->mp_regs[RD_BITMAP_U]) << 8;
    pmrvl88w8801_core->write_bitmap = (uint16_t) pmrvl88w8801_core->mp_regs[WR_BITMAP_L];
    pmrvl88w8801_core->write_bitmap |=((uint16_t) pmrvl88w8801_core->mp_regs[WR_BITMAP_U]) << 8;
		//printf("д�жϼĴ���ǰ%d\r\n",WiFi_LowLevel_GetITStatus(1));
		// ��������������������д(���)��ʱ�� wifiģ��Ҳ������д��ô����
		WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~(*status)); //�����֤�ܶ���
		//printf("д�жϼĴ�����%d\r\n",WiFi_LowLevel_GetITStatus(1));
}

/******************************************************************************
 *	������:	mrvl88w8801_get_read_port
 * ����:  		port(OUT)			-->����read port
 * ����ֵ: 	����ִ�н��
 * ����:		���ڻ�ȡread port
******************************************************************************/
static uint8_t mrvl88w8801_get_read_port(uint8_t *port)
{
		//printf("�����ȡ�˿ں�����\r\n");
    if (pmrvl88w8801_core->read_bitmap & CTRL_PORT_MASK)
    {
				//printf("���뵽����1��\r\n"); //�������ͻ��������
				//�������һһλ��ı��ֲ��� ���ж����Ҫ����λ
        pmrvl88w8801_core->read_bitmap &= (uint16_t) (~CTRL_PORT_MASK);
        *port = CTRL_PORT;
    }
    else
    {
				//��λ����һ���ҵ�����һ��Ҫ���Ķ˿�1
				//����һ��0xFF ������ÿһλ����λ��ȥ��Ҫ���Ķ˿� Ȼ��Ӷ˿�ȡ��������
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
 *	������:	mrvl88w8801_process_event
 * ����:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * ����ֵ: 	����ִ�н��
 * ����:		�����յ���event
******************************************************************************/
static uint8_t mrvl88w8801_process_event(uint8_t *rx_buffer,int len)
{
    uint16_t event = *(uint16_t *)(rx_buffer + CMD_SDIO_HDR_SIZE);

    //COMP_DEBUG("WIFI EVENT ID 0x%x\n",event);
    switch(event)
    {
			case EVENT_PORT_RELEASE: //�����ڴ�˼�����ǽ�����ʹ�ܲ������ǲ���
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
					/* AP��STA�Ƴ�����AP�رգ�STA�����յ�����Ϣ */
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
 *	������:	mrvl88w8801_get_write_port
 * ����:  		port(OUT)			-->����read port
 * ����ֵ: 	����ִ�н��
 * ����:		���ڻ�ȡwrite port
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
 *	������:	mrvl88w8801_get_send_data_buf
 * ����:  		NULL
 * ����ֵ: 	����TX buffer��payload��Ҳ����tcp/ip���ݲ���
 * ����:		����tcp/ip���ã�ֱ��дtcp/ip����
******************************************************************************/
uint8_t *mrvl88w8801_get_send_data_buf()
{
		memset(mrvl_tx_buffer,0x00,3*1024);
    TxPD *tx_packet = (TxPD *)(mrvl_tx_buffer);
    pmrvl88w8801_core->tx_data_ptr = mrvl_tx_buffer;
    return (tx_packet->payload);//��������Ч��λ
}
/******************************************************************************
 *	������:	mrvl88w8801_send_data
 * ����:  		data(IN)			-->Ҫ���͵�data
 				size(IN)			-->Ҫ���͵�data size
 * ����ֵ: 	����ִ�н��
 * ����:		����data
******************************************************************************/
uint8_t mrvl88w8801_send_data(uint8_t *data,uint16_t size)
{
    uint8_t wr_bitmap_l;
    uint8_t wr_bitmap_u;
    uint8_t port = 0;
			
    TxPD *tx_packet = (TxPD *)(pmrvl88w8801_core->tx_data_ptr);
    uint16_t tx_packet_len = sizeof(TxPD) - sizeof(tx_packet->payload) + size;
		//printf("����֮���㷨tx_packet_len ��%d\r\n",tx_packet_len);
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
    /* ���write port������ */
    mrvl88w8801_get_write_port(&port);
    WiFi_LowLevel_WriteData(1,pmrvl88w8801_core->control_io_port+port,
			(uint8_t*)tx_packet,tx_packet->pack_len,TX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
		
//		printf("��Ϊlwip��һ���ַ����������%d\r\n",size);
    return COMP_ERR_OK;
}
/******************************************************************************
 *	������:	mrvl88w8801_process_data
 * ����:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * ����ֵ: 	����ִ�н��
 * ����:		�����յ���data(Ҳ����tcp/ip data)
******************************************************************************/
static uint8_t mrvl88w8801_process_data(uint8_t *rx_buffer,int len)
{
		
    if(pmrvl88w8801_core->bss_type == BSS_TYPE_UAP)
    {
				//printf("���뵽bss_type_uap����\r\n");
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
            /* 1.�㲥�����ת�� */
						//printf("�㲥���\r\n");
            uint8_t *buffer = mrvl88w8801_get_send_data_buf();
            uint8_t *payload = (uint8_t *)((uint8_t *)rx_packet + rx_packet->rx_pkt_offset + 4);
            comp_memcpy(buffer,payload,rx_payload_len);
            mrvl88w8801_send_data(buffer,rx_payload_len);

            /* 2.�Լ�AP����㲥��� */
            ethernet_lwip_process(rx_buffer,len);
        }
        else
        {
            /* ������� */
            if(memcmp(pmrvl88w8801_core->mac_address,rx_hdr->eth803_hdr.dest_addr,MAC_ADDR_LENGTH) == 0)
            {
								//printf("�������\r\n");
                ethernet_lwip_process(rx_buffer,len);

            }
            else
            {
                /* �ж��Ƿ���STA��ַ ��ת�� */
								//printf("STAת��\r\n");
                uint8_t *buffer = mrvl88w8801_get_send_data_buf();
                uint8_t *payload = (uint8_t *)((uint8_t *)rx_packet + rx_packet->rx_pkt_offset + 4);
                comp_memcpy(buffer,payload,rx_payload_len);
                mrvl88w8801_send_data(buffer,rx_payload_len);
            }
        }

    }

    if(pmrvl88w8801_core->bss_type == BSS_TYPE_STA)
    {
				//printf("���뵽bss_type_sta���� %d\r\n",len);
        ethernet_lwip_process(rx_buffer,len);
    }

    return COMP_ERR_OK;
}
/******************************************************************************
 *	������:	mrvl88w8801_parse_rx_packet
 * ����:  		rx_buffer(IN)			-->rx buffer
 				len(IN)				-->rx buffer len
 * ����ֵ: 	����ִ�н��
 * ����:		������оƬ�յ������ݣ��ٷֱ���cmd response/event/data
 				��������������
******************************************************************************/
static uint8_t mrvl88w8801_parse_rx_packet(uint8_t *rx_buffer,int len)
{
    uint16_t rx_type = (uint16_t)rx_buffer[2];
    switch(rx_type)
    {
			case TYPE_CMD_CMDRSP: //����ֻ�õ�����Щ
			{
//					printf("�����ݽ��н��\r\n");
					mrvl88w8801_process_cmdrsp(rx_buffer,len);
				
					break;
			}
			/*--------------��������õ���-----------------*/
			case TYPE_DATA:
			{
//						printf("����data����\r\n");
						mrvl88w8801_process_data(rx_buffer,len);
						break;
			}
			case TYPE_EVENT:
			{
//					printf("�����¼�\r\n");
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
    /* ��ȡ�жϼĴ���״̬ */
    int_status = pmrvl88w8801_core->mp_regs[HOST_INT_STATUS_REG];
		if(int_status > 0 && int_status <= 3) // ����ѵ���ȥ��
		{
				// �������Բ�write�в���
				WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~(int_status)); //�����֤
				//if(int_status > 0)
						//printf("TMΪɶ�����жϼĴ���ֵȴû�����ж� %d\r\n",int_status);
		}else if(int_status)
		{
				printf("��ȡ��������������%d \r\n",int_status);
				return 0;
		}
		//ֱ�ӿ��Ĵ���������ô�� 
    while(1)
    {
         /* ���read port */
         if(mrvl88w8801_get_read_port(&port) != COMP_ERR_OK)
          {
               //COMP_DEBUG("no more rd_port to be handled\n");
              break;
           }

            /* ��ȡ��Ҫ��ȡ�ķ������ */
            len_reg_l = RD_LEN_P0_L + (port << 1);
            len_reg_u = RD_LEN_P0_U + (port << 1);
            rx_len = ((uint16_t) pmrvl88w8801_core->mp_regs[len_reg_u]) << 8;
            rx_len |= (uint16_t) pmrvl88w8801_core->mp_regs[len_reg_l];
            //COMP_DEBUG("@@@@@@@RX: port=%d rx_len=%u\n", port, rx_len);

            if(rx_buffer != NULL)
            {
                /* ʹ��CMD53��ȡ�յ������� */
									WiFi_LowLevel_ReadData(1,control_io_port+port,rx_buffer,rx_len,
																			RX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
                /* �������� */
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
		// ����ҵ���
		
		//���ڵ�������������ô���� ��Ϊ�϶���Ҫ��������� �����֮�����û�п��ٴ�����Ϣ������
		if((int_status & UP_LD_HOST_INT_MASK) == UP_LD_HOST_INT_MASK)
    {
				read_count = 0;
					//�����ݿɶ���ȥ��������Щ
					while(1) //�����������˾�����Ϊ���ǵĳ�������whileȥ����Ȼ������´��ж����� Ȼ����εĿ��ܻ�û�������
					{
            if(mrvl88w8801_get_read_port(&port) != COMP_ERR_OK)
            {
                //printf("no more rd_port to be handled\r\n");
                break;
            }
						read_count++;
						//printf("���ǵ�%d�εĶ˿� �˿ں���%d\r\n",read_count,port);
					//	printf("port is %d\r\n",port);
            /* ��ȡ��Ҫ��ȡ�ķ������ */
            len_reg_l = RD_LEN_P0_L + (port << 1); //length��ŵ�λ��������
            len_reg_u = RD_LEN_P0_U + (port << 1);
            rx_len = ((uint16_t) pmrvl88w8801_core->mp_regs[len_reg_u]) << 8;
            rx_len |= (uint16_t) pmrvl88w8801_core->mp_regs[len_reg_l];
//            printf("@@@@@@@RX: port=%d rx_len=%u\r\n", port, rx_len);

            if(rx_buffer != NULL)
            {
								/* ʹ��CMD53��ȡ�յ������� */
								// �����и����� ��ȡ�೤������
								WiFi_LowLevel_ReadData(1,control_io_port+port,rx_buffer,rx_len,
																			RX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
//                printf("׼����ʼ���\r\n");
								mrvl88w8801_parse_rx_packet(rx_buffer,rx_len);
            }
					}
							
    }
		if((int_status & DN_LD_HOST_INT_MASK) == 1)
		{
				printf("�п���д��Ķ˿���\r\n");
		}
}
// �ڶ��̻߳�����û�п��ǹ����ȫ�ֱ����Ĺ�������һ�������̶߳������˾��군��

uint8_t mrvl88w8801_process_packet(void)
{
    uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
    uint8_t port;
    int32_t int_status;
    uint32_t len_reg_l, len_reg_u;
    uint16_t rx_len;
    uint8_t* rx_buffer = mrvl_rx_buffer;

    /* �����ж� */
    mrvl88w8801_handle_interrupt(&int_status);
		//���ܶ�����״̬��� ��������ͱ�֤����������ʹ��
		if(int_status == 0) //�����˵���û���жϷ���,��ô�����õ�ƽ״̬��ȥ��
		{
				printf("��int_status == 0ʱ�˳�process_packet��״̬��%d\r\n",WiFi_LowLevel_GetITStatus(1));
				return 2;
		}
		//printf("��ȡ�����жϼĴ���״̬��%d\r\n",int_status);
		//оƬ���ܵ����ݺ��ϱ��ж�
		if((int_status & UP_LD_HOST_INT_MASK) == UP_LD_HOST_INT_MASK)
    {
					while(1) //�����������˾�����Ϊ���ǵĳ�������whileȥ����Ȼ������´��ж����� Ȼ����εĿ��ܻ�û�������
					{
            if(mrvl88w8801_get_read_port(&port) != COMP_ERR_OK)
            {
                //printf("no more rd_port to be handled\r\n");
                break;
            }
						printf("port is %d\r\n",port);
            /* ��ȡ��Ҫ��ȡ�ķ������ */
            len_reg_l = RD_LEN_P0_L + (port << 1);
            len_reg_u = RD_LEN_P0_U + (port << 1);
            rx_len = ((uint16_t) pmrvl88w8801_core->mp_regs[len_reg_u]) << 8;
            rx_len |= (uint16_t) pmrvl88w8801_core->mp_regs[len_reg_l];
            //printf("@@@@@@@RX: port=%d rx_len=%u\n", port, rx_len);
						//printf("len_reg_l is %d  len_reg_u is %d\r\n",len_reg_l,len_reg_u);

            if(rx_buffer != NULL)
            {
								/* ʹ��CMD53��ȡ�յ������� */
								// �����и����� ��ȡ�೤������
								WiFi_LowLevel_ReadData(1,control_io_port+port,rx_buffer,rx_len,
																			RX_BUFFER_SIZE,WIFI_RWDATA_ALLOWMULTIBYTE);
								//printf("��ȡ������� rx_len %d\r\n",rx_len);
                mrvl88w8801_parse_rx_packet(rx_buffer,rx_len);
            }
					}
							
    }
		//�Ƿ������ݺ���Խ��յ��ж�
		if((int_status & DN_LD_HOST_INT_MASK) == DN_LD_HOST_INT_MASK)
		{
				printf("������������ж�\r\n");
		}
		printf("��������˳�process_packet��״̬��%d\r\n",WiFi_LowLevel_GetITStatus(1));
    return COMP_ERR_OK;
}

/* �̼����� */
// WiFiģ�����Ҫ�й̼�������������, �̼���Marvell��˾ (WiFiģ���������)������
// �̼��Ǳ�����STM32��Ƭ����Flash�����, Flash���汣��������ڶϵ�ʱ���ᶪʧ
// ͨ��ʱ (��Ƭ����λ��), ��Ƭ���ѱ�����Flash����Ĺ̼�ͨ��SPI��SDIO�ӿڷ��͸�WiFiģ��
// ���浽WiFiģ���SRAM (�ڴ�)��������, SRAM���������һ�ϵ��λ�ͻᶪʧ
// ����ÿ�ζϵ��λ��Ҫ�������ع̼�
/* ��ȡ�̼�״̬ */
uint16_t WiFi_GetFirmwareStatus(void)
{
  return WiFi_LowLevel_ReadReg(1, WIFI_SCRATCH0_0) | (WiFi_LowLevel_ReadReg(1, WIFI_SCRATCH0_1) << 8);
}
/* �ڹ涨�ĳ�ʱʱ����, �ȴ�ָ���Ŀ�״̬λ��λ, �������Ӧ���жϱ�־λ */
// �ɹ�ʱ����1
int WiFi_Wait(uint8_t status, uint32_t timeout)
{
		uint32_t diff, start;
		
		start = HAL_GetTick();
		while ((WiFi_LowLevel_ReadReg(1, WIFI_INTSTATUS) & status) != status)
		{
				diff = HAL_GetTick() - start;
				if (timeout != 0 && diff > timeout)
				{
						// ����ʱʱ���ѵ�
						printf("WiFi_Wait(0x%2x): timeout!\n", status);
						return 0;
				}
		}
		// �����Ӧ���жϱ�־λ
		WiFi_LowLevel_WriteReg(1, WIFI_INTSTATUS, WIFI_INTSTATUS_ALL & ~status); 
		// ���ܽ�SDIOITλ�����! �����п��ܵ��¸�λ��Զ������λ
		return 1;
}

static void WiFi_DownloadFirmware(void)
{
	
		const uint8_t *data;
		int ret;
		uint16_t curr;
		uint8_t retry;
		uint32_t len = 255536;//�̼��Ĵ�С
		uint32_t control_io_port = pmrvl88w8801_core->control_io_port;
		uint8_t *recv = mymalloc(SRAMEX,1500); //����ǲ��ǻᳬ��������ջ�ڴ������ֱ�Ӷ���ֲ�����
		printf("%p",recv);
		uint32_t fw_addr = ADDR_FLASH_SECTOR_10;
		if (WiFi_GetFirmwareStatus() == WIFI_FIRMWARESTATUS_OK)
		{
			// PDN����û����ȷ���ӵ���Ƭ����, ��Ƭ����λʱWiFiģ����޷����Ÿ�λ
			// �������ʱ����Ҫ�������ع̼�
			// WiFiģ��ԭ�е�״̬���� (���������ϵ��ȵ�, �ѷ��͵�δ���ջ�Ӧ������)
			// ���ʱ�����������ȥ, �ܿ��ܻ����
			printf("%s: Reset signal has no effect!\r\n", __FUNCTION__);
			printf("Please make sure PDN pin is connected to the MCU!\r\n");
			myfree(SRAMEX,recv);
			return;
		}
 
		while (len)
		{
				// ��ȡ����Ӧ���ص��ֽ���
				// ÿ�ο��Է���n>=curr�ֽڵ�����, ֻ����һ��CMD53�����, WiFiģ��ֻ��ǰcurr�ֽڵ�����
				// 88W8801�������curr=0�����, �������ﲻ��Ҫwhile���
				curr = WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR0) | (WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR1) << 8);
				//printf("curr is %d\r\n",curr);
				//�´������㷢�ͼ����ֽ�
				//���currһ���ܳ���4��
				stmflash_read(fw_addr,(uint32_t *)recv,curr/4);
				fw_addr += curr; //����һ������
				//printf("��ȡ����׼������\r\n");
				// ���͹̼�����
				// res���Ǹ���CMD53���в�����
				ret = WiFi_LowLevel_WriteData(1, control_io_port, recv, curr, 0, WIFI_RWDATA_ALLOWMULTIBYTE);
				if (ret == 2)
						break; // �������� û�д���
				ret = WiFi_Wait(WIFI_INTSTATUS_DNLD, 100); // ʵ�������ﳬʱ��Ҳ��ע��
				retry = WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR0);
				
				if (retry & 1)
				{
						ret = 0; // ����Ĵ�����ֵΪ����, ˵��ģ���յ��Ĺ̼���������, ��Ҫ�ش�
						retry |= WiFi_LowLevel_ReadReg(1, WIFI_SQREADBASEADDR1) << 8;
						printf("%s: Data were not recognized! curr=%d, retry=%d, remaining=%d\n", __FUNCTION__, curr, retry, len);
						break;//��û�����ô����
				}
				len -= curr;
				printf("ʣ��ĳ�����%d\r\n",len);
				data += curr;
				//��һ�¿����Բ���
		}
  
		// �ȴ�Firmware���� --�������ﾹȻ���Գɹ���TM����
		// ע�͵�ǰ��ľ�GG��
		while (WiFi_GetFirmwareStatus() != WIFI_FIRMWARESTATUS_OK);
		printf("Firmware is successfully downloaded!\n");
		delay_ms(500);
}

// ʹ���½����ж� Ŀ�ľ�����RTOS�﷽�����һ��
void sd_irq_exit_enable(void)
{
			// ʹ�� GPIO ʱ��
		SD_IRQ_GPIO_CLK();

		// ���� GPIO Ϊ����ģʽ��������ǰ�����ò���
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = SD_IRQ_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // ѡ���½��ش����ж�
		GPIO_InitStruct.Pull = GPIO_NOPULL;          // ���ݵ�ǰʹ������������
		HAL_GPIO_Init(SD_IRQ_GPIO_PORT, &GPIO_InitStruct);
		// �����ж����ȼ� ע��Ҫ�ܵ�RTOS�Ĺ��� Ҫ����4
		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0); // ���ȼ��������Ӧ�õ���
			// ʹ���ж�
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

