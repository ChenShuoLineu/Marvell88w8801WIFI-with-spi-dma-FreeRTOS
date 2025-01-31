#ifndef __RINGBUF_H
#define __RINGBUF_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "pbuf.h"
#define RB_MAX_NUM (16)

//�����һά��
typedef struct kfifo
{
	uint8_t *buffer; 			//ָ����������
	uint32_t in;					//��ָ��
	uint32_t out;					//дָ��
	uint16_t size;				//����ĳ���
}kfifo;





uint16_t roundup_pow_of_two(uint16_t size); //����ȡ��
struct kfifo * kfifo_alloc(uint16_t size);
uint16_t kfifo_put(struct kfifo *fifo,uint8_t *buffer,uint16_t len);
uint16_t kfifo_get(struct kfifo *fifo,
             uint8_t *buffer, uint16_t len);
uint8_t kfifo_free(struct kfifo *k_buf);

uint16_t kfifo_put_pbuf(struct kfifo *fifo, const struct pbuf *buf, uint16_t len, uint16_t offset);
struct pbuf *kfifo_get_pbuf(struct kfifo *fifo, uint16_t length);
//�����Ļ���Ϣ�����о���Ҫ��ȡ���ݵĳ��Ⱦ���
#endif
