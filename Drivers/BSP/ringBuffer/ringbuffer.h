#ifndef __RINGBUF_H
#define __RINGBUF_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "pbuf.h"
#define RB_MAX_NUM (16)

//这个是一维的
typedef struct kfifo
{
	uint8_t *buffer; 			//指向分配的数组
	uint32_t in;					//读指针
	uint32_t out;					//写指针
	uint16_t size;				//数组的长度
}kfifo;





uint16_t roundup_pow_of_two(uint16_t size); //向上取整
struct kfifo * kfifo_alloc(uint16_t size);
uint16_t kfifo_put(struct kfifo *fifo,uint8_t *buffer,uint16_t len);
uint16_t kfifo_get(struct kfifo *fifo,
             uint8_t *buffer, uint16_t len);
uint8_t kfifo_free(struct kfifo *k_buf);

uint16_t kfifo_put_pbuf(struct kfifo *fifo, const struct pbuf *buf, uint16_t len, uint16_t offset);
struct pbuf *kfifo_get_pbuf(struct kfifo *fifo, uint16_t length);
//这样的话消息队列中就是要读取数据的长度就行
#endif
