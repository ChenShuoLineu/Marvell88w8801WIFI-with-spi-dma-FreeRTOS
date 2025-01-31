 #include "./BSP/ringBuffer/ringbuffer.h"
 #include "./MALLOC/malloc.h"
 #include "string.h"
 #define min(x, y) ((x) < (y) ? (x) : (y))
// 初始化ringbuffer


uint16_t roundup_pow_of_two(uint16_t size) {
    if (size == 0) {
        return 1;
    }
    
    // 计算出最高位 1 的位置，然后左移一位得到最近的 2 的幂
    return 1U << (32 - __builtin_clz(size - 1));
}
//以上都是2维的 现在需要一个一维的环形缓冲区
struct kfifo * kfifo_alloc(uint16_t size)
{
		if(size == 0)
				return NULL;
		//如果说满足就要重新计算
		if (size & (size - 1)) {
        size = roundup_pow_of_two(size);
    }
		printf("KFIFO的size是%d",size);
		struct kfifo * k_buf = (struct kfifo *) mymalloc(SRAMEX,sizeof(struct kfifo));
		k_buf->buffer = NULL;
		k_buf->size = size;
		k_buf->in = 0;
		k_buf->out = 0;
		k_buf->buffer = mymalloc(SRAMEX,size);
		configASSERT(k_buf && k_buf->buffer);
		if(k_buf->buffer == NULL)
				return NULL;
		return k_buf;

}
static inline uint16_t fifo_is_empty(struct kfifo *fifo)
{
		return fifo->in == fifo->out;
}
static inline uint16_t fifo_is_full(struct kfifo *fifo)
{
		return (fifo->in - fifo->out) == fifo->size;
}
static inline  uint16_t fifo_remain(struct kfifo * fifo)
{
	return (fifo->size - (fifo->in - fifo->out));
}
uint16_t kfifo_put(struct kfifo *fifo,uint8_t *buffer,uint16_t len)
{
		if(fifo_is_full(fifo) || fifo_remain(fifo) < len)
				return 0;
		uint32_t l;
		len =  min(len, fifo->size - fifo->in + fifo->out);
		//可能会分为两块进入
		l = min(len, fifo->size - (fifo->in & (fifo->size - 1)));
		memcpy(fifo->buffer + (fifo->in & (fifo->size - 1)), buffer, l);
    /* then put the rest (if any) at the beginning of the buffer */
    memcpy(fifo->buffer, buffer + l, len - l);
		fifo->in += len;
		return len;
}
uint16_t kfifo_get(struct kfifo *fifo,
             uint8_t *buffer, uint16_t len)
{
		uint32_t l;
		if(fifo_is_empty(fifo))
				return 0;
		len = min(len,fifo->in - fifo->out);
		l = min(len, fifo->size - (fifo->out & (fifo->size - 1)));
		memcpy(buffer, fifo->buffer + (fifo->out & (fifo->size - 1)), l);
    memcpy(buffer + l, fifo->buffer, len - l);
		fifo->out += len;
    return len;

}
// 把这个写一下
uint16_t kfifo_put_pbuf(struct kfifo *fifo, const struct pbuf *buf, uint16_t len, uint16_t offset)
{
     const struct pbuf *p;
    uint16_t copied_total = 0;  // 记录总共拷贝的字节数
    uint16_t buf_copy_len;
    // 参数有效性检查
    if (fifo == NULL || buf == NULL) {
        return 0;
    }

    // 检查 FIFO 剩余空间
    if (fifo_is_full(fifo) || fifo_remain(fifo) < len) {
        return 0;
    }

    // 遍历 pbuf 链表，处理偏移量
    for (p = buf; len > 0 && p != NULL; p = p->next) {
        if ((offset != 0) && (offset >= p->len)) {
            // 跳过不需要的数据
            offset = (uint16_t)(offset - p->len);
        } else {
            // 计算本次可以拷贝的数据长度
            buf_copy_len = (uint16_t)(p->len - offset);
            if (buf_copy_len > len) {
                buf_copy_len = len;
            }
            // 调用 kfifo_put 进行 FIFO 存储
            uint16_t put_len = kfifo_put(fifo, (uint8_t *)p->payload + offset, buf_copy_len);
						configASSERT(put_len == buf_copy_len);
            copied_total += put_len;
            len -= put_len;
            offset = 0;  // 之后的 pbuf 不再需要处理偏移量
        }
    }

    return copied_total;
}
struct pbuf *kfifo_get_pbuf(struct kfifo *fifo, uint16_t length)
{
    struct pbuf *p;
    size_t buf_copy_len;
    size_t total_copy_len = length;

    // 检查 FIFO 是否为空或剩余数据不足
    if (fifo_is_empty(fifo)) {
        return NULL;
    }

    // 分配 pbuf，用于存储从 FIFO 读取的数据
    p = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);
    if (p == NULL) {
        return NULL;
    }

    // 开始从 FIFO 中读取数据并填充到 pbuf
    for (struct pbuf *cur = p; total_copy_len > 0; cur = cur->next) {
        buf_copy_len = total_copy_len;
        if (buf_copy_len > cur->len) {
            buf_copy_len = cur->len;  // 当前 pbuf 无法容纳所有数据，分割复制
        }
        uint16_t copied_len = kfifo_get(fifo, (uint8_t *)cur->payload, buf_copy_len);  // 从 FIFO 获取数据
				configASSERT(copied_len == buf_copy_len);
        // 检查是否复制完所有数据
        if (copied_len != buf_copy_len) {
            pbuf_free(p);  // 释放 pbuf，防止内存泄漏
            return NULL;
        }

        total_copy_len -= buf_copy_len;

    }


    return p;
}
uint8_t kfifo_free(struct kfifo *k_buf)
{
	if(k_buf != NULL && k_buf->buffer !=	NULL)
	{
			myfree(SRAMEX,k_buf->buffer);
			k_buf->buffer = NULL;
			myfree(SRAMEX,k_buf);
			k_buf = NULL;
	}
	return 0;
}
