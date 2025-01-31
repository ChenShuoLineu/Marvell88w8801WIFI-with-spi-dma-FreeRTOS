 #include "./BSP/ringBuffer/ringbuffer.h"
 #include "./MALLOC/malloc.h"
 #include "string.h"
 #define min(x, y) ((x) < (y) ? (x) : (y))
// ��ʼ��ringbuffer


uint16_t roundup_pow_of_two(uint16_t size) {
    if (size == 0) {
        return 1;
    }
    
    // ��������λ 1 ��λ�ã�Ȼ������һλ�õ������ 2 ����
    return 1U << (32 - __builtin_clz(size - 1));
}
//���϶���2ά�� ������Ҫһ��һά�Ļ��λ�����
struct kfifo * kfifo_alloc(uint16_t size)
{
		if(size == 0)
				return NULL;
		//���˵�����Ҫ���¼���
		if (size & (size - 1)) {
        size = roundup_pow_of_two(size);
    }
		printf("KFIFO��size��%d",size);
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
		//���ܻ��Ϊ�������
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
// �����дһ��
uint16_t kfifo_put_pbuf(struct kfifo *fifo, const struct pbuf *buf, uint16_t len, uint16_t offset)
{
     const struct pbuf *p;
    uint16_t copied_total = 0;  // ��¼�ܹ��������ֽ���
    uint16_t buf_copy_len;
    // ������Ч�Լ��
    if (fifo == NULL || buf == NULL) {
        return 0;
    }

    // ��� FIFO ʣ��ռ�
    if (fifo_is_full(fifo) || fifo_remain(fifo) < len) {
        return 0;
    }

    // ���� pbuf ��������ƫ����
    for (p = buf; len > 0 && p != NULL; p = p->next) {
        if ((offset != 0) && (offset >= p->len)) {
            // ��������Ҫ������
            offset = (uint16_t)(offset - p->len);
        } else {
            // ���㱾�ο��Կ��������ݳ���
            buf_copy_len = (uint16_t)(p->len - offset);
            if (buf_copy_len > len) {
                buf_copy_len = len;
            }
            // ���� kfifo_put ���� FIFO �洢
            uint16_t put_len = kfifo_put(fifo, (uint8_t *)p->payload + offset, buf_copy_len);
						configASSERT(put_len == buf_copy_len);
            copied_total += put_len;
            len -= put_len;
            offset = 0;  // ֮��� pbuf ������Ҫ����ƫ����
        }
    }

    return copied_total;
}
struct pbuf *kfifo_get_pbuf(struct kfifo *fifo, uint16_t length)
{
    struct pbuf *p;
    size_t buf_copy_len;
    size_t total_copy_len = length;

    // ��� FIFO �Ƿ�Ϊ�ջ�ʣ�����ݲ���
    if (fifo_is_empty(fifo)) {
        return NULL;
    }

    // ���� pbuf�����ڴ洢�� FIFO ��ȡ������
    p = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);
    if (p == NULL) {
        return NULL;
    }

    // ��ʼ�� FIFO �ж�ȡ���ݲ���䵽 pbuf
    for (struct pbuf *cur = p; total_copy_len > 0; cur = cur->next) {
        buf_copy_len = total_copy_len;
        if (buf_copy_len > cur->len) {
            buf_copy_len = cur->len;  // ��ǰ pbuf �޷������������ݣ��ָ��
        }
        uint16_t copied_len = kfifo_get(fifo, (uint8_t *)cur->payload, buf_copy_len);  // �� FIFO ��ȡ����
				configASSERT(copied_len == buf_copy_len);
        // ����Ƿ�������������
        if (copied_len != buf_copy_len) {
            pbuf_free(p);  // �ͷ� pbuf����ֹ�ڴ�й©
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
