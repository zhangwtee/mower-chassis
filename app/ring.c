// 环形缓冲。串口收字节用,满了丢最老的
#include "ring.h"

void ring_init(ring_t *r, uint8_t *buf, uint16_t size)
{
    r->buf = buf;
    r->size = size;
    r->head = 0;
    r->tail = 0;
}

uint8_t ring_put(ring_t *r, uint8_t c)
{
    uint16_t nh = (uint16_t)((r->head + 1) & (r->size - 1));

    if (nh == r->tail) return 0;

    r->buf[r->head] = c;
    r->head = nh;
    return 1;
}

uint8_t ring_get(ring_t *r, uint8_t *c)
{
    if (r->head == r->tail) return 0;

    *c = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) & (r->size - 1));
    return 1;
}

uint16_t ring_used(const ring_t *r)
{
    return (uint16_t)((r->head - r->tail) & (r->size - 1));
}

uint16_t ring_free(const ring_t *r)
{
    //生产者读自己的 head 和消费者的 tail。tail 只会单调前进,
    //万一读到旧值,算出来的空闲量偏小,也就是宁可多丢一帧,不会写穿
    return (uint16_t)((r->size - 1u) - ring_used(r));
}
